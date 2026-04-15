"""Rate change controller for PIPE SerDes.

Implements Gen1 (5 GT/s) ↔ Gen2 (10 GT/s) rate switching by writing 7 CSR
registers through the DRP mux.  The 7-register write sequence requires
exclusive DRP bus lock to prevent interleaving with other sub-controllers.

Rate Change Procedure (PIPE Spec §8.4)
--------------------------------------
1. MAC asserts new ``Rate[3:0]``
2. PHY asserts lane resets (PMA + PCS)
3. PHY writes 7 CSR registers atomically (DRP lock held)
4. PHY toggles PLL enable for relock
5. PHY waits for PLL relock (CMU_OK_O) — timeout ~16 ms
6. PHY deasserts resets
7. PHY asserts ``PclkChangeOk``
8. MAC switches PCLK and asserts ``PclkChangeAck``
9. PHY pulses ``PhyStatus``, deasserts ``PclkChangeOk``

Timing
------
Total rate change latency: ~100–200 µs (dominated by PLL lock time).
CSR write phase: ~700–900 ns (7 writes × ~100 ns each).
PLL relock timeout: ~16 ms (safety ceiling; typical lock < 200 µs).

Register Table
--------------
The 7 registers that differ between Gen1 (5G) and Gen2 (10G) are defined
in ``pipe_config.RATE_CHANGE_REGS``:

===  ==========  ==========  ==========  ==========================
 #   Address     Gen1 Value  Gen2 Value  Description
===  ==========  ==========  ==========  ==========================
 0   0x80A020    0x0000001A  0x00000014  CPLL divider ratio
 1   0x8082A0    0x00003150  0x00001170  RX AFE gain/attenuation
 2   0x8082B8    0x00000020  0x00000040  RX AFE bias current
 3   0x8082C0    0x00000210  0x00000310  RX AFE boost
 4   0x808600    0x0000011A  0x0000021A  TX clock source select
 5   0x808620    0x00000016  0x00000026  RX clock source select
 6   0x809000    0x00000001  0x00000003  PCS rate mode
===  ==========  ==========  ==========  ==========================
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const, Array, Mux

from gowin_serdes.csr_map import rate_change_regs


# Number of CSR registers written during a rate change.
_NUM_RATE_REGS = 7


class PIPERateController(Elaboratable):
    """Rate change controller FSM.

    Orchestrates the full Gen1↔Gen2 rate switch sequence per PIPE §8.4.
    Connects to the DRP mux as client 0 (highest priority) and holds the
    DRP bus lock for the duration of the 7-register write burst.

    The controller is only active when the lane is in power state P0
    (``current_power == 0``).  Rate change requests in other power states
    are ignored until the lane returns to P0.

    Parameters
    ----------
    quad : int
        GTR12 quad index (0-based).  Used to compute lane-specific CSR
        addresses via ``compute_csr_addrs`` / ``rate_change_regs``.
    lane : int
        Lane index within the quad (0-based).

    Input Ports
    -----------
    rate : Signal(4)
        Requested rate from the MAC.  ``0`` = Gen1 (5 GT/s),
        ``1`` = Gen2 (10 GT/s).  Only bit 0 is examined.
    pclk_change_ack : Signal(1)
        MAC acknowledges that it has switched its PCLK source.
    pll_lock : Signal(1)
        PLL / CMU lock status from the PHY (CMU_OK_O).
    reset_n : Signal(1)
        Active-low asynchronous reset.  While deasserted the FSM is
        held in ``IDLE``.
    drp_ready : Signal(1)
        DRP write-accepted strobe from the DRP mux / arbiter.
    current_power : Signal(4)
        Current PIPE power state.  Rate changes are only initiated
        when this equals ``0`` (P0).

    Output Ports
    ------------
    phy_status : Signal(1)
        Single-cycle pulse indicating rate change completion.
    pclk_change_ok : Signal(1)
        Asserted to tell the MAC it may switch PCLK.
    rate_change_ip : Signal(1)
        Asserted for the entire duration of a rate change.
    current_rate : Signal(4)
        Current effective rate (updated when resets deassert after
        register programming).  Initialised to 0 (Gen1).
    pma_rst_req : Signal(1)
        Request PMA reset assertion from the reset controller.
    pcs_rst_req : Signal(1)
        Request PCS reset assertion from the reset controller.
    fsm_state : Signal(4)
        Numeric FSM state for debug visibility (maps to
        ``PIPEDebugSignature.rate_fsm_state``).
    drp_addr : Signal(24)
        DRP write address presented to the mux.
    drp_wrdata : Signal(32)
        DRP write data presented to the mux.
    drp_wren : Signal(1)
        DRP write enable strobe.
    drp_lock_req : Signal(1)
        DRP bus lock request (held for the entire register burst).

    FSM States
    ----------
    ====  ====  ======================================================
    Code  Name             Description
    ====  ====  ======================================================
     0    IDLE             Waiting for rate mismatch in P0
     1    ASSERT_RESET     Assert PMA+PCS resets, acquire DRP lock
     2    WRITE_REGS       Write 7 CSR registers sequentially
     3    WAIT_PLL_LOCK    Wait for PLL relock (CMU_OK_O)
     4    DEASSERT_RESET   Release resets, update current_rate
     5    WAIT_ACK         Wait for MAC PclkChangeAck, pulse PhyStatus
    ====  ====  ======================================================
    """

    def __init__(self, *, quad: int = 0, lane: int = 0):
        self._quad = quad
        self._lane = lane

        # -- Inputs ----------------------------------------------------------
        self.rate = Signal(4, name="rate_in")
        self.pclk_change_ack = Signal(1, name="pclk_change_ack")
        self.pll_lock = Signal(1, name="rate_pll_lock")
        self.reset_n = Signal(1, name="rate_reset_n")
        self.drp_ready = Signal(1, name="rate_drp_ready")
        self.current_power = Signal(4, name="rate_current_power")

        # -- Outputs ---------------------------------------------------------
        self.phy_status = Signal(1, name="rate_phy_status")
        self.pclk_change_ok = Signal(1, name="pclk_change_ok")
        self.rate_change_ip = Signal(1, name="rate_change_in_progress")
        self.current_rate = Signal(4, name="current_rate", init=0)
        self.pma_rst_req = Signal(1, name="rate_pma_rst_req")
        self.pcs_rst_req = Signal(1, name="rate_pcs_rst_req")
        self.fsm_state = Signal(4, name="rate_fsm_state")

        # -- DRP interface ---------------------------------------------------
        self.drp_addr = Signal(24, name="rate_drp_addr")
        self.drp_wrdata = Signal(32, name="rate_drp_wrdata")
        self.drp_wren = Signal(1, name="rate_drp_wren")
        self.drp_lock_req = Signal(1, name="rate_drp_lock_req")

    def elaborate(self, platform):
        m = Module()

        # ── Rate-change register table (computed for this quad/lane) ────
        # Addresses, Gen1 values, and Gen2 values for the 7 CSRs that
        # differ between 5 GT/s and 10 GT/s operation.  Using Array
        # allows the FSM to index into the table with a counter Signal.
        regs = rate_change_regs(self._quad, self._lane)
        reg_addrs = Array([Const(addr, 24) for addr, _, _, _ in regs])
        gen1_vals = Array([Const(g1, 32) for _, g1, _, _ in regs])
        gen2_vals = Array([Const(g2, 32) for _, _, g2, _ in regs])

        # ── Internal state registers ───────────────────────────────────
        # Register write index (0..6); range includes NUM_REGS to allow
        # the final increment comparison before transitioning.
        reg_idx = Signal(range(_NUM_RATE_REGS + 1))

        # Latched target rate from the MAC request.
        target_rate = Signal(4)

        # Convenience flag: True when switching to Gen2, False for Gen1.
        to_gen2 = Signal(1)

        # PLL relock timeout counter.  At 62.5 MHz (Gen1 PCLK with W80),
        # 20 bits gives ~16 ms maximum wait.  Bit 19 (the MSB) is used
        # as the timeout flag.
        pll_timeout = Signal(20)

        # ── Combinational defaults ─────────────────────────────────────
        # These outputs are driven to their inactive state by default;
        # individual FSM states override them as needed.
        m.d.comb += [
            self.drp_wren.eq(0),
            self.drp_addr.eq(0),
            self.drp_wrdata.eq(0),
            self.drp_lock_req.eq(0),
            self.rate_change_ip.eq(0),
            self.pma_rst_req.eq(0),
            self.pcs_rst_req.eq(0),
        ]

        # phy_status is a single-cycle pulse; clear it every cycle and
        # let the WAIT_ACK state set it for exactly one clock.
        m.d.sync += self.phy_status.eq(0)

        # ── Rate-change FSM ────────────────────────────────────────────
        with m.FSM(name="rate") as fsm:
            # ── IDLE ───────────────────────────────────────────────────
            # Wait for the MAC to request a different rate while the lane
            # is in P0 and reset_n is asserted.
            with m.State("IDLE"):
                m.d.sync += self.fsm_state.eq(0)

                with m.If(
                    (self.rate != self.current_rate)
                    & (self.current_power == 0)
                    & self.reset_n
                ):
                    m.d.sync += [
                        target_rate.eq(self.rate),
                        to_gen2.eq(self.rate[0]),  # bit 0: 0→Gen1, 1→Gen2
                        reg_idx.eq(0),
                    ]
                    m.next = "ASSERT_RESET"

            # ── ASSERT_RESET ───────────────────────────────────────────
            # Assert PMA and PCS resets and request the DRP bus lock.
            # The DRP mux grants the lock within one cycle (priority 0),
            # so we can proceed to writing on the next cycle.
            with m.State("ASSERT_RESET"):
                m.d.sync += self.fsm_state.eq(1)

                m.d.comb += [
                    self.rate_change_ip.eq(1),
                    self.pma_rst_req.eq(1),
                    self.pcs_rst_req.eq(1),
                    self.drp_lock_req.eq(1),
                ]
                m.next = "WRITE_REGS"

            # ── WRITE_REGS ─────────────────────────────────────────────
            # Sequentially write 7 CSR registers.  For each register:
            #   - Present address and data on the DRP bus
            #   - Assert drp_wren
            #   - Wait for drp_ready (write accepted)
            #   - Advance reg_idx
            # After the 7th write completes, transition to PLL relock.
            with m.State("WRITE_REGS"):
                m.d.sync += self.fsm_state.eq(2)

                m.d.comb += [
                    self.rate_change_ip.eq(1),
                    self.pma_rst_req.eq(1),
                    self.pcs_rst_req.eq(1),
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(reg_addrs[reg_idx]),
                    self.drp_wrdata.eq(
                        Mux(to_gen2, gen2_vals[reg_idx], gen1_vals[reg_idx])
                    ),
                    self.drp_wren.eq(1),
                ]

                with m.If(self.drp_ready):
                    with m.If(reg_idx == _NUM_RATE_REGS - 1):
                        # All 7 registers written — release DRP lock
                        # (lock_req goes low when we leave this state)
                        m.d.sync += pll_timeout.eq(0)
                        m.next = "WAIT_PLL_LOCK"
                    with m.Else():
                        m.d.sync += reg_idx.eq(reg_idx + 1)

            # ── WAIT_PLL_LOCK ──────────────────────────────────────────
            # Wait for PLL/CMU to relock after the register changes.
            # The PLL typically relocks in < 200 µs.  The timeout counter
            # provides a ~16 ms safety ceiling; if exceeded, we proceed
            # anyway (the link will likely fail training, which the MAC
            # will detect and retry).
            with m.State("WAIT_PLL_LOCK"):
                m.d.sync += self.fsm_state.eq(3)

                m.d.comb += [
                    self.rate_change_ip.eq(1),
                    self.pma_rst_req.eq(1),
                    self.pcs_rst_req.eq(1),
                ]

                m.d.sync += pll_timeout.eq(pll_timeout + 1)

                with m.If(self.pll_lock):
                    m.next = "DEASSERT_RESET"
                with m.Elif(pll_timeout[-1]):
                    # Timeout: proceed regardless — the MAC will detect
                    # the link failure during training.
                    m.next = "DEASSERT_RESET"

            # ── DEASSERT_RESET ─────────────────────────────────────────
            # Release PMA and PCS resets (they default to 0 in comb).
            # Update current_rate to reflect the new configuration.
            # Signal PclkChangeOk so the MAC can switch its PCLK source.
            with m.State("DEASSERT_RESET"):
                m.d.sync += self.fsm_state.eq(4)

                m.d.comb += self.rate_change_ip.eq(1)

                m.d.sync += [
                    self.current_rate.eq(target_rate),
                    self.pclk_change_ok.eq(1),
                ]
                m.next = "WAIT_ACK"

            # ── WAIT_ACK ───────────────────────────────────────────────
            # Wait for the MAC to assert PclkChangeAck, indicating it
            # has switched to the new PCLK.  Then pulse PhyStatus for
            # one cycle and deassert PclkChangeOk.
            with m.State("WAIT_ACK"):
                m.d.sync += self.fsm_state.eq(5)

                m.d.comb += self.rate_change_ip.eq(1)

                with m.If(self.pclk_change_ack):
                    m.d.sync += [
                        self.phy_status.eq(1),
                        self.pclk_change_ok.eq(0),
                    ]
                    m.next = "IDLE"

        return m
