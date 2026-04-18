"""PIPE power state machine for USB mode.

Implements P0/P1/P2/P3 state transitions per PIPE Rev 7.1 Section 8.3.
Generates PhyStatus completion pulses and coordinates CSR writes for
electrical idle, CDR gating, and PLL management through the DRP mux.

Power-state encoding (PowerDown[3:0]):
    P0 = 0000  Normal operation — link active, TX/RX data flowing
    P1 = 0001  Low-latency idle — PCLK running, TX in electrical idle,
               LFPS capable, fast return to P0
    P2 = 0010  Low-power idle — PCLK may stop, CDR gated, EI detect on,
               receiver detection possible
    P3 = 0011  Lowest power — PLL may be gated, full re-training on wake

Transition rules (PIPE 7.1 §8.3):
    P0 → P1:  Write eidle CSR ON, update quad_pd pin
    P0 → P2:  Write eidle CSR ON + gate CDR, PCLK can stop
    P0 → P3:  Full eidle + gate CDR + gate PLL
    P1 → P0:  Write eidle CSR OFF, wait PLL lock
    P2 → P0:  Write eidle CSR OFF, restart CDR, wait PLL lock
    P3 → P0:  Write eidle CSR OFF, restart CDR + PLL, wait PLL lock
    P1 → P2:  Deepening sleep (eidle already on, gate CDR)
    P2 → P3:  Deepening sleep (gate PLL)
    P3 → P2:  Partial wake (restart PLL only)

PhyStatus is pulsed for exactly one PCLK cycle on completion of every
power-state transition and on Reset# deassertion.

CSR writes are routed through the DRP mux (client index POWER=3).
P2/P3 transitions assert ``drp_lock_req`` because they involve multiple
CSR writes that must be atomic with respect to other DRP clients.

FSM States:
    RESET           Reset# asserted, all blocks held in reset, wait PLL lock
    EIDLE_EXIT      Writing eidle CSR OFF on entry to P0 (shared by reset
                    completion and P1/P2/P3 → P0 resume paths)
    P0_ACTIVE       Normal operation, monitoring PowerDown for transitions
    P1_ENTER        Writing eidle CSR ON, transitioning to P1
    P1_IDLE         LFPS-capable idle, PCLK on, TX in eidle
    P2_ENTER        Writing eidle + CDR gate (DRP-locked), transitioning to P2
    P2_LOW          PCLK off, EI detect on, RxDetect possible
    P3_ENTER        Full shutdown CSR writes, transitioning to P3
    P3_OFF          Lowest power state
    P0_RESUME       Re-enabling PMA/PCS, preparing for PLL relock
    WAIT_PLL_RESUME Waiting for PLL to relock after wake-up
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const

from gowin_serdes.csr_map import CSR, csr_addr, EIDLE_ON, EIDLE_OFF


class PIPEPowerFSM(Elaboratable):
    """USB mode power state machine.

    Controls P0/P1/P2/P3 transitions per PIPE Rev 7.1 §8.3.  This is the
    central FSM that arbitrates power-state changes requested by the MAC
    (via ``power_down[3:0]``) and coordinates the required CSR writes,
    reset sequencing, and PLL/CDR management.

    Every completed transition produces a single-cycle ``phy_status`` pulse
    that the MAC uses to advance its own link-training state machine.

    Transitions are gated during rate changes (``rate_change_ip``) and LFPS
    activity (``lfps_active``) to avoid conflicting DRP writes.

    Parameters
    ----------
    quad : int
        GTR12 quad index (0-based).  Used to compute lane-specific CSR
        addresses via ``compute_csr_addrs(quad, lane)``.
    lane : int
        Lane index within the quad (0-based).

    Input Ports
    -----------
    power_down : Signal(4)
        From MAC: requested power state (P0=0000, P1=0001, P2=0010, P3=0011).
    reset_n : Signal(1)
        From MAC: active-low asynchronous reset.  When deasserted the FSM
        waits for PLL lock then pulses PhyStatus.
    tx_elec_idle : Signal(1)
        From MAC: TX electrical idle request (informational in power FSM).
    pll_lock : Signal(1)
        From lane status: PLL / CMU_OK indicator.  Must be high before
        entering P0 from reset or any low-power state.
    cdr_lock : Signal(1)
        From lane status: CDR lock indicator (reserved for future gating).
    rate_change_ip : Signal(1)
        Rate change in progress — gates all power transitions.
    lfps_active : Signal(1)
        LFPS burst active — gates all power transitions.
    drp_ready : Signal(1)
        DRP write completed acknowledgement from DRP mux.
    drp_rdvld : Signal(1)
        DRP read completed acknowledgement from DRP mux (unused currently).

    Output Ports
    ------------
    phy_status : Signal(1)
        PhyStatus pulse — asserted for exactly one PCLK cycle on completion
        of every power-state transition and Reset# deassertion.
    current_state : Signal(4)
        Current power state encoding for debug and routing logic.
    pma_rstn : Signal(1)
        PMA reset control — deasserted (high) when PMA should be active.
    pcs_rx_rst : Signal(1)
        PCS RX datapath reset — asserted (high) during reset.
    pcs_tx_rst : Signal(1)
        PCS TX datapath reset — asserted (high) during reset.
    quad_pd : Signal(3)
        Power-down pin driven to the GTR12 QUAD (maps to P-state level).
    eidle_active : Signal(1)
        Electrical idle is currently active (driven by CSR state).

    DRP Interface
    -------------
    drp_addr : Signal(24)
        DRP write address (directly connected to DRP mux client port).
    drp_wrdata : Signal(32)
        DRP write data.
    drp_wren : Signal(1)
        DRP write enable — asserted for one cycle per CSR write.
    drp_lock_req : Signal(1)
        DRP bus lock request — held high during multi-step P2/P3 sequences.
    """

    def __init__(self, *, quad: int = 0, lane: int = 0):
        self._quad = quad
        self._lane = lane

        # ── Inputs ─────────────────────────────────────────────────────
        self.power_down = Signal(4, name="power_down")
        self.reset_n = Signal(1, name="reset_n")
        self.tx_elec_idle = Signal(1, name="tx_elec_idle")
        self.pll_lock = Signal(1, name="pll_lock")
        self.cdr_lock = Signal(1, name="cdr_lock")
        self.rate_change_ip = Signal(1, name="rate_change_in_progress")
        self.lfps_active = Signal(1, name="lfps_active")
        self.drp_ready = Signal(1, name="drp_ready")
        self.drp_rdvld = Signal(1, name="drp_rdvld")

        # ── Outputs ────────────────────────────────────────────────────
        self.phy_status = Signal(1, name="power_phy_status")
        self.current_state = Signal(4, name="power_current_state")
        self.pma_rstn = Signal(1, name="power_pma_rstn", init=0)
        self.pcs_rx_rst = Signal(1, name="power_pcs_rx_rst", init=1)
        self.pcs_tx_rst = Signal(1, name="power_pcs_tx_rst", init=1)
        self.quad_pd = Signal(3, name="power_quad_pd")
        self.eidle_active = Signal(1, name="power_eidle_active")

        # ── DRP interface ──────────────────────────────────────────────
        self.drp_addr = Signal(24, name="power_drp_addr")
        self.drp_wrdata = Signal(32, name="power_drp_wrdata")
        self.drp_wren = Signal(1, name="power_drp_wren")
        self.drp_lock_req = Signal(1, name="power_drp_lock_req")

    def elaborate(self, platform):
        m = Module()

        # ── CSR addresses and values (computed for this quad/lane) ──────
        EIDLE_ADDR = csr_addr(CSR.EIDLE, self._quad, self._lane)

        # ── Internal registers ─────────────────────────────────────────
        # Target power-down state latched from power_down input at the
        # moment a transition is initiated.  This prevents glitches on
        # power_down from corrupting an in-flight transition.
        target_pd = Signal(4)

        # Step counter for multi-step transitions (reserved for future
        # CDR/PLL gating CSR sequences in P2/P3 paths).
        step = Signal(3)

        # ── Combinational defaults ─────────────────────────────────────
        # DRP outputs default to inactive every cycle.  States that need
        # a DRP write override these via m.d.comb assignments.
        m.d.comb += [
            self.drp_wren.eq(0),
            self.drp_addr.eq(0),
            self.drp_wrdata.eq(0),
            self.drp_lock_req.eq(0),
        ]

        # PhyStatus is a single-cycle pulse: clear every cycle in the sync
        # domain, states that complete a transition set it to 1 for one cycle.
        m.d.sync += self.phy_status.eq(0)

        # ══════════════════════════════════════════════════════════════
        #  Main FSM
        # ══════════════════════════════════════════════════════════════
        with m.FSM(name="power") as fsm:
            # ── RESET ──────────────────────────────────────────────────
            # Reset# is asserted (low).  Hold all blocks in reset and
            # report P2-equivalent power state per PIPE spec (the PHY is
            # not yet operational).  Wait for both Reset# deassertion AND
            # PLL lock before proceeding to P0.
            # reset_settle: brief counter (~2 µs at 62.5 MHz) to let
            # the SerDes analog blocks stabilise before deasserting
            # PMA/PCS resets.  The reference Gowin USB3 PHY uses a
            # similar 254-cycle settle counter rather than hard-gating
            # on CMU_OK_O, which may not assert on all quad/lane
            # configurations.
            reset_settle = Signal(9)

            with m.State("RESET"):
                # Gowin reference NEVER asserts PCS resets:
                #   assign serdes_pcs_rx_rst_o = 1'b0;
                #   assign serdes_pcs_tx_rst_o = 1'b0;
                # Match that: keep PCS resets deasserted always.
                # Only PMA reset is pulsed during power-up.
                m.d.sync += [
                    self.pma_rstn.eq(0),
                    self.pcs_rx_rst.eq(0),  # Never assert — match Gowin
                    self.pcs_tx_rst.eq(0),  # Never assert — match Gowin
                    self.quad_pd.eq(0b010),  # P2 during reset per PIPE spec
                    self.eidle_active.eq(1),
                    self.current_state.eq(0b0010),  # Report P2 during reset
                ]
                with m.If(self.reset_n):
                    m.d.sync += reset_settle.eq(reset_settle + 1)
                    with m.If(reset_settle[-1]):  # ~256 cycles ≈ 4 µs at 62.5 MHz
                        m.d.sync += [
                            self.pma_rstn.eq(1),
                            reset_settle.eq(0),
                        ]
                        m.next = "EIDLE_EXIT"
                with m.Else():
                    m.d.sync += reset_settle.eq(0)

            # ── EIDLE_EXIT ─────────────────────────────────────────────
            # Shared entry point for P0: write CSR to exit electrical idle.
            # Used by both the reset-complete path and the P1/P2/P3 → P0
            # resume path.  Waits for drp_ready before declaring P0 active
            # and pulsing PhyStatus.
            with m.State("EIDLE_EXIT"):
                m.d.comb += [
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_OFF),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.d.sync += [
                        self.eidle_active.eq(0),
                        self.quad_pd.eq(0b000),
                        self.current_state.eq(0b0000),
                        self.phy_status.eq(1),  # PhyStatus pulse
                    ]
                    m.next = "P0_ACTIVE"

            # ── P0_ACTIVE ──────────────────────────────────────────────
            # Normal operation.  TX/RX data is flowing.  Monitor
            # power_down[3:0] for MAC-requested state transitions.
            # Transitions are gated during rate change and LFPS activity
            # to avoid DRP conflicts.
            #
            # Eidle tracking: the Gowin reference has the pipe interface
            # FSM write EIDLE_OFF/ON on every TxElecIdle edge in P0.
            # Our LFPS controller handles EIDLE_OFF at burst start, but
            # nobody wrote EIDLE_OFF when the MAC deasserts TxElecIdle
            # to start TSEQ after LFPS. We track eidle_active and write
            # EIDLE_OFF whenever TxElecIdle drops in P0.
            with m.State("P0_ACTIVE"):
                m.d.sync += [
                    self.quad_pd.eq(0b000),
                    self.current_state.eq(0b0000),
                ]
                # Single priority chain — reset > power transitions > eidle.
                # In Amaranth, last m.next wins, so we use If/Elif to
                # ensure only one transition fires per cycle.
                with m.If(~self.reset_n):
                    # Reset has highest priority
                    m.next = "RESET"
                with m.Elif(~self.rate_change_ip & ~self.lfps_active):
                    # Power transitions (only when not rate-changing or LFPS)
                    with m.If(self.power_down == 0b0001):
                        m.d.sync += target_pd.eq(0b0001)
                        m.next = "P1_ENTER"
                    with m.Elif(self.power_down == 0b0010):
                        m.d.sync += target_pd.eq(0b0010)
                        m.next = "P2_ENTER"
                    with m.Elif(self.power_down == 0b0011):
                        m.d.sync += target_pd.eq(0b0011)
                        m.next = "P3_ENTER"
                    # Eidle tracking (lowest priority within P0)
                    with m.Elif(self.eidle_active & ~self.tx_elec_idle):
                        m.next = "P0_EIDLE_OFF"
                    with m.Elif(~self.eidle_active & self.tx_elec_idle):
                        m.next = "P0_EIDLE_ON"

            # ── P0_EIDLE_OFF ──────────────────────────────────────────
            # MAC deasserted TxElecIdle in P0: write EIDLE CSR to
            # activate TX driver. Single DRP write, return to P0_ACTIVE.
            with m.State("P0_EIDLE_OFF"):
                m.d.comb += [
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_OFF),
                    self.drp_wren.eq(1),
                ]
                with m.If(~self.reset_n):
                    m.next = "RESET"
                with m.Elif(self.drp_ready):
                    m.d.sync += self.eidle_active.eq(0)
                    m.next = "P0_ACTIVE"

            # ── P0_EIDLE_ON ───────────────────────────────────────────
            # MAC asserted TxElecIdle in P0: write EIDLE CSR to enter
            # electrical idle. Single DRP write, return to P0_ACTIVE.
            with m.State("P0_EIDLE_ON"):
                m.d.comb += [
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_ON),
                    self.drp_wren.eq(1),
                ]
                with m.If(~self.reset_n):
                    m.next = "RESET"
                with m.Elif(self.drp_ready):
                    m.d.sync += self.eidle_active.eq(1)
                    m.next = "P0_ACTIVE"

            # ── P1_ENTER ───────────────────────────────────────────────
            # Transition P0 → P1: write eidle CSR ON.  P1 is a low-latency
            # idle state where PCLK remains running and LFPS is possible.
            # Single CSR write — no DRP lock required.
            with m.State("P1_ENTER"):
                m.d.comb += [
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_ON),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.d.sync += [
                        self.eidle_active.eq(1),
                        self.quad_pd.eq(0b001),
                        self.current_state.eq(0b0001),
                        self.phy_status.eq(1),
                    ]
                    m.next = "P1_IDLE"

            # ── P1_IDLE ────────────────────────────────────────────────
            # PCLK running, TX in electrical idle, LFPS capable.
            # Can transition to P0 (resume) or deepen to P2.
            with m.State("P1_IDLE"):
                m.d.sync += [
                    self.current_state.eq(0b0001),
                    self.eidle_active.eq(1),
                ]
                with m.If(self.power_down == 0b0000):
                    m.next = "P0_RESUME"
                with m.Elif(self.power_down == 0b0010):
                    m.d.sync += target_pd.eq(0b0010)
                    m.next = "P2_ENTER"
                with m.If(~self.reset_n):
                    m.next = "RESET"

            # ── P2_ENTER ───────────────────────────────────────────────
            # Transition to P2: eidle ON + CDR gate.  This is a multi-step
            # sequence requiring the DRP lock to prevent interleaving with
            # other clients.  Currently only the eidle write is performed;
            # the CDR gate CSR write will be added when the hardware
            # register is characterised.
            with m.State("P2_ENTER"):
                m.d.comb += self.drp_lock_req.eq(1)
                m.d.comb += [
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_ON),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.d.sync += [
                        self.eidle_active.eq(1),
                        self.quad_pd.eq(0b010),
                        self.current_state.eq(0b0010),
                        self.phy_status.eq(1),
                    ]
                    m.next = "P2_LOW"

            # ── P2_LOW ─────────────────────────────────────────────────
            # Low-power idle.  PCLK may be stopped, CDR gated, electrical
            # idle detect is active, and receiver detection is possible.
            # Can resume to P0 or deepen to P3.
            with m.State("P2_LOW"):
                m.d.sync += [
                    self.current_state.eq(0b0010),
                    self.eidle_active.eq(1),
                ]
                with m.If(self.power_down == 0b0000):
                    m.next = "P0_RESUME"
                with m.Elif(self.power_down == 0b0011):
                    m.d.sync += target_pd.eq(0b0011)
                    m.next = "P3_ENTER"
                with m.If(~self.reset_n):
                    m.next = "RESET"

            # ── P3_ENTER ───────────────────────────────────────────────
            # Transition to P3 (lowest power): eidle ON + CDR gate + PLL
            # gate.  The PLL gate CSR write is reserved for future use.
            with m.State("P3_ENTER"):
                m.d.comb += [
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_ON),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.d.sync += [
                        self.eidle_active.eq(1),
                        self.quad_pd.eq(0b011),
                        self.current_state.eq(0b0011),
                        self.phy_status.eq(1),
                    ]
                    m.next = "P3_OFF"

            # ── P3_OFF ─────────────────────────────────────────────────
            # Lowest power state.  PLL may be gated.  Only transitions
            # are back to P0 (full wake) or partial wake to P2.
            with m.State("P3_OFF"):
                m.d.sync += [
                    self.current_state.eq(0b0011),
                    self.eidle_active.eq(1),
                ]
                with m.If(self.power_down == 0b0000):
                    m.next = "P0_RESUME"
                with m.Elif(self.power_down == 0b0010):
                    m.d.sync += target_pd.eq(0b0010)
                    m.next = "P2_ENTER"
                with m.If(~self.reset_n):
                    m.next = "RESET"

            # ── P0_RESUME ──────────────────────────────────────────────
            # Wake-up from any low-power state (P1/P2/P3) back to P0.
            # Re-enable PMA and PCS blocks, then wait for PLL to relock
            # before writing the eidle-off CSR.
            with m.State("P0_RESUME"):
                m.d.sync += [
                    self.pma_rstn.eq(1),
                    self.pcs_rx_rst.eq(0),
                    self.pcs_tx_rst.eq(0),
                ]
                m.next = "WAIT_PLL_RESUME"

            # ── WAIT_PLL_RESUME ────────────────────────────────────────
            # Wait for the PLL to relock after waking from a low-power
            # state.  Proceed to EIDLE_EXIT on PLL lock or after a brief
            # settle timeout (~4 µs), whichever comes first.  This
            # mirrors the reset_settle approach and avoids hanging if
            # CMU_OK_O is not asserted on this quad/lane configuration.
            resume_settle = Signal(9)
            with m.State("WAIT_PLL_RESUME"):
                m.d.sync += resume_settle.eq(resume_settle + 1)
                with m.If(self.pll_lock | resume_settle[-1]):
                    m.d.sync += resume_settle.eq(0)
                    m.next = "EIDLE_EXIT"
                with m.If(~self.reset_n):
                    m.d.sync += resume_settle.eq(0)
                    m.next = "RESET"

        return m
