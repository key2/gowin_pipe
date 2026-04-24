"""LFPS controller for PIPE SerDes USB mode.

Manages TX LFPS generation and RX LFPS detection.

TX LFPS: Requires reconfiguring TX driver FFE for LFPS-compliant swing,
then toggling electrical idle at LFPS burst rate. The entire sequence
holds the DRP bus lock during DRP write phases.

RX LFPS: Simply monitors RxElecIdle transitions. The MAC interprets
the pattern (burst frequency, duration) for LFPS type classification.

Key concepts from the PIPE spec:
    - TX LFPS in P1: When TxElecIdle=0, PHY transmits LFPS.
    - TX LFPS in P0: When TxElecIdle=1 AND TxDetectRx=1, LFPS signaling.
    - RX LFPS: Monitor RxElecIdle transitions (asynchronous signal from GTR12).
    - TX LFPS requires reconfiguring FFE for LFPS swing via 3 CSR writes,
      then toggling the eidle CSR to produce the burst.

LFPS FFE Configuration (from CSR map)::

    LFPS_FFE_REGS = [
        (0x808234, 0x0000F000, 0x0000F000, "TX FFE_0"),  # unchanged
        (0x808238, 0x00000000, 0x00000000, "TX FFE_1"),   # no de-emphasis
        (0x8082D8, 0x00000110, 0x00000110, "TX FFE_2"),   # reload
    ]

Eidle toggle: addr=0x8003A4, ON=0x01, OFF=0x07.

The LFPS controller sequence matches the working Gowin USB3 PHY reference
(usb_pipe_interface.v LFPS_0..LFPS_3 + upar_csr.v):

    1. Save FFE registers (3 DRP writes with LFPS values)
    2. Write eidle OFF to exit electrical idle (1 DRP write)
    3. Settle: wait ~33 cycles for TX driver to power up
    4. LFPS pattern active (lfps_pattern_en asserted, MAC controls duration)
    5. Write eidle ON for clean burst termination (1 DRP write)
    6. Restore FFE registers (3 DRP writes with normal values)
    7. Release DRP lock, return to IDLE

Two outputs control different aspects:
    - ``lfps_active``:     guards power FSM transitions and DRP arbitration.
                           Asserted in ALL LFPS states except IDLE and
                           FFE_RESTORE (to allow power FSM eidle writes
                           and TSEQ data flow during restore).
    - ``lfps_pattern_en``: gates the PIPELFPSGen pattern generator.
                           Only asserted in LFPS_ACTIVE state — AFTER
                           eidle OFF + settling, so the pattern doesn't
                           start while the TX driver is still in idle.
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const, Array

from gowin_serdes.csr_map import CSR, csr_addr, EIDLE_ON, EIDLE_OFF, lfps_ffe_regs

# Settling delay: number of cycles between EIDLE_OFF and pattern start.
# Matches the working Gowin reference usb_pipe_interface.v LFPS_1 state
# which waits 33 cycles (first 15 idle, then 18 active) before LFPS_2.
# At 62.5 MHz this is ~528 ns; at 125 MHz this is ~264 ns.  Both provide
# sufficient time for the TX driver analog stage to power up and stabilise.
LFPS_SETTLE_CYCLES = 33


class PIPELFPSController(Elaboratable):
    """LFPS transmit and receive controller.

    This module handles the low-level DRP sequencing required to produce
    LFPS signaling on the SerDes TX path and to forward RX LFPS indications
    from the GTR12 RXELECIDLE_O pin to the MAC layer.

    CSR addresses are parameterised by *quad* and *lane* so that the same
    module can target any lane in a multi-quad design.

    TX LFPS Sequence
    ----------------
    The TX driver FFE coefficients must be temporarily reprogrammed to
    produce full-amplitude LFPS signaling (800-1200 mV per USB 3.2 spec).
    The FSM holds the DRP bus lock during multi-write phases and provides
    a settling delay after exiting electrical idle before the pattern
    generator starts.

    Parameters
    ----------
    quad : int
        GTR12 quad index (0-based).  Used to compute lane-specific CSR
        addresses via ``compute_csr_addrs`` / ``lfps_ffe_regs``.
    lane : int
        Lane index within the quad (0-based).

    FSM States::

        IDLE -> FFE_SAVE -> LFPS_EIDLE_OFF -> LFPS_SETTLE
             -> LFPS_ACTIVE -> LFPS_EIDLE_ON -> FFE_RESTORE -> IDLE

    RX LFPS
    -------
    RxElecIdle from the GTR12 is an asynchronous level signal. This
    module passes it through directly; the MAC is responsible for
    interpreting the burst pattern to classify LFPS type (Polling,
    Ping, U1/U2/U3 exit, etc.).

    Input Ports
    -----------
    tx_elec_idle : Signal(1)
        From MAC. In P1, deassertion (0) triggers LFPS TX.
    tx_detect_rx : Signal(1)
        From MAC. In P0, assertion with tx_elec_idle triggers LFPS.
    current_power : Signal(4)
        Current PIPE power state (0=P0, 1=P1, 2=P2, 3=P3).
    rx_elec_idle_hw : Signal(1)
        Raw RXELECIDLE_O from GTR12. Directly forwarded as rx_elec_idle.
    reset_n : Signal(1)
        Active-low reset. Aborts any in-progress LFPS sequence.
    drp_ready : Signal(1)
        DRP write-completed acknowledgement from the DRP mux/arbiter.

    Output Ports
    ------------
    lfps_active : Signal(1)
        Asserted while the LFPS TX sequence is in progress (all states
        except IDLE and FFE_RESTORE).  Used to gate power FSM
        transitions and DRP arbitration.
    lfps_pattern_en : Signal(1)
        Asserted ONLY in LFPS_ACTIVE state — after EIDLE_OFF + settling.
        Gates the PIPELFPSGen pattern generator so the all-1s/all-0s
        square wave is only driven when the TX driver is actually active.
    lfps_tx_done : Signal(1)
        Single-cycle pulse when LFPS TX sequence completes (FFE restored).
    rx_elec_idle : Signal(1)
        Processed RxElecIdle output (passthrough of rx_elec_idle_hw).
    fsm_state : Signal(4)
        Current FSM state encoding for debug visibility.
    drp_addr : Signal(24)
        DRP write address.
    drp_wrdata : Signal(32)
        DRP write data.
    drp_wren : Signal(1)
        DRP write enable strobe.
    drp_lock_req : Signal(1)
        DRP bus lock request. Held for the entire LFPS TX sequence.
    """

    def __init__(self, *, quad: int = 0, lane: int = 0):
        self._quad = quad
        self._lane = lane

        # --- Inputs ---
        self.tx_elec_idle = Signal(1)
        self.tx_detect_rx = Signal(1)
        self.current_power = Signal(4)
        self.rx_elec_idle_hw = Signal(1)
        self.reset_n = Signal(1)
        self.drp_ready = Signal(1)

        # --- Outputs ---
        self.lfps_active = Signal(1)
        self.lfps_pattern_en = Signal(1)
        self.lfps_tx_done = Signal(1)
        self.rx_elec_idle = Signal(1)
        self.fsm_state = Signal(4)

        # --- DRP interface ---
        self.drp_addr = Signal(24)
        self.drp_wrdata = Signal(32)
        self.drp_wren = Signal(1)
        self.drp_lock_req = Signal(1)

    def elaborate(self, platform):
        m = Module()

        # ------------------------------------------------------------------
        # CSR constants (computed for this quad/lane)
        # ------------------------------------------------------------------
        EIDLE_ADDR = csr_addr(CSR.EIDLE, self._quad, self._lane)

        # FFE register arrays: (address, normal_value, lfps_value)
        # These three CSRs configure the TX driver FFE coefficients.
        # LFPS values = init values (full amplitude, no de-emphasis) to
        # produce the 800-1200 mV LFPS swing per USB 3.2 Table 6-29.
        ffe_regs = lfps_ffe_regs(self._quad, self._lane)
        NUM_FFE = len(ffe_regs)
        ffe_addrs = Array([Const(addr, 24) for addr, _, _, _ in ffe_regs])
        ffe_normal = Array([Const(nv, 32) for _, nv, _, _ in ffe_regs])
        ffe_lfps = Array([Const(lv, 32) for _, _, lv, _ in ffe_regs])

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        # Index into the FFE register arrays (0..2), used by FFE_SAVE and
        # FFE_RESTORE states to iterate over the three registers.
        ffe_idx = Signal(range(NUM_FFE + 1))

        # Settling counter: counts cycles after EIDLE_OFF before LFPS
        # pattern starts.  Matches the 33-cycle delay in the working
        # Gowin USB3 PHY reference (usb_pipe_interface.v LFPS_1).
        settle_cnt = Signal(range(LFPS_SETTLE_CYCLES + 1))

        # ------------------------------------------------------------------
        # RX LFPS: pass through the async RxElecIdle from hardware
        # ------------------------------------------------------------------
        # The MAC layer interprets the timing of RxElecIdle transitions
        # to classify the LFPS type. No filtering is done here.
        m.d.comb += self.rx_elec_idle.eq(self.rx_elec_idle_hw)

        # ------------------------------------------------------------------
        # TX LFPS trigger logic
        # ------------------------------------------------------------------
        # P1 mode: power_down==1, tx_elec_idle==0 means MAC requests LFPS TX
        p1_lfps_trigger = (self.current_power == 1) & ~self.tx_elec_idle
        # P0 mode: tx_elec_idle==1 AND tx_detect_rx==1 means LFPS signaling
        p0_lfps_trigger = (
            (self.current_power == 0) & self.tx_elec_idle & self.tx_detect_rx
        )
        lfps_trigger = p1_lfps_trigger | p0_lfps_trigger
        lfps_stop = ~lfps_trigger

        # ------------------------------------------------------------------
        # Combinational defaults
        # ------------------------------------------------------------------
        m.d.comb += [
            self.drp_wren.eq(0),
            self.drp_addr.eq(0),
            self.drp_wrdata.eq(0),
            self.drp_lock_req.eq(0),
            self.lfps_active.eq(0),
            self.lfps_pattern_en.eq(0),
        ]
        # lfps_tx_done is a single-cycle pulse; clear it every cycle
        m.d.sync += self.lfps_tx_done.eq(0)

        # ------------------------------------------------------------------
        # LFPS TX FSM
        # ------------------------------------------------------------------
        with m.FSM(name="lfps") as fsm:
            # ----------------------------------------------------------
            # IDLE: Wait for LFPS trigger from MAC
            # ----------------------------------------------------------
            with m.State("IDLE"):
                m.d.sync += self.fsm_state.eq(0)
                with m.If(lfps_trigger & self.reset_n):
                    m.d.sync += ffe_idx.eq(0)
                    m.next = "FFE_SAVE"

            # ----------------------------------------------------------
            # FFE_SAVE: Write LFPS FFE values (3 registers sequentially)
            # ----------------------------------------------------------
            # Iterates ffe_idx from 0 to NUM_FFE-1, writing the LFPS
            # coefficient to each FFE CSR. Advances on each drp_ready.
            # lfps_active=1 to block power FSM, but lfps_pattern_en=0
            # so the LFPS gen does NOT override tx_data yet.
            with m.State("FFE_SAVE"):
                m.d.sync += self.fsm_state.eq(1)
                m.d.comb += [
                    self.lfps_active.eq(1),
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(ffe_addrs[ffe_idx]),
                    self.drp_wrdata.eq(ffe_lfps[ffe_idx]),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    with m.If(ffe_idx == NUM_FFE - 1):
                        m.next = "LFPS_EIDLE_OFF"
                    with m.Else():
                        m.d.sync += ffe_idx.eq(ffe_idx + 1)

            # ----------------------------------------------------------
            # LFPS_EIDLE_OFF: Deassert eidle to start waking TX driver
            # ----------------------------------------------------------
            # Writes EIDLE_OFF (0x07) to the eidle CSR.  The TX driver
            # begins powering up but needs time to stabilise.
            # lfps_pattern_en remains 0 — no data pattern yet.
            with m.State("LFPS_EIDLE_OFF"):
                m.d.sync += self.fsm_state.eq(2)
                m.d.comb += [
                    self.lfps_active.eq(1),
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_OFF),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.d.sync += settle_cnt.eq(0)
                    m.next = "LFPS_SETTLE"

            # ----------------------------------------------------------
            # LFPS_SETTLE: Wait for TX driver to power up and stabilise
            # ----------------------------------------------------------
            # Matches the 33-cycle staged-release delay from the working
            # Gowin USB3 PHY reference (usb_pipe_interface.v LFPS_1:
            # cycles 0-15 keep eidle=1, cycles 16-33 eidle=0).
            # Since we already wrote EIDLE_OFF, we just wait the full
            # 33 cycles for the analog to settle.
            # lfps_active=1 to block power FSM.
            # lfps_pattern_en=0 — no pattern until settle completes.
            # DRP lock released so init/power FSMs can proceed if needed.
            with m.State("LFPS_SETTLE"):
                m.d.sync += [
                    self.fsm_state.eq(6),
                    settle_cnt.eq(settle_cnt + 1),
                ]
                m.d.comb += self.lfps_active.eq(1)
                with m.If(settle_cnt == LFPS_SETTLE_CYCLES - 1):
                    m.next = "LFPS_ACTIVE"
                # Abort on reset
                with m.If(~self.reset_n):
                    m.d.sync += ffe_idx.eq(0)
                    m.next = "LFPS_EIDLE_ON"

            # ----------------------------------------------------------
            # LFPS_ACTIVE: LFPS burst in progress
            # ----------------------------------------------------------
            # NOW the pattern generator is enabled (lfps_pattern_en=1).
            # The PIPELFPSGen muxes in the all-1s/all-0s square wave.
            # The MAC controls burst duration by deasserting the trigger
            # signals. Reset also aborts the sequence.
            #
            # On stop: write EIDLE_ON for a clean burst termination
            # before restoring FFE.
            with m.State("LFPS_ACTIVE"):
                m.d.sync += self.fsm_state.eq(3)
                m.d.comb += [
                    self.lfps_active.eq(1),
                    self.lfps_pattern_en.eq(1),
                ]
                with m.If(lfps_stop | ~self.reset_n):
                    m.d.sync += ffe_idx.eq(0)
                    m.next = "LFPS_EIDLE_ON"

            # ----------------------------------------------------------
            # LFPS_EIDLE_ON: Clean burst termination
            # ----------------------------------------------------------
            # Write EIDLE_ON (0x01) to put the TX driver back into
            # electrical idle with a clean edge.  This matches the
            # working reference where tx_eidle is reasserted at the
            # end of the burst (usb_pipe_interface.v:666-669).
            # lfps_pattern_en=0 — pattern stops, idle on bus.
            with m.State("LFPS_EIDLE_ON"):
                m.d.sync += self.fsm_state.eq(4)
                m.d.comb += [
                    self.lfps_active.eq(1),
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(EIDLE_ADDR),
                    self.drp_wrdata.eq(EIDLE_ON),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.next = "FFE_RESTORE"

            # ----------------------------------------------------------
            # FFE_RESTORE: Restore normal FFE values (3 registers)
            # ----------------------------------------------------------
            # Same iteration pattern as FFE_SAVE but writes the normal
            # (data-mode) coefficients back. On completion, pulses
            # lfps_tx_done and returns to IDLE.
            #
            # lfps_active is NOT asserted here. This allows:
            #   1. PIPELFPSGen to pass through normal MAC tx_data
            #      (TSEQ etc.) immediately.
            #   2. The power FSM to write EIDLE_OFF when the MAC
            #      deasserts TxElecIdle for TSEQ.
            # The DRP lock is NOT held — FFE_RESTORE writes interleave
            # with power FSM eidle writes on the DRP bus, matching the
            # reference priority: eidle first, then FFE.
            with m.State("FFE_RESTORE"):
                m.d.sync += self.fsm_state.eq(5)
                m.d.comb += [
                    self.drp_addr.eq(ffe_addrs[ffe_idx]),
                    self.drp_wrdata.eq(ffe_normal[ffe_idx]),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    with m.If(ffe_idx == NUM_FFE - 1):
                        m.d.sync += self.lfps_tx_done.eq(1)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += ffe_idx.eq(ffe_idx + 1)

        return m
