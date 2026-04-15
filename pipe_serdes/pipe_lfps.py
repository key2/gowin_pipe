"""LFPS controller for PIPE SerDes USB mode.

Manages TX LFPS generation and RX LFPS detection.

TX LFPS: Requires reconfiguring TX driver FFE for LFPS-compliant swing,
then toggling electrical idle at LFPS burst rate. The entire sequence
holds the DRP bus lock.

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
        (0x808234, 0x0000F000, 0x00000040, "TX FFE_0"),
        (0x808238, 0x00000000, 0x00000001, "TX FFE_1"),
        (0x8082D8, 0x00000110, 0x00000003, "TX FFE_2"),
    ]

Eidle toggle: addr=0x8003A4, ON=0x01, OFF=0x07.

The LFPS controller is the most complex DRP sequence in the adapter
due to the FFE save/restore bracketing the burst:

    1. Save FFE registers (3 DRP writes with LFPS values)
    2. Toggle eidle OFF to start burst
    3. Wait for MAC to deassert trigger (burst active)
    4. Toggle eidle ON to end burst
    5. Restore FFE registers (3 DRP writes with normal values)
    6. Release DRP lock

The DRP lock is held for the entire sequence (steps 1-5) to prevent
other DRP users (power FSM, rate change) from interleaving writes.
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const, Array

from gowin_serdes.csr_map import CSR, csr_addr, EIDLE_ON, EIDLE_OFF, lfps_ffe_regs


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
    produce the reduced swing required by LFPS. This involves writing
    three FFE CSRs with LFPS-specific values, toggling the electrical
    idle control CSR to produce the burst, then restoring the original
    FFE values. The FSM holds the DRP bus lock for the entire duration
    to guarantee atomicity.

    Parameters
    ----------
    quad : int
        GTR12 quad index (0-based).  Used to compute lane-specific CSR
        addresses via ``compute_csr_addrs`` / ``lfps_ffe_regs``.
    lane : int
        Lane index within the quad (0-based).

    FSM States::

        IDLE -> FFE_SAVE -> LFPS_EIDLE_OFF -> LFPS_ACTIVE
             -> LFPS_EIDLE_ON -> FFE_RESTORE -> IDLE

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
        Asserted while the LFPS TX sequence is in progress.
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
        # Normal values are used during data transmission; LFPS values
        # produce the reduced swing required for LFPS signaling.
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
            # LFPS_EIDLE_OFF: Deassert eidle to start LFPS burst
            # ----------------------------------------------------------
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
                    m.next = "LFPS_ACTIVE"

            # ----------------------------------------------------------
            # LFPS_ACTIVE: Burst in progress, wait for MAC to stop
            # ----------------------------------------------------------
            # The MAC controls burst duration by deasserting the trigger
            # signals. Reset also aborts the sequence.
            with m.State("LFPS_ACTIVE"):
                m.d.sync += self.fsm_state.eq(3)
                m.d.comb += [
                    self.lfps_active.eq(1),
                    self.drp_lock_req.eq(1),
                ]
                with m.If(lfps_stop | ~self.reset_n):
                    m.next = "LFPS_EIDLE_ON"

            # ----------------------------------------------------------
            # LFPS_EIDLE_ON: Assert eidle to end LFPS burst
            # ----------------------------------------------------------
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
                    m.d.sync += ffe_idx.eq(0)
                    m.next = "FFE_RESTORE"

            # ----------------------------------------------------------
            # FFE_RESTORE: Restore normal FFE values (3 registers)
            # ----------------------------------------------------------
            # Same iteration pattern as FFE_SAVE but writes the normal
            # (data-mode) coefficients back. On completion, pulses
            # lfps_tx_done and returns to IDLE.
            with m.State("FFE_RESTORE"):
                m.d.sync += self.fsm_state.eq(5)
                m.d.comb += [
                    self.lfps_active.eq(1),
                    self.drp_lock_req.eq(1),
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
