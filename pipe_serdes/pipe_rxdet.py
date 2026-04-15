"""Receiver detection controller for PIPE SerDes.

Implements the GTR12 DRP-based receiver detection pulse sequence.
Detection requires 3 DRP operations + 250-cycle wait = ~5 us total.

The sequence is triggered when TxDetectRx/Loopback=1 and TxElecIdle=1
in P0 or P2 power state.

DRP Sequence (from CSR map):
    1. Write 0x03000000 to 0x80033F  (start detection pulse)
    2. Wait 250 DRP clock cycles (~2.5 us)
    3. Write 0x00000000 to 0x80033F  (end detection pulse)
    4. Read 0x808B34  (result: bit[0] = receiver detected)

PIPE Protocol Behavior:
    - MAC asserts TxDetectRx/Loopback=1 with TxElecIdle=1 (in P0 or P2).
    - PHY performs the detection pulse sequence over DRP.
    - PHY reports completion via a single-cycle PhyStatus pulse and
      RxStatus[2:0]:
        - RxStatus = 0b011 -> Receiver detected
        - RxStatus = 0b000 -> No receiver detected

FSM States:
    IDLE           (0) - Waiting for trigger condition.
    START_PULSE    (1) - Write pulse-start value to RXDET_PULSE_ADDR.
    WAIT_PULSE     (2) - Count 250 DRP clock cycles for pulse duration.
    END_PULSE      (3) - Write pulse-end value to RXDET_PULSE_ADDR.
    READ_RESULT    (4) - Read detection result from RXDET_RESULT_ADDR.
    REPORT         (5) - Assert PhyStatus pulse and present RxStatus.
    WAIT_DEASSERT  (6) - Hold RxStatus until MAC deasserts TxDetectRx.

State Transition Diagram::

    IDLE --[trigger & reset_n]--> START_PULSE
    START_PULSE --[drp_ready]--> WAIT_PULSE
    WAIT_PULSE --[count==249]--> END_PULSE
    END_PULSE --[drp_ready]--> READ_RESULT
    READ_RESULT --[drp_rdvld]--> REPORT
    REPORT --> WAIT_DEASSERT
    WAIT_DEASSERT --[~tx_detect_rx]--> IDLE
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const

from gowin_serdes.csr_map import CSR, csr_addr, RXDET_START, RXDET_END


class PIPERxDetController(Elaboratable):
    """Receiver detection FSM for PIPE SerDes.

    Orchestrates the GTR12 DRP-based receiver detection pulse sequence.
    The controller issues three DRP transactions (write-start, write-end,
    read-result) separated by a 250-cycle pulse window, then reports the
    outcome on the PIPE RxStatus/PhyStatus interface.

    The DRP bus is locked (``drp_lock_req=1``) for the entire duration of
    the three-step atomic sequence to prevent other DRP masters from
    interleaving transactions.

    Parameters
    ----------
    quad : int
        GTR12 quad index (0-based).  Used to compute lane-specific CSR
        addresses via ``compute_csr_addrs(quad, lane)``.
    lane : int
        Lane index within the quad (0-based).

    Input Ports
    -----------
    tx_detect_rx : Signal(1)
        TxDetectRx/Loopback request from the MAC.  Rising edge while
        ``tx_elec_idle=1`` and in P0 or P2 triggers detection.
    tx_elec_idle : Signal(1)
        TxElecIdle from the MAC.  Must be asserted concurrently with
        ``tx_detect_rx`` for a valid trigger.
    current_power : Signal(4)
        Current PIPE power state (0=P0, 2=P2).  Detection is only
        permitted in P0 and P2.
    reset_n : Signal(1)
        Active-low asynchronous reset.  While deasserted the FSM stays
        in IDLE.
    drp_ready : Signal(1)
        Asserted by the DRP mux for one cycle when a write has been
        accepted.
    drp_rdvld : Signal(1)
        Asserted by the DRP mux for one cycle when ``drp_rddata`` is
        valid after a read request.
    drp_rddata : Signal(32)
        DRP read data.  Bit [0] carries the receiver-detected flag.

    Output Ports
    ------------
    phy_status : Signal(1)
        Single-cycle pulse indicating detection is complete.  Directly
        maps to the PIPE PhyStatus output.
    rx_status : Signal(3)
        PIPE RxStatus result:
        - 0b011 : Receiver detected.
        - 0b000 : No receiver detected.
        Valid when ``rx_status_valid`` is asserted.
    rx_status_valid : Signal(1)
        Asserted while ``rx_status`` holds a valid detection result
        (REPORT and WAIT_DEASSERT states).
    busy : Signal(1)
        Asserted throughout the detection sequence (START_PULSE through
        READ_RESULT).  Can be used by higher-level logic to inhibit
        other operations.
    fsm_state : Signal(4)
        Numeric encoding of the current FSM state for debug/trace.
    drp_addr : Signal(24)
        DRP address output to the DRP mux.
    drp_wrdata : Signal(32)
        DRP write data output.
    drp_wren : Signal(1)
        DRP write enable strobe.
    drp_rden : Signal(1)
        DRP read enable strobe.
    drp_lock_req : Signal(1)
        DRP bus lock request.  Held high for the full three-step
        atomic sequence to prevent interleaving.
    """

    def __init__(self, *, quad: int = 0, lane: int = 0):
        self._quad = quad
        self._lane = lane

        # --- Inputs ---
        self.tx_detect_rx = Signal(1)
        self.tx_elec_idle = Signal(1)
        self.current_power = Signal(4)
        self.reset_n = Signal(1)
        self.drp_ready = Signal(1)
        self.drp_rdvld = Signal(1)
        self.drp_rddata = Signal(32)

        # --- Outputs: PIPE status ---
        self.phy_status = Signal(1)
        self.rx_status = Signal(3)
        self.rx_status_valid = Signal(1)
        self.busy = Signal(1)
        self.fsm_state = Signal(4)

        # --- Outputs: DRP bus ---
        self.drp_addr = Signal(24)
        self.drp_wrdata = Signal(32)
        self.drp_wren = Signal(1)
        self.drp_rden = Signal(1)
        self.drp_lock_req = Signal(1)

    def elaborate(self, platform):
        """Build the receiver detection FSM.

        The FSM proceeds through seven states:

        1. **IDLE** -- Waits for the trigger condition (``tx_detect_rx &
           tx_elec_idle`` in P0 or P2 with ``reset_n`` asserted).
        2. **START_PULSE** -- Issues a DRP write of ``0x03000000`` to
           address ``0x80033F`` to begin the detection pulse.  Advances
           on ``drp_ready``.
        3. **WAIT_PULSE** -- Counts 250 DRP clock cycles (~2.5 us) for
           the detection pulse to charge the sense line.
        4. **END_PULSE** -- Issues a DRP write of ``0x00000000`` to
           address ``0x80033F`` to terminate the pulse.  Advances on
           ``drp_ready``.
        5. **READ_RESULT** -- Issues a DRP read from address
           ``0x808B34``.  Latches ``drp_rddata[0]`` as the detected
           flag on ``drp_rdvld``.
        6. **REPORT** -- Asserts a one-cycle ``phy_status`` pulse and
           presents ``rx_status`` (0b011 if detected, 0b000 otherwise).
        7. **WAIT_DEASSERT** -- Holds ``rx_status`` valid until the MAC
           deasserts ``tx_detect_rx``, then returns to IDLE.

        Parameters
        ----------
        platform : Platform or None
            Amaranth platform (unused; present for elaboration API).

        Returns
        -------
        Module
            The elaborated Amaranth module.
        """
        m = Module()

        # -----------------------------------------------------------------
        # CSR addresses and constants (computed for this quad/lane)
        # -----------------------------------------------------------------
        RXDET_PULSE_ADDR = csr_addr(CSR.RXDET_PULSE, self._quad, self._lane)
        RXDET_RESULT_ADDR = csr_addr(CSR.RXDET_RESULT, self._quad, self._lane)
        PULSE_START_VAL = RXDET_START
        PULSE_END_VAL = RXDET_END
        PULSE_WAIT_CYCLES = 250  # ~2.5 us at 100 MHz DRP clock

        # -----------------------------------------------------------------
        # Internal signals
        # -----------------------------------------------------------------
        wait_count = Signal(range(PULSE_WAIT_CYCLES + 1))
        detected = Signal(1)

        # Trigger condition: TxDetectRx=1, TxElecIdle=1, in P0 or P2.
        #
        # Per PIPE spec §5.6.1, receiver detection is valid in P0 or P2.
        # In P0 the same condition also triggers LFPS (§5.5.1); the DRP
        # mux resolves contention via priority (LFPS=client 2 > RxDet=
        # client 3), so LFPS wins the bus when both fire in P0.
        trigger = (
            self.tx_detect_rx
            & self.tx_elec_idle
            & ((self.current_power == 0) | (self.current_power == 2))
            & self.reset_n
        )

        # -----------------------------------------------------------------
        # Combinational defaults (active-low unless overridden per-state)
        # -----------------------------------------------------------------
        m.d.comb += [
            self.drp_wren.eq(0),
            self.drp_rden.eq(0),
            self.drp_addr.eq(0),
            self.drp_wrdata.eq(0),
            self.drp_lock_req.eq(0),
            self.busy.eq(0),
            self.rx_status_valid.eq(0),
        ]
        # PhyStatus is synchronous; default deasserted each cycle
        m.d.sync += self.phy_status.eq(0)

        # -----------------------------------------------------------------
        # FSM
        # -----------------------------------------------------------------
        with m.FSM(name="rxdet") as fsm:
            # --- IDLE: wait for detection trigger -----------------------
            with m.State("IDLE"):
                m.d.sync += self.fsm_state.eq(0)
                with m.If(trigger & self.reset_n):
                    m.next = "START_PULSE"

            # --- START_PULSE: write pulse-start value -------------------
            with m.State("START_PULSE"):
                m.d.sync += self.fsm_state.eq(1)
                m.d.comb += [
                    self.busy.eq(1),
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(RXDET_PULSE_ADDR),
                    self.drp_wrdata.eq(PULSE_START_VAL),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.d.sync += wait_count.eq(0)
                    m.next = "WAIT_PULSE"

            # --- WAIT_PULSE: 250 DRP cycles for sense line charge ------
            with m.State("WAIT_PULSE"):
                m.d.sync += self.fsm_state.eq(2)
                m.d.comb += [
                    self.busy.eq(1),
                    self.drp_lock_req.eq(1),
                ]
                m.d.sync += wait_count.eq(wait_count + 1)
                with m.If(wait_count == PULSE_WAIT_CYCLES - 1):
                    m.next = "END_PULSE"

            # --- END_PULSE: write pulse-end value -----------------------
            with m.State("END_PULSE"):
                m.d.sync += self.fsm_state.eq(3)
                m.d.comb += [
                    self.busy.eq(1),
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(RXDET_PULSE_ADDR),
                    self.drp_wrdata.eq(PULSE_END_VAL),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    m.next = "READ_RESULT"

            # --- READ_RESULT: read detection result register ------------
            with m.State("READ_RESULT"):
                m.d.sync += self.fsm_state.eq(4)
                m.d.comb += [
                    self.busy.eq(1),
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(RXDET_RESULT_ADDR),
                    self.drp_rden.eq(1),
                ]
                with m.If(self.drp_rdvld):
                    # Bit [0] of the result register indicates receiver detected
                    m.d.sync += detected.eq(self.drp_rddata[0])
                    m.next = "REPORT"

            # --- REPORT: pulse PhyStatus, present RxStatus --------------
            with m.State("REPORT"):
                m.d.sync += self.fsm_state.eq(5)
                m.d.comb += [
                    self.rx_status_valid.eq(1),
                ]
                with m.If(detected):
                    m.d.comb += self.rx_status.eq(0b011)  # Receiver detected
                with m.Else():
                    m.d.comb += self.rx_status.eq(0b000)  # Not detected
                m.d.sync += self.phy_status.eq(1)
                m.next = "WAIT_DEASSERT"

            # --- WAIT_DEASSERT: hold result until MAC releases trigger --
            with m.State("WAIT_DEASSERT"):
                m.d.sync += self.fsm_state.eq(6)
                # Keep rx_status valid so MAC can sample it
                m.d.comb += self.rx_status_valid.eq(1)
                with m.If(detected):
                    m.d.comb += self.rx_status.eq(0b011)
                with m.Else():
                    m.d.comb += self.rx_status.eq(0b000)
                with m.If(~self.tx_detect_rx):
                    m.next = "IDLE"

        return m
