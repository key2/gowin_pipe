"""CSR bridge: PIPE message bus registers → Gowin DRP.

Translates PIPE 8-bit register writes from the message bus controller
into 32-bit DRP CSR writes for GTR12 registers.  Maintains shadow
registers to avoid read-modify-write overhead.

Registers that don't map to DRP are stored in a local register file
and take effect directly in FPGA logic.

Address Translation Table
-------------------------

===========  ================  ==============  ==============================
PIPE Addr    GTR12 CSR Addr    GTR12 Field     Type
===========  ================  ==============  ==============================
0x400        0x808234          TX FFE_0        DRP bridge (shadow)
0x401        N/A               TxOnesZeros     Direct (compliance, MAC handles)
0x402        0x808238          TX FFE_1        DRP bridge (shadow)
0x403        0x8082D8          TX FFE_2        DRP bridge (shadow)
0x002        N/A               EB Control      Direct
0x003        N/A               RxEqEval        Direct
0x004        N/A               RX Control      Direct
0x008        N/A               RX Ctrl 4       Direct
0x800        N/A               Common Ctrl     Direct (LFPS flag)
0x801        N/A               NELB Ctrl       Direct
===========  ================  ==============  ==============================

Shadow Register Strategy
------------------------

PIPE registers are 8 bits wide while Gowin DRP CSR registers are 32 bits.
A naive approach would require a DRP read-modify-write for every PIPE write
that targets a DRP-mapped register.  Instead we maintain 32-bit shadow
copies of each DRP register in fabric.  When an 8-bit PIPE write arrives
the shadow is updated locally (sync domain) and the full 32-bit value is
pushed to the DRP in one write.  This halves the DRP bus traffic and
eliminates the read-latency penalty.

The shadow registers are initialised to safe power-on defaults that match
the GTR12 hardware reset values documented in the Gowin SERDES reference.
"""

from amaranth.hdl import Signal, Module, Elaboratable

from gowin_serdes.csr_map import CSR, csr_addr


class PIPECSRBridge(Elaboratable):
    """CSR bridge between PIPE message bus and Gowin DRP.

    Accepts PIPE message-bus register transactions (12-bit address,
    8-bit data) and either stores them locally (direct registers) or
    translates them to 32-bit DRP writes via shadow registers.

    CSR DRP addresses are parameterised by *quad* and *lane* so that the
    same module can target any lane in a multi-quad design.

    Parameters
    ----------
    quad : int
        GTR12 quad index (0-based).  Used to compute lane-specific DRP
        addresses via ``compute_csr_addrs(quad, lane)``.
    lane : int
        Lane index within the quad (0-based).

    The FSM has two states:

    * **IDLE** – accepts a new write or read.  Local-register
      transactions complete in a single cycle.  DRP-mapped writes
      update the shadow and issue a DRP write, then transition to
      WAIT_DRP_WR.
    * **WAIT_DRP_WR** – waits for ``drp_ready`` before acknowledging
      the write back to the message bus and returning to IDLE.

    Input Ports
    -----------
    reg_addr : Signal(12)
        Register address from the message bus controller.
    reg_wrdata : Signal(8)
        Write data from the message bus controller.
    reg_wren : Signal(1)
        Write enable – pulsed for one cycle per write.
    reg_rden : Signal(1)
        Read enable – pulsed for one cycle per read.
    reset_n : Signal(1)
        Active-low asynchronous reset (directly from PIPE).
    drp_ready : Signal(1)
        Asserted by the DRP interface when a write has been accepted.
    drp_rdvld : Signal(1)
        Asserted by the DRP interface when read data is valid.
    drp_rddata : Signal(32)
        Read data returned from the DRP interface.

    Output Ports
    ------------
    rd_data : Signal(8)
        Read data returned to the message bus controller.
    rd_valid : Signal(1)
        Asserted for one cycle when ``rd_data`` is valid.
    wr_ack : Signal(1)
        Asserted for one cycle when a write has completed.

    drp_addr : Signal(24)
        DRP address driven during a DRP transaction.
    drp_wrdata : Signal(32)
        DRP write data (full 32-bit shadow value).
    drp_wren : Signal(1)
        DRP write enable.
    drp_rden : Signal(1)
        DRP read enable (reserved for future use).

    Direct Register Outputs (local, no DRP)
    ----------------------------------------
    eb_control : Signal(8)
        Elastic buffer control register (PIPE addr 0x002).
    rx_control_0 : Signal(8)
        PHY RX control 0 / RxEqEval (PIPE addr 0x003).
    rx_control_1 : Signal(8)
        PHY RX control 1 (PIPE addr 0x004).
    rx_control_4 : Signal(8)
        PHY RX control 4 (PIPE addr 0x008).
    common_ctrl_0 : Signal(8)
        PHY common control 0 (PIPE addr 0x800).
    nelb_control : Signal(8)
        Near-end loopback control (PIPE addr 0x801).

    Extracted Flags
    ---------------
    mac_transmit_lfps : Signal(1)
        Bit 0 of ``common_ctrl_0`` (PIPE addr 0x800).  Directly
        drives the LFPS transmit request to the PHY.
    nelb_enable : Signal(1)
        Bit 0 of ``nelb_control`` (PIPE addr 0x801).  Enables
        near-end loopback in the transceiver.
    """

    def __init__(self, *, quad: int = 0, lane: int = 0):
        self._quad = quad
        self._lane = lane

        # --- Inputs from message bus ---
        self.reg_addr = Signal(12)
        self.reg_wrdata = Signal(8)
        self.reg_wren = Signal(1)
        self.reg_rden = Signal(1)
        self.reset_n = Signal(1)
        self.drp_ready = Signal(1)
        self.drp_rdvld = Signal(1)
        self.drp_rddata = Signal(32)

        # --- Outputs to message bus ---
        self.rd_data = Signal(8)
        self.rd_valid = Signal(1)
        self.wr_ack = Signal(1)

        # --- DRP outputs ---
        self.drp_addr = Signal(24)
        self.drp_wrdata = Signal(32)
        self.drp_wren = Signal(1)
        self.drp_rden = Signal(1)

        # --- Local registers (direct, no DRP) ---
        self.eb_control = Signal(8, name="csr_eb_control")
        self.rx_control_0 = Signal(8, name="csr_rx_control_0")
        self.rx_control_1 = Signal(8, name="csr_rx_control_1")
        self.rx_control_4 = Signal(8, name="csr_rx_control_4")
        self.common_ctrl_0 = Signal(8, name="csr_common_ctrl_0")
        self.nelb_control = Signal(8, name="csr_nelb_control")

        # --- Extracted flags ---
        self.mac_transmit_lfps = Signal(1)
        self.nelb_enable = Signal(1)

        # --- Shadow registers for DRP-mapped 32-bit values ---
        # Each shadow holds the full 32-bit DRP register image so that
        # an 8-bit PIPE write can be merged and pushed without a DRP
        # read.  Init values match GTR12 hardware reset defaults.
        self._shadow_ffe0 = Signal(32, init=0x0000F000)
        self._shadow_ffe1 = Signal(32, init=0x00000000)
        self._shadow_ffe2 = Signal(32, init=0x00000110)

    def elaborate(self, platform):
        m = Module()

        # -----------------------------------------------------------------
        # DRP address constants (computed for this quad/lane)
        # -----------------------------------------------------------------
        DRP_FFE0_ADDR = csr_addr(CSR.TX_FFE_C0, self._quad, self._lane)
        DRP_FFE1_ADDR = csr_addr(CSR.TX_FFE_C1, self._quad, self._lane)
        DRP_FFE2_ADDR = csr_addr(CSR.TX_FFE_VDDT, self._quad, self._lane)

        # -----------------------------------------------------------------
        # PIPE register address constants
        # -----------------------------------------------------------------
        ADDR_EB_CTRL = 0x002
        ADDR_RX_CTRL0 = 0x003  # RxEqEval
        ADDR_RX_CTRL1 = 0x004
        ADDR_RX_CTRL4 = 0x008
        ADDR_TX_CTRL0 = 0x400  # TX FFE_0  → DRP 0x808234
        ADDR_TX_CTRL1 = 0x401  # TxOnesZeros (direct, compliance)
        ADDR_TX_CTRL2 = 0x402  # TX FFE_1  → DRP 0x808238
        ADDR_TX_CTRL3 = 0x403  # TX FFE_2  → DRP 0x8082D8
        ADDR_COMMON0 = 0x800
        ADDR_NELB = 0x801

        # -----------------------------------------------------------------
        # Combinational flag extraction
        # -----------------------------------------------------------------
        m.d.comb += [
            self.mac_transmit_lfps.eq(self.common_ctrl_0[0]),
            self.nelb_enable.eq(self.nelb_control[0]),
        ]

        # -----------------------------------------------------------------
        # Default combinational outputs (active-low by default)
        # -----------------------------------------------------------------
        m.d.comb += [
            self.drp_wren.eq(0),
            self.drp_rden.eq(0),
            self.drp_addr.eq(0),
            self.drp_wrdata.eq(0),
            self.rd_valid.eq(0),
            self.rd_data.eq(0),
            self.wr_ack.eq(0),
        ]

        # -----------------------------------------------------------------
        # Bridge FSM
        # -----------------------------------------------------------------
        with m.FSM(name="csr_bridge"):
            # =============================================================
            # IDLE – accept a new message-bus transaction
            # =============================================================
            with m.State("IDLE"):
                with m.If(self.reg_wren):
                    with m.Switch(self.reg_addr):
                        # -- Local registers: immediate single-cycle ack --

                        with m.Case(ADDR_EB_CTRL):
                            m.d.sync += self.eb_control.eq(self.reg_wrdata)
                            m.d.comb += self.wr_ack.eq(1)

                        with m.Case(ADDR_RX_CTRL0):
                            m.d.sync += self.rx_control_0.eq(self.reg_wrdata)
                            m.d.comb += self.wr_ack.eq(1)

                        with m.Case(ADDR_RX_CTRL1):
                            m.d.sync += self.rx_control_1.eq(self.reg_wrdata)
                            m.d.comb += self.wr_ack.eq(1)

                        with m.Case(ADDR_RX_CTRL4):
                            m.d.sync += self.rx_control_4.eq(self.reg_wrdata)
                            m.d.comb += self.wr_ack.eq(1)

                        with m.Case(ADDR_TX_CTRL1):
                            # TxOnesZeros: compliance pattern control.
                            # Handled by the MAC/compliance logic, not DRP.
                            m.d.comb += self.wr_ack.eq(1)

                        with m.Case(ADDR_COMMON0):
                            m.d.sync += self.common_ctrl_0.eq(self.reg_wrdata)
                            m.d.comb += self.wr_ack.eq(1)

                        with m.Case(ADDR_NELB):
                            m.d.sync += self.nelb_control.eq(self.reg_wrdata)
                            m.d.comb += self.wr_ack.eq(1)

                        # -- DRP bridge registers: shadow + DRP write ----
                        # Update the low 8 bits of the shadow, compose
                        # the full 32-bit word, and issue the DRP write.
                        # Ack is deferred until WAIT_DRP_WR.

                        with m.Case(ADDR_TX_CTRL0):
                            m.d.sync += self._shadow_ffe0[:8].eq(self.reg_wrdata)
                            m.d.comb += [
                                self.drp_addr.eq(DRP_FFE0_ADDR),
                                self.drp_wrdata[:8].eq(self.reg_wrdata),
                                self.drp_wrdata[8:].eq(self._shadow_ffe0[8:]),
                                self.drp_wren.eq(1),
                            ]
                            m.next = "WAIT_DRP_WR"

                        with m.Case(ADDR_TX_CTRL2):
                            m.d.sync += self._shadow_ffe1[:8].eq(self.reg_wrdata)
                            m.d.comb += [
                                self.drp_addr.eq(DRP_FFE1_ADDR),
                                self.drp_wrdata[:8].eq(self.reg_wrdata),
                                self.drp_wrdata[8:].eq(self._shadow_ffe1[8:]),
                                self.drp_wren.eq(1),
                            ]
                            m.next = "WAIT_DRP_WR"

                        with m.Case(ADDR_TX_CTRL3):
                            m.d.sync += self._shadow_ffe2[:8].eq(self.reg_wrdata)
                            m.d.comb += [
                                self.drp_addr.eq(DRP_FFE2_ADDR),
                                self.drp_wrdata[:8].eq(self.reg_wrdata),
                                self.drp_wrdata[8:].eq(self._shadow_ffe2[8:]),
                                self.drp_wren.eq(1),
                            ]
                            m.next = "WAIT_DRP_WR"

                        # -- Unknown address: ack and discard silently ----
                        with m.Default():
                            m.d.comb += self.wr_ack.eq(1)

                with m.Elif(self.reg_rden):
                    # Reads are always serviced from local/shadow storage
                    # in a single cycle — no DRP read required.
                    with m.Switch(self.reg_addr):
                        with m.Case(ADDR_EB_CTRL):
                            m.d.comb += [
                                self.rd_data.eq(self.eb_control),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_RX_CTRL0):
                            m.d.comb += [
                                self.rd_data.eq(self.rx_control_0),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_RX_CTRL1):
                            m.d.comb += [
                                self.rd_data.eq(self.rx_control_1),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_RX_CTRL4):
                            m.d.comb += [
                                self.rd_data.eq(self.rx_control_4),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_TX_CTRL0):
                            m.d.comb += [
                                self.rd_data.eq(self._shadow_ffe0[:8]),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_TX_CTRL2):
                            m.d.comb += [
                                self.rd_data.eq(self._shadow_ffe1[:8]),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_TX_CTRL3):
                            m.d.comb += [
                                self.rd_data.eq(self._shadow_ffe2[:8]),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_COMMON0):
                            m.d.comb += [
                                self.rd_data.eq(self.common_ctrl_0),
                                self.rd_valid.eq(1),
                            ]
                        with m.Case(ADDR_NELB):
                            m.d.comb += [
                                self.rd_data.eq(self.nelb_control),
                                self.rd_valid.eq(1),
                            ]
                        with m.Default():
                            # Unknown address reads back zero.
                            m.d.comb += [
                                self.rd_data.eq(0),
                                self.rd_valid.eq(1),
                            ]

            # =============================================================
            # WAIT_DRP_WR – hold until DRP accepts the write
            # =============================================================
            with m.State("WAIT_DRP_WR"):
                with m.If(self.drp_ready):
                    m.d.comb += self.wr_ack.eq(1)
                    m.next = "IDLE"

        return m
