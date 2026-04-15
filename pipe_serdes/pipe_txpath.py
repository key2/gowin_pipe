"""TX data path adapter for PIPE SerDes.

Maps PIPE TxData[N-1:0] to the GTR12 80-bit tx_data bus based on the
configured data width. The PHY is a raw bit pump — it serializes whatever
bits the MAC puts on the data bus.

At Gen1 (8b/10b): bus carries 10-bit encoded symbols packed contiguously
At Gen2 (128b/132b): bus carries raw bitstream of 132-bit blocks

For partial widths (32, 64), pad bits are ignored by the serializer.
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const


class PIPETXPath(Elaboratable):
    """TX data path width adapter.

    Parameters
    ----------
    max_width : int
        Maximum PIPE data bus width (10, 20, 32, 40, 64, or 80).

    Ports:
        pipe_tx_data:       Signal(max_width) - Input from MAC
        pipe_tx_data_valid: Signal(1)         - Input from MAC (data qualifier)
        pipe_tx_elec_idle:  Signal(1)         - Input: TX electrical idle
        pipe_power_down:    Signal(4)         - Input: current power state

        quad_tx_data:       Signal(80)        - Output to GTR12 QUAD
        quad_tx_vld:        Signal(1)         - Output: TX data valid to QUAD FIFO

        active_width:       Signal(3)         - Input: current Width[2:0] encoding
    """

    def __init__(self, max_width=80):
        self.max_width = max_width

        # PIPE side inputs
        self.pipe_tx_data = Signal(max_width, name="pipe_tx_data")
        self.pipe_tx_data_valid = Signal(1, name="pipe_tx_data_valid")
        self.pipe_tx_elec_idle = Signal(1, name="pipe_tx_elec_idle")
        self.pipe_power_down = Signal(4, name="pipe_power_down")

        # QUAD side outputs
        self.quad_tx_data = Signal(80, name="quad_tx_data")
        self.quad_tx_vld = Signal(1, name="quad_tx_vld")

        # Width control
        self.active_width = Signal(3, name="active_tx_width")

    def elaborate(self, platform):
        m = Module()

        # In P0 with TxElecIdle=0: data flows through
        # In P0 with TxElecIdle=1: data ignored by serializer (eidle via CSR)
        # TxDataValid gates whether data is consumed
        in_p0 = self.pipe_power_down == 0

        # Default: zero fill the 80-bit bus
        m.d.comb += self.quad_tx_data.eq(0)

        # Width adaptation: map PIPE TxData to the lower N bits of 80-bit bus
        # Width encoding: 0=10, 1=20, 2=40, 3=32(ext), 4=64(ext), 5=80(ext)
        with m.Switch(self.active_width):
            with m.Case(0):  # 10-bit
                m.d.comb += self.quad_tx_data[:10].eq(self.pipe_tx_data[:10])
            with m.Case(1):  # 20-bit
                m.d.comb += self.quad_tx_data[:20].eq(self.pipe_tx_data[:20])
            with m.Case(2):  # 40-bit
                m.d.comb += self.quad_tx_data[:40].eq(self.pipe_tx_data[:40])
            with m.Case(3):  # 32-bit (custom)
                m.d.comb += self.quad_tx_data[:32].eq(self.pipe_tx_data[:32])
            with m.Case(4):  # 64-bit (custom)
                m.d.comb += self.quad_tx_data[:64].eq(self.pipe_tx_data[:64])
            with m.Case(5):  # 80-bit (custom)
                m.d.comb += self.quad_tx_data.eq(self.pipe_tx_data[:80])

        # TX valid: only when in P0, data valid asserted, and NOT in electrical idle
        # (When eidle=1, CSR controls the serializer output, data is ignored)
        m.d.comb += self.quad_tx_vld.eq(
            self.pipe_tx_data_valid & in_p0 & ~self.pipe_tx_elec_idle
        )

        return m
