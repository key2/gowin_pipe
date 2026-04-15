"""RX data path adapter for PIPE SerDes.

Maps GTR12 88-bit rx_data bus to PIPE RxData[N-1:0] based on the
configured data width. Generates RxValid from CDR lock status.

RxData is synchronous to RxCLK (recovered clock), NOT PCLK.
RxValid indicates RxCLK stability (CDR lock), not per-word data validity.
The MAC must handle CDC from RxCLK to its internal clock.

The rx_data_o[87:80] bits are status information, NOT symbol data.
Only the lower N bits carry actual received data.
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const


class PIPERXPath(Elaboratable):
    """RX data path width adapter.

    Parameters
    ----------
    max_width : int
        Maximum PIPE data bus width (10, 20, 32, 40, 64, or 80).

    Ports:
        quad_rx_data:       Signal(88)       - Input from GTR12 QUAD
        quad_rx_cdr_lock:   Signal(1)        - Input: CDR lock (PMA_RX_LOCK_O)

        pipe_rx_data:       Signal(max_width) - Output to MAC (sync to RxCLK)
        pipe_rx_valid:      Signal(1)         - Output: RxCLK stable indicator

        active_width:       Signal(3)         - Input: current RxWidth[2:0] encoding
    """

    def __init__(self, max_width=80):
        self.max_width = max_width

        # QUAD side inputs (in RxCLK domain)
        self.quad_rx_data = Signal(88, name="quad_rx_data")
        self.quad_rx_cdr_lock = Signal(1, name="quad_rx_cdr_lock")

        # PIPE side outputs (in RxCLK domain)
        self.pipe_rx_data = Signal(max_width, name="pipe_rx_data")
        self.pipe_rx_valid = Signal(1, name="pipe_rx_valid")

        # Width control
        self.active_width = Signal(3, name="active_rx_width")

        # RxCLK continuation counter: PHY must keep RxCLK running for >=8 clocks
        # after RxValid deasserts. This is a fabric-side guarantee.
        self._rxclk_hold_count = Signal(4, name="rxclk_hold_count")

    def elaborate(self, platform):
        m = Module()

        # Default: zero fill
        m.d.comb += self.pipe_rx_data.eq(0)

        # Width adaptation: extract lower N bits from 88-bit GTR12 bus
        with m.Switch(self.active_width):
            with m.Case(0):  # 10-bit
                m.d.comb += self.pipe_rx_data[:10].eq(self.quad_rx_data[:10])
            with m.Case(1):  # 20-bit
                m.d.comb += self.pipe_rx_data[:20].eq(self.quad_rx_data[:20])
            with m.Case(2):  # 40-bit
                m.d.comb += self.pipe_rx_data[:40].eq(self.quad_rx_data[:40])
            with m.Case(3):  # 32-bit (custom)
                m.d.comb += self.pipe_rx_data[:32].eq(self.quad_rx_data[:32])
            with m.Case(4):  # 64-bit (custom)
                m.d.comb += self.pipe_rx_data[:64].eq(self.quad_rx_data[:64])
            with m.Case(5):  # 80-bit (custom)
                m.d.comb += self.pipe_rx_data[:80].eq(self.quad_rx_data[:80])

        # RxValid = CDR lock is stable
        # When CDR lock deasserts, RxValid goes low, but RxCLK must keep running
        # for at least 8 more clocks (managed externally or counted here as reference)
        m.d.comb += self.pipe_rx_valid.eq(self.quad_rx_cdr_lock)

        # Track RxCLK hold requirement (informational — actual clock gating external)
        with m.If(self.quad_rx_cdr_lock):
            m.d.sync += self._rxclk_hold_count.eq(0)
        with m.Elif(self._rxclk_hold_count < 8):
            m.d.sync += self._rxclk_hold_count.eq(self._rxclk_hold_count + 1)

        return m
