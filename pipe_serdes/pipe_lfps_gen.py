"""LFPS pattern generator and TX data mux for PIPE SerDes.

Generates a 20-100 MHz square wave on ``tx_data`` by alternating
all-1s and all-0s words at the TX parallel clock rate.  Implements
both PHY-generated LFPS (default) and MAC-generated LFPS
(``MacTransmitLFPS=1`` per PIPE spec §8.10).

This module is instantiated in the **max_pclk** domain (TX PCS clock)
so the pattern edges are clean and synchronous with the serializer
sample clock.

Half-period calculation
-----------------------
Given ``line_rate`` (bps) and ``width`` (bits):

    PCLK = line_rate / width
    N    = round(PCLK / (2 × 50 MHz))      # target center of 20-100 MHz
    N    = clamp(N, 1, ...)
    freq = PCLK / (2 × N)                  # actual LFPS frequency

All (rate, width) combinations produce a frequency in the 20-100 MHz
range required by USB 3.1 §6.9.1.

TX data mux
-----------
::

    MacTransmitLFPS = 0 (default)        MacTransmitLFPS = 1
    ─────────────────────                ────────────────────
    lfps_active=1:                       lfps_active=1:
      → internal 0xFFF/0x000 pattern       → MAC tx_data passthrough
    lfps_active=0:                       lfps_active=0:
      → MAC tx_data passthrough              → MAC tx_data passthrough

    In both cases, ``tx_data_valid`` is asserted during LFPS.
"""

from amaranth import *


def lfps_half_period(pclk_hz: int) -> int:
    """Compute the LFPS half-period in PCLK cycles.

    Parameters
    ----------
    pclk_hz : int
        Actual TX parallel clock frequency in Hz.
        For Gowin GTR12: ``PCS_TX_O_FABRIC_CLK`` frequency.
        This is ``line_rate / width`` for SDR, or half that for DDR.
        Use the measured/documented value for your SerDes config.

    Returns
    -------
    int
        N — number of PCLK cycles per LFPS half-period.
        Actual LFPS frequency = PCLK / (2 * N).
    """
    target_hz = 50_000_000  # 50 MHz center of [20, 100] MHz
    n = max(1, round(pclk_hz / (2 * target_hz)))
    actual_hz = pclk_hz / (2 * n)
    assert 19_000_000 <= actual_hz <= 101_000_000, (
        f"LFPS freq {actual_hz / 1e6:.1f} MHz out of range for "
        f"pclk={pclk_hz / 1e6:.1f} MHz"
    )
    return n


class PIPELFPSGen(Elaboratable):
    """LFPS pattern generator + MacTransmitLFPS TX data mux.

    Must be instantiated in the **max_pclk** domain (TX PCS clock).

    Parameters
    ----------
    width : int
        Parallel data width in bits (10, 16, 20, 32, 40, 64, or 80).
    line_rate_hz : int
        Line rate in Hz.  Default 5 Gbps (Gen1).
    """

    def __init__(self, width: int = 40, pclk_hz: int = 62_500_000):
        self.width = width
        self.pclk_hz = pclk_hz
        self.half_period = lfps_half_period(pclk_hz)

        all_ones = (1 << width) - 1

        self._all_ones = all_ones
        self._all_zeros = 0

        # ── Inputs ────────────────────────────────────
        # From MAC (PIPE command signals)
        self.mac_tx_data = Signal(width)  # MAC's normal TX data
        self.mac_tx_data_valid = Signal()  # MAC's TX data valid
        self.mac_tx_elec_idle = Signal()  # MAC's TxElecIdle

        # From LFPS controller (PHY internal)
        self.lfps_active = Signal()  # LFPS controller in ACTIVE state

        # Configuration (from MacTransmitLFPS register, default = PHY mode)
        self.mac_transmit_lfps = Signal()  # 0 = PHY generates, 1 = MAC generates

        # ── Outputs ───────────────────────────────────
        # To serializer (via txpath)
        self.tx_data = Signal(width)  # Muxed TX data
        self.tx_data_valid = Signal()  # Muxed TX data valid
        self.tx_elec_idle = Signal()  # Muxed TxElecIdle

    def elaborate(self, platform):
        m = Module()

        W = self.width
        N = self.half_period

        # ── Pattern generator (runs on sync = max_pclk after DomainRename) ──
        # Counter counts 0..2N-1.  Output is all-1s for first N cycles,
        # all-0s for next N cycles.
        if N == 1:
            # Toggle every cycle: alternate all-1s / all-0s.
            # At 125 MHz with W40: each value occupies 40 bits (8 ns)
            # on the wire → full period = 80 bits = 16 ns → 62.5 MHz.
            # Measured: 31.25 MHz — the GTR12 gearing serializes
            # lower-20 then upper-20 of each word, producing 80-bit
            # runs before transitioning.  31.25 MHz is within the
            # USB 3.1 LFPS spec (20-100 MHz).
            phase = Signal()
            m.d.sync += phase.eq(~phase)
            lfps_word = Signal(W)
            m.d.comb += lfps_word.eq(Mux(phase, self._all_ones, self._all_zeros))
        else:
            cnt = Signal(range(2 * N))
            m.d.sync += cnt.eq(Mux(cnt == 2 * N - 1, 0, cnt + 1))
            lfps_word = Signal(W)
            m.d.comb += lfps_word.eq(Mux(cnt < N, self._all_ones, self._all_zeros))

        # ── TX data mux ──────────────────────────────────────
        #
        # Priority:
        #   1. lfps_active AND mac_transmit_lfps=0 → PHY pattern
        #   2. lfps_active AND mac_transmit_lfps=1 → MAC tx_data (passthrough)
        #   3. Not lfps_active → MAC tx_data (normal operation)
        #
        # During PHY LFPS: tx_elec_idle forced low (driver active),
        #                    tx_data_valid forced high.
        # During MAC LFPS: MAC controls tx_elec_idle and tx_data_valid.

        phy_lfps = self.lfps_active & ~self.mac_transmit_lfps
        mac_lfps = self.lfps_active & self.mac_transmit_lfps

        with m.If(phy_lfps):
            # PHY-generated LFPS: internal pattern, force driver active
            m.d.comb += [
                self.tx_data.eq(lfps_word),
                self.tx_data_valid.eq(1),
                self.tx_elec_idle.eq(0),  # Driver active
            ]
        with m.Elif(mac_lfps):
            # MAC-generated LFPS: pass through MAC tx_data,
            # but force tx_elec_idle=0 so txpath lets data through.
            m.d.comb += [
                self.tx_data.eq(self.mac_tx_data),
                self.tx_data_valid.eq(self.mac_tx_data_valid),
                self.tx_elec_idle.eq(0),  # Driver active
            ]
        with m.Else():
            # Normal operation: MAC passthrough
            m.d.comb += [
                self.tx_data.eq(self.mac_tx_data),
                self.tx_data_valid.eq(self.mac_tx_data_valid),
                self.tx_elec_idle.eq(self.mac_tx_elec_idle),
            ]

        return m
