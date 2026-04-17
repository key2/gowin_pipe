"""Tests for PIPELFPSGen — LFPS pattern generator + MacTransmitLFPS mux.

Tests cover:
  1. Half-period calculation for all supported (rate, width) combos
  2. Pattern frequency verification at W40/Gen1 (N=1, 62.5 MHz)
  3. Pattern frequency verification at W10/Gen1 (N=5, 50 MHz)
  4. Pattern frequency verification at W40/Gen2 (N=3, 41.7 MHz)
  5. Mux: PHY LFPS mode (lfps_active=1, MacTransmitLFPS=0) → internal pattern
  6. Mux: MAC LFPS mode (lfps_active=1, MacTransmitLFPS=1) → MAC passthrough
  7. Mux: Normal operation (lfps_active=0) → MAC passthrough
  8. Pattern is clean: only all-1s or all-0s, never partial
  9. tx_elec_idle forced low during PHY LFPS, follows MAC otherwise

Run:
    cd gowin_pipe && python -m pytest tests/test_lfps_gen.py -v
"""

import unittest
from amaranth import *
from amaranth.sim import Simulator

from pipe_serdes.pipe_lfps_gen import PIPELFPSGen, lfps_half_period


def _run_sim(dut, testbench, *, clk_period=1e-6):
    sim = Simulator(dut)
    sim.add_clock(clk_period)
    sim.add_testbench(testbench)
    sim.run()


class TestHalfPeriodCalc(unittest.TestCase):
    """Verify half-period N for various actual PCLK frequencies.

    Gowin GTR12 uses DDR: actual PCLK = line_rate / (2 × width).
    """

    def _check(self, pclk_hz, expected_n):
        n = lfps_half_period(pclk_hz)
        freq = pclk_hz / (2 * n)
        self.assertEqual(
            n, expected_n, f"pclk={pclk_hz / 1e6}MHz: expected N={expected_n}, got {n}"
        )
        self.assertGreaterEqual(freq, 19e6)
        self.assertLessEqual(freq, 101e6)

    # Gen1 (5G): actual pclk = line_rate / width
    def test_gen1_w10(self):  # pclk = 250 MHz
        self._check(250_000_000, 2)  # 250/(2×2) = 62.5 MHz

    def test_gen1_w20(self):  # 5G/(2×20) = 125 MHz
        self._check(125_000_000, 1)  # 125/(2×1) = 62.5 MHz

    def test_gen1_w40(self):  # 5G/(2×40) = 62.5 MHz
        self._check(62_500_000, 1)  # 62.5/(2×1) = 31.25 MHz

    def test_gen1_w80(self):  # 5G/(2×80) = 31.25 MHz → 15.6 MHz, below spec
        # W80 at Gen1 DDR cannot produce 20 MHz+. This config is not
        # used in practice (W80 only makes sense at Gen2).
        with self.assertRaises(AssertionError):
            lfps_half_period(31_250_000)

    # Gen2 (10G): actual pclk = line_rate / width
    def test_gen2_w10(self):  # pclk = 500 MHz
        self._check(500_000_000, 5)  # 500/(2×5) = 50 MHz

    def test_gen2_w20(self):  # pclk = 250 MHz
        self._check(250_000_000, 2)  # 250/(2×2) = 62.5 MHz

    def test_gen2_w40(self):  # 10G/(2×40) = 125 MHz
        self._check(125_000_000, 1)  # 125/(2×1) = 62.5 MHz

    def test_gen2_w80(self):  # 10G/(2×80) = 62.5 MHz
        self._check(62_500_000, 1)  # 62.5/(2×1) = 31.25 MHz


class TestPatternGen_W40_Gen1(unittest.TestCase):
    """W40/Gen1: N=1, pattern toggles every cycle (62.5 MHz)."""

    def test_pattern_toggles_every_cycle(self):
        dut = PIPELFPSGen(width=40, pclk_hz=62_500_000)
        ALL_ONES = (1 << 40) - 1
        collected = []

        async def testbench(ctx):
            ctx.set(dut.lfps_active, 1)
            ctx.set(dut.mac_transmit_lfps, 0)
            # Let the pattern run for 20 cycles
            for _ in range(20):
                await ctx.tick()
                collected.append(ctx.get(dut.tx_data))

        _run_sim(dut, testbench)

        # After pipeline warmup, should alternate between all-1s and all-0s
        # Skip first 2 cycles for pipeline fill
        for i in range(2, len(collected)):
            val = collected[i]
            self.assertIn(
                val,
                (0, ALL_ONES),
                f"Cycle {i}: expected 0 or {ALL_ONES:#x}, got {val:#x}",
            )

        # Check toggling: consecutive values should differ (after warmup)
        toggles = 0
        for i in range(3, len(collected)):
            if collected[i] != collected[i - 1]:
                toggles += 1
        self.assertGreater(
            toggles, 10, f"Expected toggling pattern, got only {toggles} transitions"
        )

    def test_clean_pattern_no_partial_words(self):
        """Pattern must be either all-1s or all-0s, never partial."""
        dut = PIPELFPSGen(width=40, pclk_hz=62_500_000)
        ALL_ONES = (1 << 40) - 1

        async def testbench(ctx):
            ctx.set(dut.lfps_active, 1)
            ctx.set(dut.mac_transmit_lfps, 0)
            for _ in range(50):
                await ctx.tick()
                val = ctx.get(dut.tx_data)
                self.assertIn(val, (0, ALL_ONES), f"Non-clean pattern: {val:#012x}")

        _run_sim(dut, testbench)


class TestPatternGen_W10_Gen1(unittest.TestCase):
    """W10/Gen1: N=2 at 250 MHz pclk (62.5 MHz LFPS)."""

    def test_half_period_is_5(self):
        dut = PIPELFPSGen(width=10, pclk_hz=500_000_000)  # 500 MHz → N=5
        ALL_ONES = (1 << 10) - 1
        collected = []

        async def testbench(ctx):
            ctx.set(dut.lfps_active, 1)
            ctx.set(dut.mac_transmit_lfps, 0)
            for _ in range(30):
                await ctx.tick()
                collected.append(ctx.get(dut.tx_data))

        _run_sim(dut, testbench)

        # Find first transition after warmup
        start = None
        for i in range(2, len(collected) - 1):
            if collected[i] != collected[i + 1]:
                start = i + 1
                break

        self.assertIsNotNone(start, "No transition found in pattern")

        # Count consecutive same values
        run = 1
        for i in range(start + 1, min(start + 12, len(collected))):
            if collected[i] == collected[start]:
                run += 1
            else:
                break
        self.assertEqual(run, 5, f"Expected run of 5, got {run}")


class TestPatternGen_W40_Gen2(unittest.TestCase):
    """W40/Gen2: pclk=125 MHz → N=1, verify actual half-period matches."""

    def test_half_period_matches(self):
        dut = PIPELFPSGen(width=40, pclk_hz=125_000_000)
        expected_n = dut.half_period
        ALL_ONES = (1 << 40) - 1
        collected = []

        async def testbench(ctx):
            ctx.set(dut.lfps_active, 1)
            ctx.set(dut.mac_transmit_lfps, 0)
            for _ in range(30):
                await ctx.tick()
                collected.append(ctx.get(dut.tx_data))

        _run_sim(dut, testbench)

        start = None
        for i in range(2, len(collected) - 1):
            if collected[i] != collected[i + 1]:
                start = i + 1
                break

        self.assertIsNotNone(start, "No transition found in pattern")

        run = 1
        for i in range(start + 1, min(start + 10, len(collected))):
            if collected[i] == collected[start]:
                run += 1
            else:
                break
        self.assertEqual(run, expected_n, f"Expected run of {expected_n}, got {run}")


class TestMux(unittest.TestCase):
    """TX data mux tests."""

    def test_phy_lfps_overrides_mac(self):
        """lfps_active=1, MacTransmitLFPS=0 → internal pattern, ignore MAC data."""
        dut = PIPELFPSGen(width=40, pclk_hz=62_500_000)
        ALL_ONES = (1 << 40) - 1

        async def testbench(ctx):
            ctx.set(dut.lfps_active, 1)
            ctx.set(dut.mac_transmit_lfps, 0)
            ctx.set(dut.mac_tx_data, 0xDEADBEEF)
            ctx.set(dut.mac_tx_data_valid, 0)
            ctx.set(dut.mac_tx_elec_idle, 1)

            for _ in range(5):
                await ctx.tick()

            val = ctx.get(dut.tx_data)
            vld = ctx.get(dut.tx_data_valid)
            eidle = ctx.get(dut.tx_elec_idle)

            # Should be internal pattern (not 0xDEADBEEF)
            self.assertIn(val, (0, ALL_ONES))
            # tx_data_valid forced high
            self.assertEqual(vld, 1)
            # tx_elec_idle forced low (driver active)
            self.assertEqual(eidle, 0)

        _run_sim(dut, testbench)

    def test_mac_lfps_passthrough(self):
        """lfps_active=1, MacTransmitLFPS=1 → MAC tx_data passthrough."""
        dut = PIPELFPSGen(width=40, pclk_hz=62_500_000)
        MAC_PATTERN = 0xCAFEBABE

        async def testbench(ctx):
            ctx.set(dut.lfps_active, 1)
            ctx.set(dut.mac_transmit_lfps, 1)
            ctx.set(dut.mac_tx_data, MAC_PATTERN)
            ctx.set(dut.mac_tx_data_valid, 1)
            ctx.set(dut.mac_tx_elec_idle, 0)

            await ctx.tick()

            val = ctx.get(dut.tx_data)
            vld = ctx.get(dut.tx_data_valid)
            eidle = ctx.get(dut.tx_elec_idle)

            self.assertEqual(val, MAC_PATTERN)
            self.assertEqual(vld, 1)
            self.assertEqual(eidle, 0)

        _run_sim(dut, testbench)

    def test_normal_operation_passthrough(self):
        """lfps_active=0 → MAC tx_data passthrough regardless of MacTransmitLFPS."""
        dut = PIPELFPSGen(width=40, pclk_hz=62_500_000)
        MAC_DATA = 0x12345678

        async def testbench(ctx):
            ctx.set(dut.lfps_active, 0)
            ctx.set(dut.mac_transmit_lfps, 0)
            ctx.set(dut.mac_tx_data, MAC_DATA)
            ctx.set(dut.mac_tx_data_valid, 1)
            ctx.set(dut.mac_tx_elec_idle, 0)

            await ctx.tick()

            val = ctx.get(dut.tx_data)
            self.assertEqual(val, MAC_DATA)

        _run_sim(dut, testbench)

    def test_tx_elec_idle_behavior(self):
        """tx_elec_idle forced to 0 during any LFPS mode, follows MAC otherwise."""
        dut = PIPELFPSGen(width=40, pclk_hz=62_500_000)

        async def testbench(ctx):
            # Normal mode, MAC says idle
            ctx.set(dut.lfps_active, 0)
            ctx.set(dut.mac_tx_elec_idle, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.tx_elec_idle), 1)

            # Normal mode, MAC says active
            ctx.set(dut.mac_tx_elec_idle, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.tx_elec_idle), 0)

            # MAC LFPS mode: tx_elec_idle forced to 0 (driver active)
            ctx.set(dut.lfps_active, 1)
            ctx.set(dut.mac_transmit_lfps, 1)
            ctx.set(dut.mac_tx_elec_idle, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.tx_elec_idle), 0)

            # PHY LFPS mode: also forced to 0
            ctx.set(dut.mac_transmit_lfps, 0)
            ctx.set(dut.mac_tx_elec_idle, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.tx_elec_idle), 0)

            # Back to normal: follows MAC again
            ctx.set(dut.lfps_active, 0)
            ctx.set(dut.mac_tx_elec_idle, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.tx_elec_idle), 1)

        _run_sim(dut, testbench)


if __name__ == "__main__":
    unittest.main()
