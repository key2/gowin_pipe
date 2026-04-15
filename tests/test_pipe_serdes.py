"""Comprehensive tests for PIPE SerDes sub-controllers.

Tests cover all sub-controllers using Amaranth's simulation framework
with multi-domain clocking. Each test creates a DUT, drives inputs via
a testbench coroutine, and asserts specific signal values at specific times.

Run with:
    python -m pytest tests/test_pipe_serdes.py -v
    python -m unittest tests.test_pipe_serdes -v
"""

import sys
import os
import unittest

# Ensure the project root and gowin-serdes sibling are importable
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _project_root)
sys.path.insert(0, os.path.join(_project_root, "gowin-serdes"))

from amaranth.hdl import Signal, Module, Elaboratable, Const, ClockDomain
from amaranth.sim import Simulator, Tick

from pipe_serdes.pipe_config import (
    PIPEProtocol,
    PIPEWidth,
    USBRate,
    SATARate,
    PIPEPowerState,
    PIPERxStatus,
    DRPClientID,
    PIPELaneConfig,
    PIPE_WIDTH_MAP,
    PIPE_USB_HW_MAP,
    pclk_mhz,
    RATE_CHANGE_REGS,
    LFPS_FFE_REGS,
    CSR_EIDLE_ADDR,
    CSR_EIDLE_ON,
    CSR_EIDLE_OFF,
    CSR_RXDET_PULSE_ADDR,
    CSR_RXDET_PULSE_START,
    CSR_RXDET_PULSE_END,
    CSR_RXDET_RESULT_ADDR,
    RXDET_PULSE_WAIT_CYCLES,
)
from pipe_serdes.pipe_drp_mux import PIPEDRPMux
from pipe_serdes.pipe_txpath import PIPETXPath
from pipe_serdes.pipe_rxpath import PIPERXPath
from pipe_serdes.pipe_power import PIPEPowerFSM
from pipe_serdes.pipe_rate import PIPERateController
from pipe_serdes.pipe_rxdet import PIPERxDetController
from pipe_serdes.pipe_lfps import PIPELFPSController
from pipe_serdes.pipe_msgbus import PIPEMessageBus
from pipe_serdes.pipe_csr_bridge import PIPECSRBridge
from pipe_serdes.pipe_macclk import PIPEMacCLKGen


# ---------------------------------------------------------------------------
# Wrapper for combinational-only modules to ensure sync domain exists
# ---------------------------------------------------------------------------
class SyncWrapper(Elaboratable):
    """Wraps a sub-module and adds a sync domain anchor (dummy register)."""

    def __init__(self, sub):
        self.sub = sub

    def elaborate(self, platform):
        m = Module()
        m.submodules.dut = self.sub
        # Create a dummy sync register to ensure the "sync" domain exists
        dummy = Signal(1, name="sync_anchor")
        m.d.sync += dummy.eq(~dummy)
        return m


# ═══════════════════════════════════════════════════════════════════════════
#  1. TestPIPEConfig — pure-Python config / enum tests (no simulation)
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPEConfig(unittest.TestCase):
    """Tests for pipe_config.py: enums, width map, pclk calculator, config."""

    # -- PIPELaneConfig creation --

    def test_default_config(self):
        """Default config: USB3, Gen1, W40, msg_bus+mac_clk enabled."""
        cfg = PIPELaneConfig()
        self.assertEqual(cfg.protocol, PIPEProtocol.USB3)
        self.assertEqual(cfg.supported_rates, [USBRate.GEN1])
        self.assertEqual(cfg.default_width, PIPEWidth.W40)
        self.assertIsNone(cfg.default_rx_width)
        self.assertTrue(cfg.enable_msg_bus)
        self.assertTrue(cfg.enable_mac_clk)

    def test_custom_config(self):
        """Custom config with Gen1+Gen2, W80."""
        cfg = PIPELaneConfig(
            protocol=PIPEProtocol.USB3,
            supported_rates=[USBRate.GEN1, USBRate.GEN2],
            default_width=PIPEWidth.W80,
            default_rx_width=PIPEWidth.W64,
            enable_msg_bus=False,
            enable_mac_clk=False,
        )
        self.assertEqual(cfg.max_data_width, 80)
        self.assertEqual(cfg.max_rate, USBRate.GEN2)
        self.assertEqual(cfg.default_rx_width, PIPEWidth.W64)
        self.assertFalse(cfg.enable_msg_bus)

    def test_max_data_width_all_widths(self):
        """max_data_width returns correct fabric_bits for each PIPEWidth."""
        expected = {
            PIPEWidth.W10: 10,
            PIPEWidth.W20: 20,
            PIPEWidth.W40: 40,
            PIPEWidth.W32: 32,
            PIPEWidth.W64: 64,
            PIPEWidth.W80: 80,
        }
        for w, bits in expected.items():
            cfg = PIPELaneConfig(default_width=w)
            self.assertEqual(cfg.max_data_width, bits, f"Failed for {w}")

    # -- PIPE_WIDTH_MAP --

    def test_width_map_keys(self):
        """All PIPEWidth values are present in PIPE_WIDTH_MAP."""
        for w in PIPEWidth:
            self.assertIn(w, PIPE_WIDTH_MAP, f"{w} missing from map")

    def test_width_map_tuple_lengths(self):
        """Each PIPE_WIDTH_MAP entry is a 5-tuple."""
        for w, v in PIPE_WIDTH_MAP.items():
            self.assertEqual(len(v), 5, f"Bad tuple for {w}")

    def test_width_map_fabric_bits(self):
        """PIPE_WIDTH_MAP fabric_bits (index 2) matches PIPEWidth semantics."""
        expected_bits = {
            PIPEWidth.W10: 10,
            PIPEWidth.W20: 20,
            PIPEWidth.W40: 40,
            PIPEWidth.W32: 32,
            PIPEWidth.W64: 64,
            PIPEWidth.W80: 80,
        }
        for w, bits in expected_bits.items():
            self.assertEqual(PIPE_WIDTH_MAP[w][2], bits)

    # -- pclk_mhz --

    def test_pclk_mhz_gen1_w40(self):
        """Gen1 5 GT/s, 40-bit -> 125 MHz."""
        self.assertAlmostEqual(pclk_mhz(5.0, 40), 125.0)

    def test_pclk_mhz_gen2_w40(self):
        """Gen2 10 GT/s, 40-bit -> 250 MHz."""
        self.assertAlmostEqual(pclk_mhz(10.0, 40), 250.0)

    def test_pclk_mhz_gen1_w20(self):
        """Gen1 5 GT/s, 20-bit -> 250 MHz."""
        self.assertAlmostEqual(pclk_mhz(5.0, 20), 250.0)

    def test_pclk_mhz_gen1_w80(self):
        """Gen1 5 GT/s, 80-bit -> 62.5 MHz."""
        self.assertAlmostEqual(pclk_mhz(5.0, 80), 62.5)

    def test_pclk_mhz_gen2_w80(self):
        """Gen2 10 GT/s, 80-bit -> 125 MHz."""
        self.assertAlmostEqual(pclk_mhz(10.0, 80), 125.0)

    # -- to_lane_config --

    def test_to_lane_config_valid_pairs(self):
        """All (rate, width) pairs in PIPE_USB_HW_MAP produce a LaneConfig."""
        for rate, width in PIPE_USB_HW_MAP.keys():
            cfg = PIPELaneConfig(
                supported_rates=[rate],
                default_width=width,
            )
            lc = cfg.to_lane_config(rate=rate, width=width)
            self.assertIsNotNone(lc)

    def test_to_lane_config_invalid_raises(self):
        """Invalid (rate, width) pair raises ValueError."""
        cfg = PIPELaneConfig(
            supported_rates=[USBRate.GEN2],
            default_width=PIPEWidth.W10,
        )
        with self.assertRaises(ValueError):
            cfg.to_lane_config(rate=USBRate.GEN2, width=PIPEWidth.W10)

    def test_to_lane_config_defaults(self):
        """to_lane_config() with no args uses first rate and default_width."""
        cfg = PIPELaneConfig(
            supported_rates=[USBRate.GEN1, USBRate.GEN2],
            default_width=PIPEWidth.W40,
        )
        lc = cfg.to_lane_config()
        self.assertIsNotNone(lc)

    # -- Enum values --

    def test_power_state_values(self):
        """PIPEPowerState P0..P3 encoding matches PIPE spec."""
        self.assertEqual(PIPEPowerState.P0.value, 0b0000)
        self.assertEqual(PIPEPowerState.P1.value, 0b0001)
        self.assertEqual(PIPEPowerState.P2.value, 0b0010)
        self.assertEqual(PIPEPowerState.P3.value, 0b0011)

    def test_rx_status_values(self):
        """PIPERxStatus encoding matches PIPE 7.1 section 5.10."""
        self.assertEqual(PIPERxStatus.OK.value, 0)
        self.assertEqual(PIPERxStatus.RX_DETECTED.value, 3)
        self.assertEqual(PIPERxStatus.DECODE_ERROR.value, 4)

    def test_drp_client_priority_order(self):
        """DRP client IDs are in priority order: RATE(0) > ... > CSR_BRIDGE(5)."""
        self.assertEqual(DRPClientID.RATE.value, 0)
        self.assertEqual(DRPClientID.LFPS.value, 1)
        self.assertEqual(DRPClientID.RXDET.value, 2)
        self.assertEqual(DRPClientID.POWER.value, 3)
        self.assertEqual(DRPClientID.EIDLE.value, 4)
        self.assertEqual(DRPClientID.CSR_BRIDGE.value, 5)
        self.assertEqual(DRPClientID.NUM_CLIENTS.value, 6)


# ═══════════════════════════════════════════════════════════════════════════
#  2. TestPIPEDRPMux
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPEDRPMux(unittest.TestCase):
    """Tests for PIPEDRPMux priority arbiter with locking."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_drp_mux.vcd"):
            sim.run()

    def test_single_client_write(self):
        """Single client write request is forwarded to DRP output."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            # Client 0 writes
            ctx.set(dut.req_addr[0], 0x123456)
            ctx.set(dut.req_wrdata[0], 0xDEADBEEF)
            ctx.set(dut.req_wren[0], 1)
            await ctx.tick()
            # Check DRP output (combinational, same cycle)
            self.assertEqual(ctx.get(dut.drp_addr), 0x123456)
            self.assertEqual(ctx.get(dut.drp_wrdata), 0xDEADBEEF)
            self.assertEqual(ctx.get(dut.drp_wren), 1)

        self._run_sim(dut, testbench)

    def test_single_client_write_response(self):
        """DRP ready response is routed to the issuing client."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            # Client 0 writes
            ctx.set(dut.req_addr[0], 0x123456)
            ctx.set(dut.req_wrdata[0], 0xDEADBEEF)
            ctx.set(dut.req_wren[0], 1)
            await ctx.tick()  # wren seen -> active=1, active_client=0 (sync)
            # Keep writing while DRP processes, then DRP ready comes
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # active is set now, ready routes to client 0
            # Client should see ready
            self.assertEqual(ctx.get(dut.req_ready[0]), 1)

        self._run_sim(dut, testbench)

    def test_priority_lower_index_wins(self):
        """When clients 0 and 3 both request, client 0 wins (lower index = higher priority)."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            # Clients 0 and 3 both request simultaneously
            ctx.set(dut.req_addr[0], 0x000001)
            ctx.set(dut.req_wrdata[0], 0x11111111)
            ctx.set(dut.req_wren[0], 1)
            ctx.set(dut.req_addr[3], 0x000003)
            ctx.set(dut.req_wrdata[3], 0x33333333)
            ctx.set(dut.req_wren[3], 1)
            await ctx.tick()
            # Client 0 should be selected (combinational)
            self.assertEqual(ctx.get(dut.drp_addr), 0x000001)
            self.assertEqual(ctx.get(dut.drp_wrdata), 0x11111111)

        self._run_sim(dut, testbench)

    def test_lock_acquisition_and_release(self):
        """Client acquires DRP lock, holds it, then releases."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            # Client 1 requests lock with a write
            ctx.set(dut.req_addr[1], 0x100000)
            ctx.set(dut.req_wrdata[1], 0xAAAAAAAA)
            ctx.set(dut.req_wren[1], 1)
            ctx.set(dut.req_lock[1], 1)
            await ctx.tick()  # Lock grants on next sync edge
            await ctx.tick()
            # Lock should be granted
            self.assertEqual(ctx.get(dut.req_lock_ack[1]), 1)
            self.assertEqual(ctx.get(dut.dbg_locked), 1)

            # Release lock
            ctx.set(dut.req_lock[1], 0)
            ctx.set(dut.req_wren[1], 0)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.dbg_locked), 0)

        self._run_sim(dut, testbench)

    def test_locked_mux_blocks_other_clients(self):
        """When client 1 holds lock, client 0 (higher priority) is blocked."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            # Client 1 gets lock
            ctx.set(dut.req_addr[1], 0x100000)
            ctx.set(dut.req_wren[1], 1)
            ctx.set(dut.req_lock[1], 1)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.req_lock_ack[1]), 1)

            # Client 0 tries to write while locked
            ctx.set(dut.req_addr[0], 0x000000)
            ctx.set(dut.req_wren[0], 1)
            ctx.set(dut.req_addr[1], 0x200000)
            ctx.set(dut.req_wren[1], 1)
            await ctx.tick()
            # DRP addr should be from client 1 (lock owner), not client 0
            self.assertEqual(ctx.get(dut.drp_addr), 0x200000)

        self._run_sim(dut, testbench)

    def test_response_routing_to_correct_client(self):
        """DRP response is routed only to the client that issued the transaction."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            # Client 2 writes
            ctx.set(dut.req_addr[2], 0x222222)
            ctx.set(dut.req_wrdata[2], 0xBBBBBBBB)
            ctx.set(dut.req_wren[2], 1)
            await ctx.tick()  # wren seen -> active_client=2 registered
            # Keep wren asserted, DRP acks
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # active is set, ready routes to client 2
            # Only client 2 should see ready
            self.assertEqual(ctx.get(dut.req_ready[2]), 1)
            self.assertEqual(ctx.get(dut.req_ready[0]), 0)
            self.assertEqual(ctx.get(dut.req_ready[1]), 0)
            self.assertEqual(ctx.get(dut.req_ready[3]), 0)

        self._run_sim(dut, testbench)

    def test_simultaneous_all_clients(self):
        """All 6 clients request simultaneously -- client 0 wins."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            for i in range(6):
                ctx.set(dut.req_addr[i], i)
                ctx.set(dut.req_wren[i], 1)
            await ctx.tick()
            # Client 0 (highest priority) should win
            self.assertEqual(ctx.get(dut.drp_addr), 0)

        self._run_sim(dut, testbench)

    def test_lock_release_with_pending_requests(self):
        """After lock release, a pending higher-priority client gets access."""
        dut = PIPEDRPMux(num_clients=6)

        async def testbench(ctx):
            # Client 3 grabs lock
            ctx.set(dut.req_addr[3], 0x333333)
            ctx.set(dut.req_wren[3], 1)
            ctx.set(dut.req_lock[3], 1)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.req_lock_ack[3]), 1)

            # Client 1 is waiting
            ctx.set(dut.req_addr[1], 0x111111)
            ctx.set(dut.req_wren[1], 1)
            await ctx.tick()
            # Still locked on client 3
            self.assertEqual(ctx.get(dut.drp_addr), 0x333333)

            # Release lock
            ctx.set(dut.req_lock[3], 0)
            ctx.set(dut.req_wren[3], 0)
            await ctx.tick()
            await ctx.tick()
            # Now client 1 should get access
            self.assertEqual(ctx.get(dut.drp_addr), 0x111111)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  3. TestPIPETXPath
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPETXPath(unittest.TestCase):
    """Tests for PIPETXPath width adaptation and gating."""

    def _run_sim(self, inner_dut, testbench, *, clk_period=1e-6):
        """Wrap a combinational-only DUT in SyncWrapper so sim has sync domain."""
        wrapper = SyncWrapper(inner_dut)
        sim = Simulator(wrapper)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_txpath.vcd"):
            sim.run()

    def test_width_10(self):
        """10-bit width: lower 10 bits mapped, upper 70 bits zero."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 0)  # W10
            ctx.set(dut.pipe_tx_data, 0x3FF)  # 10 bits all 1
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            out = ctx.get(dut.quad_tx_data)
            self.assertEqual(out & 0x3FF, 0x3FF)
            self.assertEqual(out >> 10, 0)  # Upper bits must be zero

        self._run_sim(dut, testbench)

    def test_width_20(self):
        """20-bit width: lower 20 bits mapped, upper 60 bits zero."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 1)  # W20
            ctx.set(dut.pipe_tx_data, 0xFFFFF)
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            out = ctx.get(dut.quad_tx_data)
            self.assertEqual(out & 0xFFFFF, 0xFFFFF)
            self.assertEqual(out >> 20, 0)

        self._run_sim(dut, testbench)

    def test_width_32(self):
        """32-bit width: lower 32 bits mapped, upper 48 bits zero."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 3)  # W32
            ctx.set(dut.pipe_tx_data, 0xDEADBEEF)
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            out = ctx.get(dut.quad_tx_data)
            self.assertEqual(out & 0xFFFFFFFF, 0xDEADBEEF)
            self.assertEqual(out >> 32, 0)

        self._run_sim(dut, testbench)

    def test_width_40(self):
        """40-bit width: lower 40 bits mapped, upper 40 bits zero."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)  # W40
            ctx.set(dut.pipe_tx_data, 0xFF_DEADBEEF)
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            out = ctx.get(dut.quad_tx_data)
            self.assertEqual(out & 0xFFFFFFFFFF, 0xFF_DEADBEEF)
            self.assertEqual(out >> 40, 0)

        self._run_sim(dut, testbench)

    def test_width_64(self):
        """64-bit width: lower 64 bits mapped, upper 16 bits zero."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 4)  # W64
            ctx.set(dut.pipe_tx_data, 0xCAFE_BABE_DEAD_BEEF)
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            out = ctx.get(dut.quad_tx_data)
            self.assertEqual(out & ((1 << 64) - 1), 0xCAFE_BABE_DEAD_BEEF)
            self.assertEqual(out >> 64, 0)

        self._run_sim(dut, testbench)

    def test_width_80(self):
        """80-bit width: all 80 bits mapped."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            val = (1 << 80) - 1
            ctx.set(dut.active_width, 5)  # W80
            ctx.set(dut.pipe_tx_data, val)
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            out = ctx.get(dut.quad_tx_data)
            self.assertEqual(out, val)

        self._run_sim(dut, testbench)

    def test_tx_data_valid_gating(self):
        """quad_tx_vld is 0 when pipe_tx_data_valid is 0."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)
            ctx.set(dut.pipe_tx_data, 0x12345678AB)
            ctx.set(dut.pipe_tx_data_valid, 0)  # NOT valid
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.quad_tx_vld), 0)

        self._run_sim(dut, testbench)

    def test_tx_elec_idle_suppresses_valid(self):
        """quad_tx_vld is 0 when pipe_tx_elec_idle is 1, even if data_valid=1."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)
            ctx.set(dut.pipe_tx_data, 0x12345678AB)
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 0)
            ctx.set(dut.pipe_tx_elec_idle, 1)  # eidle
            await ctx.tick()
            self.assertEqual(ctx.get(dut.quad_tx_vld), 0)

        self._run_sim(dut, testbench)

    def test_not_in_p0_suppresses_valid(self):
        """quad_tx_vld is 0 when power_down != 0 (not P0)."""
        dut = PIPETXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)
            ctx.set(dut.pipe_tx_data, 0x12345678AB)
            ctx.set(dut.pipe_tx_data_valid, 1)
            ctx.set(dut.pipe_power_down, 1)  # P1
            ctx.set(dut.pipe_tx_elec_idle, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.quad_tx_vld), 0)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  4. TestPIPERXPath
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPERXPath(unittest.TestCase):
    """Tests for PIPERXPath width extraction and RxValid/CDR tracking."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_rxpath.vcd"):
            sim.run()

    def test_width_10(self):
        """10-bit width extracts lower 10 bits from 88-bit bus."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 0)
            ctx.set(dut.quad_rx_data, 0x3FF | (0xFF << 80))
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            out = ctx.get(dut.pipe_rx_data)
            self.assertEqual(out & 0x3FF, 0x3FF)

        self._run_sim(dut, testbench)

    def test_width_20(self):
        """20-bit width extracts lower 20 bits."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 1)
            ctx.set(dut.quad_rx_data, 0xABCDE)
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            out = ctx.get(dut.pipe_rx_data)
            self.assertEqual(out & 0xFFFFF, 0xABCDE)

        self._run_sim(dut, testbench)

    def test_width_40(self):
        """40-bit width extracts lower 40 bits."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)
            ctx.set(dut.quad_rx_data, 0xDEADBEEF01)
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            out = ctx.get(dut.pipe_rx_data)
            self.assertEqual(out & 0xFFFFFFFFFF, 0xDEADBEEF01)

        self._run_sim(dut, testbench)

    def test_width_32(self):
        """32-bit width extracts lower 32 bits."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 3)
            ctx.set(dut.quad_rx_data, 0xCAFEBABE)
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            out = ctx.get(dut.pipe_rx_data)
            self.assertEqual(out & 0xFFFFFFFF, 0xCAFEBABE)

        self._run_sim(dut, testbench)

    def test_width_64(self):
        """64-bit width extracts lower 64 bits."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 4)
            ctx.set(dut.quad_rx_data, 0x0102030405060708)
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            out = ctx.get(dut.pipe_rx_data)
            self.assertEqual(out & ((1 << 64) - 1), 0x0102030405060708)

        self._run_sim(dut, testbench)

    def test_width_80(self):
        """80-bit width extracts all 80 lower bits."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            val80 = (1 << 80) - 1
            ctx.set(dut.active_width, 5)
            ctx.set(dut.quad_rx_data, val80)
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            out = ctx.get(dut.pipe_rx_data)
            self.assertEqual(out, val80)

        self._run_sim(dut, testbench)

    def test_rx_valid_tracks_cdr_lock(self):
        """pipe_rx_valid follows quad_rx_cdr_lock."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)
            ctx.set(dut.quad_rx_data, 0)
            ctx.set(dut.quad_rx_cdr_lock, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.pipe_rx_valid), 0)

            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.pipe_rx_valid), 1)

            ctx.set(dut.quad_rx_cdr_lock, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.pipe_rx_valid), 0)

        self._run_sim(dut, testbench)

    def test_rxclk_hold_counter(self):
        """_rxclk_hold_count counts up to 8 after CDR lock loss."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)
            # CDR locked -- counter at 0
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut._rxclk_hold_count), 0)

            # CDR unlocks -- counter starts counting
            ctx.set(dut.quad_rx_cdr_lock, 0)
            for i in range(10):
                await ctx.tick()
            # Should have counted up and saturated at 8
            count = ctx.get(dut._rxclk_hold_count)
            self.assertEqual(count, 8)

        self._run_sim(dut, testbench)

    def test_cdr_lock_toggle(self):
        """CDR lock toggle: counter resets when lock reacquired."""
        dut = PIPERXPath(max_width=80)

        async def testbench(ctx):
            ctx.set(dut.active_width, 2)
            # Lock, count should be 0
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            await ctx.tick()

            # Lose lock, count advances
            ctx.set(dut.quad_rx_cdr_lock, 0)
            for _ in range(3):
                await ctx.tick()
            count = ctx.get(dut._rxclk_hold_count)
            self.assertGreater(count, 0)

            # Re-acquire lock -- counter should reset to 0
            ctx.set(dut.quad_rx_cdr_lock, 1)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut._rxclk_hold_count), 0)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  5. TestPIPEPowerFSM
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPEPowerFSM(unittest.TestCase):
    """Tests for PIPEPowerFSM: P0/P1/P2/P3 state machine."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_power.vcd"):
            sim.run()

    async def _bring_to_p0(self, ctx, dut):
        """Helper: bring the FSM from RESET to P0_ACTIVE."""
        ctx.set(dut.reset_n, 1)
        ctx.set(dut.pll_lock, 1)
        ctx.set(dut.power_down, 0)
        ctx.set(dut.rate_change_ip, 0)
        ctx.set(dut.lfps_active, 0)
        await ctx.tick()  # RESET -> EIDLE_EXIT
        ctx.set(dut.drp_ready, 1)
        await ctx.tick()  # EIDLE_EXIT -> P0_ACTIVE
        ctx.set(dut.drp_ready, 0)
        await ctx.tick()  # settle in P0_ACTIVE

    def test_reset_to_p0_with_pll_lock(self):
        """Reset deassertion with PLL lock -> P0 (phy_status pulse)."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            # Hold in reset
            ctx.set(dut.reset_n, 0)
            ctx.set(dut.pll_lock, 0)
            ctx.set(dut.power_down, 0)
            ctx.set(dut.rate_change_ip, 0)
            ctx.set(dut.lfps_active, 0)
            for _ in range(3):
                await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0010)  # P2 in reset

            # Deassert reset + PLL locks
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.pll_lock, 1)
            await ctx.tick()  # -> EIDLE_EXIT
            # Simulate DRP ready
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P0_ACTIVE
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            # Should be P0
            self.assertEqual(ctx.get(dut.current_state), 0b0000)

        self._run_sim(dut, testbench)

    def test_p0_to_p1_to_p0(self):
        """P0 -> P1 -> P0 transition with eidle CSR writes."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            await self._bring_to_p0(ctx, dut)
            self.assertEqual(ctx.get(dut.current_state), 0b0000)

            # Request P1
            ctx.set(dut.power_down, 0b0001)
            await ctx.tick()  # -> P1_ENTER
            # DRP write of eidle ON
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P1_IDLE
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            # Should be P1
            self.assertEqual(ctx.get(dut.current_state), 0b0001)
            self.assertEqual(ctx.get(dut.eidle_active), 1)

            # Request P0
            ctx.set(dut.power_down, 0b0000)
            await ctx.tick()  # -> P0_RESUME
            await ctx.tick()  # -> WAIT_PLL_RESUME (pll_lock=1 -> EIDLE_EXIT)
            await ctx.tick()  # -> EIDLE_EXIT
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P0_ACTIVE
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0000)

        self._run_sim(dut, testbench)

    def test_p0_to_p2_to_p0(self):
        """P0 -> P2 -> P0 transition."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            await self._bring_to_p0(ctx, dut)

            # Request P2
            ctx.set(dut.power_down, 0b0010)
            await ctx.tick()  # -> P2_ENTER
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P2_LOW
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0010)

            # Resume to P0
            ctx.set(dut.power_down, 0b0000)
            await ctx.tick()  # -> P0_RESUME
            await ctx.tick()  # -> WAIT_PLL_RESUME
            await ctx.tick()  # -> EIDLE_EXIT
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P0_ACTIVE
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0000)

        self._run_sim(dut, testbench)

    def test_p0_to_p3_to_p0(self):
        """P0 -> P3 -> P0 transition."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            await self._bring_to_p0(ctx, dut)

            # P0 -> P3
            ctx.set(dut.power_down, 0b0011)
            await ctx.tick()  # -> P3_ENTER
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P3_OFF
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0011)

            # P3 -> P0
            ctx.set(dut.power_down, 0b0000)
            await ctx.tick()  # -> P0_RESUME
            await ctx.tick()  # -> WAIT_PLL_RESUME
            await ctx.tick()  # -> EIDLE_EXIT
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P0_ACTIVE
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0000)

        self._run_sim(dut, testbench)

    def test_phy_status_single_cycle_pulse(self):
        """PhyStatus is asserted for exactly one cycle on P0 entry."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.pll_lock, 1)
            ctx.set(dut.power_down, 0)
            ctx.set(dut.rate_change_ip, 0)
            ctx.set(dut.lfps_active, 0)
            await ctx.tick()  # -> EIDLE_EXIT
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P0_ACTIVE, phy_status=1 (sync)
            ctx.set(dut.drp_ready, 0)

            # phy_status was set this cycle by sync; check it now
            self.assertEqual(ctx.get(dut.phy_status), 1)
            await ctx.tick()  # phy_status cleared by default eq(0)
            self.assertEqual(ctx.get(dut.phy_status), 0)

        self._run_sim(dut, testbench)

    def test_p1_to_p2_direct(self):
        """P1 -> P2 deepening transition."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            await self._bring_to_p0(ctx, dut)

            # P0 -> P1
            ctx.set(dut.power_down, 0b0001)
            await ctx.tick()
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0001)

            # P1 -> P2
            ctx.set(dut.power_down, 0b0010)
            await ctx.tick()  # -> P2_ENTER
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P2_LOW
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0010)

        self._run_sim(dut, testbench)

    def test_p2_to_p3_to_p2(self):
        """P2 -> P3 -> P2 transitions (P3_OFF supports power_down==2 -> P2_ENTER)."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            await self._bring_to_p0(ctx, dut)

            # P0 -> P2
            ctx.set(dut.power_down, 0b0010)
            await ctx.tick()
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0010)

            # P2 -> P3
            ctx.set(dut.power_down, 0b0011)
            await ctx.tick()  # -> P3_ENTER
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P3_OFF
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0011)

            # P3 -> P2
            ctx.set(dut.power_down, 0b0010)
            await ctx.tick()  # -> P2_ENTER
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()  # -> P2_LOW
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0010)

        self._run_sim(dut, testbench)

    def test_reset_during_p1(self):
        """Reset during P1 returns to RESET state."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            await self._bring_to_p0(ctx, dut)

            # P0 -> P1
            ctx.set(dut.power_down, 0b0001)
            await ctx.tick()
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.current_state), 0b0001)

            # Assert reset
            ctx.set(dut.reset_n, 0)
            await ctx.tick()
            await ctx.tick()
            await ctx.tick()
            # Should be in RESET (P2 encoding)
            self.assertEqual(ctx.get(dut.current_state), 0b0010)

        self._run_sim(dut, testbench)

    def test_rate_change_blocks_power_transition(self):
        """Rate change in progress blocks P0 -> P1 transition."""
        dut = PIPEPowerFSM()

        async def testbench(ctx):
            await self._bring_to_p0(ctx, dut)

            # Rate change in progress -- block P1 request
            ctx.set(dut.rate_change_ip, 1)
            ctx.set(dut.power_down, 0b0001)
            for _ in range(5):
                await ctx.tick()
            # Should still be P0
            self.assertEqual(ctx.get(dut.current_state), 0b0000)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  6. TestPIPERateController
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPERateController(unittest.TestCase):
    """Tests for PIPERateController: Gen1<->Gen2 rate switching."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_rate.vcd"):
            sim.run()

    def test_gen1_to_gen2_rate_change(self):
        """Gen1->Gen2: 7 CSR writes, PLL lock wait, PclkChangeOk/Ack."""
        dut = PIPERateController()

        async def testbench(ctx):
            # Setup: in P0, Gen1, reset deasserted
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 0)
            ctx.set(dut.pll_lock, 0)
            ctx.set(dut.pclk_change_ack, 0)
            ctx.set(dut.rate, 0)  # Gen1 initially
            await ctx.tick()
            await ctx.tick()

            # Request Gen2
            ctx.set(dut.rate, 1)
            await ctx.tick()  # detect mismatch
            await ctx.tick()  # -> ASSERT_RESET
            await ctx.tick()  # -> WRITE_REGS

            # Write 7 registers: for each, hold drp_ready until accepted
            writes_done = 0
            for _ in range(100):
                if ctx.get(dut.drp_wren) == 1:
                    ctx.set(dut.drp_ready, 1)
                    await ctx.tick()
                    writes_done += 1
                    ctx.set(dut.drp_ready, 0)
                else:
                    await ctx.tick()
                if ctx.get(dut.fsm_state) == 3:  # WAIT_PLL_LOCK
                    break

            self.assertGreaterEqual(writes_done, 7)

            # PLL relocks
            ctx.set(dut.pll_lock, 1)
            for _ in range(3):
                await ctx.tick()

            # MAC acks
            ctx.set(dut.pclk_change_ack, 1)
            await ctx.tick()
            ctx.set(dut.pclk_change_ack, 0)
            for _ in range(3):
                await ctx.tick()

            # Rate should be updated to Gen2
            self.assertEqual(ctx.get(dut.current_rate), 1)

        self._run_sim(dut, testbench)

    def test_correct_csr_addresses(self):
        """Verify CSR addresses match RATE_CHANGE_REGS during writes."""
        dut = PIPERateController()
        observed_addrs = []

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 0)
            ctx.set(dut.pll_lock, 0)
            ctx.set(dut.rate, 0)
            await ctx.tick()
            await ctx.tick()

            # Request Gen2
            ctx.set(dut.rate, 1)
            await ctx.tick()
            await ctx.tick()  # ASSERT_RESET
            await ctx.tick()  # WRITE_REGS

            for _ in range(100):
                if ctx.get(dut.drp_wren) == 1:
                    observed_addrs.append(ctx.get(dut.drp_addr))
                    ctx.set(dut.drp_ready, 1)
                    await ctx.tick()
                    ctx.set(dut.drp_ready, 0)
                else:
                    await ctx.tick()
                if ctx.get(dut.fsm_state) == 3:
                    break

        self._run_sim(dut, testbench)

        expected = [r[0] for r in RATE_CHANGE_REGS]
        self.assertEqual(len(observed_addrs), 7)
        self.assertEqual(observed_addrs, expected)

    def test_drp_lock_held_during_writes(self):
        """DRP lock_req is held during the ASSERT_RESET and WRITE_REGS states."""
        dut = PIPERateController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 0)
            ctx.set(dut.pll_lock, 0)
            ctx.set(dut.rate, 0)
            await ctx.tick()
            await ctx.tick()

            ctx.set(dut.rate, 1)
            # Wait until we see ASSERT_RESET (fsm_state=1) with lock_req=1
            for _ in range(10):
                await ctx.tick()
                if ctx.get(dut.fsm_state) == 1:
                    break
            self.assertEqual(ctx.get(dut.drp_lock_req), 1)

            # Now in WRITE_REGS: lock should remain held
            for _ in range(20):
                await ctx.tick()
                if ctx.get(dut.fsm_state) == 2:
                    self.assertEqual(ctx.get(dut.drp_lock_req), 1)
                    break

            # Write all registers while verifying lock
            for _ in range(20):
                if ctx.get(dut.drp_wren) == 1:
                    self.assertEqual(ctx.get(dut.drp_lock_req), 1)
                    ctx.set(dut.drp_ready, 1)
                    await ctx.tick()
                    ctx.set(dut.drp_ready, 0)
                else:
                    await ctx.tick()
                if ctx.get(dut.fsm_state) == 3:
                    break

        self._run_sim(dut, testbench)

    def test_phy_status_pulse_on_completion(self):
        """PhyStatus pulses on rate change completion after PclkChangeAck."""
        dut = PIPERateController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 0)
            ctx.set(dut.pll_lock, 0)
            ctx.set(dut.rate, 0)
            await ctx.tick()
            await ctx.tick()

            ctx.set(dut.rate, 1)
            await ctx.tick()
            await ctx.tick()  # ASSERT_RESET
            await ctx.tick()  # WRITE_REGS

            for _ in range(100):
                if ctx.get(dut.drp_wren) == 1:
                    ctx.set(dut.drp_ready, 1)
                    await ctx.tick()
                    ctx.set(dut.drp_ready, 0)
                else:
                    await ctx.tick()
                if ctx.get(dut.fsm_state) == 3:
                    break

            ctx.set(dut.pll_lock, 1)
            for _ in range(4):
                await ctx.tick()

            ctx.set(dut.pclk_change_ack, 1)
            await ctx.tick()
            # PhyStatus should pulse (set in sync, visible next cycle)
            ps = ctx.get(dut.phy_status)
            await ctx.tick()
            # After the pulse it should be cleared
            self.assertEqual(ctx.get(dut.phy_status), 0)

        self._run_sim(dut, testbench)

    def test_rate_change_ignored_not_in_p0(self):
        """Rate change request is ignored when not in P0."""
        dut = PIPERateController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 1)  # P1, not P0
            ctx.set(dut.pll_lock, 1)
            ctx.set(dut.rate, 0)
            await ctx.tick()
            await ctx.tick()

            ctx.set(dut.rate, 1)  # Request Gen2
            for _ in range(5):
                await ctx.tick()
            # Should still be IDLE (fsm_state=0), no rate change
            self.assertEqual(ctx.get(dut.fsm_state), 0)
            self.assertEqual(ctx.get(dut.rate_change_ip), 0)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  7. TestPIPERxDetController
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPERxDetController(unittest.TestCase):
    """Tests for PIPERxDetController: receiver detection sequence."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_rxdet.vcd"):
            sim.run()

    def test_full_detection_sequence_detected(self):
        """Full RxDet sequence: start -> wait -> end -> read -> REPORT with receiver detected."""
        dut = PIPERxDetController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_detect_rx, 0)
            ctx.set(dut.tx_elec_idle, 1)
            ctx.set(dut.current_power, 0)
            await ctx.tick()
            await ctx.tick()

            # Trigger detection
            ctx.set(dut.tx_detect_rx, 1)
            await ctx.tick()
            await ctx.tick()

            # START_PULSE: write pulse start
            self.assertEqual(ctx.get(dut.fsm_state), 1)
            self.assertEqual(ctx.get(dut.drp_addr), CSR_RXDET_PULSE_ADDR)
            self.assertEqual(ctx.get(dut.drp_wrdata), CSR_RXDET_PULSE_START)
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)

            # WAIT_PULSE: wait 250 cycles
            for _ in range(252):
                await ctx.tick()

            # END_PULSE
            self.assertEqual(ctx.get(dut.fsm_state), 3)
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # READ_RESULT
            self.assertEqual(ctx.get(dut.fsm_state), 4)
            # Return data with bit[1]=1 (receiver detected)
            ctx.set(dut.drp_rddata, 0x00000002)
            ctx.set(dut.drp_rdvld, 1)
            await ctx.tick()
            ctx.set(dut.drp_rdvld, 0)
            await ctx.tick()

            # REPORT: phy_status pulse, rx_status=011
            self.assertEqual(ctx.get(dut.rx_status_valid), 1)
            self.assertEqual(ctx.get(dut.rx_status), 0b011)

            # WAIT_DEASSERT: holds until tx_detect_rx deasserted
            await ctx.tick()
            self.assertEqual(ctx.get(dut.rx_status_valid), 1)
            ctx.set(dut.tx_detect_rx, 0)
            await ctx.tick()
            await ctx.tick()
            # Back to IDLE
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_receiver_not_detected(self):
        """RxDet with no receiver: rx_status=000."""
        dut = PIPERxDetController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_detect_rx, 1)
            ctx.set(dut.tx_elec_idle, 1)
            ctx.set(dut.current_power, 0)
            await ctx.tick()
            await ctx.tick()

            # START_PULSE
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)

            # Wait
            for _ in range(252):
                await ctx.tick()

            # END_PULSE
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # READ_RESULT: bit[1]=0 (not detected)
            ctx.set(dut.drp_rddata, 0x00000000)
            ctx.set(dut.drp_rdvld, 1)
            await ctx.tick()
            ctx.set(dut.drp_rdvld, 0)
            await ctx.tick()

            # Check rx_status=000
            self.assertEqual(ctx.get(dut.rx_status), 0b000)
            self.assertEqual(ctx.get(dut.rx_status_valid), 1)

        self._run_sim(dut, testbench)

    def test_wait_deassert_holds_status(self):
        """Status is held in WAIT_DEASSERT until TxDetectRx deasserted."""
        dut = PIPERxDetController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_detect_rx, 1)
            ctx.set(dut.tx_elec_idle, 1)
            ctx.set(dut.current_power, 0)
            await ctx.tick()
            await ctx.tick()

            # Fast-forward through sequence
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            for _ in range(252):
                await ctx.tick()
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()
            ctx.set(dut.drp_rddata, 0x02)
            ctx.set(dut.drp_rdvld, 1)
            await ctx.tick()
            ctx.set(dut.drp_rdvld, 0)
            await ctx.tick()

            # Now in WAIT_DEASSERT -- status should stay valid
            for _ in range(5):
                await ctx.tick()
                self.assertEqual(ctx.get(dut.rx_status_valid), 1)

            # Deassert
            ctx.set(dut.tx_detect_rx, 0)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_trigger_requires_tx_elec_idle(self):
        """Detection does not trigger when TxElecIdle=0."""
        dut = PIPERxDetController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_detect_rx, 1)
            ctx.set(dut.tx_elec_idle, 0)  # NOT idle
            ctx.set(dut.current_power, 0)
            for _ in range(5):
                await ctx.tick()
            # Should remain in IDLE
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_trigger_requires_p0_or_p2(self):
        """Detection only triggers in P0 or P2, not P1 or P3."""
        dut = PIPERxDetController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_detect_rx, 1)
            ctx.set(dut.tx_elec_idle, 1)
            ctx.set(dut.current_power, 1)  # P1
            for _ in range(5):
                await ctx.tick()
            self.assertEqual(ctx.get(dut.fsm_state), 0)

            ctx.set(dut.current_power, 3)  # P3
            for _ in range(5):
                await ctx.tick()
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_busy_during_sequence(self):
        """busy is asserted during START_PULSE through READ_RESULT."""
        dut = PIPERxDetController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_detect_rx, 1)
            ctx.set(dut.tx_elec_idle, 1)
            ctx.set(dut.current_power, 0)
            await ctx.tick()
            await ctx.tick()

            # START_PULSE
            self.assertEqual(ctx.get(dut.busy), 1)
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)

            # WAIT_PULSE
            await ctx.tick()
            self.assertEqual(ctx.get(dut.busy), 1)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  8. TestPIPELFPSController
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPELFPSController(unittest.TestCase):
    """Tests for PIPELFPSController: LFPS FFE save/toggle/restore."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_lfps.vcd"):
            sim.run()

    def test_ffe_save_eidle_toggle_ffe_restore(self):
        """Full LFPS sequence: FFE save -> eidle off -> active -> eidle on -> FFE restore."""
        dut = PIPELFPSController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_elec_idle, 0)  # P1 trigger: power=1, eidle=0
            ctx.set(dut.tx_detect_rx, 0)
            ctx.set(dut.current_power, 1)  # P1
            await ctx.tick()
            await ctx.tick()

            # Should enter FFE_SAVE
            self.assertEqual(ctx.get(dut.lfps_active), 1)

            # Write 3 FFE regs (LFPS values)
            for i in range(3):
                ctx.set(dut.drp_ready, 1)
                await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # LFPS_EIDLE_OFF
            self.assertEqual(ctx.get(dut.fsm_state), 2)
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # LFPS_ACTIVE -- wait for trigger stop
            self.assertEqual(ctx.get(dut.fsm_state), 3)

            # Stop the trigger by setting eidle=1 (stop condition)
            ctx.set(dut.tx_elec_idle, 1)
            await ctx.tick()
            await ctx.tick()

            # LFPS_EIDLE_ON
            self.assertEqual(ctx.get(dut.fsm_state), 4)
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # FFE_RESTORE: write 3 normal FFE values
            for i in range(3):
                ctx.set(dut.drp_ready, 1)
                await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # Should be back in IDLE
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_drp_lock_held_throughout(self):
        """DRP lock_req is held for the entire LFPS sequence."""
        dut = PIPELFPSController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.tx_elec_idle, 0)
            ctx.set(dut.current_power, 1)
            await ctx.tick()
            await ctx.tick()

            # Check lock during FFE_SAVE
            self.assertEqual(ctx.get(dut.drp_lock_req), 1)
            for _ in range(3):
                ctx.set(dut.drp_ready, 1)
                await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # LFPS_EIDLE_OFF -- still locked
            self.assertEqual(ctx.get(dut.drp_lock_req), 1)

        self._run_sim(dut, testbench)

    def test_p1_trigger_condition(self):
        """LFPS triggers in P1 when tx_elec_idle=0."""
        dut = PIPELFPSController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 1)
            ctx.set(dut.tx_elec_idle, 1)  # No trigger
            ctx.set(dut.tx_detect_rx, 0)
            for _ in range(4):
                await ctx.tick()
            self.assertEqual(ctx.get(dut.fsm_state), 0)  # IDLE

            # Now trigger
            ctx.set(dut.tx_elec_idle, 0)
            await ctx.tick()
            await ctx.tick()
            self.assertNotEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_p0_lfps_trigger(self):
        """LFPS triggers in P0 with tx_elec_idle=1 and tx_detect_rx=1."""
        dut = PIPELFPSController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 0)  # P0
            ctx.set(dut.tx_elec_idle, 1)
            ctx.set(dut.tx_detect_rx, 1)
            await ctx.tick()
            await ctx.tick()
            # Should trigger LFPS
            self.assertNotEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_stop_condition_terminates_burst(self):
        """Removing trigger in LFPS_ACTIVE transitions to LFPS_EIDLE_ON."""
        dut = PIPELFPSController()

        async def testbench(ctx):
            # Trigger in P1
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 1)
            ctx.set(dut.tx_elec_idle, 0)
            await ctx.tick()
            await ctx.tick()

            # FFE save (3 writes)
            for _ in range(3):
                ctx.set(dut.drp_ready, 1)
                await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # EIDLE_OFF
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # LFPS_ACTIVE
            self.assertEqual(ctx.get(dut.fsm_state), 3)

            # Stop: set eidle=1 to stop trigger
            ctx.set(dut.tx_elec_idle, 1)
            await ctx.tick()
            await ctx.tick()
            # Should be LFPS_EIDLE_ON (state 4)
            self.assertEqual(ctx.get(dut.fsm_state), 4)

        self._run_sim(dut, testbench)

    def test_reset_during_lfps(self):
        """Reset during LFPS_ACTIVE aborts to LFPS_EIDLE_ON."""
        dut = PIPELFPSController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 1)
            ctx.set(dut.tx_elec_idle, 0)
            await ctx.tick()
            await ctx.tick()

            for _ in range(3):
                ctx.set(dut.drp_ready, 1)
                await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # LFPS_ACTIVE
            self.assertEqual(ctx.get(dut.fsm_state), 3)

            # Reset
            ctx.set(dut.reset_n, 0)
            await ctx.tick()
            await ctx.tick()
            # Should move to LFPS_EIDLE_ON (state 4)
            self.assertEqual(ctx.get(dut.fsm_state), 4)

        self._run_sim(dut, testbench)

    def test_rx_elec_idle_passthrough(self):
        """rx_elec_idle output directly follows rx_elec_idle_hw input."""
        dut = PIPELFPSController()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.current_power, 0)
            ctx.set(dut.tx_elec_idle, 1)
            ctx.set(dut.tx_detect_rx, 0)

            ctx.set(dut.rx_elec_idle_hw, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.rx_elec_idle), 0)

            ctx.set(dut.rx_elec_idle_hw, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.rx_elec_idle), 1)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  9. TestPIPEMessageBus
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPEMessageBus(unittest.TestCase):
    """Tests for PIPEMessageBus: M2P decode / P2M encode."""

    CMD_NOP = 0b0000
    CMD_WRITE_UNCOMMIT = 0b0001
    CMD_WRITE_COMMIT = 0b0010
    CMD_READ = 0b0011
    CMD_READ_COMPLETION = 0b0100
    CMD_WRITE_ACK = 0b0101

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_msgbus.vcd"):
            sim.run()

    def _m2p_byte(self, cmd, payload=0):
        """Encode an M2P bus byte: cmd[7:4] | payload[3:0]."""
        return ((cmd & 0xF) << 4) | (payload & 0xF)

    def test_write_uncommitted_committed_ack(self):
        """write_uncommitted -> write_committed -> write_ack sequence."""
        dut = PIPEMessageBus(write_buffer_depth=5)

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()

            # -- write_uncommitted: addr=0x1AB, data=0x55 --
            # Byte 1: cmd=0001, addr_hi=0x1
            ctx.set(dut.m2p_bus, self._m2p_byte(self.CMD_WRITE_UNCOMMIT, 0x1))
            await ctx.tick()
            # Byte 2: addr_lo=0xAB
            ctx.set(dut.m2p_bus, 0xAB)
            await ctx.tick()
            # Byte 3: data=0x55
            ctx.set(dut.m2p_bus, 0x55)
            await ctx.tick()
            # BUFFER_WRITE state
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()
            await ctx.tick()

            # -- write_committed: addr=0x2CD, data=0xAA --
            ctx.set(dut.m2p_bus, self._m2p_byte(self.CMD_WRITE_COMMIT, 0x2))
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0xCD)
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0xAA)
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()

            # COMMIT_START -> COMMIT_BUFFERED -> COMMIT_FINAL -> SEND_WRITE_ACK
            found_ack = False
            for _ in range(30):
                ctx.set(dut.csr_wr_ack, 1)
                await ctx.tick()
                if ctx.get(dut.p2m_bus) == 0x50:
                    found_ack = True
                    break
            ctx.set(dut.csr_wr_ack, 0)
            self.assertTrue(found_ack, "write_ack (0x50) not seen on P2M bus")

        self._run_sim(dut, testbench)

    def test_read_completion(self):
        """read -> RD_EXEC -> read_completion (0x40 + data)."""
        dut = PIPEMessageBus(write_buffer_depth=5)

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()

            # Read command: addr=0x3EF
            ctx.set(dut.m2p_bus, self._m2p_byte(self.CMD_READ, 0x3))
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0xEF)
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0)

            # Wait for FSM to reach RD_EXEC (state 8), then supply data
            for _ in range(10):
                await ctx.tick()
                if ctx.get(dut.fsm_state) == 8:
                    break

            # RD_EXEC: supply read data
            ctx.set(dut.csr_rd_data, 0x42)
            ctx.set(dut.csr_rd_valid, 1)
            await ctx.tick()
            ctx.set(dut.csr_rd_valid, 0)

            # Scan for the read_completion sequence on P2M.
            # p2m_bus is combinational: check both before-tick and after-tick.
            found_cmd = False
            found_data = False
            for _ in range(10):
                p2m = ctx.get(dut.p2m_bus)
                if p2m == 0x40:
                    found_cmd = True
                elif found_cmd and p2m == 0x42:
                    found_data = True
                    break
                await ctx.tick()

            self.assertTrue(found_cmd, "read_completion cmd (0x40) not seen on P2M")
            self.assertTrue(found_data, "read_completion data (0x42) not seen on P2M")

        self._run_sim(dut, testbench)

    def test_atomic_write_buffer_3(self):
        """Buffer 3 uncommitted writes, then commit all atomically."""
        dut = PIPEMessageBus(write_buffer_depth=5)

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()

            # 3 uncommitted writes
            for i in range(3):
                ctx.set(dut.m2p_bus, self._m2p_byte(self.CMD_WRITE_UNCOMMIT, i))
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0x10 + i)
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0x80 + i)
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0)
                await ctx.tick()
                await ctx.tick()

            # Committed write
            ctx.set(dut.m2p_bus, self._m2p_byte(self.CMD_WRITE_COMMIT, 0xF))
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0xFF)
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0x99)
            await ctx.tick()
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()

            # Process all commits
            write_count = 0
            for _ in range(30):
                if ctx.get(dut.reg_wren) == 1:
                    write_count += 1
                ctx.set(dut.csr_wr_ack, 1)
                await ctx.tick()
                if ctx.get(dut.p2m_bus) == 0x50:
                    break
            ctx.set(dut.csr_wr_ack, 0)

            # Should have issued 4 writes total (3 buffered + 1 committed)
            self.assertGreaterEqual(write_count, 4)

        self._run_sim(dut, testbench)

    def test_nop_passthrough(self):
        """NOP command (cmd=0, bus=0x00) stays in IDLE."""
        dut = PIPEMessageBus(write_buffer_depth=5)

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            # NOP is 0x00 (bus == 0 is also idle, not a transition)
            ctx.set(dut.m2p_bus, 0x00)
            await ctx.tick()
            await ctx.tick()
            # Should still be in IDLE
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_buffer_overflow_handling(self):
        """Writes beyond buffer depth are silently dropped; no crash."""
        dut = PIPEMessageBus(write_buffer_depth=2)

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()

            # Write 3 uncommitted (buffer depth=2, so 3rd overflows)
            for i in range(3):
                ctx.set(dut.m2p_bus, self._m2p_byte(self.CMD_WRITE_UNCOMMIT, i))
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0x10 + i)
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0x80 + i)
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0)
                await ctx.tick()
                await ctx.tick()

            # Should return to IDLE without crash
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)

    def test_back_to_back_transactions(self):
        """Two write_committed transactions back-to-back."""
        dut = PIPEMessageBus(write_buffer_depth=5)

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            ctx.set(dut.m2p_bus, 0)
            await ctx.tick()

            for txn in range(2):
                # write_committed: addr=0x000, data=0x11
                ctx.set(dut.m2p_bus, self._m2p_byte(self.CMD_WRITE_COMMIT, 0x0))
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0x00)
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0x11)
                await ctx.tick()
                ctx.set(dut.m2p_bus, 0)
                await ctx.tick()

                # Process commit
                for _ in range(15):
                    ctx.set(dut.csr_wr_ack, 1)
                    await ctx.tick()
                    if ctx.get(dut.p2m_bus) == 0x50:
                        break
                ctx.set(dut.csr_wr_ack, 0)
                await ctx.tick()
                await ctx.tick()

            # Should be back in IDLE
            self.assertEqual(ctx.get(dut.fsm_state), 0)

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  10. TestPIPECSRBridge
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPECSRBridge(unittest.TestCase):
    """Tests for PIPECSRBridge: local/DRP register access."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_csr_bridge.vcd"):
            sim.run()

    def test_local_register_write_immediate_ack(self):
        """Writing to a local register (0x002 eb_control) gives immediate wr_ack."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            ctx.set(dut.reg_addr, 0x002)
            ctx.set(dut.reg_wrdata, 0xAB)
            ctx.set(dut.reg_wren, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.wr_ack), 1)
            ctx.set(dut.reg_wren, 0)
            await ctx.tick()

            # Verify register was written
            ctx.set(dut.reg_addr, 0x002)
            ctx.set(dut.reg_rden, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.rd_data), 0xAB)
            self.assertEqual(ctx.get(dut.rd_valid), 1)

        self._run_sim(dut, testbench)

    def test_drp_mapped_register_write(self):
        """Writing to DRP-mapped register (0x400 TX FFE_0) updates shadow and issues DRP write."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()
            await ctx.tick()

            # Write to FFE_0 (PIPE addr 0x400 -> DRP 0x808234)
            # The FSM in IDLE sees reg_wren and drives drp_wren combinationally.
            ctx.set(dut.reg_addr, 0x400)
            ctx.set(dut.reg_wrdata, 0x42)
            ctx.set(dut.reg_wren, 1)
            # Verify combinational DRP output before tick
            self.assertEqual(ctx.get(dut.drp_wren), 1)
            self.assertEqual(ctx.get(dut.drp_addr), 0x808234)
            self.assertEqual(ctx.get(dut.drp_wrdata) & 0xFF, 0x42)

            await ctx.tick()  # Shadow write committed; FSM -> WAIT_DRP_WR
            ctx.set(dut.reg_wren, 0)
            # Now in WAIT_DRP_WR; supply drp_ready
            ctx.set(dut.drp_ready, 1)
            # wr_ack is combinational in WAIT_DRP_WR when drp_ready is asserted
            self.assertEqual(ctx.get(dut.wr_ack), 1)

            await ctx.tick()  # FSM returns to IDLE
            ctx.set(dut.drp_ready, 0)

            # Verify shadow was updated by reading back (combinational read)
            ctx.set(dut.reg_addr, 0x400)
            ctx.set(dut.reg_rden, 1)
            self.assertEqual(ctx.get(dut.rd_data), 0x42)
            self.assertEqual(ctx.get(dut.rd_valid), 1)

        self._run_sim(dut, testbench)

    def test_local_register_read(self):
        """Reading a local register returns its current value."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            # Write common_ctrl_0
            ctx.set(dut.reg_addr, 0x800)
            ctx.set(dut.reg_wrdata, 0x37)
            ctx.set(dut.reg_wren, 1)
            await ctx.tick()
            ctx.set(dut.reg_wren, 0)
            await ctx.tick()

            # Read it back
            ctx.set(dut.reg_addr, 0x800)
            ctx.set(dut.reg_rden, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.rd_data), 0x37)
            self.assertEqual(ctx.get(dut.rd_valid), 1)

        self._run_sim(dut, testbench)

    def test_shadow_register_read(self):
        """Reading a shadow register returns low 8 bits of 32-bit shadow."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            # Write to FFE_0 shadow
            ctx.set(dut.reg_addr, 0x400)
            ctx.set(dut.reg_wrdata, 0xBE)
            ctx.set(dut.reg_wren, 1)
            await ctx.tick()
            ctx.set(dut.reg_wren, 0)
            await ctx.tick()
            ctx.set(dut.drp_ready, 1)
            await ctx.tick()
            ctx.set(dut.drp_ready, 0)
            await ctx.tick()

            # Read shadow back
            ctx.set(dut.reg_addr, 0x400)
            ctx.set(dut.reg_rden, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.rd_data), 0xBE)
            self.assertEqual(ctx.get(dut.rd_valid), 1)

        self._run_sim(dut, testbench)

    def test_unknown_address_write_acks(self):
        """Writing to an unknown address gives immediate ack (silently discarded)."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            ctx.set(dut.reg_addr, 0xFFF)  # Unknown
            ctx.set(dut.reg_wrdata, 0x00)
            ctx.set(dut.reg_wren, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.wr_ack), 1)

        self._run_sim(dut, testbench)

    def test_unknown_address_reads_zero(self):
        """Reading an unknown address returns 0."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            ctx.set(dut.reg_addr, 0xFFF)  # Unknown
            ctx.set(dut.reg_rden, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.rd_data), 0)
            self.assertEqual(ctx.get(dut.rd_valid), 1)

        self._run_sim(dut, testbench)

    def test_mac_transmit_lfps_flag(self):
        """mac_transmit_lfps is bit 0 of common_ctrl_0 (addr 0x800)."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            # Write 0x01 to common_ctrl_0
            ctx.set(dut.reg_addr, 0x800)
            ctx.set(dut.reg_wrdata, 0x01)
            ctx.set(dut.reg_wren, 1)
            await ctx.tick()
            ctx.set(dut.reg_wren, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_transmit_lfps), 1)

            # Write 0x00
            ctx.set(dut.reg_addr, 0x800)
            ctx.set(dut.reg_wrdata, 0x00)
            ctx.set(dut.reg_wren, 1)
            await ctx.tick()
            ctx.set(dut.reg_wren, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_transmit_lfps), 0)

        self._run_sim(dut, testbench)

    def test_nelb_enable_flag(self):
        """nelb_enable is bit 0 of nelb_control (addr 0x801)."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            ctx.set(dut.reg_addr, 0x801)
            ctx.set(dut.reg_wrdata, 0x01)
            ctx.set(dut.reg_wren, 1)
            await ctx.tick()
            ctx.set(dut.reg_wren, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.nelb_enable), 1)

        self._run_sim(dut, testbench)

    def test_multiple_local_registers(self):
        """Write and read back multiple local registers."""
        dut = PIPECSRBridge()

        async def testbench(ctx):
            ctx.set(dut.reset_n, 1)
            await ctx.tick()

            # Write different values to different registers
            regs = [
                (0x002, 0x11),  # eb_control
                (0x003, 0x22),  # rx_control_0
                (0x004, 0x33),  # rx_control_1
                (0x008, 0x44),  # rx_control_4
            ]
            for addr, data in regs:
                ctx.set(dut.reg_addr, addr)
                ctx.set(dut.reg_wrdata, data)
                ctx.set(dut.reg_wren, 1)
                await ctx.tick()
                ctx.set(dut.reg_wren, 0)
                await ctx.tick()

            # Read them back
            for addr, expected in regs:
                ctx.set(dut.reg_addr, addr)
                ctx.set(dut.reg_rden, 1)
                await ctx.tick()
                self.assertEqual(
                    ctx.get(dut.rd_data),
                    expected,
                    f"Register 0x{addr:03X} readback mismatch",
                )
                ctx.set(dut.reg_rden, 0)
                await ctx.tick()

        self._run_sim(dut, testbench)


# ═══════════════════════════════════════════════════════════════════════════
#  11. TestPIPEMacCLKGen
# ═══════════════════════════════════════════════════════════════════════════


class TestPIPEMacCLKGen(unittest.TestCase):
    """Tests for PIPEMacCLKGen: MacCLK gating with handshake."""

    def _run_sim(self, dut, testbench, *, clk_period=1e-6):
        sim = Simulator(dut)
        sim.add_clock(clk_period)
        sim.add_testbench(testbench)
        with sim.write_vcd("test_macclk.vcd"):
            sim.run()

    def test_request_acknowledge_handshake(self):
        """Request -> ack -> release handshake works correctly."""
        dut = PIPEMacCLKGen()

        async def testbench(ctx):
            ctx.set(dut.mac_clk_reset_n, 1)
            ctx.set(dut.mac_clk_req, 0)
            ctx.set(dut.life_clk, 0)
            await ctx.tick()
            # Initially no ack
            self.assertEqual(ctx.get(dut.mac_clk_ack), 0)

            # Request clock
            ctx.set(dut.mac_clk_req, 1)
            await ctx.tick()
            await ctx.tick()
            # Ack should be asserted
            self.assertEqual(ctx.get(dut.mac_clk_ack), 1)

            # Release request
            ctx.set(dut.mac_clk_req, 0)
            await ctx.tick()
            await ctx.tick()
            # Ack should be deasserted
            self.assertEqual(ctx.get(dut.mac_clk_ack), 0)

        self._run_sim(dut, testbench)

    def test_clock_gating(self):
        """mac_clk follows life_clk when enabled, stays low when disabled."""
        dut = PIPEMacCLKGen()

        async def testbench(ctx):
            ctx.set(dut.mac_clk_reset_n, 1)
            ctx.set(dut.mac_clk_req, 0)

            # Clock off: mac_clk should be 0 regardless of life_clk
            ctx.set(dut.life_clk, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_clk), 0)

            # Enable clock
            ctx.set(dut.mac_clk_req, 1)
            await ctx.tick()
            await ctx.tick()

            # Now mac_clk should follow life_clk
            ctx.set(dut.life_clk, 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_clk), 1)

            ctx.set(dut.life_clk, 0)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_clk), 0)

        self._run_sim(dut, testbench)

    def test_reset_clears_everything(self):
        """Reset deasserts mac_clk_ack and gates clock off."""
        dut = PIPEMacCLKGen()

        async def testbench(ctx):
            ctx.set(dut.mac_clk_reset_n, 1)
            ctx.set(dut.mac_clk_req, 1)
            ctx.set(dut.life_clk, 1)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_clk_ack), 1)
            self.assertEqual(ctx.get(dut.mac_clk), 1)

            # Assert reset
            ctx.set(dut.mac_clk_reset_n, 0)
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_clk_ack), 0)
            self.assertEqual(ctx.get(dut.mac_clk), 0)

        self._run_sim(dut, testbench)

    def test_rapid_request_toggling(self):
        """Rapid toggling of mac_clk_req doesn't cause glitches in ack."""
        dut = PIPEMacCLKGen()

        async def testbench(ctx):
            ctx.set(dut.mac_clk_reset_n, 1)
            ctx.set(dut.life_clk, 1)

            # Toggle rapidly
            for _ in range(5):
                ctx.set(dut.mac_clk_req, 1)
                await ctx.tick()
                ctx.set(dut.mac_clk_req, 0)
                await ctx.tick()

            # After settling with req=0, ack should be 0
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_clk_ack), 0)

            # Now hold req=1 and let it settle
            ctx.set(dut.mac_clk_req, 1)
            await ctx.tick()
            await ctx.tick()
            await ctx.tick()
            self.assertEqual(ctx.get(dut.mac_clk_ack), 1)

        self._run_sim(dut, testbench)

    def test_mac_clk_rate_accepted(self):
        """mac_clk_rate input is accepted without error (currently ignored)."""
        dut = PIPEMacCLKGen()

        async def testbench(ctx):
            ctx.set(dut.mac_clk_reset_n, 1)
            ctx.set(dut.mac_clk_rate, 0x1F)  # All bits set
            ctx.set(dut.mac_clk_req, 1)
            ctx.set(dut.life_clk, 1)
            await ctx.tick()
            await ctx.tick()
            # Should still work -- rate is ignored
            self.assertEqual(ctx.get(dut.mac_clk_ack), 1)

        self._run_sim(dut, testbench)


if __name__ == "__main__":
    unittest.main()
