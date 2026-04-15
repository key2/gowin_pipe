"""PIPE SerDes bring-up test — LTSSM through Polling.RxEQ (pre-8b10b).

Instantiates PIPESerDes and walks the USB 3.0 link training sequence
through the PIPE interface, stopping before ordered-set decoding.

Architecture
============
::

    ┌────────────────────────────────────────────────────────────┐
    │ PIPEBringUp                                                │
    │                                                            │
    │  ┌──────────────┐                                          │
    │  │  LTSSM FSM   │ sync (50 MHz) — acts as "MAC"           │
    │  │  INIT_WAIT → DETECT → LFPS loop → RXEQ → CDR_WAIT     │
    │  └──────┬───────┘                                          │
    │         │ PIPE signals (CDC → upar)                        │
    │  ┌──────▼───────┐                                          │
    │  │  PIPESerDes  │ upar (62.5 MHz) — the PHY under test    │
    │  └──────────────┘                                          │
    │                                                            │
    │  UART0 (/dev/ttyUSB5) — Command: T/S/P/D/E/F              │
    │  UART1 (/dev/ttyUSB4) — Debug trace: state tag stream     │
    │  UART2 (/dev/ttyUSB3) — RX data snapshot (5 bytes)        │
    └────────────────────────────────────────────────────────────┘

UART0 commands (115200 8N1):
  'T'     → 0x55 (ping)
  'S'     → 8 status bytes
  'P' N   → PowerDown = N
  'D'     → receiver detection (0x03=detected, 0x00=not, 0xFF=timeout)
  'E' N   → set TxElecIdle (0=off, 1=on)
  'F'     → run full LTSSM sequence, returns result byte

UART1 trace tags (115200, TX-only, streamed during 'F' command):
  'I' = INIT_WAIT        'D' = DETECT           'P' = P0_ENTER
  'B' = LFPS_BURST       'G' = LFPS_GAP         'H' = LFPS_DONE
  'Q' = RXEQ_ENTER       'C' = CDR_WAIT         'U' = MONITOR
  'X' = FAIL             'R' = result byte

UART2 RX snapshot (115200, TX-only, on CDR lock or on-demand):
  5 bytes: rx_data[7:0], rx_data[15:8], rx_data[23:16], rx_data[31:24], flags

LEDs:
  LED0: Heartbeat         LED1: PLL lock         LED2: CDR lock
  LED3: PhyStatus pulse   LED4: LTSSM state[0]   LED5: io_hash

Build:
    cd example && python top.py
"""

import sys
from pathlib import Path

from amaranth import *
from amaranth.lib.cdc import FFSynchronizer

# ── Local imports ──────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))
from uart import AsyncSerialRX, AsyncSerialTX
from gw5ast_dvk import GW5ASTDVKPlatform

# ── gowin_serdes + pipe_serdes ────────────────────────────────
_proj_root = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _proj_root)
sys.path.insert(0, str(Path(_proj_root) / "gowin-serdes"))

from gowin_serdes import GowinDevice
from pipe_serdes import (
    PIPESerDes,
    PIPELaneConfig,
    PIPEProtocol,
    PIPEWidth,
    USBRate,
)

# ── Constants ─────────────────────────────────────────────────
BOARD_FREQ = 50_000_000
BAUD_RATE = 115_200
DIVISOR = BOARD_FREQ // BAUD_RATE

# Timing at 50 MHz sync clock
TICKS_1US = 50  # 1 µs
TICKS_2US = 100  # 2 µs (1 µs DRP setup + ~1 µs actual burst)
TICKS_9US = 450  # 9 µs
TICKS_1MS = 50_000  # 1 ms
TICKS_12MS = 600_000  # 12 ms
TICKS_80MS = 4_000_000  # 80 ms
TICKS_360MS = 18_000_000  # 360 ms

PIPE_CFG = PIPELaneConfig(
    protocol=PIPEProtocol.USB3,
    device=GowinDevice.GW5AST_138,
    quad=0,
    lane=0,
    supported_rates=[USBRate.GEN1],
    default_width=PIPEWidth.W40,
)


class PIPEBringUp(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        # ══════════════════════════════════════════════════════
        # Platform resources
        # ══════════════════════════════════════════════════════
        leds = [platform.request("led", i) for i in range(6)]
        uart0_pins = platform.request("uart", 0)  # Command
        uart1_pins = platform.request("uart", 1)  # Debug trace
        uart2_pins = platform.request("uart", 2)  # RX data snap

        # ══════════════════════════════════════════════════════
        # PIPE SerDes (self-contained: GowinSerDes + adapter)
        # ══════════════════════════════════════════════════════
        phy = PIPESerDes(PIPE_CFG)
        m.submodules.phy = DomainRenamer("upar")(phy)
        m.d.comb += phy.por_n.eq(1)

        # ══════════════════════════════════════════════════════
        # PIPE command registers (upar domain, CDC'd from sync)
        # ══════════════════════════════════════════════════════
        pipe_power_down = Signal(4, name="cmd_power_down")
        pipe_tx_detect_rx = Signal(1, name="cmd_tx_detect_rx")
        pipe_tx_elec_idle = Signal(1, init=1, name="cmd_tx_elec_idle")

        # POR counter: ~8 ms at 62.5 MHz
        por_cnt = Signal(20)
        por_done = Signal()
        m.d.upar += por_cnt.eq(por_cnt + ~por_done)
        m.d.comb += por_done.eq(por_cnt[-1])

        m.d.comb += [
            phy.pipe.reset_n.eq(por_done),
            phy.pipe.phy_mode.eq(1),
            phy.pipe.rate.eq(0),
            phy.pipe.width.eq(PIPEWidth.W40.value),
            phy.pipe.rx_width.eq(PIPEWidth.W40.value),
            phy.pipe.pclk.eq(0),
            phy.pipe.power_down.eq(pipe_power_down),
            phy.pipe.tx_elec_idle.eq(pipe_tx_elec_idle),
            phy.pipe.tx_detect_rx_loopback.eq(pipe_tx_detect_rx),
            phy.pipe.tx_data.eq(0),
            phy.pipe.tx_data_valid.eq(0),
            phy.pipe.m2p_msg_bus.eq(0),
            phy.pipe.mac_clk_reset_n.eq(por_done),
            phy.pipe.mac_clk_rate.eq(0),
            phy.pipe.mac_clk_req.eq(0),
            phy.pipe.rx_polarity.eq(0),
            phy.pipe.rx_termination.eq(0),
            phy.pipe.rx_standby.eq(0),
            phy.pipe.pclk_change_ack.eq(0),
        ]

        # ══════════════════════════════════════════════════════
        # PhyStatus stretch (1 PCLK pulse → visible on LED)
        # ══════════════════════════════════════════════════════
        phy_status_stretch = Signal(24)
        with m.If(phy.pipe.phy_status):
            m.d.upar += phy_status_stretch.eq(~0)
        with m.Elif(phy_status_stretch != 0):
            m.d.upar += phy_status_stretch.eq(phy_status_stretch - 1)

        # ══════════════════════════════════════════════════════
        # UARTs — all on 50 MHz sync domain
        # ══════════════════════════════════════════════════════

        # UART0: Command interface (RX + TX)
        uart0_rx = AsyncSerialRX(divisor=DIVISOR)
        uart0_tx = AsyncSerialTX(divisor=DIVISOR)
        m.submodules.uart0_rx = uart0_rx
        m.submodules.uart0_tx = uart0_tx
        rx0_sync = Signal(init=1)
        m.submodules += FFSynchronizer(
            uart0_pins.rx.i, rx0_sync, o_domain="sync", init=1
        )
        m.d.comb += [uart0_rx.i.eq(rx0_sync), uart0_pins.tx.o.eq(uart0_tx.o)]

        # UART1: Debug trace (TX-only)
        dbg_tx = AsyncSerialTX(divisor=DIVISOR)
        m.submodules.dbg_tx = dbg_tx
        m.d.comb += uart1_pins.tx.o.eq(dbg_tx.o)

        # UART2: RX data snapshot (TX-only)
        snap_tx = AsyncSerialTX(divisor=DIVISOR)
        m.submodules.snap_tx = snap_tx
        m.d.comb += uart2_pins.tx.o.eq(snap_tx.o)

        # ══════════════════════════════════════════════════════
        # CDC: sync → upar (command registers)
        # ══════════════════════════════════════════════════════
        cmd_power_sync = Signal(4)
        cmd_detect_sync = Signal(1)
        cmd_eidle_sync = Signal(1, init=1)
        m.submodules += FFSynchronizer(cmd_power_sync, pipe_power_down, o_domain="upar")
        m.submodules += FFSynchronizer(
            cmd_detect_sync, pipe_tx_detect_rx, o_domain="upar"
        )
        m.submodules += FFSynchronizer(
            cmd_eidle_sync, pipe_tx_elec_idle, o_domain="upar"
        )

        # ══════════════════════════════════════════════════════
        # CDC: upar → sync (observation signals)
        # ══════════════════════════════════════════════════════
        obs_pll = Signal()
        obs_cdr = Signal()
        obs_physt = Signal()
        obs_rxvld = Signal()
        obs_eidle = Signal()
        obs_power = Signal(4)
        obs_rate = Signal(4)
        obs_rxdet = Signal(4)
        obs_lfps = Signal(4)
        obs_hash = Signal(8)
        obs_rx_status = Signal(3)
        obs_init_done = Signal()
        obs_signal_det = Signal()
        obs_rx_data_lo = Signal(32)  # Lower 32 bits of rx_data snapshot

        # Latch rx_status on phy_status pulse
        latched_rx_status = Signal(3)
        with m.If(phy.pipe.phy_status):
            m.d.upar += latched_rx_status.eq(phy.pipe.rx_status)

        # Snapshot rx_data in upar domain continuously (CDC'd to sync)
        rx_data_snap = Signal(32)
        m.d.upar += rx_data_snap.eq(phy.pipe.rx_data[:32])

        m.submodules += [
            FFSynchronizer(phy.debug.pll_lock, obs_pll, o_domain="sync"),
            FFSynchronizer(phy.debug.cdr_lock, obs_cdr, o_domain="sync"),
            FFSynchronizer(phy.pipe.phy_status, obs_physt, o_domain="sync"),
            FFSynchronizer(phy.pipe.rx_valid, obs_rxvld, o_domain="sync"),
            FFSynchronizer(phy.pipe.rx_elec_idle, obs_eidle, o_domain="sync"),
            FFSynchronizer(phy.debug.power_state, obs_power, o_domain="sync"),
            FFSynchronizer(phy.debug.rate_fsm_state, obs_rate, o_domain="sync"),
            FFSynchronizer(phy.debug.rxdet_fsm_state, obs_rxdet, o_domain="sync"),
            FFSynchronizer(phy.debug.lfps_fsm_state, obs_lfps, o_domain="sync"),
            FFSynchronizer(latched_rx_status, obs_rx_status, o_domain="sync"),
            FFSynchronizer(phy.debug.init_done, obs_init_done, o_domain="sync"),
            FFSynchronizer(rx_data_snap, obs_rx_data_lo, o_domain="sync"),
        ]

        # signal_detect is on the lane status port inside the adapter.
        # We need to observe it via the debug hash for now, or add it.
        # The adapter doesn't expose signal_detect on debug, but
        # RxElecIdle transitions serve as the LFPS RX indicator.

        # io_hash
        _hash = Signal(8)
        m.d.comb += _hash.eq(
            phy.pipe.rx_data[0:8].xor()
            ^ phy.debug.power_state[0:4].xor()
            ^ phy.debug.drp_mux_locked
        )
        m.submodules += FFSynchronizer(_hash, obs_hash, o_domain="sync")

        # ══════════════════════════════════════════════════════
        # LEDs
        # ══════════════════════════════════════════════════════
        heartbeat = Signal(26)
        m.d.upar += heartbeat.eq(heartbeat + 1)

        ltssm_state_reg = Signal(4)  # for LED and debug

        m.d.comb += [
            leds[0].o.eq(heartbeat[-1]),
            leds[1].o.eq(phy.debug.pll_lock),
            leds[2].o.eq(phy.debug.cdr_lock),
            leds[3].o.eq(phy_status_stretch != 0),
            leds[4].o.eq(ltssm_state_reg[0]),
            leds[5].o.eq(
                phy.pipe.rx_data[0:8].xor()
                ^ phy.debug.power_state[0:4].xor()
                ^ phy.debug.drp_mux_locked
                ^ phy.debug.serdes_arb_state.xor()
            ),
        ]

        # ══════════════════════════════════════════════════════
        # Debug trace helper (UART1)
        #
        # dbg_tag_pending + dbg_tag_byte: when the LTSSM FSM
        # sets dbg_tag_pending=1, the debug TX FSM sends the
        # byte then clears the flag.
        # ══════════════════════════════════════════════════════
        dbg_tag_pending = Signal()
        dbg_tag_byte = Signal(8)

        with m.FSM(name="dbg_fsm"):
            with m.State("DBG_IDLE"):
                with m.If(dbg_tag_pending):
                    m.d.comb += [
                        dbg_tx.data.eq(dbg_tag_byte),
                        dbg_tx.ack.eq(dbg_tx.rdy),
                    ]
                    with m.If(dbg_tx.rdy):
                        m.d.sync += dbg_tag_pending.eq(0)

        # ══════════════════════════════════════════════════════
        # RX data snapshot helper (UART2)
        #
        # snap_req: when set, sends 5 bytes on UART2:
        #   rx_data[7:0], [15:8], [23:16], [31:24], flags
        # where flags = {0, 0, 0, signal_det(obs_eidle inverted),
        #                cdr, pll, lfps_state[1:0]}
        # ══════════════════════════════════════════════════════
        snap_req = Signal()
        snap_idx = Signal(3)
        snap_data = Signal(32)
        snap_flags = Signal(8)

        with m.FSM(name="snap_fsm"):
            with m.State("SNAP_IDLE"):
                with m.If(snap_req):
                    m.d.sync += [
                        snap_data.eq(obs_rx_data_lo),
                        snap_flags.eq(
                            Cat(
                                obs_lfps[:2],  # [1:0]
                                obs_pll,  # [2]
                                obs_cdr,  # [3]
                                ~obs_eidle,  # [4] signal activity (inverted EI)
                                obs_rxvld,  # [5]
                                obs_init_done,  # [6]
                                obs_physt,  # [7]
                            )
                        ),
                        snap_idx.eq(0),
                    ]
                    m.next = "SNAP_TX"

            with m.State("SNAP_TX"):
                with m.If(snap_idx < 4):
                    m.d.comb += snap_tx.data.eq(snap_data.word_select(snap_idx, 8))
                with m.Else():
                    m.d.comb += snap_tx.data.eq(snap_flags)

                with m.If(snap_tx.rdy):
                    m.d.comb += snap_tx.ack.eq(1)
                    with m.If(snap_idx == 4):
                        m.d.sync += snap_req.eq(0)
                        m.next = "SNAP_IDLE"
                    with m.Else():
                        m.d.sync += snap_idx.eq(snap_idx + 1)

        # ══════════════════════════════════════════════════════
        # UART0 Command FSM + LTSSM FSM
        # ══════════════════════════════════════════════════════
        status_buf = Signal(8 * 8)
        tx_cnt = Signal(3)
        timer = Signal(25)  # Up to ~670 ms at 50 MHz

        # LFPS counters
        lfps_tx_cnt = Signal(6)  # TX bursts sent (need ≥16)
        lfps_rx_cnt = Signal(6)  # RX bursts received (need ≥2)
        lfps_tx_after_rx = Signal(6)  # TX after first RX (need ≥4)
        lfps_rx_prev = Signal()  # Previous obs_eidle for edge detect

        # RxElecIdle edge detector for LFPS RX counting
        # A falling edge on RxElecIdle = start of an LFPS burst from host
        eidle_prev = Signal()
        eidle_fell = Signal()
        m.d.sync += eidle_prev.eq(obs_eidle)
        m.d.comb += eidle_fell.eq(eidle_prev & ~obs_eidle)

        with m.FSM(name="main"):
            # ──────────────────────────────────────────────
            # IDLE: wait for UART0 command
            # ──────────────────────────────────────────────
            with m.State("IDLE"):
                m.d.comb += uart0_rx.ack.eq(1)
                with m.If(uart0_rx.rdy):
                    with m.Switch(uart0_rx.data):
                        with m.Case(ord("T")):
                            m.d.sync += [status_buf[:8].eq(0x55), tx_cnt.eq(0)]
                            m.next = "TX_BYTE"

                        with m.Case(ord("S")):
                            m.d.sync += [
                                status_buf[56:64].eq(
                                    Cat(
                                        obs_power,
                                        obs_eidle,
                                        obs_rxvld,
                                        obs_physt,
                                        obs_cdr,
                                    )
                                ),
                                status_buf[48:56].eq(Cat(obs_rxdet, obs_rate)),
                                status_buf[40:48].eq(obs_hash),
                                status_buf[32:40].eq(
                                    Cat(
                                        obs_pll,
                                        obs_init_done,
                                        obs_lfps[:2],
                                        Const(0, 4),
                                    )
                                ),
                                status_buf[24:32].eq(obs_rx_data_lo[0:8]),
                                status_buf[16:24].eq(obs_rx_data_lo[8:16]),
                                status_buf[8:16].eq(obs_rx_data_lo[16:24]),
                                status_buf[0:8].eq(obs_rx_data_lo[24:32]),
                                tx_cnt.eq(7),
                            ]
                            m.next = "TX_BYTE"

                        with m.Case(ord("P")):
                            m.next = "RX_POWER"

                        with m.Case(ord("D")):
                            m.d.sync += [
                                cmd_eidle_sync.eq(1),
                                cmd_detect_sync.eq(1),
                                timer.eq(0),
                            ]
                            m.next = "DET_TRIG"

                        with m.Case(ord("E")):
                            m.next = "RX_EIDLE"

                        with m.Case(ord("F")):
                            # Start full LTSSM sequence
                            m.d.sync += [
                                ltssm_state_reg.eq(0),
                                timer.eq(0),
                                lfps_tx_cnt.eq(0),
                                lfps_rx_cnt.eq(0),
                                lfps_tx_after_rx.eq(0),
                            ]
                            m.next = "LTSSM_INIT_WAIT"

            # ──────────────────────────────────────────────
            # RX_POWER / RX_EIDLE: receive parameter byte
            # ──────────────────────────────────────────────
            with m.State("RX_POWER"):
                m.d.comb += uart0_rx.ack.eq(1)
                with m.If(uart0_rx.rdy):
                    m.d.sync += [
                        cmd_power_sync.eq(uart0_rx.data[:4]),
                        status_buf[:8].eq(0x00),
                        tx_cnt.eq(0),
                    ]
                    m.next = "TX_BYTE"

            with m.State("RX_EIDLE"):
                m.d.comb += uart0_rx.ack.eq(1)
                with m.If(uart0_rx.rdy):
                    m.d.sync += [
                        cmd_eidle_sync.eq(uart0_rx.data[0]),
                        status_buf[:8].eq(0x00),
                        tx_cnt.eq(0),
                    ]
                    m.next = "TX_BYTE"

            # ──────────────────────────────────────────────
            # DETECT sequence (standalone 'D' command)
            # ──────────────────────────────────────────────
            with m.State("DET_TRIG"):
                m.d.sync += timer.eq(timer + 1)
                with m.If(timer[10]):
                    m.next = "DET_WAIT"

            with m.State("DET_WAIT"):
                m.d.sync += timer.eq(timer + 1)
                with m.If((obs_rxdet == 5) | (obs_rxdet == 6)):
                    m.d.sync += cmd_detect_sync.eq(0)
                    m.next = "DET_READ"
                with m.Elif(timer[-1]):
                    m.d.sync += [
                        cmd_detect_sync.eq(0),
                        timer.eq(0),
                        status_buf[:8].eq(0xFF),
                        tx_cnt.eq(0),
                    ]
                    m.next = "TX_BYTE"

            with m.State("DET_READ"):
                m.d.sync += [
                    timer.eq(0),
                    status_buf[:8].eq(obs_rx_status),
                    tx_cnt.eq(0),
                ]
                m.next = "TX_BYTE"

            # ──────────────────────────────────────────────
            # TX_BYTE: send status_buf bytes MSB-first
            # ──────────────────────────────────────────────
            with m.State("TX_BYTE"):
                m.d.comb += uart0_tx.data.eq(status_buf.word_select(tx_cnt, 8))
                with m.If(uart0_tx.rdy):
                    m.d.comb += uart0_tx.ack.eq(1)
                    with m.If(tx_cnt == 0):
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += tx_cnt.eq(tx_cnt - 1)

            # ══════════════════════════════════════════════
            # LTSSM FSM — full USB 3.0 link training
            # Steps through init, detect, LFPS, RxEQ entry
            # ══════════════════════════════════════════════

            # ── Step 1: Wait for PIPEInitFSM to complete ──
            # Init runs in upar domain and completes in <1 ms after POR.
            # By the time the user sends 'F', it's long done.
            # We wait 10 ms to guarantee POR + init + P-state settle.
            with m.State("LTSSM_INIT_WAIT"):
                m.d.sync += [ltssm_state_reg.eq(0), timer.eq(timer + 1)]
                with m.If(timer == 0):
                    m.d.sync += [dbg_tag_byte.eq(ord("I")), dbg_tag_pending.eq(1)]
                # 10 ms wait (500000 ticks at 50 MHz)
                with m.If(timer == 500_000):
                    m.d.sync += timer.eq(0)
                    m.next = "LTSSM_EIDLE"

            # ── Step 2: Assert electrical idle, stay in P0 ──
            # The power FSM boots to P0 after the settle counter.
            # We stay in P0 and assert EI for detect + LFPS.
            # (Transitioning to P2 and back has DRP issues with
            # the EIDLE_EXIT write when quad_pd is non-zero.)
            with m.State("LTSSM_EIDLE"):
                m.d.sync += [
                    ltssm_state_reg.eq(1),
                    cmd_eidle_sync.eq(1),
                    cmd_power_sync.eq(0),  # Stay P0
                    timer.eq(0),
                ]
                m.d.sync += [dbg_tag_byte.eq(ord("E")), dbg_tag_pending.eq(1)]
                m.next = "LTSSM_EIDLE_WAIT"

            with m.State("LTSSM_EIDLE_WAIT"):
                m.d.sync += timer.eq(timer + 1)
                with m.If(timer == TICKS_1MS):
                    m.d.sync += timer.eq(0)
                    m.next = "LTSSM_DETECT"

            # ── Step 3: Receiver detection ──
            with m.State("LTSSM_DETECT"):
                m.d.sync += [
                    ltssm_state_reg.eq(2),
                    cmd_detect_sync.eq(1),
                    timer.eq(0),
                ]
                m.d.sync += [dbg_tag_byte.eq(ord("D")), dbg_tag_pending.eq(1)]
                m.next = "LTSSM_DETECT_TRIG"

            with m.State("LTSSM_DETECT_TRIG"):
                m.d.sync += timer.eq(timer + 1)
                with m.If(timer[10]):  # ~20 µs for CDC
                    m.next = "LTSSM_DETECT_WAIT"

            with m.State("LTSSM_DETECT_WAIT"):
                m.d.sync += timer.eq(timer + 1)
                with m.If((obs_rxdet == 5) | (obs_rxdet == 6)):
                    m.d.sync += cmd_detect_sync.eq(0)
                    m.next = "LTSSM_DETECT_CHECK"
                with m.Elif(timer[-1]):
                    m.d.sync += [
                        cmd_detect_sync.eq(0),
                        dbg_tag_byte.eq(ord("X")),
                        dbg_tag_pending.eq(1),
                    ]
                    m.next = "LTSSM_FAIL"

            with m.State("LTSSM_DETECT_CHECK"):
                m.d.sync += timer.eq(0)
                with m.If(obs_rx_status == 0b011):
                    # Receiver detected — already in P0, go straight to LFPS
                    m.d.sync += [
                        dbg_tag_byte.eq(ord("P")),
                        dbg_tag_pending.eq(1),
                        lfps_tx_cnt.eq(0),
                        lfps_rx_cnt.eq(0),
                        lfps_tx_after_rx.eq(0),
                    ]
                    m.next = "LTSSM_LFPS_BURST"
                with m.Else():
                    m.d.sync += [dbg_tag_byte.eq(ord("X")), dbg_tag_pending.eq(1)]
                    m.next = "LTSSM_FAIL"

            # ── Step 4: Enter P0 ──
            with m.State("LTSSM_P0_ENTER"):
                m.d.sync += [
                    ltssm_state_reg.eq(3),
                    cmd_power_sync.eq(0),  # P0
                    cmd_eidle_sync.eq(1),  # Keep idle for LFPS trigger
                    timer.eq(0),
                ]
                m.d.sync += [dbg_tag_byte.eq(ord("P")), dbg_tag_pending.eq(1)]
                m.next = "LTSSM_P0_SETTLE"

            with m.State("LTSSM_P0_SETTLE"):
                m.d.sync += timer.eq(timer + 1)
                # Wait until power FSM reports P0, or timeout at 12 ms
                with m.If(obs_power == 0):
                    m.d.sync += [
                        timer.eq(0),
                        lfps_tx_cnt.eq(0),
                        lfps_rx_cnt.eq(0),
                        lfps_tx_after_rx.eq(0),
                    ]
                    m.next = "LTSSM_LFPS_BURST"
                with m.Elif(timer == TICKS_12MS):
                    # Power FSM didn't reach P0 — report and fail
                    m.d.sync += [
                        dbg_tag_byte.eq(ord("p")),  # lowercase = P0 timeout
                        dbg_tag_pending.eq(1),
                    ]
                    m.next = "LTSSM_FAIL"

            # ── Step 5: Polling.LFPS burst loop ──
            # TX burst: assert TxDetectRx (with TxElecIdle=1 in P0)
            # to trigger PIPELFPSController for ~1 µs.
            # Gap: deassert for ~9 µs. Count RX from RxElecIdle edges.
            #
            # Exit: TX≥16 AND RX≥2 AND TX_after_RX≥4 (USB 3.1 §7.5.4.3.2)
            # Timeout: 360 ms.

            with m.State("LTSSM_LFPS_BURST"):
                m.d.sync += [
                    ltssm_state_reg.eq(4),
                    cmd_detect_sync.eq(1),  # trigger LFPS in P0
                    timer.eq(0),
                ]
                m.d.sync += [dbg_tag_byte.eq(ord("B")), dbg_tag_pending.eq(1)]
                m.next = "LTSSM_LFPS_BURST_HOLD"

            with m.State("LTSSM_LFPS_BURST_HOLD"):
                m.d.sync += timer.eq(timer + 1)
                # Count RX edges during burst too
                with m.If(eidle_fell):
                    m.d.sync += lfps_rx_cnt.eq(lfps_rx_cnt + 1)
                # Hold burst for ~2 µs: ~0.2 µs DRP setup (FFE save +
                # eidle toggle) + ~1 µs actual RF burst on wire.
                # tBurst spec: 0.6-1.4 µs (USB 3.1 Table 6-29).
                with m.If(timer == TICKS_2US):
                    m.d.sync += [
                        cmd_detect_sync.eq(0),  # end burst trigger
                        timer.eq(0),
                        lfps_tx_cnt.eq(lfps_tx_cnt + 1),
                    ]
                    # If we've received at least 1 RX, count TX_after_RX
                    with m.If(lfps_rx_cnt > 0):
                        m.d.sync += lfps_tx_after_rx.eq(lfps_tx_after_rx + 1)
                    m.next = "LTSSM_LFPS_GAP"

            with m.State("LTSSM_LFPS_GAP"):
                m.d.sync += [
                    ltssm_state_reg.eq(5),
                    timer.eq(timer + 1),
                ]
                # Count RX edges during gap
                with m.If(eidle_fell):
                    m.d.sync += lfps_rx_cnt.eq(lfps_rx_cnt + 1)

                # Check exit conditions every gap
                with m.If(timer == TICKS_9US):
                    # USB 3.1 §7.5.4.3.2 exit conditions:
                    with m.If(
                        (lfps_tx_cnt >= 16)
                        & (lfps_rx_cnt >= 2)
                        & (lfps_tx_after_rx >= 4)
                    ):
                        m.d.sync += [dbg_tag_byte.eq(ord("H")), dbg_tag_pending.eq(1)]
                        m.next = "LTSSM_LFPS_DONE"
                    with m.Else():
                        m.next = "LTSSM_LFPS_BURST"

                # 360 ms global timeout (check accumulated timer)
                # We approximate: if tx_cnt >= 36 (36*10µs=360µs... no)
                # Better: use a separate timeout counter
                # tx_cnt of 40 × 10µs = 400µs, way under 360ms.
                # We'll use a wider counter: lfps_tx_cnt >= 36000/10=3600
                # is impractical. Instead, count total gap+burst time.
                # At 10µs per burst cycle, 360ms = 36000 cycles.
                # With tx_cnt as 6-bit (max 63), we timeout at 63 bursts.
                with m.If(lfps_tx_cnt >= 60):
                    m.d.sync += [dbg_tag_byte.eq(ord("X")), dbg_tag_pending.eq(1)]
                    m.next = "LTSSM_FAIL"

            with m.State("LTSSM_LFPS_DONE"):
                m.d.sync += [
                    ltssm_state_reg.eq(6),
                    timer.eq(0),
                ]
                # Trigger a snapshot of the current state
                m.d.sync += snap_req.eq(1)
                m.next = "LTSSM_RXEQ_ENTER"

            # ── Step 6: Enter Polling.RxEQ ──
            # De-idle TX, de-assert TxDetectRx → P0 active.
            # The host starts sending TSEQ ordered sets.
            # CDR should lock on the TSEQ bitstream.
            with m.State("LTSSM_RXEQ_ENTER"):
                m.d.sync += [
                    ltssm_state_reg.eq(7),
                    cmd_eidle_sync.eq(0),  # TX active (no more idle)
                    cmd_detect_sync.eq(0),  # No detection/LFPS
                    timer.eq(0),
                ]
                m.d.sync += [dbg_tag_byte.eq(ord("Q")), dbg_tag_pending.eq(1)]
                m.next = "LTSSM_CDR_WAIT"

            # ── Step 7: Wait for CDR lock ──
            # The host sends TSEQ; CDR should lock within ~12 ms.
            with m.State("LTSSM_CDR_WAIT"):
                m.d.sync += [ltssm_state_reg.eq(8), timer.eq(timer + 1)]
                with m.If(timer == 0):
                    m.d.sync += [dbg_tag_byte.eq(ord("C")), dbg_tag_pending.eq(1)]
                with m.If(obs_cdr):
                    m.d.sync += timer.eq(0)
                    m.next = "LTSSM_MONITOR"
                with m.Elif(timer == TICKS_80MS):
                    # No CDR lock — report but continue to monitor
                    m.d.sync += [
                        dbg_tag_byte.eq(ord("c")),  # lowercase = CDR timeout
                        dbg_tag_pending.eq(1),
                        timer.eq(0),
                    ]
                    m.next = "LTSSM_MONITOR"

            # ── Step 8: Monitor — CDR locked or timed out ──
            # Snap RX data, report success.
            with m.State("LTSSM_MONITOR"):
                m.d.sync += [ltssm_state_reg.eq(9), timer.eq(timer + 1)]
                with m.If(timer == 0):
                    m.d.sync += [
                        dbg_tag_byte.eq(ord("U")),
                        dbg_tag_pending.eq(1),
                        snap_req.eq(1),  # Take RX data snapshot
                    ]
                # Wait a short time for snapshot to complete, then report
                with m.If(timer == TICKS_1MS):
                    # Build result: 1 byte with LFPS counts + flags
                    m.d.sync += [
                        status_buf[:8].eq(
                            Cat(
                                obs_cdr,  # [0] CDR locked
                                obs_pll,  # [1] PLL locked
                                obs_init_done,  # [2] init done
                                Const(0, 1),  # [3] reserved
                                lfps_rx_cnt[:4],  # [7:4] RX count (lower 4 bits)
                            )
                        ),
                        tx_cnt.eq(0),
                    ]
                    # Send 'R' tag on debug UART
                    m.d.sync += [dbg_tag_byte.eq(ord("R")), dbg_tag_pending.eq(1)]
                    m.next = "TX_BYTE"

            # ── FAIL state ──
            with m.State("LTSSM_FAIL"):
                m.d.sync += [
                    ltssm_state_reg.eq(15),
                    # Clean up: go back to idle state on PIPE signals
                    cmd_eidle_sync.eq(1),
                    cmd_detect_sync.eq(0),
                    cmd_power_sync.eq(2),  # Back to P2
                    # Send failure result
                    status_buf[:8].eq(0xFE),  # 0xFE = LTSSM failed
                    tx_cnt.eq(0),
                ]
                m.next = "TX_BYTE"

        return m


# ══════════════════════════════════════════════════════════════
# Build entry-point
# ══════════════════════════════════════════════════════════════


def build(do_program=False):
    platform = GW5ASTDVKPlatform()

    phy = PIPESerDes(PIPE_CFG)
    csr_path = Path(__file__).parent / "serdes.csr"
    try:
        phy.generate_csr(
            output_path=str(csr_path),
            toml_path=str(csr_path.with_suffix(".toml")),
        )
        print(f"Generated {csr_path} (and .toml)")
    except FileNotFoundError as e:
        if not csr_path.exists():
            csr_path = Path(__file__).parent / "build" / "serdes.csr"
        if not csr_path.exists():
            raise FileNotFoundError(
                f"serdes.csr not found and Gowin tool unavailable:\n  {e}\n"
                "Place a pre-generated serdes.csr next to top.py."
            ) from e
        print(f"Gowin tool not found, using existing {csr_path}")

    platform.add_file("serdes.csr", csr_path.read_bytes())
    sdc = "# PIPE SerDes bring-up — no custom clock constraints needed.\n"

    platform.build(
        PIPEBringUp(),
        name="pipe_serdes_test",
        build_dir="build",
        do_program=do_program,
        add_constraints=sdc,
    )


if __name__ == "__main__":
    build(do_program="program" in sys.argv)
