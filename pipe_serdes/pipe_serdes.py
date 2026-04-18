"""Top-level self-contained PIPE SerDes PHY.

Assembles the PIPE adapter layer, the GowinSerDes hardware hard macro,
and all internal wiring into a single user-facing component.  The user
only sees the PIPE interface, debug outputs, and a power-on-reset input.

Architecture
------------
::

    ┌─────────────────────────────────────────────────────┐
    │  PIPESerDes  (this module)                          │
    │                                                     │
    │   por_n ──────────────────────────┐                 │
    │                                   ▼                 │
    │   ┌────────────────────────────────────────┐        │
    │   │  GowinSerDes  (GTR12 hard macro)       │        │
    │   │   ├── QUAD instance(s)                 │        │
    │   │   ├── UPAR arbiter                     │        │
    │   │   └── Life-clock → "upar" domain       │        │
    │   └──────┬──────────────────┬──────────────┘        │
    │     lane │                  │ DRP                    │
    │   ┌──────┴──────────────────┴──────────────┐        │
    │   │  PIPESerDesAdapter                     │        │
    │   │   ┌──────────┐  ┌───────────────┐      │        │
    │   │   │ TX Path  │  │ Power FSM     │      │        │
    │   │   ├──────────┤  ├───────────────┤      │        │
    │   │   │ RX Path  │  │ Rate Ctrl     │      │        │
    │   │   ├──────────┤  ├───────────────┤      │        │
    │   │   │ RxDet    │  │ LFPS Ctrl     │      │        │
    │   │   ├──────────┤  ├───────────────┤      │        │
    │   │   │ MsgBus   │  │ CSR Bridge    │      │        │
    │   │   ├──────────┤  ├───────────────┤      │        │
    │   │   │ MacCLK   │  │ DRP Mux       │      │        │
    │   │   └──────────┘  └───────────────┘      │        │
    │   └────────────────────────────────────────┘        │
    │                                                     │
    │   Exposed:  pipe  ←── PIPE Rev 7.1 interface        │
    │             debug ←── read-only status outputs       │
    │             por_n ──→ power-on reset to GowinSerDes  │
    └─────────────────────────────────────────────────────┘

The ``PIPESerDes`` component is fully self-contained.  It internally
creates and wires the ``GowinSerDes`` (hard macro driver),
``GowinSerDesGroup`` (lane grouping), and ``PIPESerDesAdapter`` (PIPE
protocol engine).  The user does **not** need to touch lane signals,
DRP ports, or PCS clock routing — all of that is handled inside.

Exposed ports:

``pipe``
    The PIPE Rev 7.1 low-pin-count interface (see ``PIPESerDesSignature``).
    This is what the MAC layer connects to.

``debug``
    Read-only debug/status outputs for ILA probes, LEDs, or a debug
    register file (see ``PIPEDebugSignature``).  Includes
    ``serdes_arb_state`` (2-bit) from the GowinSerDes UPAR arbiter.

``por_n``
    Active-low power-on reset input.  Directly drives ``GowinSerDes.por_n``.
    The user must assert this high after board-level power is stable.

Configurable via ``PIPELaneConfig`` for:

- Protocol: USB3, SATA, DP
- Device: GowinDevice (GW5AST_138, etc.)
- Rate: Gen1 (5 GT/s), Gen2 (10 GT/s)
- Data width: 10, 20, 32, 40, 64, 80 bits
- Quad/lane selection (inside config)
- Message bus enable
- MacCLK enable

Usage Example
-------------
::

    from gowin_serdes.config import GowinDevice
    from pipe_serdes import PIPESerDes, PIPELaneConfig, PIPEProtocol, USBRate, PIPEWidth

    cfg = PIPELaneConfig(
        protocol=PIPEProtocol.USB3,
        device=GowinDevice.GW5AST_138,
        quad=0, lane=0,
        supported_rates=[USBRate.GEN1],
        default_width=PIPEWidth.W40,
    )
    phy = PIPESerDes(cfg)

    # In elaborate():
    m.submodules.phy = DomainRenamer("upar")(phy)
    m.d.comb += [
        phy.por_n.eq(1),
        phy.pipe.reset_n.eq(my_reset_n),
        phy.pipe.power_down.eq(my_power_down),
        my_rx_data.eq(phy.pipe.rx_data),
    ]

    # To generate the .csr file for Gowin PnR:
    phy.generate_csr("serdes.csr", toml_path="serdes.toml")
"""

import sys
from typing import Optional

from amaranth.hdl import Signal, Module, ClockDomain, ClockSignal, DomainRenamer
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.wiring import Component, In, Out

from .pipe_config import PIPELaneConfig, PIPEWidth, PIPE_WIDTH_MAP
from .pipe_signature import PIPESerDesSignature, PIPEDebugSignature
from .pipe_adapter import PIPESerDesAdapter
from .pipe_lfps_gen import PIPELFPSGen

# Ensure gowin_serdes is importable
sys.path.insert(0, "/home/key2/Downloads/amaranth/serdes_pipe/gowin-serdes")

from gowin_serdes import GowinSerDes, GowinSerDesGroup  # noqa: E402


class PIPESerDes(Component):
    """Top-level self-contained PIPE SerDes PHY.

    Creates and wires ``GowinSerDes`` + ``GowinSerDesGroup`` +
    ``PIPESerDesAdapter`` internally.  The user only interacts with
    three port groups:

    - ``pipe`` — the PIPE Rev 7.1 interface (MAC ↔ PHY)
    - ``debug`` — read-only debug/status outputs
    - ``por_n`` — power-on reset input to the SerDes hard macro

    All internal wiring (lane TX/RX data, DRP bus, PCS clock loopback,
    reset routing, status feedback) is handled automatically.

    Parameters
    ----------
    pipe_config : PIPELaneConfig
        PIPE-layer configuration specifying protocol, device, quad, lane,
        supported rates, data width, and feature enables.  The ``quad``,
        ``lane``, and ``device`` fields are extracted from this config —
        no separate kwargs needed.

    Attributes
    ----------
    pipe : PIPESerDesSignature
        The PIPE Rev 7.1 interface.  Direction is ``Out`` from this
        component's perspective, meaning the MAC connects to the flipped
        view.

        Data path:
            - ``tx_data``               : In(data_width) — MAC → PHY transmit data
            - ``tx_data_valid``         : In(1) — TX data qualifier
            - ``rx_data``               : Out(data_width) — PHY → MAC receive data

        Clocks:
            - ``pclk``                  : In(1) — MAC-supplied parallel clock
            - ``max_pclk``              : Out(1) — PHY-generated max-rate PCLK
            - ``rx_clk``                : Out(1) — CDR-recovered byte clock

        Command (MAC → PHY):
            - ``phy_mode``              : In(4) — Protocol selector
            - ``reset_n``               : In(1) — Active-low async reset
            - ``power_down``            : In(4) — Power state request (P0–P3)
            - ``rate``                  : In(4) — Line rate selector
            - ``width``                 : In(3) — TX width selector
            - ``rx_width``              : In(3) — RX width selector
            - ``tx_elec_idle``          : In(4) — TX electrical idle
            - ``tx_detect_rx_loopback`` : In(1) — Receiver detect trigger
            - ``rx_polarity``           : In(1) — [USB] RX polarity inversion
            - ``rx_termination``        : In(1) — [USB] RX termination control
            - ``rx_standby``            : In(1) — [USB/SATA] RX standby
            - ``pclk_change_ack``       : In(1) — [USB/SATA] PCLK change ack

        Status (PHY → MAC):
            - ``phy_status``            : Out(1) — Completion pulse
            - ``rx_valid``              : Out(1) — RX data valid
            - ``rx_elec_idle``          : Out(1) — RX electrical idle detect
            - ``rx_status``             : Out(3) — [USB/SATA] RX status code
            - ``pclk_change_ok``        : Out(1) — [USB/SATA] PCLK change ready

        Message bus:
            - ``m2p_msg_bus``           : In(8) — MAC-to-PHY messages
            - ``p2m_msg_bus``           : Out(8) — PHY-to-MAC messages

        MacCLK domain:
            - ``mac_clk_reset_n``       : In(1) — MacCLK domain reset
            - ``mac_clk_rate``          : In(5) — MacCLK rate selector
            - ``mac_clk_req``           : In(1) — MacCLK request
            - ``mac_clk_ack``           : Out(1) — MacCLK acknowledge
            - ``mac_clk``              : Out(1) — MacCLK output

    debug : PIPEDebugSignature
        Read-only debug outputs for internal state observation:
            - ``power_state``       : Out(4) — Current PIPE power state
            - ``rate_fsm_state``    : Out(4) — Rate-change FSM state
            - ``rxdet_fsm_state``   : Out(4) — Receiver-detect FSM state
            - ``lfps_fsm_state``    : Out(4) — LFPS FSM state
            - ``msgbus_fsm_state``  : Out(4) — Message bus FSM state
            - ``drp_mux_owner``     : Out(4) — DRP arbiter current owner
            - ``drp_mux_locked``    : Out(1) — DRP bus locked
            - ``drp_busy``          : Out(1) — DRP transaction in progress
            - ``pll_lock``          : Out(1) — PLL lock indicator
            - ``cdr_lock``          : Out(1) — CDR lock indicator
            - ``serdes_arb_state``  : Out(2) — GowinSerDes UPAR arbiter state

    por_n : In(1)
        Power-on reset input (active low).  Must be driven high by the
        user after board power is stable.  Directly forwarded to the
        internal ``GowinSerDes.por_n`` port.
    """

    def __init__(self, pipe_config: PIPELaneConfig):
        self._config = pipe_config

        data_width = pipe_config.max_data_width

        members = {}

        # PIPE interface — direction is Out from PHY's perspective;
        # the MAC sees the flipped (In) view when it connects.
        members["pipe"] = Out(
            PIPESerDesSignature(
                data_width=data_width,
                usb=(pipe_config.protocol.value == 1),
                sata=(pipe_config.protocol.value == 2),
                dp=(pipe_config.protocol.value == 3),
            )
        )

        # Debug interface — always present, read-only status outputs
        members["debug"] = Out(PIPEDebugSignature())

        # Power-on reset — user drives this high after power is stable
        members["por_n"] = In(1)

        super().__init__(members)

    # ── CSR generation ────────────────────────────────────────────

    def generate_csr(
        self,
        output_path: str,
        toml_path: Optional[str] = None,
        gowin_bin_dir: Optional[str] = None,
    ):
        """Generate the SerDes ``.csr`` file needed by the Gowin PnR tool.

        Creates a fresh ``GowinSerDes`` + ``GowinSerDesGroup`` from the
        stored configuration, then delegates to
        ``GowinSerDes.generate_csr()``.

        Parameters
        ----------
        output_path : str
            Where to write the ``.csr`` file.
        toml_path : str or None
            If given, keep the intermediate TOML file at this path.
            Otherwise a temporary file is used and cleaned up.
        gowin_bin_dir : str or None
            Explicit path to the Gowin IDE ``bin/`` directory.
            If None, the tool is searched on ``$PATH``,
            then ``$GOWIN_IDE/bin/``.

        Returns
        -------
        str
            The *output_path*.

        Raises
        ------
        FileNotFoundError
            If the Gowin CSR tool binary is not found.
        subprocess.CalledProcessError
            If the tool exits non-zero.
        """
        lane_config = self._config.to_lane_config()
        group = GowinSerDesGroup(
            quad=self._config.quad,
            first_lane=self._config.lane,
            lane_configs=[lane_config],
        )
        serdes = GowinSerDes(device=self._config.device, groups=[group])
        return serdes.generate_csr(
            output_path=output_path,
            toml_path=toml_path,
            gowin_bin_dir=gowin_bin_dir,
        )

    # ── Elaboration ───────────────────────────────────────────────

    def elaborate(self, platform):
        """Build the complete PIPE SerDes hardware.

        Internally instantiates:

        1. ``GowinSerDes`` with a single ``GowinSerDesGroup`` matching
           the configured quad/lane/rate/width.
        2. ``PIPESerDesAdapter`` containing all PIPE sub-controllers
           (power FSM, rate controller, TX/RX paths, RxDet, LFPS,
           message bus, CSR bridge, MacCLK generator, DRP mux).

        Then wires everything together:

        - ``por_n`` → GowinSerDes
        - PCS clock loopback (rx.clk ← rx.pcs_clkout, tx.clk ← tx.pcs_clkout)
        - Adapter lane ↔ GowinSerDesLane (TX data, RX data, status, resets)
        - Adapter DRP ↔ GowinSerDes DRP port
        - Adapter pipe ↔ self.pipe (PIPE interface passthrough)
        - Adapter debug ↔ self.debug (debug outputs passthrough)
        - GowinSerDes ``dbg_arb_state`` → self.debug.serdes_arb_state

        Returns
        -------
        Module
            The elaborated Amaranth module containing the complete PHY.
        """
        m = Module()

        cfg = self._config

        # ══════════════════════════════════════════════════════════
        #  Step (a): Create GowinSerDes internally
        # ══════════════════════════════════════════════════════════
        lane_config = cfg.to_lane_config()
        group = GowinSerDesGroup(
            quad=cfg.quad,
            first_lane=cfg.lane,
            lane_configs=[lane_config],
        )
        serdes = GowinSerDes(device=cfg.device, groups=[group])
        m.submodules.serdes = serdes

        # Forward power-on reset from user to the hard macro
        m.d.comb += serdes.por_n.eq(self.por_n)

        # ══════════════════════════════════════════════════════════
        #  Step (b): Get lane and DRP references
        # ══════════════════════════════════════════════════════════
        lane0 = group.lanes[0]
        drp = getattr(serdes, f"drp_q{cfg.quad}_ln{cfg.lane}")

        # ══════════════════════════════════════════════════════════
        #  Step (c): PCS clock loopback + TX clock domain
        #
        #  The GTR12 PCS needs an explicit clock source.  We loop
        #  the PCS clock outputs back to the PCS clock inputs so
        #  the TX and RX PCS blocks run off their own generated
        #  clocks.  This is the standard self-clocking configuration.
        #
        #  We also create a ``pclk_tx`` clock domain from the TX PCS
        #  output.  This is the clock the serializer samples tx_data
        #  on (125 MHz for Gen1 W40).  The LFPS pattern generator
        #  runs in this domain to produce clean edges.
        #
        #  Note: pclk_tx is only valid after CMU PLL lock.  The LFPS
        #  generator is only enabled when lfps_active is high, which
        #  happens after the LFPS controller (running in upar/sync)
        #  has completed FFE setup — well after PLL lock.
        #
        #  Fabric clock assignment — matching Gowin reference design:
        #
        #  RX fabric clock = reference clock (125 MHz from CMU_CK_REF)
        #    The SerDes RX FIFO is a CDC FIFO: write side uses the
        #    CDR-recovered clock internally, read side MUST use an
        #    independent clock. Using pcs_clkout here violates the
        #    FIFO's CDC requirements and produces all-ones on rx_data.
        #
        #  TX fabric clock = recovered RX PCS clock
        #    Gowin uses the recovered clock for the TX fabric side.
        #    This keeps TX synchronous with the incoming data rate.
        # ══════════════════════════════════════════════════════════
        m.d.comb += [
            lane0.rx.clk.eq(
                lane0.tx.pcs_clkout
            ),  # TX PCS clock (62.5 MHz, independent from CDR)
            lane0.tx.clk.eq(lane0.tx.pcs_clkout),  # TX PCS clock
        ]

        m.domains += ClockDomain("pclk_tx", local=True, async_reset=True)
        m.d.comb += ClockSignal("pclk_tx").eq(lane0.rx.pcs_clkout)  # Recovered RX clock

        # ══════════════════════════════════════════════════════════
        #  Step (d): Create PIPE adapter (all sub-controllers)
        # ══════════════════════════════════════════════════════════
        adapter = PIPESerDesAdapter(
            cfg,
            quad=cfg.quad,
            lane=cfg.lane,
        )
        m.submodules.adapter = adapter

        # ══════════════════════════════════════════════════════════
        #  Step (d2): LFPS pattern generator (pclk_tx domain)
        #
        #  Sits between PIPE tx_data and the adapter txpath.
        #  In PHY-LFPS mode (MacTransmitLFPS=0): muxes in the
        #  internal all-1s/all-0s pattern at pclk_tx rate.
        #  In MAC-LFPS mode (MacTransmitLFPS=1): passes MAC's
        #  tx_data straight through.
        #  When not in LFPS: always passes through.
        #
        #  The LFPS gen runs in pclk_tx (TX PCS clock, 125 MHz
        #  for Gen1 W40).  lfps_active comes from the LFPS
        #  controller in upar, so we CDC it to pclk_tx.
        #  mac_transmit_lfps comes from the MAC in whatever domain
        #  the MAC runs — we CDC it to pclk_tx too.
        # ══════════════════════════════════════════════════════════
        fabric_width = cfg.max_data_width
        # Determine line rate from the first (default) supported rate.
        # USBRate.GEN1 = 0 → 5 GT/s, USBRate.GEN2 = 1 → 10 GT/s
        from .pipe_config import USBRate

        default_rate = cfg.supported_rates[0] if cfg.supported_rates else USBRate.GEN1
        line_rate_hz = 5_000_000_000 if default_rate == USBRate.GEN1 else 10_000_000_000

        # PCS_TX_O_FABRIC_CLK = line_rate / fabric_width (SDR).
        # Confirmed by hardware counter measurement: 125 MHz for Gen1 W40.
        pclk_hz = line_rate_hz // fabric_width
        print(
            f"[PIPESerDes] LFPS gen: width={fabric_width} line_rate={line_rate_hz / 1e9}G "
            f"pclk_hz={pclk_hz} ({pclk_hz / 1e6} MHz)"
        )
        lfps_gen = PIPELFPSGen(width=fabric_width, pclk_hz=pclk_hz)
        print(
            f"[PIPESerDes] LFPS gen instantiated: half_period={lfps_gen.half_period} "
            f"expected_freq={pclk_hz / (2 * lfps_gen.half_period) / 1e6} MHz"
        )
        m.submodules.lfps_gen = DomainRenamer("pclk_tx")(lfps_gen)

        # CDC: lfps_active (upar → pclk_tx)
        lfps_active_pclk = Signal(name="lfps_active_pclk")
        m.submodules += FFSynchronizer(
            adapter.lfps_active, lfps_active_pclk, o_domain="pclk_tx"
        )

        # CDC: mac_transmit_lfps (MAC domain → pclk_tx)
        # The MAC drives this through the PIPE interface;
        # it changes rarely (once before entering P0 for LFPS).
        mac_transmit_lfps_pclk = Signal(name="mac_xmit_lfps_pclk")
        m.submodules += FFSynchronizer(
            self.pipe.mac_transmit_lfps, mac_transmit_lfps_pclk, o_domain="pclk_tx"
        )

        # Wire LFPS gen control
        m.d.comb += [
            lfps_gen.lfps_active.eq(lfps_active_pclk),
            lfps_gen.mac_transmit_lfps.eq(mac_transmit_lfps_pclk),
        ]

        # Wire LFPS gen MAC-side inputs (from PIPE interface)
        m.d.comb += [
            lfps_gen.mac_tx_data.eq(self.pipe.tx_data),
            lfps_gen.mac_tx_data_valid.eq(self.pipe.tx_data_valid),
            lfps_gen.mac_tx_elec_idle.eq(self.pipe.tx_elec_idle[0]),
        ]

        # Raw MAC tx_elec_idle → adapter (for LFPS controller + RxDet).
        # These must see the MAC's original intent, NOT the post-mux value
        # (which the LFPS gen forces to 0 during PHY-LFPS bursts).
        m.d.comb += adapter.mac_tx_elec_idle_raw.eq(self.pipe.tx_elec_idle[0])

        # ══════════════════════════════════════════════════════════
        #  Step (e): Wire adapter.lane ↔ GowinSerDesLane
        # ══════════════════════════════════════════════════════════

        # --- TX data path → lane ---
        m.d.comb += [
            lane0.tx.data.eq(adapter.lane.tx.data),
            lane0.tx.fifo_wren.eq(adapter.lane.tx.fifo_wren),
        ]

        # --- RX data path ← lane ---
        m.d.comb += [
            adapter.lane.rx.data.eq(lane0.rx.data),
            adapter.lane.rx.valid.eq(lane0.rx.valid),
            adapter.lane.rx.fifo_aempty.eq(lane0.rx.fifo_aempty),
            lane0.rx.fifo_rden.eq(adapter.lane.rx.fifo_rden),
        ]

        # --- PCS clock feedback (lane → adapter) ---
        m.d.comb += [
            adapter.lane.tx.pcs_clkout.eq(lane0.tx.pcs_clkout),
            adapter.lane.rx.pcs_clkout.eq(lane0.rx.pcs_clkout),
        ]

        # --- Status (lane → adapter) ---
        m.d.comb += [
            adapter.lane.status.ready.eq(lane0.status.ready),
            adapter.lane.status.pll_lock.eq(lane0.status.pll_lock),
            adapter.lane.status.rx_cdr_lock.eq(lane0.status.rx_cdr_lock),
            adapter.lane.status.signal_detect.eq(lane0.status.signal_detect),
        ]

        # --- Resets ---
        # Matching Gowin reference: PMA reset and PCS resets are never
        # asserted. The Gowin reference hardwires:
        #   serdes_pma_rstn_o    = 1   (always deasserted)
        #   serdes_pcs_rx_rst_o  = 0   (never asserted)
        #   serdes_pcs_tx_rst_o  = 0   (never asserted)
        #   serdes_fabric_rstn_o = 1   (always deasserted)
        # This breaks the circular dependency where:
        #   Init needs DRP → DRP needs pma_rstn → pma_rstn needs power FSM
        #   → power FSM needs init_done → init_done needs DRP
        m.d.comb += [
            lane0.reset.pma_rstn.eq(self.por_n),  # PMA active once POR done
            lane0.reset.pcs_rx_rst.eq(0),  # Never assert PCS resets
            lane0.reset.pcs_tx_rst.eq(0),
        ]

        # --- Misc lane signals ---
        m.d.comb += [
            adapter.lane.rx_elec_idle.eq(
                0
            ),  # TODO: wire when lane exposes RXELECIDLE_O
            adapter.lane.life_clk.eq(
                0
            ),  # Not used by adapter (life_clk is internal to GowinSerDes)
        ]

        # ══════════════════════════════════════════════════════════
        #  Step (f): Wire adapter.drp ↔ GowinSerDes DRP port
        # ══════════════════════════════════════════════════════════
        m.d.comb += [
            # Request: adapter → SerDes arbiter
            drp.addr.eq(adapter.drp.addr),
            drp.wren.eq(adapter.drp.wren),
            drp.wrdata.eq(adapter.drp.wrdata),
            drp.strb.eq(adapter.drp.strb),
            drp.rden.eq(adapter.drp.rden),
            # Response: SerDes arbiter → adapter
            adapter.drp.ready.eq(drp.ready),
            adapter.drp.rdvld.eq(drp.rdvld),
            adapter.drp.rddata.eq(drp.rddata),
            adapter.drp.resp.eq(drp.resp),
        ]

        # ══════════════════════════════════════════════════════════
        #  Step (g): Wire adapter.pipe ↔ self.pipe
        #
        #  The adapter's PIPE sub-interface is a typed
        #  PIPESerDesSignature.  We forward every signal between
        #  the adapter and the user-facing component port.
        # ══════════════════════════════════════════════════════════

        # --- TX Data path: routed through LFPS gen ---
        # The LFPS gen (pclk_tx domain) muxes between:
        #   - Internal LFPS pattern (PHY-LFPS mode, MacTransmitLFPS=0)
        #   - MAC tx_data passthrough (MAC-LFPS mode or normal data)
        # Its outputs are combinational and feed the adapter's txpath.
        m.d.comb += [
            adapter.pipe.tx_data.eq(lfps_gen.tx_data),
            adapter.pipe.tx_data_valid.eq(lfps_gen.tx_data_valid),
            adapter.pipe.tx_elec_idle.eq(lfps_gen.tx_elec_idle),
        ]
        # --- RX Data path: direct from adapter (upar domain) ---
        m.d.comb += self.pipe.rx_data.eq(adapter.pipe.rx_data)

        # --- Clock interface ---
        m.d.comb += [
            self.pipe.max_pclk.eq(adapter.pipe.max_pclk),
            self.pipe.rx_clk.eq(adapter.pipe.rx_clk),
        ]

        # --- Command interface (MAC → PHY) ---
        m.d.comb += [
            adapter.pipe.phy_mode.eq(self.pipe.phy_mode),
            adapter.pipe.reset_n.eq(self.pipe.reset_n),
            adapter.pipe.power_down.eq(self.pipe.power_down),
            adapter.pipe.rate.eq(self.pipe.rate),
            adapter.pipe.width.eq(self.pipe.width),
            adapter.pipe.rx_width.eq(self.pipe.rx_width),
            # tx_elec_idle routed through lfps_gen (step d2) — not wired here.
            adapter.pipe.tx_detect_rx_loopback.eq(self.pipe.tx_detect_rx_loopback),
        ]

        # USB-only command signals (conditionally present in signature)
        if hasattr(self.pipe, "rx_polarity"):
            m.d.comb += adapter.pipe.rx_polarity.eq(self.pipe.rx_polarity)
        if hasattr(self.pipe, "rx_termination"):
            m.d.comb += adapter.pipe.rx_termination.eq(self.pipe.rx_termination)
        # mac_transmit_lfps is routed to PIPELFPSGen in step (d2),
        # not to the adapter — it's consumed only by the LFPS gen mux.

        # USB + SATA shared command signals
        if hasattr(self.pipe, "rx_standby"):
            m.d.comb += adapter.pipe.rx_standby.eq(self.pipe.rx_standby)
        if hasattr(self.pipe, "pclk_change_ack"):
            m.d.comb += adapter.pipe.pclk_change_ack.eq(self.pipe.pclk_change_ack)

        # --- Status interface (PHY → MAC) ---
        m.d.comb += [
            self.pipe.phy_status.eq(adapter.pipe.phy_status),
            self.pipe.rx_valid.eq(adapter.pipe.rx_valid),
            self.pipe.rx_elec_idle.eq(adapter.pipe.rx_elec_idle),
        ]

        # USB + SATA shared status signals
        if hasattr(self.pipe, "rx_status"):
            m.d.comb += self.pipe.rx_status.eq(adapter.pipe.rx_status)
        if hasattr(self.pipe, "pclk_change_ok"):
            m.d.comb += self.pipe.pclk_change_ok.eq(adapter.pipe.pclk_change_ok)

        # --- Message bus ---
        m.d.comb += [
            adapter.pipe.m2p_msg_bus.eq(self.pipe.m2p_msg_bus),
            self.pipe.p2m_msg_bus.eq(adapter.pipe.p2m_msg_bus),
        ]

        # --- MacCLK domain ---
        m.d.comb += [
            adapter.pipe.mac_clk_reset_n.eq(self.pipe.mac_clk_reset_n),
            adapter.pipe.mac_clk_rate.eq(self.pipe.mac_clk_rate),
            adapter.pipe.mac_clk_req.eq(self.pipe.mac_clk_req),
            self.pipe.mac_clk_ack.eq(adapter.pipe.mac_clk_ack),
            self.pipe.mac_clk.eq(adapter.pipe.mac_clk),
        ]

        # --- PCLK passthrough ---
        m.d.comb += adapter.pipe.pclk.eq(self.pipe.pclk)

        # ══════════════════════════════════════════════════════════
        #  Step (h): Wire adapter.debug ↔ self.debug
        # ══════════════════════════════════════════════════════════
        m.d.comb += [
            self.debug.power_state.eq(adapter.debug.power_state),
            self.debug.rate_fsm_state.eq(adapter.debug.rate_fsm_state),
            self.debug.rxdet_fsm_state.eq(adapter.debug.rxdet_fsm_state),
            self.debug.lfps_fsm_state.eq(adapter.debug.lfps_fsm_state),
            self.debug.msgbus_fsm_state.eq(adapter.debug.msgbus_fsm_state),
            self.debug.drp_mux_owner.eq(adapter.debug.drp_mux_owner),
            self.debug.drp_mux_locked.eq(adapter.debug.drp_mux_locked),
            self.debug.drp_busy.eq(adapter.debug.drp_busy),
            self.debug.pll_lock.eq(adapter.debug.pll_lock),
            self.debug.cdr_lock.eq(adapter.debug.cdr_lock),
        ]

        # ══════════════════════════════════════════════════════════
        #  Step (i): GowinSerDes debug — UPAR arbiter state
        #
        #  The GowinSerDes exposes a 2-bit ``dbg_arb_state`` that
        #  indicates the UPAR arbiter's internal state.  We surface
        #  this on the debug interface for anti-sweep / ILA use.
        # ══════════════════════════════════════════════════════════
        m.d.comb += self.debug.serdes_arb_state.eq(serdes.dbg_arb_state)

        return m
