"""PIPE SerDes adapter — top-level wiring component.

Assembles all PIPE sub-controllers and wires them to both the PIPE interface
(MAC side) and the GowinSerDesLane (hardware side).  This is the single
module a platform integrator instantiates to get a complete PIPE Rev 7.1
PHY on top of a Gowin GTR12 transceiver lane.

Architecture
------------
::

    ┌─────────────────────────────────────────────┐
    │ PIPESerDesAdapter                            │
    │  ├── PIPEInitFSM                             │
    │  ├── PIPEPowerFSM                            │
    │  ├── PIPERateController                      │
    │  ├── PIPETXPath                              │
    │  ├── PIPERXPath                              │
    │  ├── PIPERxDetController                     │
    │  ├── PIPELFPSController                      │
    │  ├── PIPEMessageBus                          │
    │  │    └── PIPECSRBridge                      │
    │  ├── PIPEMacCLKGen                           │
    │  └── PIPEDRPMux (7 clients)                  │
    │       └── DRP Port (single) → external       │
    └─────────────────────────────────────────────┘

DRP mux client assignment (from ``DRPClientID``):

===  =============  ==========================================
 ID  Client         Purpose
===  =============  ==========================================
  0  Init           Power-on CSR init sequence (highest prio)
  1  Rate           7-register atomic rate-change sequence
  2  LFPS           FFE save/restore + eidle toggle
  3  RxDet          3-step receiver detection pulse
  4  Power          Eidle CSR writes for P-state transitions
  5  Eidle          Reserved (power FSM handles eidle directly)
  6  CSR Bridge     Message-bus background register access
===  =============  ==========================================

Signal routing summary
----------------------

**PIPE → sub-controllers:**

- ``power_down``, ``reset_n`` → PIPEPowerFSM
- ``rate``, ``pclk_change_ack`` → PIPERateController
- ``tx_data``, ``tx_data_valid``, ``tx_elec_idle`` → PIPETXPath
- ``tx_detect_rx_loopback``, ``tx_elec_idle`` → PIPERxDetController
- ``tx_elec_idle``, ``tx_detect_rx_loopback`` → PIPELFPSController
- ``width`` → PIPETXPath (active width)
- ``rx_width`` → PIPERXPath (active width)
- ``m2p_msg_bus`` → PIPEMessageBus
- ``mac_clk_reset_n``, ``mac_clk_rate``, ``mac_clk_req`` → PIPEMacCLKGen

**Sub-controllers → PIPE:**

- ``phy_status`` ← OR of power, rate, rxdet PhyStatus outputs
- ``rx_status`` ← muxed from rxdet (priority) or RX path
- ``rx_valid`` ← PIPERXPath
- ``rx_elec_idle`` ← PIPELFPSController
- ``rx_data`` ← PIPERXPath
- ``pclk_change_ok`` ← PIPERateController
- ``p2m_msg_bus`` ← PIPEMessageBus
- ``mac_clk_ack``, ``mac_clk`` ← PIPEMacCLKGen

**Sub-controllers → DRP mux → external DRP port:**

Each sub-controller's DRP signals (addr, wrdata, wren, rden, lock_req)
connect to the corresponding client slot on ``PIPEDRPMux``.  The mux
output connects to the external ``drp`` port which is wired to the
``GowinSerDes`` top-level DRP interface.

**Clock routing:**

- ``max_pclk`` is driven from the lane's ``tx.pcs_clkout`` (TX PCS
  parallel clock output, which is the reference for PCLK at the
  maximum supported rate).
- ``rx_clk`` is driven from the lane's ``rx.pcs_clkout`` (recovered
  clock from the CDR, byte-rate).

**Reset routing:**

The PIPEPowerFSM generates ``pma_rstn``, ``pcs_rx_rst``, and
``pcs_tx_rst`` which are combined (OR'd) with the rate controller's
reset requests and driven to the lane's reset sub-interface.

Parameters
----------
config : PIPELaneConfig
    Per-lane configuration controlling protocol, supported rates,
    data-path width, and optional feature enables.
"""

from amaranth.hdl import Signal, Module, Const
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, Signature, connect

from .pipe_config import PIPELaneConfig, DRPClientID
from .pipe_signature import PIPESerDesSignature, PIPEDebugSignature
from .pipe_power import PIPEPowerFSM
from .pipe_rate import PIPERateController
from .pipe_txpath import PIPETXPath
from .pipe_rxpath import PIPERXPath
from .pipe_rxdet import PIPERxDetController
from .pipe_lfps import PIPELFPSController
from .pipe_msgbus import PIPEMessageBus
from .pipe_csr_bridge import PIPECSRBridge
from .pipe_macclk import PIPEMacCLKGen
from .pipe_drp_mux import PIPEDRPMux
from .pipe_init import PIPEInitFSM


# ═══════════════════════════════════════════════════════════════════════════
#  DRP output signature (matches gowin_serdes DRPSignature field names)
# ═══════════════════════════════════════════════════════════════════════════


class DRPOutputSignature(Signature):
    """External DRP port presented by the adapter.

    Matches the ``DRPSignature`` from ``gowin_serdes.signature`` so that
    the adapter can be wired directly to a ``GowinSerDes`` DRP port.
    Directions are from the adapter's (client's) perspective:

    - ``Out`` = driven by the adapter toward the SerDes arbiter
    - ``In``  = driven by the SerDes arbiter back to the adapter
    """

    def __init__(self):
        super().__init__(
            {
                "addr": Out(24),
                "wren": Out(1),
                "wrdata": Out(32),
                "strb": Out(8),
                "rden": Out(1),
                "ready": In(1),
                "rdvld": In(1),
                "rddata": In(32),
                "resp": In(1),
                "clk": In(1),
            }
        )


# ═══════════════════════════════════════════════════════════════════════════
#  Lane interface signature (subset of GowinSerDesLane ports used here)
# ═══════════════════════════════════════════════════════════════════════════


class LaneTXPortSignature(Signature):
    """TX sub-interface toward the GowinSerDesLane.

    Directions are from the adapter's perspective (adapter drives data
    *into* the lane TX path).
    """

    def __init__(self, data_width=80):
        super().__init__(
            {
                "pcs_clkout": In(1),  # TX PCS parallel clock (from lane)
                "data": Out(data_width),  # TX data to serialiser
                "fifo_wren": Out(1),  # TX FIFO write enable
            }
        )


class LaneRXPortSignature(Signature):
    """RX sub-interface toward the GowinSerDesLane.

    Directions are from the adapter's perspective (adapter reads data
    *from* the lane RX path).
    """

    def __init__(self, data_width=88):
        super().__init__(
            {
                "pcs_clkout": In(1),  # RX recovered clock (from lane)
                "data": In(data_width),  # RX data from deserialiser
                "fifo_rden": Out(1),  # RX FIFO read enable
                "valid": In(1),  # RX data valid
            }
        )


class LaneStatusPortSignature(Signature):
    """Lane status signals from GowinSerDesLane."""

    def __init__(self):
        super().__init__(
            {
                "pll_lock": In(1),  # PLL / CMU lock
                "rx_cdr_lock": In(1),  # CDR lock
                "ready": In(1),  # Lane ready
                "signal_detect": In(1),  # Signal detect
            }
        )


class LaneResetPortSignature(Signature):
    """Lane reset controls toward GowinSerDesLane."""

    def __init__(self):
        super().__init__(
            {
                "pma_rstn": Out(1),  # PMA reset (active low)
                "pcs_rx_rst": Out(1),  # PCS RX reset (active high)
                "pcs_tx_rst": Out(1),  # PCS TX reset (active high)
            }
        )


class LanePortSignature(Signature):
    """Complete lane interface for adapter ↔ GowinSerDesLane wiring."""

    def __init__(self, tx_data_width=80, rx_data_width=88):
        super().__init__(
            {
                "tx": Out(LaneTXPortSignature(tx_data_width)),
                "rx": Out(LaneRXPortSignature(rx_data_width)),
                "status": Out(LaneStatusPortSignature()),
                "reset": Out(LaneResetPortSignature()),
                "rx_elec_idle": In(1),  # RXELECIDLE_O from GTR12
                "life_clk": In(1),  # Free-running ~62.5 MHz ref
                "quad_pd": Out(3),  # Power-down control to QUAD
            }
        )


# ═══════════════════════════════════════════════════════════════════════════
#  PIPESerDesAdapter — top-level wiring component
# ═══════════════════════════════════════════════════════════════════════════


class PIPESerDesAdapter(wiring.Component):
    """PIPE Rev 7.1 SerDes adapter for Gowin GTR12.

    This is the top-level component that a platform integrator instantiates
    to obtain a complete PIPE PHY.  It bridges the PIPE interface (consumed
    by a MAC such as a USB 3.x XHCI controller) and the GowinSerDesLane
    hardware interface.

    The adapter instantiates and wires together all sub-controllers:

    - **PIPEInitFSM** — Power-on SerDes CSR initialization sequence
    - **PIPEPowerFSM** — P0/P1/P2/P3 power-state management
    - **PIPERateController** — Gen1 ↔ Gen2 rate switching
    - **PIPETXPath** — TX data-path width adaptation
    - **PIPERXPath** — RX data-path width adaptation
    - **PIPERxDetController** — Receiver detection pulse sequencing
    - **PIPELFPSController** — LFPS TX/RX signalling
    - **PIPEMessageBus** — PIPE 8-bit message bus protocol engine
    - **PIPECSRBridge** — Message bus ↔ DRP register translation
    - **PIPEMacCLKGen** — MacCLK domain clock gating
    - **PIPEDRPMux** — 7-client priority DRP arbiter/multiplexer

    Parameters
    ----------
    config : PIPELaneConfig
        Per-lane configuration (protocol, rates, width, feature enables).

    Interface Ports
    ---------------
    pipe : PIPESerDesSignature
        PIPE interface (MAC side).  Directly exposes all PIPE signals.
    drp : DRPOutputSignature
        External DRP port.  Connects to the GowinSerDes top-level DRP
        arbiter for CSR access to the GTR12 transceiver.
    lane : LanePortSignature
        Hardware lane interface.  Connects to a GowinSerDesLane instance
        for TX/RX data, clocking, resets, and status.
    debug : PIPEDebugSignature
        Debug/status outputs for ILA probes, LEDs, or register readback.
    """

    def __init__(self, config: PIPELaneConfig, *, quad: int = 0, lane: int = 0):
        self._config = config
        self._quad = quad
        self._lane = lane
        dw = config.max_data_width

        # Exposed for LFPS gen wiring (not part of PIPE signature;
        # internal PHY signal consumed by PIPELFPSGen in pipe_serdes.py).
        self.lfps_active = Signal(name="adapter_lfps_active")
        # Raw MAC tx_elec_idle — LFPS controller and RxDet must see the
        # MAC's original intent, NOT the post-LFPS-gen muxed value.
        self.mac_tx_elec_idle_raw = Signal(name="mac_tx_eidle_raw")

        # Build the component signature with typed sub-interfaces.
        # Directions are from the adapter's (PHY's) perspective.
        members = {
            "pipe": Out(
                PIPESerDesSignature(
                    data_width=dw,
                    usb=(config.protocol == config.protocol.USB3),
                )
            ),
            "drp": Out(DRPOutputSignature()),
            "lane": Out(LanePortSignature(tx_data_width=80, rx_data_width=88)),
            "debug": Out(PIPEDebugSignature()),
        }

        super().__init__(members)

    def elaborate(self, platform):
        """Instantiate all sub-controllers and wire them together.

        The elaboration proceeds in phases:

        1. **Instantiate** all sub-controllers as local variables.
        2. **Wire PIPE inputs** to sub-controller input ports.
        3. **Wire sub-controller DRP ports** to DRP mux client slots.
        4. **Wire DRP mux output** to the external DRP port.
        5. **Wire sub-controller outputs** to PIPE output ports.
        6. **Wire lane interface** (TX/RX data, clocks, resets, status).
        7. **Wire debug outputs**.
        """
        m = Module()
        cfg = self._config
        dw = cfg.max_data_width

        # ══════════════════════════════════════════════════════════════
        #  Phase 1: Instantiate sub-controllers
        # ══════════════════════════════════════════════════════════════

        init = PIPEInitFSM(cfg)
        power = PIPEPowerFSM(quad=self._quad, lane=self._lane)
        rate = PIPERateController(quad=self._quad, lane=self._lane)
        txpath = PIPETXPath(max_width=dw)
        rxpath = PIPERXPath(max_width=dw)
        rxdet = PIPERxDetController(quad=self._quad, lane=self._lane)
        lfps = PIPELFPSController(quad=self._quad, lane=self._lane)
        msgbus = PIPEMessageBus()
        csr_bridge = PIPECSRBridge(quad=self._quad, lane=self._lane)
        macclk = PIPEMacCLKGen()
        drp_mux = PIPEDRPMux(num_clients=DRPClientID.NUM_CLIENTS.value)

        m.submodules.init = init
        m.submodules.power = power
        m.submodules.rate = rate
        m.submodules.txpath = txpath
        m.submodules.rxpath = rxpath
        m.submodules.rxdet = rxdet
        m.submodules.lfps = lfps
        m.submodules.msgbus = msgbus
        m.submodules.csr_bridge = csr_bridge
        m.submodules.macclk = macclk
        m.submodules.drp_mux = drp_mux

        # Shorthand aliases for interface ports
        pipe = self.pipe
        drp = self.drp
        lane = self.lane
        debug = self.debug

        # ══════════════════════════════════════════════════════════════
        #  Phase 2: Wire PIPE inputs → sub-controller inputs
        # ══════════════════════════════════════════════════════════════

        # --- Init FSM inputs ---
        m.d.comb += [
            init.reset_n.eq(pipe.reset_n),
        ]

        # --- Power FSM inputs ---
        # Power FSM only starts after init sequence is complete.
        m.d.comb += [
            power.power_down.eq(pipe.power_down),
            power.reset_n.eq(pipe.reset_n & init.init_done),
            power.tx_elec_idle.eq(pipe.tx_elec_idle[0]),
            power.pll_lock.eq(lane.status.pll_lock),
            power.cdr_lock.eq(lane.status.rx_cdr_lock),
            power.rate_change_ip.eq(rate.rate_change_ip),
            power.lfps_active.eq(lfps.lfps_active),
            self.lfps_active.eq(lfps.lfps_active),
        ]

        # --- Rate controller inputs ---
        m.d.comb += [
            rate.rate.eq(pipe.rate),
            rate.pclk_change_ack.eq(pipe.pclk_change_ack),
            rate.pll_lock.eq(lane.status.pll_lock),
            rate.reset_n.eq(pipe.reset_n),
            rate.current_power.eq(power.current_state),
        ]

        # --- TX path inputs ---
        m.d.comb += [
            txpath.pipe_tx_data.eq(pipe.tx_data),
            txpath.pipe_tx_data_valid.eq(pipe.tx_data_valid),
            txpath.pipe_tx_elec_idle.eq(pipe.tx_elec_idle[0]),
            txpath.pipe_power_down.eq(pipe.power_down),
            txpath.active_width.eq(pipe.width),
        ]

        # --- RX path inputs ---
        m.d.comb += [
            rxpath.quad_rx_data.eq(lane.rx.data),
            rxpath.quad_rx_cdr_lock.eq(lane.status.rx_cdr_lock),
            rxpath.active_width.eq(pipe.rx_width),
        ]

        # --- Receiver detection inputs ---
        m.d.comb += [
            rxdet.tx_detect_rx.eq(pipe.tx_detect_rx_loopback),
            rxdet.tx_elec_idle.eq(self.mac_tx_elec_idle_raw),
            rxdet.current_power.eq(power.current_state),
            rxdet.reset_n.eq(pipe.reset_n),
        ]

        # --- LFPS controller inputs ---
        m.d.comb += [
            lfps.tx_elec_idle.eq(self.mac_tx_elec_idle_raw),
            lfps.tx_detect_rx.eq(pipe.tx_detect_rx_loopback),
            lfps.current_power.eq(power.current_state),
            lfps.rx_elec_idle_hw.eq(lane.rx_elec_idle),
            lfps.reset_n.eq(pipe.reset_n),
        ]

        # --- Message bus inputs ---
        m.d.comb += [
            msgbus.m2p_bus.eq(pipe.m2p_msg_bus),
            msgbus.reset_n.eq(pipe.reset_n),
        ]

        # --- CSR bridge inputs (from message bus) ---
        m.d.comb += [
            csr_bridge.reg_addr.eq(msgbus.reg_addr),
            csr_bridge.reg_wrdata.eq(msgbus.reg_wrdata),
            csr_bridge.reg_wren.eq(msgbus.reg_wren),
            csr_bridge.reg_rden.eq(msgbus.reg_rden),
            csr_bridge.reset_n.eq(pipe.reset_n),
        ]

        # --- CSR bridge → message bus feedback ---
        m.d.comb += [
            msgbus.csr_rd_data.eq(csr_bridge.rd_data),
            msgbus.csr_rd_valid.eq(csr_bridge.rd_valid),
            msgbus.csr_wr_ack.eq(csr_bridge.wr_ack),
        ]

        # --- MacCLK generator inputs ---
        m.d.comb += [
            macclk.mac_clk_reset_n.eq(pipe.mac_clk_reset_n),
            macclk.mac_clk_rate.eq(pipe.mac_clk_rate),
            macclk.mac_clk_req.eq(pipe.mac_clk_req),
            macclk.life_clk.eq(lane.life_clk),
        ]

        # ══════════════════════════════════════════════════════════════
        #  Phase 3: Wire sub-controller DRP ports → DRP mux clients
        # ══════════════════════════════════════════════════════════════

        # Client 0: Init FSM (highest priority — runs once at power-on)
        _id = DRPClientID.INIT.value
        m.d.comb += [
            drp_mux.req_addr[_id].eq(init.drp_addr),
            drp_mux.req_wrdata[_id].eq(init.drp_wrdata),
            drp_mux.req_wren[_id].eq(init.drp_wren),
            drp_mux.req_rden[_id].eq(0),  # Init FSM is write-only
            drp_mux.req_lock[_id].eq(init.drp_lock_req),
            init.drp_ready.eq(drp_mux.req_ready[_id]),
        ]

        # Client 1: Rate controller
        _id = DRPClientID.RATE.value
        m.d.comb += [
            drp_mux.req_addr[_id].eq(rate.drp_addr),
            drp_mux.req_wrdata[_id].eq(rate.drp_wrdata),
            drp_mux.req_wren[_id].eq(rate.drp_wren),
            drp_mux.req_rden[_id].eq(0),  # Rate controller is write-only
            drp_mux.req_lock[_id].eq(rate.drp_lock_req),
            rate.drp_ready.eq(drp_mux.req_ready[_id]),
        ]

        # Client 2: LFPS controller
        _id = DRPClientID.LFPS.value
        m.d.comb += [
            drp_mux.req_addr[_id].eq(lfps.drp_addr),
            drp_mux.req_wrdata[_id].eq(lfps.drp_wrdata),
            drp_mux.req_wren[_id].eq(lfps.drp_wren),
            drp_mux.req_rden[_id].eq(0),  # LFPS controller is write-only
            drp_mux.req_lock[_id].eq(lfps.drp_lock_req),
            lfps.drp_ready.eq(drp_mux.req_ready[_id]),
        ]

        # Client 3: Receiver detection
        _id = DRPClientID.RXDET.value
        m.d.comb += [
            drp_mux.req_addr[_id].eq(rxdet.drp_addr),
            drp_mux.req_wrdata[_id].eq(rxdet.drp_wrdata),
            drp_mux.req_wren[_id].eq(rxdet.drp_wren),
            drp_mux.req_rden[_id].eq(rxdet.drp_rden),
            drp_mux.req_lock[_id].eq(rxdet.drp_lock_req),
            rxdet.drp_ready.eq(drp_mux.req_ready[_id]),
            rxdet.drp_rdvld.eq(drp_mux.req_rdvld[_id]),
            rxdet.drp_rddata.eq(drp_mux.req_rddata[_id]),
        ]

        # Client 4: Power FSM
        _id = DRPClientID.POWER.value
        m.d.comb += [
            drp_mux.req_addr[_id].eq(power.drp_addr),
            drp_mux.req_wrdata[_id].eq(power.drp_wrdata),
            drp_mux.req_wren[_id].eq(power.drp_wren),
            drp_mux.req_rden[_id].eq(0),  # Power FSM is write-only
            drp_mux.req_lock[_id].eq(power.drp_lock_req),
            power.drp_ready.eq(drp_mux.req_ready[_id]),
            power.drp_rdvld.eq(drp_mux.req_rdvld[_id]),
        ]

        # Client 5: Eidle (reserved — power FSM handles eidle directly)
        # Tie all request signals low so the mux never selects this slot.
        _id = DRPClientID.EIDLE.value
        m.d.comb += [
            drp_mux.req_addr[_id].eq(0),
            drp_mux.req_wrdata[_id].eq(0),
            drp_mux.req_wren[_id].eq(0),
            drp_mux.req_rden[_id].eq(0),
            drp_mux.req_lock[_id].eq(0),
        ]

        # Client 6: CSR bridge (lowest priority)
        _id = DRPClientID.CSR_BRIDGE.value
        m.d.comb += [
            drp_mux.req_addr[_id].eq(csr_bridge.drp_addr),
            drp_mux.req_wrdata[_id].eq(csr_bridge.drp_wrdata),
            drp_mux.req_wren[_id].eq(csr_bridge.drp_wren),
            drp_mux.req_rden[_id].eq(csr_bridge.drp_rden),
            drp_mux.req_lock[_id].eq(0),  # CSR bridge never locks
            csr_bridge.drp_ready.eq(drp_mux.req_ready[_id]),
            csr_bridge.drp_rdvld.eq(drp_mux.req_rdvld[_id]),
            csr_bridge.drp_rddata.eq(drp_mux.req_rddata[_id]),
        ]

        # ══════════════════════════════════════════════════════════════
        #  Phase 4: Wire DRP mux output → external DRP port
        # ══════════════════════════════════════════════════════════════

        m.d.comb += [
            # Adapter → SerDes arbiter
            drp.addr.eq(drp_mux.drp_addr),
            drp.wrdata.eq(drp_mux.drp_wrdata),
            drp.wren.eq(drp_mux.drp_wren),
            drp.strb.eq(drp_mux.drp_strb),
            drp.rden.eq(drp_mux.drp_rden),
            # SerDes arbiter → adapter (feedback)
            drp_mux.drp_ready.eq(drp.ready),
            drp_mux.drp_rdvld.eq(drp.rdvld),
            drp_mux.drp_rddata.eq(drp.rddata),
        ]

        # ══════════════════════════════════════════════════════════════
        #  Phase 5: Wire sub-controller outputs → PIPE output ports
        # ══════════════════════════════════════════════════════════════

        # --- PhyStatus: OR of all sub-controller pulses ---
        # Each source produces a single-cycle pulse; OR'ing guarantees
        # the MAC sees exactly one pulse per completed operation.  If
        # two operations complete on the same cycle (extremely unlikely),
        # the MAC still sees a single pulse, which is acceptable per
        # PIPE spec since the operations are distinguishable by context.
        m.d.comb += pipe.phy_status.eq(
            power.phy_status | rate.phy_status | rxdet.phy_status
        )

        # --- RxStatus: muxed based on active operation ---
        # Receiver detection has priority because its RxStatus carries
        # the detection result (0b011 = detected, 0b000 = not detected).
        # When rxdet is not reporting, RxStatus defaults to 0b000 (OK)
        # which the RX path / elastic buffer can override in the future
        # for SKP add/remove and decode error indications.
        with m.If(rxdet.rx_status_valid):
            m.d.comb += pipe.rx_status.eq(rxdet.rx_status)
        with m.Else():
            m.d.comb += pipe.rx_status.eq(0b000)

        # --- RxValid ---
        m.d.comb += pipe.rx_valid.eq(rxpath.pipe_rx_valid)

        # --- RxElecIdle ---
        m.d.comb += pipe.rx_elec_idle.eq(lfps.rx_elec_idle)

        # --- RxData ---
        m.d.comb += pipe.rx_data.eq(rxpath.pipe_rx_data)

        # --- PclkChangeOk ---
        m.d.comb += pipe.pclk_change_ok.eq(rate.pclk_change_ok)

        # --- Message bus P2M ---
        m.d.comb += pipe.p2m_msg_bus.eq(msgbus.p2m_bus)

        # --- MacCLK outputs ---
        m.d.comb += [
            pipe.mac_clk_ack.eq(macclk.mac_clk_ack),
            pipe.mac_clk.eq(macclk.mac_clk),
        ]

        # --- Clock outputs ---
        # MaxPCLK: derived from the TX PCS parallel clock output.
        # This is the highest-rate PCLK the PHY can provide.
        m.d.comb += pipe.max_pclk.eq(lane.tx.pcs_clkout)

        # RxCLK: recovered clock from the CDR.
        m.d.comb += pipe.rx_clk.eq(lane.rx.pcs_clkout)

        # ══════════════════════════════════════════════════════════════
        #  Phase 6: Wire lane interface (TX/RX data, resets, status)
        # ══════════════════════════════════════════════════════════════

        # --- TX data path → lane ---
        m.d.comb += [
            lane.tx.data.eq(txpath.quad_tx_data[:80]),
            lane.tx.fifo_wren.eq(txpath.quad_tx_vld),
        ]

        # --- RX FIFO read enable ---
        # Always read when CDR is locked and lane is in P0.  The RX
        # FIFO read enable is effectively "RxValid" — when the CDR is
        # locked and data is flowing, we always consume from the FIFO.
        m.d.comb += lane.rx.fifo_rden.eq(
            lane.status.rx_cdr_lock & (power.current_state == 0)
        )

        # --- Reset routing ---
        # PMA reset: active-low.  Deasserted (high) when power FSM says
        # PMA should be active AND rate controller is not requesting reset.
        m.d.comb += lane.reset.pma_rstn.eq(power.pma_rstn & ~rate.pma_rst_req)

        # PCS resets: active-high.  Asserted when either power FSM or
        # rate controller requests reset.
        m.d.comb += [
            lane.reset.pcs_rx_rst.eq(power.pcs_rx_rst | rate.pcs_rst_req),
            lane.reset.pcs_tx_rst.eq(power.pcs_tx_rst | rate.pcs_rst_req),
        ]

        # --- Quad power-down ---
        m.d.comb += lane.quad_pd.eq(power.quad_pd)

        # ══════════════════════════════════════════════════════════════
        #  Phase 7: Wire debug outputs
        # ══════════════════════════════════════════════════════════════

        m.d.comb += [
            debug.power_state.eq(power.current_state),
            debug.rate_fsm_state.eq(rate.fsm_state),
            debug.rxdet_fsm_state.eq(rxdet.fsm_state),
            debug.lfps_fsm_state.eq(lfps.fsm_state),
            debug.msgbus_fsm_state.eq(msgbus.fsm_state),
            debug.drp_mux_owner.eq(drp_mux.dbg_owner),
            debug.drp_mux_locked.eq(drp_mux.dbg_locked),
            debug.drp_busy.eq(drp_mux.drp_wren | drp_mux.drp_rden),
            debug.pll_lock.eq(lane.status.pll_lock),
            debug.cdr_lock.eq(lane.status.rx_cdr_lock),
            debug.init_done.eq(init.init_done),
            debug.init_fsm_state.eq(init.fsm_state),
        ]

        return m
