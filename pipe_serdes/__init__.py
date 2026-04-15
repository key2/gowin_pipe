"""PIPE SerDes — PIPE Rev 7.1 SerDes PHY for Gowin GTR12.

A configurable PIPE low-pin-count interface implementation that wraps
the gowin_serdes hardware abstraction layer.  Supports USB 3.1 Gen1/Gen2
with configurable data widths (10–80 bits).

Package Structure
-----------------
``pipe_config``
    Enumerations, hardware mapping tables, CSR addresses, and the
    ``PIPELaneConfig`` dataclass that configures the entire stack.

``pipe_signature``
    Amaranth ``Signature`` definitions for the PIPE interface, DRP
    request ports, and debug outputs.

``pipe_serdes``
    Top-level ``PIPESerDes`` component — the user-facing entry point
    that assembles the adapter on top of the GowinSerDes hard macro.

``pipe_adapter``
    ``PIPESerDesAdapter`` — orchestrates all sub-controllers and
    presents a flat signal interface to the top-level component.

``pipe_power``
    ``PIPEPowerFSM`` — P0/P1/P2/P3 power state machine per PIPE §8.3.

``pipe_rate``
    ``PIPERateController`` — Gen1 ↔ Gen2 rate switching via 7-register
    DRP write sequence.

``pipe_txpath``
    ``PIPETXPath`` — TX data path width adaptation (PIPE → GTR12).

``pipe_rxpath``
    ``PIPERXPath`` — RX data path width adaptation (GTR12 → PIPE).

``pipe_rxdet``
    ``PIPERxDetController`` — DRP-based receiver detection pulse
    sequence.

``pipe_lfps``
    ``PIPELFPSController`` — Low-Frequency Periodic Signaling TX/RX
    with FFE tap reconfiguration.

``pipe_msgbus``
    ``PIPEMessageBus`` — PIPE 8-bit message bus M2P decoder / P2M
    encoder.

``pipe_csr_bridge``
    ``PIPECSRBridge`` — Translates PIPE message-bus register writes
    into 32-bit DRP CSR writes with shadow registers.

``pipe_macclk``
    ``PIPEMacCLKGen`` — MacCLK domain generator derived from GTR12
    LIFE_CLK for low-power state MAC communication.

``pipe_drp_mux``
    ``PIPEDRPMux`` — Fixed-priority DRP arbiter with locking for
    atomic multi-write sequences.

Typical Usage
-------------
::

    from pipe_serdes import PIPESerDes, PIPELaneConfig, PIPEProtocol, USBRate, PIPEWidth

    cfg = PIPELaneConfig(
        protocol=PIPEProtocol.USB3,
        supported_rates=[USBRate.GEN1, USBRate.GEN2],
        default_width=PIPEWidth.W40,
    )
    phy = PIPESerDes(cfg, quad=0, lane=0)
"""

from gowin_serdes.config import GowinDevice
from .pipe_config import (
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
)
from .pipe_signature import (
    DRPRequestSignature,
    PIPESerDesSignature,
    PIPEDebugSignature,
)
from .pipe_adapter import PIPESerDesAdapter
from .pipe_serdes import PIPESerDes
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

__all__ = [
    # Config (re-exported from gowin_serdes)
    "GowinDevice",
    # Config
    "PIPEProtocol",
    "PIPEWidth",
    "USBRate",
    "SATARate",
    "PIPEPowerState",
    "PIPERxStatus",
    "DRPClientID",
    "PIPELaneConfig",
    "PIPE_WIDTH_MAP",
    "PIPE_USB_HW_MAP",
    "pclk_mhz",
    # Signatures
    "DRPRequestSignature",
    "PIPESerDesSignature",
    "PIPEDebugSignature",
    # Top-level
    "PIPESerDes",
    "PIPESerDesAdapter",
    # Sub-controllers
    "PIPEPowerFSM",
    "PIPERateController",
    "PIPETXPath",
    "PIPERXPath",
    "PIPERxDetController",
    "PIPELFPSController",
    "PIPEMessageBus",
    "PIPECSRBridge",
    "PIPEMacCLKGen",
    "PIPEDRPMux",
    "PIPEInitFSM",
]
