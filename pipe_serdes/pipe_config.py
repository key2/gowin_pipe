"""PIPE Rev 7.1 SerDes PHY configuration for Gowin GTR12.

This is the foundational configuration module used by ALL other pipe_serdes
modules.  It defines:

- Protocol, width, and rate enumerations for PIPE 7.1
- Hardware mapping tables that translate PIPE-level (rate, width) tuples into
  concrete Gowin GTR12 SerDes parameters (PCS width, gear ratio, encoding mode)
- CSR register addresses and magic values for the GTR12 DRP interface,
  re-exported from ``gowin_serdes.csr_map`` (the single source of truth for
  address computation).  Backward-compat aliases for quad=0, lane=0 are kept.
- A PCLK frequency calculator
- The ``PIPELaneConfig`` dataclass that serves as the top-level per-lane
  configuration passed to every PIPE sub-module
- DRP client priority encoding, power-state encoding, and RxStatus encoding

All addresses and values are taken from the Gowin IPUG1024E GTR12 programming
guide and the custom CSR map (CUSTOM_PHY_CSR_MAP.md).

Targets
-------
- PIPE specification revision **7.1**
- Gowin **GTR** transceiver (any Gowin FPGA with a GTR SerDes block)
- Protocols: **USB 3.x** (Gen1 5 GT/s, Gen2 10 GT/s), with SATA and DP
  protocol IDs reserved for future use.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

# ---------------------------------------------------------------------------
# Path setup for sibling package
# ---------------------------------------------------------------------------
sys.path.insert(0, "/home/key2/Downloads/amaranth/serdes_pipe/gowin-serdes")

from gowin_serdes.config import GearRate, EncodingMode, GowinDevice  # noqa: E402

# ---------------------------------------------------------------------------
# CSR address computation — delegated to gowin_serdes.csr_map (single source
# of truth).  All address helpers, magic values, and register-table builders
# are imported from there.  Backward-compat aliases for quad=0/lane=0 are
# kept below.
# ---------------------------------------------------------------------------
from gowin_serdes.csr_map import (  # noqa: E402
    CSR,
    csr_addr,
    EIDLE_ON as CSR_EIDLE_ON,
    EIDLE_OFF as CSR_EIDLE_OFF,
    RXDET_START as CSR_RXDET_PULSE_START,
    RXDET_END as CSR_RXDET_PULSE_END,
    RX_POLARITY_NORMAL as CSR_RX_POLARITY_NORMAL,
    RX_POLARITY_INVERT as CSR_RX_POLARITY_INVERT,
    BYPASS_8B10B as CSR_8B10B_BYPASS_VAL,
    rate_change_regs,
    lfps_ffe_regs,
    runtime_addrs,
)


# ═══════════════════════════════════════════════════════════════════════════
#  Enumerations
# ═══════════════════════════════════════════════════════════════════════════


class PIPEProtocol(Enum):
    """PIPE protocol selector.

    Only USB3 is actively implemented.  SATA and DP are reserved for future
    expansion of the PIPE adapter.
    """

    USB3 = 1
    SATA = 2
    DP = 3


class PIPEWidth(Enum):
    """PIPE data-path width selector.

    The standard PIPE 7.1 widths are 10, 20, and 40 bits (W10, W20, W40).
    W32, W64, and W80 are **custom extensions** that allow non-power-of-two
    symbol counts per fabric clock cycle, enabling higher fabric frequencies
    or better BRAM utilisation.

    The integer value is used as an index into ``PIPE_WIDTH_MAP``.
    """

    W10 = 0  # 10-bit — 1 symbol/cycle  (8b10b coded)
    W20 = 1  # 20-bit — 2 symbols/cycle
    W40 = 2  # 40-bit — 4 symbols/cycle
    W32 = 3  # 32-bit — 3 symbols/cycle  (non-clean, custom)
    W64 = 4  # 64-bit — 6 symbols/cycle  (non-clean, custom)
    W80 = 5  # 80-bit — 8 symbols/cycle


class USBRate(Enum):
    """USB 3.x line-rate selector (PIPE ``Rate`` encoding for USB).

    ====  =========  ============
    Name  Value      Line Rate
    ====  =========  ============
    GEN1  0          5 GT/s
    GEN2  1          10 GT/s
    ====  =========  ============
    """

    GEN1 = 0  # 5 GT/s  (USB 3.1 Gen1 / USB 3.0)
    GEN2 = 1  # 10 GT/s (USB 3.1 Gen2)


class SATARate(Enum):
    """SATA line-rate selector (reserved — not yet wired to hardware).

    ====  =========  ============
    Name  Value      Line Rate
    ====  =========  ============
    GEN1  0          1.5 GT/s
    GEN2  1          3.0 GT/s
    GEN3  2          6.0 GT/s
    ====  =========  ============
    """

    GEN1 = 0  # 1.5 GT/s
    GEN2 = 1  # 3.0 GT/s
    GEN3 = 2  # 6.0 GT/s


# ═══════════════════════════════════════════════════════════════════════════
#  Width configuration map
# ═══════════════════════════════════════════════════════════════════════════

# Mapping from ``PIPEWidth`` to a 5-tuple:
#   (pcs_width, gear_rate, fabric_bits, symbols_per_cycle, symbol_clean)
#
# - **pcs_width**:         GTR12 PCS interface width (10, 16, or 20 bits)
# - **gear_rate**:         GTR12 gearing ratio (1:1, 1:2, or 1:4)
# - **fabric_bits**:       Total data bits seen by the FPGA fabric per PCLK
# - **symbols_per_cycle**: Number of 10-bit symbols transported per PCLK
# - **symbol_clean**:      True when ``fabric_bits`` is an exact multiple of
#                          10 (the 8b10b symbol size).  When False, symbol
#                          boundaries do not align to the bus width and the
#                          elastic buffer / SKP logic must handle partial
#                          symbols.

PIPE_WIDTH_MAP = {
    PIPEWidth.W10: (10, GearRate.G1_1, 10, 1, True),
    PIPEWidth.W20: (10, GearRate.G1_2, 20, 2, True),
    PIPEWidth.W40: (20, GearRate.G1_2, 40, 4, True),
    PIPEWidth.W32: (16, GearRate.G1_2, 32, 3, False),
    PIPEWidth.W64: (16, GearRate.G1_4, 64, 6, False),
    PIPEWidth.W80: (20, GearRate.G1_4, 80, 8, True),
}


# ═══════════════════════════════════════════════════════════════════════════
#  USB Hardware Map
# ═══════════════════════════════════════════════════════════════════════════

# Mapping from ``(USBRate, PIPEWidth)`` to a 4-tuple:
#   (tx_data_rate, pcs_width, gear_rate, encode_mode)
#
# - **tx_data_rate**: Gowin SerDes string rate selector ("5G" or "10G")
# - **pcs_width**:    PCS interface width passed to ``LaneConfig.width_mode``
# - **gear_rate**:    Gearing ratio passed to ``LaneConfig.tx/rx_gear_rate``
# - **encode_mode**:  ``EncodingMode.OFF`` — 8b10b is handled in fabric by
#                     the PIPE adapter, not inside the GTR12 hard PCS.
#
# Not every (rate, width) combination is valid.  Attempting to use an
# unlisted pair will raise ``ValueError`` in ``PIPELaneConfig.to_lane_config``.

PIPE_USB_HW_MAP = {
    (USBRate.GEN1, PIPEWidth.W10): ("5G", 10, GearRate.G1_1, EncodingMode.OFF),
    (USBRate.GEN1, PIPEWidth.W20): ("5G", 10, GearRate.G1_2, EncodingMode.OFF),
    (USBRate.GEN1, PIPEWidth.W32): ("5G", 16, GearRate.G1_2, EncodingMode.OFF),
    (USBRate.GEN1, PIPEWidth.W40): ("5G", 20, GearRate.G1_2, EncodingMode.OFF),
    (USBRate.GEN1, PIPEWidth.W64): ("5G", 16, GearRate.G1_4, EncodingMode.OFF),
    (USBRate.GEN1, PIPEWidth.W80): ("5G", 20, GearRate.G1_4, EncodingMode.OFF),
    (USBRate.GEN2, PIPEWidth.W32): ("10G", 8, GearRate.G1_4, EncodingMode.OFF),
    (USBRate.GEN2, PIPEWidth.W40): ("10G", 10, GearRate.G1_4, EncodingMode.OFF),
    (USBRate.GEN2, PIPEWidth.W64): ("10G", 16, GearRate.G1_4, EncodingMode.OFF),
    (USBRate.GEN2, PIPEWidth.W80): ("10G", 20, GearRate.G1_4, EncodingMode.OFF),
}


# ═══════════════════════════════════════════════════════════════════════════
#  CSR Register Tables — GTR12 DRP addresses and magic values
# ═══════════════════════════════════════════════════════════════════════════
#
# All address computation and magic-value definitions are now in
# ``gowin_serdes.csr_map`` (the single source of truth).  The module-level
# ``CSR_*`` constants below are **backward-compat aliases for quad=0,
# lane=0 only**.  New code should use ``csr_addr(CSR.XXX, quad, lane)``
# or the ``rate_change_regs`` / ``lfps_ffe_regs`` functions directly.
# ---------------------------------------------------------------------------

_DEFAULT_ADDRS = runtime_addrs(quad=0, lane=0)

# ---------------------------------------------------------------------------
# Backward-compat convenience aliases (quad=0, lane=0)
# ---------------------------------------------------------------------------
CSR_EIDLE_ADDR = csr_addr(CSR.EIDLE, 0, 0)
# CSR_EIDLE_ON / CSR_EIDLE_OFF imported above from csr_map

CSR_RXDET_PULSE_ADDR = csr_addr(CSR.RXDET_PULSE, 0, 0)
# CSR_RXDET_PULSE_START / CSR_RXDET_PULSE_END imported above from csr_map
CSR_RXDET_RESULT_ADDR = csr_addr(CSR.RXDET_RESULT, 0, 0)

CSR_RX_POLARITY_ADDR = csr_addr(CSR.RX_POLARITY, 0, 0)
# CSR_RX_POLARITY_NORMAL / CSR_RX_POLARITY_INVERT imported above from csr_map

CSR_8B10B_BYPASS_ADDR = csr_addr(CSR.PCS_8B10B, 0, 0)
# CSR_8B10B_BYPASS_VAL imported above from csr_map

# ---------------------------------------------------------------------------
# Module-level register tables (quad=0, lane=0 defaults, backward compat)
# ---------------------------------------------------------------------------
RATE_CHANGE_REGS: list[tuple[int, int, int, str]] = rate_change_regs(0, 0)
LFPS_FFE_REGS: list[tuple[int, int, int, str]] = lfps_ffe_regs(0, 0)


# ═══════════════════════════════════════════════════════════════════════════
#  PCLK Rate Calculator
# ═══════════════════════════════════════════════════════════════════════════


def pclk_mhz(line_rate_gbps: float, fabric_width: int) -> float:
    """Calculate PCLK frequency in MHz for a given line rate and fabric width.

    The parallel clock (PCLK) that drives the PIPE interface runs at::

        PCLK = line_rate / fabric_width

    Parameters
    ----------
    line_rate_gbps : float
        SerDes line rate in gigabits per second (e.g. 5.0 for USB Gen1).
    fabric_width : int
        Number of fabric-side data bits per PCLK cycle (from
        ``PIPE_WIDTH_MAP[width][2]``).

    Returns
    -------
    float
        PCLK frequency in MHz.

    Examples
    --------
    >>> pclk_mhz(5.0, 40)
    125.0
    >>> pclk_mhz(10.0, 40)
    250.0
    >>> pclk_mhz(5.0, 20)
    250.0
    """
    return (line_rate_gbps * 1e3) / fabric_width


# ═══════════════════════════════════════════════════════════════════════════
#  PIPELaneConfig — top-level per-lane configuration dataclass
# ═══════════════════════════════════════════════════════════════════════════


@dataclass
class PIPELaneConfig:
    """Per-lane PIPE configuration.

    This is the primary configuration object consumed by every pipe_serdes
    module (the PIPE top-level, TX/RX data-paths, power management, rate
    change, LFPS, electrical idle, and receiver detection sub-blocks).

    Attributes
    ----------
    protocol : PIPEProtocol
        Which protocol family this lane serves.  Currently only
        ``PIPEProtocol.USB3`` is fully supported.
    quad : int
        0-based GTR12 quad index.  Used to compute CSR addresses.
    lane : int
        0-based lane index within the quad.  Used to compute CSR addresses.
    supported_rates : list[USBRate]
        Ordered list of line-rates the lane must support.  The first entry
        is the initial / default rate.
    default_width : PIPEWidth
        Default TX data-path width.  Also used for RX unless
        ``default_rx_width`` is set.
    default_rx_width : Optional[PIPEWidth]
        If set, allows asymmetric TX/RX widths (e.g. wider RX for SKP
        elasticity).  Defaults to ``default_width``.
    enable_msg_bus : bool
        When True the lane instantiates a DRP message-bus arbiter for CSR
        access.  Should be True for any lane that needs runtime CSR writes
        (rate change, LFPS FFE, electrical idle, RxDet, polarity).
    enable_mac_clk : bool
        When True the MAC-side clock domain (PCLK) is generated and
        active.  Set to False only for loopback / BIST configurations
        that do not use an external MAC.
    """

    protocol: PIPEProtocol = PIPEProtocol.USB3
    device: GowinDevice = GowinDevice.GW5AST_138
    quad: int = 0
    lane: int = 0
    supported_rates: list = field(default_factory=lambda: [USBRate.GEN1])
    default_width: PIPEWidth = PIPEWidth.W40
    default_rx_width: Optional[PIPEWidth] = None  # defaults to default_width
    enable_msg_bus: bool = True
    enable_mac_clk: bool = True

    # ── CSR address properties ─────────────────────────────────────────

    @property
    def csr_addrs(self) -> dict[str, int]:
        """CSR address map for this lane's quad/lane indices."""
        return runtime_addrs(self.quad, self.lane)

    @property
    def lane_rate_change_regs(self) -> list[tuple[int, int, int, str]]:
        """Rate-change register table for this lane."""
        return rate_change_regs(self.quad, self.lane)

    @property
    def lane_lfps_ffe_regs(self) -> list[tuple[int, int, int, str]]:
        """LFPS FFE register table for this lane."""
        return lfps_ffe_regs(self.quad, self.lane)

    # ── Derived properties ─────────────────────────────────────────────

    @property
    def max_data_width(self) -> int:
        """Maximum fabric data width (bits) based on ``default_width``.

        This equals the ``fabric_bits`` entry in ``PIPE_WIDTH_MAP`` and is
        used to size ``TxData``, ``RxData``, and elastic-buffer RAM.
        """
        return PIPE_WIDTH_MAP[self.default_width][2]  # fabric_bits

    @property
    def max_rate(self) -> USBRate:
        """Highest supported USB rate (by enum value)."""
        return max(self.supported_rates, key=lambda r: r.value)

    # ── Conversion to low-level LaneConfig ─────────────────────────────

    def to_lane_config(self, rate=None, width=None):
        """Generate an underlying ``gowin_serdes.config.LaneConfig``.

        Translates PIPE-level (rate, width) into the concrete Gowin SerDes
        parameters needed by the GTR12 wrapper.

        Parameters
        ----------
        rate : USBRate, optional
            Line rate to configure.  Defaults to the first entry in
            ``supported_rates``.
        width : PIPEWidth, optional
            Data-path width to configure.  Defaults to ``default_width``.

        Returns
        -------
        LaneConfig
            Fully populated ``gowin_serdes.config.LaneConfig`` ready for
            instantiation.

        Raises
        ------
        ValueError
            If the requested ``(rate, width)`` pair is not present in
            ``PIPE_USB_HW_MAP``.
        """
        from gowin_serdes.config import LaneConfig, OperationMode

        r = rate or self.supported_rates[0]
        w = width or self.default_width
        key = (r, w)
        if key not in PIPE_USB_HW_MAP:
            raise ValueError(f"Unsupported (rate={r}, width={w}) combination")
        data_rate_str, pcs_width, gear, enc = PIPE_USB_HW_MAP[key]
        return LaneConfig(
            operation_mode=OperationMode.TX_RX,
            tx_data_rate=data_rate_str,
            rx_data_rate=data_rate_str,
            tx_gear_rate=gear,
            rx_gear_rate=gear,
            width_mode=pcs_width,
            tx_encoding=enc,
            rx_encoding=enc,
        )


# ═══════════════════════════════════════════════════════════════════════════
#  DRP Client IDs (priority-encoded arbiter slots)
# ═══════════════════════════════════════════════════════════════════════════


class DRPClientID(Enum):
    """DRP message-bus client identifiers.

    The internal DRP arbiter uses fixed-priority encoding.  Lower numeric
    values have **higher** priority so that time-critical operations
    (rate change, LFPS FFE) are never starved by background CSR traffic.

    The init FSM occupies slot 0 (highest priority) because it runs once
    at power-on and must complete before any other DRP client is active.

    ============  =====  ========================================
    Client        ID     Purpose
    ============  =====  ========================================
    INIT          0      Power-on init sequence (highest priority)
    RATE          1      Rate-change register sequence
    LFPS          2      LFPS FFE tap reconfiguration
    RXDET         3      Receiver detection pulse control
    POWER         4      Power-state CSR writes
    EIDLE         5      Electrical idle control
    CSR_BRIDGE    6      Generic MAC/software CSR bridge (lowest)
    ============  =====  ========================================
    """

    INIT = 0  # Highest priority (power-on only)
    RATE = 1
    LFPS = 2
    RXDET = 3
    POWER = 4
    EIDLE = 5
    CSR_BRIDGE = 6  # Lowest priority

    NUM_CLIENTS = 7


# ═══════════════════════════════════════════════════════════════════════════
#  Receiver Detection Timing
# ═══════════════════════════════════════════════════════════════════════════

#: Number of PCLK cycles to hold the RxDet pulse high before de-asserting.
#: This value satisfies the GTR12 minimum pulse width requirement across all
#: supported PCLK frequencies (125–250 MHz → 1.0–2.0 µs pulse).
RXDET_PULSE_WAIT_CYCLES: int = 250


# ═══════════════════════════════════════════════════════════════════════════
#  PIPE Power State Encoding
# ═══════════════════════════════════════════════════════════════════════════


class PIPEPowerState(Enum):
    """PIPE power-state encoding (``PowerDown[3:0]`` field).

    Defined by PIPE 7.1 §5.7.  Only P0–P3 are used for USB 3.x.

    ====  ======  ============================================
    Name  Value   Description
    ====  ======  ============================================
    P0    0000    Normal operation — link active
    P1    0001    Low-latency recovery — clocks running
    P2    0010    Low-power — clocks may be stopped
    P3    0011    Lowest power — may require re-training
    ====  ======  ============================================
    """

    P0 = 0b0000
    P1 = 0b0001
    P2 = 0b0010
    P3 = 0b0011


# ═══════════════════════════════════════════════════════════════════════════
#  PIPE RxStatus Encoding
# ═══════════════════════════════════════════════════════════════════════════


class PIPERxStatus(Enum):
    """PIPE ``RxStatus[2:0]`` encoding.

    Defined by PIPE 7.1 §5.10.  The PHY drives this field to report
    per-symbol receive status to the MAC.

    ===============  =====  ==========================================
    Name             Value  Meaning
    ===============  =====  ==========================================
    OK               000    Received data OK
    SKP_ADDED        001    SKP ordered-set added by elastic buffer
    SKP_REMOVED      010    SKP ordered-set removed by elastic buffer
    RX_DETECTED      011    Receiver detected on lane
    DECODE_ERROR     100    8b10b decode error
    EB_OVERFLOW      101    Elastic buffer overflow
    EB_UNDERFLOW     110    Elastic buffer underflow
    DISPARITY_ERROR  111    8b10b running disparity error
    ===============  =====  ==========================================
    """

    OK = 0b000
    SKP_ADDED = 0b001
    SKP_REMOVED = 0b010
    RX_DETECTED = 0b011
    DECODE_ERROR = 0b100
    EB_OVERFLOW = 0b101
    EB_UNDERFLOW = 0b110
    DISPARITY_ERROR = 0b111
