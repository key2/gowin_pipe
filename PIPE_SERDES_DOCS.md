# PIPE SerDes — Comprehensive Documentation

> PIPE Rev 7.1 SerDes PHY for Gowin GTR12 Transceivers

---

## Table of Contents

1. [Overview](#1-overview)
2. [Quick Start](#2-quick-start)
3. [Configuration](#3-configuration)
4. [Architecture](#4-architecture)
5. [PIPE Interface Signals](#5-pipe-interface-signals)
6. [Module Reference](#6-module-reference)
7. [DRP Serialization and Race Prevention](#7-drp-serialization-and-race-prevention)
8. [Testing](#8-testing)
9. [Custom Width Extension](#9-custom-width-extension)
10. [Limitations and Open Questions](#10-limitations-and-open-questions)

---

## 1. Overview

### What the Library Does

`pipe_serdes` is an Amaranth HDL library that implements a **PIPE Rev 7.1 low-pin-count PHY interface** on top of the Gowin GTR12 SerDes transceiver hard macro. It bridges the gap between a MAC layer (e.g., a USB 3.x XHCI controller) and the physical transceiver by providing:

- Power state management (P0–P3)
- Rate switching (Gen1 5 GT/s ↔ Gen2 10 GT/s)
- TX/RX data path width adaptation
- Receiver detection
- LFPS (Low-Frequency Periodic Signaling)
- PIPE 8-bit message bus protocol engine
- CSR bridge with shadow registers
- MacCLK domain generation for low-power states
- DRP (Dynamic Reconfiguration Port) arbitration with locking

### Target Hardware

| Parameter | Value |
|-----------|-------|
| **FPGA Family** | Any Gowin FPGA with a GTR SerDes transceiver |
| **Transceiver** | Gowin GTR (quad-based, up to 4 lanes per quad) |
| **Reference Guide** | Gowin IPUG1024E |

### Supported Protocols

| Protocol | Status | Line Rates |
|----------|--------|------------|
| **USB 3.1 Gen1** | Fully implemented | 5 GT/s |
| **USB 3.1 Gen2** | Fully implemented | 10 GT/s |
| **SATA** | Reserved (enum placeholder) | 1.5 / 3.0 / 6.0 GT/s |
| **DisplayPort (eDP)** | Reserved (enum placeholder) | — |

### PIPE Rev 7.1 Compliance Level

The implementation targets the **PIPE Rev 7.1** specification with the following coverage:

- **§5.7** Power states P0–P3 (fully implemented)
- **§5.10** RxStatus encoding (fully implemented)
- **§6.1.4** Message bus protocol (M2P decode, P2M encode, atomic writes)
- **§8.3** Power state transitions (all P0↔P1/P2/P3 paths)
- **§8.4** Rate change procedure (7-register atomic DRP sequence)
- **§5.6** MacCLK domain (handshake protocol)

### Architecture: SerDes (Not Legacy)

This is the **SerDes architecture** variant of the PIPE interface. It does NOT use the legacy/original PIPE pin-level interface. Instead, the PHY presents typed Amaranth `Signature` ports and communicates with the GTR12 hard macro through an internal DRP-based CSR abstraction layer.

---

## 2. Quick Start

### Minimal Instantiation

```python
from pipe_serdes import PIPESerDes, PIPELaneConfig, PIPEWidth, USBRate, PIPEProtocol

# Configure a USB 3.1 Gen1+Gen2 lane with 40-bit data width
config = PIPELaneConfig(
    protocol=PIPEProtocol.USB3,
    supported_rates=[USBRate.GEN1, USBRate.GEN2],
    default_width=PIPEWidth.W40,
)

# Instantiate the PHY on Quad 0, Lane 0
phy = PIPESerDes(config, quad=0, lane=0)
```

### Wiring in an Amaranth `elaborate()` Method

```python
def elaborate(self, platform):
    m = Module()
    m.submodules.phy = phy

    # TX data path
    m.d.comb += [
        phy.pipe.tx_data.eq(mac_tx_data),
        phy.pipe.tx_data_valid.eq(mac_tx_valid),
        phy.pipe.tx_elec_idle.eq(mac_tx_eidle),
    ]

    # RX data path
    m.d.comb += [
        mac_rx_data.eq(phy.pipe.rx_data),
        mac_rx_valid.eq(phy.pipe.rx_valid),
    ]

    # Control
    m.d.comb += [
        phy.pipe.reset_n.eq(1),
        phy.pipe.power_down.eq(0b0000),   # P0 = active
        phy.pipe.rate.eq(0b0000),          # Gen1
        phy.pipe.width.eq(0b010),          # 40-bit
    ]

    # Debug LEDs
    m.d.comb += [
        led_pll.eq(phy.debug.pll_lock),
        led_cdr.eq(phy.debug.cdr_lock),
    ]

    return m
```

### USB Gen1-Only Configuration

```python
config = PIPELaneConfig(
    protocol=PIPEProtocol.USB3,
    supported_rates=[USBRate.GEN1],
    default_width=PIPEWidth.W20,
)
phy = PIPESerDes(config, quad=0, lane=0)
```

### Maximum Throughput Configuration (80-bit, Gen2)

```python
config = PIPELaneConfig(
    protocol=PIPEProtocol.USB3,
    supported_rates=[USBRate.GEN1, USBRate.GEN2],
    default_width=PIPEWidth.W80,
)
phy = PIPESerDes(config, quad=0, lane=0)
```

---

## 3. Configuration

### PIPELaneConfig Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `protocol` | `PIPEProtocol` | `USB3` | Protocol family. Only `USB3` is fully implemented. |
| `supported_rates` | `list[USBRate]` | `[GEN1]` | Ordered list of line rates. First entry = initial rate. |
| `default_width` | `PIPEWidth` | `W40` | Default TX data path width. Also used for RX unless `default_rx_width` is set. |
| `default_rx_width` | `PIPEWidth \| None` | `None` | Asymmetric RX width override. Defaults to `default_width`. |
| `enable_msg_bus` | `bool` | `True` | Enable DRP message bus arbiter for runtime CSR access. |
| `enable_mac_clk` | `bool` | `True` | Enable MAC-side clock domain generation. |

#### Derived Properties

| Property | Returns | Description |
|----------|---------|-------------|
| `max_data_width` | `int` | Maximum fabric data width in bits (from `PIPE_WIDTH_MAP`) |
| `max_rate` | `USBRate` | Highest supported rate by enum value |

### Width Encoding Table

The PIPE `Width[2:0]` field maps to physical fabric widths through `PIPE_WIDTH_MAP`:

| `PIPEWidth` | Width[2:0] | PCS Width | Gear Rate | Fabric Bits | Symbols/Cycle | Symbol Clean |
|-------------|-----------|-----------|-----------|-------------|---------------|-------------|
| `W10` | 0 | 10 | 1:1 | 10 | 1 | Yes |
| `W20` | 1 | 10 | 1:2 | 20 | 2 | Yes |
| `W40` | 2 | 20 | 1:2 | 40 | 4 | Yes |
| `W32` | 3 | 16 | 1:2 | 32 | 3 | **No** |
| `W64` | 4 | 16 | 1:4 | 64 | 6 | **No** |
| `W80` | 5 | 20 | 1:4 | 80 | 8 | Yes |

**Symbol Clean**: When `True`, `fabric_bits` is an exact multiple of 10 (the 8b10b symbol size). Non-clean widths (W32, W64) require special elastic buffer/SKP handling for partial symbols.

### USB Hardware Map (Rate × Width → GTR12 Config)

| Rate | Width | TX Rate | PCS Width | Gear | Encoding |
|------|-------|---------|-----------|------|----------|
| Gen1 | W10 | 5G | 10 | 1:1 | OFF |
| Gen1 | W20 | 5G | 10 | 1:2 | OFF |
| Gen1 | W32 | 5G | 16 | 1:2 | OFF |
| Gen1 | W40 | 5G | 20 | 1:2 | OFF |
| Gen1 | W64 | 5G | 16 | 1:4 | OFF |
| Gen1 | W80 | 5G | 20 | 1:4 | OFF |
| Gen2 | W32 | 10G | 8 | 1:4 | OFF |
| Gen2 | W40 | 10G | 10 | 1:4 | OFF |
| Gen2 | W64 | 10G | 16 | 1:4 | OFF |
| Gen2 | W80 | 10G | 20 | 1:4 | OFF |

> **Note:** Encoding is always `OFF` — 8b10b encode/decode is performed in fabric by the PIPE adapter, not inside the GTR12 hard PCS.

> **Note:** Gen2 does not support W10 or W20. Attempting to use an unlisted (rate, width) pair raises `ValueError`.

### PCLK Frequency Table

PCLK = line_rate / fabric_width:

| Rate | Width | Fabric Bits | PCLK (MHz) |
|------|-------|-------------|------------|
| Gen1 (5 Gbps) | W10 | 10 | 500.0 |
| Gen1 (5 Gbps) | W20 | 20 | 250.0 |
| Gen1 (5 Gbps) | W32 | 32 | 156.25 |
| Gen1 (5 Gbps) | W40 | 40 | 125.0 |
| Gen1 (5 Gbps) | W64 | 64 | 78.125 |
| Gen1 (5 Gbps) | W80 | 80 | 62.5 |
| Gen2 (10 Gbps) | W32 | 32 | 312.5 |
| Gen2 (10 Gbps) | W40 | 40 | 250.0 |
| Gen2 (10 Gbps) | W64 | 64 | 156.25 |
| Gen2 (10 Gbps) | W80 | 80 | 125.0 |

### Recommended Configurations

| Use Case | Width | Rationale |
|----------|-------|-----------|
| **USB 3.1 Gen1 only** | `W20` or `W40` | Standard PIPE widths, clean symbol boundaries |
| **USB 3.1 Gen1+Gen2** | `W40` | 125 MHz Gen1 / 250 MHz Gen2, manageable timing |
| **Low PCLK frequency** | `W80` | 62.5 MHz Gen1 / 125 MHz Gen2, easiest timing closure |
| **BRAM-efficient** | `W32` | Non-clean but smaller buffers (caution: partial symbol handling) |

---

## 4. Architecture

### Block Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│  PIPESerDes (pipe_serdes.py) — User-Facing Entry Point           │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  PIPESerDesAdapter (pipe_adapter.py) — Main Wiring Hub     │  │
│  │                                                            │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐ │  │
│  │  │ PIPETXPath   │  │ PIPEPowerFSM │  │ PIPERateCtrl     │ │  │
│  │  │ (Width adapt)│  │ (P0-P3 FSM)  │  │ (Gen1↔Gen2)      │ │  │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────────┘ │  │
│  │  ┌──────┴───────┐  ┌──────┴───────┐  ┌──────┴───────────┐ │  │
│  │  │ PIPERXPath   │  │ PIPERxDet    │  │ PIPELFPSCtrl     │ │  │
│  │  │ (Width adapt)│  │ (Detect seq) │  │ (FFE+EIdle)      │ │  │
│  │  └──────────────┘  └──────────────┘  └──────────────────┘ │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐ │  │
│  │  │ PIPEMsgBus   │  │ PIPEMacCLK   │  │ PIPECSRBridge    │ │  │
│  │  │ (M2P/P2M)    │  │ (Clock gate) │  │ (Shadow regs)    │ │  │
│  │  └──────┬───────┘  └──────────────┘  └──────┬───────────┘ │  │
│  │         │                                    │             │  │
│  │  ┌──────┴────────────────────────────────────┴───────────┐ │  │
│  │  │  PIPEDRPMux (6-client priority arbiter with locking)  │ │  │
│  │  └────────────────────────┬──────────────────────────────┘ │  │
│  └───────────────────────────┼────────────────────────────────┘  │
│                              │ DRP Port                          │
│  ┌───────────────────────────┴──────────────────────────────┐   │
│  │  GowinSerDes (GTR12 Hard Macro)                          │   │
│  │   └── GowinSerDesLane (TX/RX/Status/Reset)               │   │
│  └──────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

### Clock Domains

| Clock Domain | Source | Frequency | Usage |
|-------------|--------|-----------|-------|
| **PCLK** | TX PCS parallel clock (`tx.pcs_clkout`) | 62.5–500 MHz (depends on rate+width) | PIPE interface, all sub-controller FSMs |
| **RxCLK** | CDR recovered clock (`rx.pcs_clkout`) | Byte-rate (varies) | RX data output; MAC must handle CDC |
| **UPAR (DRP)** | Internal to GTR12 | ~100 MHz | DRP CSR transactions |
| **MacCLK** | Gated LIFE_CLK | ~62.5 MHz | MAC-PHY communication during P2/P3 |
| **LIFE_CLK** | GTR12 free-running reference | ~62.5 MHz | Always-on reference for MacCLK |

### Data Flow

**TX Path (MAC → SerDes):**
```
MAC → pipe.tx_data[N-1:0] → PIPETXPath (width adapt) → lane.tx.data[79:0] → GTR12 QUAD
                                  ↑
                         pipe.width[2:0] selects active width
```

**RX Path (SerDes → MAC):**
```
GTR12 QUAD → lane.rx.data[87:0] → PIPERXPath (width extract) → pipe.rx_data[N-1:0] → MAC
                                        ↑
                               pipe.rx_width[2:0] selects active width
```

> **Note:** `rx_data[87:80]` from the GTR12 are status bits, NOT symbol data. Only the lower N bits carry received data.

---

## 5. PIPE Interface Signals

All directions are from the **PHY's perspective** (`In` = driven by MAC, `Out` = driven by PHY).

### Data Interface

| Signal | Width | Direction | Description |
|--------|-------|-----------|-------------|
| `tx_data` | data_width | In | TX data from MAC for serialization |
| `tx_data_valid` | 1 | In | TX data qualifier (MAC asserts when tx_data is meaningful) |
| `rx_data` | data_width | Out | RX data from PHY after deserialization |

### Clock Interface

| Signal | Width | Direction | Description |
|--------|-------|-----------|-------------|
| `pclk` | 1 | In | MAC-supplied parallel clock |
| `max_pclk` | 1 | Out | PHY-generated PCLK at maximum supported rate (from `tx.pcs_clkout`) |
| `rx_clk` | 1 | Out | CDR-recovered byte-rate clock (from `rx.pcs_clkout`) |

### Command Interface (MAC → PHY)

| Signal | Width | Direction | Description |
|--------|-------|-----------|-------------|
| `phy_mode` | 4 | In | Protocol selector: 0=default, 1=USB, 2=SATA, 3=DP |
| `reset_n` | 1 | In | Active-low asynchronous reset |
| `power_down` | 4 | In | Power state request (P0=0000, P1=0001, P2=0010, P3=0011) |
| `rate` | 4 | In | Signaling rate selector (0=Gen1, 1=Gen2) |
| `width` | 3 | In | TX data path width selector (see Width Encoding Table) |
| `rx_width` | 3 | In | RX data path width selector (independent of TX) |
| `tx_elec_idle` | 4 | In | TX electrical idle per sub-lane |
| `tx_detect_rx_loopback` | 1 | In | Triggers receiver detection or loopback entry |
| `rx_polarity` | 1 | In | [USB] RX polarity inversion (swap D+/D−) |
| `rx_termination` | 1 | In | [USB] RX termination control (enable/disable 45 Ω) |
| `rx_standby` | 1 | In | [USB/SATA] RX low-power standby |
| `pclk_change_ack` | 1 | In | [USB/SATA] MAC acknowledges PHY-initiated PCLK rate change |

### Status Interface (PHY → MAC)

| Signal | Width | Direction | Description |
|--------|-------|-----------|-------------|
| `phy_status` | 1 | Out | Single-cycle completion pulse for async PHY operations |
| `rx_valid` | 1 | Out | CDR locked and recovered clock stable |
| `rx_elec_idle` | 1 | Out | Asynchronous electrical-idle detect on RX pair |
| `rx_status` | 3 | Out | [USB/SATA] Encoded RX status (see RxStatus table) |
| `pclk_change_ok` | 1 | Out | [USB/SATA] PHY ready for PCLK frequency change |

#### RxStatus Encoding

| Value | Name | Meaning |
|-------|------|---------|
| 000 | OK | Received data OK |
| 001 | SKP_ADDED | SKP ordered-set added by elastic buffer |
| 010 | SKP_REMOVED | SKP ordered-set removed by elastic buffer |
| 011 | RX_DETECTED | Receiver detected on lane |
| 100 | DECODE_ERROR | 8b10b decode error |
| 101 | EB_OVERFLOW | Elastic buffer overflow |
| 110 | EB_UNDERFLOW | Elastic buffer underflow |
| 111 | DISPARITY_ERROR | 8b10b running disparity error |

### Message Bus

| Signal | Width | Direction | Description |
|--------|-------|-----------|-------------|
| `m2p_msg_bus` | 8 | In | MAC-to-PHY 8-bit encoded messages |
| `p2m_msg_bus` | 8 | Out | PHY-to-MAC 8-bit encoded messages |

### MacCLK Domain

| Signal | Width | Direction | Description |
|--------|-------|-----------|-------------|
| `mac_clk_reset_n` | 1 | In | Active-low async reset for MacCLK domain |
| `mac_clk_rate` | 5 | In | MacCLK rate selector (reserved; currently unused) |
| `mac_clk_req` | 1 | In | MAC requests MacCLK output to be active |
| `mac_clk_ack` | 1 | Out | PHY acknowledges and delivers MacCLK |
| `mac_clk` | 1 | Out | MacCLK output (~62.5 MHz gated LIFE_CLK) |

### Debug Interface

| Signal | Width | Description |
|--------|-------|-------------|
| `power_state` | 4 | Current PIPE power state (P0–P3) |
| `rate_fsm_state` | 4 | Rate-change FSM state |
| `rxdet_fsm_state` | 4 | Receiver-detect FSM state |
| `lfps_fsm_state` | 4 | LFPS FSM state |
| `msgbus_fsm_state` | 4 | Message bus FSM state |
| `drp_mux_owner` | 4 | DRP arbiter current owner |
| `drp_mux_locked` | 1 | DRP bus locked by current owner |
| `drp_busy` | 1 | DRP transaction in progress |
| `pll_lock` | 1 | PLL lock indicator |
| `cdr_lock` | 1 | CDR lock indicator |

---

## 6. Module Reference

### 6.1 `pipe_config` — Configuration Foundation

**File:** `pipe_serdes/pipe_config.py`

The foundational module consumed by all other `pipe_serdes` modules.

#### Enumerations

| Enum | Values | Description |
|------|--------|-------------|
| `PIPEProtocol` | `USB3=1`, `SATA=2`, `DP=3` | Protocol family selector |
| `PIPEWidth` | `W10=0` through `W80=5` | Data path width selector |
| `USBRate` | `GEN1=0` (5 GT/s), `GEN2=1` (10 GT/s) | USB line rate |
| `SATARate` | `GEN1=0`, `GEN2=1`, `GEN3=2` | SATA line rate (reserved) |
| `PIPEPowerState` | `P0=0000` through `P3=0011` | Power state encoding |
| `PIPERxStatus` | `OK=000` through `DISPARITY_ERROR=111` | RX status encoding |
| `DRPClientID` | `RATE=0` through `CSR_BRIDGE=5`, `NUM_CLIENTS=6` | DRP arbiter priority slots |

#### Maps

- **`PIPE_WIDTH_MAP`**: Maps `PIPEWidth` → `(pcs_width, gear_rate, fabric_bits, symbols_per_cycle, symbol_clean)`
- **`PIPE_USB_HW_MAP`**: Maps `(USBRate, PIPEWidth)` → `(tx_data_rate, pcs_width, gear_rate, encode_mode)`

#### CSR Register Tables

| Category | Address | Values | Description |
|----------|---------|--------|-------------|
| **Electrical Idle** | `0x8003A4` | ON=`0x01`, OFF=`0x07` | TX driver idle control |
| **RxDet Pulse** | `0x80033F` | Start=`0x03000000`, End=`0x00000000` | Receiver detection pulse control |
| **RxDet Result** | `0x808B34` | Bit [1] = detected | Receiver detection result |
| **RX Polarity** | `0x809008` | Normal=`0x08`, Invert=`0x88` | RX polarity inversion |
| **8b10b Bypass** | `0x809068` | `0x000001FF` | Always bypassed (fabric 8b10b) |

#### Rate Change Registers (`RATE_CHANGE_REGS`)

| # | Address | Gen1 Value | Gen2 Value | Description |
|---|---------|------------|------------|-------------|
| 0 | `0x80A020` | `0x0000001A` | `0x00000014` | CPLL divider ratio |
| 1 | `0x8082A0` | `0x00003150` | `0x00001170` | RX AFE gain/attenuation |
| 2 | `0x8082B8` | `0x00000020` | `0x00000040` | RX AFE bias current |
| 3 | `0x8082C0` | `0x00000210` | `0x00000310` | RX AFE boost |
| 4 | `0x808600` | `0x0000011A` | `0x0000021A` | TX clock source select |
| 5 | `0x808620` | `0x00000016` | `0x00000026` | RX clock source select |
| 6 | `0x809000` | `0x00000001` | `0x00000003` | PCS rate mode |

#### LFPS FFE Registers (`LFPS_FFE_REGS`)

| # | Address | Normal Value | LFPS Value | Description |
|---|---------|-------------|------------|-------------|
| 0 | `0x808234` | `0x0000F000` | `0x00000040` | TX FFE_0 |
| 1 | `0x808238` | `0x00000000` | `0x00000001` | TX FFE_1 |
| 2 | `0x8082D8` | `0x00000110` | `0x00000003` | TX FFE_2 |

#### Functions

- **`pclk_mhz(line_rate_gbps, fabric_width)`** — Returns PCLK frequency in MHz: `(line_rate × 1000) / fabric_width`

---

### 6.2 `pipe_signature` — Interface Definitions

**File:** `pipe_serdes/pipe_signature.py`

Defines Amaranth `Signature` classes for typed port interfaces. Directions are from the PHY's perspective.

#### `DRPRequestSignature`

Internal DRP request port for sub-controllers. Each sub-controller presents this to the central DRP arbiter.

| Signal | Width | Direction | Description |
|--------|-------|-----------|-------------|
| `addr` | 24 | Out | CSR address (24-bit reconfiguration space) |
| `wrdata` | 32 | Out | Write data |
| `wren` | 1 | Out | Write enable |
| `rden` | 1 | Out | Read enable |
| `lock_req` | 1 | Out | Request exclusive DRP bus lock |
| `lock_ack` | 1 | In | Lock granted by arbiter |
| `ready` | 1 | In | Write accepted / arbiter ready |
| `rdvld` | 1 | In | Read data valid strobe |
| `rddata` | 32 | In | Read data returned from PHY CSR |

#### `PIPESerDesSignature`

Complete PIPE Rev 7.1 low-pin-count interface. Constructor parameters:

- `data_width` (int): Maximum data bus width (10/20/32/40/64/80)
- `usb` (bool): Include USB-specific signals (`rx_polarity`, `rx_termination`)
- `sata` (bool): Include SATA-specific signals
- `dp` (bool): Include DisplayPort signals (reserved)

#### `PIPEDebugSignature`

Read-only debug/status outputs for ILA probes, LEDs, or debug register files.

---

### 6.3 `pipe_adapter` — Main Wiring Hub

**File:** `pipe_serdes/pipe_adapter.py`

`PIPESerDesAdapter` is the top-level wiring component that assembles all sub-controllers and wires them to both the PIPE interface (MAC side) and the GowinSerDesLane (hardware side).

#### Interface Ports

| Port | Signature | Description |
|------|-----------|-------------|
| `pipe` | `PIPESerDesSignature` | PIPE interface (MAC side) |
| `drp` | `DRPOutputSignature` | External DRP port → GowinSerDes |
| `lane` | `LanePortSignature` | Hardware lane (TX/RX data, clocks, resets, status) |
| `debug` | `PIPEDebugSignature` | Debug/status outputs |

#### Lane Sub-Interfaces

| Sub-Interface | Signals | Description |
|---------------|---------|-------------|
| `lane.tx` | `pcs_clkout`(In), `data`(Out, 80b), `fifo_wren`(Out) | TX data to serializer |
| `lane.rx` | `pcs_clkout`(In), `data`(In, 88b), `fifo_rden`(Out), `valid`(In) | RX data from deserializer |
| `lane.status` | `pll_lock`, `rx_cdr_lock`, `ready`, `signal_detect` (all In) | Lane status indicators |
| `lane.reset` | `pma_rstn`(Out), `pcs_rx_rst`(Out), `pcs_tx_rst`(Out) | Reset controls |
| `lane.rx_elec_idle` | 1-bit (In) | RXELECIDLE_O from GTR12 |
| `lane.life_clk` | 1-bit (In) | Free-running ~62.5 MHz reference |
| `lane.quad_pd` | 3-bit (Out) | Power-down control to QUAD |

#### Elaboration Phases

1. **Instantiate** all 10 sub-controllers as submodules
2. **Wire PIPE inputs** → sub-controller input ports
3. **Wire sub-controller DRP ports** → DRP mux client slots
4. **Wire DRP mux output** → external DRP port
5. **Wire sub-controller outputs** → PIPE output ports
6. **Wire lane interface** (TX/RX data, clocks, resets, status)
7. **Wire debug outputs**

#### Signal Routing Summary

- **PhyStatus**: OR of `power.phy_status | rate.phy_status | rxdet.phy_status`
- **RxStatus**: Muxed — `rxdet` has priority (carries detection result), defaults to `0b000`
- **RxValid**: Directly from `rxpath.pipe_rx_valid`
- **RxElecIdle**: From `lfps.rx_elec_idle` (passthrough of GTR12 RXELECIDLE_O)
- **Reset routing**: `pma_rstn = power.pma_rstn & ~rate.pma_rst_req`; PCS resets = `power | rate`

---

### 6.4 `pipe_serdes` — Top-Level Entry Point

**File:** `pipe_serdes/pipe_serdes.py`

`PIPESerDes` is the user-facing component. It wraps `PIPESerDesAdapter` and presents a clean PIPE Rev 7.1 component interface.

#### Constructor

```python
PIPESerDes(pipe_config: PIPELaneConfig, *, quad: int = 0, lane: int = 0)
```

| Parameter | Description |
|-----------|-------------|
| `pipe_config` | PIPE-layer configuration |
| `quad` | Quad index (0–N, device-dependent; see device datasheet) |
| `lane` | Lane index within quad (0–3) |

#### Exposed Interfaces

| Interface | Type | Direction | Description |
|-----------|------|-----------|-------------|
| `pipe` | `PIPESerDesSignature` | Out (from PHY) | Complete PIPE interface |
| `debug` | `PIPEDebugSignature` | Out | Read-only debug outputs |

Protocol-conditional signals (`rx_polarity`, `rx_standby`, etc.) are connected only when they exist in the signature based on `PIPELaneConfig.protocol`.

---

### 6.5 `pipe_power` — Power State Machine

**File:** `pipe_serdes/pipe_power.py`

`PIPEPowerFSM` implements P0/P1/P2/P3 state transitions per PIPE Rev 7.1 §8.3.

#### FSM State Diagram

```
                    ┌──────────┐
          Reset# ──→│  RESET   │←─── Reset# from any state
          PLL lock  └────┬─────┘
                         │ reset_n & pll_lock
                         ▼
                   ┌──────────────┐
                   │  EIDLE_EXIT  │ Write eidle CSR OFF
                   └──────┬───────┘
                          │ drp_ready
                          ▼
                   ┌──────────────┐
              ┌───→│  P0_ACTIVE   │←──────────────────┐
              │    └──┬───┬───┬───┘                    │
              │  P1   │   │   │ P3                     │
              │       ▼   │   ▼                        │
              │  ┌────────┐ ┌────────┐                 │
              │  │P1_ENTER│ │P3_ENTER│                 │
              │  └───┬────┘ └───┬────┘                 │
              │      ▼          ▼                      │
              │  ┌────────┐ ┌────────┐                 │
              │  │P1_IDLE │ │P3_OFF  │                 │
              │  └───┬──┬─┘ └─┬───┬──┘                 │
              │  P0  │  │P2   │P0 │P2                  │
              │      │  ▼     │   ▼                    │
              │      │ ┌──────┴──┐                     │
              │      │ │P2_ENTER │                     │
              │      │ └────┬────┘                     │
              │      │      ▼                          │
              │      │ ┌─────────┐                     │
              │      │ │ P2_LOW  │                     │
              │      │ └────┬────┘                     │
              │      │ P0   │                          │
              │      ▼      ▼                          │
              │  ┌──────────────┐                      │
              │  │  P0_RESUME   │                      │
              │  └──────┬───────┘                      │
              │         ▼                              │
              │  ┌──────────────────┐                  │
              │  │ WAIT_PLL_RESUME  │──── pll_lock ────┘
              │  └──────────────────┘
              │          (goes to EIDLE_EXIT)
              └──────────────────────────
```

#### CSR Actions per Transition

| Transition | CSR Write | DRP Lock | quad_pd |
|------------|-----------|----------|---------|
| → P0 (from any) | Eidle OFF (`0x07` to `0x8003A4`) | No | `0b000` |
| P0 → P1 | Eidle ON (`0x01` to `0x8003A4`) | No | `0b001` |
| → P2 | Eidle ON | **Yes** | `0b010` |
| → P3 | Eidle ON | No | `0b011` |

#### Transition Gating

Power transitions are **blocked** when:
- `rate_change_ip` is asserted (rate change in progress)
- `lfps_active` is asserted (LFPS burst in progress)

---

### 6.6 `pipe_rate` — Rate Change Controller

**File:** `pipe_serdes/pipe_rate.py`

`PIPERateController` implements Gen1 ↔ Gen2 rate switching per PIPE §8.4.

#### Rate Change Procedure

```
 1. MAC asserts new Rate[3:0]
 2. FSM detects rate mismatch in P0 → ASSERT_RESET
 3. Assert PMA + PCS resets, acquire DRP lock
 4. Write 7 CSR registers sequentially (WRITE_REGS)
    ├── reg[0]: CPLL divider ratio
    ├── reg[1]: RX AFE gain/attenuation
    ├── reg[2]: RX AFE bias current
    ├── reg[3]: RX AFE boost
    ├── reg[4]: TX clock source select
    ├── reg[5]: RX clock source select
    └── reg[6]: PCS rate mode
 5. Release DRP lock, wait for PLL relock (WAIT_PLL_LOCK)
    └── Timeout: ~16 ms (20-bit counter at 62.5 MHz)
 6. Deassert resets, update current_rate (DEASSERT_RESET)
 7. Assert PclkChangeOk
 8. Wait for MAC PclkChangeAck (WAIT_ACK)
 9. Pulse PhyStatus, deassert PclkChangeOk → IDLE
```

#### Timing

| Phase | Duration |
|-------|----------|
| CSR write phase | ~700–900 ns (7 writes × ~100 ns each) |
| PLL relock | Typical < 200 µs |
| PLL timeout ceiling | ~16 ms |
| **Total** | **~100–200 µs typical** |

#### FSM States

| Code | Name | Description |
|------|------|-------------|
| 0 | IDLE | Waiting for rate mismatch in P0 |
| 1 | ASSERT_RESET | Assert PMA+PCS resets, acquire DRP lock |
| 2 | WRITE_REGS | Write 7 CSR registers sequentially |
| 3 | WAIT_PLL_LOCK | Wait for PLL relock (CMU_OK_O) |
| 4 | DEASSERT_RESET | Release resets, update current_rate |
| 5 | WAIT_ACK | Wait for MAC PclkChangeAck, pulse PhyStatus |

---

### 6.7 `pipe_txpath` — TX Data Path

**File:** `pipe_serdes/pipe_txpath.py`

`PIPETXPath` maps PIPE `TxData[N-1:0]` to the GTR12 80-bit TX data bus.

#### Width Adaptation Logic

The `active_width[2:0]` signal selects which bits of the PIPE bus are placed onto the 80-bit GTR12 bus:

| Width[2:0] | Operation |
|-----------|-----------|
| 0 (W10) | `quad_tx_data[9:0] = pipe_tx_data[9:0]` |
| 1 (W20) | `quad_tx_data[19:0] = pipe_tx_data[19:0]` |
| 2 (W40) | `quad_tx_data[39:0] = pipe_tx_data[39:0]` |
| 3 (W32) | `quad_tx_data[31:0] = pipe_tx_data[31:0]` |
| 4 (W64) | `quad_tx_data[63:0] = pipe_tx_data[63:0]` |
| 5 (W80) | `quad_tx_data[79:0] = pipe_tx_data[79:0]` |

Unused upper bits are zero-filled. Pad bits in partial widths (32, 64) are ignored by the serializer.

#### TX Valid Generation

```
quad_tx_vld = pipe_tx_data_valid & in_p0 & ~pipe_tx_elec_idle
```

TX data is only forwarded when:
- `pipe_tx_data_valid` = 1 (MAC has valid data)
- Power state = P0 (active)
- `tx_elec_idle` = 0 (not in electrical idle)

---

### 6.8 `pipe_rxpath` — RX Data Path

**File:** `pipe_serdes/pipe_rxpath.py`

`PIPERXPath` maps GTR12 88-bit RX data to PIPE `RxData[N-1:0]`.

#### Key Design Decisions

- **RxData is synchronous to RxCLK** (recovered clock), NOT PCLK. The MAC must handle clock domain crossing.
- **RxValid indicates CDR lock stability**, not per-word data validity.
- **rx_data[87:80]** from GTR12 are status information, not symbol data.
- **RxCLK hold**: After CDR lock loss, a 4-bit counter tracks the minimum 8-cycle hold requirement.

#### RxValid Generation

```
pipe_rx_valid = quad_rx_cdr_lock
```

Directly driven from CDR lock status (PMA_RX_LOCK_O).

---

### 6.9 `pipe_rxdet` — Receiver Detection

**File:** `pipe_serdes/pipe_rxdet.py`

`PIPERxDetController` implements the GTR12 DRP-based receiver detection pulse sequence.

#### Detection Sequence

```
 Trigger: tx_detect_rx=1 & tx_elec_idle=1 & (P0 or P2) & reset_n

 IDLE ──[trigger]──→ START_PULSE
                     │ Write 0x03000000 → 0x80033F (start pulse)
                     │ drp_ready
                     ▼
               WAIT_PULSE
                     │ Count 250 cycles (~2.5 µs)
                     ▼
               END_PULSE
                     │ Write 0x00000000 → 0x80033F (end pulse)
                     │ drp_ready
                     ▼
               READ_RESULT
                     │ Read 0x808B34, bit[1] = detected
                     │ drp_rdvld
                     ▼
               REPORT
                     │ Pulse PhyStatus
                     │ RxStatus = 0b011 (detected) or 0b000 (not)
                     ▼
               WAIT_DEASSERT
                     │ Hold RxStatus until ~tx_detect_rx
                     ▼
               IDLE
```

#### Timing

- Total sequence: ~5 µs (3 DRP operations + 250-cycle wait)
- Pulse duration: 250 DRP clock cycles (PCLK domain, ~1.0–2.0 µs at 125–250 MHz)
- DRP bus is locked for the entire 3-step atomic sequence

---

### 6.10 `pipe_lfps` — LFPS Controller

**File:** `pipe_serdes/pipe_lfps.py`

`PIPELFPSController` manages TX LFPS generation and RX LFPS detection.

#### TX LFPS Sequence (FFE Save/Restore)

```
 IDLE ──[trigger]──→ FFE_SAVE (3 DRP writes: LFPS FFE coefficients)
                     │
                     ▼
               LFPS_EIDLE_OFF (write eidle CSR OFF → start burst)
                     │
                     ▼
               LFPS_ACTIVE (wait for MAC to deassert trigger)
                     │
                     ▼
               LFPS_EIDLE_ON (write eidle CSR ON → end burst)
                     │
                     ▼
               FFE_RESTORE (3 DRP writes: normal FFE coefficients)
                     │
                     ▼
               IDLE (pulse lfps_tx_done)
```

The DRP bus lock is held for the entire sequence (FFE_SAVE through FFE_RESTORE) — up to 8 DRP transactions.

#### TX LFPS Trigger Logic

| Power State | Trigger Condition |
|------------|-------------------|
| P1 | `power_down==1` AND `tx_elec_idle==0` |
| P0 | `tx_elec_idle==1` AND `tx_detect_rx==1` |

#### RX LFPS

RX LFPS is a simple passthrough: `rx_elec_idle = rx_elec_idle_hw` (RXELECIDLE_O from GTR12). The MAC interprets burst patterns for LFPS type classification (Polling, Ping, U1/U2/U3 exit).

---

### 6.11 `pipe_msgbus` — Message Bus Protocol Engine

**File:** `pipe_serdes/pipe_msgbus.py`

`PIPEMessageBus` implements the M2P decoder and P2M encoder per PIPE Rev 7.1 §6.1.4.

#### Protocol Overview

The message bus is an 8-bit, PCLK-synchronous interface. The bus idles at `0x00`.

#### Command Encodings (Table 6-10)

| Cmd[3:0] | Name | Cycles | Direction | Hex |
|----------|------|--------|-----------|-----|
| 0000 | NOP | 1 | M2P | `0x00` |
| 0001 | write_uncommitted | 3 | M2P | `0x1x` |
| 0010 | write_committed | 3 | M2P | `0x2x` |
| 0011 | read | 2 | M2P | `0x3x` |
| 0100 | read_completion | 2 | P2M | `0x40` |
| 0101 | write_ack | 1 | P2M | `0x50` |

#### Frame Formats

```
Write:           [Cmd(7:4) | Addr_hi(3:0)] → [Addr_lo(7:0)] → [Data(7:0)]
Read:            [Cmd(7:4) | Addr_hi(3:0)] → [Addr_lo(7:0)]
Read completion: [0100      | 0000]         → [Data(7:0)]
Write ack / NOP: [Cmd(7:4)  | 0000]
```

#### Atomic Writes

1. MAC sends one or more `write_uncommitted` commands → buffered but not applied
2. MAC sends a single `write_committed` command
3. PHY applies all buffered writes + committed write sequentially
4. PHY sends `write_ack` on P2M bus

Write buffer depth: 5 entries (PIPE spec minimum), each holding a 12-bit address and 8-bit data.

#### FSM States

| Code | Name | Description |
|------|------|-------------|
| 0 | IDLE | Wait for non-zero M2P byte |
| 1 | ADDR_LO | Capture address low byte |
| 2 | WR_DATA | Capture write data byte |
| 3 | BUFFER_WRITE | Store uncommitted write |
| 4 | COMMIT_START | Begin atomic commit |
| 5 | COMMIT_BUFFERED | Apply each buffered write |
| 6 | COMMIT_FINAL | Apply committed write |
| 7 | SEND_WRITE_ACK | Transmit `0x50` on P2M |
| 8 | RD_EXEC | Issue read, wait for data |
| 9 | SEND_RD_COMP_CMD | Transmit `0x40` on P2M |
| 10 | SEND_RD_COMP_DATA | Transmit read data byte |

---

### 6.12 `pipe_csr_bridge` — CSR Address Translation

**File:** `pipe_serdes/pipe_csr_bridge.py`

`PIPECSRBridge` translates PIPE message-bus register writes into 32-bit DRP CSR writes with shadow registers.

#### Address Translation Table

| PIPE Addr | GTR12 CSR Addr | GTR12 Field | Type |
|-----------|----------------|-------------|------|
| `0x400` | `0x808234` | TX FFE_0 | DRP bridge (shadow) |
| `0x401` | N/A | TxOnesZeros | Direct (compliance) |
| `0x402` | `0x808238` | TX FFE_1 | DRP bridge (shadow) |
| `0x403` | `0x8082D8` | TX FFE_2 | DRP bridge (shadow) |
| `0x002` | N/A | EB Control | Direct |
| `0x003` | N/A | RxEqEval | Direct |
| `0x004` | N/A | RX Control | Direct |
| `0x008` | N/A | RX Ctrl 4 | Direct |
| `0x800` | N/A | Common Ctrl | Direct (LFPS flag) |
| `0x801` | N/A | NELB Ctrl | Direct |

#### Shadow Register Strategy

PIPE registers are 8-bit; GTR12 DRP CSRs are 32-bit. Shadow registers avoid read-modify-write overhead:

1. 32-bit shadow copy maintained in fabric for each DRP-mapped register
2. On 8-bit PIPE write → update shadow low 8 bits → push full 32-bit shadow to DRP
3. Reads are served from shadow/local storage (zero DRP latency)
4. Shadows initialized to GTR12 hardware reset defaults

#### FSM States

| State | Description |
|-------|-------------|
| IDLE | Accept write/read. Local registers: single-cycle ack. DRP registers: update shadow + DRP write → WAIT_DRP_WR |
| WAIT_DRP_WR | Wait for `drp_ready` → ack → IDLE |

#### Extracted Flags

- `mac_transmit_lfps` — Bit 0 of `common_ctrl_0` (PIPE addr `0x800`)
- `nelb_enable` — Bit 0 of `nelb_control` (PIPE addr `0x801`)

---

### 6.13 `pipe_macclk` — MacCLK Generator

**File:** `pipe_serdes/pipe_macclk.py`

`PIPEMacCLKGen` generates the MacCLK domain from GTR12 LIFE_CLK for low-power state MAC communication.

#### Handshake Protocol (PIPE Rev 7.1 §5.6)

```
 MAC                         PHY
  │                           │
  │── mac_clk_req = 1 ──────→│  MAC requests clock
  │                           │  PHY enables gate
  │←── mac_clk_ack = 1 ──────│  Clock is stable
  │←── mac_clk (running) ────│  ~62.5 MHz LIFE_CLK gated
  │                           │
  │── mac_clk_req = 0 ──────→│  MAC releases clock
  │←── mac_clk_ack = 0 ──────│  Clock gated off
  │←── mac_clk = 0 ──────────│
```

#### Implementation

- Clock gate: `mac_clk = life_clk & clk_running` (combinational AND)
- In synthesis, should map to `BUFGCE` primitive
- `mac_clk_rate` input accepted but currently unused (future programmable divider)
- One sync-edge latency on ack (acceptable per PIPE protocol)

---

### 6.14 `pipe_drp_mux` — DRP Arbiter

**File:** `pipe_serdes/pipe_drp_mux.py`

`PIPEDRPMux` is a fixed-priority DRP arbiter with exclusive locking for atomic multi-write sequences.

#### Priority Scheme

| ID | Client | Purpose | Priority |
|----|--------|---------|----------|
| 0 | Rate | 7-register atomic rate-change sequence | **Highest** |
| 1 | LFPS | FFE save/restore + eidle toggle | High |
| 2 | RxDet | 3-step receiver detection pulse | Medium |
| 3 | Power | Eidle CSR writes for P-state transitions | Medium-low |
| 4 | Eidle | Reserved (power FSM handles eidle directly) | Low |
| 5 | CSR Bridge | Message-bus background register access | **Lowest** |

#### Locking Mechanism

```
 ┌─── When no lock is held: ───────────────────────────────────┐
 │  Priority encoder selects lowest-index client with req      │
 │  If selected client asserts lock_req → lock granted          │
 │    locked=1, owner=client_id                                 │
 └─────────────────────────────────────────────────────────────┘

 ┌─── While locked: ───────────────────────────────────────────┐
 │  ONLY the lock owner can issue DRP transactions              │
 │  Other clients are blocked (their req signals are ignored)   │
 │  Lock released when owner deasserts lock_req                 │
 └─────────────────────────────────────────────────────────────┘
```

#### Response Routing

- Transactions are tracked via `active` + `active_client` registers
- `drp_ready` and `drp_rdvld` are routed back only to the client that initiated the transaction
- `drp_rddata` is broadcast to all clients but only valid when `rdvld` is asserted for a specific client

#### DRP strobe

`drp_strb` is always set to `0xFF` (full 32-bit write).

---

## 7. DRP Serialization and Race Prevention

### The Problem

The PIPE adapter contains **six independent sub-controllers** that all need access to a **single DRP port** on the GTR12 SerDes. Several of these controllers perform **multi-step atomic sequences** that must not be interleaved.

### Race Condition Catalog

| ID | Race | Scenario | Consequence |
|----|------|----------|-------------|
| **R1** | Rate vs. Power | Rate change in progress while MAC requests P1 | Power FSM writes eidle CSR mid-rate-change, corrupting register state |
| **R2** | Rate vs. LFPS | Rate change starts while LFPS burst is active | FFE coefficients overwritten before LFPS restore completes |
| **R3** | LFPS vs. RxDet | LFPS FFE save interleaved with RxDet pulse sequence | Detection result corrupted by FFE write |
| **R4** | CSR Bridge vs. Rate | Background CSR write interleaved with rate-change registers | One of 7 rate registers written with wrong value |
| **R5** | RxDet vs. Power | Detection pulse active while P-state transition writes eidle | Eidle toggled during detection, corrupting result |
| **R6** | Any vs. Any (multi-write) | Any atomic sequence interrupted by another client | Partial register set applied, leaving hardware in inconsistent state |

### Priority Scheme

Fixed priority (lower ID = higher priority) ensures time-critical operations are never starved:

1. **Rate change** (ID=0): Highest priority because it holds resets and must complete quickly
2. **LFPS** (ID=1): High priority because the LFPS burst window is time-constrained
3. **RxDet** (ID=2): Medium priority — detection has a MAC-side timeout
4. **Power** (ID=3): Can tolerate some latency
5. **Eidle** (ID=4): Reserved/unused slot
6. **CSR Bridge** (ID=5): Background — can wait indefinitely

### Locking Mechanism

The DRP mux supports exclusive locking via `lock_req`/`lock_ack`:

- A client asserts `lock_req` when it needs atomic multi-write access
- The mux grants the lock when the client is selected by the priority encoder
- While locked, **only the lock owner** can issue transactions
- All other clients are starved until the lock is released
- Lock is released when the owner deasserts `lock_req`

#### Clients that use locking:

| Client | Locked Operations | # DRP Writes |
|--------|-------------------|-------------|
| Rate | 7 CSR register writes | 7 |
| LFPS | 3 FFE saves + eidle OFF + eidle ON + 3 FFE restores | Up to 8 |
| RxDet | Write start + write end + read result | 3 |
| Power | P2/P3 multi-step transitions | 1–2 |

#### Clients that do NOT lock:

| Client | Reason |
|--------|--------|
| Eidle (ID=4) | Unused (power FSM handles eidle directly) |
| CSR Bridge (ID=5) | Single-write transactions, never locks |

### Why It's Necessary

Without the DRP mux and locking mechanism:
- Interleaved writes could corrupt GTR12 register state
- The PLL could fail to lock after a partial rate change
- LFPS bursts could have wrong swing due to partial FFE reconfiguration
- Receiver detection could produce false results
- Power state transitions could leave the SerDes in an undefined state

---

## 8. Testing

### How to Run Tests

Tests are located in `/home/key2/Downloads/amaranth/serdes_pipe/tests/`. The test directory currently contains an `__init__.py` module.

```bash
# From the project root
cd /home/key2/Downloads/amaranth/serdes_pipe

# Run all tests (when implemented)
python -m pytest tests/

# Run with verbose output
python -m pytest tests/ -v
```

### Test Coverage Overview

The testing infrastructure is established (`tests/__init__.py` exists). Test modules should cover:

| Module | Key Test Scenarios |
|--------|--------------------|
| `pipe_power` | P0↔P1/P2/P3 transitions, PhyStatus pulses, reset sequence, gating during rate change/LFPS |
| `pipe_rate` | Gen1→Gen2 and Gen2→Gen1 full sequences, 7-register writes, PLL relock, timeout |
| `pipe_txpath` | Width adaptation for all 6 widths, eidle gating, P0 gating |
| `pipe_rxpath` | Width extraction for all 6 widths, CDR lock → RxValid |
| `pipe_rxdet` | Full detection sequence, detected/not-detected results, trigger conditions |
| `pipe_lfps` | FFE save/restore, eidle toggle, trigger logic (P0 vs P1) |
| `pipe_msgbus` | NOP, write_uncommitted, write_committed, read, atomic writes, buffer overflow |
| `pipe_csr_bridge` | Local register writes, DRP shadow writes, read-back |
| `pipe_macclk` | Handshake protocol, reset behavior |
| `pipe_drp_mux` | Priority encoding, locking, response routing, starvation |

### Multi-Domain Simulation Setup

For testing with multiple clock domains (PCLK, RxCLK, MacCLK):

```python
from amaranth.sim import Simulator

def testbench():
    sim = Simulator(dut)
    sim.add_clock(1 / 125e6, domain="sync")     # PCLK at 125 MHz
    sim.add_clock(1 / 125e6, domain="rx")        # RxCLK at 125 MHz
    sim.add_clock(1 / 62.5e6, domain="macclk")   # MacCLK at 62.5 MHz
```

---

## 9. Custom Width Extension

### Why 32/64/80-bit Widths Exist

The standard PIPE 7.1 specification defines only 10, 20, and 40-bit widths. This implementation adds **custom extensions** (32, 64, 80 bits) for practical reasons:

| Width | Standard? | Motivation |
|-------|-----------|-----------|
| W10 | Yes | Minimum width, highest PCLK |
| W20 | Yes | Common for Gen1-only designs |
| W40 | Yes | Standard for Gen1+Gen2 |
| **W32** | **Custom** | Non-power-of-two; maps to 16-bit PCS with 1:2 gear. Better BRAM utilization for some buffer depths |
| **W64** | **Custom** | Non-power-of-two; maps to 16-bit PCS with 1:4 gear. Lower PCLK than W40 |
| **W80** | **Custom** | Maximum width; lowest possible PCLK. Easiest timing closure |

### PCLK Frequency Impact

Wider data paths result in lower PCLK frequencies, making timing closure easier:

```
         Gen1 (5 Gbps)              Gen2 (10 Gbps)
Width    PCLK (MHz)                 PCLK (MHz)
─────    ──────────                 ──────────
W10      500.0                      N/A
W20      250.0                      N/A
W32      156.25                     312.5
W40      125.0                      250.0
W64       78.125                    156.25
W80       62.5                      125.0
```

### MAC Implications

- **Clean widths** (W10, W20, W40, W80): Symbol boundaries align to the bus. Standard elastic buffer and SKP insertion/deletion logic works directly.
- **Non-clean widths** (W32, W64): Symbol boundaries do NOT align to the bus width. The elastic buffer and SKP logic must handle **partial symbols** that span across bus words. This adds complexity but can be worthwhile for the PCLK reduction.

---

## 10. Limitations and Open Questions

### Hardware Unknowns

| Item | Status | Notes |
|------|--------|-------|
| **CDR gate CSR** | Uncharacterized | P2/P3 states should gate CDR for power savings, but the specific GTR12 CSR for CDR gating is not yet identified. Currently only eidle is written. |
| **PLL gate CSR** | Uncharacterized | P3 should gate the PLL, but the CSR is not yet characterized. Reserved for future implementation. |
| **RxDet result bit position** | Assumed bit[1] | Documentation indicates bit[1] of `0x808B34` carries the detected flag. Needs silicon validation. |
| **RXDET pulse timing** | 250 cycles assumed | The 250-cycle wait satisfies the documented minimum pulse width but may need tuning per silicon characterization. |

### CTRL_I Bus Investigation

The GTR12 has a `CTRL_I` bus input whose function in the USB context is not fully documented. Possible uses include:
- Per-lane operational mode selection
- Compliance test mode control
- Power-down fine-grained control

This bus is currently **not driven** by the PIPE adapter and is tied to default values at the GowinSerDes level. Investigation is needed to determine if any CTRL_I bits are required for correct USB 3.x operation.

### PD_I / RATE_I Pin Functionality

The GTR12 QUAD exposes `PD_I[2:0]` (power-down) and `RATE_I[1:0]` (rate select) pins at the hard macro level. The relationship between these pins and the DRP CSR-based control used by this adapter is unclear:

- **PD_I**: Currently driven by `power.quad_pd`. Mapping to P-state levels is assumed but not fully verified.
- **RATE_I**: Not currently used (rate switching is done entirely via DRP CSR writes). It's unknown whether `RATE_I` needs to be asserted in conjunction with CSR writes for correct operation.

### Other Open Items

| Item | Description |
|------|-------------|
| **Elastic buffer** | Not yet implemented in the PIPE adapter. SKP ordered-set insertion/removal is currently expected to be handled externally by the MAC or a separate elastic buffer module. |
| **8b10b codec** | Encoding mode is `OFF` in the GTR12 — 8b10b is expected to be performed in fabric. The codec implementation is not part of this package. |
| **RX polarity inversion** | CSR address and values are defined (`0x809008`) but the polarity write is not yet wired in the adapter FSM. |
| **SATA/DP protocols** | Enum placeholders exist but no hardware mapping, CSR tables, or adapter logic is implemented. |
| **Loopback modes** | `tx_detect_rx_loopback` currently only triggers receiver detection. Near-end loopback (NELB) CSR exists but is not wired to the loopback path. |
| **Multi-lane** | The adapter operates on a single lane. Multi-lane USB 3.2 configurations would require multiple adapter instances with shared QUAD resources. |

---

*Generated from source code analysis of `pipe_serdes` package. All addresses, values, and behavioral descriptions are derived from the actual Amaranth HDL implementation.*
