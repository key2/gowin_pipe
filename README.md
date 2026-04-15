# gowin\_pipe

PIPE Rev 7.1 PHY adapter for Gowin GW5A/GW5AST SerDes transceivers, written in [Amaranth HDL](https://amaranth-lang.org/).

Wraps the GTR12 hard-macro SerDes into a standard PIPE low-pin-count interface suitable for USB 3.0/3.1 link controllers. The MAC sees a clean PIPE port; the adapter handles all DRP register programming, power-state sequencing, LFPS generation, receiver detection, and rate switching internally.

## Status

Tested on the **Sipeed Tang Mega 138K Pro** (GW5AST-LV138FPG676A) with a USB 3.0 host. The PHY successfully completes:

- Power-on CSR initialization (11 DRP writes)
- P0/P1/P2/P3 power-state transitions
- Receiver detection (cable plug/unplug)
- LFPS handshake (Polling.LFPS)
- CDR lock on TSEQ from host (Polling.RxEQ entry)

8b10b encode/decode and ordered-set processing are not yet implemented (planned for fabric, not the SerDes hard IP).

## Repository Structure

```
gowin_pipe/
  pipe_serdes/           PIPE adapter library (the PHY)
    __init__.py           Public API exports
    pipe_serdes.py        Top-level PIPESerDes component
    pipe_adapter.py       Internal wiring hub
    pipe_power.py         P0/P1/P2/P3 state machine
    pipe_rxdet.py         Receiver detection controller
    pipe_lfps.py          LFPS burst generation
    pipe_init.py          Power-on CSR init FSM
    pipe_rate.py          Gen1/Gen2 rate switching
    pipe_drp_mux.py       DRP priority arbiter (7 clients)
    pipe_txpath.py        TX data path adapter
    pipe_rxpath.py        RX data path adapter
    pipe_msgbus.py        PIPE message bus decoder
    pipe_csr_bridge.py    Message bus to DRP bridge
    pipe_macclk.py        MacCLK generator
    pipe_config.py        Enums, maps, PIPELaneConfig
    pipe_signature.py     Amaranth Signature definitions
  gowin-serdes/          Git submodule: low-level SerDes driver
  example/               Bring-up test and CLI
    top.py                LTSSM test bitstream
    pipe_cli.py           Host-side command tool
    gw5ast_dvk.py         Platform definition (Tang Mega 138K)
    uart.py               UART IP core
    gowin_pll_gen.py      PLL IP generator
  tests/                 Amaranth simulation tests
  PIPE_SERDES_DOCS.md    Detailed internal documentation
```

## Dependencies

- Python 3.10+
- [Amaranth HDL](https://github.com/amaranth-lang/amaranth) 0.5.x
- Gowin EDA (gw\_sh) for synthesis
- [openFPGALoader](https://github.com/trabucayre/openFPGALoader) for programming
- pyserial (for the CLI tool)

Clone with submodules:

```bash
git clone --recursive https://github.com/key2/gowin_pipe.git
```

---

## Quick Start

### Minimal Instantiation

```python
from pipe_serdes import PIPESerDes, PIPELaneConfig, PIPEProtocol, PIPEWidth, USBRate
from gowin_serdes import GowinDevice

config = PIPELaneConfig(
    protocol=PIPEProtocol.USB3,
    device=GowinDevice.GW5AST_138,
    quad=0,
    lane=0,
    supported_rates=[USBRate.GEN1],
    default_width=PIPEWidth.W40,
)

phy = PIPESerDes(config)
```

In your `Elaboratable.elaborate()`:

```python
m.submodules.phy = DomainRenamer("upar")(phy)
m.d.comb += phy.por_n.eq(1)  # Release power-on reset

# Connect PIPE signals
m.d.comb += [
    phy.pipe.reset_n.eq(1),
    phy.pipe.power_down.eq(0),       # P0
    phy.pipe.rate.eq(0),             # Gen1
    phy.pipe.width.eq(PIPEWidth.W40.value),
    phy.pipe.rx_width.eq(PIPEWidth.W40.value),
    phy.pipe.tx_data.eq(your_tx_data),
    phy.pipe.tx_data_valid.eq(your_tx_valid),
    phy.pipe.tx_elec_idle.eq(your_eidle),
    phy.pipe.tx_detect_rx_loopback.eq(your_detect),
    # ... other PIPE signals
]

# Read PIPE outputs
your_rx_data = phy.pipe.rx_data
your_rx_valid = phy.pipe.rx_valid
your_phy_status = phy.pipe.phy_status
your_rx_status = phy.pipe.rx_status
```

### CSR File Generation

The Gowin toolchain requires a `.csr` file for SerDes configuration. Generate it before synthesis:

```python
phy.generate_csr(
    output_path="serdes.csr",
    toml_path="serdes.toml",  # Optional: human-readable config
)
```

Add it to the platform:

```python
platform.add_file("serdes.csr", Path("serdes.csr").read_bytes())
```

### Build and Program

```bash
cd example
python top.py               # Build bitstream
python top.py program       # Build and program via openFPGALoader
```

---

## Configuration

### PIPELaneConfig

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `protocol` | `PIPEProtocol` | `USB3` | Protocol family (`USB3`, `SATA`, `DP`) |
| `device` | `GowinDevice` | `GW5AST_138` | Target FPGA |
| `quad` | `int` | `0` | GTR12 quad index (0 or 1) |
| `lane` | `int` | `0` | Lane within quad (0-3) |
| `supported_rates` | `list[USBRate]` | `[GEN1]` | Line rates to support |
| `default_width` | `PIPEWidth` | `W40` | TX data-path width |
| `default_rx_width` | `PIPEWidth\|None` | `None` | RX width override (symmetric if None) |
| `enable_msg_bus` | `bool` | `True` | Enable PIPE message bus |
| `enable_mac_clk` | `bool` | `True` | Enable MacCLK domain |

### Data Width Encoding

| PIPEWidth | Bits | Symbols/cycle | Gear Ratio | PCLK (Gen1) |
|-----------|------|---------------|------------|-------------|
| `W10` | 10 | 1 | 1:1 | 500 MHz |
| `W20` | 20 | 2 | 1:2 | 250 MHz |
| `W40` | 40 | 4 | 1:4 | 125 MHz |

### USB Rate / Width Hardware Mapping

| Rate | Width | TX Data Rate | PCS Width | Gear | Encode |
|------|-------|-------------|-----------|------|--------|
| Gen1 | W10 | 5.0G | 10 | 1:1 | 8b10b |
| Gen1 | W20 | 5.0G | 10 | 1:2 | 8b10b |
| Gen1 | W40 | 5.0G | 10 | 1:4 | 8b10b |

---

## PIPE Interface Signals

All signals follow PIPE Rev 7.1 naming and semantics. Direction is from the PHY's perspective (In = driven by MAC, Out = driven by PHY).

### MAC to PHY (Inputs)

| Signal | Width | Description |
|--------|-------|-------------|
| `tx_data` | 40-80 | Transmit data |
| `tx_data_valid` | 1 | TX data qualifier |
| `tx_elec_idle` | 4 | TX electrical idle request |
| `tx_detect_rx_loopback` | 1 | Receiver detection / loopback trigger |
| `power_down` | 4 | Power state request (P0=0, P1=1, P2=2, P3=3) |
| `rate` | 4 | Line rate (0=Gen1, 1=Gen2) |
| `width` | 3 | TX data width selector |
| `rx_width` | 3 | RX data width selector |
| `reset_n` | 1 | Active-low asynchronous reset |
| `phy_mode` | 4 | Protocol selector |
| `pclk` | 1 | Parallel clock from MAC |
| `rx_polarity` | 1 | RX polarity inversion |
| `rx_termination` | 1 | RX termination control |
| `rx_standby` | 1 | RX low-power standby |
| `pclk_change_ack` | 1 | PCLK rate change acknowledgement |
| `m2p_msg_bus` | 8 | MAC-to-PHY message bus |
| `mac_clk_reset_n` | 1 | MacCLK domain reset |
| `mac_clk_rate` | 5 | MacCLK rate selector |
| `mac_clk_req` | 1 | MacCLK request |

### PHY to MAC (Outputs)

| Signal | Width | Description |
|--------|-------|-------------|
| `rx_data` | 40-80 | Received data (synchronous to RxCLK) |
| `rx_valid` | 1 | RX data valid (CDR locked) |
| `rx_elec_idle` | 1 | RX electrical idle detected |
| `rx_status` | 3 | RX status code (see below) |
| `phy_status` | 1 | Completion pulse for power/rate/detect |
| `pclk_change_ok` | 1 | PHY ready for PCLK change |
| `p2m_msg_bus` | 8 | PHY-to-MAC message bus |
| `max_pclk` | 1 | PHY-generated max-rate PCLK |
| `rx_clk` | 1 | CDR recovered clock |
| `mac_clk_ack` | 1 | MacCLK acknowledgement |
| `mac_clk` | 1 | Gated MacCLK output |

### RxStatus Encoding

| Value | Name | Description |
|-------|------|-------------|
| `000` | OK | No error, data valid |
| `001` | SKP\_ADDED | SKP ordered set added |
| `010` | SKP\_REMOVED | SKP ordered set removed |
| `011` | RX\_DETECTED | Receiver detected (on PhyStatus pulse) |
| `100` | DECODE\_ERROR | 8b10b decode error |
| `101` | EB\_OVERFLOW | Elastic buffer overflow |
| `110` | EB\_UNDERFLOW | Elastic buffer underflow |
| `111` | DISPARITY\_ERROR | Running disparity error |

### Debug Port

| Signal | Width | Description |
|--------|-------|-------------|
| `power_state` | 4 | Current P-state |
| `rate_fsm_state` | 4 | Rate controller FSM |
| `rxdet_fsm_state` | 4 | Receiver detection FSM |
| `lfps_fsm_state` | 4 | LFPS controller FSM |
| `msgbus_fsm_state` | 4 | Message bus FSM |
| `init_fsm_state` | 4 | Init FSM (0=IDLE, 1=WRITE, 2=DONE) |
| `init_done` | 1 | Init sequence completed |
| `drp_mux_owner` | 4 | DRP arbiter current client |
| `drp_mux_locked` | 1 | DRP bus locked |
| `drp_busy` | 1 | DRP transaction in progress |
| `pll_lock` | 1 | PLL lock indicator |
| `cdr_lock` | 1 | CDR lock indicator |
| `serdes_arb_state` | 2 | UPAR arbiter state |

---

## Architecture

### Block Diagram

```
          MAC (user logic)
              |
         PIPE Interface
              |
    +---------v----------+
    |   PIPESerDes       |
    |                    |
    |  +- PIPEAdapter -+ |    DRP Mux (7 clients, priority-based)
    |  | InitFSM     0 |------+
    |  | RateCtrl    1 |------+
    |  | LFPSCtrl   2 |------+---> GowinSerDes UPAR Arbiter
    |  | RxDetCtrl  3 |------+         |
    |  | PowerFSM   4 |------+    GTR12 Hard Macro
    |  | (Eidle)    5 |------+    (DRP registers)
    |  | CSRBridge  6 |------+
    |  |               |
    |  | TXPath ------>| TX data --> Serializer
    |  | RXPath <------| RX data <-- Deserializer
    |  | MsgBus        |
    |  | MacCLKGen     |
    |  +---------------+
    +--------------------+
```

### Clock Domains

| Domain | Frequency | Source | Used By |
|--------|-----------|--------|---------|
| `sync` | 50 MHz | Board oscillator | UART FSMs, LED logic |
| `upar` | 62.5 MHz | GTR12 LIFE\_CLK | PIPE adapter, all sub-controllers |
| `pclk` | 125 MHz (W40) | GTR12 TX PCS CLK | TX data path |
| `rxclk` | Recovered | GTR12 RX PCS CLK | RX data path |

### DRP Mux Priority

| Slot | Client | Lock? | Description |
|------|--------|-------|-------------|
| 0 | InitFSM | Yes (11 writes) | Power-on init, highest priority |
| 1 | RateCtrl | Yes (7 writes) | Gen1/Gen2 rate switching |
| 2 | LFPSCtrl | Yes (3+1+1+3 writes) | FFE save, eidle toggle, FFE restore |
| 3 | RxDetCtrl | Yes (3 ops) | Start pulse, wait, end pulse, read result |
| 4 | PowerFSM | Yes (P2/P3) | EIDLE CSR writes for P-state transitions |
| 5 | (Eidle) | No | Reserved slot (power FSM handles eidle) |
| 6 | CSRBridge | No | Message bus background register access |

---

## Sub-Controller Reference

### PIPEPowerFSM

P0/P1/P2/P3 state machine per PIPE Rev 7.1 Section 8.3.

**FSM states:** `RESET` -> `EIDLE_EXIT` -> `P0_ACTIVE` -> `P1_ENTER`/`P2_ENTER`/`P3_ENTER` -> `P0_RESUME` -> `WAIT_PLL_RESUME` -> `EIDLE_EXIT`

Every completed transition produces a single-cycle `phy_status` pulse. Transitions are gated during rate changes and LFPS activity to avoid DRP conflicts.

### PIPERxDetController

Receiver detection via DRP pulse sequence (PIPE Section 5.6.1):

1. Write `0x03000000` to RXDET\_PULSE address (start pulse)
2. Wait 250 cycles (~2.5 us)
3. Write `0x00000000` to RXDET\_PULSE address (end pulse)
4. Read RXDET\_RESULT address, bit\[0\] = detected

**Trigger:** `TxDetectRx=1 AND TxElecIdle=1 AND (PowerDown=P0 OR P2)`

**Result:** `rx_status=0b011` (detected) or `0b000` (not detected), signaled with `phy_status` pulse.

### PIPELFPSController

LFPS burst generation for Polling.LFPS (PIPE Section 5.5.1):

1. Save 3 FFE registers (normal values)
2. Write 3 FFE registers (LFPS swing values)
3. Write EIDLE OFF (start burst on wire)
4. MAC deasserts trigger -> Write EIDLE ON (end burst)
5. Restore 3 FFE registers (normal values)

**Trigger in P0:** `TxElecIdle=1 AND TxDetectRx=1`
**Trigger in P1:** `TxElecIdle=0`

### PIPEInitFSM

Writes 11 CSR registers at power-on:

| # | Register | Address (Q0L0) | Value |
|---|----------|----------------|-------|
| 0 | TX\_FFE\_0 | 0x808234 | 0x0000F000 |
| 1 | TX\_FFE\_1 | 0x808238 | 0x00000000 |
| 2 | TX\_FFE\_2 | 0x8082D8 | 0x00000110 |
| 3 | CDR\_CFG\_shared | 0x8083F8 | 0x00038002 |
| 4 | LN\_CTRL\_shared | 0x808830 | 0xFFFFF9FF |
| 5 | CDR\_CFG\_0 | 0x800253 | 0x7F000000 |
| 6 | CDR\_CFG\_1 | 0x80025E | 0x007F0000 |
| 7 | CDR\_CFG\_2 | 0x80025F | 0x7F000000 |
| 8 | CDR\_CFG\_3 | 0x800254 | 0x0000004F |
| 9 | CDR\_CFG\_4 | 0x800260 | 0x0000004F |
| 10 | CDR\_CFG\_5 | 0x800261 | 0x00004F00 |

Addresses scale automatically for other quad/lane combinations.

### PIPERateController

Gen1 <-> Gen2 switching via 7 atomic DRP writes. Asserts PMA/PCS resets, writes all 7 registers under DRP lock, waits for PLL relock, then pulses PhyStatus.

### PIPEDRPMux

Fixed-priority arbiter with exclusive locking. Lower client index = higher priority. When a client asserts `lock_req` and wins the bus, all other clients are blocked until the lock is released.

---

## Example: Bring-Up Test

The `example/` directory contains a complete LTSSM bring-up test for the Tang Mega 138K Pro board.

### Hardware Setup

- Sipeed Tang Mega 138K Pro (GW5AST-LV138FPG676A)
- USB 3.0 cable connected to the SerDes connector
- FTDI USB-to-serial adapter providing 3 UARTs:
  - `/dev/ttyUSB5` — Command interface (UART0)
  - `/dev/ttyUSB4` — Debug trace (UART1, TX-only)
  - `/dev/ttyUSB3` — RX data snapshots (UART2, TX-only)

### CLI Tool

```bash
cd example
python pipe_cli.py /dev/ttyUSB5           # Interactive mode
python pipe_cli.py /dev/ttyUSB5 ping      # Single command
```

### Commands

| Command | UART Protocol | Description |
|---------|--------------|-------------|
| `ping` | `'T'` -> `0x55` | Verify FPGA communication |
| `status` | `'S'` -> 8 bytes | Read all PHY status registers |
| `power N` | `'P' N` -> `0x00` | Set PowerDown to N (0-3) |
| `detect` | `'D'` -> 1 byte | Receiver detection. Returns `0x03`=detected, `0x00`=not detected, `0xFF`=timeout |
| `eidle N` | `'E' N` -> `0x00` | Set TxElecIdle (0=off, 1=on) |
| `ltssm` | `'F'` -> 1 byte | Run full LTSSM sequence (see below) |
| `monitor` | Repeated `'S'` | Continuous status polling |

### LTSSM Sequence (`ltssm` command)

Walks the USB 3.0 link training through Polling.RxEQ:

| Step | Tag | What Happens |
|------|-----|-------------|
| INIT\_WAIT | `I` | Wait 10 ms for POR + CSR init |
| EIDLE | `E` | Assert electrical idle in P0 |
| DETECT | `D` | Receiver detection (TxDetectRx) |
| LFPS\_BURST | `B` | TX LFPS burst (~2 us hold) |
| LFPS\_GAP | | 9 us gap, count RX edges |
| LFPS\_DONE | `H` | Handshake pass (TX>=16, RX>=2) |
| RXEQ\_ENTER | `Q` | Deassert TxElecIdle, enter P0 active |
| CDR\_WAIT | `C` | Wait for CDR lock on TSEQ |
| MONITOR | `U` | CDR locked, snapshot RX data |
| FAIL | `X` | Timeout at any step |

The LTSSM command uses all 3 UARTs:

```bash
python pipe_cli.py /dev/ttyUSB5 ltssm /dev/ttyUSB4 /dev/ttyUSB3
```

### Status Byte Layout (8 bytes, `'S'` command)

| Byte | Bits | Content |
|------|------|---------|
| 0 | [3:0] | Power state |
| 0 | [4] | RxElecIdle |
| 0 | [5] | RxValid |
| 0 | [6] | PhyStatus |
| 0 | [7] | CDR lock |
| 1 | [3:0] | RxDet FSM state |
| 1 | [7:4] | Rate FSM state |
| 2 | [7:0] | IO hash |
| 3 | [0] | PLL lock |
| 3 | [1] | Init done |
| 3 | [3:2] | LFPS FSM state |
| 4-7 | [31:0] | rx\_data snapshot |

### LTSSM Result Byte (`'F'` command response)

| Bit | Meaning |
|-----|---------|
| [0] | CDR locked |
| [1] | PLL locked |
| [2] | Init done |
| [7:4] | LFPS RX burst count (lower 4 bits) |
| `0xFE` | LTSSM failed (timeout at some step) |

### Example Session

```
$ python pipe_cli.py /dev/ttyUSB5
Opened /dev/ttyUSB5 @ 115200
PING: OK (0x55)

pipe> status
  Power State    :          P0 (active)
  CDR Lock       :                   no
  Init Done      :                   no
  RX Data        :   0xFFFFFFFF

pipe> detect
DETECT: RECEIVER DETECTED (cable plugged) -- RxStatus=0x03

pipe> ltssm
Starting LTSSM sequence...
  [snap] rx_data=0x50444549  CDR=n  Init=Y
LTSSM Result:
  CDR Lock  : YES
  PLL Lock  : no
  LFPS RX   : 3 bursts received
```

---

## Platform: Tang Mega 138K Pro

The `gw5ast_dvk.py` platform definition supports:

| Resource | Count | Pins |
|----------|-------|------|
| LEDs | 6 | Active-low, accent LEDs |
| UARTs | 3 | Via FT4232H (ttyUSB3-5) |
| Oscillator | 1 | 50 MHz |

### LED Assignment

| LED | Signal |
|-----|--------|
| 0 | Heartbeat (~1 Hz) |
| 1 | PLL lock |
| 2 | CDR lock |
| 3 | PhyStatus (stretched pulse) |
| 4 | LTSSM state bit 0 |
| 5 | IO hash (anti-sweep) |

---

## Known Limitations

- **8b10b encode/decode** is not implemented. The SerDes runs in raw bit mode. 8b10b will be done in fabric.
- **PLL lock observation** (`FABRIC_LANE{n}_CMU_OK_O`) is unreliable on some quad/lane configurations. The power FSM uses a settle timer instead.
- **P2 -> P0 resume** has a known DRP issue when `quad_pd != 0`. The example stays in P0 for the entire LTSSM sequence.
- **LFPS handshake** success rate is ~33% due to burst timing sensitivity. The LFPS controller generates valid bursts; the counting logic in the example needs calibration.
- **CDR/PLL gate CSR** writes for P2/P3 deep sleep are not yet characterized.
- **Multi-lane** operation is not tested (single-lane only).
- **SATA and DisplayPort** protocol modes are defined but not implemented.

## License

See individual source files for license information.
