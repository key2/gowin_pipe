# Driving the gowin_pipe PHY from Rx.Detect to U0

Step-by-step guide for a MAC layer to bring a USB 3.0 Gen1 SuperSpeed
link to the active U0 state through the gowin_pipe PIPE interface.

The gowin_pipe PHY is a **raw bit pump** — the MAC owns 8b10b
encoding/decoding and generates all ordered sets (TSEQ, TS1, TS2, SKP,
idle symbols) and LFPS data directly on `tx_data`. The PHY only manages
the SerDes analog configuration (FFE, EIDLE, receiver detection) via
internal DRP writes.

## Signal Reference

```
MAC → PHY (Command)              PHY → MAC (Status)
────────────────────              ──────────────────
pipe.reset_n                      pipe.phy_status
pipe.power_down[3:0]              pipe.rx_valid
pipe.tx_data[39:0]                pipe.rx_data[39:0]
pipe.tx_data_valid                pipe.rx_elec_idle
pipe.tx_elec_idle[3:0]            pipe.rx_status[2:0]
pipe.tx_detect_rx_loopback
pipe.rate[3:0]
pipe.width[2:0]
```

---

## Overview

```
    Rx.Detect         Polling              Polling         Polling          Polling
┌─────────────┐  ┌────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────┐
│ .Quiet      │→ │   .LFPS    │→ │    .RxEQ     │→ │   .Active    │→ │  .Config │→
│ .Active     │  │            │  │   (TSEQ)     │  │  (TS1→TS1)   │  │ (TS2→TS2)│
│ (RxDet)     │  │ (16 bursts │  │  (65536 ×    │  │              │  │          │
│             │  │  handshake)│  │   32 bytes)  │  │              │  │          │
└─────────────┘  └────────────┘  └──────────────┘  └──────────────┘  └──────────┘
                                                                          │
                                     ┌───────────────────────────────────┘
                                     ▼
                              ┌──────────────┐    ┌───────┐
                              │ Polling.Idle │ →  │  U0   │
                              │ (Idle symbol │    │       │
                              │  handshake)  │    │       │
                              └──────────────┘    └───────┘
```

---

## Step 0 — Power-On Reset

### What the MAC drives

```
pipe.reset_n         = 0        (hold reset)
pipe.power_down      = 0b0010   (P2 — reset state)
pipe.tx_elec_idle    = 0b0001   (idle)
pipe.tx_data_valid   = 0
pipe.tx_detect_rx_loopback = 0
pipe.rate            = 0        (Gen1)
pipe.width           = 2        (W40)
```

### What the PHY does internally

- POR held low → init FSM writes 11 CSR registers (FFE, CDR config).
- Power FSM stays in RESET state.

### MAC action

After a few milliseconds, deassert reset:

```
pipe.reset_n = 1
```

### What to wait for

The PHY's init FSM completes its 11 DRP writes, then the power FSM
transitions through RESET → WAIT_PLL → EIDLE_EXIT → P0_ACTIVE.

**Wait for:** `pipe.phy_status` pulse = 1 (indicates P0 entry).

---

## Step 1 — Rx.Detect.Quiet → Rx.Detect.Active

*USB 3.2 §7.5.3: Detect whether a receiver is present on the link.*

The MAC must instruct the PHY to perform **receiver detection** — the PHY
sends a voltage pulse on the TX pair and checks the reflected impedance.

### What the MAC drives

```
pipe.power_down    = 0b0000     (P0)
pipe.tx_elec_idle  = 0b0001     (idle — required for RxDet)
pipe.tx_detect_rx_loopback = 1  (trigger receiver detection)
```

### What the PHY does internally

The RxDet controller (pipe_rxdet.py) runs:

1. Write `CSR_WRITE_PLUSE = 0x03000000` (send detection pulse)
2. Wait 250 cycles (~2 µs)
3. Write `CSR_WRITE_PLUSE = 0x00000000` (end pulse)
4. Read `CSR_READ_RXDET` — check bit[1] for receiver presence

### What to wait for

**Watch:** `pipe.phy_status` pulse AND `pipe.rx_status`:

| `rx_status` | Meaning |
|---|---|
| `3'b011` | **Receiver detected** → proceed to Polling |
| `3'b000` | Receiver not detected → retry after 12 ms |

### MAC action on receiver detected

```
pipe.tx_detect_rx_loopback = 0   (clear trigger)
```

Wait for `pipe.phy_status` to deassert, then proceed to Polling.LFPS.

---

## Step 2 — Polling.LFPS

*USB 3.2 §7.5.4.3: Establish DC operating point and synchronise with
the link partner using LFPS handshake.*

This is the critical LFPS step. The MAC generates the Polling.LFPS
pattern directly on `tx_data` and the PHY handles the SerDes FFE/EIDLE
configuration.

### Polling.LFPS Burst Parameters (USB 3.2 Table 6-29)

| Parameter | Min | Max |
|---|---|---|
| tBurst | 0.6 µs | 1.4 µs |
| tRepeat | 6 µs | 14 µs |
| LFPS frequency | 10 MHz | 50 MHz |

### LFPS Burst Sequence

An LFPS burst = square wave at 10–50 MHz for tBurst duration, then
electrical idle for (tRepeat − tBurst). Repeat.

The MAC must generate **at least 16 consecutive Polling.LFPS bursts** and
complete the handshake (receive 2 bursts + send 4 more after receiving 1).

### What the MAC drives — Starting each burst

```
pipe.power_down              = 0b0000   (P0)
pipe.tx_elec_idle            = 0b0001   (asserted — P0 LFPS trigger)
pipe.tx_detect_rx_loopback   = 1        (combined with eidle = LFPS trigger)
pipe.tx_data_valid           = 1
pipe.tx_data                 = <LFPS square wave pattern>
```

The LFPS trigger condition in P0 is:
`TxElecIdle=1 AND TxDetectRx=1 AND PowerDown=P0`.

This tells the PHY's LFPS controller to:
1. Write LFPS FFE values (3 DRP writes)
2. Write EIDLE_OFF (TX driver active)
3. Wait 33 cycles (settling)
4. Assert `lfps_pattern_en` → txpath bypasses P0/eidle gates

### Generating the LFPS Square Wave

The MAC must alternate all-1s and all-0s on `tx_data[39:0]` at the
right cadence to produce a 10–50 MHz signal when serialised at 5 Gbps.

**Example: 15.625 MHz (matching the Gowin reference design)**

At PCLK = 125 MHz with W40: each word = 40 bits = 8 ns on the wire.
For 15.625 MHz: half-period = 32 ns = 4 PCLK cycles.

```
Cycle  0: tx_data = 40'hFF_FFFF_FFFF   (all ones — HIGH phase)
Cycle  1: tx_data = 40'hFF_FFFF_FFFF
Cycle  2: tx_data = 40'hFF_FFFF_FFFF
Cycle  3: tx_data = 40'hFF_FFFF_FFFF
Cycle  4: tx_data = 40'h00_0000_0000   (all zeros — LOW phase)
Cycle  5: tx_data = 40'h00_0000_0000
Cycle  6: tx_data = 40'h00_0000_0000
Cycle  7: tx_data = 40'h00_0000_0000
Cycle  8: tx_data = 40'hFF_FFFF_FFFF   (repeat)
...
```

Repeat for tBurst (0.6–1.4 µs = 75–175 cycles at 125 MHz).

### Ending Each Burst

After tBurst duration, the MAC deasserts the LFPS trigger:

```
pipe.tx_detect_rx_loopback = 0    (clear LFPS trigger)
pipe.tx_data_valid         = 0
```

The PHY's LFPS controller then:
1. Writes EIDLE_ON (TX enters electrical idle — clean burst edge)
2. Restores normal FFE values (3 DRP writes)

### Idle Gap Between Bursts

During the idle gap (tRepeat − tBurst ≈ 5–12.6 µs), the MAC keeps:

```
pipe.tx_detect_rx_loopback = 0
pipe.tx_data_valid         = 0
pipe.tx_elec_idle          = 0b0001   (idle)
```

The PHY is in electrical idle. The TX driver outputs common-mode voltage.

### Repeat

After the idle gap, the MAC reasserts the trigger to start the next burst.
This repeats for at least 16 bursts.

### Monitoring the Handshake

While transmitting, the MAC monitors `pipe.rx_elec_idle`:

- `rx_elec_idle` = 0 → Remote LFPS burst is being received
- `rx_elec_idle` = 1 → Remote is in electrical idle (gap)

**Exit conditions (all three must be met):**

1. At least **16** consecutive Polling.LFPS bursts sent
2. **2** consecutive Polling.LFPS bursts received (rx_elec_idle
   transitions)
3. **4** consecutive Polling.LFPS bursts sent after first received burst

### Timeout

If the handshake is not complete within **360 ms**, the MAC must
transition back to Rx.Detect (or Compliance/SS.Disabled depending on
port type).

---

## Step 3 — Polling.RxEQ

*USB 3.2 §7.5.4.7: Receiver equaliser training using TSEQ ordered sets.*

After the LFPS handshake, the MAC transitions to sending TSEQ training
data. This is the first time actual SuperSpeed 8b10b-encoded data is
transmitted.

### What the MAC drives

```
pipe.tx_elec_idle          = 0b0000   (NOT idle — active data)
pipe.tx_detect_rx_loopback = 0
pipe.tx_data_valid         = 1
pipe.tx_data               = <8b10b-encoded TSEQ ordered sets>
```

When `TxElecIdle` goes to 0, the power FSM writes EIDLE_OFF to activate
the TX driver for normal data.

### TSEQ Ordered Set (Gen1, 32 symbols)

```
Symbol 0:  K28.5 (COM)
Symbol 1:  D31.7 (0xFF)
Symbol 2:  D23.0 (0x17)
Symbol 3:  D0.6  (0xC0)
Symbol 4:  D20.0 (0x14)
Symbol 5:  D18.5 (0xB2)
Symbol 6:  D7.7  (0xE7)
Symbol 7:  D2.0  (0x02)
Symbol 8:  D2.4  (0x82)
Symbol 9:  D18.3 (0x72)
Symbol 10: D14.3 (0x6E)
Symbol 11: D8.1  (0x28)
Symbol 12: D6.5  (0xA6)
Symbol 13: D30.5 (0xBE)
Symbol 14: D13.3 (0x6D)
Symbol 15: D31.5 (0xBF)
Symbols 16–31: D10.2 (0x4A)
```

The MAC must 8b10b-encode each symbol and pack 4 encoded 10-bit symbols
per PCLK cycle into the 40-bit `tx_data` bus.

### Duration

Transmit **65,536** consecutive TSEQ ordered sets (each 32 symbols =
320 bits = 8 PCLK cycles at W40). Total: 65,536 × 8 = 524,288 PCLK
cycles = ~4.2 ms at 125 MHz.

### What to watch

While transmitting TSEQ, the MAC monitors `pipe.rx_valid` and
`pipe.rx_data` for the remote's TSEQ. The CDR should lock during this
phase (`rx_valid` goes high).

### Exit

After 65,536 TSEQ ordered sets are transmitted → transition to
Polling.Active.

---

## Step 4 — Polling.Active

*USB 3.2 §7.5.4.8: TS1/TS2 handshake for symbol lock and link parameter
negotiation.*

### What the MAC drives

```
pipe.tx_data_valid = 1
pipe.tx_data       = <8b10b-encoded TS1 ordered sets>
```

### TS1 Ordered Set (16 symbols)

```
Symbol 0:     K28.5  (COM)
Symbol 1:     D10.2  (Link number — set to PAD = K23.7 during Polling)
Symbol 2:     D10.2  (Lane number — set to PAD = K23.7 during Polling)
Symbol 3:     D10.2  (N_FTS — number of FTS for low power exit)
Symbol 4:     D10.2  (Rate and control field)
Symbol 5:     D0.0   (Training control: set per state)
Symbols 6–15: D10.2  (TS1 identifier = 0x4A repeated)
```

The MAC transmits TS1 continuously while monitoring `pipe.rx_data` for
incoming TS1 or TS2 from the host.

### Exit Condition

**Receive 8 consecutive identical TS1 or TS2 ordered sets** on
`pipe.rx_data` → transition to Polling.Configuration.

### Timeout

12 ms — if not met, return to Rx.Detect.

---

## Step 5 — Polling.Configuration

*USB 3.2 §7.5.4.9: TS2 handshake confirms link parameters.*

### What the MAC drives

Upon entering Polling.Configuration:

```
pipe.tx_data = <8b10b-encoded TS2 ordered sets>
```

### TS2 Ordered Set (16 symbols)

Same structure as TS1 but with the TS2 identifier (D5.2 = 0x45 repeated
in symbols 6–15 instead of 0x4A).

### Exit Conditions (both must be met)

1. **8 consecutive identical TS2** received on `pipe.rx_data`
2. **16 TS2 sent** after receiving the first of the 8 consecutive TS2

→ Transition to Polling.Idle.

### Timeout

12 ms — if not met, return to Rx.Detect.

---

## Step 6 — Polling.Idle

*USB 3.2 §7.5.4.10: Idle symbol handshake — final step before U0.*

### What the MAC drives

```
pipe.tx_data       = <8b10b-encoded Idle symbols (logical idle = 0x00)>
pipe.tx_data_valid = 1
pipe.tx_elec_idle  = 0b0000   (still actively driving)
```

An Idle Symbol in Gen1 8b10b is simply a stream of `D0.0` (0x00) data
symbols (no K-character). The MAC sends these continuously.

### Exit Conditions (both must be met)

1. **8 consecutive Idle Symbols** received on `pipe.rx_data`
2. **16 Idle Symbols sent** after receiving the first Idle Symbol

### Timeout

2 ms — if not met, return to Rx.Detect.

---

## Step 7 — U0 (Active)

*USB 3.2 §7.5.5: Link is operational. Data packets can flow.*

### MAC enters normal operation

```
pipe.power_down    = 0b0000   (P0)
pipe.tx_elec_idle  = 0b0000   (active)
pipe.tx_data_valid = 1
pipe.tx_data       = <link-layer packets, SKP ordered sets, etc.>
```

The link is trained. The MAC can now send/receive USB data packets,
link management packets (LMP), and must maintain the link by:

- Sending **SKP ordered sets** periodically (every 354 µs ± 6 µs)
  to compensate for clock frequency differences.
- Responding to link partner commands (LGO_U1, LGO_U2, etc.)

---

## Complete Timing Summary

| Phase | Duration | PIPE Signals | On the Wire |
|---|---|---|---|
| POR + Init | ~10 µs | reset_n=0→1 | Nothing (idle) |
| Rx.Detect | ~2 µs | TxDetectRx=1 | Detection pulse |
| Polling.LFPS | ~160–360 µs | TxEIdle=1,TxDetRx=1 | 16+ LFPS bursts |
| Polling.RxEQ | ~4.2 ms | TxEIdle=0,TxData=TSEQ | 65K × TSEQ |
| Polling.Active | 0.1–12 ms | TxData=TS1 | TS1 → receive 8×TS1/TS2 |
| Polling.Config | 0.1–12 ms | TxData=TS2 | TS2 → receive 8×TS2 |
| Polling.Idle | 0.01–2 ms | TxData=Idle | Idle → receive 8×Idle |
| **U0** | — | Normal operation | Data packets |

**Typical total time from power-on to U0: ~5–20 ms.**

---

## PHY Internal Activity Summary

What the gowin_pipe PHY does autonomously at each stage — the MAC does
not need to manage any of this:

| Stage | PHY Action |
|---|---|
| POR | Init FSM: 11 CSR writes (FFE, CDR) |
| Rx.Detect | RxDet: pulse CSR write → wait → read result |
| Polling.LFPS (each burst) | LFPS ctrl: FFE save → EIDLE_OFF → 33-cycle settle → pattern_en → EIDLE_ON → FFE restore |
| Polling.RxEQ entry | Power FSM: EIDLE_OFF write (TxElecIdle goes 0) |
| All data phases | txpath: width adapt + valid gating |
| Any power transition | Power FSM: EIDLE CSR + PMA/PCS reset management |

---

## Waveform: Polling.LFPS Burst Detail

```
MAC signals:
                 ┌─ Burst 1 ──────┐    ┌─ Burst 2 ─...
TxDetectRx:  ────┘                 └────┘
TxElecIdle:  ════════════════1══════════════════════════
TxDataValid: ────┐  ┌────────┐  ┌──────┐  ┌────────...
             ────┘  └────────┘  └──────┘  └────────
TxData[39:0]:    FF..FF 00..00 FF..FF 00..00 ...

PHY internal:
LFPS FSM:    IDLE│FFE_SAVE│EIDLE_OFF│SETTLE│ACTIVE│EIDLE_ON│FFE_RESTORE│IDLE│...
EIDLE CSR:   ON  │  ON    │  OFF    │ OFF  │ OFF  │  ON    │    ON     │ ON │
lfps_patt_en: 0  │  0     │  0      │  0   │  1   │  0     │    0     │ 0  │

Wire:        ──────idle──────────────┐┌─┐┌─┐┌─┐┌──────idle──────...
                                     └┘ └┘ └┘ └┘
                                     LFPS 15.6 MHz
                                     ← tBurst →
                         ←──────── tRepeat ────────→
```
