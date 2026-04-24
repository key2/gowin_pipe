# gowin_pipe LFPS — Why the Host Never Sees Our LFPS

## Executive Summary

The `gowin_pipe` Amaranth PIPE adapter has **six critical bugs** in its LFPS
TX path that, in combination, prevent any valid LFPS signal from reaching the
host. The host computer never sees the Polling.LFPS handshake and therefore
never advances past Rx.Detect into Polling, so the link never trains.

Each bug is described below with a side-by-side comparison to the working
`usb30_phy_modules` Verilog reference.

---

## BUG 1 — WRONG FFE VALUES: TX amplitude is wrong during LFPS (CRITICAL)

This is likely the **single biggest reason** the host ignores our LFPS.

### What the spec requires

Per USB 3.2 Table 6-29, LFPS requires **800–1200 mV peak-peak** differential
swing. This is achieved by configuring the TX FFE for **no de-emphasis** (flat
response, full amplitude).

### What the working design does

The Verilog `upar_csr` during LFPS (`FSM_FFE_WRITE`, `ffe_en_d1=1`):

```
FFE_0 = 0x0000F000   ← NEVER CHANGED (keeps init value = full main cursor)
FFE_1 = 0x00000000   ← No de-emphasis (same as init)
FFE_2 = 0x00000110   ← Load coefficients (same as init)
```

The key: **FFE_0 is never touched during LFPS**. The working design only
toggles FFE_1 between `0x0` (LFPS, no de-emphasis) and `0x00000805` (normal
data, with de-emphasis). FFE_0 stays at `0x0000F000` at all times.

*(upar_csr.v:716–743)*

### What gowin_pipe does (WRONG)

`lfps_ffe_regs()` in `csr_map.py:831-834`:

```python
(lc.tx_ffe_0, 0x0000F000, 0x00000040, "TX FFE_0"),  # init → 0x40 !!
(lc.tx_ffe_1, 0x00000000, 0x00000001, "TX FFE_1"),  # init → 0x01 !!
(lc.tx_ffe_2, 0x00000110, 0x00000003, "TX FFE_2"),  # init → 0x03 !!
```

During LFPS, gowin_pipe writes:

| Register | Working design | gowin_pipe (BROKEN) |
|---|---|---|
| FFE_0 | `0x0000F000` (unchanged) | `0x00000040` (main cursor destroyed) |
| FFE_1 | `0x00000000` (no de-emphasis) | `0x00000001` (wrong value) |
| FFE_2 | `0x00000110` (load coefficients) | `0x00000003` (wrong control bits) |

**Impact**: FFE_0 controls the main cursor coefficient. Changing it from
`0xF000` to `0x40` drastically reduces the TX driver output amplitude. The
LFPS signal will be far below the 800 mV minimum. The host's analog LFPS
detector will not trigger.

FFE_2 = `0x03` instead of `0x0110` means the "load new coefficients" control
bit pattern is wrong — the new coefficients may not even take effect.

### Fix

The LFPS FFE values should match what the working design uses — which is
**the init-time values** (no de-emphasis, full amplitude):

```python
def lfps_ffe_regs(quad=0, lane=0):
    return [
        (lc.tx_ffe_0, 0x0000F000, 0x0000F000, "TX FFE_0"),  # UNCHANGED
        (lc.tx_ffe_1, 0x00000000, 0x00000000, "TX FFE_1"),   # No de-emphasis
        (lc.tx_ffe_2, 0x00000110, 0x00000110, "TX FFE_2"),   # Load coefficients
    ]
```

Or better: since the LFPS values are identical to the normal/init values,
**skip the FFE save/restore entirely** and only toggle de-emphasis when
switching between LFPS and normal data modes. The working design only writes
FFE_1 and a reload of FFE_2.

---

## BUG 2 — RACE CONDITION: Power FSM writes EIDLE_ON during the LFPS burst (CRITICAL)

### The race

When Polling.LFPS triggers from P0 (MAC sets `TxElecIdle=1` +
`TxDetectRx=1`), **two FSMs react simultaneously on the same clock edge**:

```
Clock N:
  LFPS FSM: state=IDLE, sees trigger → m.next = "FFE_SAVE"
  Power FSM: state=P0_ACTIVE, sees eidle_active=0 & tx_elec_idle=1
             AND lfps_active=0 (still IDLE this cycle!)
             → m.next = "P0_EIDLE_ON"

Clock N+1:
  LFPS FSM: state=FFE_SAVE, lfps_active=1 (combinational)
  Power FSM: state=P0_EIDLE_ON, writing EIDLE_ON to CSR
```

The `lfps_active` guard in `P0_ACTIVE` (`~self.lfps_active`) is checked
**before** the LFPS FSM transitions out of IDLE. On the trigger cycle,
`lfps_active` is still 0, so the guard does not block the Power FSM.

*(pipe_power.py:285–288, pipe_lfps.py:216–220)*

### What happens next

```
LFPS: FFE_SAVE (drp_lock_req=1, writing FFE CSRs)
Power: P0_EIDLE_ON (waiting for DRP — LFPS has the lock)

LFPS completes FFE → LFPS_EIDLE_OFF → writes EIDLE_OFF (0x07)
LFPS → LFPS_ACTIVE (drp_lock_req released! No DRP activity!)

Power FSM finally gets DRP bus → writes EIDLE_ON (0x01)
```

**The Power FSM writes EIDLE_ON (0x01) right after the LFPS controller wrote
EIDLE_OFF (0x07), immediately putting the TX driver back into electrical
idle.** The LFPS burst is killed before it even starts.

### Why the working design doesn't have this problem

The Verilog `usb_pipe_interface` is a **single FSM** that controls both EIDLE
and LFPS. When the FSM enters `LFPS_0/1/2/3`, it is the sole owner of
`tx_eidle`. There are no concurrent FSMs fighting over the EIDLE CSR.

*(usb_pipe_interface.v:480–783 — all in one always block)*

### Fix

The Power FSM must not transition to `P0_EIDLE_ON` when the LFPS trigger
condition is present. Either:

1. Add: `& ~(self.tx_elec_idle & self.tx_detect_rx)` to the P0_EIDLE_ON
   guard, or
2. Register the LFPS trigger one cycle earlier and use it as a combinational
   block in the Power FSM, or
3. Merge EIDLE tracking into the LFPS controller when the LFPS trigger is
   active.

---

## BUG 3 — NO SETTLING TIME: EIDLE OFF → LFPS pattern starts instantly (SERIOUS)

### What the working design does

The Verilog FSM has a **33-cycle staged EIDLE release** in `LFPS_1`:

```
Cycles  0–15:  tx_eidle = 1  (still idle, driver powering up)
Cycles 16–33:  tx_eidle = 0  (driver active, stabilizing)
Cycle 34:      → LFPS_2      (pattern starts)
```

*(usb_pipe_interface.v:625–645)*

This 33-cycle delay (~264 ns at 125 MHz) lets the SerDes TX analog driver:
- Power up its output stage
- Settle the bias currents
- Stabilize the common-mode voltage

### What gowin_pipe does (BROKEN)

```
FFE_SAVE:      Write 3 FFE CSRs (drp_lock_req=1)
LFPS_EIDLE_OFF: Write EIDLE_OFF → drp_ready fires
                → IMMEDIATE transition to LFPS_ACTIVE (ZERO delay)
```

*(pipe_lfps.py:245–255)*

The LFPS pattern starts on the very next cycle after the EIDLE_OFF DRP write
is acknowledged. The TX driver hasn't had time to power up. The first several
hundred nanoseconds of the "burst" are at wrong/undefined amplitude.

### Fix

Add a settling counter between LFPS_EIDLE_OFF and LFPS_ACTIVE, matching the
working design's 33-cycle delay. A new state `LFPS_SETTLE` should count to
~33 cycles before releasing to `LFPS_ACTIVE`.

---

## BUG 4 — PATTERN SENT BEFORE EIDLE_OFF: data overrides start too early (SERIOUS)

### What the working design does

The Verilog `usb_pipe_interface` only starts the all-1s/all-0s pattern in
`LFPS_2`, **after** `tx_eidle` has been deasserted for 18 cycles. Before
that (in LFPS_0 and LFPS_1), it sends an idle pattern:

```verilog
// LFPS_0 and LFPS_1: idle pattern, NOT LFPS square wave
RxData  = 32'hAAAAAAAA;
RxDataK = 4'b0000;
RxDataN = 4'b1111;
```

*(usb_pipe_interface.v:611–614, 634–637)*

### What gowin_pipe does (BROKEN)

`lfps_active` is asserted in both `FFE_SAVE` and `LFPS_EIDLE_OFF` states:

```python
# FFE_SAVE:
m.d.comb += self.lfps_active.eq(1),  # Pattern starts HERE

# LFPS_EIDLE_OFF:
m.d.comb += self.lfps_active.eq(1),  # Pattern still going
```

*(pipe_lfps.py:229–234, 247–253)*

Since `lfps_active=1`, the `PIPELFPSGen` immediately starts generating the
all-1s/all-0s square wave on `tx_data`. But the EIDLE CSR is still set to
`0x01` (idle) during FFE_SAVE! The SerDes TX driver is in electrical idle
and **ignores** the data on its input pins.

The pattern is being generated, but it goes nowhere. By the time EIDLE_OFF
is actually written and takes effect, the timing of the first burst is
corrupted.

### Fix

Split `lfps_active` into two signals:
- `lfps_busy` — for DRP lock/guard purposes (asserted in FFE_SAVE through
  LFPS_ACTIVE)
- `lfps_pattern_en` — for the PIPELFPSGen (only asserted in LFPS_ACTIVE,
  after EIDLE_OFF + settling)

Or: only assert `lfps_active` in `LFPS_ACTIVE` state, and use a separate
`lfps_drp_lock` for DRP-level coordination.

---

## BUG 5 — NO EIDLE_ON AT BURST END: gowin_pipe never writes EIDLE_ON to end the burst (MODERATE)

### What the working design does

In `LFPS_2`, when `lfps_cnt <= 18`, the working design **explicitly writes
EIDLE_ON** via `tx_eidle <= 1'b1`:

```verilog
// usb_pipe_interface.v:666-669
if(lfps_cnt <= 18)
    tx_eidle <= 1'b1;   // Near end: enter idle
else
    tx_eidle <= 1'b0;   // Still transmitting
```

This causes the `upar_csr` to write `EIDLE_ON` to the CSR, cleanly ending
the burst. The transmitter transitions to electrical idle with a clean edge.

### What gowin_pipe does (PARTIALLY BROKEN)

The comment in `pipe_lfps.py:264–270` explicitly says:

```python
# On stop, go directly to FFE_RESTORE — do NOT write
# EIDLE_ON here. The adapter handles eidle DRP writes
# based on pipe.tx_elec_idle.
```

So nobody writes EIDLE_ON at the burst boundary. The LFPS controller goes
`LFPS_ACTIVE → FFE_RESTORE`, and during FFE_RESTORE, `lfps_active=0`. The
Power FSM may eventually write EIDLE_ON, but only after:
1. The DRP mux arbitration (FFE_RESTORE writes are still competing)
2. The Power FSM detects `~eidle_active & tx_elec_idle` (may take cycles)

This means the burst end is **smeared** — the TX driver stays active for an
indeterminate number of cycles after the LFPS pattern stops. The tBurst
timing becomes unpredictable.

### Fix

The LFPS controller should write EIDLE_ON before transitioning to
FFE_RESTORE, creating a clean burst boundary.

---

## BUG 6 — txpath GATES DATA IN NON-P0 STATES (MODERATE)

### The issue

`pipe_txpath.py:81-83`:

```python
m.d.comb += self.quad_tx_vld.eq(
    self.pipe_tx_data_valid & in_p0 & ~self.pipe_tx_elec_idle
)
```

Where `in_p0 = self.pipe_power_down == 0`.

For P1 LFPS (wake-up LFPS from low-power state), the MAC sets
`PowerDown=0b0001` (P1). The `in_p0` check fails, so `quad_tx_vld=0`.
**The LFPS pattern never reaches the SerDes FIFO.**

This doesn't affect the initial Polling.LFPS (which happens in P0), but it
completely breaks U1/U2 exit LFPS and any LFPS signaling from P1/P2/P3
states.

### Fix

`quad_tx_vld` should also be asserted during LFPS active, regardless of
power state. The simplest fix:

```python
m.d.comb += self.quad_tx_vld.eq(
    (self.pipe_tx_data_valid & in_p0 & ~self.pipe_tx_elec_idle)
    | self.lfps_active  # Always pass LFPS data
)
```

This requires routing `lfps_active` to the txpath.

---

## Side-by-Side: Full LFPS Sequence Comparison

```
WORKING (usb30_phy_modules)         BROKEN (gowin_pipe)
═══════════════════════════         ═══════════════════
                                    
1. P0_ACTIVE                        1. P0_ACTIVE (pipe_power.py)
   - Single FSM controls all           - Power FSM + LFPS FSM separate
   - MAC sets TxEIdle=1,TxDetRx=1     - Same trigger condition
                                    
2. Check tx_ffe (FFE configured?)    2. lfps trigger fires...
   - Yes → skip to LFPS_1             SIMULTANEOUSLY:
   - No → LFPS_0 (configure FFE)       - LFPS: IDLE → FFE_SAVE ←BUG2
                                        - Power: P0 → P0_EIDLE_ON ←BUG2
                                    
3. LFPS_0: Write FFE (NO de-emph)   3. FFE_SAVE: Write WRONG FFE ←BUG1
   FFE_1 = 0x0 (keep full swing)       FFE_0=0x40, FFE_1=0x01, FFE_2=0x03
   Wait for ack                         lfps_active=1 → pattern starts! ←BUG4
                                        (but EIDLE still ON → pattern lost)
                                    
4. LFPS_1: Gradual EIDLE release     4. LFPS_EIDLE_OFF: Write EIDLE_OFF
   Cycles 0-15: tx_eidle=1             Single DRP write, wait for ack
   Cycles 16-33: tx_eidle=0            → immediate transition ←BUG3
   (CSR writes happen via upar_csr)     (zero settling time)
                                    
5. LFPS_2: LFPS pattern active       5. LFPS_ACTIVE:
   cnt1[2] → FF/00 alternating          lfps_gen: all-1s/all-0s pattern
   tx_eidle managed per lfps_cnt         BUT Power FSM writes EIDLE_ON
   EIDLE_ON when lfps_cnt≤18            → TX driver goes idle! ←BUG2
                                         Pattern is on data bus but TX
                                         driver is in electrical idle.
                                    
6. LFPS_3: Cleanup                   6. FFE_RESTORE (lfps_active=0)
   Idle pattern on bus                   No EIDLE_ON written ←BUG5
   Return to Px                          Normal FFE values restored
                                         Pattern stops (no clean edge)
```

---

## Priority-Ordered Fix List

| Priority | Bug | Fix | Effort |
|---|---|---|---|
| **P0** | BUG 1: Wrong FFE values | Change `lfps_ffe_regs()` to use init values | Trivial |
| **P0** | BUG 2: Race with Power FSM | Guard P0_EIDLE_ON against LFPS trigger | Small |
| **P1** | BUG 3: No settling time | Add ~33-cycle delay state after EIDLE_OFF | Small |
| **P1** | BUG 4: Pattern before EIDLE_OFF | Split lfps_active into busy/pattern_en | Small |
| **P2** | BUG 5: No EIDLE_ON at burst end | Write EIDLE_ON before FFE_RESTORE | Small |
| **P2** | BUG 6: txpath P0 gate | Add lfps_active bypass in txpath | Trivial |
