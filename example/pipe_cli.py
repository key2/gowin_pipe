#!/usr/bin/env python3
"""PIPE SerDes bring-up CLI — talk to the FPGA over UART.

Speaks the binary protocol from top.py:
  'T'     → 0x55 (ping)
  'S'     → 8 status bytes (flags + rx_data snapshot)
  'P' N   → set PowerDown to N
  'D'     → receiver detection (0x03=detected, 0x00=not, 0xFF=timeout)
  'E' N   → set TxElecIdle (0=off, 1=on)
  'F'     → run full LTSSM sequence through Polling.RxEQ

UART ports:
  uart0 (/dev/ttyUSB5) — command/response
  uart1 (/dev/ttyUSB4) — debug trace (state tags during 'F')
  uart2 (/dev/ttyUSB3) — RX data snapshots (5-byte packets)

Usage:
    python pipe_cli.py                          # interactive
    python pipe_cli.py /dev/ttyUSB5             # specify port
    python pipe_cli.py /dev/ttyUSB5 ltssm       # run LTSSM
    python pipe_cli.py /dev/ttyUSB5 detect
    python pipe_cli.py /dev/ttyUSB5 status
    python pipe_cli.py /dev/ttyUSB5 monitor
"""

import sys
import time
import threading
import serial


# ── Names ─────────────────────────────────────────────────────
POWER_STATES = {0: "P0 (active)", 1: "P1 (standby)", 2: "P2 (low-power)", 3: "P3 (off)"}

RATE_FSM = {
    0: "IDLE",
    1: "ASSERT_RESET",
    2: "WRITE_REGS",
    3: "WAIT_PLL",
    4: "DEASSERT_RST",
    5: "WAIT_ACK",
}

RXDET_FSM = {
    0: "IDLE",
    1: "START_PULSE",
    2: "WAIT_PULSE",
    3: "END_PULSE",
    4: "READ_RESULT",
    5: "REPORT",
    6: "WAIT_DEASSERT",
}

LFPS_FSM = {
    0: "IDLE",
    1: "FFE_SAVE",
    2: "EIDLE_OFF",
    3: "LFPS_ACTIVE",
    4: "EIDLE_ON",
    5: "FFE_RESTORE",
}

LTSSM_TAGS = {
    ord("I"): "INIT_WAIT",
    ord("E"): "EIDLE",
    ord("D"): "DETECT",
    ord("P"): "P0_ENTER",
    ord("B"): "LFPS_BURST",
    ord("G"): "LFPS_GAP",
    ord("H"): "LFPS_DONE",
    ord("Q"): "RXEQ_ENTER",
    ord("C"): "CDR_WAIT",
    ord("c"): "CDR_TIMEOUT",
    ord("U"): "MONITOR",
    ord("R"): "RESULT",
    ord("X"): "FAIL",
}

# ANSI colors
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
BOLD = "\033[1m"
DIM = "\033[2m"
RST = "\033[0m"


def open_port(path, baud=115200, timeout=1.0):
    s = serial.Serial(path, baudrate=baud, timeout=timeout)
    s.reset_input_buffer()
    return s


def ping(ser):
    ser.reset_input_buffer()
    ser.write(b"T")
    resp = ser.read(1)
    if resp == b"\x55":
        print("PING: OK (0x55)")
        return True
    elif len(resp) == 0:
        print("PING: TIMEOUT")
        return False
    else:
        print(f"PING: UNEXPECTED 0x{resp[0]:02X}")
        return False


def read_status(ser):
    ser.reset_input_buffer()
    ser.write(b"S")
    data = ser.read(8)
    if len(data) < 8:
        print(f"STATUS: TIMEOUT (got {len(data)}/8 bytes)")
        return None

    b = list(data)
    status = {
        "power_state": b[0] & 0x0F,
        "rx_elec_idle": bool(b[0] & 0x10),
        "rx_valid": bool(b[0] & 0x20),
        "phy_status": bool(b[0] & 0x40),
        "cdr_lock": bool(b[0] & 0x80),
        "rxdet_fsm": b[1] & 0x0F,
        "rate_fsm": (b[1] >> 4) & 0x0F,
        "io_hash": b[2],
        "pll_lock": bool(b[3] & 0x01),
        "init_done": bool(b[3] & 0x02),
        "lfps_fsm": b[3] >> 2 & 0x03,
        "rx_data": (b[4] | (b[5] << 8) | (b[6] << 16) | (b[7] << 24)),
        "raw": data.hex(),
    }
    return status


def print_status(st):
    if st is None:
        return

    pwr = POWER_STATES.get(st["power_state"], f"?({st['power_state']})")
    rate_name = RATE_FSM.get(st["rate_fsm"], f"?({st['rate_fsm']})")
    rxdet_name = RXDET_FSM.get(st["rxdet_fsm"], f"?({st['rxdet_fsm']})")
    lfps_name = LFPS_FSM.get(st["lfps_fsm"], f"?({st['lfps_fsm']})")

    print("┌─────────────────────────────────────────┐")
    print("│         PIPE SerDes Status               │")
    print("├─────────────────────────────────────────┤")
    print(f"│  PLL Lock       : {'YES' if st['pll_lock'] else 'no':>20s}  │")
    print(f"│  CDR Lock       : {'YES' if st['cdr_lock'] else 'no':>20s}  │")
    print(f"│  Init Done      : {'YES' if st['init_done'] else 'no':>20s}  │")
    print(f"│  Power State    : {pwr:>20s}  │")
    print(f"│  RxValid        : {'YES' if st['rx_valid'] else 'no':>20s}  │")
    print(f"│  RxElecIdle     : {'YES' if st['rx_elec_idle'] else 'no':>20s}  │")
    print(f"│  PhyStatus      : {'PULSE' if st['phy_status'] else '-':>20s}  │")
    print(f"│  Rate FSM       : {rate_name:>20s}  │")
    print(f"│  RxDet FSM      : {rxdet_name:>20s}  │")
    print(f"│  LFPS FSM       : {lfps_name:>20s}  │")
    print(f"│  IO Hash        :         0x{st['io_hash']:02X}          │")
    print(f"│  RX Data        :   0x{st['rx_data']:08X}          │")
    print(f"│  Raw            :   {st['raw']}  │")
    print("└─────────────────────────────────────────┘")


def set_power(ser, state):
    ser.reset_input_buffer()
    ser.write(b"P" + bytes([state & 0x0F]))
    resp = ser.read(1)
    if resp == b"\x00":
        print(f"POWER: Set to {POWER_STATES.get(state, state)} — OK")
    elif len(resp) == 0:
        print("POWER: TIMEOUT")
    else:
        print(f"POWER: response 0x{resp[0]:02X}")


def trigger_detect(ser):
    ser.reset_input_buffer()
    ser.write(b"D")
    resp = ser.read(1)
    if len(resp) == 0:
        print("DETECT: TIMEOUT (no response)")
        return
    val = resp[0]
    if val == 0x03:
        print(
            f"DETECT: {GREEN}{BOLD}RECEIVER DETECTED{RST} (cable plugged) — 0x{val:02X}"
        )
    elif val == 0x00:
        print(f"DETECT: {YELLOW}No receiver{RST} (cable unplugged) — 0x{val:02X}")
    elif val == 0xFF:
        print(f"DETECT: {RED}TIMEOUT{RST} — FSM did not complete (0x{val:02X})")
    else:
        print(f"DETECT: unexpected 0x{val:02X}")
    time.sleep(0.01)
    st = read_status(ser)
    if st:
        print(
            f"  RxDet FSM: {RXDET_FSM.get(st['rxdet_fsm'], '?')}  |  Power: {POWER_STATES.get(st['power_state'], '?')}"
        )


def set_eidle(ser, state):
    ser.reset_input_buffer()
    ser.write(b"E" + bytes([state & 0x01]))
    resp = ser.read(1)
    eidle_str = "ON (idle)" if state else "OFF (active)"
    if resp == b"\x00":
        print(f"EIDLE: Set to {eidle_str} — OK")
    elif len(resp) == 0:
        print("EIDLE: TIMEOUT")
    else:
        print(f"EIDLE: response 0x{resp[0]:02X}")


def run_ltssm(ser, dbg_port=None, snap_port=None):
    """Run the full LTSSM sequence ('F' command).

    Optionally opens debug trace (dbg_port) and RX snapshot (snap_port)
    readers in background threads.
    """
    stop_event = threading.Event()

    # Debug trace reader thread (UART1)
    def dbg_reader(port):
        try:
            s = open_port(port, timeout=0.1)
            s.reset_input_buffer()
            print(f"{DIM}[dbg] Listening on {port}{RST}")
            while not stop_event.is_set():
                data = s.read(1)
                if data:
                    tag = data[0]
                    name = LTSSM_TAGS.get(tag, f"?({tag:02X})")
                    color = (
                        RED
                        if tag == ord("X")
                        else GREEN
                        if tag == ord("H") or tag == ord("U")
                        else CYAN
                    )
                    print(f"  {DIM}[dbg]{RST} {color}{name}{RST} (0x{tag:02X})")
            s.close()
        except Exception as e:
            print(f"  {RED}[dbg] Error: {e}{RST}")

    # RX data snapshot reader thread (UART2)
    def snap_reader(port):
        try:
            s = open_port(port, timeout=0.1)
            s.reset_input_buffer()
            print(f"{DIM}[snap] Listening on {port}{RST}")
            while not stop_event.is_set():
                data = s.read(5)
                if len(data) == 5:
                    rxd = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
                    flags = data[4]
                    cdr = bool(flags & 0x08)
                    pll = bool(flags & 0x04)
                    sig = bool(flags & 0x10)
                    rxvld = bool(flags & 0x20)
                    init = bool(flags & 0x40)
                    color = GREEN if cdr else YELLOW
                    print(
                        f"  {DIM}[snap]{RST} rx_data={color}0x{rxd:08X}{RST}  "
                        f"CDR={'Y' if cdr else 'n'}  PLL={'Y' if pll else 'n'}  "
                        f"SIG={'Y' if sig else 'n'}  RxVld={'Y' if rxvld else 'n'}  "
                        f"Init={'Y' if init else 'n'}"
                    )
            s.close()
        except Exception as e:
            print(f"  {RED}[snap] Error: {e}{RST}")

    threads = []
    if dbg_port:
        t = threading.Thread(target=dbg_reader, args=(dbg_port,), daemon=True)
        t.start()
        threads.append(t)
    if snap_port:
        t = threading.Thread(target=snap_reader, args=(snap_port,), daemon=True)
        t.start()
        threads.append(t)

    time.sleep(0.2)  # Let readers initialize

    print(f"\n{BOLD}Starting LTSSM sequence...{RST}")
    ser.reset_input_buffer()
    ser.write(b"F")

    # Wait for result byte (may take up to ~500 ms for full sequence)
    ser.timeout = 2.0
    resp = ser.read(1)
    ser.timeout = 1.0

    if len(resp) == 0:
        print(f"\n{RED}LTSSM: TIMEOUT — no response{RST}")
    else:
        val = resp[0]
        if val == 0xFE:
            print(f"\n{RED}{BOLD}LTSSM: FAILED{RST} (0x{val:02X})")
        else:
            cdr = bool(val & 0x01)
            pll = bool(val & 0x02)
            init = bool(val & 0x04)
            rx_cnt = (val >> 4) & 0x0F

            cdr_str = f"{GREEN}YES{RST}" if cdr else f"{RED}no{RST}"
            pll_str = f"{GREEN}YES{RST}" if pll else f"{RED}no{RST}"
            init_str = f"{GREEN}YES{RST}" if init else f"{RED}no{RST}"

            print(f"\n{BOLD}LTSSM Result:{RST}")
            print(f"  CDR Lock  : {cdr_str}")
            print(f"  PLL Lock  : {pll_str}")
            print(f"  Init Done : {init_str}")
            print(f"  LFPS RX   : {rx_cnt} bursts received")
            print(f"  Raw       : 0x{val:02X}")

    # Give debug readers a moment to flush
    time.sleep(0.3)
    stop_event.set()
    for t in threads:
        t.join(timeout=1.0)

    # Follow up with status
    time.sleep(0.05)
    st = read_status(ser)
    print_status(st)


def monitor(ser, interval=0.5):
    print("Monitoring (Ctrl-C to stop)...\n")
    try:
        while True:
            st = read_status(ser)
            if st:
                pwr = POWER_STATES.get(st["power_state"], "?")
                flags = []
                if st["pll_lock"]:
                    flags.append("PLL")
                if st["cdr_lock"]:
                    flags.append("CDR")
                if st["init_done"]:
                    flags.append("Init")
                if st["rx_valid"]:
                    flags.append("RxVld")
                if st["phy_status"]:
                    flags.append("PhySt")
                if st["rx_elec_idle"]:
                    flags.append("EI")
                flag_str = "+".join(flags) if flags else "none"
                rxdet_name = RXDET_FSM.get(st["rxdet_fsm"], "?")
                lfps_name = LFPS_FSM.get(st["lfps_fsm"], "?")
                print(
                    f"  {pwr:<16s}  flags={flag_str:<24s}  "
                    f"rxdet={rxdet_name:<14s}  lfps={lfps_name:<12s}  "
                    f"rx=0x{st['rx_data']:08X}  hash=0x{st['io_hash']:02X}"
                )
            else:
                print("  (timeout)")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")


def interactive(ser):
    print(
        "PIPE SerDes CLI — commands: ping, status, power N, detect, eidle N, ltssm, monitor, quit"
    )
    while True:
        try:
            cmd = input("\npipe> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd:
            continue
        parts = cmd.split()
        verb = parts[0]

        if verb in ("q", "quit", "exit"):
            break
        elif verb in ("t", "ping"):
            ping(ser)
        elif verb in ("s", "status"):
            st = read_status(ser)
            print_status(st)
        elif verb in ("p", "power"):
            if len(parts) < 2:
                print("Usage: power N  (N=0..3)")
            else:
                set_power(ser, int(parts[1]))
        elif verb in ("d", "detect"):
            trigger_detect(ser)
        elif verb in ("e", "eidle"):
            if len(parts) < 2:
                print("Usage: eidle N  (0=off, 1=on)")
            else:
                set_eidle(ser, int(parts[1]))
        elif verb in ("f", "ltssm"):
            dbg = parts[1] if len(parts) > 1 else "/dev/ttyUSB4"
            snap = parts[2] if len(parts) > 2 else "/dev/ttyUSB3"
            run_ltssm(ser, dbg_port=dbg, snap_port=snap)
        elif verb in ("m", "monitor"):
            interval = float(parts[1]) if len(parts) > 1 else 0.5
            monitor(ser, interval)
        else:
            print(f"Unknown: {verb}")
            print(
                "Commands: ping, status, power N, detect, eidle N, ltssm, monitor, quit"
            )


def find_port():
    import glob

    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    candidates.sort()
    if not candidates:
        return None
    for c in candidates:
        if "USB5" in c:
            return c
    return candidates[-1]


def main():
    args = sys.argv[1:]

    if args and not args[0].startswith("-") and "/" in args[0]:
        port_path = args.pop(0)
    else:
        port_path = find_port()
        if port_path is None:
            print("No serial port found. Usage: pipe_cli.py /dev/ttyUSBx [command]")
            sys.exit(1)
        print(f"Auto-detected port: {port_path}")

    ser = open_port(port_path)
    print(f"Opened {port_path} @ 115200")

    if not args:
        if ping(ser):
            st = read_status(ser)
            print_status(st)
        interactive(ser)
    else:
        verb = args[0].lower()
        if verb == "ping":
            ping(ser)
        elif verb == "status":
            st = read_status(ser)
            print_status(st)
        elif verb == "power":
            set_power(ser, int(args[1]) if len(args) > 1 else 0)
        elif verb == "detect":
            trigger_detect(ser)
        elif verb == "eidle":
            set_eidle(ser, int(args[1]) if len(args) > 1 else 1)
        elif verb in ("ltssm", "f"):
            dbg = args[1] if len(args) > 1 else "/dev/ttyUSB4"
            snap = args[2] if len(args) > 2 else "/dev/ttyUSB3"
            run_ltssm(ser, dbg_port=dbg, snap_port=snap)
        elif verb == "monitor":
            monitor(ser, float(args[1]) if len(args) > 1 else 0.5)
        else:
            print(f"Unknown command: {verb}")

    ser.close()


if __name__ == "__main__":
    main()
