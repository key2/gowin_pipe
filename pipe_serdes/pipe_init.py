"""SerDes power-on initialization FSM.

Writes the 11 essential CSR registers that configure the TX driver (FFE),
global CDR config, and idle detect filters.  This sequence must complete
before the PLL can lock.

Reproduced from the Gowin USB3 PHY ``upar_csr`` module's ``FSM_INIT``
state (usb3_0_phy_decrypted.v).  The 11 writes are always identical
in structure for every device variant; only the per-lane addresses
(Part A + C) vary with quad/lane selection.  The two global writes
(Part B: 0x9083F8 and 0x908830) are fixed addresses for all devices.

The init FSM runs once after POR and holds the DRP bus lock for the
entire write sequence (11 writes).  All other DRP clients (power,
rate, rxdet, lfps, csr_bridge) are blocked until init_done asserts.
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const, Array


class PIPEInitFSM(Elaboratable):
    """Power-on SerDes initialization FSM.

    Runs a fixed sequence of DRP writes at power-on to configure the TX
    driver FFE taps, idle-detect filter thresholds, and (on 138K devices)
    inter-quad reference-clock routing.  The sequence is built at Python
    construction time from ``csr_init_table()`` so the FSM body is purely
    structural — no runtime address computation.

    The FSM holds ``drp_lock_req`` for the entire write burst so that no
    other DRP client can interleave.  Once the last write is acknowledged
    by ``drp_ready``, the FSM asserts ``init_done`` and parks in DONE
    until a reset pulls it back to IDLE.

    Parameters
    ----------
    config : PIPELaneConfig
        Lane configuration (provides quad, lane for per-lane address
        computation).

    Input Ports
    -----------
    reset_n : Signal(1)
        Active-low reset (directly from POR / system reset).  The FSM
        waits in IDLE while ``reset_n`` is low.
    drp_ready : Signal(1)
        DRP write-accepted strobe from the DRP mux.  Asserted for one
        cycle when the current write has been latched by the DRP fabric.

    Output Ports
    ------------
    init_done : Signal(1)
        Asserted (and held) once the full init sequence has been written.
        The power FSM gates its own ``RESET → EIDLE_EXIT`` transition on
        this signal.
    fsm_state : Signal(4)
        Numeric FSM state for debug / ILA capture:
        0 = IDLE, 1 = WRITE, 2 = DONE.

    DRP Interface
    -------------
    drp_addr : Signal(24)
        DRP address for the current write.
    drp_wrdata : Signal(32)
        DRP write data for the current write.
    drp_wren : Signal(1)
        DRP write enable — asserted while a write is pending.
    drp_lock_req : Signal(1)
        DRP bus lock request — held high from the first write until the
        last write is acknowledged, preventing other clients from
        interleaving.
    """

    def __init__(self, config):
        from gowin_serdes.csr_map import csr_init_table

        self._config = config

        # Build the init table at Python time.
        # All 11 writes are always needed regardless of device variant.
        # The two global CSR writes (0x9083F8, 0x908830) are fixed
        # addresses; the per-lane writes are computed from quad/lane.
        self._init_table = csr_init_table(
            quad=config.quad,
            lane=config.lane,
        )

        # ── Inputs ─────────────────────────────────────────────────────
        self.reset_n = Signal(1, name="init_reset_n")
        self.drp_ready = Signal(1, name="init_drp_ready")

        # ── Outputs ────────────────────────────────────────────────────
        self.init_done = Signal(1, name="init_done")
        self.fsm_state = Signal(4, name="init_fsm_state")

        # ── DRP interface ──────────────────────────────────────────────
        self.drp_addr = Signal(24, name="init_drp_addr")
        self.drp_wrdata = Signal(32, name="init_drp_wrdata")
        self.drp_wren = Signal(1, name="init_drp_wren")
        self.drp_lock_req = Signal(1, name="init_drp_lock_req")

    def elaborate(self, platform):
        m = Module()

        num_writes = len(self._init_table)

        # Build Amaranth Arrays from the Python-time init table.
        # Each entry is (name, addr, data) — we only need addr and data
        # for the hardware; the name is for documentation / debug only.
        addrs = Array([Const(addr, 24) for _, addr, _ in self._init_table])
        datas = Array([Const(data, 32) for _, _, data in self._init_table])

        # Write index counter — counts from 0 to num_writes-1, sized to
        # hold num_writes (one past the last valid index) for the terminal
        # comparison.
        idx = Signal(range(num_writes + 1))

        # ── Combinational defaults ─────────────────────────────────────
        # DRP outputs default to inactive every cycle.  Active states
        # override via m.d.comb inside their FSM branches.
        m.d.comb += [
            self.drp_wren.eq(0),
            self.drp_addr.eq(0),
            self.drp_wrdata.eq(0),
            self.drp_lock_req.eq(0),
            self.init_done.eq(0),
        ]

        # ══════════════════════════════════════════════════════════════
        #  Main FSM
        # ══════════════════════════════════════════════════════════════
        with m.FSM(name="init") as fsm:
            # ── IDLE ───────────────────────────────────────────────────
            # Wait for POR deassertion (reset_n going high).  While in
            # IDLE the write index is held at zero and no DRP activity
            # occurs.
            with m.State("IDLE"):
                m.d.sync += self.fsm_state.eq(0)
                with m.If(self.reset_n):
                    m.d.sync += idx.eq(0)
                    m.next = "WRITE"

            # ── WRITE ──────────────────────────────────────────────────
            # Drive the current (addr, data) pair from the init table
            # onto the DRP bus and assert wren + lock_req.  When the DRP
            # mux acknowledges with drp_ready, advance the index.  After
            # the last entry is acknowledged, transition to DONE.
            with m.State("WRITE"):
                m.d.sync += self.fsm_state.eq(1)
                m.d.comb += [
                    self.drp_lock_req.eq(1),
                    self.drp_addr.eq(addrs[idx]),
                    self.drp_wrdata.eq(datas[idx]),
                    self.drp_wren.eq(1),
                ]
                with m.If(self.drp_ready):
                    with m.If(idx == num_writes - 1):
                        m.next = "DONE"
                    with m.Else():
                        m.d.sync += idx.eq(idx + 1)

            # ── DONE ──────────────────────────────────────────────────
            # Init sequence complete.  Assert init_done so downstream
            # logic (power FSM) can proceed.  The FSM parks here until
            # reset_n is reasserted (low), at which point it returns to
            # IDLE to replay the sequence on the next POR.
            with m.State("DONE"):
                m.d.sync += self.fsm_state.eq(2)
                m.d.comb += self.init_done.eq(1)
                with m.If(~self.reset_n):
                    m.next = "IDLE"

        return m
