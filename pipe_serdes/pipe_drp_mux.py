"""Internal DRP multiplexer with locking for PIPE SerDes adapter.

Serializes DRP requests from multiple internal sub-controllers into a single
DRP port output. Supports locking for multi-write atomic sequences.

Priority (highest to lowest):
  0: Init FSM (power-on CSR init sequence, runs once)
  1: Rate change FSM (7-register atomic sequence)
  2: LFPS controller (FFE save/restore + eidle toggle)
  3: Receiver detection (3-step pulse sequence)
  4: Power state FSM (1-2 CSR writes)
  5: Electrical idle controller (single CSR write)
  6: CSR bridge / Message bus (background register access)
"""

from amaranth.hdl import Signal, Module, Elaboratable, Cat, Const, Array


class PIPEDRPMux(Elaboratable):
    """Priority-based DRP multiplexer with exclusive locking.

    Parameters
    ----------
    num_clients : int
        Number of internal DRP clients (default 6).

    Ports (all arrays of num_clients elements):
        req_addr[i]:    Signal(24) - CSR address from client i
        req_wrdata[i]:  Signal(32) - Write data from client i
        req_wren[i]:    Signal(1)  - Write enable from client i
        req_rden[i]:    Signal(1)  - Read enable from client i
        req_lock[i]:    Signal(1)  - Lock request from client i
        req_lock_ack[i]:Signal(1)  - Lock granted to client i (output)
        req_ready[i]:   Signal(1)  - Write accepted for client i (output)
        req_rdvld[i]:   Signal(1)  - Read data valid for client i (output)
        req_rddata[i]:  Signal(32) - Read data for client i (output)

    DRP output port (single):
        drp_addr:       Signal(24) - To external DRP arbiter
        drp_wrdata:     Signal(32)
        drp_wren:       Signal(1)
        drp_strb:       Signal(8)  - Always 0xFF
        drp_rden:       Signal(1)
        drp_ready:      Signal(1)  - From external arbiter
        drp_rdvld:      Signal(1)  - From external arbiter
        drp_rddata:     Signal(32) - From external arbiter

    Debug:
        dbg_owner:      Signal(4)  - Current mux owner index
        dbg_locked:     Signal(1)  - Lock active
    """

    def __init__(self, num_clients=6):
        self.num_clients = num_clients

        # Client-side ports (arrays)
        self.req_addr = [Signal(24, name=f"req_addr_{i}") for i in range(num_clients)]
        self.req_wrdata = [
            Signal(32, name=f"req_wrdata_{i}") for i in range(num_clients)
        ]
        self.req_wren = [Signal(1, name=f"req_wren_{i}") for i in range(num_clients)]
        self.req_rden = [Signal(1, name=f"req_rden_{i}") for i in range(num_clients)]
        self.req_lock = [Signal(1, name=f"req_lock_{i}") for i in range(num_clients)]
        self.req_lock_ack = [
            Signal(1, name=f"req_lock_ack_{i}") for i in range(num_clients)
        ]
        self.req_ready = [Signal(1, name=f"req_ready_{i}") for i in range(num_clients)]
        self.req_rdvld = [Signal(1, name=f"req_rdvld_{i}") for i in range(num_clients)]
        self.req_rddata = [
            Signal(32, name=f"req_rddata_{i}") for i in range(num_clients)
        ]

        # DRP output (single port to external arbiter)
        self.drp_addr = Signal(24, name="drp_out_addr")
        self.drp_wrdata = Signal(32, name="drp_out_wrdata")
        self.drp_wren = Signal(1, name="drp_out_wren")
        self.drp_strb = Signal(8, name="drp_out_strb")
        self.drp_rden = Signal(1, name="drp_out_rden")
        self.drp_ready = Signal(1, name="drp_in_ready")
        self.drp_rdvld = Signal(1, name="drp_in_rdvld")
        self.drp_rddata = Signal(32, name="drp_in_rddata")

        # Debug
        self.dbg_owner = Signal(4, name="dbg_mux_owner")
        self.dbg_locked = Signal(1, name="dbg_mux_locked")

    def elaborate(self, platform):
        m = Module()
        nc = self.num_clients

        # State
        locked = Signal(1)  # DRP bus is locked
        owner = Signal(range(nc))  # Current lock owner
        active = Signal(1)  # A transaction is in progress
        active_client = Signal(range(nc))

        # Debug
        m.d.comb += [
            self.dbg_owner.eq(owner),
            self.dbg_locked.eq(locked),
        ]

        # Strobe always 0xFF
        m.d.comb += self.drp_strb.eq(0xFF)

        # Build request vector
        req = Signal(nc)
        for i in range(nc):
            m.d.comb += req[i].eq(self.req_wren[i] | self.req_rden[i])

        # Priority encoder: find highest-priority (lowest index) active client
        # When locked, only the owner can issue transactions
        selected = Signal(range(nc))
        has_req = Signal(1)

        with m.If(locked):
            # Only lock owner can issue
            m.d.comb += [
                selected.eq(owner),
                has_req.eq(req.bit_select(owner, 1)),
            ]
        with m.Else():
            # Priority encode: client 0 has highest priority
            m.d.comb += has_req.eq(0)
            for i in reversed(range(nc)):  # reversed so lower index overrides
                with m.If(req[i]):
                    m.d.comb += [
                        selected.eq(i),
                        has_req.eq(1),
                    ]

        # Lock management FSM
        with m.If(~locked):
            for i in range(nc):
                with m.If(self.req_lock[i] & (selected == i) & has_req):
                    m.d.sync += [
                        locked.eq(1),
                        owner.eq(i),
                    ]
        with m.Else():
            # Release lock when owner stops requesting it
            req_lock_arr = Array(self.req_lock)
            with m.If(~req_lock_arr[owner]):
                m.d.sync += locked.eq(0)

        # Route lock acknowledgments
        for i in range(nc):
            m.d.comb += self.req_lock_ack[i].eq(locked & (owner == i))

        # Mux: route selected client to DRP output
        with m.If(has_req):
            # Use Switch for clean muxing
            with m.Switch(selected):
                for i in range(nc):
                    with m.Case(i):
                        m.d.comb += [
                            self.drp_addr.eq(self.req_addr[i]),
                            self.drp_wrdata.eq(self.req_wrdata[i]),
                            self.drp_wren.eq(self.req_wren[i]),
                            self.drp_rden.eq(self.req_rden[i]),
                        ]

        # Track active transaction for response routing
        with m.If(self.drp_wren | self.drp_rden):
            m.d.sync += [
                active.eq(1),
                active_client.eq(selected),
            ]
        with m.Elif(self.drp_ready | self.drp_rdvld):
            m.d.sync += active.eq(0)

        # Route responses back to the active client
        for i in range(nc):
            m.d.comb += [
                self.req_ready[i].eq(self.drp_ready & active & (active_client == i)),
                self.req_rdvld[i].eq(self.drp_rdvld & active & (active_client == i)),
                self.req_rddata[i].eq(
                    self.drp_rddata
                ),  # broadcast, only valid when rdvld
            ]

        return m
