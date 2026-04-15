"""PIPE message bus controller.

Implements the M2P (MAC→PHY) decoder and P2M (PHY→MAC) encoder for the
PIPE 8-bit message bus protocol per PIPE Rev 7.1 Section 6.1.4.

Protocol overview:
    The message bus is an 8-bit, PCLK-synchronous interface used for
    indirect register access between the MAC and PHY layers.  The bus
    idles at all-zeros; a transaction begins when a non-zero byte appears.

    Two independent buses exist:
      - M2P (MAC→PHY): carries write/read commands from the MAC.
      - P2M (PHY→MAC): carries read completions and write acknowledgements.

Commands (Table 6-10):
    Cmd[3:0]  Name               Cycles  Direction
    ────────  ─────────────────  ──────  ─────────
    0000      NOP                1       M2P
    0001      write_uncommitted  3       M2P
    0010      write_committed    3       M2P
    0011      read               2       M2P
    0100      read_completion    2       P2M
    0101      write_ack          1       P2M (also M2P for idle fill)

Frame format (byte-per-cycle):
    Write:           [Cmd(7:4) | Addr_hi(3:0)] → [Addr_lo(7:0)] → [Data(7:0)]
    Read:            [Cmd(7:4) | Addr_hi(3:0)] → [Addr_lo(7:0)]
    Read completion: [Cmd(7:4) | 0000]         → [Data(7:0)]
    Write ack / NOP: [Cmd(7:4) | 0000]

Atomic writes:
    The protocol supports atomic multi-register writes.  The MAC issues
    one or more ``write_uncommitted`` commands (buffered but not applied),
    followed by a single ``write_committed`` command.  On receipt of the
    committed write, the PHY applies all buffered writes plus the committed
    write in sequence, then sends a ``write_ack`` on the P2M bus.  The
    write buffer holds a minimum of 5 entries per the PIPE specification.

Supports: NOP, write_uncommitted, write_committed (atomic apply),
read, read_completion, write_ack.

The message bus bridges to an internal register file.  Registers that
map to GTR12 CSR are forwarded to the CSR bridge for DRP translation.
"""

from amaranth.hdl import Signal, Module, Elaboratable, Const, Cat, Array


class PIPEMessageBus(Elaboratable):
    """PIPE message bus protocol engine.

    Decodes M2P (MAC→PHY) bus transactions and generates P2M (PHY→MAC)
    responses.  The engine implements a multi-state FSM that processes
    each command frame according to its cycle count and semantics.

    The write buffer stores ``write_uncommitted`` transactions until a
    ``write_committed`` command triggers atomic application of all pending
    writes followed by the committed write itself.  Each buffered write
    is applied sequentially, waiting for ``csr_wr_ack`` before advancing.
    After all writes complete, a ``write_ack`` frame is transmitted on
    the P2M bus.

    Read commands issue a register read via ``reg_rden`` and wait for
    ``csr_rd_valid``, then transmit a two-cycle ``read_completion``
    frame (command byte + data byte) on the P2M bus.

    Parameters
    ----------
    write_buffer_depth : int
        Number of uncommitted write buffer entries (default 5, per PIPE
        spec minimum).  Each entry stores a 12-bit address and 8-bit data.

    Input Ports
    -----------
    m2p_bus : Signal(8)
        MAC→PHY message bus.  Directly sampled on every PCLK rising edge.
        Byte format: ``[Cmd(7:4) | Payload(3:0)]`` on the first cycle of
        a transaction; subsequent cycles carry address or data bytes.
    reset_n : Signal(1)
        Active-low reset.  When deasserted (low), the FSM returns to IDLE
        and all internal state (including the write buffer) is undefined
        until re-initialised.
    csr_rd_data : Signal(8)
        Read data returned from the CSR bridge / register file.  Sampled
        when ``csr_rd_valid`` is asserted.
    csr_rd_valid : Signal(1)
        Asserted for one cycle when ``csr_rd_data`` contains valid data
        in response to a register read (``reg_rden``).
    csr_wr_ack : Signal(1)
        Asserted for one cycle when a register write (``reg_wren``) has
        been accepted by the CSR bridge.

    Output Ports
    ------------
    p2m_bus : Signal(8)
        PHY→MAC message bus.  Driven to 0x00 when idle; non-zero during
        ``read_completion`` and ``write_ack`` frames.
    reg_addr : Signal(12)
        Register address presented to the CSR bridge.  Formed from the
        4-bit ``Addr_hi`` field (from the command byte) concatenated with
        the 8-bit ``Addr_lo`` byte: ``{Addr_hi[3:0], Addr_lo[7:0]}``.
    reg_wrdata : Signal(8)
        Register write data, valid when ``reg_wren`` is asserted.
    reg_wren : Signal(1)
        Register write enable.  Asserted for one cycle per write during
        the commit sequence (both buffered and committed writes).
    reg_rden : Signal(1)
        Register read enable.  Asserted while waiting for ``csr_rd_valid``
        in the ``RD_EXEC`` state.
    fsm_state : Signal(4)
        Numeric encoding of the current FSM state for debug visibility.

    FSM States
    ----------
    IDLE (0)
        Wait for non-zero byte on M2P bus.  Single-cycle commands (NOP,
        write_ack) are consumed without leaving this state.
    ADDR_LO (1)
        Capture the second byte as the low address.  Route to WR_DATA
        for write commands or RD_EXEC for read commands.
    WR_DATA (2)
        Capture the third byte as write data.  Route to BUFFER_WRITE
        for uncommitted writes or COMMIT_START for committed writes.
    BUFFER_WRITE (3)
        Store the {address, data} pair in the next available write buffer
        slot and increment the buffer count.
    COMMIT_START (4)
        Begin the atomic commit sequence.  If buffered writes exist,
        proceed to COMMIT_BUFFERED; otherwise skip to COMMIT_FINAL.
    COMMIT_BUFFERED (5)
        Apply each buffered write sequentially.  Wait for ``csr_wr_ack``
        before advancing ``commit_idx``.  After all buffered entries are
        applied, proceed to COMMIT_FINAL.
    COMMIT_FINAL (6)
        Apply the committed write itself (the one from the
        ``write_committed`` command).  On ``csr_wr_ack``, clear the write
        buffer and proceed to SEND_WRITE_ACK.
    SEND_WRITE_ACK (7)
        Transmit a single-cycle ``write_ack`` frame (0x50) on the P2M bus,
        then return to IDLE.
    RD_EXEC (8)
        Assert ``reg_rden`` and wait for ``csr_rd_valid``.  Latch the
        read data and proceed to SEND_RD_COMP_CMD.
    SEND_RD_COMP_CMD (9)
        Transmit the ``read_completion`` command byte (0x40) on P2M.
    SEND_RD_COMP_DATA (10)
        Transmit the read data byte on P2M, then return to IDLE.
    """

    # Command encodings (PIPE Rev 7.1 Table 6-10)
    CMD_NOP = 0b0000  # No operation / idle fill
    CMD_WRITE_UNCOMMIT = 0b0001  # Buffer a write (not yet applied)
    CMD_WRITE_COMMIT = 0b0010  # Apply this write + all buffered writes
    CMD_READ = 0b0011  # Read a register
    CMD_READ_COMPLETION = 0b0100  # Return read data (P2M direction)
    CMD_WRITE_ACK = 0b0101  # Acknowledge a committed write (P2M)

    def __init__(self, write_buffer_depth=5):
        """Initialise the message bus controller.

        Parameters
        ----------
        write_buffer_depth : int
            Depth of the uncommitted write buffer.  The PIPE specification
            requires a minimum of 5 entries.  Each entry stores a 12-bit
            register address and 8-bit data value.
        """
        self.write_buffer_depth = write_buffer_depth

        # ── Inputs ─────────────────────────────────────────────────────
        self.m2p_bus = Signal(8, name="m2p_bus")
        self.reset_n = Signal(1, name="msgbus_reset_n")
        self.csr_rd_data = Signal(8, name="csr_rd_data")
        self.csr_rd_valid = Signal(1, name="csr_rd_valid")
        self.csr_wr_ack = Signal(1, name="csr_wr_ack")

        # ── Outputs ────────────────────────────────────────────────────
        self.p2m_bus = Signal(8, name="p2m_bus")
        self.reg_addr = Signal(12, name="msgbus_reg_addr")
        self.reg_wrdata = Signal(8, name="msgbus_reg_wrdata")
        self.reg_wren = Signal(1, name="msgbus_reg_wren")
        self.reg_rden = Signal(1, name="msgbus_reg_rden")
        self.fsm_state = Signal(4, name="msgbus_fsm_state")

    def elaborate(self, platform):
        """Build the message bus decode/encode datapath and FSM.

        The FSM decodes M2P bus frames cycle-by-cycle, buffers uncommitted
        writes, applies committed writes atomically, and generates P2M
        responses (read_completion, write_ack).

        Dynamic indexing into the write buffer uses ``Array`` wrappers
        around the buffer signal lists, which is required by Amaranth for
        runtime-variable index selection in synchronous/combinational
        logic (plain Python lists only support static elaboration-time
        indexing via ``m.If`` chains).
        """
        m = Module()

        # ── Decode M2P bus fields ──────────────────────────────────────
        # First byte of every transaction:
        #   [7:4] = Cmd   — command opcode
        #   [3:0] = Payload — Addr_hi for write/read, or 0000 for others
        m2p_cmd = self.m2p_bus[4:8]  # Bits [7:4]: command opcode
        m2p_data = self.m2p_bus[0:4]  # Bits [3:0]: addr_hi or zeroes

        # ── Latched command state ──────────────────────────────────────
        # These registers hold the decoded fields across multi-cycle
        # transactions so downstream states can reference them.
        cmd_latch = Signal(4)  # Latched command from first byte
        addr_hi = Signal(4)  # Upper 4 bits of 12-bit address
        addr_lo = Signal(8)  # Lower 8 bits of 12-bit address
        full_addr = Signal(12)  # Assembled 12-bit register address
        wr_data = Signal(8)  # Write data (also reused as read temp)

        # ── Write buffer (uncommitted writes) ──────────────────────────
        # Each entry stores a {12-bit address, 8-bit data} pair.  Entries
        # are filled by write_uncommitted commands and drained atomically
        # by write_committed.  Array() wrapping enables dynamic (runtime)
        # indexing by commit_idx during the drain phase.
        buf_addr = Array(
            [Signal(12, name=f"buf_addr_{i}") for i in range(self.write_buffer_depth)]
        )
        buf_data = Array(
            [Signal(8, name=f"buf_data_{i}") for i in range(self.write_buffer_depth)]
        )
        buf_valid = Array(
            [Signal(1, name=f"buf_valid_{i}") for i in range(self.write_buffer_depth)]
        )
        buf_count = Signal(range(self.write_buffer_depth + 1))

        # ── Commit sequencer ───────────────────────────────────────────
        # commit_idx tracks which buffer entry is currently being applied
        # during the COMMIT_BUFFERED state.  committing is a flag used
        # to indicate an atomic commit sequence is in progress.
        commit_idx = Signal(range(self.write_buffer_depth + 1))
        committing = Signal(1)

        # ── Combinational defaults ─────────────────────────────────────
        # All outputs default to inactive/zero every cycle.  States that
        # need to drive these signals override via m.d.comb assignments.
        m.d.comb += [
            self.p2m_bus.eq(0),
            self.reg_wren.eq(0),
            self.reg_rden.eq(0),
            self.reg_addr.eq(0),
            self.reg_wrdata.eq(0),
        ]

        # ══════════════════════════════════════════════════════════════
        #  Main FSM — M2P bus decoder / P2M bus encoder
        # ══════════════════════════════════════════════════════════════
        with m.FSM(name="m2p_decode") as fsm:
            # ── IDLE: Wait for non-zero on M2P bus ─────────────────────
            # The bus idles at 0x00.  Any non-zero value is the start of
            # a new transaction.  Single-cycle commands (NOP, write_ack)
            # are consumed immediately without leaving IDLE.
            with m.State("IDLE"):
                m.d.sync += self.fsm_state.eq(0)
                with m.If(self.m2p_bus != 0):
                    m.d.sync += [
                        cmd_latch.eq(m2p_cmd),
                        addr_hi.eq(m2p_data),
                    ]
                    # NOP and write_ack are single-cycle — stay in IDLE
                    with m.If(
                        (m2p_cmd == self.CMD_NOP) | (m2p_cmd == self.CMD_WRITE_ACK)
                    ):
                        pass  # Consumed; remain in IDLE
                    with m.Else():
                        m.next = "ADDR_LO"

            # ── ADDR_LO: Receive address low byte ─────────────────────
            # Second cycle of multi-cycle commands.  The full M2P byte is
            # the low 8 bits of the register address.  Combined with the
            # latched addr_hi to form the 12-bit address.
            with m.State("ADDR_LO"):
                m.d.sync += [
                    self.fsm_state.eq(1),
                    addr_lo.eq(self.m2p_bus),
                    full_addr.eq(Cat(self.m2p_bus, addr_hi)),
                ]
                with m.If(
                    (cmd_latch == self.CMD_WRITE_UNCOMMIT)
                    | (cmd_latch == self.CMD_WRITE_COMMIT)
                ):
                    m.next = "WR_DATA"
                with m.Elif(cmd_latch == self.CMD_READ):
                    m.next = "RD_EXEC"
                with m.Else():
                    # Unknown command — return to idle
                    m.next = "IDLE"

            # ── WR_DATA: Receive write data byte ──────────────────────
            # Third cycle of write commands.  The full M2P byte is the
            # 8-bit data to be written.
            with m.State("WR_DATA"):
                m.d.sync += [
                    self.fsm_state.eq(2),
                    wr_data.eq(self.m2p_bus),
                ]
                with m.If(cmd_latch == self.CMD_WRITE_UNCOMMIT):
                    m.next = "BUFFER_WRITE"
                with m.Elif(cmd_latch == self.CMD_WRITE_COMMIT):
                    m.next = "COMMIT_START"

            # ── BUFFER_WRITE: Store uncommitted write in buffer ────────
            # Appends the {full_addr, wr_data} pair to the next available
            # buffer slot (indexed by buf_count).  If the buffer is full,
            # the write is silently dropped (overflow condition — the PIPE
            # spec mandates the MAC not exceed the buffer depth).
            with m.State("BUFFER_WRITE"):
                m.d.sync += self.fsm_state.eq(3)
                with m.If(buf_count < self.write_buffer_depth):
                    for i in range(self.write_buffer_depth):
                        with m.If(buf_count == i):
                            m.d.sync += [
                                buf_addr[i].eq(full_addr),
                                buf_data[i].eq(wr_data),
                                buf_valid[i].eq(1),
                            ]
                    m.d.sync += buf_count.eq(buf_count + 1)
                m.next = "IDLE"

            # ── COMMIT_START: Begin atomic commit sequence ─────────────
            # Initiates the drain of all buffered writes followed by the
            # committed write itself.  If no buffered writes exist, skip
            # directly to COMMIT_FINAL to apply only the committed write.
            with m.State("COMMIT_START"):
                m.d.sync += [
                    self.fsm_state.eq(4),
                    commit_idx.eq(0),
                    committing.eq(1),
                ]
                with m.If(buf_count > 0):
                    m.next = "COMMIT_BUFFERED"
                with m.Else():
                    m.next = "COMMIT_FINAL"

            # ── COMMIT_BUFFERED: Apply each buffered write ─────────────
            # Iterates through buffer entries 0..buf_count-1, presenting
            # each {addr, data} pair on the register interface and waiting
            # for csr_wr_ack before advancing.  Uses Array-wrapped signals
            # for dynamic indexing by commit_idx at runtime.
            with m.State("COMMIT_BUFFERED"):
                m.d.sync += self.fsm_state.eq(5)
                # Drive register interface from current buffer entry
                # (Array indexing enables runtime-variable selection)
                m.d.comb += [
                    self.reg_addr.eq(buf_addr[commit_idx]),
                    self.reg_wrdata.eq(buf_data[commit_idx]),
                    self.reg_wren.eq(buf_valid[commit_idx]),
                ]
                # Wait for CSR write acknowledgement, then advance
                with m.If(self.csr_wr_ack | ~buf_valid[commit_idx]):
                    with m.If(commit_idx == buf_count - 1):
                        m.next = "COMMIT_FINAL"
                    with m.Else():
                        m.d.sync += commit_idx.eq(commit_idx + 1)

            # ── COMMIT_FINAL: Apply the committed write itself ─────────
            # This is the write from the write_committed command.  After
            # csr_wr_ack, the entire write buffer is cleared and a
            # write_ack frame is sent on P2M.
            with m.State("COMMIT_FINAL"):
                m.d.sync += self.fsm_state.eq(6)
                m.d.comb += [
                    self.reg_addr.eq(full_addr),
                    self.reg_wrdata.eq(wr_data),
                    self.reg_wren.eq(1),
                ]
                with m.If(self.csr_wr_ack):
                    # Clear the entire write buffer
                    m.d.sync += buf_count.eq(0)
                    for i in range(self.write_buffer_depth):
                        m.d.sync += buf_valid[i].eq(0)
                    m.d.sync += committing.eq(0)
                    m.next = "SEND_WRITE_ACK"

            # ── SEND_WRITE_ACK: Transmit write_ack on P2M bus ─────────
            # Single-cycle frame: Cmd=0101 (write_ack), Payload=0000.
            # Encoded as 0x50 on the 8-bit bus.
            with m.State("SEND_WRITE_ACK"):
                m.d.sync += self.fsm_state.eq(7)
                m.d.comb += self.p2m_bus.eq(0x50)
                m.next = "IDLE"

            # ── RD_EXEC: Issue register read, wait for data ────────────
            # Asserts reg_rden with the target address until the CSR
            # bridge returns valid data via csr_rd_valid + csr_rd_data.
            # The read data is latched into wr_data (reused as a
            # temporary register) for transmission in the read_completion
            # frame.
            with m.State("RD_EXEC"):
                m.d.sync += self.fsm_state.eq(8)
                m.d.comb += [
                    self.reg_addr.eq(full_addr),
                    self.reg_rden.eq(1),
                ]
                with m.If(self.csr_rd_valid):
                    m.d.sync += wr_data.eq(self.csr_rd_data)
                    m.next = "SEND_RD_COMP_CMD"

            # ── SEND_RD_COMP_CMD: Transmit read_completion command ─────
            # First cycle of the two-cycle read_completion frame.
            # Cmd=0100 (read_completion), Payload=0000 → 0x40.
            with m.State("SEND_RD_COMP_CMD"):
                m.d.sync += self.fsm_state.eq(9)
                m.d.comb += self.p2m_bus.eq(0x40)
                m.next = "SEND_RD_COMP_DATA"

            # ── SEND_RD_COMP_DATA: Transmit read data byte ─────────────
            # Second cycle of the read_completion frame.  The full 8-bit
            # bus carries the register read data.
            with m.State("SEND_RD_COMP_DATA"):
                m.d.sync += self.fsm_state.eq(10)
                m.d.comb += self.p2m_bus.eq(wr_data)
                m.next = "IDLE"

        return m
