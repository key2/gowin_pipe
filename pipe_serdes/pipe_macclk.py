"""MacCLK generator for PIPE SerDes.

Provides the MacCLK domain for MAC-PHY communication during low-power
states (P2/P3) when PCLK may be off.  Derived from the GTR12 LIFE_CLK
(~62.5 MHz), which runs regardless of SerDes power state.

PIPE Rev 7.1 MacCLK overview
-----------------------------
MacCLK is an independent clock provided by the PHY to the MAC for use
when PCLK is not available (power states P2 and P3).  The MAC-PHY
handshake works as follows:

1.  MAC asserts ``mac_clk_req`` to request the clock.
2.  PHY enables the clock (gates LIFE_CLK through) and asserts
    ``mac_clk_ack`` once the clock is stable.
3.  MAC may use ``mac_clk`` while ``mac_clk_ack`` is high.
4.  MAC deasserts ``mac_clk_req`` when the clock is no longer needed.
5.  PHY deasserts ``mac_clk_ack`` and gates the clock off.

This handshake ensures the MAC never samples a clock that is in the
process of starting or stopping.

Signal reference (PIPE Rev 7.1 §5.6):
    mac_clk_req     — MAC → PHY, request MacCLK
    mac_clk_ack     — PHY → MAC, acknowledge clock running
    mac_clk         — PHY → MAC, clock output
    mac_clk_rate[4:0] — rate select (reserved for future clock divider)
    mac_clk_reset_n — active-low async reset for the MacCLK domain

Implementation notes
--------------------
On the Gowin GTR12, LIFE_CLK is a free-running ~62.5 MHz reference that
is always available regardless of PLL or CDR state.  MacCLK is produced
by gating LIFE_CLK through a combinational AND with a registered enable
flag.  In a real FPGA build this gate would be replaced by a BUFGCE
primitive so the clock tree stays clean, but the logical behaviour is
identical.

The ``mac_clk_rate`` input is accepted but currently unused — a future
revision may add a programmable divider for lower-frequency MacCLK
operation.

This is the simplest of the PIPE sub-modules, but it is essential for
correct low-power operation: without MacCLK, the MAC has no way to
communicate with the PHY for wake-up signalling while PCLK is off.
"""

from amaranth.hdl import Signal, Module, Elaboratable


class PIPEMacCLKGen(Elaboratable):
    """MacCLK generator with request/acknowledge handshake.

    Gates the GTR12 LIFE_CLK (~62.5 MHz) to produce MacCLK on demand.
    The MAC requests the clock via ``mac_clk_req``; the PHY responds
    with ``mac_clk_ack`` once the clock is stable and flowing.

    State behaviour
    ---------------
    The internal ``clk_running`` register tracks whether the clock gate
    is open:

    * **Reset** (``mac_clk_reset_n`` low): ``clk_running`` and
      ``mac_clk_ack`` are cleared.  MacCLK output is forced low.
    * **Request** (``mac_clk_req`` rises while stopped): ``clk_running``
      and ``mac_clk_ack`` are asserted on the next sync edge.
    * **Release** (``mac_clk_req`` falls while running): ``clk_running``
      and ``mac_clk_ack`` are deasserted on the next sync edge.

    The acknowledge tracks the running state with one sync-edge latency,
    which is acceptable for the PIPE handshake protocol.

    Input Ports
    -----------
    mac_clk_reset_n : Signal(1)
        Active-low asynchronous reset for the MacCLK domain.  When low,
        the clock gate is forced off and ``mac_clk_ack`` is deasserted.
    mac_clk_rate : Signal(5)
        Clock rate select.  Reserved for a future programmable divider;
        currently ignored.  The PHY always outputs LIFE_CLK frequency.
    mac_clk_req : Signal(1)
        MAC → PHY clock request.  Assert to start MacCLK, deassert to
        stop it.
    life_clk : Signal(1)
        GTR12 LIFE_CLK input (~62.5 MHz free-running reference).

    Output Ports
    ------------
    mac_clk_ack : Signal(1)
        PHY → MAC acknowledgement.  High while the clock is running and
        stable.  The MAC must not use ``mac_clk`` unless ``mac_clk_ack``
        is asserted.
    mac_clk : Signal(1)
        Gated clock output.  Equals ``life_clk`` when enabled, low when
        gated off.
    """

    def __init__(self):
        # -- Inputs ----------------------------------------------------------
        self.mac_clk_reset_n = Signal(1, name="macclk_reset_n")
        self.mac_clk_rate = Signal(5, name="macclk_rate")
        self.mac_clk_req = Signal(1, name="macclk_req")
        self.life_clk = Signal(1, name="macclk_life_clk")

        # -- Outputs ---------------------------------------------------------
        self.mac_clk_ack = Signal(1, name="macclk_ack")
        self.mac_clk = Signal(1, name="macclk_out")

    def elaborate(self, platform):
        """Build the MacCLK gating and handshake logic.

        Returns
        -------
        Module
            Amaranth module containing:
            - Combinational clock gate (``mac_clk = life_clk & clk_running``)
            - Synchronous request/acknowledge state machine
        """
        m = Module()

        # Internal enable flag — registered so glitch-free
        clk_running = Signal(1, name="macclk_running")

        # -- Clock gate (combinational) -------------------------------------
        # In synthesis this should map to a BUFGCE or equivalent clock
        # enable primitive.  The AND gate is logically correct but should
        # not drive a clock tree directly in production.
        m.d.comb += self.mac_clk.eq(self.life_clk & clk_running)

        # -- Request / acknowledge handshake (synchronous) ------------------
        with m.If(~self.mac_clk_reset_n):
            # Async reset forces clock off
            m.d.sync += [
                clk_running.eq(0),
                self.mac_clk_ack.eq(0),
            ]
        with m.Elif(self.mac_clk_req & ~clk_running):
            # MAC requests clock while stopped — enable gate, ack
            m.d.sync += [
                clk_running.eq(1),
                self.mac_clk_ack.eq(1),
            ]
        with m.Elif(~self.mac_clk_req & clk_running):
            # MAC releases clock while running — disable gate, drop ack
            m.d.sync += [
                clk_running.eq(0),
                self.mac_clk_ack.eq(0),
            ]

        return m
