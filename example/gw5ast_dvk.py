"""Gowin GW5AST-138B Platform for the SerDes USB3.1 Sniffer on the Tang Mega 138K Pro.

Derived from the PCIe sniffer platform with USB3.1 SerDes-specific timing
constraints.  No PCIe connector is used — the SerDes captures USB3.1 PIPE
data directly from the Gowin USB3.1 PHY hard IP.

For SerDes designs where the clock tree is managed explicitly (PLL + SerDes),
:meth:`create_missing_domain` returns ``None`` to prevent Amaranth from
auto-creating clock domains.

Typical usage::

    from serdes_sniffer.platform import GW5ASTDVKPlatform

    platform = GW5ASTDVKPlatform(toolchain="Gowin")
    platform.build(my_design, name="top", build_dir="build")
"""

import re
import subprocess

from amaranth.build import *
from amaranth.build.plat import TemplatedPlatform
from amaranth.vendor import GowinPlatform


__all__ = ["GW5ASTDVKPlatform"]


class GW5ASTDVKPlatform(GowinPlatform):
    """Sipeed Tang Mega 138K Pro Platform for the SerDes USB3.1 Sniffer.

    Targets the GW5AST-LV138FPG676AC1/I0 FPGA in the FCPBGA676A package.
    The board provides a 50 MHz oscillator on pin P16.

    Pin assignments are verified against:
    - Official LiteX board definition (litex-hub/litex-boards)
    - Working ``dvk_cfg_v2.cst`` from GitHub issue #1
    - Gowin PCIe DMA demo ``Pins.cst``

    UART pins (directly connected to external FT4232 via J3 connector):
    - TX: A19 (FPGA → FT4232 RX, connector J3 pin 87)
    - RX: A18 (FT4232 TX → FPGA, connector J3 pin 81)

    Legacy BL616 UART (uart resource 1, directly on SOM):
    - TX: P15 (FPGA → BL616)
    - RX: N16 (BL616 → FPGA)

    .. note::

       This platform does NOT include the ``pcie_rstn`` resource since the
       USB3.1 SerDes sniffer does not use the PCIe edge connector.

    Attributes
    ----------
    gowin_path : :class:`str`
        Absolute path to the Gowin installation root (the directory
        that contains ``IDE/``).
    """

    # GowinPlatform requires 'part' and 'family' as class attributes.
    part = "GW5AST-LV138FPG676AC1/I0"
    family = "GW5AST-138B"

    def __init__(self, *, toolchain="Gowin"):
        super().__init__(toolchain=toolchain)

    def parse_part(self):
        """Override parse_part for GW5AST series not yet in Amaranth 0.5.x.

        Amaranth 0.5.8's GowinPlatform.parse_part() only recognises
        GW[12][AN]... series names.  The GW5AST is a newer Gowin family
        not covered by that regex, so we hard-code the parsed fields
        for the specific part used on this board.
        """
        # ---- part string: GW5AST-LV138FPG676AC1/I0 ----
        m = re.match(
            r"(GW5AST)-(LV)(138)()(FPG676A)(C1/I0)$",
            self.part,
        )
        if not m:
            raise ValueError(f"Unexpected part name: {self.part}")

        self.series = m.group(1)  # "GW5AST"
        self.voltage = m.group(2)  # "LV"
        self.size = m.group(3)  # "138"
        self.subseries = m.group(4)  # ""
        self.package = m.group(5)  # "FPG676A"
        self.speed = m.group(6)  # "C1/I0"

        # ---- family string: GW5AST-138B ----
        m2 = re.match(r"(GW5AST)-(138)(B?)$", self.family)
        if not m2:
            raise ValueError(f"Unexpected family name: {self.family}")

        self.series_f = m2.group(1)  # "GW5AST"
        self.size_f = m2.group(2)  # "138"
        self.subseries_f = m2.group(3)  # "B"

    # Path to the Gowin installation root (the directory that contains IDE/)
    gowin_path = "/home/key2/Downloads/gowin"

    # ------------------------------------------------------------------
    # Board resources
    # ------------------------------------------------------------------

    resources = [
        # 50 MHz board oscillator (P16, LVCMOS33)
        # dvk_cfg_v2.cst: PULL_MODE=NONE, BANK_VCCIO=3.3
        Resource(
            "clk50",
            0,
            Pins("P16", dir="i"),
            Clock(50e6),
            Attrs(IO_TYPE="LVCMOS33", PULL_MODE="NONE"),
        ),
        # NOTE: Board reset pin (U4, LVCMOS15) is NOT used.
        # The dvk_cfg_v2.cst has rst_n commented out, and the pin is in
        # an LVCMOS15 bank that may be unpowered when the board is in a
        # PCIe slot.  The top-level design uses a POR counter instead.
        # User LEDs (active-low: LED turns ON when pin is driven LOW)
        # Using PinsN so Amaranth auto-inverts: writing 1 → pin LOW → LED ON.
        #
        # Pin assignments from LiteX board definition (litex-hub/litex-boards):
        #   led_n 0: J14   led_n 1: R26   led_n 2: L20   led_n 3: M25
        #   led_n 4: N21   led_n 5: N23
        Resource("led", 0, PinsN("J14", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 1, PinsN("R26", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 2, PinsN("L20", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 3, PinsN("M25", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 4, PinsN("N21", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 5, PinsN("N23", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        # UART via external FT4232 (connected to J3 connector)
        # TX: A19 (FPGA → FT4232 RX), RX: A18 (FT4232 TX → FPGA)
        Resource(
            "uart",
            0,
            Subsignal("tx", Pins("A19", dir="o")),
            Subsignal("rx", Pins("A18", dir="i")),
            Attrs(IO_TYPE="LVCMOS33"),
        ),


        Resource(
            "uart",
            1,
            Subsignal("tx", Pins("B19", dir="o")),
            Subsignal("rx", Pins("A17", dir="i")),
            Attrs(IO_TYPE="LVCMOS33"),
        ),
        Resource(
            "uart",
            2,
            Subsignal("tx", Pins("C21", dir="o")),
            Subsignal("rx", Pins("B20", dir="i")),
            Attrs(IO_TYPE="LVCMOS33"),
        ),
    ]

    connectors = []

    # Default clock / reset resource names
    default_clk = "clk50"
    # No default_rst — the top-level design uses a POR counter instead
    # of an external reset pin.

    # ------------------------------------------------------------------
    # Clock domain management
    # ------------------------------------------------------------------

    def create_missing_domain(self, name):
        """Create ``sync`` from the 50 MHz board clock; suppress others.

        SerDes designs on this board manage their own clock domains via
        PLL and the SerDes recovered clock.  Only the ``sync`` domain is
        auto-created (from the 50 MHz oscillator on P16) because the
        UART and other board-clock logic require it.  All other missing
        domains return ``None`` so Amaranth does not auto-create them.
        """
        if name == "sync":
            return super().create_missing_domain(name)
        return None

    # ------------------------------------------------------------------
    # Toolchain overrides
    # ------------------------------------------------------------------

    @property
    def file_templates(self):
        """Override file templates to include .csr files in the TCL script.

        The base GowinPlatform TCL template only iterates over
        ``.v``, ``.sv``, ``.vhd``, ``.vhdl`` files.  SerDes designs
        need ``.csr`` (SerDes configuration) files to be added too.
        """
        templates = dict(super().file_templates)

        # Override the TCL template to also add .csr files.
        # The Gowin PnR tool requires CSR files for SerDes configuration.
        # We create a process config JSON that the PnR tool reads to
        # find the CSR file path.
        templates["{{name}}.tcl"] = r"""
            # {{autogenerated}}
            {% for file in platform.iter_files(".v",".sv",".vhd",".vhdl") -%}
                add_file {{file}}
            {% endfor %}
            add_file -type verilog {{name}}.v
            add_file -type cst {{name}}.cst
            add_file -type sdc {{name}}.sdc
            {% for file in platform.iter_files(".csr") -%}
                set_csr {{file}}
            {% endfor %}
            set_device -name {{platform.family}} {{platform.part}}
            set_option -verilog_std v2001 -print_all_synthesis_warning 1 -show_all_warn 1
            {{get_override("add_options")|default("# (add_options placeholder)")}}
            run all
            file delete -force {{name}}.fs
            file copy -force impl/pnr/project.fs {{name}}.fs
        """

        # Create a process config JSON that tells the PnR tool about
        # SerDes retiming and other optimization options.
        templates["impl/project_process_config.json"] = r"""
            {
                "SerDes_retiming" : false,
                "Correct_Hold_Violation" : true,
                "Clock_Route_Order" : 1,
                "Place_Option" : "3",
                "Route_Option" : "1",
                "Route_Maxfan" : 23,
                "Run_Timing_Driven" : true,
                "Promote_Physical_Constraint_Warning_to_Error" : true,
                "Verilog_Standard" : "Vlg_Std_Sysv2017",
                "Process_Configuration_Verion" : "1.0"
            }
        """
        return templates

    def toolchain_prepare(self, fragment, name, **kwargs):
        """Prepare the build plan with Gowin-specific options.

        Adds synthesis options, CSR file references, and timing
        constraints via the ``add_options`` and ``add_constraints``
        overrides supported by :class:`GowinPlatform`.

        Timing constraints are mode-dependent:

        **PIPE mode** (default, ``name != "raw_serdes_sniffer"``):
        - **pclk** (156.25 MHz, 6.4 ns period) — the PIPE clock output
          from the USB3.1 PHY SerDes_Top.
        - **False paths** between 50 MHz board clock and ``pclk``.
        - **Multicycle paths** for UPAR arbiter DRP interface.

        **Raw mode** (``name == "raw_serdes_sniffer"``):
        - **rxclk** (156.25 MHz, 6.4 ns period) — the PCS RX recovered
          clock from SerDes_Raw.  Only our FIFO write logic runs in this
          domain, so timing closure is trivial.
        - **False paths** between 50 MHz board clock and ``rxclk``.
        """
        # Build the add_options string, including CSR file references
        # for any SerDes IP cores registered via add_file().
        add_options_lines = [
            "set_option -verilog_std sysv2017",
            "set_option -print_all_synthesis_warning 1",
            "set_option -show_all_warn 1",
        ]

        # ---------------------------------------------------------------
        # Critical Gowin toolchain options (from LiteX board definition)
        # ---------------------------------------------------------------
        # These options free up special-purpose pins for GPIO use and
        # configure bitstream security/compression.  Without these, the
        # Gowin tools may reserve MSPI/SSPI/READY/DONE/CPU pins for
        # their dedicated functions, potentially conflicting with the
        # SerDes or other I/O.
        add_options_lines += [
            "set_option -use_ready_as_gpio 1",
            "set_option -use_done_as_gpio 1",
            "set_option -use_mspi_as_gpio 1",
            "set_option -use_sspi_as_gpio 1",
            "set_option -use_cpu_as_gpio 1",
            "set_option -rw_check_on_ram 1",
            "set_option -bit_security 0",
            "set_option -bit_encrypt 0",
            "set_option -bit_compress 0",
        ]

        # NOTE: CSR files (SerDes configuration) are included in the
        # TCL script via the overridden file_templates property.
        # We also need to create a process config JSON that tells
        # the PnR tool about the SerDes retiming option.
        add_options_lines.append("set_option -serdesRetiming 0")

        # ---------------------------------------------------------------
        # SDC timing constraints — mode-dependent
        # ---------------------------------------------------------------
        is_raw = name in (
            "raw_serdes_sniffer",
            "raw_lfps_sniffer",
            "raw_lfps_v2_sniffer",
            "phy_test_sniffer",
            "cpll_raw_sniffer",
            "phy_test_cpll_sniffer",
        )
        is_link_training = name == "link_training"
        is_csr_tuner = name == "csr_tuner"
        is_lfps_pipe = name in (
            "lfps_pipe_sniffer",
            "gowin_phy_test",
            "gowin_gen1_cpll_sniffer",
            "gowin_gen1_125_sniffer",
        )

        if is_link_training:
            # Link Training mode: Custom PHY with ss clock at 125 MHz from
            # rx_pcs_clkout_o and sync at 50 MHz.  The ss domain runs the
            # LUNA LTSSM and width adapters; the rxclk alias drives the
            # capture pipeline FIFO write side.
            #
            # The recovered clock is driven from the SerDes hard macro.
            # Amaranth names the domain wire ``ss_clk`` but the PnR may
            # not resolve it via get_nets because it comes directly from
            # a hard macro output pin.  We let the Gowin PnR auto-detect
            # the SerDes clocks and only declare false paths to ensure
            # CDC crossings are not timed as single-clock paths.
            #
            # The auto-detected board clock is ``clk50_0__io``.
            # All other clocks (SerDes recovered, UPAR life_clk) are
            # asynchronous to it.
            # No explicit create_clock for SerDes clocks — the Gowin PnR
            # auto-detects them from the hard macro outputs.  The board
            # clock (clk50_0__io at 50 MHz) is auto-created by Amaranth.
            # All CDC crossings use AsyncFIFO or FFSynchronizer, so the
            # PnR's default handling of unrelated clocks is sufficient.
            sdc_constraints = []
        elif is_lfps_pipe:
            # LFPS PIPE mode: follows the working PCIe project pattern.
            #
            # Key insight: do NOT create_clock for SerDes-internal clocks
            # (pclk, rxclk, life_clk).  These are inside the hard IP and
            # managed internally.  The PCIe project only constrains the
            # board clock (auto) + PLL output.
            #
            # We DO need to declare rxclk for our AsyncFIFO write domain
            # since it's a PCS RX fabric clock output driving fabric FFs.
            sdc_constraints = [
                # Board clock (clk50_0__io) is auto-created by Amaranth.
                # NO create_clock for pclk — it's SerDes-internal.
                "",
                # rxclk — 156.25 MHz (6.4 ns) PCS RX fabric clock output.
                # This drives our AsyncFIFO write-side registers in fabric.
                "create_clock -name rxclk -period 6.4 [get_nets {rxclk_clk}]",
                "",
                # CDC false paths between board clock and rxclk
                "set_false_path -from [get_clocks {clk50_0__io}] -to [get_clocks {rxclk}]",
                "set_false_path -from [get_clocks {rxclk}] -to [get_clocks {clk50_0__io}]",
                "",
                # UPAR arbiter multicycle paths — exact pin names from
                # working PCIe project, with hierarchy prefix "serdes/"
                # (our Instance name) instead of "phy/serdes/".
                "set_multicycle_path 2 -setup -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_wren_o_s1/Q}]",
                "set_multicycle_path 1 -hold -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_wren_o_s1/Q}]",
                "",
                "set_multicycle_path 2 -setup -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_rden_o_s1/Q}]",
                "set_multicycle_path 1 -hold -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_rden_o_s1/Q}]",
                "",
                "set_multicycle_path 2 -setup -from [get_pins "
                "{serdes/gtr12_upar_inst/UPAR_READY_S}]",
                "set_multicycle_path 1 -hold -from [get_pins "
                "{serdes/gtr12_upar_inst/UPAR_READY_S}]",
                "",
                "set_multicycle_path 2 -setup -to [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_addr_o_*_s1/D}]",
                "set_multicycle_path 1 -hold -to [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_addr_o_*_s1/D}]",
                "",
                "set_multicycle_path 2 -setup -to [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_strb_o_*_s1/D}]",
                "set_multicycle_path 1 -hold -to [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_strb_o_*_s1/D}]",
                "",
                "set_multicycle_path 2 -setup -to [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_wrdata_o_*_s1/D}]",
                "set_multicycle_path 1 -hold -to [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/u_upar_arbiter/upar_wrdata_o_*_s1/D}]",
            ]
        elif is_csr_tuner:
            # CSR tuner: Custom PHY with rxclk from PCS RX recovered clock.
            # The GowinClockGen PLL is swept (Custom PHY manages its own PLL
            # internally), so we only constrain rxclk and life_clk.
            sdc_constraints = [
                # rxclk — 156.25 MHz (6.4 ns) PCS RX recovered clock
                "create_clock -name rxclk -period 6.4 [get_nets {rxclk_clk}]",
                "",
                # CDC false paths: 50 MHz board clock ↔ rxclk
                "set_false_path -from [get_clocks {clk50_0__io}] -to [get_clocks {rxclk}]",
                "set_false_path -from [get_clocks {rxclk}] -to [get_clocks {clk50_0__io}]",
                "",
                # UPAR lifecycle clock (q1_fabric_cm_life_clk_o)
                "create_clock -name life_clk -period 10.0 "
                "[get_pins {serdes/gtr12_quad_inst1/FABRIC_CM_LIFE_CLK_O}]",
                "set_false_path -from [get_clocks {life_clk}]",
                "set_false_path -to [get_clocks {life_clk}]",
            ]
        elif is_raw:
            # Raw mode: rxclk from SerDes_Raw PCS RX recovered clock.
            # Only our trivial FIFO write logic runs in the rxclk domain
            # (pack and write), so timing closure is straightforward.
            sdc_constraints = [
                # rxclk — 156.25 MHz (6.4 ns) PCS RX recovered clock
                "create_clock -name rxclk -period 6.4 [get_nets {rxclk_clk}]",
                "",
                # CDC false paths: 50 MHz board clock ↔ rxclk
                "set_false_path -from [get_clocks {clk50_0__io}] -to [get_clocks {rxclk}]",
                "set_false_path -from [get_clocks {rxclk}] -to [get_clocks {clk50_0__io}]",
                "",
                # PLL CLKOUT1 (100 MHz) feeds the SerDes as LANE0_FABRIC_RX_CLK.
                # rxclk comes back out from the SerDes PCS as the recovered clock.
                # These are NOT independent clocks — CLKOUT1 enters the GTR12_QUAD
                # hard IP and rxclk exits it.  The data path through the hard macro
                # is not a fabric-constrained path.  Mark them asynchronous to
                # eliminate the false cross-domain violations.
                #
                # The PLL output clock must be explicitly created in the SDC
                # because the Gowin PnR auto-detection happens AFTER SDC parsing.
                # Without this, any set_false_path referencing the PLL clock name
                # will fail with TA2004 "Cannot get clock".  Period 10 ns = 100 MHz.
                "create_clock -name clkout1 -period 10.0 [get_pins {clock_gen/pll/CLKOUT1}]",
                "set_false_path -from [get_clocks {clkout1}] -to [get_clocks {rxclk}]",
                "set_false_path -from [get_clocks {rxclk}] -to [get_clocks {clkout1}]",
                "",
                # UPAR lifecycle clock (q1_fabric_cm_life_clk_o) is auto-detected
                # by Gowin PnR at 100 MHz (10 ns).  It drives only the slow UPAR
                # initialization register interface inside the encrypted SerDes IP.
                # All 64 setup violations on this clock are IP-internal paths that
                # do not affect run-time data path operation.
                #
                # Like the PLL clock above, we must explicitly create the life_clk
                # in the SDC since auto-detected clocks are not available when the
                # SDC is parsed.  The FABRIC_CM_LIFE_CLK_O pin on the GTR12_QUAD
                # hard macro is the clock source.
                "create_clock -name life_clk -period 10.0 "
                "[get_pins {serdes/gtr12_quad_inst1/FABRIC_CM_LIFE_CLK_O}]",
                "set_false_path -from [get_clocks {life_clk}]",
                "set_false_path -to [get_clocks {life_clk}]",
                "",
                # DRP clock from UPAR arbiter channel 4 (drp_clk_o).
                # This clock is internal to the encrypted UPAR arbiter IP
                # and is NOT routed to any fabric registers — the LFPS
                # responder's DRP outputs are gated combinationally in the
                # sync domain.  The PnR will not auto-detect this clock
                # as a fabric clock, so no SDC constraints are needed.
                # If the PnR ever promotes it to a global clock, the
                # life_clk wildcard false paths above cover it since both
                # clocks originate from the same UPAR arbiter source.
            ]
        else:
            # PIPE mode: pclk from SerDes_Top USB3.1 PHY.
            sdc_constraints = [
                # SerDes pclk — 156.25 MHz (6.4 ns) for USB3.1 Gen1 10Gbps
                # with 64-bit PIPE data: 10G / 64 = 156.25 MHz.
                # The pclk is generated by the SerDes PLL and drives both the
                # Gowin USB3_1_PHY_Top internal logic and our capture FIFO.
                "create_clock -name pclk -period 6.4 [get_nets {pclk_clk}]",
                "",
                # CDC false paths: 50 MHz board clock ↔ SerDes pclk
                # These crossings use AsyncFIFO / FFSynchronizer and must not
                # be timed as single-clock paths.
                "set_false_path -from [get_clocks {clk50_0__io}] -to [get_clocks {pclk}]",
                "set_false_path -from [get_clocks {pclk}] -to [get_clocks {clk50_0__io}]",
                "",
                # NOTE: The DRP clock (drp_clk_o[4]) from the UPAR arbiter is
                # auto-detected by the Gowin PnR tool.  Its full hierarchical
                # name (serdes/upar_arbiter_wrap_SerDes_Top_inst_drp_clk_o[4])
                # is only available after PnR, so we cannot reference it in
                # SDC via get_clocks.  Since the DRP clock drives only the
                # slow UPAR register interface and has no CDC paths to our
                # capture logic, omitting explicit false paths is safe — the
                # tool will not create timing paths between unrelated clocks
                # that have no register-to-register connectivity.
                "",
                # Multicycle path for UPAR arbiter DRP interface.
                # The upar_arbiter is a configuration/management bus arbiter
                # (Dynamic Reconfiguration Port) that arbitrates access from
                # up to 8 DRP clients to the SerDes UPAR register interface.
                # Its FSM spends 3 cycles in JUDG_ADDR state before enabling
                # UPAR, plus a WAIT cycle after each transaction, so the
                # combinational paths between upar_wren_o and upar_addr_o
                # registers have multiple cycles of slack.  Allowing 2-cycle
                # setup relaxes the ~0.8 ns violations on these paths.
                "set_multicycle_path 2 -setup -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/"
                "u_upar_arbiter/upar_wren_o_s1/Q}]",
                "set_multicycle_path 1 -hold -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/"
                "u_upar_arbiter/upar_wren_o_s1/Q}]",
                # Same relaxation for upar_rden_o (read enable) — identical
                # FSM timing: the rden path also goes through JUDG_ADDR (3
                # cycles) before the UPAR_EN state samples upar_rdvld_i.
                "set_multicycle_path 2 -setup -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/"
                "u_upar_arbiter/upar_rden_o_s1/Q}]",
                "set_multicycle_path 1 -hold -from [get_pins "
                "{serdes/upar_arbiter_wrap_SerDes_Top_inst/"
                "u_upar_arbiter/upar_rden_o_s1/Q}]",
            ]

        overrides = {
            "add_options": "\n".join(add_options_lines),
            "add_constraints": "\n".join(sdc_constraints),
        }
        # Merge caller-provided kwargs with our overrides (caller wins)
        merged = {**overrides, **kwargs}
        return super().toolchain_prepare(fragment, name, **merged)

    def toolchain_program(self, products, name, **kwargs):
        """Program the FPGA using openFPGALoader.

        Uses ``--ftdi-serial`` to target the BL616 debugger (SIPEED USB
        Debugger) when multiple FTDI devices are present (e.g. an external
        FT4232 for UART).
        """
        with products.extract("{}.fs".format(name)) as bitstream_filename:
            cmd = [
                "openFPGALoader",
                "--cable",
                "ft2232",
                "--ftdi-serial",
                "2023102515",
                "--bitstream",
                bitstream_filename,
            ]
            subprocess.check_call(cmd)


if __name__ == "__main__":
    # Quick test: build a blinky to verify the platform works
    from amaranth import *

    class Blinky(Elaboratable):
        def elaborate(self, platform):
            m = Module()
            led = platform.request("led", 0)
            counter = Signal(26)
            m.d.sync += counter.eq(counter + 1)
            m.d.comb += led.o.eq(counter[-1])
            return m

    GW5ASTDVKPlatform(toolchain="Gowin").build(Blinky(), do_program=False)
