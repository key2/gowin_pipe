#!/usr/bin/env python3
"""
Gowin PLL IP Core Generator
============================

Programmatically generates Gowin PLL (Phase-Locked Loop) IP cores without the
GUI, replicating the pipeline that the Gowin IDE performs:

  1. Generate ``.ipc`` (config persistence) and ``.mod`` (GowinModGen input)
  2. Run ``GowinModGen -do <name>.mod`` to produce ``_mod.v`` and ``_mod_tmp.v``
  3. Generate the top-level wrapper ``.v`` and ``_tmp.v`` (connects PLL_MOD to PLL_INIT)
  4. Copy ``pll_init.v`` (Gowin-provided FSM) to the output directory

Programmatic usage::

    from gowin_pll_gen import PLLConfig, GowinPLLGenerator

    config = PLLConfig(
        gowin_dir="./Gowin_V1.9.12.01_linux",
        output_dir="./my_pll_project",
        fclkin=100.0,
        mdiv_sel=16,
        odiv0_sel=8,
        odiv1_sel=4,
        clkout1_en=True,
    )
    gen = GowinPLLGenerator(config)
    gen.generate()
"""

from __future__ import annotations

import argparse
import logging
import os
import shutil
import subprocess
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

logger = logging.getLogger("gowin_pll_gen")

# ---------------------------------------------------------------------------
# Embedded PLL_INIT source
# ---------------------------------------------------------------------------

_PLL_INIT_V = r"""`timescale 1ns/1ns

module PLL_INIT
#(
parameter CLK_PERIOD = 50,
parameter MULTI_FAC  = 30
)
(
input CLKIN,
input I_RST,
input PLLLOCK,
output O_RST,
output [5:0] ICPSEL,
output [2:0] LPFRES,
output O_LOCK

);

localparam WAIT_TIME = 'd2000_000;      
localparam WAIT_CNT = (WAIT_TIME + CLK_PERIOD - 1) / CLK_PERIOD; 
localparam WAIT_WIDTH = $clog2(WAIT_CNT + 1);

reg [WAIT_WIDTH-1:0] waitcnt = 'd0;
reg [3:0] RomAddr = 'd0;
reg [15:0] Rom [15:0];
reg [15:0] RomDreg ='d0;
wire [5:0] Regicp = RomDreg[5:0];
wire [2:0] Regres = RomDreg[10:8];
reg Waitlock = 'd0;
reg [7:0] locksig = 8'b0000_0000;
reg laststep = 1'b0;

wire validsig = RomDreg[12];
reg [1:0]enable_r = 2'b00;

wire Enable = enable_r[1];
always @(posedge CLKIN or posedge I_RST)
begin
    if (I_RST)
        enable_r <= 2'b00;
    else
        begin
            enable_r <= {enable_r[0], 1'b1};
        end
end 

always @(posedge CLKIN or negedge Enable)
begin
    if (Enable == 1'b0)
        begin
            RomDreg <= 16'h0000;
            
            Rom[00] <= 16'h1400;    //1
            Rom[01] <= (MULTI_FAC > 34) ? 16'h1401
                    :  (MULTI_FAC > 16) ? 16'h1400
                    :                     16'h1400;    //2
            Rom[02] <= (MULTI_FAC > 34) ? 16'h1501
                    :  (MULTI_FAC > 16) ? 16'h1500
                    :                     16'h1500;    //3
            Rom[03] <= (MULTI_FAC > 34) ? 16'h1503
                    :  (MULTI_FAC > 16) ? 16'h1501
                    :                     16'h1500;    //4
            Rom[04] <= (MULTI_FAC > 34) ? 16'h1507
                    :  (MULTI_FAC > 16) ? 16'h1503
                    :                     16'h1501;    //5
            Rom[05] <= (MULTI_FAC > 34) ? 16'h0605
                    :  (MULTI_FAC > 16) ? 16'h0602
                    :                     16'h0601;    //6
            Rom[06] <= 16'h0000;    //over
                        
            Rom[07] <= (MULTI_FAC > 34) ? 16'h1402
                    :  (MULTI_FAC > 16) ? 16'h1401
                    :                     16'h1400;    //2.5
            Rom[08] <= (MULTI_FAC > 34) ? 16'h1502
                    :  (MULTI_FAC > 16) ? 16'h1501
                    :                     16'h1500;    //3.5
            Rom[09] <= (MULTI_FAC > 34) ? 16'h1504
                    :  (MULTI_FAC > 16) ? 16'h1502
                    :                     16'h1501;    //4.5
            Rom[10] <= (MULTI_FAC > 34) ? 16'h1603
                    :  (MULTI_FAC > 16) ? 16'h1601
                    :                     16'h1600;    //5.5
            Rom[11] <= 16'h1400;    //1.5
            Rom[12] <= 16'h0000;
            Rom[13] <= 16'h0000;
            Rom[14] <= 16'h0000;
            Rom[15] <= 16'h0000;
        end
    else
        RomDreg <= Rom[RomAddr[3:0]];
end

reg [3:0]RomAddrVld = 'd0;
always @ (*) begin
    casex (locksig[5:0])
        6'b111_111  :   RomAddrVld = 4'd8;
        6'b011_111  :   RomAddrVld = 4'd8;
        6'b111_110  :   RomAddrVld = 4'd8;
        6'b111_10x  :   RomAddrVld = 4'd9;
        6'bx01_111  :   RomAddrVld = 4'd7;
        6'b011_110  :   RomAddrVld = 4'd8;
        6'bxx0_111  :   RomAddrVld = 4'd1;
        6'bx01_110  :   RomAddrVld = 4'd2;
        6'b011_10x  :   RomAddrVld = 4'd3;
        6'b111_0xx  :   RomAddrVld = 4'd4;
                                     
        6'bxxx_011  :   RomAddrVld = 4'd1;
        6'bxx0_110  :   RomAddrVld = 4'd7;
        6'bx01_100  :   RomAddrVld = 4'd8;
        6'b011_000  :   RomAddrVld = 4'd9;
        6'b110_000  :   RomAddrVld = 4'd10;
        6'bxxx_x01  :   RomAddrVld = 4'd0;
        6'bxxx_010  :   RomAddrVld = 4'd1;
        6'bxx0_100  :   RomAddrVld = 4'd2;
        6'bx01_000  :   RomAddrVld = 4'd3;
        6'b010_000  :   RomAddrVld = 4'd4;
        6'b100_000  :   RomAddrVld = 4'd5;
                                     
        6'b000_000  :   RomAddrVld = 4'd8;
                                     
        default     :   RomAddrVld = 4'd8;
    endcase                          
end


reg [3:0]state='d0;
localparam IDLE  = 4'd0;
localparam STATE1 = 4'd1;
localparam STATE2 = 4'd2;
localparam STATE3 = 4'd3;
localparam STATE4 = 4'd4;
localparam STATE5 = 4'd5;
localparam STATE6 = 4'd6;
localparam STATE7 = 4'd7;



always @(posedge CLKIN or negedge Enable)
begin
    if (Enable == 1'b0)
        begin
        state <= IDLE;
        RomAddr<=4'b0000;
        laststep <= 1'b0;
        end
    else
        begin
            case (state)
                IDLE: 
                    begin
                    state<=STATE1;
                    end
                STATE1:
                    begin
                    state<=STATE2;
                    end
                STATE2:
                    begin
                    if (laststep==1'b1)
                        state<=STATE7;
                    else
                        state<=STATE3;
                    end
                STATE3:
                    begin
                    if (Waitlock==1'b1)
                        state<=STATE4;
                    else
                        state<= STATE3;
                    end
                STATE4:
                    begin
                    state<=STATE5;               
                    end
                STATE5:
                    begin
                    if (validsig==1'b1)
                        begin
                            RomAddr <= RomAddr + 1;
                            state <= STATE1;
                        end
                    else if (validsig==1'b0 && laststep==1'b0)
                            state <= STATE6;
                    end
                STATE6:
                    begin
                    RomAddr <= RomAddrVld;
                    laststep <= 1'b1;
                    state <= STATE1;
                    end
                STATE7:
                    begin
                    state<=STATE7;
                    end

                default: state<=IDLE;
            endcase
        end
end

always @(posedge CLKIN)
begin


    Waitlock <= (&waitcnt == 1'b1) ? 1'b1 : 1'b0; 
    
    waitcnt <= (state==STATE3) ? (waitcnt+1) : 'd0;
    
    if (~Enable)    locksig <= 8'b0000_0000;
    else    if (state==STATE4)   locksig[RomAddr[3:0]] <= PLLLOCK;


end

assign ICPSEL = Regicp;
assign LPFRES = Regres;
assign O_RST = (~Enable) || (state==STATE2) ? 1'b1 : 1'b0;
assign O_LOCK = (state==STATE7) ? PLLLOCK : 1'b0;

endmodule
"""


# ---------------------------------------------------------------------------
# Data class
# ---------------------------------------------------------------------------

@dataclass
class PLLConfig:
    """All parameters needed to generate a Gowin PLL IP core."""

    # Gowin toolchain
    gowin_dir: str = ""             # Path to Gowin installation
    output_dir: str = ""            # Where to generate files

    # Module naming
    module_name: str = "Gowin_PLL"  # Top-level module name
    file_name: str = "gowin_pll"    # Base filename

    # Device info (default: GW5AST-138)
    series: str = "GW5AST"
    device: str = "GW5AST-138"
    device_version: str = "C"
    package: str = "FCPBGA676A"
    part_number: str = "GW5AST-LV138FPG676AC1/I0"
    target_device: str = "gw5ast138c-003"

    # Clock configuration
    fclkin: float = 100.0           # Input clock frequency in MHz

    # PLL dividers
    idiv_sel: int = 2               # Input divider (1-64)
    fbdiv_sel: int = 1              # Feedback divider (1-64)
    mdiv_sel: int = 16              # VCO multiplier (2-128)
    mdiv_frac_sel: int = 0          # VCO fractional (0-7)
    odiv0_sel: int = 8              # Output 0 divider (1-128)
    odiv0_frac_sel: int = 0         # Output 0 fractional (0-7)
    odiv1_sel: int = 4              # Output 1 divider
    odiv2_sel: int = 8              # Output 2 divider
    odiv3_sel: int = 8              # Output 3 divider
    odiv4_sel: int = 8              # Output 4 divider
    odiv5_sel: int = 8              # Output 5 divider
    odiv6_sel: int = 8              # Output 6 divider

    # Channel enables (CLKOUT0 always enabled)
    clkout1_en: bool = True
    clkout2_en: bool = False
    clkout3_en: bool = False
    clkout4_en: bool = False
    clkout5_en: bool = False
    clkout6_en: bool = False

    # Features
    enable_lock: bool = True
    enable_clkfbout: bool = False
    clkfb_internal: bool = True     # True=internal, False=external
    clkfb_external_value: str = "CLKOUT0"

    # Dynamic control
    dyn_icp_sel: bool = True
    dyn_lpf_sel: bool = True
    dyn_idiv: bool = False
    dyn_fbdiv: bool = False
    dyn_mdiv: bool = False
    dyn_odiv0: bool = False
    dyn_odiv1: bool = False
    dyn_odiv2: bool = False
    dyn_odiv3: bool = False
    dyn_odiv4: bool = False
    dyn_odiv5: bool = False
    dyn_odiv6: bool = False
    dyn_dt0: bool = False
    dyn_dt1: bool = False
    dyn_dt2: bool = False
    dyn_dt3: bool = False

    # Reset/power
    pll_reset: bool = True
    pll_powerdown: bool = False
    reset_i_en: bool = False
    reset_o_en: bool = False

    # Phase per channel (coarse, fine)
    clkout0_pe_coarse: int = 0
    clkout0_pe_fine: int = 0
    clkout1_pe_coarse: int = 0
    clkout1_pe_fine: int = 0
    clkout2_pe_coarse: int = 0
    clkout2_pe_fine: int = 0
    clkout3_pe_coarse: int = 0
    clkout3_pe_fine: int = 0
    clkout4_pe_coarse: int = 0
    clkout4_pe_fine: int = 0
    clkout5_pe_coarse: int = 0
    clkout5_pe_fine: int = 0
    clkout6_pe_coarse: int = 0
    clkout6_pe_fine: int = 0

    # Duty trim per channel (dir, step) - only 0-3
    clkout0_dt_dir: int = 1         # 1'b1
    clkout0_dt_step: int = 0
    clkout1_dt_dir: int = 1
    clkout1_dt_step: int = 0
    clkout2_dt_dir: int = 1
    clkout2_dt_step: int = 0
    clkout3_dt_dir: int = 1
    clkout3_dt_step: int = 0

    # Bypass per channel
    clkout0_bypass: bool = False
    clkout1_bypass: bool = False
    clkout2_bypass: bool = False
    clkout3_bypass: bool = False
    clkout4_bypass: bool = False
    clkout5_bypass: bool = False
    clkout6_bypass: bool = False

    # Clock input/output mux
    clk0_in_sel: int = 0
    clk0_out_sel: int = 0
    clk1_in_sel: int = 0
    clk1_out_sel: int = 0
    clk2_in_sel: int = 0
    clk2_out_sel: int = 0
    clk3_in_sel: int = 0
    clk3_out_sel: int = 0
    clk4_in_sel: int = 0            # Note: CLK4_IN_SEL is 2'b00 in reference
    clk4_out_sel: int = 0
    clk5_in_sel: int = 0
    clk5_out_sel: int = 0
    clk6_in_sel: int = 0
    clk6_out_sel: int = 0

    # Duty cycle enable
    de0_en: bool = False
    de1_en: bool = False
    de2_en: bool = False
    de3_en: bool = False
    de4_en: bool = False
    de5_en: bool = False
    de6_en: bool = False

    # SSC
    ssc_enable: bool = False

    # Static ICP/LPF (used when not dynamic)
    icp_sel: str = "6'bXXXXXX"
    lpf_res: str = "3'bXXX"
    lpf_cap: str = "2'b00"

    # Enable clock gating
    enclk0: bool = False
    enclk1: bool = False
    enclk2: bool = False
    enclk3: bool = False
    enclk4: bool = False
    enclk5: bool = False
    enclk6: bool = False

    # Dynamic phase adjust
    dyn_dpa_en: bool = False

    # Clock enable ports
    clock_enable_ports: bool = False

    # Initialization clock frequency (MHz) for IPC
    init_clk_freq: int = 50

    # ------------------------------------------------------------------
    # Derived helpers
    # ------------------------------------------------------------------

    @property
    def inner_module_name(self) -> str:
        """Inner PLL module name (e.g. ``Gowin_PLL_MOD``)."""
        return f"{self.module_name}_MOD"

    @property
    def inner_file_name(self) -> str:
        """Inner PLL file base name (e.g. ``gowin_pll_mod``)."""
        return f"{self.file_name}_mod"

    @property
    def clk_period_ns(self) -> int:
        """CLK_PERIOD for PLL_INIT: integer ns = 1000/fclkin * idiv_sel."""
        return int(1000.0 / self.fclkin * self.idiv_sel)

    @property
    def enabled_clkouts(self) -> list[int]:
        """List of enabled clock output indices (0 is always enabled)."""
        outs = [0]
        for i, en in enumerate([
            self.clkout1_en, self.clkout2_en, self.clkout3_en,
            self.clkout4_en, self.clkout5_en, self.clkout6_en,
        ], start=1):
            if en:
                outs.append(i)
        return outs


# ---------------------------------------------------------------------------
# Generator
# ---------------------------------------------------------------------------

class GowinPLLGenerator:
    """Orchestrates the full Gowin PLL IP-core generation pipeline."""

    def __init__(self, config: PLLConfig) -> None:
        self.cfg = config
        self._gowin = Path(config.gowin_dir).resolve()
        self._out = Path(config.output_dir).resolve()
        self._pll_dir = self._out / "src" / config.file_name.replace("_", "_")
        # Use the same directory name as the file_name
        # e.g. output_dir/src/gowin_pll/
        self._pll_dir = self._out / "src" / config.file_name

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def generate(self) -> None:
        """Run the full generation pipeline."""
        logger.info("=== Gowin PLL IP Core Generator ===")
        logger.info("Output directory : %s", self._out)
        logger.info("Gowin IDE        : %s", self._gowin)
        logger.info("Device           : %s%s (%s)",
                     self.cfg.device, self.cfg.device_version,
                     self.cfg.part_number)
        logger.info("PLL              : fclkin=%.1f MHz, MDIV=%d, ODIV0=%d",
                     self.cfg.fclkin, self.cfg.mdiv_sel, self.cfg.odiv0_sel)

        self._create_directory_structure()
        self._generate_ipc()
        self._generate_mod()
        self._run_gowin_modgen()
        self._generate_top_v()
        self._generate_top_tmp_v()
        self._copy_pll_init()

        logger.info("=== Generation complete ===")

    # ------------------------------------------------------------------
    # Directory structure
    # ------------------------------------------------------------------

    def _create_directory_structure(self) -> None:
        logger.info("Creating directory structure …")
        self._pll_dir.mkdir(parents=True, exist_ok=True)
        # Also ensure src/ exists for pll_init.v
        (self._out / "src").mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # .ipc file
    # ------------------------------------------------------------------

    def _generate_ipc(self) -> None:
        path = self._pll_dir / f"{self.cfg.file_name}.ipc"
        logger.info("Generating %s", path)

        c = self.cfg

        def _b(val: bool) -> str:
            return "true" if val else "false"

        # Compute expected frequencies for each output
        # VCO freq = fclkin / idiv_sel * mdiv_sel
        vco_freq = c.fclkin / c.idiv_sel * c.mdiv_sel
        odivs = [c.odiv0_sel, c.odiv1_sel, c.odiv2_sel, c.odiv3_sel,
                 c.odiv4_sel, c.odiv5_sel, c.odiv6_sel]
        # Enabled channels use their actual divider; disabled channels
        # and the feedback output use VCO / 2 (Gowin IDE convention).
        enabled_flags = [True, c.clkout1_en, c.clkout2_en, c.clkout3_en,
                         c.clkout4_en, c.clkout5_en, c.clkout6_en]
        expected_freqs = [
            vco_freq / od if en else vco_freq / 2
            for od, en in zip(odivs, enabled_flags)
        ]
        # Feedback expected frequency: VCO / 2 (Gowin IDE convention)
        fb_expected = vco_freq / 2

        lines = [
            "[General]",
            f"file={c.file_name}",
            "ipc_version=4",
            f"module={c.module_name}",
            f"target_device={c.target_device}",
            "type=clock_plladv",
            "version=1.0",
            "",
            "[Config]",
            "AdvancedMode=false",
            f"ClkfbDivideFactorDynamic={_b(not c.dyn_fbdiv)}",
            f"ClkfbDivideFactorDynamicValue={c.fbdiv_sel}",
            f"ClkfbDivideFactorStatic={_b(c.dyn_fbdiv)}",
            f"ClkfbDivideFactorStaticValue={c.fbdiv_sel}",
            f"ClkfbInternal={_b(c.clkfb_internal)}",
            f"ClkfboutExpectedFrequency={int(fb_expected)}",
            "ClkfboutTolerance=0.0",
            f"ClkfboutVCODivideFactorDynamic={_b(not c.dyn_mdiv)}",
            f"ClkfboutVCODivideFactorDynamicValue={c.odiv0_sel}",
            f"ClkfboutVCODivideFactorStatic={_b(c.dyn_mdiv)}",
            f"ClkfboutVCODivideFactorStaticValue={c.mdiv_sel}",
            f"ClkfboutVCOFractionalDivideFactorDynamicValue={c.mdiv_frac_sel}",
            f"ClkfboutVCOFractionalDivideFactorStaticValue={c.mdiv_frac_sel}",
            f"ClkinClockFrequency={int(c.fclkin)}",
            f"ClkinDividerFactorDynamic={_b(not c.dyn_idiv)}",
            f"ClkinDividerFactorStatic={_b(c.dyn_idiv)}",
            f"ClkinDividerFactorStaticValue={c.idiv_sel}",
            "ClkinDividerReset=false",
        ]

        # Clkout 1-3 BPhaseStatic
        for i in range(1, 4):
            lines.append(f"Clkou{i}BPhaseStatic=true")

        # Per-channel config for 0-6
        for i in range(7):
            bypass = getattr(c, f"clkout{i}_bypass")
            pe_coarse = getattr(c, f"clkout{i}_pe_coarse")
            pe_fine = getattr(c, f"clkout{i}_pe_fine")
            odiv = odivs[i]
            dyn_odiv = getattr(c, f"dyn_odiv{i}")

            lines.append(f"Clkout{i}Bypass={_b(bypass)}")

            if i <= 3:
                # Channels 0-3 have DutyCycle and DutyTrim
                dt_dir = getattr(c, f"clkout{i}_dt_dir")
                dt_step = getattr(c, f"clkout{i}_dt_step")
                dyn_dt = getattr(c, f"dyn_dt{i}")

                lines.append(f"Clkout{i}DutyCycleDynamic={_b(dyn_dt)}")
                lines.append(f"Clkout{i}DutyCycleDynamicValue={dt_step}")
                lines.append(f"Clkout{i}DutyCycleStatic={_b(not dyn_dt)}")
                lines.append(f"Clkout{i}DutyTrimDynamic={_b(dyn_dt)}")
                lines.append(f"Clkout{i}DutyTrimStatic={_b(not dyn_dt)}")
                lines.append(f"Clkout{i}DutyTrimStaticFalling={_b(dt_dir == 0)}")
                lines.append(f"Clkout{i}DutyTrimStaticRising={_b(dt_dir == 1)}")
                lines.append(f"Clkout{i}DutyTrimStaticStep={dt_step}")
            else:
                # Channels 4-6 have simpler DutyCycle
                lines.append(f"Clkout{i}DutyCycleDynamic=false")
                lines.append(f"Clkout{i}DutyCycleDynamicValue=0")
                lines.append(f"Clkout{i}DutyCycleStatic=true")

            lines.append(f"Clkout{i}ExpectedFrequency={int(expected_freqs[i])}")
            lines.append(f"Clkout{i}PhaseDynamic=false")

            if i == 0 or i >= 4:
                lines.append(f"Clkout{i}PhaseStatic=true")

            lines.append(f"Clkout{i}PhaseStaticValue={pe_coarse}")
            lines.append(f"Clkout{i}Tolerance=0.0")
            lines.append(f"Clkout{i}VCODivideFactorDynamic={_b(not dyn_odiv)}")
            # Gowin IDE always uses odiv0_sel as the default dynamic value
            lines.append(f"Clkout{i}VCODivideFactorDynamicValue={c.odiv0_sel}")
            lines.append(f"Clkout{i}VCODivideFactorStatic={_b(dyn_odiv)}")
            lines.append(f"Clkout{i}VCODivideFactorStaticValue={odiv}")

            if i == 0:
                lines.append(f"Clkout{i}VCOFractionalDivideFactorDynamicValue={c.odiv0_frac_sel}")
                lines.append(f"Clkout{i}VCOFractionalDivideFactorStaticValue={c.odiv0_frac_sel}")

        # Remaining config
        lines.extend([
            "ClkoutDividerReset=false",
            f"ClockEnablePorts={_b(c.clock_enable_ports)}",
            "EnableCascade=false",
            f"EnableClkfbout={_b(c.enable_clkfbout)}",
            "EnableClkout0Divider=false",
            f"EnableClkout1={_b(c.clkout1_en)}",
            "EnableClkout1Divider=false",
            f"EnableClkout2={_b(c.clkout2_en)}",
            "EnableClkout2Divider=false",
            f"EnableClkout3={_b(c.clkout3_en)}",
            "EnableClkout3Divider=false",
            f"EnableClkout4={_b(c.clkout4_en)}",
            "EnableClkout4Divider=false",
            f"EnableClkout5={_b(c.clkout5_en)}",
            "EnableClkout5Divider=false",
            f"EnableClkout6={_b(c.clkout6_en)}",
            "EnableClkout6Divider=false",
            f"EnableLock={_b(c.enable_lock)}",
            f"EnableSsc={_b(c.ssc_enable)}",
            "GeneralMode=true",
            f"ICPSELDynamic={_b(c.dyn_icp_sel)}",
            f"ICPSELStatic={_b(not c.dyn_icp_sel)}",
            "ICPSELStaticValue=ICP1",
            f"Initialization_Clock_Frequency={c.init_clk_freq}",
            "LANG=0",
            f"LPFCAPStaticValue=C0",
            f"LPFRESStaticValue=X",
            f"LPFSELDynamic={_b(c.dyn_lpf_sel)}",
            f"LPFSELStatic={_b(not c.dyn_lpf_sel)}",
            f"PLLPowerDown={_b(c.pll_powerdown)}",
            f"PLLReset={_b(not c.pll_reset)}",
            f"clkfbExternal={_b(not c.clkfb_internal)}",
            f"clkfbExternalValue={c.clkfb_external_value}",
        ])

        path.write_text("\n".join(lines) + "\n")

    # ------------------------------------------------------------------
    # .mod file
    # ------------------------------------------------------------------

    def _generate_mod(self) -> None:
        path = self._pll_dir / f"{self.cfg.file_name}.mod"
        logger.info("Generating %s", path)

        c = self.cfg
        pll_abs = str(self._pll_dir) + "/"

        def _b(val: bool) -> str:
            return "true" if val else "false"

        lines = [
            f"-series {c.series}",
            f"-device {c.device}",
            f"-device_version {c.device_version}",
            f"-package {c.package}",
            f"-part_number {c.part_number}",
            "",
            "",
            f"-mod_name {c.inner_module_name}",
            f"-file_name {c.inner_file_name}",
            f"-path {pll_abs}",
            "-type PLL_ADV",
            "-file_type vlg",
            "-ip_version 1.0",
            f"-ssc {_b(c.ssc_enable)}",
            f"-clock_en {_b(c.clock_enable_ports)}",
            f"-rst {_b(c.pll_reset)}",
            f"-rst_pwd {_b(c.pll_powerdown)}",
            f"-rst_i {_b(c.reset_i_en)}",
            f"-rst_o {_b(c.reset_o_en)}",
            f"-fclkin {int(c.fclkin)}",
            f"-dyn_idiv_sel {_b(c.dyn_idiv)}",
            f"-idiv_sel {c.idiv_sel}",
            f"-clkfb_sel {'0' if c.clkfb_internal else '1'}",
            f"-dyn_fbdiv_sel {_b(c.dyn_fbdiv)}",
            f"-fbdiv_sel {c.fbdiv_sel}",
            f"-dyn_icp_sel {_b(c.dyn_icp_sel)}",
            f"-dyn_lpf_sel {_b(c.dyn_lpf_sel)}",
            f"-en_lock {_b(c.enable_lock)}",
            f"-dyn_dpa_en {_b(c.dyn_dpa_en)}",
        ]

        # CLKOUT0 config
        lines.extend([
            f"-clkout0_bypass {_b(c.clkout0_bypass)}",
            f"-dyn_odiv0_sel {_b(c.dyn_odiv0)}",
            f"-odiv0_sel {c.odiv0_sel}",
            f"-odiv0_frac_sel {c.odiv0_frac_sel}",
            f"-dyn_dt0_sel {_b(c.dyn_dt0)}",
            f"-clkout0_dt_dir {c.clkout0_dt_dir}",
            f"-clkout0_dt_step {c.clkout0_dt_step}",
            f"-dyn_pe0_sel false",
            f"-clkout0_pe_coarse {c.clkout0_pe_coarse}",
            f"-clkout0_pe_fine {c.clkout0_pe_fine}",
            f"-de0_en {_b(c.de0_en)}",
            f"-dyn_dt0_sel {_b(c.dyn_dt0)}",
            f"-clkout0_dt_dir {c.clkout0_dt_dir}",
            f"-clkout0_dt_step {c.clkout0_dt_step}",
        ])

        # CLKOUT1 config (if enabled)
        lines.append(f"-en_clkout1 {_b(c.clkout1_en)}")
        if c.clkout1_en:
            lines.extend([
                f"-clkout1_bypass {_b(c.clkout1_bypass)}",
                f"-dyn_odiv1_sel {_b(c.dyn_odiv1)}",
                f"-odiv1_sel {c.odiv1_sel}",
                f"-dyn_dt1_sel {_b(c.dyn_dt1)}",
                f"-clkout1_dt_dir {c.clkout1_dt_dir}",
                f"-clkout1_dt_step {c.clkout1_dt_step}",
                f"-dyn_pe1_sel false",
                f"-clkout1_pe_coarse {c.clkout1_pe_coarse}",
                f"-clkout1_pe_fine {c.clkout1_pe_fine}",
                f"-de1_en {_b(c.de1_en)}",
            ])

        # CLKOUT2-6 enable flags
        for i in range(2, 7):
            en = getattr(c, f"clkout{i}_en")
            lines.append(f"-en_clkout{i} {_b(en)}")

        # Feedback output and MDIV
        lines.extend([
            f"-en_clkfbout {_b(c.enable_clkfbout)}",
            f"-dyn_mdiv_sel {_b(c.dyn_mdiv)}",
            f"-mdiv_sel {c.mdiv_sel}",
            f"-mdiv_frac_sel {c.mdiv_frac_sel}",
        ])

        path.write_text("\n".join(lines) + "\n")

    # ------------------------------------------------------------------
    # Run GowinModGen
    # ------------------------------------------------------------------

    def _run_gowin_modgen(self) -> None:
        modgen = self._gowin / "IDE" / "bin" / "GowinModGen"
        mod_file = self._pll_dir / f"{self.cfg.file_name}.mod"
        self._run_tool(
            [str(modgen), "-do", str(mod_file)],
            "GowinModGen (PLL inner module)",
        )

    # ------------------------------------------------------------------
    # Top-level wrapper .v
    # ------------------------------------------------------------------

    def _generate_top_v(self) -> None:
        path = self._pll_dir / f"{self.cfg.file_name}.v"
        logger.info("Generating %s", path)

        c = self.cfg

        # Build port list
        ports = ["clkin", "init_clk"]
        for i in c.enabled_clkouts:
            ports.append(f"clkout{i}")
        if c.enable_lock:
            ports.append("lock")

        port_list = ",\n    ".join(ports)

        # Build input/output declarations
        decls = ["input clkin;", "input init_clk;"]
        for i in c.enabled_clkouts:
            decls.append(f"output clkout{i};")
        if c.enable_lock:
            decls.append("output lock;")

        # Internal wires
        decls.extend([
            "wire [5:0] icpsel;",
            "wire [2:0] lpfres;",
            "wire pll_lock;",
            "wire pll_rst;",
        ])

        decl_block = "\n".join(decls)

        # Build inner module instantiation port connections
        # Gowin IDE lists clock outputs in reverse order
        inner_ports = []
        for i in reversed(c.enabled_clkouts):
            inner_ports.append(f"        .clkout{i}(clkout{i})")
        inner_ports.append("        .lock(pll_lock)")
        inner_ports.append("        .clkin(clkin)")
        inner_ports.append("        .reset(pll_rst)")
        inner_ports.append("        .icpsel(icpsel)")
        inner_ports.append("        .lpfres(lpfres)")
        inner_ports.append(f"        .lpfcap({c.lpf_cap})")

        inner_port_block = ",\n".join(inner_ports)

        # Build PLL_INIT instantiation
        pll_init_ports = [
            "        .CLKIN(init_clk)",
            "        .I_RST(1'b0)",
            "        .O_RST(pll_rst)",
            "        .PLLLOCK(pll_lock)",
        ]
        if c.enable_lock:
            pll_init_ports.append("        .O_LOCK(lock)")
        else:
            pll_init_ports.append("        .O_LOCK()")
        pll_init_ports.extend([
            "        .ICPSEL(icpsel)",
            "        .LPFRES(lpfres)",
        ])

        pll_init_port_block = ",\n".join(pll_init_ports)

        content = f"""\
module {c.module_name}(
    {port_list}
);


{decl_block}


    {c.inner_module_name} u_pll(
{inner_port_block}
    );


    PLL_INIT u_pll_init(
{pll_init_port_block}
    );
    defparam u_pll_init.CLK_PERIOD = {c.clk_period_ns};
    defparam u_pll_init.MULTI_FAC = {c.mdiv_sel};


endmodule
"""
        path.write_text(content)

    # ------------------------------------------------------------------
    # Top-level template _tmp.v
    # ------------------------------------------------------------------

    def _generate_top_tmp_v(self) -> None:
        path = self._pll_dir / f"{self.cfg.file_name}_tmp.v"
        logger.info("Generating %s", path)

        c = self.cfg

        # Build port list for template
        ports = []
        ports.append(f"        .clkin(clkin), //input  clkin")
        ports.append(f"        .init_clk(init_clk), //input  init_clk")
        for i in c.enabled_clkouts:
            ports.append(f"        .clkout{i}(clkout{i}), //output  clkout{i}")
        if c.enable_lock:
            # Last port has no trailing comma
            ports.append(f"        .lock(lock) //output  lock")

        # Remove trailing comma from the last clkout if lock is not enabled
        if not c.enable_lock and ports:
            last = ports[-1]
            # Replace the comma after the closing paren
            last = last.replace("),", ")")
            ports[-1] = last

        port_block = "\n".join(ports)

        content = f"""\
//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Part Number: {c.part_number}
//Device: {c.device}
//Device Version: {c.device_version}


//Change the instance name and port connections to the signal names
//--------Copy here to design--------
    {c.module_name} your_instance_name(
{port_block}
);


//--------Copy end-------------------
"""
        path.write_text(content)

    # ------------------------------------------------------------------
    # Copy PLL_INIT
    # ------------------------------------------------------------------

    def _copy_pll_init(self) -> None:
        dest = self._out / "src" / "pll_init.v"
        logger.info("Copying pll_init.v to %s", dest)

        # Try to find it in the Gowin IDE installation first
        gowin_pll_init = self._gowin / "IDE" / "ipcore" / "PLL_ADV" / "data" / "pll_init.v"
        if gowin_pll_init.exists():
            shutil.copy2(gowin_pll_init, dest)
            logger.info("Copied pll_init.v from Gowin IDE: %s", gowin_pll_init)
            return

        # Also check alternative locations
        alt_paths = [
            self._gowin / "IDE" / "ipcore" / "pll_init.v",
            self._gowin / "IDE" / "bin" / "prim_syns" / "gw5a" / "pll_init.v",
        ]
        for alt in alt_paths:
            if alt.exists():
                shutil.copy2(alt, dest)
                logger.info("Copied pll_init.v from: %s", alt)
                return

        # Fall back to embedded version
        logger.info("Using embedded pll_init.v (not found in Gowin IDE)")
        dest.write_text(_PLL_INIT_V)

    # ------------------------------------------------------------------
    # Tool execution helper (same pattern as gowin_pcie_gen.py)
    # ------------------------------------------------------------------

    def _run_tool(self, cmd: list[str], description: str, cwd: Optional[Path] = None) -> None:
        """Run an external tool with logging and error handling.

        Sets ``LD_LIBRARY_PATH`` so that Gowin binaries use the Qt (and
        other) shared libraries bundled with the IDE instead of any
        system-installed versions that may be incompatible.
        """
        logger.info("Running: %s", description)
        logger.debug("Command: %s", " ".join(cmd))

        # Build a subprocess environment that prioritises the Gowin IDE
        # libraries.  This avoids "Cannot mix incompatible Qt library"
        # errors caused by the system Qt being picked up instead.
        ide_lib = str(self._gowin / "IDE" / "lib")
        ide_bin = str(self._gowin / "IDE" / "bin")
        env = os.environ.copy()
        existing_ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = (
            f"{ide_lib}:{existing_ld}" if existing_ld else ide_lib
        )
        # Also ensure IDE/bin is on PATH so wrapper tools can find siblings.
        existing_path = env.get("PATH", "")
        if ide_bin not in existing_path:
            env["PATH"] = f"{ide_bin}:{existing_path}" if existing_path else ide_bin
        # Point Qt to the IDE's own plugin directory so the platform
        # plugins (xcb, offscreen, …) match the bundled Qt version.
        ide_plugins = str(self._gowin / "IDE" / "plugins" / "qt")
        env["QT_PLUGIN_PATH"] = ide_plugins
        # Use the offscreen platform so no display server is required.
        env.setdefault("QT_QPA_PLATFORM", "offscreen")
        # The Gowin IDE bundles an older libfreetype.so.6 that lacks
        # FT_Done_MM_Var, which the system libfontconfig.so.1 requires.
        # Preload the system libfreetype to avoid the symbol conflict.
        sys_freetype = Path("/lib/x86_64-linux-gnu/libfreetype.so.6")
        if sys_freetype.exists():
            existing_preload = env.get("LD_PRELOAD", "")
            env["LD_PRELOAD"] = (
                f"{sys_freetype}:{existing_preload}" if existing_preload
                else str(sys_freetype)
            )

        try:
            result = subprocess.run(
                cmd,
                cwd=str(cwd) if cwd else None,
                capture_output=True,
                text=True,
                timeout=600,
                env=env,
            )
        except FileNotFoundError:
            raise FileNotFoundError(
                f"Tool not found: {cmd[0]}. "
                f"Ensure the Gowin IDE is installed at: {self._gowin}"
            )
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"{description} timed out after 600 seconds")

        if result.stdout:
            for line in result.stdout.strip().splitlines():
                logger.debug("  [stdout] %s", line)
        if result.stderr:
            for line in result.stderr.strip().splitlines():
                logger.debug("  [stderr] %s", line)

        if result.returncode != 0:
            logger.error("%s failed (exit code %d)", description, result.returncode)
            if result.stderr:
                logger.error("stderr:\n%s", result.stderr)
            raise RuntimeError(
                f"{description} failed with exit code {result.returncode}.\n"
                f"stderr: {result.stderr[:2000]}"
            )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate Gowin PLL IP cores without the GUI.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              %(prog)s --gowin-dir ./Gowin_V1.9.12.01_linux --output-dir ./output
              %(prog)s --gowin-dir ./Gowin_V1.9.12.01_linux --output-dir ./output \\
                       --fclkin 100 --mdiv 16 --odiv0 8 --odiv1 4 --clkout1
              %(prog)s --gowin-dir ./Gowin_V1.9.12.01_linux --output-dir ./output \\
                       --module-name My_PLL --file-name my_pll
        """),
    )

    parser.add_argument("--gowin-dir", required=True,
                        help="Path to Gowin IDE installation")
    parser.add_argument("--output-dir", required=True,
                        help="Output directory for generated project")

    # Device
    parser.add_argument("--series", default="GW5AST",
                        help="Device series (default: GW5AST)")
    parser.add_argument("--device", default="GW5AST-138",
                        help="Target device (default: GW5AST-138)")
    parser.add_argument("--device-version", default="C",
                        help="Device version (default: C)")
    parser.add_argument("--package", default="FCPBGA676A",
                        help="Package (default: FCPBGA676A)")
    parser.add_argument("--part-number", default="GW5AST-LV138FPG676AC1/I0",
                        help="Full part number")
    parser.add_argument("--target-device", default="gw5ast138c-003",
                        help="Target device ID for project files")

    # Naming
    parser.add_argument("--module-name", default="Gowin_PLL",
                        help="Top-level module name (default: Gowin_PLL)")
    parser.add_argument("--file-name", default="gowin_pll",
                        help="Base file name (default: gowin_pll)")

    # Clock
    parser.add_argument("--fclkin", type=float, default=100.0,
                        help="Input clock frequency in MHz (default: 100)")

    # Dividers
    parser.add_argument("--idiv", type=int, default=2,
                        help="Input divider IDIV_SEL (default: 2)")
    parser.add_argument("--fbdiv", type=int, default=1,
                        help="Feedback divider FBDIV_SEL (default: 1)")
    parser.add_argument("--mdiv", type=int, default=16,
                        help="VCO multiplier MDIV_SEL (default: 16)")
    parser.add_argument("--mdiv-frac", type=int, default=0,
                        help="VCO fractional MDIV_FRAC_SEL (default: 0)")
    parser.add_argument("--odiv0", type=int, default=8,
                        help="Output 0 divider ODIV0_SEL (default: 8)")
    parser.add_argument("--odiv0-frac", type=int, default=0,
                        help="Output 0 fractional ODIV0_FRAC_SEL (default: 0)")
    parser.add_argument("--odiv1", type=int, default=4,
                        help="Output 1 divider ODIV1_SEL (default: 4)")
    parser.add_argument("--odiv2", type=int, default=8,
                        help="Output 2 divider ODIV2_SEL (default: 8)")
    parser.add_argument("--odiv3", type=int, default=8,
                        help="Output 3 divider ODIV3_SEL (default: 8)")
    parser.add_argument("--odiv4", type=int, default=8,
                        help="Output 4 divider ODIV4_SEL (default: 8)")
    parser.add_argument("--odiv5", type=int, default=8,
                        help="Output 5 divider ODIV5_SEL (default: 8)")
    parser.add_argument("--odiv6", type=int, default=8,
                        help="Output 6 divider ODIV6_SEL (default: 8)")

    # Channel enables
    parser.add_argument("--clkout1", action="store_true", default=True,
                        help="Enable CLKOUT1 (default: enabled)")
    parser.add_argument("--no-clkout1", action="store_true",
                        help="Disable CLKOUT1")
    parser.add_argument("--clkout2", action="store_true",
                        help="Enable CLKOUT2")
    parser.add_argument("--clkout3", action="store_true",
                        help="Enable CLKOUT3")
    parser.add_argument("--clkout4", action="store_true",
                        help="Enable CLKOUT4")
    parser.add_argument("--clkout5", action="store_true",
                        help="Enable CLKOUT5")
    parser.add_argument("--clkout6", action="store_true",
                        help="Enable CLKOUT6")

    # Features
    parser.add_argument("--no-lock", action="store_true",
                        help="Disable lock output")
    parser.add_argument("--enable-clkfbout", action="store_true",
                        help="Enable feedback clock output")
    parser.add_argument("--clkfb-external", action="store_true",
                        help="Use external feedback clock (default: internal)")

    # Dynamic control
    parser.add_argument("--no-dyn-icp", action="store_true",
                        help="Disable dynamic ICP selection")
    parser.add_argument("--no-dyn-lpf", action="store_true",
                        help="Disable dynamic LPF selection")

    # Reset
    parser.add_argument("--no-reset", action="store_true",
                        help="Disable PLL reset")
    parser.add_argument("--pll-powerdown", action="store_true",
                        help="Enable PLL power down")

    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable verbose (DEBUG) logging")

    args = parser.parse_args()

    # Configure logging
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    config = PLLConfig(
        gowin_dir=args.gowin_dir,
        output_dir=args.output_dir,
        series=args.series,
        device=args.device,
        device_version=args.device_version,
        package=args.package,
        part_number=args.part_number,
        target_device=args.target_device,
        module_name=args.module_name,
        file_name=args.file_name,
        fclkin=args.fclkin,
        idiv_sel=args.idiv,
        fbdiv_sel=args.fbdiv,
        mdiv_sel=args.mdiv,
        mdiv_frac_sel=args.mdiv_frac,
        odiv0_sel=args.odiv0,
        odiv0_frac_sel=args.odiv0_frac,
        odiv1_sel=args.odiv1,
        odiv2_sel=args.odiv2,
        odiv3_sel=args.odiv3,
        odiv4_sel=args.odiv4,
        odiv5_sel=args.odiv5,
        odiv6_sel=args.odiv6,
        clkout1_en=not args.no_clkout1,
        clkout2_en=args.clkout2,
        clkout3_en=args.clkout3,
        clkout4_en=args.clkout4,
        clkout5_en=args.clkout5,
        clkout6_en=args.clkout6,
        enable_lock=not args.no_lock,
        enable_clkfbout=args.enable_clkfbout,
        clkfb_internal=not args.clkfb_external,
        dyn_icp_sel=not args.no_dyn_icp,
        dyn_lpf_sel=not args.no_dyn_lpf,
        pll_reset=not args.no_reset,
        pll_powerdown=args.pll_powerdown,
    )

    gen = GowinPLLGenerator(config)
    gen.generate()


if __name__ == "__main__":
    main()
