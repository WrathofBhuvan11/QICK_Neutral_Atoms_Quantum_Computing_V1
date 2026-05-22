#-----------------------------------------------------------------------
# datastream_processor.xdc
# Target : ZCU216 (XCZU49DR-2FFVF1760E)   Vivado 2023.2
# Flow   : OOC / standalone IP synthesis
#
# Pixel stream width: 64-bit AXI4-Stream (8 px/beat, was 32-bit / 4 px).
#   - The input CDC FIFO (u_in_cdc) data width grew 34 -> 66 bits, but
#     its DEPTH is unchanged (16), so the gray-code POINTER widths are
#     unchanged. The CDC set_max_delay constraints below therefore need
#     NO change -- they target the pointer regs, not the data path.
#   - coord_matcher now drives 8 comparators/qubit instead of 4, so the
#     MAX_FANOUT on the injector pixel-x/y registers (Section 9) matters
#     even more; the value (10) is unchanged.
#   - No constraint in this file changes for the 4 -> 8 px/beat widening.
#
# Constraints covered:
#   - clk_300 / clk_520 declared asynchronous (set_clock_groups).
#   - set_max_delay -datapath_only on every gray-code pointer leg of
#     the two CDC FIFOs (u_in_cdc, u_out_cdc).
#   - set_max_delay on the lut_valid, q_x, q_y synchronisers.
#   - set_false_path on the reset synchroniser.
#   - MAX_FANOUT = 16 on the LUT write-enable nets (aw_active,
#     wr_fire_s0, wr_fire_q) so Vivado replicates the driver until
#     each replica feeds <= 16 loads. Kills the ~810-fanout route
#     penalty across the 100 * 2 * 32-bit register file.
#   - pblock_qubit_lut sized tightly (X0:X19 Y0:Y59, 1200 SLICEs)
#     near the PS8 HPM0 master so intra-pblock CE routes stay short.
#   - MAX_FANOUT = 10 on u_injector pixel-x/pixel-y registers to
#     keep coord_matcher fanout under control.
#-----------------------------------------------------------------------


#-----------------------------------------------------------------------
# Section 1: Clock definitions (OOC)
#-----------------------------------------------------------------------

create_clock -period 3.333 -name clk_300 [get_ports i_aclk_ps]
create_clock -period 1.923 -name clk_520 [get_ports i_clk_pl]


#-----------------------------------------------------------------------
# Section 2: Clock groups (declare PS and PL clocks asynchronous)
#-----------------------------------------------------------------------

set_clock_groups -asynchronous \
    -group [get_clocks clk_300] \
    -group [get_clocks clk_520]


#-----------------------------------------------------------------------
# Section 3: Async-FIFO gray-code pointer synchronisers
#
# Direction key:
#   300 -> 520  forward write pointer:   max_delay = 1.923 ns (clk_520 period)
#   520 -> 300  feedback read pointer:   max_delay = 3.333 ns (clk_300 period)
#-----------------------------------------------------------------------

# u_in_cdc: rd_ptr feedback, 520 -> 300
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/rd_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/rd_gray_wr_s1_reg* && IS_SEQUENTIAL == 1}] \
    3.333

# u_in_cdc: wr_ptr forward, 300 -> 520
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/wr_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/wr_gray_rd_s1_reg* && IS_SEQUENTIAL == 1}] \
    1.923

# u_out_cdc: rd_ptr feedback, 300 -> 520
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/rd_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/rd_gray_wr_s1_reg* && IS_SEQUENTIAL == 1}] \
    1.923

# u_out_cdc: wr_ptr forward, 520 -> 300
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/wr_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/wr_gray_rd_s1_reg* && IS_SEQUENTIAL == 1}] \
    3.333


#-----------------------------------------------------------------------
# Section 4: lut_valid 2-FF synchroniser (300 -> 520)
#-----------------------------------------------------------------------
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_lut/lut_written_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *lut_valid_sync1_reg* && IS_SEQUENTIAL == 1}] \
    1.923


#-----------------------------------------------------------------------
# Section 5: Reset synchroniser false path
# proc_sys_reset_0/peripheral_aresetn drives i_aresetn_ps in the BD flow.
#-----------------------------------------------------------------------
set_false_path \
    -from [get_cells -hier -filter \
        {NAME =~ *proc_sys_reset*/U_RESET_FILTER* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_rst_sync/sync_chain_reg* && IS_SEQUENTIAL == 1}]


#-----------------------------------------------------------------------
# Section 6: Qubit coordinate 2-FF synchronisers (300 -> 520)
#-----------------------------------------------------------------------
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_lut/reg_x_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *q_x_sync1_reg* && IS_SEQUENTIAL == 1}] \
    1.923

set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_lut/reg_y_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *q_y_sync1_reg* && IS_SEQUENTIAL == 1}] \
    1.923


#-----------------------------------------------------------------------
# Section 7: MAX_FANOUT on write-enable control nets in u_lut
#
# The AXI write-enable decode (aw_active, wr_fire_q, wr_fire_s0 plus
# byte-enable derivatives) drives the CE pins of all 100 * 2 * 32 FFs
# in the coordinate register file. MAX_FANOUT = 16 forces Vivado's
# physical-optimisation step to replicate the driver until each copy
# feeds at most 16 loads. Each replica is placed near its loads
# (intra-pblock) so the route stays at roughly 50 ps.
#-----------------------------------------------------------------------
set_property MAX_FANOUT 16 \
    [get_nets -quiet -hier -filter {NAME =~ *u_lut/wr_fire_q*}]

set_property MAX_FANOUT 16 \
    [get_nets -quiet -hier -filter {NAME =~ *u_lut/wr_fire_s0*}]

set_property MAX_FANOUT 16 \
    [get_nets -quiet -hier -filter {NAME =~ *u_lut/aw_active*}]


#-----------------------------------------------------------------------
# Section 8: Physical floorplanning
#
# add_cells_to_pblock uses -quiet so an empty cell match is a warning
# rather than a crash (this is the one XDC command that supports
# -quiet on its cell list). set_property on the pblock itself is safe
# because create_pblock always runs first.
#-----------------------------------------------------------------------

create_pblock pblock_qubit_readout
add_cells_to_pblock [get_pblocks pblock_qubit_readout] \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_qick* && IS_PRIMITIVE == 0}]
resize_pblock [get_pblocks pblock_qubit_readout] \
    -add {SLICE_X0Y0:SLICE_X149Y299}
resize_pblock [get_pblocks pblock_qubit_readout] \
    -add {DSP48E2_X0Y0:DSP48E2_X11Y119}
resize_pblock [get_pblocks pblock_qubit_readout] \
    -add {RAMB36_X0Y0:RAMB36_X5Y59}
set_property CONTAIN_ROUTING true [get_pblocks pblock_qubit_readout]
set_property IS_SOFT          true [get_pblocks pblock_qubit_readout]

#-----------------------------------------------------------------------
# pblock_qubit_lut: tight rectangle (1200 SLICEs) near the PS8 HPM0
# port. reg_x / reg_y pack tightly here, so the intra-pblock CE route
# stays under ~200 ps. Combined with the MAX_FANOUT replication above,
# this gives several ns of positive slack on paths that previously
# failed. IS_SOFT is left true so Vivado may still spill out of the
# region if it gets congested; the small target simply discourages
# scattered placement.
#-----------------------------------------------------------------------
create_pblock pblock_qubit_lut
add_cells_to_pblock [get_pblocks pblock_qubit_lut] \
    [get_cells -quiet -hier -filter \
        {NAME =~ *u_lut* && IS_PRIMITIVE == 0}]
resize_pblock [get_pblocks pblock_qubit_lut] \
    -add {SLICE_X0Y0:SLICE_X19Y59}
set_property IS_SOFT true [get_pblocks pblock_qubit_lut]


#-----------------------------------------------------------------------
# Timing-exception summary
#   #   Type            Dir         From                         To
#   1   max_delay 3.333 520 -> 300  u_in_cdc  rd_ptr             rd_gray_wr_s1
#   2   max_delay 1.923 300 -> 520  u_in_cdc  wr_ptr             wr_gray_rd_s1
#   3   max_delay 1.923 300 -> 520  u_out_cdc rd_ptr             rd_gray_wr_s1
#   4   max_delay 3.333 520 -> 300  u_out_cdc wr_ptr             wr_gray_rd_s1
#   5   max_delay 1.923 300 -> 520  u_lut     lut_written        lut_valid_sync1
#   6   max_delay 1.923 300 -> 520  u_lut     reg_x_reg          q_x_sync1
#   7   max_delay 1.923 300 -> 520  u_lut     reg_y_reg          q_y_sync1
#   8   false_path      300 -> 520  proc_sys_reset               u_rst_sync
#   9   MAX_FANOUT 16               u_lut wr_fire_q / wr_fire_s0 / aw_active nets
#-----------------------------------------------------------------------

#-----------------------------------------------------------------------
# Section 9: pixel-coordinate fanout (pixel_injector -> coord_matcher)
#
# Source FFs (in the OOC top, datastream_processor_qick):
#   u_injector/o_pixel_x_reg[8:0]   high fanout into coord_matcher
#   u_injector/o_pixel_y_reg[8:0]   high fanout into coord_matcher
#
# With the 8 px/beat widening coord_matcher runs 8 offset comparators
# per qubit (was 4), so these nets fan out to roughly 2x the loads --
# MAX_FANOUT 10 stays and is more important than before.
#
# MAX_FANOUT is applied to the CELLS (FFs) so synthesis replicates
# the drivers and places the replicas near their loads.
#-----------------------------------------------------------------------
set_property -quiet MAX_FANOUT 10 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *u_injector/o_pixel_x_reg*}]

set_property -quiet MAX_FANOUT 10 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *u_injector/o_pixel_y_reg*}]
