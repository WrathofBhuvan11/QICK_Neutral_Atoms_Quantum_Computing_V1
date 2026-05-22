#-----------------------------------------------------------------------
# design_qubit_readout_wrapper.xdc
# Target : ZCU216 (XCZU49DR-2FFVF1760E)  |  Vivado 2023.2
# Flow   : BD wrapper synthesis + implementation
#
# MAX_FANOUT=16 on u_lut write-enable nets
# pblock_qubit_lut tightened from X0:X59 Y0:Y149
#           to X0:X19 Y0:Y59
#
# Pixel stream width: 64-bit AXI4-Stream (8 px/beat, was 32-bit / 4 px).
#   - axi_vdma_0 MM2S stream and datastream_processor_0/s_axis_pix are
#     both 64-bit; no datawidth converter is inserted on that net.
#   - The input CDC FIFO (u_in_cdc) data width grew 34 -> 66 bits, but
#     its DEPTH is unchanged (16), so the gray-code POINTER widths are
#     unchanged. The CDC set_max_delay constraints (Section 4) need NO
#     change -- they target the pointer regs, not the data path.
#   - coord_matcher now runs 8 comparators/qubit (was 4), so the
#     pixel-x/y and matcher control-net MAX_FANOUT (Sections 9 / 9b)
#     matter even more; the values are unchanged.
#   - No constraint in this file changes for the 4 -> 8 px/beat widening.
#-----------------------------------------------------------------------


#-----------------------------------------------------------------------
# SECTION 1: CLOCK DEFINITIONS
# Each clock defined EXACTLY ONCE.
# clk_300 -> PS8 PLCLK[0]       (299.97 MHz actual)
# clk_520 -> MMCM CLKOUT0       (519.99 MHz actual)
#-----------------------------------------------------------------------

create_clock -period 3.334 -name clk_300 \
    [get_pins design_qubit_readout_i/zynq_ultra_ps_e_0/inst/PS8_i/PLCLK[0]]

create_clock -period 1.9231 -name clk_520 \
    [get_pins design_qubit_readout_i/clk_wiz_0/inst/mmcme4_adv_inst/CLKOUT0]


#-----------------------------------------------------------------------
# SECTION 2: SILENCE CLK_WIZ SECONDARY CLOCK
#-----------------------------------------------------------------------
set_clock_groups -logically_exclusive \
    -group [get_clocks clk_300] \
    -group [get_clocks -quiet -of_objects \
        [get_pins design_qubit_readout_i/clk_wiz_0/inst/clk_in1]]


#-----------------------------------------------------------------------
# SECTION 3: ASYNC CLOCK DOMAINS
#-----------------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks clk_300] \
    -group [get_clocks clk_520]


#-----------------------------------------------------------------------
# SECTION 4: ASYNC FIFO GRAY-CODE POINTER SYNCHRONISERS
#
# u_in_cdc  : write@300 MHz, read@520 MHz
# u_out_cdc : write@520 MHz, read@300 MHz
#-----------------------------------------------------------------------

# u_in_cdc: rd_ptr feedback 520->300
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/rd_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/rd_gray_wr_s1_reg* && IS_SEQUENTIAL == 1}] \
    3.334

# u_in_cdc: wr_ptr forward 300->520
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/wr_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_in_cdc/wr_gray_rd_s1_reg* && IS_SEQUENTIAL == 1}] \
    1.9231

# u_out_cdc: rd_ptr feedback 300->520
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/rd_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/rd_gray_wr_s1_reg* && IS_SEQUENTIAL == 1}] \
    1.9231

# u_out_cdc: wr_ptr forward 520->300
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/wr_ptr_gray_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_out_cdc/wr_gray_rd_s1_reg* && IS_SEQUENTIAL == 1}] \
    3.334


#-----------------------------------------------------------------------
# SECTION 5: lut_valid 2-FF SYNCHRONISER (300 -> 520 MHz)
#-----------------------------------------------------------------------
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_lut/lut_written_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *lut_valid_sync1_reg* && IS_SEQUENTIAL == 1}] \
    1.9231


#-----------------------------------------------------------------------
# SECTION 6: RESET SYNCHRONISER FALSE PATH
# proc_sys_reset_0 output FFs -> u_rst_sync 3-stage chain
#-----------------------------------------------------------------------
set_false_path \
    -from [get_cells -hier -filter \
        {NAME =~ *proc_sys_reset_0* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *u_rst_sync/sync_chain_reg* && IS_SEQUENTIAL == 1}]


#-----------------------------------------------------------------------
# SECTION 7: QUBIT COORDINATE 2-FF SYNCHRONISERS (300 -> 520 MHz)
#-----------------------------------------------------------------------
set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_lut/reg_x_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *q_x_sync1_reg* && IS_SEQUENTIAL == 1}] \
    1.9231

set_max_delay -datapath_only \
    -from [get_cells -hier -filter \
        {NAME =~ *u_lut/reg_y_reg* && IS_SEQUENTIAL == 1}] \
    -to   [get_cells -hier -filter \
        {NAME =~ *q_y_sync1_reg* && IS_SEQUENTIAL == 1}] \
    1.9231


#-----------------------------------------------------------------------
# MAX_FANOUT on u_lut write-enable control nets
#-----------------------------------------------------------------------
set_property MAX_FANOUT 16 \
    [get_nets -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_lut*wr_fire_q*}]

set_property MAX_FANOUT 16 \
    [get_nets -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_lut*wr_fire_s0*}]

set_property MAX_FANOUT 16 \
    [get_nets -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_lut*aw_active*}]



#-----------------------------------------------------------------------
# SECTION 8: PHYSICAL FLOORPLANNING
#-----------------------------------------------------------------------

create_pblock pblock_qubit_readout
add_cells_to_pblock [get_pblocks pblock_qubit_readout] \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0* && IS_PRIMITIVE == 0}]
resize_pblock [get_pblocks pblock_qubit_readout] \
    -add {SLICE_X0Y0:SLICE_X149Y299}
resize_pblock [get_pblocks pblock_qubit_readout] \
    -add {DSP48E2_X0Y0:DSP48E2_X11Y119}
resize_pblock [get_pblocks pblock_qubit_readout] \
    -add {RAMB36_X0Y0:RAMB36_X5Y59}
set_property CONTAIN_ROUTING true [get_pblocks pblock_qubit_readout]
set_property IS_SOFT          true [get_pblocks pblock_qubit_readout]

#-----------------------------------------------------------------------
# pblock_qubit_lut 
#-----------------------------------------------------------------------
# SLICE_X0Y0:SLICE_X19Y59  (1200 SLICEs, near PS8 HPM0)
#      -> intra-pblock CE route < 200 ps
#-----------------------------------------------------------------------
create_pblock pblock_qubit_lut
add_cells_to_pblock [get_pblocks pblock_qubit_lut] \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_lut* && IS_PRIMITIVE == 0}]
resize_pblock [get_pblocks pblock_qubit_lut] \
    -add {SLICE_X0Y0:SLICE_X19Y59}
set_property IS_SOFT true [get_pblocks pblock_qubit_lut]


#-----------------------------------------------------------------------
# SECTION 9: PIXEL COORDINATE FANOUT  (pixel_injector -> coord_matcher)
#
#   design_qubit_readout_i/datastream_processor_0/inst/u_injector/o_pixel_x_reg[8:0]
#   design_qubit_readout_i/datastream_processor_0/inst/u_injector/o_pixel_y_reg[8:0]
#-----------------------------------------------------------------------
set_property -quiet MAX_FANOUT 10 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0/inst/u_injector/o_pixel_x_reg*}]

set_property -quiet MAX_FANOUT 10 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0/inst/u_injector/o_pixel_y_reg*}]

#-----------------------------------------------------------------------
# SECTION 9b: MATCHER CONTROL-NET FANOUT
#
# coord_matcher gates all 100 parallel qubit comparators with a small
# set of control signals (valid / sync_lval / sync_fval). In the
# post-route STA these surface as 2-level, very-high-fanout paths INTO
# u_match:
#   u_in_cdc/rd_data_valid  -> u_match/match_offset_reg   fanout ~87
#   u_injector/frame_active -> u_match/match_offset_reg   fanout ~54
# Net delay dominates (~1.4 ns). Same problem class as the pixel
# coordinate fanout in Section 9 -- replicate the source FF so each
# copy drives a small, local group of comparators.
#-----------------------------------------------------------------------
set_property -quiet MAX_FANOUT 12 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_in_cdc*rd_data_valid_reg*}]

set_property -quiet MAX_FANOUT 12 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_injector*frame_active_reg*}]

#-----------------------------------------------------------------------
# SECTION 9c: MATCHER COORDINATE-REGISTER FANOUT  (8 px/beat closure)
#
# After the 4 -> 8 px/beat widening, coord_matcher runs 8 offset
# comparators per qubit instead of 4. Post-route STA showed WNS
# ~ -0.24 ns at 520 MHz, with EVERY worst path inside u_match and
# net delay dominating (~1.4 ns of ~2.0 ns total, only 4-6 logic
# levels). The startpoints are the internal registered coordinate
# copies u_match/q_x_r_reg / q_y_r_reg, previously unconstrained, each
# fanning out to all 8 comparators of its qubit.
#
# MAX_FANOUT 6 lets Vivado replicate each coordinate register so a
# replica drives a small, local comparator group -> shorter routes.
# Pair this with phys_opt_design in the implementation flow. If a
# re-run still misses, tighten to 4.
#-----------------------------------------------------------------------
set_property -quiet MAX_FANOUT 6 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_match*q_x_r_reg*}]

set_property -quiet MAX_FANOUT 6 \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_match*q_y_r_reg*}]

#-----------------------------------------------------------------------
# pblock_match_cone -- co-locate coord_matcher with pixel_injector.
#
# q_x_sync*/q_y_sync* are deliberately NOT pinned : they are
# ASYNC_REG CDC synchronisers, best left for the tool to place
# adjacently for MTBF (governed by the set_max_delay in Section 7).
# The RTL q_x_r / q_y_r registers in coord_matcher now buffer them,
# so co-locating them in this region is no longer needed.
#-----------------------------------------------------------------------
create_pblock pblock_match_cone
add_cells_to_pblock [get_pblocks pblock_match_cone] \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_match* && IS_PRIMITIVE == 0}]
add_cells_to_pblock [get_pblocks pblock_match_cone] \
    [get_cells -quiet -hier -filter \
        {NAME =~ *datastream_processor_0*u_injector* && IS_PRIMITIVE == 0}]
resize_pblock [get_pblocks pblock_match_cone] \
    -add {SLICE_X20Y0:SLICE_X99Y139}
set_property IS_SOFT true [get_pblocks pblock_match_cone]
