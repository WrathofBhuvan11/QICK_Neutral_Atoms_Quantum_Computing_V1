// ----------------------------------------------------------------------
// coord_matcher.sv -- 8-pixel-per-beat coordinate matcher
// ----------------------------------------------------------------------
// The pixel stream delivers PIXELS_PER_BEAT = 8 pixels per beat, so
// curr_x advances in steps of 8. A qubit centre Qx can therefore land
// on any of the eight pixel slots in a beat. The matcher checks all
// eight positions in parallel by asking: "does Qx+1 appear as slot
// 0..7 of this beat?". The +1 offset comes from the ROI window
// geometry: the captured 3x3 ROI is anchored such that Qx+1 is the
// right-hand column at the moment of capture.
//
//   offset s:  curr_x + s == Qx+1      for s in 0 .. PIXELS_PER_BEAT-1
//
// At most one offset can fire per qubit per frame row because curr_x
// steps by 8 and the minimum qubit spacing is 51 px (> 8), so no two
// qubits share a beat.
//
// Pipeline (3 stages -- pixel beat N -> o_match_found at cycle N+3):
//   Stage 1a : 8 parallel offset compares per qubit -> match_p_reg.
//              This is the wide compare cone; it gets its own clock
//              period so the q_x_r -> comparator nets close 520 MHz.
//   Stage 1b : per-qubit reduce -> matches_ (any slot hit) and
//              match_offset_reg (lowest set slot).
//   Stage 2  : priority encoder across the 100 qubits -> outputs.
// Splittin Stage 1 into 1a/1b adds +1 cycle of
// roi_extractor delays its pixel-side inputs by 1
// cycle to stay aligned with the later match trigger.
// ----------------------------------------------------------------------

`timescale 1ns / 1ps
import params_pkg::*;

module coord_matcher (
    input  logic       i_clk,
    input  logic       i_rst_n,

    // Qubit coordinate LUT (from qubit_lookup_axi)
    input  logic [COORD_WIDTH-1:0] i_q_x [0:NUM_QUBITS-1],
    input  logic [COORD_WIDTH-1:0] i_q_y [0:NUM_QUBITS-1],

    // Pixel stream (from pixel_injector)
    input  logic [COORD_WIDTH-1:0] i_curr_x,   // column of p0 in this beat
    input  logic [COORD_WIDTH-1:0] i_curr_y,
    input  logic       i_valid,

    // Sync signals (from pixel_injector)
    input  logic       i_sync_lval,
    input  logic       i_sync_fval,

    // Match outputs
    output logic       o_match_found,
    output logic [QUBIT_ID_WIDTH-1:0] o_qubit_index,
    output logic [MATCH_OFFSET_WIDTH-1:0] o_match_offset,  // 0..7, which pixel slot held Qx+1
    output logic       o_valid_out
);

    //-------------------------------------------------------------------
    // Stage 1a state: per-qubit, per-slot compare results.
    //   match_p_reg[i][s] = 1  when  curr_x + s == Qx+1  on qubit i's row.
    //-------------------------------------------------------------------
    logic [PIXELS_PER_BEAT-1:0] match_p_reg [0:NUM_QUBITS-1];
    logic valid_p1a;
    logic sync_lval_p1a, sync_fval_p1a;

    // Combinational scratch for the per-slot compares -- module scope so
    // the 8 compares share a single mux into match_p_reg.
    logic [PIXELS_PER_BEAT-1:0] match_p;

    //-------------------------------------------------------------------
    // Stage 1b state: per-qubit match flag + chosen offset.
    //-------------------------------------------------------------------
    logic [NUM_QUBITS-1:0]             matches_;
    logic [MATCH_OFFSET_WIDTH-1:0]     match_offset_reg [0:NUM_QUBITS-1];
    logic valid_p1b;

    // Registered copy of the quasi-static qubit-coordinate inputs.
    // Pipeline cut so the long net from the coordinate synchronisers
    // ends at a flop, not at the 100-wide compare. Quasi-static path,
    // so this does not change pixel->match latency.
    logic [COORD_WIDTH-1:0] q_x_r [0:NUM_QUBITS-1];
    logic [COORD_WIDTH-1:0] q_y_r [0:NUM_QUBITS-1];

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            for (int i = 0; i < NUM_QUBITS; i++) begin
                q_x_r[i] <= '0;
                q_y_r[i] <= '0;
            end
        end else begin
            for (int i = 0; i < NUM_QUBITS; i++) begin
                q_x_r[i] <= i_q_x[i];
                q_y_r[i] <= i_q_y[i];
            end
        end
    end

    //-------------------------------------------------------------------
    // Stage 1a: 8 parallel offset compares per qubit, registered into
    // match_p_reg. Giving the wide q_x_r -> comparator cone its own
    // clock period is the 520 MHz timing-closure cut.
    //-------------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            for (int j = 0; j < NUM_QUBITS; j++) match_p_reg[j] <= '0;
            valid_p1a     <= 1'b0;
            sync_lval_p1a <= 1'b0;
            sync_fval_p1a <= 1'b0;
        end else begin
            valid_p1a     <= i_valid;
            sync_lval_p1a <= i_sync_lval;
            sync_fval_p1a <= i_sync_fval;

            if (i_valid && i_sync_fval && i_sync_lval) begin
                for (int i = 0; i < NUM_QUBITS; i++) begin
                    // Does Qx+1 land on slot 0..PIXELS_PER_BEAT-1 of this
                    // beat:  curr_x + s == Qx+1  on the qubit's row.
                    for (int s = 0; s < PIXELS_PER_BEAT; s++)
                        match_p[s] = (i_curr_x + COORD_WIDTH'(s)
                                          == q_x_r[i] + COORD_WIDTH'(1))
                                     && (i_curr_y == q_y_r[i]);
                    match_p_reg[i] <= match_p;
                end
            end else begin
                for (int j = 0; j < NUM_QUBITS; j++) match_p_reg[j] <= '0;
            end
        end
    end

    //-------------------------------------------------------------------
    // Stage 1b: reduce each qubit's PIXELS_PER_BEAT compare bits to a
    // single match flag (matches_) and the chosen offset (lowest set
    // slot). Only ~3 levels of logic -- comfortably meets timing.
    //-------------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            matches_  <= '0;
            for (int j = 0; j < NUM_QUBITS; j++) match_offset_reg[j] <= '0;
            valid_p1b <= 1'b0;
        end else begin
            valid_p1b <= valid_p1a;

            if (valid_p1a && sync_fval_p1a && sync_lval_p1a) begin
                for (int i = 0; i < NUM_QUBITS; i++) begin
                    automatic logic [MATCH_OFFSET_WIDTH-1:0] off = '0;
                    // Lowest set slot wins (only one bit can be set anyway).
                    for (int s = PIXELS_PER_BEAT-1; s >= 0; s--)
                        if (match_p_reg[i][s]) off = MATCH_OFFSET_WIDTH'(s);
                    matches_[i]         <= |match_p_reg[i];
                    match_offset_reg[i] <= off;
                end
            end else begin
                matches_ <= '0;
                for (int j = 0; j < NUM_QUBITS; j++) match_offset_reg[j] <= '0;
            end
        end
    end

    //-------------------------------------------------------------------
    // Pipeline stage 2: priority encoder.
    // Iterates high-to-low so the lowest index wins if (against design
    // intent) more than one match fires in the same cycle.
    //-------------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            o_match_found  <= 1'b0;
            o_qubit_index  <= '0;
            o_match_offset <= '0;
            o_valid_out    <= 1'b0;
        end else begin
            o_valid_out    <= valid_p1b;
            o_match_found  <= 1'b0;
            o_qubit_index  <= '0;
            o_match_offset <= '0;

            for (int i = NUM_QUBITS-1; i >= 0; i--) begin
                if (matches_[i]) begin
                    o_match_found  <= 1'b1;
                    o_qubit_index  <= QUBIT_ID_WIDTH'(i);
                    o_match_offset <= match_offset_reg[i];
                end
            end
        end
    end

// synthesis translate_off
    // Simulation-only sanity check: by construction, qubit spacing
    // guarantees a single match per beat. If two ever fire the spacing
    // assumption has been violated.
    logic [7:0] match_count;
    always_comb begin
        match_count = 0;
        for (int i = 0; i < NUM_QUBITS; i++)
            match_count += matches_[i];
    end

    property p_single_match;
        @(posedge i_clk) disable iff (!i_rst_n)
        (match_count > 0) |-> (match_count == 1);
    endproperty

    assert property (p_single_match)
    else $warning("Multiple qubit matches at (%0d,%0d) -- check spacing..", i_curr_x, i_curr_y);
// synthesis translate_on

endmodule
