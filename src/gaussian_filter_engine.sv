// ----------------------------------------------------------------------
// gaussian_filter_engine.sv -- 4-stage pipelined 3x3 Gaussian classifier
// ----------------------------------------------------------------------
// Computes a 3x3 Gaussian-weighted sum over an extracted ROI and
// compares it against a runtime threshold to decide Rydberg vs Ground.
//
// The threshold used to be a compile-time parameter; it is now driven
// by the i_threshold[15:0] port so software can adjust it on the fly
// via the AXI4-Lite register (qubit_lookup_axi 0x324).
//
// Pipeline (4 stages):
//   Stage 1: multiply  -- 9 parallel 8x8 multipliers
//   Stage 2: row sums  -- 3 partial sums, breaks the 9-input adder tree
//   Stage 3: final sum -- 3-input adder
//   Stage 4: threshold compare and output
// ----------------------------------------------------------------------

`timescale 1ns / 1ps

module gaussian_filter_engine #(
    // Standard 3x3 Gaussian approximation (8-bit signed coefficients):
    //   1 2 1
    //   2 4 2
    //   1 2 1
    parameter signed [7:0] W_00 = 8'd1, parameter signed [7:0] W_01 = 8'd2, parameter signed [7:0] W_02 = 8'd1,
    parameter signed [7:0] W_10 = 8'd2, parameter signed [7:0] W_11 = 8'd4, parameter signed [7:0] W_12 = 8'd2,
    parameter signed [7:0] W_20 = 8'd1, parameter signed [7:0] W_21 = 8'd2, parameter signed [7:0] W_22 = 8'd1
)(
    input  logic        i_clk,
    input  logic        i_rst_n,
    input  logic [71:0] i_roi_data,
    input  logic        i_valid,
    input  logic [6:0]  i_base_id,

    // Runtime decision threshold. Unsigned 16-bit; final score fits in
    // 20 signed bits so no overflow when zero-extended for the compare.
    input  logic [15:0] i_threshold,

    output logic        o_decision, // 1 = Rydberg excited, 0 = Ground
    output logic [15:0] o_score,    // debug: weighted sum, low 16 bits
    output logic [6:0]  o_base_id,  // pipeline-aligned qubit ID
    output logic        o_valid     // pipeline-aligned valid
);

    // --- Stage 0: pixel unpack (combinational) ---
    // Window layout: p[row][0] = LEFT, p[row][1] = CENTER, p[row][2] = RIGHT
    //   Row 0 (bottom): bits [71:48]
    //   Row 1 (mid)   : bits [47:24]
    //   Row 2 (top)   : bits [23: 0]
    logic [7:0] p[2:0][2:0];
    always_comb begin
        // Row 0 (bottom): LEFT, CENTER, RIGHT
        p[0][0] = i_roi_data[71:64]; p[0][1] = i_roi_data[63:56]; p[0][2] = i_roi_data[55:48];
        // Row 1 (mid): LEFT, CENTER, RIGHT
        p[1][0] = i_roi_data[47:40]; p[1][1] = i_roi_data[39:32]; p[1][2] = i_roi_data[31:24];
        // Row 2 (top): LEFT, CENTER, RIGHT
        p[2][0] = i_roi_data[23:16]; p[2][1] = i_roi_data[15:8]; p[2][2] = i_roi_data[7:0];
    end

    // --- Stage 1: multiply ---
    logic signed [15:0] prod_reg [2:0][2:0];
    logic               valid_s1;
    logic [6:0]         id_s1;

    // --- Stage 2: partial row sums ---
    logic signed [16:0] sum_row0, sum_row1, sum_row2;
    logic               valid_s2;
    logic [6:0]         id_s2;

    // --- Stage 3: final sum ---
    logic signed [19:0] sum_total_reg;
    logic               valid_s3;
    logic [6:0]         id_s3;

    // --- Stage 4: threshold compare and output ---
    // Threshold is quasi-static (synchronised quasi-static signal in
    // the parent), so this combinational extension meets timing.
    logic signed [19:0] thresh_ext;
    assign thresh_ext = $signed({4'b0000, i_threshold}); // zero-extend to signed 20b

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            valid_s1 <= 1'b0; valid_s2 <= 1'b0; valid_s3 <= 1'b0; o_valid <= 1'b0;
            o_decision <= 1'b0; o_score <= '0; o_base_id <= '0;
        end else begin
            // -------------------------------------------------------
            // Stage 1: 9 parallel multipliers
            // -------------------------------------------------------
            prod_reg[0][0] <= $signed({1'b0, p[0][0]}) * W_00;
            prod_reg[0][1] <= $signed({1'b0, p[0][1]}) * W_01;
            prod_reg[0][2] <= $signed({1'b0, p[0][2]}) * W_02;

            prod_reg[1][0] <= $signed({1'b0, p[1][0]}) * W_10;
            prod_reg[1][1] <= $signed({1'b0, p[1][1]}) * W_11;
            prod_reg[1][2] <= $signed({1'b0, p[1][2]}) * W_12;

            prod_reg[2][0] <= $signed({1'b0, p[2][0]}) * W_20;
            prod_reg[2][1] <= $signed({1'b0, p[2][1]}) * W_21;
            prod_reg[2][2] <= $signed({1'b0, p[2][2]}) * W_22;

            valid_s1 <= i_valid;
            id_s1    <= i_base_id;

            // -------------------------------------------------------
            // Stage 2: partial row sums (avoids a 9-input adder tree)
            // -------------------------------------------------------
            sum_row0 <= prod_reg[0][0] + prod_reg[0][1] + prod_reg[0][2];
            sum_row1 <= prod_reg[1][0] + prod_reg[1][1] + prod_reg[1][2];
            sum_row2 <= prod_reg[2][0] + prod_reg[2][1] + prod_reg[2][2];

            valid_s2 <= valid_s1;
            id_s2    <= id_s1;

            // -------------------------------------------------------
            // Stage 3: final 3-input sum
            // -------------------------------------------------------
            sum_total_reg <= sum_row0 + sum_row1 + sum_row2;

            valid_s3 <= valid_s2;
            id_s3    <= id_s2;

            // -------------------------------------------------------
            // Stage 4: threshold compare and output.
            // thresh_ext is combinational from i_threshold; the signal
            // is quasi-static so this path meets 520 MHz easily.
            // -------------------------------------------------------
            o_decision <= (sum_total_reg > thresh_ext) ? 1'b1 : 1'b0;
            o_score    <= sum_total_reg[15:0];
            o_valid    <= valid_s3;
            o_base_id  <= id_s3;
        end
    end

endmodule

