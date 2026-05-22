// ----------------------------------------------------------------------
// read_stream.sv -- ROI storage reader with noise subtraction
// ----------------------------------------------------------------------
// Sweeps all ROWS_PER_BANK (25) rows of roi_storage, reading 4 ROI
// lanes plus 4 baseline-noise lanes in parallel. Applies a registered
// per-pixel saturating subtraction and forwards the cleaned ROIs to
// the Gaussian filter bank.
//
// Subtraction:
//   For each of the 9 pixel bytes in each 72-bit ROI word,
//     out[k] = (roi[k] > noise[k]) ? (roi[k] - noise[k]) : 0
//   This cancels camera dark current, fixed-pattern read noise and
//   any static optical background captured during the dark-mode phase.
//
// Default behaviour at power-on / pre-dark-frame:
//   The base_noise banks read back as zero (BRAM init), so subtracting
//   zero is a no-op and the pipeline behaves as if there were no
//   noise stage.
//
// Microarchitecture (STA for 520 MHz):
//   The subtract is implemented as TWO registered stages instead of
//   one. A previous single-cycle implementation packed the borrow-
//   based clamp-to-zero into the destination FF's SR pin, producing
//   a 4-level path from base_noise BRAM CLKARDCLK -> 9-bit subtract
//   -> borrow detect -> sub_lane_X /R pin
//   Splitting the pipeline into:
//     (a) registered raw byte-wise 9-bit subtract (sub_roi_raw),
//     (b) registered borrow-based clamp           (clamp_roi),
//   forces the BRAM-clk-to-out path to terminate in an ordinary D
//   input on diff_lane_X, well within budget, and the diff -> clamp
//   -> sub_lane_X path is only a single mux level.
//
// Latency: 6 cycles from i_start to first valid output beat
//   (address gen + BRAM registered read + registered BRAM-output
//    capture + registered raw subtract + registered clamp + output
//    register).
// ----------------------------------------------------------------------
`timescale 1ns / 1ps
import params_pkg::*;

module read_streamer (
    input               i_clk,
    input               i_rst_n,
    input  logic        i_start,

    // Read interface to roi_storage (ping-pong banks)
    output logic        o_rd_en,
    output logic [BANK_ADDR_WIDTH-1:0]  o_rd_addr,

    // 4 parallel ROI input lanes (from ping-pong banks)
    input  logic [ROI_BITS-1:0] i_rd_data_0,
    input  logic [ROI_BITS-1:0] i_rd_data_1,
    input  logic [ROI_BITS-1:0] i_rd_data_2,
    input  logic [ROI_BITS-1:0] i_rd_data_3,

    // 4 parallel baseline-noise lanes (from base_noise banks).
    // Same address as the ROI read, registered with 1-cycle BRAM
    // latency.
    input  logic [ROI_BITS-1:0] i_noise_data_0,
    input  logic [ROI_BITS-1:0] i_noise_data_1,
    input  logic [ROI_BITS-1:0] i_noise_data_2,
    input  logic [ROI_BITS-1:0] i_noise_data_3,

    // Outputs to the Gaussian filter bank (noise-subtracted)
    output logic [ROI_BITS-1:0] o_pixeldata_lane_0,
    output logic [ROI_BITS-1:0] o_pixeldata_lane_1,
    output logic [ROI_BITS-1:0] o_pixeldata_lane_2,
    output logic [ROI_BITS-1:0] o_pixeldata_lane_3,
    output logic [QUBIT_ID_WIDTH-1:0]  o_pixeldata_lane_base_id,
    output logic        o_pixeldata_lane_valid
);

    localparam MAX_ROW = ROWS_PER_BANK - 1;

    // ------------------------------------------------------------------
    // Width of the raw byte-wise diff vector: 9 bits per byte (8 data
    // bits + 1 borrow bit), packed across all 9 bytes of the ROI.
    // ------------------------------------------------------------------
    localparam int DIFF_BITS = (ROI_BITS / 8) * 9;   // 9 * 9 = 81

    // ------------------------------------------------------------------
    // Combinational byte-wise 9-bit subtract, no clamp.
    // For each byte i:
    //   diff[i] = {1'b0, a[i]} - {1'b0, b[i]}     (9-bit result)
    // The borrow flag is bit 8 of the diff for that byte.
    // ------------------------------------------------------------------
    function automatic logic [DIFF_BITS-1:0] sub_roi_raw(
        input logic [ROI_BITS-1:0] a,
        input logic [ROI_BITS-1:0] b
    );
        logic [DIFF_BITS-1:0] result;
        for (int i = 0; i < (ROI_BITS / 8); i++) begin
            result[i*9 +: 9] = {1'b0, a[i*8 +: 8]} - {1'b0, b[i*8 +: 8]};
        end
        return result;
    endfunction

    // ------------------------------------------------------------------
    // Combinational clamp from a registered diff vector:
    //   out[i] = borrow_i ? 0 : low_8_bits_of_diff_i
    // This is a single mux level per byte and lives between the
    // diff_lane_X register and the sub_lane_X register.
    // ------------------------------------------------------------------
    function automatic logic [ROI_BITS-1:0] clamp_roi(input logic [DIFF_BITS-1:0] d);
        logic [ROI_BITS-1:0] result;
        for (int i = 0; i < (ROI_BITS / 8); i++) begin
            result[i*8 +: 8] = d[i*9 + 8] ? 8'h00 : d[i*9 +: 8];
        end
        return result;
    endfunction

    // ------------------------------------------------------------------
    // Pipeline state
    //   Stage 1 : address generation (row_cnt, reading)
    //   Stage 2 : control delay aligning with BRAM registered output
    //   Stage 2b: registered BRAM outputs (rd_data_q_X / noise_data_q_X)
    //   Stage 3a: registered raw byte-wise subtract (diff_lane_X)
    //   Stage 3b: registered clamp (sub_lane_X)
    //   Stage 4 : output register (o_pixeldata_lane_*)
    // ------------------------------------------------------------------
    // Stage 1
    logic [ROW_COUNT_WIDTH-1:0] row_cnt;
    logic       reading;

    // Stage 2
    logic       valid_pipe;
    logic [QUBIT_ID_WIDTH-1:0] id_pipe;

    // Stage 2b -- registered BRAM outputs (pipeline cut for 520 MHz STA)
    logic [ROI_BITS-1:0] rd_data_q_0, rd_data_q_1, rd_data_q_2, rd_data_q_3;
    logic [ROI_BITS-1:0] noise_data_q_0, noise_data_q_1, noise_data_q_2, noise_data_q_3;
    logic                rd_valid_q;
    logic [QUBIT_ID_WIDTH-1:0] id_q;

    // Stage 3a -- raw subtract result, 9 bits per byte (carries the
    // borrow flag forward to stage 3b)
    logic [DIFF_BITS-1:0] diff_lane_0, diff_lane_1, diff_lane_2, diff_lane_3;
    logic                 diff_valid;
    logic [QUBIT_ID_WIDTH-1:0] diff_base_id;

    // Stage 3b -- clamped result, 8 bits per byte
    logic [ROI_BITS-1:0] sub_lane_0, sub_lane_1, sub_lane_2, sub_lane_3;
    logic                sub_valid;
    logic [QUBIT_ID_WIDTH-1:0] sub_base_id;

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            // Stage 1 reset
            o_rd_en                  <= 1'b0;
            o_rd_addr                <= '0;
            row_cnt                  <= '0;
            reading                  <= 1'b0;

            // Stage 2 reset
            valid_pipe               <= 1'b0;
            id_pipe                  <= '0;

            // Stage 2b reset
            rd_valid_q               <= 1'b0;
            id_q                     <= '0;
            rd_data_q_0              <= '0;
            rd_data_q_1              <= '0;
            rd_data_q_2              <= '0;
            rd_data_q_3              <= '0;
            noise_data_q_0           <= '0;
            noise_data_q_1           <= '0;
            noise_data_q_2           <= '0;
            noise_data_q_3           <= '0;

            // Stage 3a reset
            diff_valid               <= 1'b0;
            diff_base_id             <= '0;
            diff_lane_0              <= '0;
            diff_lane_1              <= '0;
            diff_lane_2              <= '0;
            diff_lane_3              <= '0;

            // Stage 3b reset
            sub_valid                <= 1'b0;
            sub_base_id              <= '0;
            sub_lane_0               <= '0;
            sub_lane_1               <= '0;
            sub_lane_2               <= '0;
            sub_lane_3               <= '0;

            // Output reset
            o_pixeldata_lane_valid   <= 1'b0;
            o_pixeldata_lane_base_id <= '0;
            o_pixeldata_lane_0       <= '0;
            o_pixeldata_lane_1       <= '0;
            o_pixeldata_lane_2       <= '0;
            o_pixeldata_lane_3       <= '0;

        end else begin

            // ----------------------------------------------------------
            // Stage 1: address generation, one row per cycle.
            // ----------------------------------------------------------
            if (i_start) begin
                reading   <= 1'b1;
                row_cnt   <= '0;
                o_rd_en   <= 1'b1;
                o_rd_addr <= '0;          // request row 0
            end else if (reading) begin
                o_rd_en <= 1'b1;
                if (row_cnt == ROW_COUNT_WIDTH'(MAX_ROW)) begin
                    reading <= 1'b0;
                    row_cnt <= '0;
                    o_rd_en <= 1'b0;
                end else begin
                    row_cnt   <= row_cnt + 1'b1;
                    o_rd_addr <= row_cnt + 1'b1;  // pipelined address
                end
            end else begin
                o_rd_en <= 1'b0;
            end

            // ----------------------------------------------------------
            // Stage 2: 1-cycle delay so valid_pipe lines up with the
            // BRAM's registered output. When valid_pipe is high, both
            // i_rd_data_X and i_noise_data_X carry the row addressed
            // one cycle earlier.
            // ----------------------------------------------------------
            valid_pipe <= reading;
            id_pipe    <= QUBIT_ID_WIDTH'(row_cnt << $clog2(NUM_BANKS));

            // ----------------------------------------------------------
            // Stage 2b: register the BRAM outputs in fabric flops.
            // Pipeline cut added so the BRAM clock-to-out path ends at
            // a flop instead of the subtract logic (STA timing closure).
            // ----------------------------------------------------------
            rd_valid_q <= valid_pipe;
            id_q       <= id_pipe;
            if (valid_pipe) begin
                rd_data_q_0    <= i_rd_data_0;
                rd_data_q_1    <= i_rd_data_1;
                rd_data_q_2    <= i_rd_data_2;
                rd_data_q_3    <= i_rd_data_3;
                noise_data_q_0 <= i_noise_data_0;
                noise_data_q_1 <= i_noise_data_1;
                noise_data_q_2 <= i_noise_data_2;
                noise_data_q_3 <= i_noise_data_3;
            end

            // ----------------------------------------------------------
            // Breaking Stage 3 into Stage 3a and Stage 3b for STA
            // Stage 3a: registered raw byte-wise subtract.
            // The BRAM-clk-to-out path now terminates in an ordinary
            // D input here, well within the 520 MHz.
            // ----------------------------------------------------------
            diff_valid   <= rd_valid_q;
            diff_base_id <= id_q;
            if (rd_valid_q) begin
                diff_lane_0 <= sub_roi_raw(rd_data_q_0, noise_data_q_0);
                diff_lane_1 <= sub_roi_raw(rd_data_q_1, noise_data_q_1);
                diff_lane_2 <= sub_roi_raw(rd_data_q_2, noise_data_q_2);
                diff_lane_3 <= sub_roi_raw(rd_data_q_3, noise_data_q_3);
            end

            // ----------------------------------------------------------
            // Stage 3b: registered borrow-based clamp. Single mux
            // level per byte; the path from diff_lane_X to sub_lane_X
            // is short, so Vivadoo Tool is free to pack sub_lane_X with
            // standard CE-driven D-inputs 
            // ----------------------------------------------------------
            sub_valid   <= diff_valid;
            sub_base_id <= diff_base_id;
            if (diff_valid) begin
                sub_lane_0 <= clamp_roi(diff_lane_0);
                sub_lane_1 <= clamp_roi(diff_lane_1);
                sub_lane_2 <= clamp_roi(diff_lane_2);
                sub_lane_3 <= clamp_roi(diff_lane_3);
            end

            // ----------------------------------------------------------
            // Stage 4: output register.
            // ----------------------------------------------------------
            o_pixeldata_lane_valid   <= sub_valid;
            o_pixeldata_lane_base_id <= sub_base_id;
            if (sub_valid) begin
                o_pixeldata_lane_0 <= sub_lane_0;
                o_pixeldata_lane_1 <= sub_lane_1;
                o_pixeldata_lane_2 <= sub_lane_2;
                o_pixeldata_lane_3 <= sub_lane_3;
            end else begin
                o_pixeldata_lane_valid <= 1'b0;
            end

        end
    end

endmodule
