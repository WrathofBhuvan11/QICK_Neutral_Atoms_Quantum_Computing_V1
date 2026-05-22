// ----------------------------------------------------------------------
// roi_extractor.sv -- 8-pixel-per-beat 3x3 ROI extractor
// ----------------------------------------------------------------------
// Input is the 64-bit pixel stream from pixel_injector:
//   [63:56]=p7 [55:48]=p6 [47:40]=p5 [39:32]=p4
//   [31:24]=p3 [23:16]=p2 [15:8]=p1  [7:0]=p0
//
// 16-column sliding window per row:
//   win[0..7]  = current beat  (beat N,   pixels p0..p7)
//   win[8..15] = previous beat (beat N-1, pixels p0..p7)
//   slot s (0..7)  -> column curr_x + s
//   slot s (8..15) -> column curr_x - 16 + s
//
// ROI capture timing
//   coord_matcher is a 3-stage pipeline: i_match_trigger fires at
//   cycle N+3 for beat N. The pixel-side inputs are delayed 1 cycle on
//   entry (the *_q registers below), so the window absorbs beat N at
//   cycle N+2 and the capture -- driven by the match pulse at N+3 --
//   reads the PRE-update window. So at capture time:
//     win[0..7] holds beat-N pixels and win[8..15] holds beat-(N-1).
//
// For match offset k (0..7), curr_x + k == Qx+1, so the 3-pixel slice
// [Qx-1, Qx, Qx+1] sits at window slots (mod 16):
//     Qx-1 -> (k-2) mod 16     Qx -> (k-1) mod 16     Qx+1 -> k
//   e.g. k=0: [14,15,0]   k=1: [15,0,1]   k=2: [0,1,2] ...  k=7: [5,6,7]
//   computed combinationally as sx_m1 / sx_0 / sx_p1 below.
//
// Two line buffers (lb0 for row Y-1, lb1 for row Y-2) carry the prior
// rows. Both are 64-bit wide, depth IMAGE_WIDTH/8 = 64.
// ----------------------------------------------------------------------

`timescale 1ns / 1ps
import params_pkg::*;

module roi_extractor (
    input  logic        i_clk,
    input  logic        i_rst_n,

    // 64-bit pixel stream (8 pixels per beat)
    input  logic [BEAT_BITS-1:0] i_pixel_data,   // {p7..p0}
    input  logic        i_pixel_valid,

    // Sync from pixel_injector
    input  logic        i_sync_lval,
    input  logic        i_sync_fval,

    // Match trigger from coord_matcher
    input  logic        i_match_trigger,
    input  logic [MATCH_OFFSET_WIDTH-1:0] i_match_offset,  // 0..7
    input  logic [QUBIT_ID_WIDTH-1:0] i_qubit_index,

    // Output to ROI storage
    output logic [ROI_BITS-1:0]       o_roi_flat,
    output logic [QUBIT_ID_WIDTH-1:0] o_qubit_index,
    output logic                      o_write_enable
);

    //-----------------------------------------------------------------
    // Front-end input delay (1 cycle).
    //
    // coord_matcher is a 3-stage pipeline: i_match_trigger
    // arrives one cycle later relative to the pixel stream. To keep the
    // window / capture timing identical to the original design, the
    // pixel-side inputs (data, valid, lval, fval) are delayed by one
    // cycle here, and every block below uses the *_q versions. The
    // match-side inputs (i_match_trigger / i_match_offset /
    // i_qubit_index) are used directly. Net effect: roi_extractor sees
    // a uniformly 1-cycle-later pixel world, re-aligned with the later
    // match trigger -- every internal relative timing is preserved.
    //-----------------------------------------------------------------
    logic [BEAT_BITS-1:0] i_pixel_data_q;
    logic                 i_pixel_valid_q;
    logic                 i_sync_lval_q;
    logic                 i_sync_fval_q;

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            i_pixel_data_q  <= '0;
            i_pixel_valid_q <= 1'b0;
            i_sync_lval_q   <= 1'b0;
            i_sync_fval_q   <= 1'b0;
        end else begin
            i_pixel_data_q  <= i_pixel_data;
            i_pixel_valid_q <= i_pixel_valid;
            i_sync_lval_q   <= i_sync_lval;
            i_sync_fval_q   <= i_sync_fval;
        end
    end

    //-----------------------------------------------------------------
    // Line buffers, 64-bit per entry (one beat), depth IMAGE_WIDTH/8
    //-----------------------------------------------------------------
    localparam LB_DEPTH = (IMAGE_WIDTH + PIXELS_PER_BEAT - 1) / PIXELS_PER_BEAT;  // 64

    (* ram_style = "block" *) logic [BEAT_BITS-1:0] lb0 [0:LB_DEPTH-1];  // row Y-1
    (* ram_style = "block" *) logic [BEAT_BITS-1:0] lb1 [0:LB_DEPTH-1];  // row Y-2

    //-----------------------------------------------------------------
    // Line-buffer pointers (one count per beat)
    //-----------------------------------------------------------------
    logic [COORD_WIDTH-1:0] wr_ptr;
    logic [COORD_WIDTH-1:0] rd_ptr;

    // Edge detection on lval/fval to reset the line-buffer pointers
    // at start of each new line and end of each frame.
    logic sync_lval_r, sync_fval_r;

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            sync_lval_r <= 1'b0;
            sync_fval_r <= 1'b0;
        end else begin
            sync_lval_r <= i_sync_lval_q;
            sync_fval_r <= i_sync_fval_q;
        end
    end

    wire lval_rising  = !sync_lval_r && i_sync_lval_q;
    wire fval_falling = sync_fval_r && !i_sync_fval_q;

    //-----------------------------------------------------------------
    // 16-column sliding window across 3 rows
    //   win_r0 = current row Y
    //   win_r1 = row Y-1 (from lb0)
    //   win_r2 = row Y-2 (from lb1)
    //   slots [0..7] = current beat, [8..15] = previous beat
    //-----------------------------------------------------------------
    localparam int WIN = 2 * PIXELS_PER_BEAT;   // 16

    logic [7:0] win_r0 [0:WIN-1];
    logic [7:0] win_r1 [0:WIN-1];
    logic [7:0] win_r2 [0:WIN-1];

    logic [BEAT_BITS-1:0] r_lb0, r_lb1;

    //-----------------------------------------------------------------
    // ROI column slot indices, combinational from i_match_offset.
    //   Qx+1 -> slot k           (k = i_match_offset, 0..7)
    //   Qx   -> slot (k-1) mod 16
    //   Qx-1 -> slot (k-2) mod 16
    // 4-bit arithmetic wraps mod 16 (= WIN) for free.
    //-----------------------------------------------------------------
    logic [3:0] sx_m1, sx_0, sx_p1;
    always_comb begin
        sx_p1 = {1'b0, i_match_offset};            // k
        sx_0  = {1'b0, i_match_offset} + 4'd15;    // (k - 1) mod 16
        sx_m1 = {1'b0, i_match_offset} + 4'd14;    // (k - 2) mod 16
    end

    //-----------------------------------------------------------------
    // Stage1.Second pixel-pipeline stage. Combined with the front-end delay
    // above, the pixel data is 2 cycles behind its input port, so the
    // window update at cycle N+2 reflects beat N while the capture
    // (driven by the match pulse at cycle N+3) sees the correct
    // pre-update window. match_d1 / offset_d1 / index_d1 register the
    // match-side inputs directly (kept for completeness; the capture
    // path uses the direct i_match_* signals).
    //-----------------------------------------------------------------
    logic [BEAT_BITS-1:0] pixel_d1;
    logic        valid_d1;
    logic        match_d1;
    logic [MATCH_OFFSET_WIDTH-1:0] offset_d1;
    logic [QUBIT_ID_WIDTH-1:0] index_d1;

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            pixel_d1  <= '0;
            valid_d1  <= 1'b0;
            match_d1  <= 1'b0;
            offset_d1 <= '0;
            index_d1  <= '0;
        end else begin
            pixel_d1  <= i_pixel_data_q;
            valid_d1  <= i_pixel_valid_q;
            match_d1  <= i_match_trigger;
            offset_d1 <= i_match_offset;
            index_d1  <= i_qubit_index;
        end
    end

    //-----------------------------------------------------------------
    // Line-buffer read path. Pointer resets at start of line / end of
    // frame; otherwise increments once per accepted beat.
    //-----------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            rd_ptr <= '0;
        end else begin
            if (fval_falling || lval_rising) begin
                rd_ptr <= '0;
            end else if (i_pixel_valid_q && i_sync_lval_q) begin
                r_lb0 <= lb0[rd_ptr];
                r_lb1 <= lb1[rd_ptr];
                rd_ptr <= (rd_ptr == COORD_WIDTH'(LB_DEPTH-1)) ? '0 : rd_ptr + 1;
            end
        end
    end

    //-----------------------------------------------------------------
    // Window update, line-buffer write-back, and ROI capture.
    //
    // Timing :
    //   The window update is driven from valid_d1 (the pixel data 2
    //   cycles behind its input port: front-end delay + this stage),
    //   so at cycle N+2 it incorporates beat N. coord_matcher's
    //   3-stage output (i_match_trigger) arrives at cycle N+3. The
    //   capture below uses i_match_trigger directly, so it reads the
    //   window registers as set at cycle N+2, i.e. the pre-update
    //   state where:
    //     win[0..7]  = beat N   pixels (p0..p7)
    //     win[8..15] = beat N-1 pixels (p0..p7)
    //-----------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            wr_ptr         <= '0;
            o_write_enable <= 1'b0;
            o_qubit_index  <= '0;
            o_roi_flat     <= '0;
            for (int k = 0; k < WIN; k++) begin
                win_r0[k] <= '0;
                win_r1[k] <= '0;
                win_r2[k] <= '0;
            end
        end else begin

            //----------------------------------------------------------
            // Pointer reset and per-beat window / line-buffer update
            //----------------------------------------------------------
            if (fval_falling || lval_rising) begin
                wr_ptr <= '0;
            end else if (valid_d1 && i_sync_lval_q) begin

                // Cascade write: lb0 stores the row just consumed,
                // lb1 stores what lb0 held one row earlier.
                lb0[wr_ptr] <= pixel_d1;
                lb1[wr_ptr] <= r_lb0;

                // Slide window: shift the current-beat half [0..7] into
                // the previous-beat half [8..15], then load the new beat
                // into [0..7]. Row 0 = Y, Row 1 = Y-1, Row 2 = Y-2.
                // The shift reads the OLD win values (non-blocking), so
                // the read-before-write semantics are preserved.
                for (int s = 0; s < PIXELS_PER_BEAT; s++) begin
                    win_r0[PIXELS_PER_BEAT + s] <= win_r0[s];
                    win_r1[PIXELS_PER_BEAT + s] <= win_r1[s];
                    win_r2[PIXELS_PER_BEAT + s] <= win_r2[s];
                    win_r0[s] <= pixel_d1[s*8 +: 8];   // current row Y
                    win_r1[s] <= r_lb0   [s*8 +: 8];   // row Y-1 (from lb0)
                    win_r2[s] <= r_lb1   [s*8 +: 8];   // row Y-2 (from lb1)
                end

                wr_ptr <= (wr_ptr == COORD_WIDTH'(LB_DEPTH-1)) ? '0 : wr_ptr + 1;
            end

            //----------------------------------------------------------
            // ROI capture: reads the PRE-update window state. The three
            // ROI columns [Qx-1, Qx, Qx+1] sit at window slots
            // sx_m1 / sx_0 / sx_p1 (derived combinationally from
            // i_match_offset above). Packed MSB-first as
            //   {row Y, row Y-1, row Y-2}, each [Qx-1, Qx, Qx+1].
            //----------------------------------------------------------
            o_write_enable <= i_match_trigger && i_sync_fval_q && i_sync_lval_q;
            o_qubit_index  <= i_qubit_index;

            if (i_match_trigger && i_sync_fval_q && i_sync_lval_q) begin
                o_roi_flat <= {win_r0[sx_m1], win_r0[sx_0], win_r0[sx_p1],
                               win_r1[sx_m1], win_r1[sx_0], win_r1[sx_p1],
                               win_r2[sx_m1], win_r2[sx_0], win_r2[sx_p1]};
            end

        end
    end

endmodule
