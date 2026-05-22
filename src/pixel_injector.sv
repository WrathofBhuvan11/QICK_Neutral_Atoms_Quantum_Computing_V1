// ----------------------------------------------------------------------
// pixel_injector.sv -- 8-pixel-per-beat AXI4-Stream pixel front-end
// ----------------------------------------------------------------------
// Input: AXI4-Stream from VDMA, 64-bit beat = 8 packed pixels.
//   tdata[63:0] = {p7, p6, p5, p4, p3, p2, p1, p0}
//                 (p0 in [7:0], p7 in [63:56])
//   tvalid / tready / tlast / tuser[0] = SOF
//
// Outputs:
//   o_pixel_data  [63:0]  registered, 8 pixels per beat
//   o_pixel_valid         registered
//   o_pixel_x     [8:0]   registered, column of p0 (0, 8, 16, ... , 504)
//   o_pixel_y     [8:0]   registered, row (0..511)
//   o_frame_done          registered, 1-cycle pulse after the last beat
//   o_sync_lval           combinational, asserted 1 cycle ahead of o_pixel_valid
//   o_sync_fval           combinational, same as o_sync_lval here
//
// lval and fval lead the registered datapath by one cycle so that
// downstream blocks (coord_matcher, roi_extractor) can gate on them
// without losing the first beat of a frame.
// ----------------------------------------------------------------------

`timescale 1ns / 1ps
import params_pkg::*;

module pixel_injector (
    input  logic        i_aclk,
    input  logic        i_aresetn,

    input  logic [BEAT_BITS-1:0] s_axis_tdata,   // 64-bit beat = 8 px
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    input  logic        s_axis_tlast,    // end-of-line
    input  logic [0:0]  s_axis_tuser,    // tuser[0] = SOF

    output logic [BEAT_BITS-1:0]   o_pixel_data,   // registered, 8 px/beat
    output logic                   o_pixel_valid,  // registered
    output logic [COORD_WIDTH-1:0] o_pixel_x,      // registered, column of p0
    output logic [COORD_WIDTH-1:0] o_pixel_y,      // registered
    output logic                   o_frame_done,   // registered, 1-cycle pulse
    output logic                   o_sync_lval,    // combinational
    output logic                   o_sync_fval     // combinational
);

    localparam int LINES_PER_FRAME = IMAGE_HEIGHT;  // 512

    logic [COORD_WIDTH-1:0] x_cnt;
    logic [COORD_WIDTH-1:0] y_cnt;
    logic                   frame_active;

    // Always ready; back-pressure is handled upstream by the CDC FIFO.
    assign s_axis_tready = 1'b1;
    logic beat_accepted;
    assign beat_accepted = s_axis_tvalid;  // tready == 1 by construction

    // Look-ahead form of frame_active so lval/fval lead the registered
    // pixel valid by one cycle.
    logic next_frame_active;
    assign next_frame_active = frame_active | (beat_accepted & s_axis_tuser[0]);

    assign o_sync_lval = next_frame_active;
    assign o_sync_fval = next_frame_active;

    // Frame / line FSM
    always_ff @(posedge i_aclk) begin
        if (!i_aresetn) begin
            x_cnt        <= '0;
            y_cnt        <= '0;
            frame_active <= 1'b0;
            o_frame_done <= 1'b0;
        end else begin
            o_frame_done <= 1'b0;  // default deassert; pulsed below

            if (beat_accepted) begin

                if (s_axis_tuser[0]) begin
                    // SOF: resync. Pre-increment x_cnt to PIXELS_PER_BEAT (8)
                    // so that beat-1 reports x = 8. Beat-0 reports x = 0 via
                    // the tuser mux on the output register below.
                    frame_active <= 1'b1;
                    x_cnt        <= COORD_WIDTH'(PIXELS_PER_BEAT);
                    y_cnt        <= '0;

                end else if (frame_active) begin
                    x_cnt <= x_cnt + COORD_WIDTH'(PIXELS_PER_BEAT);

                    if (s_axis_tlast) begin
                        x_cnt <= '0;
                        if (y_cnt == COORD_WIDTH'(LINES_PER_FRAME - 1)) begin
                            y_cnt        <= '0;
                            frame_active <= 1'b0;
                            o_frame_done <= 1'b1;
                        end else begin
                            y_cnt <= y_cnt + COORD_WIDTH'(1);
                        end
                    end
                end

            end
        end
    end

    // Output register stage
    always_ff @(posedge i_aclk) begin
        if (!i_aresetn) begin
            o_pixel_data  <= '0;
            o_pixel_valid <= 1'b0;
            o_pixel_x     <= '0;
            o_pixel_y     <= '0;
        end else begin
            o_pixel_data  <= s_axis_tdata;
            o_pixel_valid <= beat_accepted & next_frame_active;
            // SOF beat: x_cnt was pre-incremented to PIXELS_PER_BEAT (8)
            // above, so override to 0 here.
            o_pixel_x     <= s_axis_tuser[0] ? '0 : x_cnt;
            o_pixel_y     <= y_cnt;
        end
    end

endmodule
