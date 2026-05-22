// ----------------------------------------------------------------------
// roi_storage.sv -- ping-pong ROI storage with baseline noise memory
// ----------------------------------------------------------------------
// Dark mode (i_dark_mode = 1):
//   Writes are diverted to the base_noise_X banks instead of the
//   ping-pong banks. o_frame_ready is suppressed so read_streamer and
//   the Gaussian engines stay idle. The noise baseline lives in a
//   single (non-ping-pong) BRAM set that persists across frames.
//
// Normal mode (i_dark_mode = 0):
//   Writes target the ping-pong banks as usual. o_frame_ready fires on
//   each i_frame_done toggle, releasing read_streamer to drain the
//   completed bank. The four noise output ports are always driven from
//   the base_noise banks so read_streamer can perform its per-pixel
//   saturating subtraction unconditionally.
//
// Reset / power-on default:
//   Until a dark frame has been captured, the base_noise_X arrays read
//   back as zero (Vivado BRAM init), so subtraction is a no-op and the
//   pipeline behaves exactly as the noise-free design.
// ----------------------------------------------------------------------
`timescale 1ns / 1ps
import params_pkg::*;

module roi_storage (
    input  logic             i_wr_clk,
    input  logic             i_rd_clk,
    input  logic             i_rst_n,

    // Write port
    input  logic             i_wr_en,
    input  logic [QUBIT_ID_WIDTH-1:0] i_wr_addr,
    input  logic [ROI_BITS-1:0]       i_wr_data,
    input  logic             i_frame_done,

    // Dark-mode control. Synchronised to both clock domains upstream
    // in datastream_processor.
    input  logic             i_dark_mode,  // 1 = capture to noise baseline, suppress frame_ready

    // Read port
    input  logic             i_rd_en,
    input  logic [BANK_ADDR_WIDTH-1:0] i_rd_addr,

    // Four ping-pong read lanes (normal ROI data)
    output logic [ROI_BITS-1:0]      o_rd_data_0,
    output logic [ROI_BITS-1:0]      o_rd_data_1,
    output logic [ROI_BITS-1:0]      o_rd_data_2,
    output logic [ROI_BITS-1:0]      o_rd_data_3,

    // Four baseline-noise read lanes, always driven
    output logic [ROI_BITS-1:0]      o_noise_data_0,
    output logic [ROI_BITS-1:0]      o_noise_data_1,
    output logic [ROI_BITS-1:0]      o_noise_data_2,
    output logic [ROI_BITS-1:0]      o_noise_data_3,

    output logic             o_frame_ready   // suppressed during dark_mode
);

    // ------------------------------------------------------------------
    // 1. Ping-pong storage: 4 lanes x 2 banks
    // ------------------------------------------------------------------
    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank0_0 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank0_1 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank0_2 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank0_3 [0:BANK_DEPTH-1];

    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank1_0 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank1_1 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank1_2 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] bank1_3 [0:BANK_DEPTH-1];

    // ------------------------------------------------------------------
    // 2. Baseline noise memory: 4 lanes, single bank.
    //    No ping-pong needed because the baseline is overwritten in
    //    full on each dark-mode capture.
    // ------------------------------------------------------------------
    (* ram_style = "block" *) logic [ROI_BITS-1:0] base_noise_0 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] base_noise_1 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] base_noise_2 [0:BANK_DEPTH-1];
    (* ram_style = "block" *) logic [ROI_BITS-1:0] base_noise_3 [0:BANK_DEPTH-1];

    // ------------------------------------------------------------------
    // 3. Ping-pong control. frame_done_toggle is a toggle flag in the
    //    write clock domain; the read side detects edges via a 3-FF
    //    synchroniser.
    // ------------------------------------------------------------------
    logic wr_bank_sel;
    logic rd_bank_sel;
    logic frame_done_toggle;

    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic frame_done_sync1, frame_done_sync2, frame_done_sync3;

    always_ff @(posedge i_wr_clk) begin
        if (!i_rst_n) begin
            frame_done_toggle <= 1'b0;
            wr_bank_sel       <= 1'b0;
        end else if (i_frame_done) begin
            frame_done_toggle <= ~frame_done_toggle;
            wr_bank_sel       <= ~wr_bank_sel;
        end
    end

    // ------------------------------------------------------------------
    // 4. Write logic: demux to either the noise bank set or the active
    //    ping-pong bank depending on i_dark_mode.
    // ------------------------------------------------------------------
    logic [$clog2(NUM_BANKS)-1:0] lane_sel;
    logic [BANK_ADDR_WIDTH-1:0]   row_addr;

    assign lane_sel = i_wr_addr[$clog2(NUM_BANKS)-1:0];
    assign row_addr = i_wr_addr[QUBIT_ID_WIDTH-1:$clog2(NUM_BANKS)];

    always_ff @(posedge i_wr_clk) begin
        if (i_wr_en) begin
            if (i_dark_mode) begin
                // ---- Capture into baseline noise memory ----
                case (lane_sel)
                    2'd0: base_noise_0[row_addr] <= i_wr_data;
                    2'd1: base_noise_1[row_addr] <= i_wr_data;
                    2'd2: base_noise_2[row_addr] <= i_wr_data;
                    2'd3: base_noise_3[row_addr] <= i_wr_data;
                endcase
            end else begin
                // ---- Normal ping-pong write ----
                if (wr_bank_sel == 1'b0) begin
                    case (lane_sel)
                        2'd0: bank0_0[row_addr] <= i_wr_data;
                        2'd1: bank0_1[row_addr] <= i_wr_data;
                        2'd2: bank0_2[row_addr] <= i_wr_data;
                        2'd3: bank0_3[row_addr] <= i_wr_data;
                    endcase
                end else begin
                    case (lane_sel)
                        2'd0: bank1_0[row_addr] <= i_wr_data;
                        2'd1: bank1_1[row_addr] <= i_wr_data;
                        2'd2: bank1_2[row_addr] <= i_wr_data;
                        2'd3: bank1_3[row_addr] <= i_wr_data;
                    endcase
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 5. Read logic: ping-pong read plus always-on noise read.
    // ------------------------------------------------------------------
    always_ff @(posedge i_rd_clk) begin
        if (!i_rst_n) begin
            frame_done_sync1 <= 1'b0;
            frame_done_sync2 <= 1'b0;
            frame_done_sync3 <= 1'b0;
            o_frame_ready    <= 1'b0;
            rd_bank_sel      <= 1'b1;
        end else begin
            // Toggle synchroniser chain (wr_clk -> rd_clk)
            frame_done_sync1 <= frame_done_toggle;
            frame_done_sync2 <= frame_done_sync1;
            frame_done_sync3 <= frame_done_sync2;

            // Edge-detect the synchronised toggle and pulse frame_ready,
            // unless dark mode is active.
            if (frame_done_sync2 ^ frame_done_sync3) begin
                o_frame_ready <= ~i_dark_mode;   // suppressed during dark mode
                rd_bank_sel   <= ~frame_done_sync2;
            end else begin
                o_frame_ready <= 1'b0;
            end

            // ---- Ping-pong bank read ----
            if (rd_bank_sel == 1'b0) begin
                o_rd_data_0 <= bank0_0[i_rd_addr];
                o_rd_data_1 <= bank0_1[i_rd_addr];
                o_rd_data_2 <= bank0_2[i_rd_addr];
                o_rd_data_3 <= bank0_3[i_rd_addr];
            end else begin
                o_rd_data_0 <= bank1_0[i_rd_addr];
                o_rd_data_1 <= bank1_1[i_rd_addr];
                o_rd_data_2 <= bank1_2[i_rd_addr];
                o_rd_data_3 <= bank1_3[i_rd_addr];
            end

            // ---- Noise baseline read, always active, same address ----
            // read_streamer applies result = max(0, roi - noise) per
            // byte. Before any dark frame is captured these arrays read
            // back as zero (BRAM init), so subtraction is a no-op.
            o_noise_data_0 <= base_noise_0[i_rd_addr];
            o_noise_data_1 <= base_noise_1[i_rd_addr];
            o_noise_data_2 <= base_noise_2[i_rd_addr];
            o_noise_data_3 <= base_noise_3[i_rd_addr];
        end
    end

endmodule

