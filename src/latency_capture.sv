// ----------------------------------------------------------------------
// latency_capture.sv -- per-frame PL-pipeline latency instrumentation
// ----------------------------------------------------------------------
// Free-running 32-bit cycle counter clocked at i_clk_pl (520 MHz).
//   Resolution: 1.923 ns per tick.
//   Wrap:       ~8.25 s, which is far longer than any single frame.
//
// Snapshot registers (one per observation tap):
//   ts_sof_in        first valid pixel out of the input CDC FIFO (SOF)
//   ts_first_match   coord_matcher fires its first match_found pulse
//   ts_first_roi_wr  roi_extractor fires its first write_enable
//   ts_frame_ready   roi_storage asserts frame_ready (ping-pong swap)
//   ts_first_result  first valid result from the Gaussian bank
//   ts_last_result   last result beat (TLAST qualifier)
//   ts_frame_done    pixel_injector frame_done pulse
//
// A frame sequence number (frame_seq) increments on every frame_done.
// Software can read it before and after fetching the timestamps to
// detect frame-boundary tearing.
//
// AXI4-Lite read-only register map (300 MHz domain):
//   0x00  cycle_cnt        free-running counter
//   0x04  frame_seq        frames completed since reset
//   0x08  ts_sof_in
//   0x0C  ts_first_match
//   0x10  ts_first_roi_wr
//   0x14  ts_frame_ready
//   0x18  ts_first_result
//   0x1C  ts_last_result
//   0x20  ts_frame_done
//   0x24  delta_pipe       total PL pipeline (last_result - sof_in)
//   0x28  delta_match      sof_in        -> first_match
//   0x2C  delta_roi        first_match   -> first_roi_wr
//   0x30  delta_storage    frame_done    -> frame_ready
//   0x34  delta_readout    frame_ready   -> last_result
//   0x38  delta_filter     first_result  -> last_result
// ----------------------------------------------------------------------

`timescale 1ns / 1ps
import params_pkg::*;

module latency_capture (
    // PL processing clock (520 MHz)
    input  logic        i_clk_pl,
    input  logic        i_rst_n_pl,

    // ---- Observation taps (all in the 520 MHz domain) ----
    input  logic        i_sof_valid,       // first valid pixel out of input FIFO (tuser & tvalid)
    input  logic        i_match_found,     // coord_matcher o_match_found
    input  logic        i_roi_wr_en,       // roi_extractor o_write_enable
    input  logic        i_frame_ready,     // roi_storage o_frame_ready
    input  logic        i_qubit_valid,     // gaussian bank o_valid
    input  logic        i_qubit_tlast,     // last beat qualifier (base_id == 96)
    input  logic        i_frame_done,      // pixel_injector o_frame_done

    // ---- AXI4-Lite read-only slave (300 MHz) ----
    input  logic        s_axi_aclk,
    input  logic        s_axi_aresetn,
    input  logic [5:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    // Write channel: tied off. Reads-only peripheral, but a complete
    // AXI4-Lite slave is provided for bus-fabric compatibility.
    input  logic [5:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready
);

    // --------------------------------------------------------------
    // 1. Free-running cycle counter, 520 MHz domain.
    // --------------------------------------------------------------
    logic [31:0] cycle_cnt;

    always_ff @(posedge i_clk_pl) begin
        if (!i_rst_n_pl)
            cycle_cnt <= '0;
        else
            cycle_cnt <= cycle_cnt + 1;
    end

    // --------------------------------------------------------------
    // 2. Edge detection on every tap, 520 MHz domain.
    // --------------------------------------------------------------
    logic r_sof, r_match, r_roi_wr, r_frame_ready, r_qubit_valid, r_qubit_last, r_frame_done;

    always_ff @(posedge i_clk_pl) begin
        if (!i_rst_n_pl) begin
            r_sof         <= 1'b0;
            r_match       <= 1'b0;
            r_roi_wr      <= 1'b0;
            r_frame_ready <= 1'b0;
            r_qubit_valid <= 1'b0;
            r_qubit_last  <= 1'b0;
            r_frame_done  <= 1'b0;
        end else begin
            r_sof         <= i_sof_valid;
            r_match       <= i_match_found;
            r_roi_wr      <= i_roi_wr_en;
            r_frame_ready <= i_frame_ready;
            r_qubit_valid <= i_qubit_valid;
            r_qubit_last  <= i_qubit_tlast;
            r_frame_done  <= i_frame_done;
        end
    end

    wire sof_rise       = i_sof_valid    & ~r_sof;
    wire match_rise     = i_match_found  & ~r_match;
    wire roi_wr_rise    = i_roi_wr_en    & ~r_roi_wr;
    wire ready_rise     = i_frame_ready  & ~r_frame_ready;
    wire result_rise    = i_qubit_valid  & ~r_qubit_valid;
    wire last_rise      = (i_qubit_valid & i_qubit_tlast) & ~(r_qubit_valid & r_qubit_last);
    wire frame_done_rise = i_frame_done  & ~r_frame_done;

    // --------------------------------------------------------------
    // 3. Snapshot registers and frame sequencer.
    //    Each event has a per-frame "armed" flag, set on SOF and
    //    cleared on the first matching event so subsequent events
    //    in the same frame do not overwrite the first-event time.
    //    ts_last_result is the exception -- it always captures so
    //    the latest end-of-frame timestamp wins.
    // --------------------------------------------------------------
    logic [31:0] ts_sof_in;
    logic [31:0] ts_first_match;
    logic [31:0] ts_first_roi_wr;
    logic [31:0] ts_frame_ready;
    logic [31:0] ts_first_result;
    logic [31:0] ts_last_result;
    logic [31:0] ts_frame_done;
    logic [31:0] frame_seq;

    logic armed_match, armed_roi, armed_ready, armed_result;

    always_ff @(posedge i_clk_pl) begin
        if (!i_rst_n_pl) begin
            ts_sof_in        <= '0;
            ts_first_match   <= '0;
            ts_first_roi_wr  <= '0;
            ts_frame_ready   <= '0;
            ts_first_result  <= '0;
            ts_last_result   <= '0;
            ts_frame_done    <= '0;
            frame_seq        <= '0;
            armed_match      <= 1'b0;
            armed_roi        <= 1'b0;
            armed_ready      <= 1'b0;
            armed_result     <= 1'b0;
        end else begin

            // SOF: stamp the entry time and re-arm all event flags
            // for the new frame.
            if (sof_rise) begin
                ts_sof_in    <= cycle_cnt;
                armed_match  <= 1'b1;
                armed_roi    <= 1'b1;
                armed_ready  <= 1'b1;
                armed_result <= 1'b1;
            end

            // First coord_matcher hit in this frame.
            if (match_rise && armed_match) begin
                ts_first_match <= cycle_cnt;
                armed_match    <= 1'b0;
            end

            // First ROI write in this frame.
            if (roi_wr_rise && armed_roi) begin
                ts_first_roi_wr <= cycle_cnt;
                armed_roi       <= 1'b0;
            end

            // Ping-pong swap completes.
            if (ready_rise && armed_ready) begin
                ts_frame_ready <= cycle_cnt;
                armed_ready    <= 1'b0;
            end

            // First Gaussian result valid.
            if (result_rise && armed_result) begin
                ts_first_result <= cycle_cnt;
                armed_result    <= 1'b0;
            end

            // Last result beat (TLAST). Always captures so the
            // stored value is the latest end-of-frame timestamp.
            if (last_rise) begin
                ts_last_result <= cycle_cnt;
            end

            // Frame done: stamp and increment the sequence counter.
            if (frame_done_rise) begin
                ts_frame_done <= cycle_cnt;
                frame_seq     <= frame_seq + 1;
            end
        end
    end

    // Pre-computed deltas (combinational, stable between frames).
    logic [31:0] delta_pipe, delta_match, delta_roi, delta_storage, delta_readout, delta_filter;

    assign delta_pipe    = ts_last_result  - ts_sof_in;        // total PL pipe
    assign delta_match   = ts_first_match  - ts_sof_in;        // SOF -> first match
    assign delta_roi     = ts_first_roi_wr - ts_first_match;   // match -> first ROI write
    assign delta_storage = ts_frame_ready  - ts_frame_done;    // frame_done -> ping-pong ready
    assign delta_readout = ts_last_result  - ts_frame_ready;   // frame_ready -> last result
    assign delta_filter  = ts_last_result  - ts_first_result;  // first -> last result

    // --------------------------------------------------------------
    // 4. AXI4-Lite read-only slave (300 MHz).
    // The timestamps are quasi-static (stable for ~250 us between
    // frames), so direct reads without an explicit CDC handshake
    // are safe. Software that wants belt-and-braces semantics can
    // read frame_seq before and after the snapshot and discard the
    // sample if it changes mid-read.
    // --------------------------------------------------------------
    assign s_axi_arready = ~s_axi_rvalid;
    assign s_axi_rresp   = 2'b00;

    // Write channel stubs: always accept, never respond.
    assign s_axi_awready = 1'b1;
    assign s_axi_wready  = 1'b1;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_bvalid  = 1'b0;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= '0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                case (s_axi_araddr[5:2])
                    4'h0: s_axi_rdata <= cycle_cnt;
                    4'h1: s_axi_rdata <= frame_seq;
                    4'h2: s_axi_rdata <= ts_sof_in;
                    4'h3: s_axi_rdata <= ts_first_match;
                    4'h4: s_axi_rdata <= ts_first_roi_wr;
                    4'h5: s_axi_rdata <= ts_frame_ready;
                    4'h6: s_axi_rdata <= ts_first_result;
                    4'h7: s_axi_rdata <= ts_last_result;
                    4'h8: s_axi_rdata <= ts_frame_done;
                    4'h9: s_axi_rdata <= delta_pipe;
                    4'hA: s_axi_rdata <= delta_match;
                    4'hB: s_axi_rdata <= delta_roi;
                    4'hC: s_axi_rdata <= delta_storage;
                    4'hD: s_axi_rdata <= delta_readout;
                    4'hE: s_axi_rdata <= delta_filter;
                    default: s_axi_rdata <= 32'hDEAD_C0DE;
                endcase
            end
            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;
        end
    end

endmodule
