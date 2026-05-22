// ----------------------------------------------------------------------
// axis_async_fifo.sv -- AXI4-Stream asynchronous clock-domain FIFO
// ----------------------------------------------------------------------
// Gray-code pointer based async FIFO for crossing AXI4-Stream signals
// between two unrelated clock domains.
//
// Features:
//   - Parameterised DATA_W (carries tdata plus any sideband such as
//     tlast and tuser packed in by the caller).
//   - Parameterised DEPTH, must be a power of two, minimum 4.
//   - Gray-code write/read pointers with 2-FF synchronisers in each
//     direction.
//   - Full/empty generated locally in each clock domain.
//   - AXI4-Stream tvalid/tready handshake on both sides.
//   - ASYNC_REG + KEEP on the synchroniser FFs so Vivado places them
//     together and does not optimise them away.
//   - Memory array carries no reset (saves resources, contents are
//     only read when pointers say data is valid).
//
// Latency: 2-3 cycles write-to-read, typical for a gray-code FIFO.
//
// Interface:
//   Write side (s_axis_*): producer clock domain, e.g. 300 MHz.
//   Read  side (m_axis_*): consumer clock domain, e.g. 520 MHz.
// ----------------------------------------------------------------------

`timescale 1ns / 1ps

module axis_async_fifo #(
    parameter int DATA_W = 18,   // tdata + any packed sideband
    parameter int DEPTH  = 16    // must be a power of two
)(
    // ---- Write (producer) clock domain ----
    input  logic             wr_clk,
    input  logic             wr_rst_n,    // active-low, synchronous to wr_clk
    input  logic [DATA_W-1:0] s_axis_tdata,
    input  logic             s_axis_tvalid,
    output logic             s_axis_tready,

    // ---- Read (consumer) clock domain ----
    input  logic             rd_clk,
    input  logic             rd_rst_n,    // active-low, synchronous to rd_clk
    output logic [DATA_W-1:0] m_axis_tdata,
    output logic             m_axis_tvalid,
    input  logic             m_axis_tready
);

    // ------------------------------------------------------------
    // Pointer width: one extra bit beyond ADDR_W lets full and empty
    // be distinguished even when the addresses themselves are equal.
    // ------------------------------------------------------------
    localparam int ADDR_W = $clog2(DEPTH);
    localparam int PTR_W  = ADDR_W + 1;

    // ------------------------------------------------------------
    // Memory
    // ------------------------------------------------------------
    (* ram_style = "distributed" *) logic [DATA_W-1:0] mem [0:DEPTH-1];

    // ------------------------------------------------------------
    // Write-domain signals
    // ------------------------------------------------------------
    logic [PTR_W-1:0] wr_ptr_bin  = '0;   // binary write pointer
    logic [PTR_W-1:0] wr_ptr_gray = '0;   // gray-code write pointer
    logic [PTR_W-1:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;  // synced read ptr
    logic             full;

    // ------------------------------------------------------------
    // Read-domain signals
    // ------------------------------------------------------------
    logic [PTR_W-1:0] rd_ptr_bin  = '0;   // binary read pointer
    logic [PTR_W-1:0] rd_ptr_gray = '0;   // gray-code read pointer
    logic [PTR_W-1:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;  // synced write ptr
    logic             empty;

    // ------------------------------------------------------------
    // Gray <-> binary conversion helpers
    // ------------------------------------------------------------
    function automatic logic [PTR_W-1:0] bin2gray(input logic [PTR_W-1:0] b);
        return b ^ (b >> 1);
    endfunction

    function automatic logic [PTR_W-1:0] gray2bin(input logic [PTR_W-1:0] g);
        logic [PTR_W-1:0] b;
        b[PTR_W-1] = g[PTR_W-1];
        for (int i = PTR_W-2; i >= 0; i--)
            b[i] = b[i+1] ^ g[i];
        return b;
    endfunction

    // ------------------------------------------------------------
    // WRITE DOMAIN (wr_clk)
    // ------------------------------------------------------------

    // 2-FF synchroniser: read pointer (gray code) into the write domain.
    // KEEP stops Vivado from merging these FFs into adjacent logic and
    // breaking the side-by-side placement that ASYNC_REG requests.
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *) logic [PTR_W-1:0] rd_gray_wr_s1, rd_gray_wr_s2;
    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_wr_s1 <= '0;
            rd_gray_wr_s2 <= '0;
        end else begin
            rd_gray_wr_s1 <= rd_ptr_gray;
            rd_gray_wr_s2 <= rd_gray_wr_s1;
        end
    end

    // Gray-code full condition:
    //   top two bits of wr_gray are the inverse of the synced rd_gray
    //   and the remaining bits match.
    assign full = (wr_ptr_gray == {~rd_gray_wr_s2[PTR_W-1:PTR_W-2],
                                     rd_gray_wr_s2[PTR_W-3:0]});

    assign s_axis_tready = ~full;

    // Write enable
    wire wr_en = s_axis_tvalid & s_axis_tready;

    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_bin  <= '0;
            wr_ptr_gray <= '0;
        end else if (wr_en) begin
            mem[wr_ptr_bin[ADDR_W-1:0]] <= s_axis_tdata;
            wr_ptr_bin  <= wr_ptr_bin + 1'b1;
            wr_ptr_gray <= bin2gray(wr_ptr_bin + 1'b1);
        end
    end

    // ------------------------------------------------------------
    // READ DOMAIN (rd_clk)
    // ------------------------------------------------------------

    // 2-FF synchroniser: write pointer (gray code) into the read domain.
    // Same KEEP rationale as above.
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *) logic [PTR_W-1:0] wr_gray_rd_s1, wr_gray_rd_s2;
    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_rd_s1 <= '0;
            wr_gray_rd_s2 <= '0;
        end else begin
            wr_gray_rd_s1 <= wr_ptr_gray;
            wr_gray_rd_s2 <= wr_gray_rd_s1;
        end
    end

    // Empty when the two gray pointers are bitwise equal.
    assign empty = (rd_ptr_gray == wr_gray_rd_s2);

    // Read path: registered output for timing margin downstream.
    logic [DATA_W-1:0] rd_data_reg;
    logic              rd_data_valid;

    wire rd_en = ~empty & (m_axis_tready | ~rd_data_valid);

    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin    <= '0;
            rd_ptr_gray   <= '0;
            rd_data_reg   <= '0;
            rd_data_valid <= 1'b0;
        end else begin
            if (rd_en) begin
                rd_data_reg  <= mem[rd_ptr_bin[ADDR_W-1:0]];
                rd_data_valid <= 1'b1;
                rd_ptr_bin   <= rd_ptr_bin + 1'b1;
                rd_ptr_gray  <= bin2gray(rd_ptr_bin + 1'b1);
            end else if (m_axis_tready && rd_data_valid) begin
                rd_data_valid <= 1'b0;
            end
        end
    end

    assign m_axis_tdata  = rd_data_reg;
    assign m_axis_tvalid = rd_data_valid;

endmodule
