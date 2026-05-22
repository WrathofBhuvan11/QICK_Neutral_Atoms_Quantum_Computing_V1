// ----------------------------------------------------------------------
// qubit_lookup_axi.sv -- AXI4-Lite slave: qubit coordinates + control
// ----------------------------------------------------------------------
// Stores the 100 qubit (X, Y) coordinates and two software control
// registers (dark_mode and gaussian_threshold).
//
// Address map (byte addresses, 32-bit registers):
//   0x000 + qubit_idx*8 + 0   X coordinate, 9-bit value in [8:0]
//   0x000 + qubit_idx*8 + 4   Y coordinate, 9-bit value in [8:0]
//   0x320                     reg_ctrl, dark_mode in bit 0
//   0x324                     reg_gauss_thresh [15:0]
//   Coord space size = 100 * 8 = 800 bytes = 0x320
//
// Outputs:
//   o_q_x[0:99]              -> coord_matcher (X coord array)
//   o_q_y[0:99]              -> coord_matcher (Y coord array)
//   o_lut_valid              -> gates the input CDC FIFO until the
//                               first coordinate has been written
//   o_dark_mode              -> roi_storage write-path select
//   o_gaussian_threshold[15:0] -> gaussian_filter_engine i_threshold
//
// Write path is pipelined into two clocked stages:
//   Stage 0:  AXI handshake -> wr_fire_s0 combinational pulse,
//             drives BVALID and sets lut_written on the first coord
//             write.
//   Stage 1:  wr_fire_q / wr_addr_q / wr_data_q / wr_strb_q
//             registered locally near the register file. The CE
//             driver is therefore short-routed (intra-pblock), which
//             keeps the high-fanout write-enable path inside one
//             clock period. Vivado is free to replicate wr_fire_q,
//             so it does not need a KEEP attribute.
//   Stage 2:  Actual register-file write.
//
// AXI4-Lite compliance:
//   BVALID asserts the cycle the write data handshake completes
//   (wr_fire_s0 pulse).
//
// Latency note:
//   AXI write-to-read latency on reg_x / reg_y increases by 1 clk_300
//   cycle (~3.3 ns). This is invisible to the runtime, since these
//   registers are quasi-static configuration.
// ----------------------------------------------------------------------

`timescale 1ns / 1ps
import params_pkg::*;

module qubit_lookup_axi #(
    parameter int BASE_ADDR_OFFSET = 0
)(
    // AXI4-Lite clock / reset (shared with rest of the PS-side AXI)
    input  logic        s_axi_aclk,
    input  logic        s_axi_aresetn,

    // Write address channel
    input  logic [9:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,

    // Write data channel
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,

    // Write response channel
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,

    // Read address channel
    input  logic [9:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,

    // Read data channel
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    // Coordinate outputs to coord_matcher (300 MHz, quasi-static)
    output logic [COORD_WIDTH-1:0] o_q_x [0:NUM_QUBITS-1],
    output logic [COORD_WIDTH-1:0] o_q_y [0:NUM_QUBITS-1],
    output logic        o_lut_valid,    // set on the first qubit coord write

    // Pipeline control outputs (crossed to 520 MHz domain upstream)
    output logic        o_dark_mode,           // 1 = capture noise baseline
    output logic [15:0] o_gaussian_threshold   // runtime decision threshold
);

    // -------------------------------------------------------
    // Default coordinate generator. Mirrors the reset values
    // so software can read back a sensible grid before writing.
    // -------------------------------------------------------
    function automatic int calc_x_coord(int col);
        int x = QUBIT_START_X;
        for (int c = 0; c < col; c++)
            x += (c == 4) ? 52 : 51;
        return x;
    endfunction

    function automatic int calc_y_coord(int row);
        int y = QUBIT_START_Y;
        for (int r = 0; r < row; r++)
            y += (r == 4) ? 52 : 51;
        return y;
    endfunction

    // -------------------------------------------------------
    // Register file: qubit coordinates and control registers
    // -------------------------------------------------------
    logic [31:0] reg_x [0:NUM_QUBITS-1];
    logic [31:0] reg_y [0:NUM_QUBITS-1];

    // reg_ctrl[0]              = dark_mode
    // reg_gauss_thresh[15:0]   = Gaussian decision threshold
    logic [31:0] reg_ctrl;
    logic [31:0] reg_gauss_thresh;

    genvar gi;
    generate
        for (gi = 0; gi < NUM_QUBITS; gi++) begin : gen_out
            assign o_q_x[gi] = COORD_WIDTH'(reg_x[gi]);
            assign o_q_y[gi] = COORD_WIDTH'(reg_y[gi]);
        end
    endgenerate

    assign o_dark_mode          = reg_ctrl[0];
    assign o_gaussian_threshold = reg_gauss_thresh[15:0];

    // -------------------------------------------------------
    // AXI4-Lite write FSM, stage 0 (handshake + BVALID).
    // wr_fire_s0 is the combinational pulse asserted when W and AW
    // have both handshaken. lut_written latches on the first
    // successful coordinate write and clears only on reset.
    // -------------------------------------------------------
    logic        aw_active;
    logic [9:0]  aw_addr_latch;
    logic        lut_written;
    logic        wr_fire_s0;   // combinational: W accepted this cycle

    assign o_lut_valid   = lut_written;
    assign s_axi_awready = !aw_active;
    assign s_axi_wready  = aw_active;
    assign s_axi_bresp   = 2'b00;

    // wr_fire_s0 goes high the cycle the wvalid/wready handshake
    // closes. It drives BVALID and the pipeline-stage registers
    // below; it does NOT directly clock the register file.
    assign wr_fire_s0 = s_axi_wvalid & aw_active;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            aw_active     <= 1'b0;
            aw_addr_latch <= '0;
            s_axi_bvalid  <= 1'b0;
            lut_written   <= 1'b0;
            // reg_x / reg_y reset lives in the dedicated reg-file
            // block below.
        end else begin
            // Latch write address
            if (s_axi_awvalid && s_axi_awready) begin
                aw_active     <= 1'b1;
                aw_addr_latch <= s_axi_awaddr;
            end

            // On the write handshake: clear aw_active, raise BVALID,
            // and mark the LUT as populated when the write targeted a
            // coordinate register.
            if (wr_fire_s0) begin
                aw_active <= 1'b0;
                if (int'(aw_addr_latch[9:3]) < NUM_QUBITS)
                    lut_written <= 1'b1;
                // BVALID takes priority over a simultaneous B-clear.
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------
    // Stage 1: registered write pipeline. Local copy of the write
    // event, placed beside the register file (same pblock) so the
    // CE route is short.
    // -------------------------------------------------------
    logic        wr_fire_q;
    logic [9:0]  wr_addr_q;
    logic [31:0] wr_data_q;
    logic [3:0]  wr_strb_q;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            wr_fire_q <= 1'b0;
            wr_addr_q <= '0;
            wr_data_q <= '0;
            wr_strb_q <= '0;
        end else begin
            // Capture the write event one cycle before the reg-file
            // write fires. wr_fire_s0 is a single-cycle pulse, so
            // wr_fire_q is also single-cycle.
            wr_fire_q <= wr_fire_s0;
            // Hold addr / data / strb steady between writes so the
            // register-file flops do not toggle unnecessarily.
            if (wr_fire_s0) begin
                wr_addr_q <= aw_addr_latch;
                wr_data_q <= s_axi_wdata;
                wr_strb_q <= s_axi_wstrb;
            end
        end
    end

    // -------------------------------------------------------
    // Stage 2: register-file write.
    //   addr[9:3] selects the register index.
    //     idx < NUM_QUBITS   : qubit X / Y coord registers
    //     idx == NUM_QUBITS  : control registers
    //       addr[2] = 0 (0x320) : reg_ctrl  (dark_mode in bit 0)
    //       addr[2] = 1 (0x324) : reg_gauss_thresh [15:0]
    // -------------------------------------------------------
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            for (int i = 0; i < NUM_QUBITS; i++) begin
                reg_x[i] <= 32'(calc_x_coord(i % GRID_COLS));
                reg_y[i] <= 32'(calc_y_coord(i / GRID_COLS));
            end
            reg_ctrl         <= 32'h0;
            reg_gauss_thresh <= 32'(GAUSS_THRESHOLD_DEFAULT);
        end else if (wr_fire_q) begin
            automatic int idx = int'(wr_addr_q[9:3]);
            if (idx < NUM_QUBITS) begin
                // Qubit coordinate registers, byte-granular wstrb.
                if (!wr_addr_q[2]) begin
                    if (wr_strb_q[0]) reg_x[idx][ 7: 0] <= wr_data_q[ 7: 0];
                    if (wr_strb_q[1]) reg_x[idx][15: 8] <= wr_data_q[15: 8];
                    if (wr_strb_q[2]) reg_x[idx][23:16] <= wr_data_q[23:16];
                    if (wr_strb_q[3]) reg_x[idx][31:24] <= wr_data_q[31:24];
                end else begin
                    if (wr_strb_q[0]) reg_y[idx][ 7: 0] <= wr_data_q[ 7: 0];
                    if (wr_strb_q[1]) reg_y[idx][15: 8] <= wr_data_q[15: 8];
                    if (wr_strb_q[2]) reg_y[idx][23:16] <= wr_data_q[23:16];
                    if (wr_strb_q[3]) reg_y[idx][31:24] <= wr_data_q[31:24];
                end
            end else if (idx == NUM_QUBITS) begin
                // Control registers
                if (!wr_addr_q[2]) begin
                    // 0x320: reg_ctrl (dark_mode in bit 0)
                    if (wr_strb_q[0]) reg_ctrl[ 7: 0] <= wr_data_q[ 7: 0];
                    if (wr_strb_q[1]) reg_ctrl[15: 8] <= wr_data_q[15: 8];
                    if (wr_strb_q[2]) reg_ctrl[23:16] <= wr_data_q[23:16];
                    if (wr_strb_q[3]) reg_ctrl[31:24] <= wr_data_q[31:24];
                end else begin
                    // 0x324: Gaussian decision threshold [15:0]
                    if (wr_strb_q[0]) reg_gauss_thresh[ 7: 0] <= wr_data_q[ 7: 0];
                    if (wr_strb_q[1]) reg_gauss_thresh[15: 8] <= wr_data_q[15: 8];
                    if (wr_strb_q[2]) reg_gauss_thresh[23:16] <= wr_data_q[23:16];
                    if (wr_strb_q[3]) reg_gauss_thresh[31:24] <= wr_data_q[31:24];
                end
            end
        end
    end

    // -------------------------------------------------------
    // AXI4-Lite read FSM.
    // Reads return the pre-write value of reg_x / reg_y when a write
    // is in flight (one-cycle latency window). Acceptable because
    // these registers are quasi-static and software does not poll
    // them around the write event.
    // -------------------------------------------------------
    logic [9:0] ar_addr_latch;

    assign s_axi_arready = !s_axi_rvalid;
    assign s_axi_rresp   = 2'b00;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= '0;
            ar_addr_latch <= '0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                automatic int ridx = int'(s_axi_araddr[9:3]);
                ar_addr_latch <= s_axi_araddr;

                if (ridx < NUM_QUBITS) begin
                    s_axi_rdata <= (!s_axi_araddr[2]) ? reg_x[ridx] : reg_y[ridx];
                end else if (ridx == NUM_QUBITS) begin
                    s_axi_rdata <= (!s_axi_araddr[2]) ? reg_ctrl : reg_gauss_thresh;
                end else begin
                    s_axi_rdata <= 32'hDEAD_BEEF;
                end

                s_axi_rvalid <= 1'b1;
            end

            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;
        end
    end

endmodule
