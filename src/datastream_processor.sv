// ----------------------------------------------------------------------
// datastream_processor_qick.sv -- dual-clock top level
// ----------------------------------------------------------------------
// Clock domains
//
//   i_aclk_ps  (300 MHz)  PS / AXI infrastructure clock
//     AXI4-Stream slave  (s_axis_pix_*)    from VDMA
//     AXI4-Lite  slave   (s_axi_coord_*)   from PS HPM (coords + ctrl)
//     AXI4-Lite  slave   (s_axi_lat_*)     latency capture readout
//     AXI4-Stream master (m_axis_qubit_*)  to DMA S2MM
//
//   i_clk_pl   (520 MHz)  fast PL processing clock (from MMCM)
//     pixel_injector
//     coord_matcher
//     roi_extractor
//     roi_storage (ping-pong + noise baseline)
//     read_streamer
//     gaussian_filter_engine x 4
//     latency_capture (observation taps + cycle counter)
//
// CDC strategy
//
//   1. INPUT  (300 -> 520):  axis_async_fifo packs {tuser, tlast,
//      tdata[63:0]}. 64-bit beat = 8 pixels. Gray-code pointers, 16-deep
//      distributed RAM.
//      The 520 MHz consumer runs ~1.73x faster than the 300 MHz
//      producer, so the FIFO drains faster than it fills and cannot
//      overflow under sustained streaming.
//
//   2. OUTPUT (520 -> 300):  axis_async_fifo packs {tlast, tdata[15:0]}.
//      Only 25 beats per frame, so the FIFO is never stressed.
//
//   3. QUBIT LUT (AXI4-Lite @ 300 MHz -> coord_matcher @ 520 MHz):
//      The coordinate file is quasi-static. Software writes all
//      coordinates and then writes lut_valid; the coords are stable
//      for millions of cycles before any frame arrives. A 2-FF
//      synchroniser carries lut_valid into the 520 MHz domain and
//      gates the input FIFO read side (in_fifo_rready) so that
//      pixel data cannot reach the matcher before the coords have
//      settled. The coordinate words themselves cross as quasi-
//      static signals (false_path in the XDC); see the per-word
//      2-FF sync arrays below.
//
//   4. RESET:  reset_sync (3-stage) produces rst_n_pl synchronised
//      to i_clk_pl from i_aresetn_ps (300 MHz domain reset).
// ----------------------------------------------------------------------

`timescale 1ns / 1ps
import params_pkg::*;

module datastream_processor_qick (
    // -------------------------------------------------------
    // PS-domain clock / reset (300 MHz)
    // -------------------------------------------------------
    input  logic        i_aclk_ps,       // 300 MHz PS clock
    input  logic        i_aresetn_ps,    // active-low reset (PS domain)

    // -------------------------------------------------------
    // PL-domain fast clock (520 MHz from MMCM)
    // -------------------------------------------------------
    input  logic        i_clk_pl,        // 520 MHz PL processing clock

    // -------------------------------------------------------
    // AXI4-Stream slave: pixel data from VDMA (300 MHz)
    // -------------------------------------------------------
    input  logic [BEAT_BITS-1:0] s_axis_pix_tdata,   // 64-bit beat = 8 px
    input  logic        s_axis_pix_tvalid,
    output logic        s_axis_pix_tready,
    input  logic        s_axis_pix_tlast,
    input  logic [0:0]  s_axis_pix_tuser,

    // -------------------------------------------------------
    // AXI4-Lite slave: qubit coordinates + pipeline controls.
    // Address space:
    //   0x000-0x31F  qubit X/Y coords
    //   0x320        dark_mode (bit 0)
    //   0x324        gaussian_threshold [15:0]
    // -------------------------------------------------------
    input  logic [9:0]  s_axi_coord_awaddr,
    input  logic        s_axi_coord_awvalid,
    output logic        s_axi_coord_awready,
    input  logic [31:0] s_axi_coord_wdata,
    input  logic [3:0]  s_axi_coord_wstrb,
    input  logic        s_axi_coord_wvalid,
    output logic        s_axi_coord_wready,
    output logic [1:0]  s_axi_coord_bresp,
    output logic        s_axi_coord_bvalid,
    input  logic        s_axi_coord_bready,
    input  logic [9:0]  s_axi_coord_araddr,
    input  logic        s_axi_coord_arvalid,
    output logic        s_axi_coord_arready,
    output logic [31:0] s_axi_coord_rdata,
    output logic [1:0]  s_axi_coord_rresp,
    output logic        s_axi_coord_rvalid,
    input  logic        s_axi_coord_rready,

    // -------------------------------------------------------
    // AXI4-Lite slave: latency_capture readout (300 MHz)
    // -------------------------------------------------------
    input  logic [5:0]  s_axi_lat_araddr,
    input  logic        s_axi_lat_arvalid,
    output logic        s_axi_lat_arready,
    output logic [31:0] s_axi_lat_rdata,
    output logic [1:0]  s_axi_lat_rresp,
    output logic        s_axi_lat_rvalid,
    input  logic        s_axi_lat_rready,

    input  logic [5:0]  s_axi_lat_awaddr,
    input  logic        s_axi_lat_awvalid,
    output logic        s_axi_lat_awready,
    input  logic [31:0] s_axi_lat_wdata,
    input  logic [3:0]  s_axi_lat_wstrb,
    input  logic        s_axi_lat_wvalid,
    output logic        s_axi_lat_wready,
    output logic [1:0]  s_axi_lat_bresp,
    output logic        s_axi_lat_bvalid,
    input  logic        s_axi_lat_bready,

    // -------------------------------------------------------
    // AXI4-Stream master: qubit result stream to DMA (300 MHz)
    // -------------------------------------------------------
    output logic [15:0] m_axis_qubit_tdata,
    output logic        m_axis_qubit_tvalid,
    input  logic        m_axis_qubit_tready,
    output logic        m_axis_qubit_tlast
);

    // ------------------------------------------------------------------
    // A. Reset synchroniser (300 -> 520 MHz)
    // ------------------------------------------------------------------
    logic rst_n_pl;   // active-low reset synchronised to i_clk_pl

    (* KEEP_HIERARCHY = "yes" *) reset_sync #(.STAGES(3)) u_rst_sync (
        .clk_dst   (i_clk_pl),
        .rst_n_src (i_aresetn_ps),
        .rst_n_dst (rst_n_pl)
    );

    // ------------------------------------------------------------------
    // B. Qubit lookup table -- AXI4-Lite @ 300 MHz
    //    Produces quasi-static outputs:
    //      q_x_ps[], q_y_ps[]   100 (X,Y) coordinate pairs
    //      lut_valid_ps         set on the first coord write
    //      dark_mode_ps         ctrl bit 0
    //      gauss_thresh_ps      runtime threshold [15:0]
    //
    // CDC strategy for the coordinate arrays:
    //   A 2-FF synchroniser is applied to each coordinate word
    //   independently.
    //     (a) Software writes all coordinates, then writes lut_valid.
    //     (b) lut_valid_pl (2-FF synced) gates in_fifo_rready so no
    //         pixel can reach coord_matcher until lut_valid_pl rises.
    //     (c) lut_valid_pl rises at least 2 cycles of clk_520 AFTER
    //         lut_written rises in clk_300, by which time q_x/q_y
    //         have been stable for thousands of 520 MHz cycles.
    //     (d) The metastability window for each q_x/q_y word is
    //         therefore closed long before any compare runs.
    // ------------------------------------------------------------------
    logic [COORD_WIDTH-1:0] q_x_ps [0:NUM_QUBITS-1];   // 300 MHz raw outputs
    logic [COORD_WIDTH-1:0] q_y_ps [0:NUM_QUBITS-1];
    logic                   lut_valid_ps;
    logic                   dark_mode_ps;
    logic [15:0]            gauss_thresh_ps;

    qubit_lookup_axi u_lut (
        .s_axi_aclk    (i_aclk_ps),
        .s_axi_aresetn (i_aresetn_ps),

        .s_axi_awaddr  (s_axi_coord_awaddr),
        .s_axi_awvalid (s_axi_coord_awvalid),
        .s_axi_awready (s_axi_coord_awready),
        .s_axi_wdata   (s_axi_coord_wdata),
        .s_axi_wstrb   (s_axi_coord_wstrb),
        .s_axi_wvalid  (s_axi_coord_wvalid),
        .s_axi_wready  (s_axi_coord_wready),
        .s_axi_bresp   (s_axi_coord_bresp),
        .s_axi_bvalid  (s_axi_coord_bvalid),
        .s_axi_bready  (s_axi_coord_bready),
        .s_axi_araddr  (s_axi_coord_araddr),
        .s_axi_arvalid (s_axi_coord_arvalid),
        .s_axi_arready (s_axi_coord_arready),
        .s_axi_rdata   (s_axi_coord_rdata),
        .s_axi_rresp   (s_axi_coord_rresp),
        .s_axi_rvalid  (s_axi_coord_rvalid),
        .s_axi_rready  (s_axi_coord_rready),

        .o_q_x               (q_x_ps),
        .o_q_y               (q_y_ps),
        .o_lut_valid         (lut_valid_ps),
        .o_dark_mode         (dark_mode_ps),
        .o_gaussian_threshold(gauss_thresh_ps)
    );

    // ------------------------------------------------------------------
    // Per-word 2-FF synchroniser arrays for q_x and q_y (300 -> 520 MHz).
    // q_xy_sync1 is the metastability-capture stage, q_xy_sync2 is the
    // stable resolved output fed to coord_matcher. Both arrays carry
    // ASYNC_REG and KEEP so Vivado co-places the FFs and does not
    // optimise them away.
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic [COORD_WIDTH-1:0] q_x_sync1 [0:NUM_QUBITS-1];
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic [COORD_WIDTH-1:0] q_x_sync2 [0:NUM_QUBITS-1];
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic [COORD_WIDTH-1:0] q_y_sync1 [0:NUM_QUBITS-1];
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic [COORD_WIDTH-1:0] q_y_sync2 [0:NUM_QUBITS-1];

    always_ff @(posedge i_clk_pl or negedge rst_n_pl) begin
        if (!rst_n_pl) begin
            for (int s = 0; s < NUM_QUBITS; s++) begin
                q_x_sync1[s] <= '0;  q_x_sync2[s] <= '0;
                q_y_sync1[s] <= '0;  q_y_sync2[s] <= '0;
            end
        end else begin
            for (int s = 0; s < NUM_QUBITS; s++) begin
                q_x_sync1[s] <= q_x_ps[s];  q_x_sync2[s] <= q_x_sync1[s];
                q_y_sync1[s] <= q_y_ps[s];  q_y_sync2[s] <= q_y_sync1[s];
            end
        end
    end

    // Stable 520 MHz copies fed to coord_matcher.
    logic [COORD_WIDTH-1:0] q_x [0:NUM_QUBITS-1];
    logic [COORD_WIDTH-1:0] q_y [0:NUM_QUBITS-1];
    always_comb begin
        for (int c = 0; c < NUM_QUBITS; c++) begin
            q_x[c] = q_x_sync2[c];
            q_y[c] = q_y_sync2[c];
        end
    end

    // ------------------------------------------------------------------
    // 2-FF synchroniser for lut_valid (300 -> 520 MHz).
    // Gates the input CDC FIFO read side until coords are settled.
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic lut_valid_sync1, lut_valid_sync2;
    always_ff @(posedge i_clk_pl or negedge rst_n_pl) begin
        if (!rst_n_pl) begin
            lut_valid_sync1 <= 1'b0;
            lut_valid_sync2 <= 1'b0;
        end else begin
            lut_valid_sync1 <= lut_valid_ps;
            lut_valid_sync2 <= lut_valid_sync1;
        end
    end
    logic lut_valid_pl;
    assign lut_valid_pl = lut_valid_sync2;  // gates in_fifo_rready

    // ------------------------------------------------------------------
    // 2-FF synchroniser for dark_mode (300 -> 520 MHz).
    // Quasi-static: software writes between frames.
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic dark_mode_sync1, dark_mode_sync2;
    always_ff @(posedge i_clk_pl or negedge rst_n_pl) begin
        if (!rst_n_pl) begin
            dark_mode_sync1 <= 1'b0;
            dark_mode_sync2 <= 1'b0;
        end else begin
            dark_mode_sync1 <= dark_mode_ps;
            dark_mode_sync2 <= dark_mode_sync1;
        end
    end
    logic dark_mode_pl;
    assign dark_mode_pl = dark_mode_sync2;

    // ------------------------------------------------------------------
    // 2-FF synchroniser for gaussian_threshold (300 -> 520 MHz).
    // Reset value is GAUSS_THRESHOLD_DEFAULT so the engines behave
    // correctly before software ever writes the register.
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *)
    logic [15:0] gauss_thresh_sync1, gauss_thresh_sync2;
    always_ff @(posedge i_clk_pl or negedge rst_n_pl) begin
        if (!rst_n_pl) begin
            gauss_thresh_sync1 <= 16'(GAUSS_THRESHOLD_DEFAULT);
            gauss_thresh_sync2 <= 16'(GAUSS_THRESHOLD_DEFAULT);
        end else begin
            gauss_thresh_sync1 <= gauss_thresh_ps;
            gauss_thresh_sync2 <= gauss_thresh_sync1;
        end
    end
    logic [15:0] gauss_thresh_pl;
    assign gauss_thresh_pl = gauss_thresh_sync2;

    // ------------------------------------------------------------------
    // C. Input CDC FIFO (300 -> 520 MHz)
    // ------------------------------------------------------------------
    localparam int IN_FIFO_W     = BEAT_BITS + 2;   // {tuser, tlast, tdata[63:0]} = 66
    localparam int IN_FIFO_DEPTH = 16;

    logic [IN_FIFO_W-1:0] in_fifo_wdata;
    logic [IN_FIFO_W-1:0] in_fifo_rdata;
    logic                  in_fifo_rvalid;
    logic                  in_fifo_rready;

    assign in_fifo_wdata = {s_axis_pix_tuser[0], s_axis_pix_tlast, s_axis_pix_tdata};

    (* KEEP_HIERARCHY = "yes" *) axis_async_fifo #(
        .DATA_W (IN_FIFO_W),
        .DEPTH  (IN_FIFO_DEPTH)
    ) u_in_cdc (
        // Write side: 300 MHz (from VDMA)
        .wr_clk       (i_aclk_ps),
        .wr_rst_n     (i_aresetn_ps),
        .s_axis_tdata  (in_fifo_wdata),
        .s_axis_tvalid (s_axis_pix_tvalid),
        .s_axis_tready (s_axis_pix_tready),

        // Read side: 520 MHz (to processing pipeline)
        .rd_clk       (i_clk_pl),
        .rd_rst_n     (rst_n_pl),
        .m_axis_tdata  (in_fifo_rdata),
        .m_axis_tvalid (in_fifo_rvalid),
        .m_axis_tready (in_fifo_rready)
    );

    // Unpack the FIFO output for the pixel_injector input (520 MHz).
    logic [BEAT_BITS-1:0] pl_pix_tdata;
    logic        pl_pix_tvalid;
    logic        pl_pix_tready;
    logic        pl_pix_tlast;
    logic [0:0]  pl_pix_tuser;

    assign pl_pix_tdata   = in_fifo_rdata[BEAT_BITS-1:0];
    assign pl_pix_tlast   = in_fifo_rdata[BEAT_BITS];
    assign pl_pix_tuser   = in_fifo_rdata[BEAT_BITS+1];
    // Suppress tvalid until the LUT is settled in this domain.
    assign pl_pix_tvalid  = in_fifo_rvalid & lut_valid_pl;
    // Stall the FIFO until the LUT is settled.
    assign in_fifo_rready = pl_pix_tready  & lut_valid_pl;

    // ------------------------------------------------------------------
    // D. Processing pipeline @ 520 MHz
    // ------------------------------------------------------------------

    // ---- D.1  Pixel injector ----
    logic [BEAT_BITS-1:0]   core_pixel_data;
    logic                   core_pixel_valid;
    logic [COORD_WIDTH-1:0] proc_x, proc_y;
    logic                   frame_done_pulse;
    logic                   sync_lval, sync_fval;

    pixel_injector u_injector (
        .i_aclk        (i_clk_pl),
        .i_aresetn     (rst_n_pl),
        .s_axis_tdata  (pl_pix_tdata),
        .s_axis_tvalid (pl_pix_tvalid),
        .s_axis_tready (pl_pix_tready),
        .s_axis_tlast  (pl_pix_tlast),
        .s_axis_tuser  (pl_pix_tuser),
        .o_pixel_data  (core_pixel_data),
        .o_pixel_valid (core_pixel_valid),
        .o_pixel_x     (proc_x),
        .o_pixel_y     (proc_y),
        .o_frame_done  (frame_done_pulse),
        .o_sync_lval   (sync_lval),
        .o_sync_fval   (sync_fval)
    );

    // ---- D.2  Coordinate matcher ----
    logic                          match_found;
    logic [QUBIT_ID_WIDTH-1:0]     match_idx;
    logic                          match_valid;
    logic [MATCH_OFFSET_WIDTH-1:0] match_offset;

    coord_matcher u_match (
        .i_clk         (i_clk_pl),
        .i_rst_n       (rst_n_pl),
        // Quasi-static LUT, written @ 300 MHz, read @ 520 MHz
        .i_q_x         (q_x),
        .i_q_y         (q_y),

        .i_curr_x      (proc_x),
        .i_curr_y      (proc_y),
        .i_valid       (core_pixel_valid),
        .i_sync_lval   (sync_lval),
        .i_sync_fval   (sync_fval),

        .o_match_found (match_found),
        .o_match_offset(match_offset),
        .o_qubit_index (match_idx),
        .o_valid_out   (match_valid)
    );

    // ---- D.3  ROI extractor ----
    logic [ROI_BITS-1:0]       roi_flat;
    logic [QUBIT_ID_WIDTH-1:0] roi_idx;
    logic                      roi_wr_en;

    roi_extractor u_extract (
        .i_clk           (i_clk_pl),
        .i_rst_n         (rst_n_pl),
        .i_pixel_data    (core_pixel_data),
        .i_pixel_valid   (core_pixel_valid),
        .i_sync_lval     (sync_lval),
        .i_sync_fval     (sync_fval),
        .i_match_trigger (match_found),
        .i_match_offset  (match_offset),
        .i_qubit_index   (match_idx),
        .o_roi_flat      (roi_flat),
        .o_qubit_index   (roi_idx),
        .o_write_enable  (roi_wr_en)
    );

    // ---- D.4  ROI storage (ping-pong + noise baseline) ----
    logic                      storage_rd_en;
    logic [BANK_ADDR_WIDTH-1:0] storage_rd_addr;

    logic [ROI_BITS-1:0] rd_data_0, rd_data_1, rd_data_2, rd_data_3;
    logic [ROI_BITS-1:0] rd_noise_0, rd_noise_1, rd_noise_2, rd_noise_3;
    logic                frame_ready;

    roi_storage u_storage (
        .i_wr_clk      (i_clk_pl),
        .i_rd_clk      (i_clk_pl),
        .i_rst_n       (rst_n_pl),

        .i_wr_en       (roi_wr_en),
        .i_wr_addr     (roi_idx),
        .i_wr_data     (roi_flat),
        .i_frame_done  (frame_done_pulse),

        // Dark mode routes ROIs into base_noise banks and suppresses
        // frame_ready.
        .i_dark_mode   (dark_mode_pl),

        .i_rd_en       (storage_rd_en),
        .i_rd_addr     (storage_rd_addr),

        .o_rd_data_0   (rd_data_0),
        .o_rd_data_1   (rd_data_1),
        .o_rd_data_2   (rd_data_2),
        .o_rd_data_3   (rd_data_3),

        .o_noise_data_0(rd_noise_0),
        .o_noise_data_1(rd_noise_1),
        .o_noise_data_2(rd_noise_2),
        .o_noise_data_3(rd_noise_3),

        .o_frame_ready (frame_ready)
    );

    // ---- D.5  Read streamer (with noise subtraction) ----
    logic [ROI_BITS-1:0]       pd_lane_0, pd_lane_1, pd_lane_2, pd_lane_3;
    logic [QUBIT_ID_WIDTH-1:0] pd_base_id;
    logic                      pd_valid;

    read_streamer u_stream (
        .i_clk                   (i_clk_pl),
        .i_rst_n                 (rst_n_pl),
        .i_start                 (frame_ready),

        .o_rd_en                 (storage_rd_en),
        .o_rd_addr               (storage_rd_addr),

        // Ping-pong ROI data
        .i_rd_data_0             (rd_data_0),
        .i_rd_data_1             (rd_data_1),
        .i_rd_data_2             (rd_data_2),
        .i_rd_data_3             (rd_data_3),

        // Noise baseline (subtracted inside read_streamer per pixel)
        .i_noise_data_0          (rd_noise_0),
        .i_noise_data_1          (rd_noise_1),
        .i_noise_data_2          (rd_noise_2),
        .i_noise_data_3          (rd_noise_3),

        .o_pixeldata_lane_0      (pd_lane_0),
        .o_pixeldata_lane_1      (pd_lane_1),
        .o_pixeldata_lane_2      (pd_lane_2),
        .o_pixeldata_lane_3      (pd_lane_3),
        .o_pixeldata_lane_base_id(pd_base_id),
        .o_pixeldata_lane_valid  (pd_valid)
    );

    // ---- D.6  Gaussian filter bank (4 lanes in parallel) ----
    // All four engines receive the same runtime threshold.
    logic [3:0]                 pl_qubit_state;
    logic [QUBIT_ID_WIDTH-1:0]  pl_qubit_base_id;
    logic                       pl_qubit_valid;

    genvar k;
    generate
        for (k = 0; k < 4; k++) begin : gen_filter
            logic [ROI_BITS-1:0]       lane_data;
            logic [QUBIT_ID_WIDTH-1:0] eng_id_in;
            logic                      eng_decision, eng_valid;
            logic [QUBIT_ID_WIDTH-1:0] eng_id_out;

            assign lane_data = (k==0) ? pd_lane_0 :
                               (k==1) ? pd_lane_1 :
                               (k==2) ? pd_lane_2 : pd_lane_3;
            assign eng_id_in = pd_base_id + QUBIT_ID_WIDTH'(k);

            gaussian_filter_engine u_eng (
                .i_clk      (i_clk_pl),
                .i_rst_n    (rst_n_pl),
                .i_roi_data (lane_data),
                .i_valid    (pd_valid),
                .i_base_id  (eng_id_in),
                .i_threshold(gauss_thresh_pl),
                .o_decision (eng_decision),
                .o_score    (),
                .o_base_id  (eng_id_out),
                .o_valid    (eng_valid)
            );

            assign pl_qubit_state[k] = eng_decision;

            if (k == 0) begin : gen_ctrl
                assign pl_qubit_valid   = eng_valid;
                assign pl_qubit_base_id = eng_id_out;
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // E. Output CDC FIFO (520 -> 300 MHz)
    // ------------------------------------------------------------------
    localparam int OUT_FIFO_W     = 17;
    localparam int OUT_FIFO_DEPTH = 32;
    localparam LAST_BATCH_ID = (NUM_QUBITS / NUM_BANKS - 1) * NUM_BANKS;  // 96

    // Pack qubit results in the PL domain.
    logic [15:0] pl_qubit_tdata;
    logic        pl_qubit_tvalid;
    logic        pl_qubit_tlast;

    assign pl_qubit_tdata  = {5'b0, pl_qubit_base_id, pl_qubit_state};
    assign pl_qubit_tvalid = pl_qubit_valid;
    assign pl_qubit_tlast  = pl_qubit_valid &&
                              (pl_qubit_base_id == QUBIT_ID_WIDTH'(LAST_BATCH_ID));

    logic [OUT_FIFO_W-1:0] out_fifo_wdata;
    logic                  out_fifo_wready;
    logic [OUT_FIFO_W-1:0] out_fifo_rdata;
    logic                  out_fifo_rvalid;

    assign out_fifo_wdata = {pl_qubit_tlast, pl_qubit_tdata};

    (* KEEP_HIERARCHY = "yes" *) axis_async_fifo #(
        .DATA_W (OUT_FIFO_W),
        .DEPTH  (OUT_FIFO_DEPTH)
    ) u_out_cdc (
        // Write side: 520 MHz (from processing pipeline)
        .wr_clk       (i_clk_pl),
        .wr_rst_n     (rst_n_pl),
        .s_axis_tdata  (out_fifo_wdata),
        .s_axis_tvalid (pl_qubit_tvalid),
        .s_axis_tready (out_fifo_wready),   // should always be ready

        // Read side: 300 MHz (to PS DMA)
        .rd_clk       (i_aclk_ps),
        .rd_rst_n     (i_aresetn_ps),
        .m_axis_tdata  (out_fifo_rdata),
        .m_axis_tvalid (out_fifo_rvalid),
        .m_axis_tready (m_axis_qubit_tready)
    );

    // Unpack onto the AXI4-Stream master (300 MHz domain).
    assign m_axis_qubit_tdata  = out_fifo_rdata[15:0];
    assign m_axis_qubit_tlast  = out_fifo_rdata[16] & out_fifo_rvalid;
    assign m_axis_qubit_tvalid = out_fifo_rvalid;

    // ------------------------------------------------------------------
    // F. Latency-capture instrumentation
    //
    // All 7 taps are existing internal signals; no extra logic on
    // the critical path. The SOF signal is the first valid pixel
    // emerging from the input CDC FIFO with tuser = 1, which is the
    // earliest observable point after the 300 -> 520 MHz crossing,
    // i.e. the natural "PL entry" timestamp for the frame.
    // ------------------------------------------------------------------
    logic latcap_sof_valid;
    assign latcap_sof_valid = pl_pix_tvalid & pl_pix_tuser[0];

    latency_capture u_latcap (
        // PL processing clock (520 MHz)
        .i_clk_pl       (i_clk_pl),
        .i_rst_n_pl     (rst_n_pl),

        // Observation taps (all 520 MHz, directly wired)
        .i_sof_valid    (latcap_sof_valid),    // first pixel out of CDC FIFO with SOF
        .i_match_found  (match_found),          // coord_matcher o_match_found
        .i_roi_wr_en    (roi_wr_en),            // roi_extractor o_write_enable
        .i_frame_ready  (frame_ready),          // roi_storage o_frame_ready
        .i_qubit_valid  (pl_qubit_valid),       // gaussian bank[0] o_valid
        .i_qubit_tlast  (pl_qubit_tlast),       // last beat qualifier (base_id == 96)
        .i_frame_done   (frame_done_pulse),     // pixel_injector o_frame_done

        // AXI4-Lite read-only slave (300 MHz)
        .s_axi_aclk     (i_aclk_ps),
        .s_axi_aresetn  (i_aresetn_ps),

        .s_axi_araddr   (s_axi_lat_araddr),
        .s_axi_arvalid  (s_axi_lat_arvalid),
        .s_axi_arready  (s_axi_lat_arready),
        .s_axi_rdata    (s_axi_lat_rdata),
        .s_axi_rresp    (s_axi_lat_rresp),
        .s_axi_rvalid   (s_axi_lat_rvalid),
        .s_axi_rready   (s_axi_lat_rready),

        .s_axi_awaddr   (s_axi_lat_awaddr),
        .s_axi_awvalid  (s_axi_lat_awvalid),
        .s_axi_awready  (s_axi_lat_awready),
        .s_axi_wdata    (s_axi_lat_wdata),
        .s_axi_wstrb    (s_axi_lat_wstrb),
        .s_axi_wvalid   (s_axi_lat_wvalid),
        .s_axi_wready   (s_axi_lat_wready),
        .s_axi_bresp    (s_axi_lat_bresp),
        .s_axi_bvalid   (s_axi_lat_bvalid),
        .s_axi_bready   (s_axi_lat_bready)
    );

    // ------------------------------------------------------------------
    // G. Assertions (simulation only)
    // ------------------------------------------------------------------
    // synthesis translate_off

    // The output FIFO should always accept a result.
    property p_out_fifo_no_drop;
        @(posedge i_clk_pl) disable iff (!rst_n_pl)
        (pl_qubit_tvalid) |-> (out_fifo_wready);
    endproperty
    assert property (p_out_fifo_no_drop)
    else $error("[CDC] Output FIFO overflow -- qubit result lost.");

    // No pixel should reach coord_matcher before the LUT is valid.
    property p_lut_before_pixels;
        @(posedge i_clk_pl) disable iff (!rst_n_pl)
        (core_pixel_valid) |-> (lut_valid_pl);
    endproperty
    assert property (p_lut_before_pixels)
    else $fatal(1, "[LUT] Pixel reached coord_matcher before lut_valid_pl -- check programming order.");

    // frame_ready must never fire during dark_mode.
    property p_no_frame_ready_in_dark;
        @(posedge i_clk_pl) disable iff (!rst_n_pl)
        (dark_mode_pl) |-> !(frame_ready);
    endproperty
    assert property (p_no_frame_ready_in_dark)
    else $error("[DARK] frame_ready asserted during dark_mode -- logic error.");

    // Watch for long back-pressure stalls on the input FIFO.
    property p_in_fifo_no_long_stall;
        @(posedge i_aclk_ps) disable iff (!i_aresetn_ps)
        (s_axis_pix_tvalid && !s_axis_pix_tready) |->
            ##[1:8] s_axis_pix_tready;
    endproperty
    assert property (p_in_fifo_no_long_stall)
    else $warning("[CDC] Input FIFO stalling >8 cycles -- possible throughput issue.");

    // synthesis translate_on

endmodule
