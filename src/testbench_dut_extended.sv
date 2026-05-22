 // --------------------------------------------------------------------
 // testbench_dut_extended.sv -- qubit readout pipeline verification
 // --------------------------------------------------------------------
 // Adds latency-capture readback and dark-mode noise-baseline coverage
 // to the baseline pipeline test.
 //
 // Dark-mode capture phase (runs once before the main frame loop):
 //   1. Program dark_mode = 1 via AXI write to 0x320.
 //   2. Program gaussian_threshold = 500 via AXI write to 0x324.
 //   3. Inject an all-zero frame (closed-shutter equivalent).
 //   4. Pipeline writes the 100 ROIs into the base_noise banks.
 //   5. frame_ready is suppressed, so no Gaussian output during dark
 //      capture.
 //   6. Program dark_mode = 0 to resume normal operation.
 //
 //   The dark frame uses all-zero pixels, so the noise baseline is
 //   zero everywhere and saturating subtraction of zero is a no-op.
 //   That keeps the existing test vectors valid while still
 //   exercising:
 //     (a) dark_mode = 1 silences the Gaussian output path
 //     (b) dark_mode = 0 re-enables the pipeline correctly
 //     (c) gaussian_threshold can be programmed via AXI at runtime
 //     (d) AXI readback of 0x320 and 0x324 returns the written values
 //
 // Main frames (8):
 //   F0  ALL-GROUND       0/100 Rydberg  -- dark frame, baseline
 //   F1  COL-STRIPES     50/100 Rydberg  -- even columns lit
 //   F2  CHECKERBOARD    50/100 Rydberg  -- (row+col)%2 + backpressure
 //   F3  ALL-RYDBERG    100/100 Rydberg  -- fully bright frame
 //   F4  SINGLE-Q0        1/100 Rydberg  -- only Q0 lit; crosstalk canary
 //   F5  ALL-ZERO         0/100 Rydberg  -- all pixels 0
 //   F6  ROW-STRIPES     50/100 Rydberg  -- even rows lit (Y spacing)
 //   F7  NEAR-THRESHOLD  50/100 Rydberg  -- tight margin either side
 //                                          even q: pixel=32, score=512 > 500 -> Rydberg
 //                                          odd  q: pixel=31, score=496 < 500 -> Ground
 //
 // Reprogram phase frames (2):
 //   F8  RP-ALL-GROUND   coord grid shifted +5,+5
 //   F9  RP-ALL-RYDBERG  coord grid shifted +5,+5
 //
 // Target: Neutral-atom QP real-time readout, ZCU216, QICK firmware
 // --------------------------------------------------------------------

 `timescale 1ns / 1ps
 import params_pkg::*;

 module tb_qubit_readout_extended;

 // --------------------------------------------------------------------
 // 1. Simulation parameters
 // --------------------------------------------------------------------
 localparam int  NUM_MAIN_FRAMES = 8;   // frames 0-7: main test patterns
 localparam int  NUM_RP_FRAMES   = 2;   // frames 8-9: reprogrammed-grid phase
 localparam int  TOTAL_FRAMES    = NUM_MAIN_FRAMES + NUM_RP_FRAMES;
 localparam int  COORD_SHIFT     = 5;   // pixel shift applied to grid in reprogram phase

 localparam real PS_CLK_PERIOD   = 3.333;     // 300 MHz
 localparam real PL_CLK_PERIOD   = 1.923;     // 520 MHz

 localparam int  BEATS_PER_LINE  = IMAGE_WIDTH / PIXELS_PER_BEAT;   // 64 beats per line @ 8 px/beat
 localparam int  LAST_BATCH_ID   = (NUM_QUBITS / NUM_BANKS - 1) * NUM_BANKS;  // 96

 // Pixel model constants
 localparam int  BG_LO          = 8;
 localparam int  BG_HI          = 28;
 localparam int  FL_CENTER      = 210;
 localparam int  FL_ADJACENT    = 140;
 localparam int  FL_DIAGONAL    = 80;
 localparam int  GAUSS_THRESH   = 500;

 // Near-threshold constants for frame 7:
 //   Flat background of 16 across the whole 3x3 ROI. Then:
 //     Rydberg: pixel = 16 + FL_NEAR_ABOVE = 32  -> score = 16*32 = 512 > 500
 //     Ground : pixel = 16 + FL_NEAR_BELOW = 31  -> score = 16*31 = 496 < 500
 //   Margins are deliberately tight (+12 / -4 from threshold).
 localparam int  FL_NEAR_ABOVE  = 16;
 localparam int  FL_NEAR_BELOW  = 15;
 localparam int  BG_NEAR_THRESH = 16;

 localparam int  VERBOSE_LIVE   = 0;

 // ---- AXI address map for control registers ----
 // Qubit coords occupy 0x000 .. 0x31F  (100 qubits * 8 bytes)
 // Control regs: 0x320 (dark_mode), 0x324 (gaussian_threshold)
 localparam logic [9:0] DARK_MODE_ADDR    = 10'(NUM_QUBITS * 8);      // 0x320
 localparam logic [9:0] GAUSS_THRESH_ADDR = 10'(NUM_QUBITS * 8 + 4);  // 0x324

 // ---- Latency-capture register address map ----
 localparam logic [5:0] LAT_COUNTER     = 6'h00;
 localparam logic [5:0] LAT_FRAME_SEQ   = 6'h04;
 localparam logic [5:0] LAT_TS_SOF      = 6'h08;
 localparam logic [5:0] LAT_TS_MATCH    = 6'h0C;
 localparam logic [5:0] LAT_TS_ROI_WR   = 6'h10;
 localparam logic [5:0] LAT_TS_READY    = 6'h14;
 localparam logic [5:0] LAT_TS_FIRST_R  = 6'h18;
 localparam logic [5:0] LAT_TS_LAST_R   = 6'h1C;
 localparam logic [5:0] LAT_TS_FDONE    = 6'h20;
 localparam logic [5:0] LAT_D_PIPE      = 6'h24;
 localparam logic [5:0] LAT_D_MATCH     = 6'h28;
 localparam logic [5:0] LAT_D_ROI       = 6'h2C;
 localparam logic [5:0] LAT_D_STORAGE   = 6'h30;
 localparam logic [5:0] LAT_D_READOUT   = 6'h34;
 localparam logic [5:0] LAT_D_FILTER    = 6'h38;

 // --------------------------------------------------------------------
 // 2. Signal declarations
 // --------------------------------------------------------------------
 logic        i_aclk_ps    = 0;
 logic        i_clk_pl     = 0;
 logic        i_aresetn_ps = 0;

 // AXI4-Stream pixel input  (300 MHz)  -- 64-bit beat = 8 px
 logic [BEAT_BITS-1:0] s_axis_pix_tdata  = '0;
 logic        s_axis_pix_tvalid = 0;
 logic        s_axis_pix_tready;
 logic        s_axis_pix_tlast  = 0;
 logic [0:0]  s_axis_pix_tuser  = 0;

 // AXI4-Lite coordinate / control config  (300 MHz)
 logic [9:0]  s_axi_coord_awaddr  = '0;  logic s_axi_coord_awvalid = 0;  logic s_axi_coord_awready;
 logic [31:0] s_axi_coord_wdata   = '0;  logic [3:0] s_axi_coord_wstrb = 4'hF;
 logic        s_axi_coord_wvalid  = 0;   logic s_axi_coord_wready;
 logic [1:0]  s_axi_coord_bresp;         logic s_axi_coord_bvalid;  logic s_axi_coord_bready = 1;
 logic [9:0]  s_axi_coord_araddr  = '0;  logic s_axi_coord_arvalid = 0;  logic s_axi_coord_arready;
 logic [31:0] s_axi_coord_rdata;         logic [1:0] s_axi_coord_rresp;  logic s_axi_coord_rvalid;
 logic        s_axi_coord_rready  = 1;

 // AXI4-Lite latency-capture readout  (300 MHz)
 logic [5:0]  s_axi_lat_araddr  = '0;
 logic        s_axi_lat_arvalid = 0;
 logic        s_axi_lat_arready;
 logic [31:0] s_axi_lat_rdata;
 logic [1:0]  s_axi_lat_rresp;
 logic        s_axi_lat_rvalid;
 logic        s_axi_lat_rready  = 1;

 logic [5:0]  s_axi_lat_awaddr  = '0;
 logic        s_axi_lat_awvalid = 0;
 logic        s_axi_lat_awready;
 logic [31:0] s_axi_lat_wdata   = '0;
 logic [3:0]  s_axi_lat_wstrb   = 4'hF;
 logic        s_axi_lat_wvalid  = 0;
 logic        s_axi_lat_wready;
 logic [1:0]  s_axi_lat_bresp;
 logic        s_axi_lat_bvalid;
 logic        s_axi_lat_bready  = 1;

 // AXI4-Stream qubit result output  (300 MHz)
 logic [15:0] m_axis_qubit_tdata;
 logic        m_axis_qubit_tvalid;
 logic        m_axis_qubit_tready = 1'b1;   // default ready; toggled in backpressure test
 logic        m_axis_qubit_tlast;

 // --------------------------------------------------------------------
 // 3. DUT instantiation
 // --------------------------------------------------------------------
 datastream_processor_qick DUT (
     .i_aclk_ps             (i_aclk_ps),
     .i_aresetn_ps          (i_aresetn_ps),
     .i_clk_pl              (i_clk_pl),

     .s_axis_pix_tdata      (s_axis_pix_tdata),
     .s_axis_pix_tvalid     (s_axis_pix_tvalid),
     .s_axis_pix_tready     (s_axis_pix_tready),
     .s_axis_pix_tlast      (s_axis_pix_tlast),
     .s_axis_pix_tuser      (s_axis_pix_tuser),

     .s_axi_coord_awaddr    (s_axi_coord_awaddr),
     .s_axi_coord_awvalid   (s_axi_coord_awvalid),
     .s_axi_coord_awready   (s_axi_coord_awready),
     .s_axi_coord_wdata     (s_axi_coord_wdata),
     .s_axi_coord_wstrb     (s_axi_coord_wstrb),
     .s_axi_coord_wvalid    (s_axi_coord_wvalid),
     .s_axi_coord_wready    (s_axi_coord_wready),
     .s_axi_coord_bresp     (s_axi_coord_bresp),
     .s_axi_coord_bvalid    (s_axi_coord_bvalid),
     .s_axi_coord_bready    (s_axi_coord_bready),
     .s_axi_coord_araddr    (s_axi_coord_araddr),
     .s_axi_coord_arvalid   (s_axi_coord_arvalid),
     .s_axi_coord_arready   (s_axi_coord_arready),
     .s_axi_coord_rdata     (s_axi_coord_rdata),
     .s_axi_coord_rresp     (s_axi_coord_rresp),
     .s_axi_coord_rvalid    (s_axi_coord_rvalid),
     .s_axi_coord_rready    (s_axi_coord_rready),

     // Latency-capture AXI4-Lite port
     .s_axi_lat_araddr      (s_axi_lat_araddr),
     .s_axi_lat_arvalid     (s_axi_lat_arvalid),
     .s_axi_lat_arready     (s_axi_lat_arready),
     .s_axi_lat_rdata       (s_axi_lat_rdata),
     .s_axi_lat_rresp       (s_axi_lat_rresp),
     .s_axi_lat_rvalid      (s_axi_lat_rvalid),
     .s_axi_lat_rready      (s_axi_lat_rready),
     .s_axi_lat_awaddr      (s_axi_lat_awaddr),
     .s_axi_lat_awvalid     (s_axi_lat_awvalid),
     .s_axi_lat_awready     (s_axi_lat_awready),
     .s_axi_lat_wdata       (s_axi_lat_wdata),
     .s_axi_lat_wstrb       (s_axi_lat_wstrb),
     .s_axi_lat_wvalid      (s_axi_lat_wvalid),
     .s_axi_lat_wready      (s_axi_lat_wready),
     .s_axi_lat_bresp       (s_axi_lat_bresp),
     .s_axi_lat_bvalid      (s_axi_lat_bvalid),
     .s_axi_lat_bready      (s_axi_lat_bready),

     .m_axis_qubit_tdata    (m_axis_qubit_tdata),
     .m_axis_qubit_tvalid   (m_axis_qubit_tvalid),
     .m_axis_qubit_tready   (m_axis_qubit_tready),
     .m_axis_qubit_tlast    (m_axis_qubit_tlast)
 );

 // --------------------------------------------------------------------
 // 4. Clock generation. The PL clock has a deliberate phase offset
 //    relative to PS so the CDC gray-code pointers are exercised
 //    against a non-trivial phase relationship.
 // --------------------------------------------------------------------
 always #(PS_CLK_PERIOD / 2.0) i_aclk_ps = ~i_aclk_ps;

 initial begin
     i_clk_pl = 0;
     #(PL_CLK_PERIOD * 0.37);
     forever #(PL_CLK_PERIOD / 2.0) i_clk_pl = ~i_clk_pl;
 end

 // --------------------------------------------------------------------
 // 5. Ground truth, frame buffers, per-frame coordinate offsets
 // --------------------------------------------------------------------
 // qubit_state_gt[f][q] = 1 if qubit q is expected to be Rydberg in frame f.
 // frame_buf covers all TOTAL_FRAMES.
 // frame_dx/dy give the per-frame coordinate offset added to the base
 // grid: zero for frames 0-7, COORD_SHIFT for the reprogram phase.
 logic       qubit_state_gt [TOTAL_FRAMES][NUM_QUBITS];
 logic [7:0] frame_buf      [TOTAL_FRAMES][IMAGE_HEIGHT][IMAGE_WIDTH];
 int         frame_dx       [TOTAL_FRAMES];
 int         frame_dy       [TOTAL_FRAMES];

 // ---- Base coordinate functions, mirror qubit_lookup_axi reset logic ----
 function automatic int calc_qx(int id);
     int col = id % GRID_COLS;  int x = QUBIT_START_X;
     for (int c = 0; c < col; c++) x += (c == 4) ? 52 : 51;
     return x;
 endfunction

 function automatic int calc_qy(int id);
     int row = id / GRID_COLS;  int y = QUBIT_START_Y;
     for (int r = 0; r < row; r++) y += (r == 4) ? 52 : 51;
     return y;
 endfunction

 // Effective coordinate for frame f, including the per-frame offset.
 // The reference model, display tasks and ROI checker all go through
 // get_qx/get_qy so the reprogram phase is handled transparently.
 function automatic int get_qx(int id, int f);
     return calc_qx(id) + frame_dx[f];
 endfunction
 function automatic int get_qy(int id, int f);
     return calc_qy(id) + frame_dy[f];
 endfunction

 function automatic int count_excited(int f);
     int n = 0;
     for (int q = 0; q < NUM_QUBITS; q++) n += int'(qubit_state_gt[f][q]);
     return n;
 endfunction


 function automatic string frame_name(int f);
     case (f)
         0: return "ALL-GROUND";
         1: return "COL-STRIPES";
         2: return "CHECKERBOARD";
         3: return "ALL-RYDBERG";
         4: return "SINGLE-Q0";
         5: return "ALL-ZERO";
         6: return "ROW-STRIPES";
         7: return "NEAR-THRESHOLD";
         8: return "RP-ALL-GROUND";
         9: return "RP-ALL-RYDBERG";
         default: return "UNKNOWN";
     endcase
 endfunction

 // ---- Per-frame qubit-state ground truth ----
 task build_qubit_gt();
     int row, col;
     for (int f = 0; f < TOTAL_FRAMES; f++) begin
         for (int q = 0; q < NUM_QUBITS; q++) begin
             row = q / GRID_COLS;  col = q % GRID_COLS;
             case (f)
                 // ---- Main frames ----
                 0: qubit_state_gt[f][q] = 1'b0;                    // all ground
                 1: qubit_state_gt[f][q] = (col % 2 == 0);          // even-col stripes (50/100)
                 2: qubit_state_gt[f][q] = ((row + col) % 2 == 0);  // checkerboard    (50/100)
                 3: qubit_state_gt[f][q] = 1'b1;                    // all Rydberg
                 4: qubit_state_gt[f][q] = (q == 0);                // single Q0       (1/100)
                 5: qubit_state_gt[f][q] = 1'b0;                    // all-zero frame  (0/100)
                 6: qubit_state_gt[f][q] = (row % 2 == 0);          // even-row stripes (50/100)
                 // Frame 7: even qubits = Rydberg (score 512 > 500),
                 //          odd  qubits = Ground  (score 496 < 500).
                 7: qubit_state_gt[f][q] = (q % 2 == 0);
                 // ---- Reprogram phase ----
                 8: qubit_state_gt[f][q] = 1'b0;                    // reprogrammed, all ground
                 9: qubit_state_gt[f][q] = 1'b1;                    // reprogrammed, all Rydberg
             endcase
         end
     end
 endtask

 // ---- Build a realistic pixel frame ----
 // Normal frames    : pseudo-random background + Gaussian-like blob on Rydberg qubits.
 // Frame 5 (ALL-ZERO)        : all pixels 0; no background, no blobs.
 // Frame 7 (NEAR-THRESHOLD)  : fixed background BG_NEAR_THRESH = 16 everywhere; then
 //                              uniform glow on each qubit's 3x3 ROI:
 //                              Rydberg -> +FL_NEAR_ABOVE = 16  (pixel = 32, score = 512)
 //                              Ground  -> +FL_NEAR_BELOW = 15  (pixel = 31, score = 496)
 // Reprogram frames (8-9)    : same Gaussian-blob model, blobs placed at
 //                              calc_qx + frame_dx, calc_qy + frame_dy.
 task build_frame(int f);
     int qx, qy, py, px, dist_sq, glow, bg, total;
     int dx, dy;
     dx = frame_dx[f];
     dy = frame_dy[f];

     // --- Background ---
     if (f == 5) begin
         // ALL-ZERO: all pixels dark
         for (int y = 0; y < IMAGE_HEIGHT; y++)
             for (int x = 0; x < IMAGE_WIDTH; x++)
                 frame_buf[f][y][x] = 8'd0;
     end else if (f == 7) begin
         // NEAR-THRESHOLD: flat background, no noise
         for (int y = 0; y < IMAGE_HEIGHT; y++)
             for (int x = 0; x < IMAGE_WIDTH; x++)
                 frame_buf[f][y][x] = 8'(BG_NEAR_THRESH);
     end else begin
         // Standard pseudo-random background in [BG_LO, BG_HI].
         // Max Gaussian score from background alone = 28*16 = 448 < 500,
         // so a Rydberg blob is required to push a qubit over threshold.
         for (int y = 0; y < IMAGE_HEIGHT; y++)
             for (int x = 0; x < IMAGE_WIDTH; x++) begin
                 bg = (x * 7919 + y * 3571 + f * 1013) & 32'hFFFF;
                 frame_buf[f][y][x] = 8'(BG_LO + (bg % (BG_HI - BG_LO + 1)));
             end
     end

     // --- Rydberg blobs / glows ---
     for (int q = 0; q < NUM_QUBITS; q++) begin
         qx = calc_qx(q) + dx;
         qy = calc_qy(q) + dy;

         if (f == 7) begin
             // Near-threshold: uniform glow on a 3x3 ROI.
             glow = qubit_state_gt[f][q] ? FL_NEAR_ABOVE : FL_NEAR_BELOW;
             for (int ry = -1; ry <= 1; ry++) begin
                 for (int rx = -1; rx <= 1; rx++) begin
                     py = qy + ry;  px = qx + rx;
                     if (px >= 0 && px < IMAGE_WIDTH && py >= 0 && py < IMAGE_HEIGHT)
                         frame_buf[f][py][px] = 8'(int'(frame_buf[f][py][px]) + glow);
                 end
             end
         end else if (qubit_state_gt[f][q]) begin
             // Gaussian-like blob: 5x5 footprint, intensity drops with
             // squared distance from the centre.
             for (int dy2 = -2; dy2 <= 2; dy2++) begin
                 for (int dx2 = -2; dx2 <= 2; dx2++) begin
                     py = qy + dy2;  px = qx + dx2;
                     if (px >= 0 && px < IMAGE_WIDTH && py >= 0 && py < IMAGE_HEIGHT) begin
                         dist_sq = dx2*dx2 + dy2*dy2;
                         glow    = (dist_sq == 0) ? FL_CENTER :
                                   (dist_sq <= 2) ? FL_ADJACENT : FL_DIAGONAL;
                         total = int'(frame_buf[f][py][px]) + glow;
                         frame_buf[f][py][px] = 8'(total > 255 ? 255 : total);
                     end
                 end
             end
         end
     end
 endtask

 // --------------------------------------------------------------------
 // 6. Reference model
 //    The dark frame uses all-zero pixels, so the noise baseline is 0
 //    and sat_sub(roi, 0) = roi. Existing vectors are therefore
 //    unaffected. If a non-zero dark frame is used in future,
 //    get_expected_roi will need to apply per-pixel saturating
 //    subtraction of the captured baseline.
 // --------------------------------------------------------------------
 // get_expected_roi uses get_qx/get_qy so reprogram-phase frames pick
 // up their shifted coordinates automatically.
 function automatic logic [ROI_BITS-1:0] get_expected_roi(int id, int frame_num);
     int qx = get_qx(id, frame_num);
     int qy = get_qy(id, frame_num);
     return {frame_buf[frame_num][qy  ][qx-1], frame_buf[frame_num][qy  ][qx], frame_buf[frame_num][qy  ][qx+1],
             frame_buf[frame_num][qy-1][qx-1], frame_buf[frame_num][qy-1][qx], frame_buf[frame_num][qy-1][qx+1],
             frame_buf[frame_num][qy-2][qx-1], frame_buf[frame_num][qy-2][qx], frame_buf[frame_num][qy-2][qx+1]};
 endfunction

 function automatic int calc_expected_score(int id, int frame_num);
     logic [ROI_BITS-1:0] roi = get_expected_roi(id, frame_num);
     logic [7:0] p[0:2][0:2];
     {p[0][0],p[0][1],p[0][2],p[1][0],p[1][1],p[1][2],p[2][0],p[2][1],p[2][2]} = roi;
     return p[0][0]*1 + p[0][1]*2 + p[0][2]*1 +
            p[1][0]*2 + p[1][1]*4 + p[1][2]*2 +
            p[2][0]*1 + p[2][1]*2 + p[2][2]*1;
 endfunction

 function automatic logic get_expected_decision(int id, int frame_num);
     return (calc_expected_score(id, frame_num) > GAUSS_THRESH) ? 1'b1 : 1'b0;
 endfunction

 // --------------------------------------------------------------------
 // 7. AXI4-Lite BFM, coordinate LUT + control registers
 //    All registers share the s_axi_coord_* bus (qubit_lookup_axi).
 //    qubit_lookup_axi requires AW before W (s_axi_wready = aw_active,
 //    gated by AW), which is what the BFM below honours.
 // --------------------------------------------------------------------
 task automatic axi_write(input [9:0] addr, input [31:0] data);
     @(posedge i_aclk_ps);
     s_axi_coord_awaddr  <= addr;
     s_axi_coord_awvalid <= 1'b1;
     s_axi_coord_wvalid  <= 1'b0;              // W not valid yet
     @(posedge i_aclk_ps iff s_axi_coord_awready);
     s_axi_coord_awvalid <= 1'b0;
     s_axi_coord_wdata   <= data;
     s_axi_coord_wvalid  <= 1'b1;
     @(posedge i_aclk_ps iff s_axi_coord_wready);
     s_axi_coord_wvalid  <= 1'b0;
     @(posedge i_aclk_ps iff s_axi_coord_bvalid);
 endtask

 task automatic axi_read(input [9:0] addr, output logic [31:0] rdata);
     @(posedge i_aclk_ps);
     s_axi_coord_araddr  <= addr;
     s_axi_coord_arvalid <= 1'b1;
     @(posedge i_aclk_ps iff s_axi_coord_arready);
     s_axi_coord_arvalid <= 1'b0;
     @(posedge i_aclk_ps iff s_axi_coord_rvalid);
     rdata = s_axi_coord_rdata;
 endtask

 // Program all NUM_QUBITS qubit coordinates with an optional dx/dy
 // offset. Always performs a full 100-qubit readback verification.
 // dx/dy = 0 for the initial programming, COORD_SHIFT for the
 // reprogram phase.
 task program_qubit_coords(int dx = 0, int dy = 0);
     logic [31:0] rb_x, rb_y;
     int rb_errors = 0;
     string phase_str;
     phase_str = (dx == 0 && dy == 0) ? "default grid" :
                 $sformatf("SHIFTED grid  dx=+%0d  dy=+%0d", dx, dy);

     $display("[COORD] Programming %0d qubit (X,Y) pairs [%s] via AXI4-Lite @ 300 MHz...",
              NUM_QUBITS, phase_str);
     for (int i = 0; i < NUM_QUBITS; i++) begin
         axi_write(10'(i*8),   32'(calc_qx(i) + dx));
         axi_write(10'(i*8+4), 32'(calc_qy(i) + dy));
     end

     // Full 100-qubit readback verification
     $display("[COORD] Full 100-qubit readback verification...");
     for (int i = 0; i < NUM_QUBITS; i++) begin
         axi_read(10'(i*8),   rb_x);
         axi_read(10'(i*8+4), rb_y);
         if (rb_x !== 32'(calc_qx(i)+dx) || rb_y !== 32'(calc_qy(i)+dy)) begin
             $display("[COORD FAIL] Q%0d  exp X=%0d Y=%0d  got X=%0d Y=%0d",
                      i, calc_qx(i)+dx, calc_qy(i)+dy, rb_x, rb_y);
             rb_errors++;
         end else if (VERBOSE_LIVE)
             $display("[COORD OK] Q%0d  X=%0d  Y=%0d", i, rb_x, rb_y);
     end
     if (rb_errors == 0)
         $display("[COORD] All 100/100 readback checks PASS.\n");
     else
         $display("[COORD] %0d/100 readback ERRORS.\n", rb_errors);

     repeat(10) @(posedge i_clk_pl);
     $display("[COORD] Coordinates settled in 520 MHz PL domain [%s]. Pipeline gate open.\n",
              phase_str);
 endtask

 // ---- Program dark_mode (0x320 bit 0) ----
 task automatic program_dark_mode(int dm);
     logic [31:0] rb;
     axi_write(DARK_MODE_ADDR, 32'(dm));
     axi_read(DARK_MODE_ADDR, rb);
     if (rb[0] !== dm[0])
         $display("[DARK FAIL] dark_mode readback = %0d, expected %0d", rb[0], dm);
     else
         $display("[CTRL] dark_mode = %0d  (readback OK)", dm);
     // Give the 2-FF synchroniser time to propagate to the 520 MHz domain.
     repeat(10) @(posedge i_clk_pl);
 endtask

 // ---- Program gaussian_threshold (0x324 [15:0]) ----
 task automatic program_gauss_threshold(int thresh);
     logic [31:0] rb;
     axi_write(GAUSS_THRESH_ADDR, 32'(thresh));
     axi_read(GAUSS_THRESH_ADDR, rb);
     if (rb[15:0] !== 16'(thresh))
         $display("[THRESH FAIL] gaussian_threshold readback = %0d, expected %0d",
                  rb[15:0], thresh);
     else
         $display("[CTRL] gaussian_threshold = %0d  (readback OK)", thresh);
     repeat(10) @(posedge i_clk_pl);
 endtask

 // --------------------------------------------------------------------
 // 7b. Latency-capture BFM (read-only)
 // --------------------------------------------------------------------
 // latency_capture is a read-only peripheral; the write channel is
 // tied off inside the module.
 task automatic lat_read(input [5:0] addr, output logic [31:0] rdata);
     @(posedge i_aclk_ps);
     s_axi_lat_araddr  <= addr;
     s_axi_lat_arvalid <= 1'b1;
     @(posedge i_aclk_ps iff s_axi_lat_arready);
     s_axi_lat_arvalid <= 1'b0;
     @(posedge i_aclk_ps iff s_axi_lat_rvalid);
     rdata = s_axi_lat_rdata;
 endtask

 // --------------------------------------------------------------------
 // 7c. Per-frame latency report via register readout
 // --------------------------------------------------------------------
 // Reads all 15 latency-capture registers via AXI4-Lite and prints a
 // formatted cycle-accurate breakdown of the PL pipeline for the
 // completed frame.
 //
 // Call timing: after frame_done, between frames. Timestamps are
 // quasi-static in this window, so reading without coherency tricks
 // is safe.
 //
 // Conversion: cycles * PL_CLK_PERIOD = time in ns.
 //             cycles * PL_CLK_PERIOD / 1000 = time in us.
 task automatic read_latency_report(int fn);
     logic [31:0] r_counter, r_frame_seq;
     logic [31:0] r_ts_sof, r_ts_match, r_ts_roi_wr, r_ts_ready;
     logic [31:0] r_ts_first_r, r_ts_last_r, r_ts_fdone;
     logic [31:0] r_d_pipe, r_d_match, r_d_roi, r_d_storage, r_d_readout, r_d_filter;

     // Snapshot all 15 registers (~45 PS cycles total).
     lat_read(LAT_COUNTER,     r_counter);
     lat_read(LAT_FRAME_SEQ,   r_frame_seq);
     lat_read(LAT_TS_SOF,      r_ts_sof);
     lat_read(LAT_TS_MATCH,    r_ts_match);
     lat_read(LAT_TS_ROI_WR,   r_ts_roi_wr);
     lat_read(LAT_TS_READY,    r_ts_ready);
     lat_read(LAT_TS_FIRST_R,  r_ts_first_r);
     lat_read(LAT_TS_LAST_R,   r_ts_last_r);
     lat_read(LAT_TS_FDONE,    r_ts_fdone);
     lat_read(LAT_D_PIPE,      r_d_pipe);
     lat_read(LAT_D_MATCH,     r_d_match);
     lat_read(LAT_D_ROI,       r_d_roi);
     lat_read(LAT_D_STORAGE,   r_d_storage);
     lat_read(LAT_D_READOUT,   r_d_readout);
     lat_read(LAT_D_FILTER,    r_d_filter);

     $display("");
     $display("  +------------------------------------------------------------------+");
     $display("  | LATENCY CAPTURE Frame %0d (%s)  seq=%0d", fn, frame_name(fn), r_frame_seq);
     $display("  | Clock: 520 MHz (%0.3f ns/tick)   Counter: %0d", PL_CLK_PERIOD, r_counter);
     $display("  +------------------------------------------------------------------+");
     $display("  |  TIMESTAMPS (absolute cycle count @ 520 MHz)                     |");
     $display("  |    ts_sof_in       = %10d  (PL entry: first pixel + tuser)  |", r_ts_sof);
     $display("  |    ts_first_match  = %10d  (coord_matcher first hit)        |", r_ts_match);
     $display("  |    ts_first_roi_wr = %10d  (roi_extractor first write)      |", r_ts_roi_wr);
     $display("  |    ts_frame_done   = %10d  (pixel_injector frame_done)      |", r_ts_fdone);
     $display("  |    ts_frame_ready  = %10d  (roi_storage ping-pong swap)     |", r_ts_ready);
     $display("  |    ts_first_result = %10d  (gaussian bank first valid)      |", r_ts_first_r);
     $display("  |    ts_last_result  = %10d  (gaussian bank last valid+TLAST) |", r_ts_last_r);
     $display("  +------------------------------------------------------------------+");
     $display("  |  DELTAS (cycles)        cycles    time (ns)                      |");
     $display("  |    SOF -> last_result  %8d  %10.1f  [total PL pipe]      |",
              r_d_pipe,   real'(r_d_pipe)   * PL_CLK_PERIOD);
     $display("  |    SOF -> first_match  %8d  %10.1f                       |",
              r_d_match,  real'(r_d_match)  * PL_CLK_PERIOD);
     $display("  |    match -> ROI write  %8d  %10.1f  [extraction]         |",
              r_d_roi,    real'(r_d_roi)    * PL_CLK_PERIOD);
     $display("  |    fdone -> frm_ready  %8d  %10.1f  [storage]            |",
              r_d_storage,real'(r_d_storage)* PL_CLK_PERIOD);
     $display("  |    frm_ready -> last_r %8d  %10.1f  [readout+subtract]   |",
              r_d_readout,real'(r_d_readout)* PL_CLK_PERIOD);
     $display("  |    first_r -> last_r   %8d  %10.1f  [filter bank]        |",
              r_d_filter, real'(r_d_filter) * PL_CLK_PERIOD);
     $display("  +------------------------------------------------------------------+");
     $display("");
 endtask

 // --------------------------------------------------------------------
 // 8. Pixel frame injection BFM
 // --------------------------------------------------------------------
 task automatic inject_frame(input int frame_num);
     logic [7:0] p0, p1, p2, p3, p4, p5, p6, p7;
     $display("[PIXEL] Frame %0d -- %0d/%0d Rydberg | %0d/%0d Ground | pattern: %s | grid-offset: dx=%0d dy=%0d",
         frame_num, count_excited(frame_num), NUM_QUBITS,
         NUM_QUBITS - count_excited(frame_num), NUM_QUBITS,
         frame_name(frame_num), frame_dx[frame_num], frame_dy[frame_num]);

     for (int row = 0; row < IMAGE_HEIGHT; row++) begin
         for (int beat = 0; beat < BEATS_PER_LINE; beat++) begin
             p0 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 0];
             p1 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 1];
             p2 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 2];
             p3 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 3];
             p4 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 4];
             p5 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 5];
             p6 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 6];
             p7 = frame_buf[frame_num][row][beat*PIXELS_PER_BEAT + 7];
             @(posedge i_aclk_ps iff s_axis_pix_tready);
             // [63:56]=p7 [55:48]=p6 [47:40]=p5 [39:32]=p4
             // [31:24]=p3 [23:16]=p2 [15:8]=p1  [7:0]=p0
             s_axis_pix_tdata  <= {p7, p6, p5, p4, p3, p2, p1, p0};
             s_axis_pix_tvalid <= 1'b1;
             s_axis_pix_tuser  <= ((row == 0) && (beat == 0)) ? 1'b1 : 1'b0;
             s_axis_pix_tlast  <= (beat == BEATS_PER_LINE - 1) ? 1'b1 : 1'b0;
         end
     end
     @(posedge i_aclk_ps);
     s_axis_pix_tvalid <= 1'b0;
     s_axis_pix_tuser  <= 1'b0;
     s_axis_pix_tlast  <= 1'b0;
     $display("[PIXEL] Frame %0d injection complete.\n", frame_num);
 endtask

 // --------------------------------------------------------------------
 // 9. Capture arrays and hierarchical probes
 // --------------------------------------------------------------------
 logic [ROI_BITS-1:0] qubit_roi_cap  [0:NUM_QUBITS-1];
 logic [15:0]         qubit_score_cap[0:NUM_QUBITS-1];
 logic                qubit_dec_cap  [0:NUM_QUBITS-1];
 logic                qubit_roi_ok   [0:NUM_QUBITS-1];
 logic                qubit_dec_ok   [0:NUM_QUBITS-1];

 // Hierarchical probes -- subtracted lane data from read_streamer
 logic [ROI_BITS-1:0]       lane_0, lane_1, lane_2, lane_3;
 logic [QUBIT_ID_WIDTH-1:0] lane_base_id;
 logic                      lane_valid;

 assign lane_0       = DUT.u_stream.o_pixeldata_lane_0;
 assign lane_1       = DUT.u_stream.o_pixeldata_lane_1;
 assign lane_2       = DUT.u_stream.o_pixeldata_lane_2;
 assign lane_3       = DUT.u_stream.o_pixeldata_lane_3;
 assign lane_base_id = DUT.u_stream.o_pixeldata_lane_base_id;
 assign lane_valid   = DUT.u_stream.o_pixeldata_lane_valid;

 // Gaussian engine scores: hierarchical probes. o_score is left
 // unconnected at the parent but driven internally.
 wire [15:0] eng_score_0 = DUT.gen_filter[0].u_eng.o_score;
 wire [15:0] eng_score_1 = DUT.gen_filter[1].u_eng.o_score;
 wire [15:0] eng_score_2 = DUT.gen_filter[2].u_eng.o_score;
 wire [15:0] eng_score_3 = DUT.gen_filter[3].u_eng.o_score;

 // Internal 520 MHz PL-domain signals for score capture before the
 // output CDC FIFO.
 wire [3:0]                pl_state_i   = DUT.pl_qubit_state;
 wire [QUBIT_ID_WIDTH-1:0] pl_base_id_i = DUT.pl_qubit_base_id;
 wire                      pl_valid_i   = DUT.pl_qubit_valid;

 // --------------------------------------------------------------------
 // 10. Scoreboard counters
 // --------------------------------------------------------------------
 int error_cnt = 0, roi_match_cnt = 0;
 int gauss_match_cnt = 0, gauss_error_cnt = 0;
 int axi_beat_cnt = 0;
 int current_proc_frame = -1, check_frame = 0;
 int bp_events = 0;

 // Reprogram-phase counters, kept separate so the final report can
 // split main-phase results from reprogram-phase results.
 int rp_roi_match = 0, rp_roi_error = 0;
 int rp_dec_match = 0, rp_dec_error = 0;

 // $realtime timestamp for frame_ready, used only for the printed
 // frame sequence header. All latency numbers come from the AXI
 // register readback.
 realtime frame_ready_t = 0;

 // Dark-mode spurious-output counter.
 int dark_mode_output_cnt = 0;

 // --------------------------------------------------------------------
 // 11. Verification always_ff blocks (520 MHz PL domain)
 // --------------------------------------------------------------------

 // ---- Track frame_ready rising edge ----
 // check_frame advances 0,1,...,9 as frames complete. For frames 8-9,
 // frame_dx[check_frame] = COORD_SHIFT, so get_qx/get_qy and
 // get_expected_roi pick up the shifted reference grid automatically.
 always @(posedge i_clk_pl) begin
     if (DUT.frame_ready && !$past(DUT.frame_ready)) begin
         frame_ready_t = $realtime;
         current_proc_frame++;
         check_frame = current_proc_frame;
         $display("[FRAME_READY] Frame %0d (%s) ready @ %.2f us",
                  current_proc_frame, frame_name(current_proc_frame), $realtime/1e3);
     end
 end

 // ---- ROI capture and checker ----
 always @(posedge i_clk_pl) begin
     if (lane_valid) begin
         for (int k = 0; k < NUM_BANKS; k++) begin
             automatic int cid = int'(lane_base_id) + k;
             if (cid < NUM_QUBITS) begin
                 automatic logic [ROI_BITS-1:0] exp_roi = get_expected_roi(cid, check_frame);
                 automatic logic [ROI_BITS-1:0] act_roi;
                 case (k)
                     0: act_roi = lane_0;  1: act_roi = lane_1;
                     2: act_roi = lane_2;  default: act_roi = lane_3;
                 endcase
                 qubit_roi_cap[cid] = act_roi;
                 if (act_roi !== exp_roi) begin
                     error_cnt++;
                     qubit_roi_ok[cid] = 1'b0;
                     $display("[ROI FAIL] Frame %0d Q%0d @ (%0d,%0d)  exp[71:64]=%02h act=%02h",
                              check_frame, cid,
                              get_qx(cid, check_frame), get_qy(cid, check_frame),
                              exp_roi[71:64], act_roi[71:64]);
                 end else begin
                     roi_match_cnt++;
                     qubit_roi_ok[cid] = 1'b1;
                     if (VERBOSE_LIVE)
                         $display("[ROI OK] Frame %0d Q%0d", check_frame, cid);
                 end
             end
         end
     end
 end

 // ---- 520 MHz: score capture ----
 always @(posedge i_clk_pl) begin
     if (pl_valid_i) begin
         for (int k = 0; k < NUM_BANKS; k++) begin
             automatic int qid = int'(pl_base_id_i) + k;
             if (qid < NUM_QUBITS) begin
                 case (k)
                     0: qubit_score_cap[qid] = eng_score_0;
                     1: qubit_score_cap[qid] = eng_score_1;
                     2: qubit_score_cap[qid] = eng_score_2;
                     3: qubit_score_cap[qid] = eng_score_3;
                 endcase
             end
         end
     end
 end

 // --------------------------------------------------------------------
 // 12. AXI-S output monitor and decision verifier (300 MHz PS domain)
 // --------------------------------------------------------------------
 // Latency reporting goes through read_latency_report() after each
 // frame completes, which is deterministic cycle-count based and
 // matches what PS software would see via devmem/mmap.
 always @(posedge i_aclk_ps) begin
     if (s_axis_pix_tvalid && !s_axis_pix_tready)
         bp_events++;

     if (m_axis_qubit_tvalid && m_axis_qubit_tready) begin
         // Defensive: if dark_mode is somehow still on, count any
         // spurious outputs so the final report flags them.
         if (DUT.dark_mode_pl) begin
             dark_mode_output_cnt++;
             $display("[DARK ERROR] Qubit output during dark_mode! (beat %0d)", axi_beat_cnt+1);
         end

         axi_beat_cnt++;
         begin : verify_beat
             automatic logic [QUBIT_ID_WIDTH-1:0] bid = m_axis_qubit_tdata[10:4];
             automatic logic [3:0]                 st  = m_axis_qubit_tdata[3:0];
             for (int k = 0; k < NUM_BANKS; k++) begin
                 automatic int  qid     = int'(bid) + k;
                 automatic logic exp_dec = get_expected_decision(qid, check_frame);
                 automatic logic act_dec = st[k];
                 if (qid < NUM_QUBITS) begin
                     qubit_dec_cap[qid] = act_dec;
                     if (act_dec !== exp_dec) begin
                         gauss_error_cnt++;
                         qubit_dec_ok[qid] = 1'b0;
                         $display("[DEC FAIL] F%0d Q%0d  score=%0d  exp=%0b  act=%0b",
                                  check_frame, qid, qubit_score_cap[qid], exp_dec, act_dec);
                     end else begin
                         gauss_match_cnt++;
                         qubit_dec_ok[qid] = 1'b1;
                     end
                 end
             end
         end

         if (m_axis_qubit_tlast) begin
             $display("[AXI-S OUT] Last beat #%0d  base_id=%0d  @ %.2f us",
                      axi_beat_cnt, m_axis_qubit_tdata[10:4], $realtime/1e3);

         end
     end
 end

 // --------------------------------------------------------------------
 // 13. Display tasks
 // --------------------------------------------------------------------
 task print_roi_grid(int qid, int fn);
     automatic logic [ROI_BITS-1:0] roi = qubit_roi_cap[qid];
     automatic logic [7:0] p[0:2][0:2];
     {p[0][0],p[0][1],p[0][2],p[1][0],p[1][1],p[1][2],p[2][0],p[2][1],p[2][2]} = roi;
     $display("  +-- Q%3d (%3d,%3d) Score=%4d [%s] ROI=%s DEC=%s --+",
              qid, get_qx(qid, fn), get_qy(qid, fn), qubit_score_cap[qid],
              qubit_dec_cap[qid] ? "RYDBERG" : "GROUND ",
              qubit_roi_ok[qid]  ? "OK  " : "FAIL",
              qubit_dec_ok[qid]  ? "OK  " : "FAIL");
     $display("  |  y=%3d  | %3d | %3d | %3d |  (top  qy  )", get_qy(qid,fn),   p[0][0],p[0][1],p[0][2]);
     $display("  |  y=%3d  | %3d | %3d | %3d |  (mid  qy-1)", get_qy(qid,fn)-1, p[1][0],p[1][1],p[1][2]);
     $display("  |  y=%3d  | %3d | %3d | %3d |  (btm  qy-2)", get_qy(qid,fn)-2, p[2][0],p[2][1],p[2][2]);
     $display("  +-----------------------------------------------------------+");
 endtask

 task automatic print_qubit_map(int fn);
     int excited = 0;
     string line, hdr_note;
     hdr_note = (fn >= NUM_MAIN_FRAMES) ?
                $sformatf(" [REPROGRAM  dx=+%0d dy=+%0d]", frame_dx[fn], frame_dy[fn]) : "";
     $display("\n  +-- QUBIT STATE MAP frame %0d  (%s)%s --+",
              fn, frame_name(fn), hdr_note);
     $display("  |   col:  0   1   2   3   4   5   6   7   8   9              |");
     $display("  +-------------------------------------------------------------+");
     for (int row = 0; row < GRID_ROWS; row++) begin
         line = $sformatf("  | row%2d: ", row);
         for (int col = 0; col < GRID_COLS; col++) begin
             automatic int qid = row * GRID_COLS + col;
             automatic string sym;
             if (!qubit_dec_ok[qid])           sym = " ! ";   // mismatch
             else if (qubit_dec_cap[qid])       sym = " R ";   // Rydberg
             else                               sym = " . ";   // Ground
             line = {line, sym};
         end
         $display("%s  |", line);
     end
     $display("  +-------------------------------------------------------------+");
     for (int q = 0; q < NUM_QUBITS; q++) excited += int'(qubit_dec_cap[q]);
     $display("  | Rydberg detected: %3d / %3d    Expected: %3d / %3d            |",
              excited, NUM_QUBITS, count_excited(fn), NUM_QUBITS);
     $display("  +-------------------------------------------------------------+\n");
 endtask

 task automatic print_frame_report(int fn);
     int roi_pass = 0, dec_pass = 0;
     string extra_note;
     case (fn)
         4: extra_note = "  [single-qubit: verifies no phantom matches]";
         5: extra_note = "  [all-zero: all scores=0, all decisions=Ground]";
         7: extra_note = $sformatf("  [near-threshold: even=%0d>%0d Rydberg, odd=%0d<%0d Ground]",
                BG_NEAR_THRESH+FL_NEAR_ABOVE, GAUSS_THRESH, BG_NEAR_THRESH+FL_NEAR_BELOW, GAUSS_THRESH);
         8, 9: extra_note = $sformatf("  [reprogram: grid shifted +%0d,+%0d]",
                frame_dx[fn], frame_dy[fn]);
         default: extra_note = "";
     endcase

     $display("\n----------------------------------------------------------");
     $display("## FRAME %0d REPORT  pattern=%-14s  excited=%3d/%3d%s",
              fn, frame_name(fn), count_excited(fn), NUM_QUBITS, extra_note);
     $display("----------------------------------------------------------");

     $display("\n--- 3x3 ROI grids (mismatches + every 10th qubit) ---\n");
     for (int qid = 0; qid < NUM_QUBITS; qid++) begin
         if (!qubit_roi_ok[qid] || !qubit_dec_ok[qid] || (qid % 10 == 0))
             print_roi_grid(qid, fn);
         roi_pass += int'(qubit_roi_ok[qid]);
         dec_pass += int'(qubit_dec_ok[qid]);
     end

     print_qubit_map(fn);

     $display("  ROI  match : %3d / %3d  %s", roi_pass, NUM_QUBITS,
              (roi_pass == NUM_QUBITS) ? "[ALL PASS]" : "[*** FAIL ***]");
     $display("  DEC  match : %3d / %3d  %s", dec_pass, NUM_QUBITS,
              (dec_pass == NUM_QUBITS) ? "[ALL PASS]" : "[*** FAIL ***]");
     $display("-----------------------------------------------------------\n");
 endtask

 // --------------------------------------------------------------------
 // 14. Main stimulus
 // --------------------------------------------------------------------
 initial begin

     // Per-frame coord offsets: zero for the main loop, COORD_SHIFT
     // for the reprogram phase.
     for (int i = 0; i < NUM_MAIN_FRAMES; i++) begin
         frame_dx[i] = 0;  frame_dy[i] = 0;
     end
     frame_dx[8] = COORD_SHIFT;  frame_dy[8] = COORD_SHIFT;
     frame_dx[9] = COORD_SHIFT;  frame_dy[9] = COORD_SHIFT;

     // Pre-compute ground truth and all pixel frames.
     for (int qi = 0; qi < NUM_QUBITS; qi++) begin
         qubit_roi_cap[qi] = '0;  qubit_score_cap[qi] = '0;  qubit_dec_cap[qi] = '0;
         qubit_roi_ok[qi]  = '0;  qubit_dec_ok[qi]    = '0;
     end
     build_qubit_gt();
     for (int f = 0; f < TOTAL_FRAMES; f++) build_frame(f);

     $display("\n||----------------------------------------------------------||\n");
     $display("||   QUBIT READOUT PIPELINE TESTBENCH  (Dark Mode + Thresh)  ||\n");
     $display("||   PS: 300 MHz  |  PL: 520 MHz  |  CDC: async FIFO + 2FF  ||\n");
     $display("||   Image: %0dx%0d  |  Qubits: %0d  |  Frames: %0d main + %0d RP ||\n",
              IMAGE_WIDTH, IMAGE_HEIGHT, NUM_QUBITS, NUM_MAIN_FRAMES, NUM_RP_FRAMES);
     $display("||   Control regs: 0x320=dark_mode  0x324=gaussian_threshold ||\n");
     $display("||   Dark frame: ALL-ZERO (noise=0, subtraction is no-op)    ||\n");
     $display("||----------------------------------------------------------||");

     // Reset
     i_aresetn_ps = 0;
     repeat(25) @(posedge i_aclk_ps);
     i_aresetn_ps = 1;
     repeat(10) @(posedge i_aclk_ps);

     // Program qubit coordinates (default grid).
     program_qubit_coords(0, 0);

     // Program Gaussian threshold (software-controllable, default 500).
     program_gauss_threshold(GAUSS_THRESH);

     // ------------------------------------------------------------------
     // Dark-mode capture phase.
     // Inject an all-zero frame with dark_mode = 1. ROIs land in the
     // base_noise banks; frame_ready is suppressed so the Gaussian and
     // output paths stay silent. dark_mode_output_cnt is checked at
     // the end to confirm there were zero spurious qubit outputs.
     // Uses frame_buf[5] (ALL-ZERO) so no extra storage is needed.
     // ------------------------------------------------------------------
     $display("\n||----------------------------------------------------------||\n");
     $display("||   DARK MODE CAPTURE PHASE                                  ||\n");
     $display("||   Setting dark_mode=1, injecting ALL-ZERO calibration frame||\n");
     $display("||   Expect: NO m_axis_qubit output during dark capture        ||\n");
     $display("||----------------------------------------------------------||");

     program_dark_mode(1);

     // Inject dark frame (reuses the ALL-ZERO frame buffer).
     inject_frame(5);

     // Wait for the last ROI to land in the base_noise banks
     // (pipeline-depth cycles after the last pixel beat).
     repeat(2000) @(posedge i_clk_pl);

     $display("[DARK] Capture complete. dark_mode_output_cnt = %0d (expect 0)",
              dark_mode_output_cnt);
     if (dark_mode_output_cnt != 0)
         $display("[DARK FAIL] Spurious qubit outputs during dark capture!");
     else
         $display("[DARK PASS] No qubit outputs during dark capture.\n");

     program_dark_mode(0);

     $display("[DARK] dark_mode=0 set. Normal pipeline resuming.\n");

     // ------------------------------------------------------------------
     // Main frame loop (frames 0-7).
     // ------------------------------------------------------------------
     for (int f = 0; f < NUM_MAIN_FRAMES; f++) begin

         for (int qi = 0; qi < NUM_QUBITS; qi++) begin
             qubit_roi_ok[qi] = 1'b0;  qubit_dec_ok[qi] = 1'b0;
         end

         // Backpressure test on frame 2.
         if (f == 2) begin
             fork
                 begin : bp_injector
                     @(posedge i_aclk_ps iff (m_axis_qubit_tvalid && m_axis_qubit_tready));
                     @(posedge i_aclk_ps iff (m_axis_qubit_tvalid && m_axis_qubit_tready));
                     m_axis_qubit_tready <= 1'b0;
                     $display("[BP TEST] Frame 2: backpressure asserted @ %.2f us...",
                              $realtime/1e3);
                     repeat(5) @(posedge i_aclk_ps);
                     m_axis_qubit_tready <= 1'b1;
                     $display("[BP TEST] Backpressure released.");
                 end
             join_none
         end

         inject_frame(f);
         @(posedge i_aclk_ps iff (m_axis_qubit_tvalid && m_axis_qubit_tlast));
         $display("[SYNC] Frame %0d last qubit beat @ %.2f us", f, $realtime/1e3);
         repeat(200) @(posedge i_clk_pl);

         read_latency_report(f);

         print_frame_report(f);
     end

     // ------------------------------------------------------------------
     // Coord reprogramming phase (frames 8-9).
     // ------------------------------------------------------------------
     $display("\n||----------------------------------------------------------||\n");
     $display("||   COORD REPROGRAMMING PHASE  (grid +%0d,+%0d)               ||\n",
              COORD_SHIFT, COORD_SHIFT);
     $display("||----------------------------------------------------------||");

     program_qubit_coords(COORD_SHIFT, COORD_SHIFT);

     for (int f = NUM_MAIN_FRAMES; f < TOTAL_FRAMES; f++) begin

         for (int qi = 0; qi < NUM_QUBITS; qi++) begin
             qubit_roi_ok[qi] = 1'b0;  qubit_dec_ok[qi] = 1'b0;
         end

         inject_frame(f);
         @(posedge i_aclk_ps iff (m_axis_qubit_tvalid && m_axis_qubit_tlast));
         $display("[SYNC] Frame %0d last qubit beat @ %.2f us", f, $realtime/1e3);
         repeat(200) @(posedge i_clk_pl);

         for (int qi = 0; qi < NUM_QUBITS; qi++) begin
             rp_roi_match += int'( qubit_roi_ok[qi]);
             rp_roi_error += int'(!qubit_roi_ok[qi]);
             rp_dec_match += int'( qubit_dec_ok[qi]);
             rp_dec_error += int'(!qubit_dec_ok[qi]);
         end

         read_latency_report(f);

         print_frame_report(f);
     end

     repeat(100) @(posedge i_aclk_ps);

     // ------------------------------------------------------------------
     // Final report
     // ------------------------------------------------------------------
     $display("\n||----------------------------------------------------------||\n");
     $display("||              FINAL SIMULATION REPORT                      ||\n");
     $display("||----------------------------------------------------------||");
     $display("||  Total frames: %2d  (%0d main + %0d reprogram)               ||\n",
              TOTAL_FRAMES, NUM_MAIN_FRAMES, NUM_RP_FRAMES);
     $display("||  DARK MODE PHASE:                                          ||\n");
     $display("||    Spurious qubit outputs: %3d  (expect 0)                 ||\n",
              dark_mode_output_cnt);
     $display("||  MAIN LOOP  frames 0-%0d:                                   ||\n",
              NUM_MAIN_FRAMES-1);
     $display("||    ROI  matches : %4d / %4d                             ||\n",
              roi_match_cnt  - rp_roi_match, NUM_MAIN_FRAMES * NUM_QUBITS);
     $display("||    DEC  matches : %4d / %4d                             ||\n",
              gauss_match_cnt - rp_dec_match, NUM_MAIN_FRAMES * NUM_QUBITS);
     $display("||    ROI  errors  : %4d                                    ||\n",
              error_cnt       - rp_roi_error);
     $display("||    DEC  errors  : %4d                                    ||\n",
              gauss_error_cnt - rp_dec_error);
     $display("||  REPROGRAM PHASE  frames %0d-%0d  (grid +%0d,+%0d):          ||\n",
              NUM_MAIN_FRAMES, TOTAL_FRAMES-1, COORD_SHIFT, COORD_SHIFT);
     $display("||    ROI  matches : %4d / %4d                             ||\n",
              rp_roi_match, NUM_RP_FRAMES * NUM_QUBITS);
     $display("||    DEC  matches : %4d / %4d                             ||\n",
              rp_dec_match, NUM_RP_FRAMES * NUM_QUBITS);
     $display("||    ROI  errors  : %4d  (0 = blobs at new coords)         ||\n", rp_roi_error);
     $display("||    DEC  errors  : %4d  (0 = decisions follow AXI data)   ||\n", rp_dec_error);
     $display("||  AXI-S beats out: %4d  (expected %0d)                    ||\n",
              axi_beat_cnt, TOTAL_FRAMES * (NUM_QUBITS / NUM_BANKS));
     $display("||  Input BP events: %4d cycles                             ||\n", bp_events);
     $display("||----------------------------------------------------------||");

     if (error_cnt == 0 && gauss_error_cnt == 0 && dark_mode_output_cnt == 0)
         $display("||              *** ALL TESTS PASSED ***                     ||\n");
     else
         $display("||           *** %0d TESTS FAILED ***                         ||\n",
                  error_cnt + gauss_error_cnt + dark_mode_output_cnt);
     $display("||----------------------------------------------------------||");
     $finish;
 end

 // --------------------------------------------------------------------
 // 15. Watchdog
 // --------------------------------------------------------------------
 initial begin
     // +1 frame budget for the dark-capture phase
     #((TOTAL_FRAMES + 1) * IMAGE_HEIGHT * BEATS_PER_LINE * PS_CLK_PERIOD * 6 + 2_000_000);
     $display("[WATCHDOG] Timeout -- simulation did not complete");
     $finish;
 end

 final begin
     $display("[STATS] Input FIFO backpressure events: %0d", bp_events);
     $display("[STATS] Dark mode spurious outputs: %0d", dark_mode_output_cnt);
 end

 endmodule
