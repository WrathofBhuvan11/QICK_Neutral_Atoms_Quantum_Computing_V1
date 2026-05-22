// ----------------------------------------------------------------------
// params_pkg.sv
// ----------------------------------------------------------------------
// Parameters and type aliases shared by the qubit readout pipeline.
//
// Image geometry, qubit grid layout, ROI window size, storage banking,
// and the default Gaussian threshold all live here so the rest of the
// RTL stays generic. Derived parameters are pre-computed (no $clog2 in
// downstream modules) to keep tool elaboration predictable.
// ----------------------------------------------------------------------

package params_pkg;

    // -------------------------------------------------------
    // Primary parameters
    // -------------------------------------------------------
    parameter int IMAGE_WIDTH    = 512;
    parameter int IMAGE_HEIGHT   = 512;
    parameter int PIXEL_DEPTH    = 8;
    parameter int NUM_QUBITS     = 100;
    parameter int GRID_COLS      = 10;
    parameter int GRID_ROWS      = 10;
    parameter int QUBIT_START_X  = 26;
    parameter int QUBIT_START_Y  = 27;
    parameter int QUBIT_SPACING  = 51;
    parameter int NUM_BANKS      = 4;
    parameter int ROI_SIZE       = 3;

    // -------------------------------------------------------
    // Derived parameters
    // -------------------------------------------------------
    parameter int COORD_WIDTH     = 9;   // ceil(log2(max(IMAGE_WIDTH, IMAGE_HEIGHT))) = ceil(log2(512))
    parameter int QUBIT_ID_WIDTH  = 7;   // ceil(log2(NUM_QUBITS))                     = ceil(log2(100))
    parameter int ROWS_PER_BANK   = 25;  // (NUM_QUBITS + NUM_BANKS - 1) / NUM_BANKS
    parameter int BANK_DEPTH      = 32;  // next power of 2 >= ROWS_PER_BANK
    parameter int BANK_ADDR_WIDTH = 5;   // ceil(log2(BANK_DEPTH))
    parameter int ROW_COUNT_WIDTH = 5;   // ceil(log2(ROWS_PER_BANK))
    parameter int ROI_BITS        = 72;  // ROI_SIZE * ROI_SIZE * PIXEL_DEPTH = 3 * 3 * 8

    // -------------------------------------------------------
    // Pixel-stream geometry (input AXI4-Stream from VDMA)
    // -------------------------------------------------------
    // The VDMA delivers PIXELS_PER_BEAT 8-bit pixels per beat. This was 4
    // (32-bit beat) in the v1 design; it is now 8 (64-bit beat), which halves
    // the beats-per-line (128 -> 64) and the frame transfer time (~218 us ->
    // ~109 us). Everything downstream of roi_extractor is unaffected -- the
    // ROI is always 3x3 = ROI_BITS regardless of the beat width.
    parameter int PIXELS_PER_BEAT    = 8;                            // px per beat
    parameter int BEAT_BITS          = PIXELS_PER_BEAT * PIXEL_DEPTH; // 64-bit AXIS tdata
    parameter int MATCH_OFFSET_WIDTH = 3;   // ceil(log2(PIXELS_PER_BEAT)) -- pixel slot in a beat

    // -------------------------------------------------------
    // Software-controllable defaults
    // -------------------------------------------------------
    // Reset value for the Gaussian decision threshold. Software can
    // override this at runtime via qubit_lookup_axi register 0x324.
    parameter int GAUSS_THRESHOLD_DEFAULT = 500;

    // Type aliases
    typedef logic [COORD_WIDTH-1:0]    coord_t;
    typedef logic [QUBIT_ID_WIDTH-1:0] qubit_id_t;

endpackage
