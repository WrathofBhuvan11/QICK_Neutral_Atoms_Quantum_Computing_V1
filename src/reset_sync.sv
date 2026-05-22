// ----------------------------------------------------------------------
// reset_sync.sv -- async-assert, sync-deassert reset synchroniser
// ----------------------------------------------------------------------
// Produces an active-low reset (rst_n_dst) in the clk_dst domain from
// an asynchronous active-low source (rst_n_src).
//
//   - Assertion is asynchronous: rst_n_dst goes low the instant
//     rst_n_src goes low.
//   - Deassertion is synchronous: rst_n_dst returns high only after
//     STAGES rising edges of clk_dst with rst_n_src held high.
//
// Three FFs gives adequate MTBF for high-frequency targets (520 MHz).
//
// Usage:
//   reset_sync u_rst_sync (
//       .clk_dst   (clk_520mhz),
//       .rst_n_src (aresetn_300mhz),   // from PS proc_sys_reset
//       .rst_n_dst (rst_n_520mhz)      // safe to use in 520 MHz domain
//   );
// ----------------------------------------------------------------------

`timescale 1ns / 1ps

module reset_sync #(
    parameter int STAGES = 3   // 3 stages for high-frequency targets
)(
    input  logic clk_dst,      // destination clock
    input  logic rst_n_src,    // asynchronous active-low reset source
    output logic rst_n_dst     // synchronised active-low reset
);

    // ASYNC_REG instructs Vivado to place the chain FFs in the same
    // slice for best MTBF. KEEP prevents the optimiser from absorbing
    // the chain into surrounding logic.
    (* ASYNC_REG = "TRUE" *) (* KEEP = "TRUE" *) logic [STAGES-1:0] sync_chain;

    always_ff @(posedge clk_dst or negedge rst_n_src) begin
        if (!rst_n_src)
            sync_chain <= '0;                              // async assert
        else
            sync_chain <= {sync_chain[STAGES-2:0], 1'b1};  // sync deassert
    end

    assign rst_n_dst = sync_chain[STAGES-1];

endmodule
