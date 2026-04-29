// ============================================================================
// Module: ascon_STATE_REGISTER  (OPT v5 — fix: latch state_in thay vì init_state)
//
// FIX vs v4:
//   BUG: always block latch `init_state` thay vì `state_in`
//        → dp_state và perm_state không bao giờ được lưu vào state_out
//        → toàn bộ AD/PT/Final phase đều sai
//   FIX: đổi `init_state` → `state_in` trong always block
//        CORE chịu trách nhiệm mux state_next_final → state_in
// ============================================================================
module ascon_STATE_REGISTER (
    input  wire         clk,
    input  wire         rst_n,

    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [1:0]   src_sel,    // legacy port, not used (mux done in CORE)
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire         load,

    input  wire [319:0] state_in,   // already muxed in CORE (state_next_final)

    // Legacy ports — kept for interface compatibility
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [319:0] init_state,
    input  wire [319:0] dp_state,
    input  wire [319:0] perm_state,
    /* verilator lint_on UNUSEDSIGNAL */

    output reg  [319:0] state_out
);
    // FIX: dùng state_in (= state_next_final từ CORE) thay vì init_state
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state_out <= 320'b0;
        else if (load) state_out <= state_in;  // FIX: was `init_state`
    end

endmodule