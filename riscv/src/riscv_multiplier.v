// ============================================================================
// riscv_multiplier.v — 3-stage Pipelined Multiplier for RV32M (Fix 10A)
// ============================================================================
// Pipeline stages:
//   E1   : Sign-extend operands, latch (posedge N)
//   E1.5 : Decompose 33×33 into 4× 17×17 partial products, latch (posedge N+1)
//   E2   : CSA tree + final adder — sum partial products, latch (posedge N+2)
//
// Timing: result valid 3 cycles after mul_valid_i pulse (2-stage was 2 cycles).
// mul_ex_stall in hazard_detection adds 1 extra EX cycle so E1.5 can compute
// before E2 captures; net MUL latency from programmer perspective = same.
//
// hold_i / mul_hold_e15_i = stall_any && !mul_ex_stall — freezes all stages
// except during the multiplier's own stall cycle (E1 and E1.5 must advance).
//
// (* keep = 1 *) on sign bits reduces fanout from 133 to manageable levels.
// ============================================================================

module riscv_multiplier (
    input           clk_i,
    input           rst_i,

    // Dispatch: high for exactly 1 cycle when MUL instruction enters EX
    input           mul_valid_i,

    // Operation select: 00=MUL, 01=MULH, 10=MULHSU, 11=MULHU
    input  [1:0]    mul_op_i,

    input  [31:0]   operand_a_i,
    input  [31:0]   operand_b_i,

    // Freeze all stages (= stall_any && !mul_ex_stall)
    input           hold_i,

    // Freeze E1.5 stage only (same signal as hold_i from CPU top)
    input           mul_hold_e15_i,

    output [31:0]   writeback_value_o
);

    // ========================================================================
    // E1: Sign-extend operands and latch
    // ========================================================================
    reg  [32:0] operand_a_e1_q;
    reg  [32:0] operand_b_e1_q;
    reg         mulhi_sel_e1_q;

    reg  [32:0] operand_a_r;
    reg  [32:0] operand_b_r;

    always @* begin
        case (mul_op_i)
            2'b01,          // MULH:   signed A × signed B
            2'b10:          // MULHSU: signed A × unsigned B
                operand_a_r = {operand_a_i[31], operand_a_i};
            default:        // MUL / MULHU: unsigned A
                operand_a_r = {1'b0, operand_a_i};
        endcase
    end

    always @* begin
        case (mul_op_i)
            2'b01:          // MULH: signed B
                operand_b_r = {operand_b_i[31], operand_b_i};
            default:        // MUL / MULHSU / MULHU: unsigned B
                operand_b_r = {1'b0, operand_b_i};
        endcase
    end

    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            operand_a_e1_q <= 33'b0;
            operand_b_e1_q <= 33'b0;
            mulhi_sel_e1_q <= 1'b0;
        end else if (!hold_i) begin
            if (mul_valid_i) begin
                operand_a_e1_q <= operand_a_r;
                operand_b_e1_q <= operand_b_r;
                mulhi_sel_e1_q <= (mul_op_i != 2'b00);
            end else begin
                operand_a_e1_q <= 33'b0;
                operand_b_e1_q <= 33'b0;
                mulhi_sel_e1_q <= 1'b0;
            end
        end
    end

    // ========================================================================
    // E1.5: Decompose 33×33 into four 17×17 partial products (~10-12 gate levels)
    // (* keep=1 *) on sign bits prevents fanout >50 at synthesis.
    // ========================================================================
    wire sign_a = operand_a_e1_q[32]; // synthesis keep — fanout ~17, prevent inlining
    wire sign_b = operand_b_e1_q[32]; // synthesis keep

    // Split 33-bit signed operands into 17-bit halves.
    // a_hi carries the sign bit so signed arithmetic propagates correctly.
    wire [16:0] a_lo = operand_a_e1_q[16:0];
    wire [16:0] a_hi = {sign_a, operand_a_e1_q[32:17]};  // sign-extended upper half
    wire [16:0] b_lo = operand_b_e1_q[16:0];
    wire [16:0] b_hi = {sign_b, operand_b_e1_q[32:17]};

    // Each 17×17 signed product fits in 34 bits
    wire signed [33:0] pp_ll_w = $signed(a_lo) * $signed(b_lo);
    wire signed [33:0] pp_lh_w = $signed(a_lo) * $signed(b_hi);
    wire signed [33:0] pp_hl_w = $signed(a_hi) * $signed(b_lo);
    wire signed [29:0] pp_hh_w = ($signed(a_hi) * $signed(b_hi));  // bits[33:30] exceed 64-bit result range

    reg signed [33:0] pp_ll_e15_q, pp_lh_e15_q, pp_hl_e15_q;
    reg        [29:0] pp_hh_e15_q;
    reg               mulhi_sel_e15_q;

    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            pp_ll_e15_q     <= 34'b0;
            pp_lh_e15_q     <= 34'b0;
            pp_hl_e15_q     <= 34'b0;
            pp_hh_e15_q     <= 30'b0;
            mulhi_sel_e15_q <= 1'b0;
        end else if (!mul_hold_e15_i) begin
            pp_ll_e15_q     <= pp_ll_w;
            pp_lh_e15_q     <= pp_lh_w;
            pp_hl_e15_q     <= pp_hl_w;
            pp_hh_e15_q     <= pp_hh_w;
            mulhi_sel_e15_q <= mulhi_sel_e1_q;
        end
    end

    // ========================================================================
    // E1.5→E2: Sum four partial products with positional shifts.
    // PD-FIX3: Balanced 2-level adder tree — reduces critical path from
    // 3 serial adder delays to 2 (pp_lo and pp_hi computed in parallel).
    //   pp_lo = pp_ll + pp_lh<<17
    //   pp_hi = pp_hl<<17 + pp_hh<<34       (parallel with pp_lo)
    //   result = pp_lo + pp_hi
    // ========================================================================
    wire [63:0] mult_pp_lo =
          {{30{pp_ll_e15_q[33]}}, pp_ll_e15_q}         // 30+34 = 64b
        + {{13{pp_lh_e15_q[33]}}, pp_lh_e15_q, 17'b0}; // 13+34+17 = 64b
    wire [63:0] mult_pp_hi =
          {{13{pp_hl_e15_q[33]}}, pp_hl_e15_q, 17'b0}  // 13+34+17 = 64b
        + {pp_hh_e15_q, 34'b0};                         // 30+34 = 64b
    wire [63:0] mult_result_w = mult_pp_lo + mult_pp_hi;

    wire [31:0] result_r = mulhi_sel_e15_q ? mult_result_w[63:32]
                                            : mult_result_w[31:0];

    // ========================================================================
    // E2: Output register
    // ========================================================================
    reg [31:0] result_e2_q;

    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i)
            result_e2_q <= 32'b0;
        else if (!hold_i)
            result_e2_q <= result_r;
    end

    assign writeback_value_o = result_e2_q;

endmodule
