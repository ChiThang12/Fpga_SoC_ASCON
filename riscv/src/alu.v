// ============================================================================
// alu.v — RISC-V Arithmetic Logic Unit (RV32I)
// ============================================================================
// Performs arithmetic, logic, shift, and comparison operations.
// MUL/MULH are handled by riscv_multiplier (pipelined, separate unit).
//
// Timing Optimization Notes:
//   - Comparison flags (less_than, less_than_u) computed independently from
//     alu_result to avoid adding MUX delay to the flag output path
// ============================================================================

module alu (
    input  wire [31:0] in1,
    input  wire [31:0] in2,
    input  wire [3:0]  alu_control,

    output reg  [31:0] alu_result,
    output wire        zero_flag,
    output wire        less_than,
    output wire        less_than_u
);

    // ========================================================================
    // ALU Operation Codes
    // ========================================================================
    localparam [3:0]
        ALU_ADD   = 4'b0000,
        ALU_SUB   = 4'b0001,
        ALU_AND   = 4'b0010,
        ALU_OR    = 4'b0011,
        ALU_XOR   = 4'b0100,
        ALU_SLL   = 4'b0101,
        ALU_SRL   = 4'b0110,
        ALU_SRA   = 4'b0111,
        ALU_SLT   = 4'b1000,
        ALU_SLTU  = 4'b1001;

    // ========================================================================
    // Signed operands (reused for comparisons)
    // ========================================================================
    wire signed [31:0] in1_signed = $signed(in1);
    wire signed [31:0] in2_signed = $signed(in2);

    // ========================================================================
    // Main ALU Operation MUX
    // ========================================================================
    // Pre-compute add/sub as separate wires so synthesis sees them as
    // independent parallel adders and avoids sharing gate-level logic.
    wire [31:0] add_result = in1 + in2;
    wire [31:0] sub_result = in1 - in2;
    always @(*) begin
        case (alu_control)
            // --- RV32I: Arithmetic ---
            ALU_ADD:  alu_result = add_result;
            ALU_SUB:  alu_result = sub_result;

            // --- RV32I: Logic ---
            ALU_AND:  alu_result = in1 & in2;
            ALU_OR:   alu_result = in1 | in2;
            ALU_XOR:  alu_result = in1 ^ in2;

            // --- RV32I: Shifts ---
            ALU_SLL:  alu_result = in1 << in2[4:0];
            ALU_SRL:  alu_result = in1 >> in2[4:0];
            ALU_SRA:  alu_result = $signed(in1) >>> in2[4:0];

            // --- RV32I: Set Less Than ---
            ALU_SLT:  alu_result = (in1_signed < in2_signed) ? 32'd1 : 32'd0;
            ALU_SLTU: alu_result = (in1 < in2) ? 32'd1 : 32'd0;

            // MUL/MULH handled by riscv_multiplier — WB mux overrides this 0
            default:  alu_result = 32'd0;
        endcase
    end

    // ========================================================================
    // Comparison Flags — independent of alu_result (shorter timing path)
    // ========================================================================
    // zero_flag computed directly from operands (32-bit XOR + NOR tree, ~1.3 ns)
    // rather than from alu_result (~4-5 ns adder path).
    // Correct because branch_logic only uses zero_flag for BEQ/BNE, where
    // alu_control is always ALU_SUB, so (in1 - in2 == 0) ≡ (in1 == in2).
    assign zero_flag   = (in1 == in2);
    assign less_than   = (in1_signed < in2_signed);
    assign less_than_u = (in1 < in2);

endmodule
