// ============================================================================
// branch_logic.v — Branch Condition Evaluator
// ============================================================================
// Evaluates branch conditions based on ALU comparison flags.
// Pure combinational — no timing-critical paths.
//
// funct3 encoding (RISC-V B-type):
//   000 = BEQ   (branch if equal)
//   001 = BNE   (branch if not equal)
//   100 = BLT   (branch if less than, signed)
//   101 = BGE   (branch if greater or equal, signed)
//   110 = BLTU  (branch if less than, unsigned)
//   111 = BGEU  (branch if greater or equal, unsigned)
// ============================================================================

module branch_logic (
    input wire        branch,        // Branch instruction (from control)
    input wire [2:0]  funct3,        // Function field (branch type)
    input wire        zero_flag,     // Zero flag from ALU
    input wire        less_than,     // Signed less than from ALU
    input wire        less_than_u,   // Unsigned less than from ALU
    output reg        taken          // Branch taken signal
);

    always @(*) begin
        if (branch) begin
            case (funct3)
                3'b000:  taken = zero_flag;           // BEQ
                3'b001:  taken = ~zero_flag;          // BNE
                3'b100:  taken = less_than;           // BLT
                3'b101:  taken = ~less_than;          // BGE
                3'b110:  taken = less_than_u;         // BLTU
                3'b111:  taken = ~less_than_u;        // BGEU
                default: taken = 1'b0;
            endcase
        end else begin
            taken = 1'b0;
        end
    end

endmodule