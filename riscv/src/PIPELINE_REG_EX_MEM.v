// ============================================================================
// PIPELINE_REG_EX_MEM.v — EX/MEM Pipeline Register
// ============================================================================
// Supports:
//   - stall_ex_mem: freeze when LSU dependency stall
//   - bubble insertion: when upstream stall_any && !fence_stall, insert NOP
//     to prevent double-issue of the instruction held in EX
//   - fence_stall bypass: instruction before FENCE must pass through to MEM
//
// Design note: data path always latches when !stall_ex_mem.
// Control signals are selectively cleared (bubble) based on stall conditions.
// ============================================================================

module PIPELINE_REG_EX_MEM (
    input wire        clock,
    input wire        reset,

    // Stall controls
    input wire        stall_ex_mem,     // freeze entire register (LSU dependency)
    input wire        stall_any,        // upstream stall (imem miss, load-use, debug)
    input wire        fence_stall,      // FENCE stall — do NOT bubble pre-fence insn

    // --- Control signals ---
    input wire        regwrite_in,
    input wire        memread_in,
    input wire        memwrite_in,
    input wire        jump_in,

    // --- Data ---
    input wire [31:0] alu_result_in,
    input wire [31:0] write_data_in,    // rs2 forwarded value for store
    input wire [31:0] pc_plus_4_in,

    // --- Register address ---
    input wire [4:0]  rd_in,
    input wire [1:0]  byte_size_in,
    input wire [2:0]  funct3_in,

    // --- MUL flag ---
    input wire        is_mul_in,

    // --- Control outputs ---
    output reg        regwrite_out,
    output reg        memread_out,
    output reg        memwrite_out,
    output reg        jump_out,

    // --- Data outputs ---
    output reg [31:0] alu_result_out,
    output reg [31:0] write_data_out,
    output reg [31:0] pc_plus_4_out,

    // --- Register address output ---
    output reg [4:0]  rd_out,
    output reg [1:0]  byte_size_out,
    output reg [2:0]  funct3_out,

    // --- MUL flag output ---
    output reg        is_mul_out
);

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            regwrite_out  <= 1'b0;
            memread_out   <= 1'b0;
            memwrite_out  <= 1'b0;
            jump_out      <= 1'b0;
            is_mul_out    <= 1'b0;
            alu_result_out <= 32'h0;
            write_data_out <= 32'h0;
            pc_plus_4_out  <= 32'h0;
            rd_out        <= 5'b0;
            byte_size_out <= 2'b0;
            funct3_out    <= 3'b0;
        end else if (!stall_ex_mem) begin
            // Data path: always latch
            alu_result_out <= alu_result_in;
            write_data_out <= write_data_in;
            pc_plus_4_out  <= pc_plus_4_in;
            rd_out         <= rd_in;
            byte_size_out  <= byte_size_in;
            funct3_out     <= funct3_in;

            // Control path: bubble insertion logic
            // [FIX-FENCE-SQUASH] When stall_any is caused solely by fence_stall,
            // allow the instruction already in EX (the sw before the fence) to
            // pass through into MEM — do NOT insert a bubble.
            if (stall_any && !fence_stall) begin
                regwrite_out <= 1'b0;
                memread_out  <= 1'b0;
                memwrite_out <= 1'b0;
                jump_out     <= 1'b0;
                is_mul_out   <= 1'b0;
            end else begin
                regwrite_out <= regwrite_in;
                memread_out  <= memread_in;
                memwrite_out <= memwrite_in;
                jump_out     <= jump_in;
                is_mul_out   <= is_mul_in;
            end
        end
    end

endmodule
