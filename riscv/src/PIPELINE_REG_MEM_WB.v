// ============================================================================
// PIPELINE_REG_MEM_WB.v — MEM/WB Pipeline Register
// ============================================================================
// Two input sources:
//   1. LSU result (load data) — priority, latches when lsu_result_valid=1
//   2. Normal EX→MEM pass-through (ALU result, jump return address)
//
// Design note: LSU results take priority because loads are non-deterministic
// latency. When lsu_result_valid fires, it immediately overwrites the
// register regardless of other stall conditions.
// ============================================================================

module PIPELINE_REG_MEM_WB (
    input wire        clock,
    input wire        reset,

    // Stall/guard controls
    input wire        stall_ex_mem,      // freeze guard (sync with EX/MEM)
    input wire        lsu_committed,     // LSU result was committed last cycle

    // --- LSU result path (priority) ---
    input wire        lsu_result_valid,
    input wire [31:0] lsu_result_data,
    input wire [4:0]  lsu_result_rd,

    // --- Normal MEM stage path ---
    input wire        regwrite_in,
    input wire        memread_in,        // for masking regwrite on pending loads
    input wire        jump_in,
    input wire [31:0] alu_result_in,
    input wire [31:0] pc_plus_4_in,
    input wire [4:0]  rd_in,

    // --- MUL result path ---
    input wire        is_mul_in,

    // --- Outputs to WB ---
    output reg        regwrite_out,
    output reg        memtoreg_out,
    output reg        jump_out,
    output reg [31:0] alu_result_out,
    output reg [31:0] mem_data_out,
    output reg [31:0] pc_plus_4_out,
    output reg [4:0]  rd_out,
    output reg        is_mul_out
);

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            regwrite_out  <= 1'b0;
            memtoreg_out  <= 1'b0;
            jump_out      <= 1'b0;
            is_mul_out    <= 1'b0;
            alu_result_out <= 32'h0;
            mem_data_out   <= 32'h0;
            pc_plus_4_out  <= 32'h0;
            rd_out         <= 5'b0;
        end else begin
            if (lsu_result_valid) begin
                // LSU result takes priority — load completed
                mem_data_out   <= lsu_result_data;
                rd_out         <= lsu_result_rd;
                regwrite_out   <= 1'b1;
                memtoreg_out   <= 1'b1;
                jump_out       <= 1'b0;
                is_mul_out     <= 1'b0;
                alu_result_out <= alu_result_in;
                pc_plus_4_out  <= pc_plus_4_in;
            end else if (!stall_ex_mem && !lsu_committed) begin
                // Normal pass-through from MEM stage
                alu_result_out <= alu_result_in;
                pc_plus_4_out  <= pc_plus_4_in;
                regwrite_out   <= regwrite_in & ~memread_in;
                memtoreg_out   <= 1'b0;
                jump_out       <= jump_in;
                rd_out         <= rd_in;
                is_mul_out     <= is_mul_in;
                mem_data_out   <= 32'h0; // [LINT-FIX] clear when no LSU result (memtoreg_out=0 so WB ignores this)
            end
        end
    end

endmodule
