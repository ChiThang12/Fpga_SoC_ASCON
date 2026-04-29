// ============================================================================
// PIPELINE_REG_ID_EX.v — ID/EX Pipeline Register (Refactored)
// ============================================================================
// Supports:
//   - flush: insert NOP bubble (clear all control + data)
//   - stall: hold current values
//   - fwd_capture: when stalled, capture forwarded operand values to prevent
//     stale data after forwarding window closes (FIX-FWD-STALL)
//
// Port naming: _in suffix = from ID stage, _out suffix = to EX stage
// ============================================================================

module PIPELINE_REG_ID_EX (
    input wire        clock,
    input wire        reset,
    input wire        flush,
    input wire        stall,

    // --- Forwarding capture during stall (FIX-FWD-STALL) ---
    input wire [1:0]  fwd_a_sel,          // current forward_a selection
    input wire [1:0]  fwd_b_sel,          // current forward_b selection
    input wire [31:0] fwd_a_data,         // forwarded operand A value
    input wire [31:0] fwd_b_data,         // forwarded operand B (pre-mux) value

    // --- Control signals ---
    input wire        regwrite_in,
    input wire        alusrc_in,
    input wire        memread_in,
    input wire        memwrite_in,
    input wire        branch_in,
    input wire        predict_taken_in,
    input wire        jump_in,

    // --- Data ---
    input wire [31:0] read_data1_in,
    input wire [31:0] read_data2_in,
    input wire [31:0] imm_in,
    input wire [31:0] pc_in,
    input wire [31:0] branch_target_in,

    // --- Register addresses ---
    input wire [4:0]  rs1_in,
    input wire [4:0]  rs2_in,
    input wire [4:0]  rd_in,

    // --- Function codes & opcode ---
    input wire [2:0]  funct3_in,
    input wire [3:0]  alu_control_in,
    input wire [1:0]  byte_size_in,
    input wire [6:0]  opcode_in,

    // --- Control outputs ---
    output reg        regwrite_out,
    output reg        alusrc_out,
    output reg        memread_out,
    output reg        memwrite_out,
    output reg        branch_out,
    output reg        predict_taken_out,
    output reg        jump_out,

    // --- Data outputs ---
    output reg [31:0] read_data1_out,
    output reg [31:0] read_data2_out,
    output reg [31:0] imm_out,
    output reg [31:0] pc_out,
    output reg [31:0] branch_target_out,

    // --- Register address outputs ---
    output reg [4:0]  rs1_out,
    output reg [4:0]  rs2_out,
    output reg [4:0]  rd_out,

    // --- Function code & opcode outputs ---
    output reg [2:0]  funct3_out,
    output reg [3:0]  alu_control_out,
    output reg [1:0]  byte_size_out,
    output reg [6:0]  opcode_out
);

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            regwrite_out    <= 1'b0;
            alusrc_out      <= 1'b0;
            memread_out     <= 1'b0;
            memwrite_out    <= 1'b0;
            branch_out      <= 1'b0;
            predict_taken_out <= 1'b0;
            jump_out        <= 1'b0;
            read_data1_out  <= 32'h0;
            read_data2_out  <= 32'h0;
            imm_out         <= 32'h0;
            pc_out          <= 32'h0;
            branch_target_out <= 32'h0;
            rs1_out         <= 5'b0;
            rs2_out         <= 5'b0;
            rd_out          <= 5'b0;
            funct3_out      <= 3'b0;
            alu_control_out <= 4'b0;
            byte_size_out   <= 2'b0;
            opcode_out      <= 7'b0;
        end else if (flush) begin
            regwrite_out    <= 1'b0;
            alusrc_out      <= 1'b0;
            memread_out     <= 1'b0;
            memwrite_out    <= 1'b0;
            branch_out      <= 1'b0;
            predict_taken_out <= 1'b0;
            jump_out        <= 1'b0;
            read_data1_out  <= 32'h0;
            read_data2_out  <= 32'h0;
            imm_out         <= 32'h0;
            pc_out          <= 32'h0;
            branch_target_out <= 32'h0;
            rs1_out         <= 5'b0;
            rs2_out         <= 5'b0;
            rd_out          <= 5'b0;
            funct3_out      <= 3'b0;
            alu_control_out <= 4'b0;
            byte_size_out   <= 2'b0;
            opcode_out      <= 7'b0;
        end else if (!stall) begin
            // Normal latch from ID stage
            regwrite_out    <= regwrite_in;
            alusrc_out      <= alusrc_in;
            memread_out     <= memread_in;
            memwrite_out    <= memwrite_in;
            branch_out      <= branch_in;
            predict_taken_out <= predict_taken_in;
            jump_out        <= jump_in;
            read_data1_out  <= read_data1_in;
            read_data2_out  <= read_data2_in;
            imm_out         <= imm_in;
            pc_out          <= pc_in;
            branch_target_out <= branch_target_in;
            rs1_out         <= rs1_in;
            rs2_out         <= rs2_in;
            rd_out          <= rd_in;
            funct3_out      <= funct3_in;
            alu_control_out <= alu_control_in;
            byte_size_out   <= byte_size_in;
            opcode_out      <= opcode_in;
        end else begin
            // [FIX-FWD-STALL] During stall, capture forwarded values
            // to prevent stale data when forwarding window expires.
            // NOTE: posedge-clocked FF with conditional enable — NOT a latch.
            // Explicit self-assignment on else prevents synthesis latch warnings.
            if (fwd_a_sel != 2'b00) read_data1_out <= fwd_a_data;
            else                    read_data1_out <= read_data1_out;
            if (fwd_b_sel != 2'b00) read_data2_out <= fwd_b_data;
            else                    read_data2_out <= read_data2_out;
        end
    end

endmodule