module PIPELINE_REG_IF_ID (
    input clock,
    input reset,
    input flush,
    input stall,
    input [31:0] instr_in,
    input [31:0] pc_in,
    output reg [31:0] instr_out,
    output reg [31:0] pc_out
);

    // NOP instruction (ADDI x0, x0, 0)
    localparam NOP = 32'h00000013;

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            // Reset: khởi tạo với NOP
            instr_out <= NOP;
            pc_out <= 32'h00000000;
        end
        else if (flush) begin
            // Flush: chèn NOP vào pipeline (xóa instruction hiện tại)
            instr_out <= NOP;
            pc_out <= 32'h00000000;
        end
        else if (!stall) begin
            // Normal operation: cập nhật khi không bị stall
            instr_out <= instr_in;
            pc_out <= pc_in;
        end
        // Nếu stall=1: giữ nguyên giá trị (không có else)
    end

endmodule