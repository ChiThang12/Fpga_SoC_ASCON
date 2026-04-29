// ============================================================================
// reg_file.v — RISC-V Register File (32 x 32-bit)
// ============================================================================
// PD-FIX7: Posedge write (standard) + WB→ID bypass mux.
//   - WRITE at POSEDGE clock — avoids negedge-FF timing risk at slow PVT corner
//     where T_cq(negedge) may exceed Tck/2, causing silent hold-time violations.
//   - WB→ID bypass mux: 5-bit comparator + 32-bit 2:1 mux on read path (~1.5ns).
//     This replaces the implicit forwarding from negedge timing assumption.
//   - x0 is hardwired to zero via read bypass (no register write to r0).
// ============================================================================

module reg_file (
    input wire        clock,
    input wire        reset,

    // Read ports (asynchronous, combinational)
    input wire [4:0]  read_reg_num1,      // rs1
    input wire [4:0]  read_reg_num2,      // rs2
    output wire [31:0] read_data1,        // data from rs1
    output wire [31:0] read_data2,        // data from rs2

    // Write port (synchronous — negedge clock)
    input wire        regwrite,           // write enable
    input wire [4:0]  write_reg,          // rd
    input wire [31:0] write_data          // data to write to rd
);

    // 32 registers, each 32-bit
    reg [31:0] registers [31:0];

    integer i;

    // ========================================================================
    // WRITE at POSEDGE clock (standard)
    // ========================================================================
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 32'h00000000;
        end else begin
            if (regwrite && (write_reg != 5'b00000))
                registers[write_reg] <= write_data;
        end
    end

    // ========================================================================
    // READ: Combinational with WB→ID bypass mux
    // Bypass fires when WB writes to the same register being read this cycle.
    // 5-bit comparator + 32-bit 2:1 mux = ~1.5ns, placed before ID/EX register.
    // ========================================================================
    assign read_data1 = (read_reg_num1 == 5'b00000)            ? 32'h00000000 :
                        (regwrite && write_reg == read_reg_num1) ? write_data   :
                                                                   registers[read_reg_num1];

    assign read_data2 = (read_reg_num2 == 5'b00000)            ? 32'h00000000 :
                        (regwrite && write_reg == read_reg_num2) ? write_data   :
                                                                   registers[read_reg_num2];

endmodule