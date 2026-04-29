module forwarding_unit (
    // Source registers in EX stage
    input [4:0] rs1_ex,
    input [4:0] rs2_ex,
    
    // Destination registers in MEM and WB stages
    input [4:0] rd_mem,
    input [4:0] rd_wb,
    
    // Write enable signals
    input regwrite_mem,
    input regwrite_wb,
    
    // Forwarding control outputs
    output reg [1:0] forward_a,  // For rs1
    output reg [1:0] forward_b   // For rs2
);

    // Forwarding encoding:
    // 2'b00: No forwarding (use value from register file)
    // 2'b01: Forward from WB stage
    // 2'b10: Forward from MEM stage

    always @(*) begin
        // Default: no forwarding
        forward_a = 2'b00;
        forward_b = 2'b00;
        
        // ============================================
        // Forwarding for rs1 (operand A)
        // ============================================
        
        // Priority 1: Forward from MEM stage (most recent)
        if (regwrite_mem && (rd_mem != 5'b00000) && (rd_mem == rs1_ex)) begin
            forward_a = 2'b10;
        end
        // Priority 2: Forward from WB stage (older instruction)
        else if (regwrite_wb && (rd_wb != 5'b00000) && (rd_wb == rs1_ex)) begin
            forward_a = 2'b01;
        end
        // Otherwise: no forwarding (use register file value)
        else begin
            forward_a = 2'b00;
        end
        
        // ============================================
        // Forwarding for rs2 (operand B)
        // ============================================
        
        // Priority 1: Forward from MEM stage (most recent)
        if (regwrite_mem && (rd_mem != 5'b00000) && (rd_mem == rs2_ex)) begin
            forward_b = 2'b10;
        end
        // Priority 2: Forward from WB stage (older instruction)
        else if (regwrite_wb && (rd_wb != 5'b00000) && (rd_wb == rs2_ex)) begin
            forward_b = 2'b01;
        end
        // Otherwise: no forwarding (use register file value)
        else begin
            forward_b = 2'b00;
        end
    end

endmodule