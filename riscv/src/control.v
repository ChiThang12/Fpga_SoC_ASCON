module control (
    input wire [6:0] opcode,
    input wire [2:0] funct3,
    input wire [6:0] funct7,

    output reg [3:0] alu_control,
    output reg regwrite,
    output reg alusrc,
    output reg memread,
    output reg memwrite,
    output reg branch,
    output reg jump,
    output reg [1:0] byte_size,
    output reg fence
);

    localparam [6:0]
        OP_R_TYPE   = 7'b0110011,
        OP_I_TYPE   = 7'b0010011,
        OP_LOAD     = 7'b0000011,
        OP_STORE    = 7'b0100011,
        OP_BRANCH   = 7'b1100011,
        OP_JAL      = 7'b1101111,
        OP_JALR     = 7'b1100111,
        OP_LUI      = 7'b0110111,
        OP_AUIPC    = 7'b0010111,
        OP_FENCE    = 7'b0001111;

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
        ALU_SLTU  = 4'b1001,
        ALU_MUL   = 4'b1010,
        ALU_MULH  = 4'b1011,
        ALU_DIV   = 4'b1100,
        ALU_DIVU  = 4'b1101,
        ALU_REM   = 4'b1110,
        ALU_REMU  = 4'b1111;

    localparam [2:0]
        F3_ADD_SUB  = 3'b000,
        F3_SLL      = 3'b001,
        F3_SLT      = 3'b010,
        F3_SLTU     = 3'b011,
        F3_XOR      = 3'b100,
        F3_SRL_SRA  = 3'b101,
        F3_OR       = 3'b110,
        F3_AND      = 3'b111,
        F3_BYTE     = 3'b000,
        F3_HALF     = 3'b001,
        F3_WORD     = 3'b010,
        F3_BYTE_U   = 3'b100,
        F3_HALF_U   = 3'b101,
        F3_MUL      = 3'b000,
        F3_MULH     = 3'b001,
        F3_DIV      = 3'b100,
        F3_DIVU     = 3'b101,
        F3_REM      = 3'b110,
        F3_REMU     = 3'b111;

    localparam [6:0]
        F7_SUB_SRA  = 7'b0100000,
        F7_MULDIV   = 7'b0000001;

    always @(*) begin
        regwrite    = 0;
        alusrc      = 0;
        memread     = 0;
        memwrite    = 0;
        branch      = 0;
        jump        = 0;
        fence       = 0;
        byte_size   = 2'b10;
        alu_control = ALU_ADD;

        case (opcode)
            OP_R_TYPE: begin
                regwrite = 1;
                alusrc   = 0;
                if (funct7 == F7_MULDIV) begin
                    case (funct3)
                        F3_MUL:   alu_control = ALU_MUL;
                        F3_MULH:  alu_control = ALU_MULH;
                        F3_DIV:   alu_control = ALU_DIV;
                        F3_DIVU:  alu_control = ALU_DIVU;
                        F3_REM:   alu_control = ALU_REM;
                        F3_REMU:  alu_control = ALU_REMU;
                        default:  alu_control = ALU_ADD;
                    endcase
                end else begin
                    case (funct3)
                        F3_ADD_SUB: begin
                            if (funct7 == F7_SUB_SRA) alu_control = ALU_SUB;
                            else                       alu_control = ALU_ADD;
                        end
                        F3_SLL:     alu_control = ALU_SLL;
                        F3_SLT:     alu_control = ALU_SLT;
                        F3_SLTU:    alu_control = ALU_SLTU;
                        F3_XOR:     alu_control = ALU_XOR;
                        F3_SRL_SRA: begin
                            if (funct7 == F7_SUB_SRA) alu_control = ALU_SRA;
                            else                       alu_control = ALU_SRL;
                        end
                        F3_OR:      alu_control = ALU_OR;
                        F3_AND:     alu_control = ALU_AND;
                        default:    alu_control = ALU_ADD;
                    endcase
                end
            end

            OP_I_TYPE: begin
                regwrite = 1;
                alusrc   = 1;
                case (funct3)
                    F3_ADD_SUB: alu_control = ALU_ADD;
                    F3_SLL:     alu_control = ALU_SLL;
                    F3_SLT:     alu_control = ALU_SLT;
                    F3_SLTU:    alu_control = ALU_SLTU;
                    F3_XOR:     alu_control = ALU_XOR;
                    F3_SRL_SRA: begin
                        if (funct7 == F7_SUB_SRA) alu_control = ALU_SRA;
                        else                       alu_control = ALU_SRL;
                    end
                    F3_OR:      alu_control = ALU_OR;
                    F3_AND:     alu_control = ALU_AND;
                    default:    alu_control = ALU_ADD;
                endcase
            end

            OP_LOAD: begin
                regwrite    = 1;
                alusrc      = 1;
                memread     = 1;
                alu_control = ALU_ADD;
                case (funct3)
                    F3_BYTE:   byte_size = 2'b00;
                    F3_HALF:   byte_size = 2'b01;
                    F3_WORD:   byte_size = 2'b10;
                    F3_BYTE_U: byte_size = 2'b00;
                    F3_HALF_U: byte_size = 2'b01;
                    default:   byte_size = 2'b10;
                endcase
            end

            OP_STORE: begin
                alusrc      = 1;
                memwrite    = 1;
                alu_control = ALU_ADD;
                case (funct3)
                    F3_BYTE: byte_size = 2'b00;
                    F3_HALF: byte_size = 2'b01;
                    F3_WORD: byte_size = 2'b10;
                    default: byte_size = 2'b10;
                endcase
            end

            OP_BRANCH: begin
                branch      = 1;
                alusrc      = 0;
                alu_control = ALU_SUB;
            end

            OP_JAL: begin
                regwrite    = 1;
                jump        = 1;
                alu_control = ALU_ADD;
            end

            OP_JALR: begin
                regwrite    = 1;
                jump        = 1;
                alusrc      = 1;
                alu_control = ALU_ADD;
            end

            OP_LUI: begin
                regwrite    = 1;
                alusrc      = 1;
                alu_control = ALU_ADD;
            end

            OP_AUIPC: begin
                regwrite    = 1;
                alusrc      = 1;
                alu_control = ALU_ADD;
            end

            OP_FENCE: begin
                fence = 1;
            end

            default: begin
                regwrite    = 0;
                alusrc      = 0;
                memread     = 0;
                memwrite    = 0;
                branch      = 0;
                jump        = 0;
                fence       = 0;
                alu_control = ALU_ADD;
            end
        endcase
    end

endmodule