// ============================================================
// Module: ascon_INITIALIZATION  (v2 — fix IV_128A)
//
// FIX vs v1:
//   IV_128A: placeholder → giá trị đúng theo NIST Ascon-AEAD128a spec
//     SW ascon.py builds IV bytes: [version=1, 0, (b<<4)|a=0x8c,
//                                    taglen=128 as LE16 → 0x80,0x00,
//                                    rate=32=0x20, 0, 0]
//     = 01 00 8c 80 00 20 00 00  (8 bytes)
//     SW loads as LE 64-bit int: int.from_bytes(...,'little')
//                               = 0x00002000808c0001
//
// State layout (320 bits):
//   [319:256] = x0 = IV
//   [255:192] = x1 = bswap(key_in[127:64])
//   [191:128] = x2 = bswap(key_in[63:0])
//   [127: 64] = x3 = bswap(nonce_in[127:64])
//   [ 63:  0] = x4 = bswap(nonce_in[63:0])
// ============================================================
module ascon_INITIALIZATION (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         load_key,
    input  wire         load_nonce,
    input  wire [1:0]   mode,
    input  wire         init_start,

    input  wire [127:0] key_in,
    input  wire [127:0] nonce_in,

    output reg  [319:0] init_state_out,
    output reg          init_valid
);

    // NIST Ascon-AEAD128:  IV bytes 01 00 8c 80 00 10 00 00 → LE64 = 0x00001000808c0001
    // NIST Ascon-AEAD128a: IV bytes 01 00 8c 80 00 20 00 00 → LE64 = 0x00002000808c0001
    //   (rate=32=0x20, b=12,a=8 → (b<<4)|a = 0x8c, taglen=128 → 0x80 LE16)
    localparam [63:0] IV_128  = 64'h00001000808c0001; // NIST Ascon-AEAD128
    localparam [63:0] IV_128A = 64'h00002000808c0001; // FIX: NIST Ascon-AEAD128a (rate=32 bytes)
    localparam [63:0] IV_HASH = 64'h00400c0000000100; // Ascon-Hash

    function [63:0] bswap64;
        input [63:0] x;
        begin
            bswap64 = { x[ 7: 0], x[15: 8], x[23:16], x[31:24],
                        x[39:32], x[47:40], x[55:48], x[63:56] };
        end
    endfunction

    reg [127:0] key_reg;
    reg [127:0] nonce_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) key_reg <= 128'b0;
        else if (load_key) key_reg <= key_in;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) nonce_reg <= 128'b0;
        else if (load_nonce) nonce_reg <= nonce_in;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_state_out <= 320'b0;
            init_valid     <= 1'b0;
        end else if (init_start) begin
            init_valid <= 1'b1;
            case (mode)
                2'b00: init_state_out <= {
                    IV_128,
                    bswap64(key_reg[127:64]),
                    bswap64(key_reg[63:0]),
                    bswap64(nonce_reg[127:64]),
                    bswap64(nonce_reg[63:0])
                };
                2'b01: init_state_out <= {
                    IV_128A,                     // FIX: dùng giá trị đúng
                    bswap64(key_reg[127:64]),
                    bswap64(key_reg[63:0]),
                    bswap64(nonce_reg[127:64]),
                    bswap64(nonce_reg[63:0])
                };
                2'b10: init_state_out <= {
                    IV_HASH,
                    bswap64(key_reg[127:64]),
                    bswap64(key_reg[63:0]),
                    bswap64(nonce_reg[127:64]),
                    bswap64(nonce_reg[63:0])
                };
                default: init_state_out <= {
                    IV_128,
                    bswap64(key_reg[127:64]),
                    bswap64(key_reg[63:0]),
                    bswap64(nonce_reg[127:64]),
                    bswap64(nonce_reg[63:0])
                };
            endcase
        end else begin
            init_valid <= 1'b0;
        end
    end

endmodule