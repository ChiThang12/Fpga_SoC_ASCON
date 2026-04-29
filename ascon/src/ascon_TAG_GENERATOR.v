// ============================================================================
// Module: ascon_TAG_GENERATOR  (OPT v2 — bswap rút gọn)
//
// OPTIMIZATION: bswap(A ^ bswap(B)) = bswap(A) ^ B
//
//   Baseline:
//     tag_out = { bswap(state[127:64] ^ bswap(key[127:64])),
//                 bswap(state[ 63: 0] ^ bswap(key[ 63: 0])) }
//     → 4 lần bswap64
//
//   Optimized:
//     tag_out = { bswap(state[127:64]) ^ key[127:64],
//                 bswap(state[ 63: 0]) ^ key[ 63: 0] }
//     → 2 lần bswap64 (giảm 50% logic bswap)
//
// Correctness:
//   bswap(x3 ^ bswap(key_hi)) = bswap(x3) ^ bswap(bswap(key_hi))
//                              = bswap(x3) ^ key_hi    ✓
// ============================================================================
module ascon_TAG_GENERATOR (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         gen_tag,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [319:0] state_in,  // [319:128] unused; tag uses x3/x4 = [127:0]
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [127:0] key_in,

    output reg  [127:0] tag_out,
    output reg          tag_valid
);

    function [63:0] bswap64;
        input [63:0] x;
        begin
            bswap64 = { x[ 7: 0], x[15: 8], x[23:16], x[31:24],
                        x[39:32], x[47:40], x[55:48], x[63:56] };
        end
    endfunction

    // bswap(x3) ^ key_hi  và  bswap(x4) ^ key_lo
    // = output tag bytes đúng chuẩn NIST Ascon-AEAD128
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tag_out   <= 128'b0;
            tag_valid <= 1'b0;
        end else if (gen_tag) begin
            tag_out <= {
                bswap64(state_in[127:64]) ^ key_in[127:64],  // tag[127:64]
                bswap64(state_in[ 63: 0]) ^ key_in[ 63: 0]   // tag[ 63: 0]
            };
            tag_valid <= 1'b1;
        end else begin
            tag_valid <= 1'b0;
        end
    end

endmodule