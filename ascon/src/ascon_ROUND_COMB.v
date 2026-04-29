// ============================================================================
// Module: ASCON_ROUND_COMB
//
// Một round ASCON hoàn toàn combinational:
//   CA → SL (combinational) → LD
//
// Dùng trong unrolled pipeline: mỗi instance là 1 round,
// pipeline register chèn GIỮA các instance.
// ============================================================================

module ASCON_ROUND_COMB (
    input  wire [319:0] state_in,
    input  wire [3:0]   round_const,   // absolute round index
    output wire [319:0] state_out
);

    // ---- Unpack ----
    wire [63:0] x0 = state_in[319:256];
    wire [63:0] x1 = state_in[255:192];
    wire [63:0] x2 = state_in[191:128];
    wire [63:0] x3 = state_in[127: 64];
    wire [63:0] x4 = state_in[ 63:  0];

    // ---- Constant Addition ----
    wire [7:0]  rc  = 8'hF0 - ({4'h0, round_const} * 8'h0F);
    wire [63:0] x2c = x2 ^ {56'h0, rc};

    // ---- Substitution Layer (combinational, bit-sliced) ----
    wire [63:0] s0, s1, s2, s3, s4;

    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin : sl
            wire [4:0] sb_in  = {x4[i], x3[i], x2c[i], x1[i], x0[i]};
            wire [4:0] sb_out;

            // Correct ASCON 5-bit S-box with affine transformations
            wire x0_in = sb_in[0];
            wire x1_in = sb_in[1];
            wire x2_in = sb_in[2];
            wire x3_in = sb_in[3];
            wire x4_in = sb_in[4];

            // 1. First Affine Transformation
            wire v0 = x0_in ^ x4_in;
            wire v1 = x1_in;
            wire v2 = x2_in ^ x1_in;
            wire v3 = x3_in;
            wire v4 = x4_in ^ x3_in;

            // 2. Chi (Non-linear Layer)
            wire t0 = (~v0) & v1;
            wire t1 = (~v1) & v2;
            wire t2 = (~v2) & v3;
            wire t3 = (~v3) & v4;
            wire t4 = (~v4) & v0;

            wire u0 = v0 ^ t1;
            wire u1 = v1 ^ t2;
            wire u2 = v2 ^ t3;
            wire u3 = v3 ^ t4;
            wire u4 = v4 ^ t0;

            // 3. Second Affine Transformation
            assign sb_out[0] = u0 ^ u4;
            assign sb_out[1] = u1 ^ u0;
            assign sb_out[2] = ~u2;
            assign sb_out[3] = u3 ^ u2;
            assign sb_out[4] = u4;

            assign s0[i] = sb_out[0];
            assign s1[i] = sb_out[1];
            assign s2[i] = sb_out[2];
            assign s3[i] = sb_out[3];
            assign s4[i] = sb_out[4];
        end
    endgenerate

    // ---- Linear Diffusion ----
    function [63:0] ROR;
        input [63:0] v;
        input integer n;
        begin ROR = (v >> n) | (v << (64-n)); end
    endfunction

    wire [63:0] d0 = s0 ^ ROR(s0, 19) ^ ROR(s0, 28);
    wire [63:0] d1 = s1 ^ ROR(s1, 61) ^ ROR(s1, 39);
    wire [63:0] d2 = s2 ^ ROR(s2,  1) ^ ROR(s2,  6);
    wire [63:0] d3 = s3 ^ ROR(s3, 10) ^ ROR(s3, 17);
    wire [63:0] d4 = s4 ^ ROR(s4,  7) ^ ROR(s4, 41);

    assign state_out = {d0, d1, d2, d3, d4};

endmodule