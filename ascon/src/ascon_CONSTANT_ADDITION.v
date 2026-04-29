// ============================================================================
// Module: CONSTANT_ADDITION  (FIXED)
//
// BUG FIX:
//   Phiên bản cũ: round_number là 4-bit, nhân 4'hF (4-bit) → kết quả truncate
//   về 4-bit trước khi gán cho round_constant 8-bit → sai với mọi round > 1.
//
//   Fix: mở rộng round_number lên 8-bit trước khi nhân.
//   round_constant = 8'hF0 - (8'(round_number) * 8'h0F)
//
// NOTE: round_number ở đây là ABSOLUTE round index (0..11), không phải
// iteration counter. Xem ascon_PERMUTATION.v để biết cách tính.
// ============================================================================

module CONSTANT_ADDITION (
    input  wire [63:0] state_x2,
    input  wire [3:0]  round_number,    // absolute round index: 0..11
    output wire [63:0] state_x2_modified
);

    // FIX: dùng 8-bit để tránh truncation khi nhân
    wire [7:0] round_constant;
    assign round_constant = 8'hF0 - ({4'h0, round_number} * 8'h0F);

    // XOR constant vào 8 bit thấp của x2
    assign state_x2_modified = state_x2 ^ {56'h0, round_constant};

endmodule