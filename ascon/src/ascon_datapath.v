// ============================================================
// Module: ascon_DATAPATH  (v2 — GCD dual-rate fix)
//
// FIX vs v1:
//   FIX-1: Rate theo đúng spec ASCON:
//     ASCON-128  (mode=00): rate = 64-bit  → chỉ XOR vào x0
//                           x1 không thay đổi (zero-pad phần upper)
//     ASCON-128a (mode=01): rate = 128-bit → XOR vào cả x0 và x1
//     Cả 2 mode đều dùng chung datapath 128-bit với 1-bit mode MUX
//     (đây chính là G_DUAL_RATE unified datapath theo spec)
//
//   FIX-2: G_DUAL_RATE parameter điều khiển:
//     G_DUAL_RATE=1 (default): unified 128-bit datapath như mô tả trên
//     G_DUAL_RATE=0: chỉ hỗ trợ ASCON-128 đơn (64-bit rate cứng)
//
//   FIX-3: Output data_out chỉ xuất rate portion:
//     ASCON-128:  data_out[127:64] = ciphertext/plaintext 64-bit
//                 data_out[63:0]   = 64'h0 (padding)
//     ASCON-128a: data_out[127:0]  = ciphertext/plaintext 128-bit
//
// Retained from v1:
//   Padding (0x01, extra_pad_block_needed)
//   Decrypt state update (mask/padx formula)
//   Byte-swap (BE→LE)
//   NIST Ascon-AEAD128 correctness
// ============================================================
module ascon_DATAPATH #(
    parameter G_DUAL_RATE = 1    // 1=unified 128-bit datapath, 0=128-only 64-bit
) (
    input  wire         clk,
    input  wire         rst_n,

    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [1:0]   mode,  // mode[1] unused; mode[0] selects 128 vs 128a rate
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire         enc_dec,
    input  wire         pad_enable,
    input  wire [1:0]   block_sel,

    input  wire [127:0] ad_in,
    input  wire [127:0] data_in,
    input  wire [6:0]   data_len,
    input  wire [319:0] state_in,

    output reg  [319:0] state_xored,
    output reg  [127:0] data_out,
    output reg          data_out_valid,
    output wire         extra_pad_block_needed
);

    // ------------------------------------------------------------------
    // Rate bytes:
    //   ASCON-128  (mode=00): rate = 8  bytes (64-bit)
    //   ASCON-128a (mode=01): rate = 16 bytes (128-bit)
    // FIX-1: rate_bytes phụ thuộc mode khi G_DUAL_RATE=1
    // ------------------------------------------------------------------
    wire [6:0] rate_bytes;
    generate
        if (G_DUAL_RATE == 1) begin : gen_dual_rate
            // mode[0]=0 → ASCON-128 (rate=8 bytes), mode[0]=1 → ASCON-128a (rate=16 bytes)
            assign rate_bytes = (mode[0] == 1'b0) ? 7'd8 : 7'd16;
        end else begin : gen_single_rate
            assign rate_bytes = 7'd8;  // ASCON-128 only
        end
    endgenerate

    // extra_pad_block_needed: block đầy, cần thêm block pad riêng
    assign extra_pad_block_needed = pad_enable && (data_len == rate_bytes);

    // Select correct input source
    wire [127:0] raw_input = (block_sel == 2'b00) ? ad_in : data_in;

    // ------------------------------------------------------------------
    // Byte-swap: reverse byte order within a 64-bit word
    // ------------------------------------------------------------------
    function [63:0] bswap64;
        input [63:0] x;
        begin
            bswap64 = { x[ 7: 0], x[15: 8], x[23:16], x[31:24],
                        x[39:32], x[47:40], x[55:48], x[63:56] };
        end
    endfunction

    // ------------------------------------------------------------------
    // Padding:
    //   len < rate : copy bytes[0..len-1], byte[len]=0x01, zero rest
    //   len == rate: extra_pad_block_needed asserted, block này raw
    // ------------------------------------------------------------------
    function [127:0] apply_padding;
        input [127:0] blk;
        input [6:0]   len;
        input [6:0]   rate;
        integer i;
        reg [127:0] out;
        begin
            out = 128'b0;
            for (i = 0; i < 16; i = i + 1) begin
                if (i < {{25{1'b0}}, len})          // FIX-WIDTHEXPAND: cast len to 32-bit
                    out[127 - i*8 -: 8] = blk[127 - i*8 -: 8];
                else if (i == {{25{1'b0}}, len} && len < rate) // FIX-WIDTHEXPAND: cast len to 32-bit
                    out[127 - i*8 -: 8] = 8'h01;
            end
            apply_padding = out;
        end
    endfunction

    // ------------------------------------------------------------------
    // Decrypt helper: mask và padx cho last block
    // ------------------------------------------------------------------
    function [127:0] build_dec_mask;
        input [6:0] len;
        integer i;
        reg [127:0] m;
        begin
            m = 128'b0;
            for (i = 0; i < 16; i = i + 1)
                if (i >= {{25{1'b0}}, len}) m[127 - i*8 -: 8] = 8'hFF; // FIX-WIDTHEXPAND
            build_dec_mask = m;
        end
    endfunction

    function [127:0] build_dec_padx;
        input [6:0] len;
        // integer i removed (unused, FIX-UNUSEDSIGNAL)
        reg [127:0] p;
        begin
            p = 128'b0;
            if (len < 7'd16) p[127 - {{25{1'b0}}, len}*8 -: 8] = 8'h01; // FIX-WIDTHEXPAND
            build_dec_padx = p;
        end
    endfunction

    // ---- state words ----
    wire [63:0] x0 = state_in[319:256];
    wire [63:0] x1 = state_in[255:192];

    // ---- padded data ----
    reg [127:0] padded_data;
    always @(*) begin
        if (pad_enable)
            padded_data = apply_padding(raw_input, data_len, rate_bytes);
        else
            padded_data = raw_input;
    end

    wire [63:0] pd_hi_bswap = bswap64(padded_data[127:64]);
    wire [63:0] pd_lo_bswap = bswap64(padded_data[ 63: 0]);

    // ---- CT zero-padded → LE integers ----
    wire [127:0] ct_zero_padded = raw_input;
    wire [63:0] ci_hi = bswap64(ct_zero_padded[127:64]);
    wire [63:0] ci_lo = bswap64(ct_zero_padded[ 63: 0]);

    // ---- Decrypt mask/padx ----
    wire [127:0] dec_mask_be = build_dec_mask(data_len);
    wire [127:0] dec_padx_be = build_dec_padx(data_len);
    wire [63:0]  mask_hi     = bswap64(dec_mask_be[127:64]);
    wire [63:0]  mask_lo     = bswap64(dec_mask_be[ 63: 0]);
    wire [63:0]  padx_hi     = bswap64(dec_padx_be[127:64]);
    wire [63:0]  padx_lo     = bswap64(dec_padx_be[ 63: 0]);

    wire [63:0] dec_new_x0 = (x0 & mask_hi) ^ ci_hi ^ padx_hi;
    wire [63:0] dec_new_x1 = (x1 & mask_lo) ^ ci_lo ^ padx_lo;

    // ------------------------------------------------------------------
    // Main combinational logic
    // FIX-1: phân biệt rate=64-bit (128) vs rate=128-bit (128a)
    // ------------------------------------------------------------------
    reg [127:0] data_out_comb;
    reg         data_out_valid_comb;
    reg [319:0] state_temp;

    // Determine active rate:
    //   is_128a=1 → 128-bit rate (x0 và x1)
    //   is_128a=0 → 64-bit rate  (chỉ x0, x1 giữ nguyên)
    // mode[0]=0 → ASCON-128 (64-bit rate), mode[0]=1 → ASCON-128a (128-bit rate)
    wire is_128a = (G_DUAL_RATE == 1) && (mode[0] == 1'b1);

    always @(*) begin
        state_temp          = state_in;
        data_out_comb       = 128'b0;
        data_out_valid_comb = 1'b0;
        state_xored         = state_in;

        // ---------------------------------------------------------------
        // FIX-1: unified dual-rate datapath
        //   is_128a=0 (ASCON-128, 64-bit rate):
        //     - Chỉ x0 tham gia XOR
        //     - x1 giữ nguyên trong state
        //     - data_out[127:64] = result 64-bit, data_out[63:0]=0
        //   is_128a=1 (ASCON-128a, 128-bit rate):
        //     - Cả x0 và x1 tham gia XOR
        //     - data_out[127:0] = result 128-bit
        // ---------------------------------------------------------------

        if (block_sel == 2'b00) begin
            // ---- AD block: always encrypt-style XOR absorb ----
            if (!is_128a) begin
                // ASCON-128: rate=64-bit, chỉ x0
                state_temp[319:256] = x0 ^ pd_hi_bswap;
                state_temp[255:192] = x1;           // x1 unchanged
            end else begin
                // ASCON-128a: rate=128-bit, x0 và x1
                state_temp[319:256] = x0 ^ pd_hi_bswap;
                state_temp[255:192] = x1 ^ pd_lo_bswap;
            end
            data_out_valid_comb = 1'b0;  // AD không output data

        end else begin
            // ---- Data block (PT/CT) ----
            if (!is_128a) begin
                // ASCON-128: rate=64-bit
                if (enc_dec == 1'b0) begin
                    // ENCRYPT
                    data_out_comb[127:64] = bswap64(x0 ^ pd_hi_bswap);
                    data_out_comb[ 63: 0] = 64'h0;      // upper lane output=0
                    state_temp[319:256]   = x0 ^ pd_hi_bswap;
                    state_temp[255:192]   = x1;          // x1 unchanged
                end else begin
                    // DECRYPT
                    data_out_comb[127:64] = bswap64(x0 ^ ci_hi);
                    data_out_comb[ 63: 0] = 64'h0;
                    state_temp[319:256]   = dec_new_x0;
                    state_temp[255:192]   = x1;          // x1 unchanged
                end
            end else begin
                // ASCON-128a: rate=128-bit
                if (enc_dec == 1'b0) begin
                    // ENCRYPT
                    data_out_comb[127:64] = bswap64(x0 ^ pd_hi_bswap);
                    data_out_comb[ 63: 0] = bswap64(x1 ^ pd_lo_bswap);
                    state_temp[319:256]   = x0 ^ pd_hi_bswap;
                    state_temp[255:192]   = x1 ^ pd_lo_bswap;
                end else begin
                    // DECRYPT
                    data_out_comb[127:64] = bswap64(x0 ^ ci_hi);
                    data_out_comb[ 63: 0] = bswap64(x1 ^ ci_lo);
                    state_temp[319:256]   = dec_new_x0;
                    state_temp[255:192]   = dec_new_x1;
                end
            end
            data_out_valid_comb = 1'b1;
        end

        state_xored = state_temp;
    end

    // ---- Registered output ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out       <= 128'b0;
            data_out_valid <= 1'b0;
        end else begin
            data_out_valid <= data_out_valid_comb;
            if (data_out_valid_comb)
                data_out <= data_out_comb;
        end
    end

endmodule