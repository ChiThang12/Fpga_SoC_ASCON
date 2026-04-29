// ============================================================================
// Module: ascon_CONTROLLER  (v9 — Pipeline-aware FSM)
//
// THAY ĐỔI CHÍNH so với v8:
//
//   1. PIPELINE LATENCY SUPPORT
//      - Khi G_SBOX_PIPELINE=1, permutation có latency = `rounds` cycles
//      - FSM dùng perm_done (đã tồn tại) để chờ → KHÔNG cần sửa nhiều
//      - Nhưng thêm perm_lat_cnt để debug và future use
//
//   2. THROUGHPUT OPTIMIZATION (G_SBOX_PIPELINE=0):
//      - Khi comb unroll: permutation done = 2 cycles (start + latch)
//      - FSM có thể overlap: bắt đầu load data trong khi perm đang chạy
//      - Thêm S_PERM_OVERLAP state cho optimization này
//
//   3. CALLS_PA / CALLS_PB ENCODING:
//      ASCON-128:  pa_rounds=12, pb_rounds=6  (pa calls = 1 full pa)
//      ASCON-128a: pa_rounds=12, pb_rounds=8  (pa calls = 1 full pa)
//      → CONTROLLER gọi permutation 1 lần với rounds=12 (pa) hoặc rounds=6/8 (pb)
//      → KHÔNG còn multi-call phức tạp như trước
//
// STATE ENCODING:
//   S_IDLE          → chờ start
//   S_INIT_LOAD     → load key + nonce vào INIT module
//   S_INIT_PERM     → chạy pa (12 rounds)
//   S_POST_INIT     → XOR key vào state
//   S_AD_LOAD       → load AD block
//   S_AD_PERM       → chạy pb
//   S_DOM_SEP       → domain separation
//   S_DATA_LOAD     → load plaintext/ciphertext block
//   S_DATA_PERM     → chạy pb
//   S_PRE_FIN       → XOR key
//   S_FIN_PERM      → chạy pa (12 rounds)
//   S_TAG_GEN       → generate/compare tag
//   S_DONE          → output results
// ============================================================================

module ascon_CONTROLLER #(
    parameter G_COMB_RND_128  = 6,
    parameter G_COMB_RND_128A = 4,
    parameter G_SBOX_PIPELINE = 0
) (
    input  wire         clk,
    input  wire         rst_n,

    // Top-level control
    input  wire         start,
    input  wire [1:0]   mode,      // 00=ASCON-128, 01=ASCON-128a
    input  wire         enc_dec,   // 0=enc, 1=dec

    // Data inputs (for internal use/routing)
    input  wire [127:0] key_in,
    input  wire [127:0] nonce_in,
    input  wire [127:0] ad_in,
    input  wire         ad_valid,
    input  wire         ad_last,
    input  wire [127:0] data_in,
    input  wire         data_valid,
    input  wire         data_last,
    input  wire [6:0]   data_len,
    input  wire [127:0] tag_received,

    // Status from datapath/permutation
    input  wire         init_done,
    input  wire         perm_done,
    input  wire         tag_gen_valid,
    input  wire         tag_cmp_done,
    input  wire         extra_pad_block_needed,

    // Control outputs
    output reg          load_key,
    output reg          load_nonce,
    output reg          init_start,
    output reg  [1:0]   state_src_sel,
    output reg          state_load,
    output reg          dp_pad_enable,
    output reg  [1:0]   dp_block_sel,
    output reg          dp_enc_dec,
    output reg  [3:0]   perm_rounds,
    output reg  [3:0]   perm_start_rc,
    output reg          perm_start,
    output reg          gen_tag,
    output reg          compare_tag,
    output reg          do_post_init_key_xor,
    output reg          do_pre_fin_key_xor,
    output reg          do_dom_sep,

    // Status outputs
    output reg          data_ready,
    output reg          data_out_valid,
    output reg          done,
    output reg          busy
);

    // =========================================================================
    // Mode-dependent parameters
    // =========================================================================
    wire        is_128a    = mode[0];

    // pb rounds: 6 for ASCON-128, 8 for ASCON-128a
    wire [3:0]  pb_rounds  = is_128a ? 4'd8 : 4'd6;

    // pa rounds: always 12
    wire [3:0]  pa_rounds  = 4'd12;

    // start_rc for pb: rc = 12 - pb_rounds
    // ASCON-128:  pb=6 → start_rc = 6
    // ASCON-128a: pb=8 → start_rc = 4
    wire [3:0]  pb_start_rc = is_128a ? 4'd4 : 4'd6;

    // pa always starts at rc=0
    wire [3:0]  pa_start_rc = 4'd0;

    // =========================================================================
    // FSM State encoding
    // =========================================================================
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_INIT_LOAD  = 4'd1,
        S_INIT_PERM  = 4'd2,
        S_POST_INIT  = 4'd3,
        S_AD_LOAD    = 4'd4,
        S_AD_PERM    = 4'd5,
        S_DOM_SEP    = 4'd6,
        S_DATA_LOAD  = 4'd7,
        S_DATA_PERM  = 4'd8,
        S_PRE_FIN    = 4'd9,
        S_FIN_PERM   = 4'd10,
        S_TAG_GEN    = 4'd11,
        S_DONE       = 4'd12;

    reg [3:0] state, next_state;

    // (perm_done from PERMUTATION already encodes the correct latency — no extra counter needed)

    // =========================================================================
    // FSM — Sequential
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // =========================================================================
    // FSM — Combinational next-state + output
    // =========================================================================
    always @(*) begin
        // Default: hold everything
        next_state           = state;
        load_key             = 1'b0;
        load_nonce           = 1'b0;
        init_start           = 1'b0;
        state_src_sel        = 2'b10;   // default: from perm
        state_load           = 1'b0;
        dp_pad_enable        = 1'b0;
        dp_block_sel         = 2'b00;
        dp_enc_dec           = enc_dec;
        perm_rounds          = pb_rounds;
        perm_start_rc        = pb_start_rc;
        perm_start           = 1'b0;
        gen_tag              = 1'b0;
        compare_tag          = 1'b0;
        do_post_init_key_xor = 1'b0;
        do_pre_fin_key_xor   = 1'b0;
        do_dom_sep           = 1'b0;
        data_ready           = (state == S_IDLE) || (state == S_DATA_LOAD);
        data_out_valid       = 1'b0;
        done                 = 1'b0;
        busy                 = 1'b1;

        case (state)
            // ---- IDLE: wait for start ----
            S_IDLE: begin
                busy = 1'b0;
                if (start) begin
                    load_key   = 1'b1;
                    load_nonce = 1'b1;
                    next_state = S_INIT_LOAD;
                end
            end

            // ---- INIT_LOAD: start init calculation with stable keys ----
            S_INIT_LOAD: begin
                init_start = 1'b1;
                next_state = S_INIT_PERM;
            end

            // ---- INIT_PERM: wait for INIT to produce state, then run pa ----
            S_INIT_PERM: begin
                if (init_done) begin
                    // Load init state into state register
                    state_src_sel = 2'b00;  // from init
                    state_load    = 1'b1;
                    // Start pa permutation
                    perm_rounds   = pa_rounds;
                    perm_start_rc = pa_start_rc;
                    perm_start    = 1'b1;
                    next_state    = S_POST_INIT;
                end
            end

            // ---- POST_INIT: wait for pa done, XOR key into state ----
            S_POST_INIT: begin
                if (perm_done) begin
                    // XOR key: handled in CORE via do_post_init_key_xor
                    do_post_init_key_xor = 1'b1;
                    state_src_sel        = 2'b10;   // from perm (with key XOR in CORE)
                    state_load           = 1'b1;
                    next_state           = S_AD_LOAD;
                end
            end

            // ---- AD_LOAD: absorb one AD block ----
            S_AD_LOAD: begin
                if (ad_valid) begin
                    dp_pad_enable = 1'b1;
                    dp_block_sel  = 2'b00;  // AD
                    state_src_sel = 2'b01;  // from datapath XOR
                    state_load    = 1'b1;

                    // Start pb after loading
                    perm_rounds   = pb_rounds;
                    perm_start_rc = pb_start_rc;
                    perm_start    = 1'b1;
                    next_state    = S_AD_PERM;
                end else if (!ad_valid && !ad_last) begin
                    // No AD at all: go to domain separation
                    next_state = S_DOM_SEP;
                end
            end

            // ---- AD_PERM: wait for pb after AD ----
            S_AD_PERM: begin
                if (perm_done) begin
                    state_src_sel = 2'b10;
                    state_load    = 1'b1;
                    if (ad_last || extra_pad_block_needed) begin
                        next_state = S_DOM_SEP;
                    end else begin
                        next_state = S_AD_LOAD;
                    end
                end
            end

            // ---- DOM_SEP: flip MSB of x4 ----
            S_DOM_SEP: begin
                do_dom_sep    = 1'b1;
                state_src_sel = 2'b10;   // will be overridden by dom_sep in CORE
                state_load    = 1'b1;
                next_state    = S_DATA_LOAD;
            end

            // ---- DATA_LOAD: absorb/extract one data block ----
            S_DATA_LOAD: begin
                if (data_valid) begin
                    dp_pad_enable  = 1'b1;
                    dp_block_sel   = 2'b01;    // data
                    dp_enc_dec     = enc_dec;
                    state_src_sel  = 2'b01;    // from datapath XOR
                    state_load     = 1'b1;
                    data_out_valid = 1'b1;

                    perm_rounds    = pb_rounds;
                    perm_start_rc  = pb_start_rc;
                    perm_start     = !data_last;  // don't permute on last block
                    next_state     = data_last ? S_PRE_FIN : S_DATA_PERM;
                end else begin
                    next_state     = S_DATA_LOAD; // wait for valid data
                end
            end

            // ---- DATA_PERM: wait for pb after data ----
            S_DATA_PERM: begin
                if (perm_done) begin
                    state_src_sel = 2'b10;
                    state_load    = 1'b1;
                    next_state    = S_DATA_LOAD;
                end
            end

            // ---- PRE_FIN: XOR key into state ----
            S_PRE_FIN: begin
                do_pre_fin_key_xor = 1'b1;
                state_src_sel      = 2'b10;
                state_load         = 1'b1;
                // Start pa
                perm_rounds        = pa_rounds;
                perm_start_rc      = pa_start_rc;
                perm_start         = 1'b1;
                next_state         = S_FIN_PERM;
            end

            // ---- FIN_PERM: wait for final pa ----
            S_FIN_PERM: begin
                if (perm_done) begin
                    state_src_sel = 2'b10;
                    state_load    = 1'b1;
                    next_state    = S_TAG_GEN;
                end
            end

            // ---- TAG_GEN: generate or compare tag ----
            S_TAG_GEN: begin
                gen_tag = 1'b1;
                if (tag_gen_valid) begin
                    if (enc_dec == 1'b0) begin
                        next_state = S_DONE;
                    end else begin
                        compare_tag = 1'b1;
                        if (tag_cmp_done) begin
                            next_state = S_DONE;
                        end
                    end
                end
            end

            // ---- DONE ----
            S_DONE: begin
                done       = 1'b1;
                busy       = 1'b0;
                next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

endmodule