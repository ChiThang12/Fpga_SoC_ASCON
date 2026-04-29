// ============================================================================
// Module  : ascon_axi_slave
// Version : 2.4  (FIX-MODE-OVERLAP: tách bit enc_dec khỏi bit variant)
//
// Changes vs v2.3:
//   FIX-MODE-OVERLAP : reg_mode[0] và core_enc_dec trước đây cùng dùng bit[0]
//                      khiến "chọn variant" và "chọn hướng" xung đột nhau.
//                      Fix: reg_mode[1] = direction (enc=0/dec=1),
//                           reg_mode[0] = variant   (128=0/128a=1).
//                      Bảng giá trị ghi ADDR_MODE:
//                        0x0 = ASCON-128  Encrypt
//                        0x1 = ASCON-128a Encrypt
//                        0x2 = ASCON-128  Decrypt
//                        0x3 = ASCON-128a Decrypt
//
// Changes vs v2.2:
//   FIX-INCR-BURST : WR_DATA không tăng wr_addr_lat qua các beat của burst.
//                    Khi CPU dùng memcpy để nạp Key[127:0] hoặc Nonce[127:0],
//                    GCC/CPU phát AXI INCR burst: AWLEN=3, 4 beat × 32-bit.
//                    v2.2 decode tất cả 4 beat vào địa chỉ đầu (KEY_0),
//                    KEY_1/KEY_2/KEY_3 không bao giờ được ghi → key sai.
//                    Fix:
//                      WR_IDLE: set WREADY=1 khi AW handshake (mở kênh W)
//                      WR_DATA: decode mỗi beat WVALID ngay lập tức,
//                               tăng wr_addr_lat += 4 nếu !WLAST,
//                               chuyển WR_RESP chỉ khi WLAST=1.
//
// Changes vs v2.1:
//   FIX-BUG-CTRL2 : core_start bị suppressed khi DMA_EN=1.
//   FIX-BUG-AWREADY : AWREADY=1 chuyển sang WR_DONE (sau BVALID=0).
//   FIX-WLAST       : Thêm wr_wlast_lat, decode khi WLAST=1.
//   FIX-BUG-CTRL    : Tách điều kiện core_start / dma_start.
//
// Changes vs v1.9:
//   FIX-BUG1 : Nâng cấp AXI4-Lite → AXI4-Full, burst read/write.
//   FIX-BUG2 : Thêm register ADDR_DATA_LEN (offset 0x03C).
// ============================================================================
module ascon_axi_slave #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 4
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // ── AXI4-Full Write Address Channel
    input  wire [ID_WIDTH-1:0]     S_AXI_AWID,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ADDR_WIDTH-1:0]   S_AXI_AWADDR,   // [31:12] unused (12-bit offset only)
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [7:0]              S_AXI_AWLEN,    // AXI4-Full port, not decoded (single-reg)
    input  wire [2:0]              S_AXI_AWSIZE,
    input  wire [1:0]              S_AXI_AWBURST,
    input  wire [2:0]              S_AXI_AWPROT,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                    S_AXI_AWVALID,
    output reg                     S_AXI_AWREADY,

    // ── AXI4-Full Write Data Channel
    input  wire [DATA_WIDTH-1:0]   S_AXI_WDATA,
    input  wire [DATA_WIDTH/8-1:0] S_AXI_WSTRB,
    input  wire                    S_AXI_WLAST,
    input  wire                    S_AXI_WVALID,
    output reg                     S_AXI_WREADY,

    // ── AXI4-Full Write Response Channel
    output reg  [ID_WIDTH-1:0]     S_AXI_BID,
    output reg  [1:0]              S_AXI_BRESP,
    output reg                     S_AXI_BVALID,
    input  wire                    S_AXI_BREADY,

    // ── AXI4-Full Read Address Channel
    input  wire [ID_WIDTH-1:0]     S_AXI_ARID,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ADDR_WIDTH-1:0]   S_AXI_ARADDR,   // [31:12] unused (12-bit offset only)
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [7:0]              S_AXI_ARLEN,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [2:0]              S_AXI_ARSIZE,   // AXI4-Full port, not decoded (fixed-reg read)
    input  wire [1:0]              S_AXI_ARBURST,
    input  wire [2:0]              S_AXI_ARPROT,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                    S_AXI_ARVALID,
    output reg                     S_AXI_ARREADY,

    // ── AXI4-Full Read Data Channel
    output reg  [ID_WIDTH-1:0]     S_AXI_RID,
    output reg  [DATA_WIDTH-1:0]   S_AXI_RDATA,
    output reg  [1:0]              S_AXI_RRESP,
    output reg                     S_AXI_RLAST,
    output reg                     S_AXI_RVALID,
    input  wire                    S_AXI_RREADY,

    // ── Interface to ascon_CORE
    output wire [127:0]            core_key,
    output wire [127:0]            core_nonce,
    output wire [127:0]            core_data_in,
    output wire [6:0]              core_data_len,
    output wire                    core_enc_dec,
    output wire [1:0]              core_mode,
    output reg                     core_start,
    output reg                     core_soft_rst,

    input  wire                    core_busy,
    input  wire                    core_done,
    input  wire                    core_data_out_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [127:0]            core_data_out,  // [63:0] zero in ASCON-128 mode
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [127:0]            core_tag_out,
    input  wire                    core_tag_valid,

    // ── Interface to ascon_dma
    output wire [31:0]             dma_src_addr,
    output wire [31:0]             dma_dst_addr,
    output wire [31:0]             dma_length,
    output wire [7:0]              dma_burst_len,
    output wire                    dma_en,
    output reg                     dma_start,
    output reg                     dma_soft_rst,

    input  wire                    dma_busy,
    input  wire                    dma_done,
    input  wire                    dma_error,

    // ── Interrupt
    output wire                    irq
);

    // =========================================================================
    // Address offset localparams  (12-bit offset from base)
    // =========================================================================
    localparam [11:0]
        ADDR_CTRL      = 12'h020,
        ADDR_STATUS    = 12'h004,
        ADDR_MODE      = 12'h000,
        ADDR_IRQ_EN    = 12'h00C,
        ADDR_KEY_0     = 12'h010,
        ADDR_KEY_1     = 12'h014,
        ADDR_KEY_2     = 12'h018,
        ADDR_KEY_3     = 12'h01C,
        ADDR_NONCE_0   = 12'h024,
        ADDR_NONCE_1   = 12'h028,
        ADDR_NONCE_2   = 12'h02C,
        ADDR_NONCE_3   = 12'h030,
        ADDR_PTEXT_0   = 12'h034,
        ADDR_PTEXT_1   = 12'h038,
        ADDR_DATA_LEN  = 12'h03C,   // Changed
        ADDR_CTEXT_0   = 12'h040,
        ADDR_CTEXT_1   = 12'h044,
        ADDR_TAG_0     = 12'h048,
        ADDR_TAG_1     = 12'h04C,
        ADDR_TAG_2     = 12'h050,
        ADDR_TAG_3     = 12'h054,
        ADDR_DMA_SRC   = 12'h100,
        ADDR_DMA_DST   = 12'h104,
        ADDR_DMA_LEN   = 12'h108,
        ADDR_DMA_BURST = 12'h114;

    // =========================================================================
    // Write FSM states
    // =========================================================================
    localparam [1:0]
        WR_IDLE = 2'b00,
        WR_DATA = 2'b01,
        WR_RESP = 2'b10,
        WR_DONE = 2'b11;

    reg [1:0] wr_state;

    // =========================================================================
    // Read FSM states
    // =========================================================================
    localparam [1:0]
        RD_IDLE  = 2'b00,
        RD_VALID = 2'b01;

    reg [1:0] rd_state;

    // =========================================================================
    // Write channel pipeline registers
    // =========================================================================
    reg [11:0]         wr_addr_lat;
    reg [ID_WIDTH-1:0] wr_id_lat;
    reg [31:0]         wr_data_lat;
    reg [3:0]          wr_strb_lat;
    // wr_len_lat removed (unused)
    // wr_beat_cnt removed (unused)
    reg                wr_wlast_lat;  // [FIX-WLAST] latch WLAST khi pre-latch W

    // module-level decode wires (combinational, FIX-BLKSEQ: was blocking reg in seq block)
    wire [31:0] wr_exec_data;
    wire [3:0]  wr_exec_strb;
    // Combinational mux: select bus data (case B) or latched data (case A)
    assign wr_exec_data = (S_AXI_WVALID && S_AXI_WREADY) ? S_AXI_WDATA : wr_data_lat;
    assign wr_exec_strb = (S_AXI_WVALID && S_AXI_WREADY) ? S_AXI_WSTRB : wr_strb_lat;

    // =========================================================================
    // Read channel pipeline registers
    // =========================================================================
    /* verilator lint_off UNUSEDSIGNAL */
    reg [11:0]           rd_addr_lat;  // latched but read via rd_data_lat
    /* verilator lint_on UNUSEDSIGNAL */
    reg [ID_WIDTH-1:0]   rd_id_lat;
    reg [DATA_WIDTH-1:0] rd_data_lat;
    reg [7:0]            rd_len_lat;   // FIX-BUG1: burst length latch
    reg [7:0]            rd_beat_cnt;  // FIX-BUG1: beat counter

    // =========================================================================
    // Storage registers
    // =========================================================================
    // reg_mode[0] = variant  : 0=ASCON-128, 1=ASCON-128a
    // reg_mode[1] = direction: 0=Encrypt,   1=Decrypt
    reg [1:0]  reg_mode;
    reg [2:0]  reg_irq_en;
    reg        reg_dma_en;
    reg [6:0]  reg_data_len;   // FIX-BUG2: số byte thực của PT

    reg [31:0] reg_key_0,   reg_key_1,   reg_key_2,   reg_key_3;
    reg [31:0] reg_nonce_0, reg_nonce_1, reg_nonce_2, reg_nonce_3;
    reg [31:0] reg_ptext_0, reg_ptext_1;

    reg [31:0] reg_ctext_0, reg_ctext_1;
    reg [31:0] reg_tag_0,   reg_tag_1,   reg_tag_2,   reg_tag_3;

    reg [31:0] reg_dma_src, reg_dma_dst, reg_dma_len;
    reg [7:0]  reg_dma_burst_len;

    reg        status_done,  status_dma_done;
    reg        status_error, status_dma_error;

    // =========================================================================
    // Byte-enable helper
    // =========================================================================
    function [31:0] apply_strb;
        input [31:0] old_val, new_val;
        input [3:0]  strb;
        begin
            apply_strb[31:24] = strb[3] ? new_val[31:24] : old_val[31:24];
            apply_strb[23:16] = strb[2] ? new_val[23:16] : old_val[23:16];
            apply_strb[15: 8] = strb[1] ? new_val[15: 8] : old_val[15: 8];
            apply_strb[ 7: 0] = strb[0] ? new_val[ 7: 0] : old_val[ 7: 0];
        end
    endfunction

    // =========================================================================
    // Register read mux
    // =========================================================================
    wire [31:0] status_word = {
        26'h0,
        status_dma_error, // [5]
        status_error,     // [4]
        status_dma_done,  // [3]
        dma_busy,         // [2]
        status_done,      // [1]
        core_busy         // [0]
    };

    function [31:0] reg_read_mux;
        input [11:0] addr;
        begin
            case (addr)
                ADDR_CTRL:     reg_read_mux = {29'h0, reg_dma_en, 1'b0, 1'b0};
                ADDR_STATUS:   reg_read_mux = status_word;
                ADDR_MODE:     reg_read_mux = {30'h0, reg_mode};
                ADDR_IRQ_EN:   reg_read_mux = {29'h0, reg_irq_en};
                ADDR_DATA_LEN: reg_read_mux = {25'h0, reg_data_len};   // FIX-BUG2
                ADDR_KEY_0:    reg_read_mux = reg_key_0;
                ADDR_KEY_1:    reg_read_mux = reg_key_1;
                ADDR_KEY_2:    reg_read_mux = reg_key_2;
                ADDR_KEY_3:    reg_read_mux = reg_key_3;
                ADDR_NONCE_0:  reg_read_mux = reg_nonce_0;
                ADDR_NONCE_1:  reg_read_mux = reg_nonce_1;
                ADDR_NONCE_2:  reg_read_mux = reg_nonce_2;
                ADDR_NONCE_3:  reg_read_mux = reg_nonce_3;
                ADDR_PTEXT_0, ADDR_PTEXT_1: reg_read_mux = 32'h0;
                ADDR_CTEXT_0: reg_read_mux = reg_ctext_0;
                ADDR_CTEXT_1: reg_read_mux = reg_ctext_1;
                ADDR_TAG_0:   reg_read_mux = reg_tag_0;
                ADDR_TAG_1:   reg_read_mux = reg_tag_1;
                ADDR_TAG_2:   reg_read_mux = reg_tag_2;
                ADDR_TAG_3:   reg_read_mux = reg_tag_3;
                ADDR_DMA_SRC: reg_read_mux = reg_dma_src;
                ADDR_DMA_DST: reg_read_mux = reg_dma_dst;
                ADDR_DMA_LEN: reg_read_mux = reg_dma_len;
                ADDR_DMA_BURST: reg_read_mux = {24'h0, reg_dma_burst_len};
                default:      reg_read_mux = 32'h0;
            endcase
        end
    endfunction

    // =========================================================================
    // WRITE CHANNEL FSM  (FIX-BUG1: AXI4-Full burst support)
    //
    // Burst model: slave hỗ trợ INCR và FIXED burst.
    // Với register-mapped IP, tất cả beat trong burst đều ghi vào cùng
    // địa chỉ wr_addr_lat (simplified). Decode chỉ khi WLAST=1.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state      <= WR_IDLE;
            S_AXI_AWREADY <= 1'b1;
            S_AXI_WREADY  <= 1'b1;
            S_AXI_BVALID  <= 1'b0;
            S_AXI_BRESP   <= 2'b00;
            S_AXI_BID     <= {ID_WIDTH{1'b0}};
            wr_addr_lat   <= 12'h0;
            wr_id_lat     <= {ID_WIDTH{1'b0}};
            wr_data_lat   <= 32'h0;
            wr_strb_lat   <= 4'h0;
            // wr_len_lat removed
            // wr_beat_cnt removed
            wr_wlast_lat  <= 1'b0;
            // Storage registers
            reg_mode      <= 2'h0;
            reg_irq_en    <= 3'h0;
            reg_dma_en    <= 1'b0;
            reg_data_len  <= 7'd8;   // default 8 bytes
            reg_key_0     <= 32'h0; reg_key_1   <= 32'h0;
            reg_key_2     <= 32'h0; reg_key_3   <= 32'h0;
            reg_nonce_0   <= 32'h0; reg_nonce_1 <= 32'h0;
            reg_nonce_2   <= 32'h0; reg_nonce_3 <= 32'h0;
            reg_ptext_0   <= 32'h0; reg_ptext_1 <= 32'h0;
            reg_dma_src   <= 32'h0;
            reg_dma_dst   <= 32'h0;
            reg_dma_len   <= 32'd8;
            reg_dma_burst_len <= 8'h0;
            core_start    <= 1'b0;
            core_soft_rst <= 1'b0;
            dma_start     <= 1'b0;
            dma_soft_rst  <= 1'b0;
        end else begin
            // Default: deassert 1-cycle control pulses every cycle
            core_start    <= 1'b0;
            core_soft_rst <= 1'b0;
            dma_start     <= 1'b0;
            dma_soft_rst  <= 1'b0;

            // [BUG3-FIX] Clear reg_dma_en on soft_rst (moved here from status block
            // to avoid multiple-driver conflict on reg_dma_en).
            if (core_soft_rst)
                reg_dma_en <= 1'b0;

            case (wr_state)

                // ── WR_IDLE ──────────────────────────────────────────────────
                // Accept AW. Pre-latch W nếu đến cùng cycle (AXI4 chuẩn).
                // WREADY=1 tại đây (từ reset hoặc sau WR_DONE).
                WR_IDLE: begin
                    // Pre-latch W nếu valid trước hoặc cùng lúc AW
                    if (S_AXI_WVALID && S_AXI_WREADY) begin
                        wr_data_lat  <= S_AXI_WDATA;
                        wr_strb_lat  <= S_AXI_WSTRB;
                        wr_wlast_lat <= S_AXI_WLAST;  // [FIX-WLAST]
                        S_AXI_WREADY <= 1'b0;   // đã latch W, không nhận thêm
                    end
                    if (S_AXI_AWVALID && S_AXI_AWREADY) begin
                        wr_addr_lat   <= S_AXI_AWADDR[11:0];
                        wr_id_lat     <= S_AXI_AWID;
                        S_AXI_AWREADY <= 1'b0;
                        S_AXI_WREADY  <= 1'b1;   // [FIX-INCR-BURST] mở sẵn WREADY để nhận burst
                        wr_state      <= WR_DATA;
                    end
                end

                // ── WR_DATA ──────────────────────────────────────────────────
                // [FIX-INCR-BURST] Hỗ trợ INCR burst từ CPU memcpy (nạp Key/Nonce).
                // Mỗi beat WVALID ghi vào wr_addr_lat, sau đó tăng địa chỉ +4 byte.
                // Chỉ chuyển sang WR_RESP khi WLAST=1 (beat cuối của burst).
                // Pre-latch path (WREADY=0, wr_wlast_lat=1): đường A như cũ.
                WR_DATA: begin
                    if (S_AXI_WVALID && S_AXI_WREADY) begin

                        // 1. Ghi dữ liệu vào Register tại địa chỉ hiện tại
                        case (wr_addr_lat)
                            ADDR_CTRL: begin
                                if (wr_exec_strb[0]) begin
                                    if (wr_exec_data[1]) begin
                                        core_soft_rst <= 1'b1;
                                        dma_soft_rst  <= 1'b1;
                                    end
                                    if (wr_exec_data[0]) begin
                                        // Sử dụng trực tiếp bit 2 của dữ liệu đang ghi để quyết định start core hay dma
                                        if (!core_busy && !wr_exec_data[2])
                                            core_start <= 1'b1;
                                        
                                        if (wr_exec_data[2] && !dma_busy)
                                            dma_start <= 1'b1;
                                    end
                                    reg_dma_en <= wr_exec_data[2];
                                end
                            end
                            ADDR_MODE:     if (wr_exec_strb[0]) reg_mode     <= wr_exec_data[1:0];
                            ADDR_IRQ_EN:   if (wr_exec_strb[0]) reg_irq_en   <= wr_exec_data[2:0];
                            ADDR_DATA_LEN: if (wr_exec_strb[0]) reg_data_len <= wr_exec_data[6:0];
                            ADDR_KEY_0:    reg_key_0   <= apply_strb(reg_key_0,   wr_exec_data, wr_exec_strb);
                            ADDR_KEY_1:    reg_key_1   <= apply_strb(reg_key_1,   wr_exec_data, wr_exec_strb);
                            ADDR_KEY_2:    reg_key_2   <= apply_strb(reg_key_2,   wr_exec_data, wr_exec_strb);
                            ADDR_KEY_3:    reg_key_3   <= apply_strb(reg_key_3,   wr_exec_data, wr_exec_strb);
                            ADDR_NONCE_0:  reg_nonce_0 <= apply_strb(reg_nonce_0, wr_exec_data, wr_exec_strb);
                            ADDR_NONCE_1:  reg_nonce_1 <= apply_strb(reg_nonce_1, wr_exec_data, wr_exec_strb);
                            ADDR_NONCE_2:  reg_nonce_2 <= apply_strb(reg_nonce_2, wr_exec_data, wr_exec_strb);
                            ADDR_NONCE_3:  reg_nonce_3 <= apply_strb(reg_nonce_3, wr_exec_data, wr_exec_strb);
                            ADDR_PTEXT_0:  reg_ptext_0 <= apply_strb(reg_ptext_0, wr_exec_data, wr_exec_strb);
                            ADDR_PTEXT_1:  reg_ptext_1 <= apply_strb(reg_ptext_1, wr_exec_data, wr_exec_strb);
                            ADDR_DMA_SRC:  reg_dma_src <= apply_strb(reg_dma_src, wr_exec_data, wr_exec_strb);
                            ADDR_DMA_DST:  reg_dma_dst <= apply_strb(reg_dma_dst, wr_exec_data, wr_exec_strb);
                            ADDR_DMA_LEN:  reg_dma_len <= apply_strb(reg_dma_len, wr_exec_data, wr_exec_strb);
                            ADDR_DMA_BURST: begin
                                if (wr_exec_strb[0]) reg_dma_burst_len <= wr_exec_data[7:0];
                            end
                            default: ;
                        endcase

                        // 2. Xử lý AXI Burst
                        if (S_AXI_WLAST) begin
                            // Beat cuối: phát response
                            S_AXI_BID    <= wr_id_lat;
                            S_AXI_BRESP  <= 2'b00;
                            S_AXI_BVALID <= 1'b1;
                            S_AXI_WREADY <= 1'b0;
                            wr_state     <= WR_RESP;
                        end else begin
                            // Còn beat tiếp: tăng địa chỉ +4 (32-bit bus = 4 byte/beat)
                            // WHY: CPU memcpy phát INCR burst liên tiếp để nạp Key[127:0]
                            //      (4 word × 4 byte = 16 byte). Không tăng địa chỉ →
                            //      tất cả 4 beat đều ghi vào KEY_0, bỏ mất KEY_1/2/3.
                            wr_addr_lat  <= wr_addr_lat + 12'h4;
                            S_AXI_WREADY <= 1'b1;
                            wr_state     <= WR_DATA;
                        end

                    end else if (!S_AXI_WREADY && wr_wlast_lat) begin
                        // Pre-latch path (case A): W đã latch tại WR_IDLE với WLAST=1
                        S_AXI_BID    <= wr_id_lat;
                        S_AXI_BRESP  <= 2'b00;
                        S_AXI_BVALID <= 1'b1;
                        wr_state     <= WR_RESP;
                    end
                end

                // ── WR_RESP ───────────────────────────────────────────────────
                // [FIX-BUG-AWREADY] KHÔNG set AWREADY tại đây.
                // Nếu AWREADY=1 trong khi BVALID=1 còn chưa clear (WR_DONE cycle),
                // master có thể handshake AW nhưng WR_DONE không latch → transaction LOST.
                // AWREADY=1 được chuyển sang WR_DONE sau khi BVALID đã clear.
                WR_RESP: begin
                    if (S_AXI_BREADY) begin
                        S_AXI_WREADY  <= 1'b1;
                        wr_state      <= WR_DONE;
                    end
                end

                // ── WR_DONE ───────────────────────────────────────────────────
                // [FIX-BUG-AWREADY] BVALID clear TRƯỚC khi AWREADY=1.
                // Đảm bảo master không thể handshake AW mới trong cycle này
                // vì AWREADY chỉ vừa được set → master sẽ thấy AWREADY=1 ở cycle SAU.
                WR_DONE: begin
                    S_AXI_BVALID  <= 1'b0;
                    S_AXI_AWREADY <= 1'b1;   // [FIX] set sau khi BVALID=0
                    wr_state      <= WR_IDLE;
                end

                default: begin
                    wr_state      <= WR_IDLE;
                    S_AXI_AWREADY <= 1'b1;
                    S_AXI_WREADY  <= 1'b1;
                end
            endcase
        end
    end

    // =========================================================================
    // Status sticky bits + ctext/tag capture
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            status_done      <= 1'b0; status_dma_done  <= 1'b0;
            status_error     <= 1'b0; status_dma_error <= 1'b0;
            reg_ctext_0 <= 32'h0; reg_ctext_1 <= 32'h0;
            reg_tag_0   <= 32'h0; reg_tag_1   <= 32'h0;
            reg_tag_2   <= 32'h0; reg_tag_3   <= 32'h0;
        end else begin
            if (core_soft_rst) begin
                // Reset có ưu tiên cao nhất, xóa tất cả các bit sticky
                status_done      <= 1'b0;
                status_dma_done  <= 1'b0;
                status_error     <= 1'b0;
                status_dma_error <= 1'b0;
            end else begin
                // Latch dữ liệu CT/Tag
                if (core_data_out_valid) begin
                    reg_ctext_0 <= core_data_out[127:96];
                    reg_ctext_1 <= core_data_out[95:64];
                end
                if (core_tag_valid) begin
                    reg_tag_0 <= core_tag_out[127:96];
                    reg_tag_1 <= core_tag_out[95:64];
                    reg_tag_2 <= core_tag_out[63:32];
                    reg_tag_3 <= core_tag_out[31:0];
                end
                
                // Latch trạng thái (Sticky Bits) - Chỉ set, không tự động clear
                if (core_done)  status_done      <= 1'b1;
                if (dma_done)   status_dma_done  <= 1'b1;
                if (dma_error)  status_dma_error <= 1'b1;
            end
        end
    end

    // =========================================================================
    // READ CHANNEL FSM  (FIX-BUG1: AXI4-Full burst support)
    //
    // Burst model: phát (ARLEN+1) beat, tất cả cùng dữ liệu từ rd_data_lat
    // (register-read pattern — địa chỉ không increment cho register-mapped IP).
    // RLAST = 1 tại beat cuối.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state      <= RD_IDLE;
            S_AXI_ARREADY <= 1'b1;
            S_AXI_RVALID  <= 1'b0;
            S_AXI_RLAST   <= 1'b0;
            S_AXI_RDATA   <= {DATA_WIDTH{1'b0}};
            S_AXI_RRESP   <= 2'b00;
            S_AXI_RID     <= {ID_WIDTH{1'b0}};
            rd_addr_lat   <= 12'h0;
            rd_id_lat     <= {ID_WIDTH{1'b0}};
            rd_data_lat   <= {DATA_WIDTH{1'b0}};
            rd_len_lat    <= 8'h0;
            rd_beat_cnt   <= 8'h0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (S_AXI_ARVALID && S_AXI_ARREADY) begin
                        rd_addr_lat   <= S_AXI_ARADDR[11:0];
                        rd_id_lat     <= S_AXI_ARID;
                        rd_data_lat   <= reg_read_mux(S_AXI_ARADDR[11:0]);
                        rd_len_lat    <= S_AXI_ARLEN;   // FIX-BUG1
                        rd_beat_cnt   <= 8'h0;
                        S_AXI_ARREADY <= 1'b0;
                        rd_state      <= RD_VALID;
                    end
                end

                RD_VALID: begin
                    if (!S_AXI_RVALID) begin
                        S_AXI_RVALID <= 1'b1;
                        S_AXI_RDATA  <= rd_data_lat;
                        S_AXI_RRESP  <= 2'b00;
                        S_AXI_RID    <= rd_id_lat;
                        // FIX-BUG1: set RLAST=1 when this is the last beat
                        S_AXI_RLAST  <= (rd_beat_cnt == rd_len_lat) ? 1'b1 : 1'b0;
                    end else if (S_AXI_RREADY) begin
                        if (rd_beat_cnt == rd_len_lat) begin
                            // Last beat acknowledged
                            S_AXI_RVALID  <= 1'b0;
                            S_AXI_RLAST   <= 1'b0;
                            S_AXI_ARREADY <= 1'b1;
                            rd_state      <= RD_IDLE;
                        end else begin
                            // More beats to send
                            rd_beat_cnt  <= rd_beat_cnt + 8'h1;
                            S_AXI_RDATA  <= rd_data_lat;  // same register data
                            S_AXI_RLAST  <= (rd_beat_cnt + 8'h1 == rd_len_lat) ? 1'b1 : 1'b0;
                        end
                    end
                end

                default: begin
                    rd_state      <= RD_IDLE;
                    S_AXI_ARREADY <= 1'b1;
                    S_AXI_RLAST   <= 1'b0;
                end
            endcase
        end
    end

    // =========================================================================
    // Output wires to ascon_CORE
    // =========================================================================
    assign core_key      = {reg_key_0,   reg_key_1,   reg_key_2,   reg_key_3};
    assign core_nonce    = {reg_nonce_0, reg_nonce_1, reg_nonce_2, reg_nonce_3};
    assign core_data_in  = {reg_ptext_0, reg_ptext_1, 64'h0};
    assign core_data_len = reg_data_len;   // FIX-BUG2: từ register, không hardcode
    // FIX-MODE-OVERLAP: tách 2 chức năng ra 2 bit riêng biệt
    //   reg_mode[0] → core_mode[0]: chọn ASCON variant (128=0 / 128a=1)
    //   reg_mode[1] → core_enc_dec: chọn hướng         (Encrypt=0 / Decrypt=1)
    assign core_enc_dec  = reg_mode[1];   // bit[1] = direction
    assign core_mode     = reg_mode;      // bit[0] = variant (dùng bởi CORE nội bộ)

    // =========================================================================
    // Output wires to ascon_dma
    // =========================================================================
    assign dma_src_addr = reg_dma_src;
    assign dma_dst_addr = reg_dma_dst;
    assign dma_length   = reg_dma_len;
    assign dma_burst_len= reg_dma_burst_len;
    assign dma_en       = reg_dma_en;

    // =========================================================================
    // Interrupt
    // =========================================================================
    assign irq = (status_done      & reg_irq_en[0]) |
                 (status_dma_done  & reg_irq_en[1]) |
                 (status_error     & reg_irq_en[2]) |
                 (status_dma_error & reg_irq_en[2]);

endmodule