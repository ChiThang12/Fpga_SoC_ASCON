module LSU (
    input wire clk,
    input wire rst,

    input  wire        req_valid,
    output wire        req_ready,
    input  wire [31:0] req_addr,
    input  wire [31:0] req_wdata,
    input  wire [3:0]  req_wstrb,
    input  wire        req_is_load,
    input  wire [4:0]  req_rd,
    input  wire [2:0]  req_funct3,

    input  wire        fence,

    output reg         result_valid,
    output reg  [31:0] result_data,
    output reg  [4:0]  result_rd,
    input  wire        result_ack,

    output wire [31:0] scoreboard,
    output wire        lsu_idle,

    output reg         dcache_req,
    output reg         dcache_we,
    output reg  [31:0] dcache_addr,
    output reg  [31:0] dcache_wdata,
    output reg  [3:0]  dcache_wstrb,
    input  wire [31:0] dcache_rdata,
    input  wire        dcache_ready
);

    localparam SB_DEPTH = 4;
    localparam SB_BITS  = 2;

    reg [31:0] sb_addr  [0:SB_DEPTH-1];
    reg [31:0] sb_wdata [0:SB_DEPTH-1];
    reg [3:0]  sb_wstrb [0:SB_DEPTH-1];
    reg        sb_valid [0:SB_DEPTH-1];

    reg [SB_BITS-1:0] sb_wr_ptr;
    reg [SB_BITS-1:0] sb_rd_ptr;
    reg [SB_BITS:0]   sb_count;

    wire sb_full  = (sb_count == SB_DEPTH);
    wire sb_empty = (sb_count == 0);

    localparam LQ_DEPTH = 4;
    localparam LQ_BITS  = 2;

    reg [31:0] lq_addr    [0:LQ_DEPTH-1];
    reg [4:0]  lq_rd      [0:LQ_DEPTH-1];
    reg [2:0]  lq_funct3  [0:LQ_DEPTH-1];
    reg        lq_fwd     [0:LQ_DEPTH-1];
    reg [31:0] lq_fwd_data[0:LQ_DEPTH-1];

    reg [LQ_BITS-1:0] lq_wr_ptr;
    reg [LQ_BITS-1:0] lq_rd_ptr;
    reg [LQ_BITS:0]   lq_count;

    wire lq_full  = (lq_count == LQ_DEPTH);
    wire lq_empty = (lq_count == 0);

    reg [31:0] scoreboard_reg;
    assign scoreboard = scoreboard_reg;

    // lsu_idle: không có in-flight transaction nào
    // Dùng bởi hazard_detection để biết khi nào FENCE có thể commit
    assign lsu_idle = sb_empty && lq_empty &&
                      (load_state == LOAD_IDLE) &&
                      (drain_state == DRAIN_IDLE) &&
                      !result_valid;

    // Khi fence active: không nhận request mới
    assign req_ready = fence ? 1'b0 :
                       (req_is_load ? !lq_full : !sb_full);

    wire        fwd_hit;
    wire [31:0] fwd_data;
    reg         fwd_hit_r;
    reg  [31:0] fwd_data_r;
    reg  [3:0]  fwd_strb_r;    // [FIX-FWD] track which byte lanes are covered

    // =========================================================================
    // [FIX-FWD] Store-to-Load Forwarding — byte-lane merge
    //
    // TRƯỚC: fwd_data_r = sb_wdata[fi] (nguyên word) → nếu store buffer chứa
    //        sb (wstrb=0010), forward trả 0x0000XX00 cho cả word → byte 0,2,3
    //        bị zero thay vì giữ giá trị cũ từ cache.
    //
    // SAU:   Merge từng byte lane từ tất cả store buffer entries cùng word addr.
    //        Entry sau (index cao hơn nếu cùng addr) ghi đè entry trước — đúng
    //        thứ tự program order vì sb_wr_ptr tăng dần.
    //
    //        fwd_strb_r track byte nào đã được cover. Nếu < 4'b1111 (partial),
    //        KHÔNG forward — phải đợi store drain rồi đọc cache để lấy đủ data.
    //        Lý do: LSU không có đường đọc cache song song để merge.
    //
    // TRADE-OFF: Partial forward bị disable → sb store phải drain trước khi
    //   load cùng word addr hoàn thành. Tăng latency ~vài cycle nhưng đúng.
    //   Full partial merge cần đọc cache + merge = thay đổi kiến trúc lớn.
    // =========================================================================
    integer fi;
    always @(*) begin
        fwd_hit_r  = 1'b0;
        fwd_data_r = 32'h0;
        fwd_strb_r = 4'b0000;
        for (fi = 0; fi < SB_DEPTH; fi = fi + 1) begin
            if (sb_valid[fi] && (sb_addr[fi][31:2] == req_addr[31:2])) begin
                fwd_hit_r = 1'b1;
                // Merge từng byte: entry mới (index cao) ghi đè entry cũ
                if (sb_wstrb[fi][0]) begin fwd_data_r[7:0]   = sb_wdata[fi][7:0];   fwd_strb_r[0] = 1'b1; end
                if (sb_wstrb[fi][1]) begin fwd_data_r[15:8]  = sb_wdata[fi][15:8];  fwd_strb_r[1] = 1'b1; end
                if (sb_wstrb[fi][2]) begin fwd_data_r[23:16] = sb_wdata[fi][23:16]; fwd_strb_r[2] = 1'b1; end
                if (sb_wstrb[fi][3]) begin fwd_data_r[31:24] = sb_wdata[fi][31:24]; fwd_strb_r[3] = 1'b1; end
            end
        end
    end
    // [FIX-FWD] Chỉ forward khi toàn bộ 4 byte lanes đều có trong store buffer.
    // Partial match → fwd_hit=0 → load đợi store drain, rồi đọc cache (đúng data).
    assign fwd_hit  = fwd_hit_r && (fwd_strb_r == 4'b1111);
    assign fwd_data = fwd_data_r;

    wire do_store = req_valid && req_ready && !req_is_load;
    wire do_load  = req_valid && req_ready &&  req_is_load;

    localparam [1:0]
        LOAD_IDLE   = 2'b00,
        LOAD_DCACHE = 2'b01,
        LOAD_RESULT = 2'b10;

    localparam
        DRAIN_IDLE = 1'b0,
        DRAIN_REQ  = 1'b1;

    reg [1:0] load_state;
    reg       drain_state;

    wire load_using_dcache = (load_state == LOAD_DCACHE);

    reg [31:0] cur_load_addr;
    reg [4:0]  cur_load_rd;
    reg [2:0]  cur_load_funct3;

    wire load_fsm_ready    = (load_state == LOAD_IDLE) && !result_valid;
    wire do_load_dequeue   = !lq_empty && load_fsm_ready && !fence;

    wire do_drain_pop = (drain_state == DRAIN_REQ)
                      && dcache_ready
                      && !load_using_dcache;

    function [31:0] apply_funct3;
        input [31:0] raw;
        input [2:0]  f3;
        case (f3)
            3'b000:  apply_funct3 = {{24{raw[7]}},  raw[7:0]};
            3'b001:  apply_funct3 = {{16{raw[15]}}, raw[15:0]};
            3'b010:  apply_funct3 = raw;
            3'b100:  apply_funct3 = {24'h0, raw[7:0]};
            3'b101:  apply_funct3 = {16'h0, raw[15:0]};
            default: apply_funct3 = raw;
        endcase
    endfunction

    integer si;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sb_wr_ptr      <= 0;
            sb_rd_ptr      <= 0;
            sb_count       <= 0;
            lq_wr_ptr      <= 0;
            lq_rd_ptr      <= 0;
            lq_count       <= 0;
            scoreboard_reg <= 32'h0;
            cur_load_addr   <= 32'h0;
            cur_load_rd     <= 5'h0;
            cur_load_funct3 <= 3'h0;
            for (si = 0; si < SB_DEPTH; si = si + 1)
                sb_valid[si] <= 1'b0;
        end else begin
            if (do_store) begin
                sb_addr  [sb_wr_ptr] <= req_addr;
                sb_wdata [sb_wr_ptr] <= req_wdata;
                sb_wstrb [sb_wr_ptr] <= req_wstrb;
                sb_valid [sb_wr_ptr] <= 1'b1;
                sb_wr_ptr <= sb_wr_ptr + 1'b1;
            end

            if (do_drain_pop) begin
                sb_valid[sb_rd_ptr] <= 1'b0;
                sb_rd_ptr <= sb_rd_ptr + 1'b1;
            end

            case ({do_store, do_drain_pop})
                2'b10:   sb_count <= sb_count + 1'b1;
                2'b01:   sb_count <= sb_count - 1'b1;
                default: ;
            endcase

            if (do_load) begin
                lq_addr    [lq_wr_ptr] <= req_addr;
                lq_rd      [lq_wr_ptr] <= req_rd;
                lq_funct3  [lq_wr_ptr] <= req_funct3;
                lq_fwd     [lq_wr_ptr] <= fwd_hit;
                lq_fwd_data[lq_wr_ptr] <= fwd_hit ? fwd_data : 32'h0;
                lq_wr_ptr  <= lq_wr_ptr + 1'b1;
                if (req_rd != 5'b0)
                    scoreboard_reg[req_rd] <= 1'b1;
            end

            if (do_load_dequeue) begin
                cur_load_addr   <= lq_addr  [lq_rd_ptr];
                cur_load_rd     <= lq_rd    [lq_rd_ptr];
                cur_load_funct3 <= lq_funct3[lq_rd_ptr];
                lq_rd_ptr <= lq_rd_ptr + 1'b1;
            end

            case ({do_load, do_load_dequeue})
                2'b10:   lq_count <= lq_count + 1'b1;
                2'b01:   lq_count <= lq_count - 1'b1;
                default: ;
            endcase

            if (result_valid && result_ack) begin
                if (result_rd != 5'b0)
                    scoreboard_reg[result_rd] <= 1'b0;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            load_state   <= LOAD_IDLE;
            result_valid <= 1'b0;
            result_data  <= 32'h0;
            result_rd    <= 5'h0;
        end else begin
            case (load_state)
                LOAD_IDLE: begin
                    if (do_load_dequeue) begin
                        if (lq_fwd[lq_rd_ptr]) begin
                            result_valid <= 1'b1;
                            result_rd    <= lq_rd[lq_rd_ptr];
                            result_data  <= apply_funct3(
                                               lq_fwd_data[lq_rd_ptr],
                                               lq_funct3  [lq_rd_ptr]);
                            load_state   <= LOAD_RESULT;
                        end else begin
                            load_state <= LOAD_DCACHE;
                        end
                    end
                end

                LOAD_DCACHE: begin
                    if (dcache_ready) begin
                        result_valid <= 1'b1;
                        result_rd    <= cur_load_rd;
                        result_data  <= apply_funct3(dcache_rdata, cur_load_funct3);
                        load_state   <= LOAD_RESULT;
                    end
                end

                LOAD_RESULT: begin
                    if (result_ack) begin
                        result_valid <= 1'b0;
                        load_state   <= LOAD_IDLE;
                    end
                end

                default: load_state <= LOAD_IDLE;
            endcase
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            drain_state <= DRAIN_IDLE;
        end else begin
            case (drain_state)
                DRAIN_IDLE: begin
                    if (!sb_empty && !load_using_dcache)
                        drain_state <= DRAIN_REQ;
                end

                DRAIN_REQ: begin
                    if (load_using_dcache) begin
                        drain_state <= DRAIN_IDLE;
                    end else if (dcache_ready) begin
                        drain_state <= DRAIN_IDLE;
                    end
                end

                default: drain_state <= DRAIN_IDLE;
            endcase
        end
    end

    always @(*) begin
        dcache_req   = 1'b0;
        dcache_we    = 1'b0;
        dcache_addr  = 32'h0;
        dcache_wdata = 32'h0;
        dcache_wstrb = 4'h0;

        if (load_using_dcache) begin
            dcache_req  = 1'b1;
            dcache_we   = 1'b0;
            dcache_addr = cur_load_addr;
        end else if (drain_state == DRAIN_REQ) begin
            dcache_req   = 1'b1;
            dcache_we    = 1'b1;
            dcache_addr  = sb_addr [sb_rd_ptr];
            dcache_wdata = sb_wdata[sb_rd_ptr];
            dcache_wstrb = sb_wstrb[sb_rd_ptr];
        end
    end

endmodule