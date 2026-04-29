// ============================================================================
// Module  : dma_read_engine
// Project : ASCON Crypto Accelerator IP
// Parent  : ascon_dma
//
// Description:
//   AXI4-Full Master read engine. Issues one read transaction to fetch
//   plaintext from memory and pushes the received data into the RD FIFO.
//
// AXI burst behaviour (configurable via burst_len):
//   ARLEN  = burst_len   (0 = 1 beat, N = N+1 beats)
//   ARSIZE = 3'b011      (8 bytes/beat — matches 64-bit bus)
//   ARBURST= 2'b01       (INCR)
//
// FSM states:
//   RD_IDLE → RD_ADDR → RD_DATA → RD_DONE → RD_IDLE
//
// Error handling:
//   [FIX-1] rd_error is now STICKY across beats (was accidentally cleared on
//           subsequent valid beats in the same burst when RRESP was OK).
//   [FIX-2] FIFO-full error now captures the current read-address beat
//           (M_AXI_ARADDR + beat_count*8) instead of the fixed ARADDR.
//   [FIX-3] rd_error is cleared at the START of a new transaction (rd_start),
//           not unconditionally each cycle, so the host can read the sticky
//           error flag after the burst completes.
//   [FIX-4] Beat counter added so multi-beat bursts push every beat into FIFO.
//           Previous code had no counter → only last beat was pushed correctly
//           (fifo_push de-asserted between beats by the default clearing).
//   [FIX-5] M_AXI_RREADY is de-asserted when FIFO is full (back-pressure),
//           preventing data loss on slow FIFOs. Previous code left RREADY high
//           and set rd_error instead — data was silently dropped.
//   [FIX-6] rd_busy is kept high through RD_DONE state so that the upstream
//           FSM cannot issue a new rd_start before rd_done is seen.
//
// ============================================================================

module dma_read_engine #(
    parameter ADDR_WIDTH     = 32,
    parameter AXI_DATA_WIDTH = 64,
    parameter AXI_ID_WIDTH   = 4
) (
    input  wire                       clk,
    input  wire                       rst_n,

    // ── Control (from DMA top FSM) ────────────────────────────────────────────
    input  wire [ADDR_WIDTH-1:0]      src_addr,
    input  wire [7:0]                 burst_len,    // ARLEN value (0 = 1 beat)
    input  wire                       dma_start,    // 1-cycle pulse: reset cur_src_addr to src_addr
    input  wire                       rd_start,     // 1-cycle pulse: begin transaction
    output reg                        rd_busy,
    output reg                        rd_done,      // 1-cycle pulse: all beats received
    output reg                        rd_error,     // sticky: AXI returned error or FIFO full
    output reg  [ADDR_WIDTH-1:0]      rd_err_addr,

    // ── RD FIFO push interface ────────────────────────────────────────────────
    output reg  [AXI_DATA_WIDTH-1:0]  fifo_din,
    output reg                        fifo_push,
    input  wire                       fifo_full,

    // ── AXI4 Read Address Channel ─────────────────────────────────────────────
    output reg  [AXI_ID_WIDTH-1:0]    M_AXI_ARID,
    output reg  [ADDR_WIDTH-1:0]      M_AXI_ARADDR,
    output reg  [7:0]                 M_AXI_ARLEN,
    output wire [2:0]                 M_AXI_ARSIZE,
    output wire [1:0]                 M_AXI_ARBURST,
    output wire [3:0]                 M_AXI_ARCACHE,
    output wire [2:0]                 M_AXI_ARPROT,
    output reg                        M_AXI_ARVALID,
    input  wire                       M_AXI_ARREADY,

    // ── AXI4 Read Data Channel ────────────────────────────────────────────────
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [AXI_ID_WIDTH-1:0]    M_AXI_RID,   // ID not checked (single outstanding)
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [AXI_DATA_WIDTH-1:0]  M_AXI_RDATA,
    input  wire [1:0]                 M_AXI_RRESP,
    input  wire                       M_AXI_RLAST,
    input  wire                       M_AXI_RVALID,
    output reg                        M_AXI_RREADY
);

    // ─── Fixed AXI parameters ────────────────────────────────────────────────
    assign M_AXI_ARSIZE  = 3'b011;   // 8 bytes/beat (64-bit bus)
    assign M_AXI_ARBURST = 2'b01;    // INCR
    assign M_AXI_ARCACHE = 4'b0010;  // Normal Non-cacheable Bufferable
    assign M_AXI_ARPROT  = 3'b000;

    // ─── FSM state encoding ───────────────────────────────────────────────────
    localparam [1:0]
        RD_IDLE = 2'd0,
        RD_ADDR = 2'd1,
        RD_DATA = 2'd2,
        RD_DONE = 2'd3;

    reg [1:0] state;

    // ─── Current source address (auto-increments per burst for multi-block DMA) ─
    reg [ADDR_WIDTH-1:0] cur_src_addr;

    // ─── Beat counter (tracks which beat of burst we are on) ─────────────────
    // [FIX-4] Needed to correctly push every beat and to compute err_addr per beat
    reg [7:0] beat_cnt;

    // ─── Registered burst_len (latched on rd_start so it cannot change mid-burst)
    /* verilator lint_off UNUSEDSIGNAL */
    reg [7:0] burst_len_r;  // safety latch, not read back in FSM
    /* verilator lint_on UNUSEDSIGNAL */

    // ─── Current beat address (base + beat_cnt * bytes_per_beat) ─────────────
    // Used only for rd_err_addr capture; AXI INCR addressing is done by slave.
    wire [ADDR_WIDTH-1:0] cur_beat_addr =
        M_AXI_ARADDR + {{(ADDR_WIDTH-11){1'b0}}, beat_cnt, 3'b000};
        // beat_cnt * 8  (3-bit left-shift, zero-extended to ADDR_WIDTH)

    // =========================================================================
    // Main FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= RD_IDLE;
            rd_busy        <= 1'b0;
            rd_done        <= 1'b0;
            rd_error       <= 1'b0;
            rd_err_addr    <= {ADDR_WIDTH{1'b0}};
            M_AXI_ARVALID  <= 1'b0;
            M_AXI_ARID     <= {AXI_ID_WIDTH{1'b0}};
            M_AXI_ARADDR   <= {ADDR_WIDTH{1'b0}};
            M_AXI_ARLEN    <= 8'h00;
            M_AXI_RREADY   <= 1'b0;
            fifo_push      <= 1'b0;
            fifo_din       <= {AXI_DATA_WIDTH{1'b0}};
            beat_cnt       <= 8'h00;
            burst_len_r    <= 8'h00;
            cur_src_addr   <= {ADDR_WIDTH{1'b0}};
        end else begin
            // ── Default: clear 1-cycle strobes ───────────────────────────────
            rd_done   <= 1'b0;
            fifo_push <= 1'b0;

            // Capture base address on DMA start (also used for simultaneous rd_start)
            if (dma_start) cur_src_addr <= src_addr;

            case (state)

                // ─────────────────────────────────────────────────────────────
                RD_IDLE: begin
                    rd_busy       <= 1'b0;
                    M_AXI_ARVALID <= 1'b0;
                    M_AXI_RREADY  <= 1'b0;
                    beat_cnt      <= 8'h00;

                    if (rd_start) begin
                        // [FIX-3] Clear error only when a new transaction begins
                        rd_error      <= 1'b0;
                        rd_err_addr   <= {ADDR_WIDTH{1'b0}};

                        rd_busy       <= 1'b1;
                        burst_len_r   <= burst_len;        // latch for safety
                        M_AXI_ARID    <= {AXI_ID_WIDTH{1'b0}};
                        // dma_start and rd_start arrive together for block-0; use src_addr directly.
                        // For blocks 1+, dma_start=0 and cur_src_addr holds the next address.
                        M_AXI_ARADDR  <= dma_start ? src_addr : cur_src_addr;
                        M_AXI_ARLEN   <= burst_len;
                        M_AXI_ARVALID <= 1'b1;
                        state         <= RD_ADDR;
                    end
                end

                // ─────────────────────────────────────────────────────────────
                RD_ADDR: begin
                    if (M_AXI_ARREADY && M_AXI_ARVALID) begin
                        M_AXI_ARVALID <= 1'b0;
                        // [FIX-5] Only assert RREADY when FIFO has space
                        M_AXI_RREADY  <= ~fifo_full;
                        state         <= RD_DATA;
                    end
                end

                // ─────────────────────────────────────────────────────────────
                RD_DATA: begin
                    // [FIX-5] Dynamic back-pressure: follow FIFO availability
                    M_AXI_RREADY <= ~fifo_full;

                    if (M_AXI_RVALID && M_AXI_RREADY) begin
                        // ── AXI response check ────────────────────────────────
                        // [FIX-1] Use |= semantics: once set, rd_error stays set
                        if (M_AXI_RRESP != 2'b00) begin
                            rd_error    <= 1'b1;
                            // Only capture the first error address
                            if (!rd_error)
                                rd_err_addr <= cur_beat_addr;
                        end

                        // ── Push data into FIFO ───────────────────────────────
                        // RREADY was only asserted when !fifo_full, so we should
                        // never arrive here with fifo_full=1.  Guard anyway.
                        if (!fifo_full) begin
                            fifo_din  <= M_AXI_RDATA;
                            fifo_push <= 1'b1;
                        end else begin
                            // [FIX-2] Capture per-beat address, not fixed ARADDR
                            rd_error <= 1'b1;
                            if (!rd_error)
                                rd_err_addr <= cur_beat_addr;
                        end

                        // [FIX-4] Advance beat counter on every accepted beat
                        beat_cnt <= beat_cnt + 8'h01;

                        // ── End-of-burst ──────────────────────────────────────
                        if (M_AXI_RLAST) begin
                            M_AXI_RREADY <= 1'b0;
                            // Advance cur_src_addr by (ARLEN+1)*8 bytes for the next burst
                            cur_src_addr <= M_AXI_ARADDR + {22'b0, burst_len_r, 3'b000} + 32'd8;
                            state        <= RD_DONE;
                        end
                    end
                end

                // ─────────────────────────────────────────────────────────────
                // Wait 1 cycle after the last fifo_push so the FIFO's wr_ptr
                // has incremented and empty=0 before asserting rd_done.
                // [FIX-6] rd_busy stays high here — cleared in IDLE entry.
                RD_DONE: begin
                    rd_done <= 1'b1;
                    // rd_busy will be cleared at the top of RD_IDLE next cycle
                    state   <= RD_IDLE;
                end

                default: state <= RD_IDLE;

            endcase
        end
    end

endmodule