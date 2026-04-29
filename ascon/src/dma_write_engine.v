// ============================================================================
// Module  : dma_write_engine
// Version : v4.0  (FWFT + Dynamic AXI Burst)
//
// CHANGES from v3.0:
//   [OPT-1] FWFT read: uses fifo_fwft_dout (combinational) instead of
//           fifo_dout (registered). Eliminates WR_WAIT_H + WR_LATCH_H +
//           WR_WAIT_L states → 3 fewer cycles per AXI beat.
//
//   [OPT-2] Dynamic AXI burst: AWLEN = min(remaining_beats-1, MAX_BURST_LEN).
//           With MAX_BURST_LEN=15, CT (16 beats) → AWLEN=15 (1 transaction).
//           TAG (2 beats) → AWLEN=1. Only 2 AXI transactions vs 18 before.
//           Amortises IDLE+ADDR+RESP overhead across 16 beats instead of 1.
//
//   [OPT-3] New input: total_wr_beats [28:0]. Initialises remaining_beats
//           counter on dma_start. Enables per-transaction AWLEN computation
//           without changing ctrl_fsm interface.
//
//   wr_done still pulsed once per AXI W handshake (not per transaction)
//   → fully compatible with dma_ctrl_fsm beat-counting logic.
//
// FSM states (6):
//   WR_IDLE   — wait for fifo_count>=2 && remaining_beats>0; issue AWVALID
//   WR_ADDR   — wait AWREADY; latch H via FWFT (or go WR_LOAD_H if empty)
//   WR_LOAD_H — stall until H word appears in FIFO; latch + pop
//   WR_DATA_L — wait for L word; form 64-bit WDATA; assert WVALID+WLAST
//   WR_BEAT   — wait WREADY; pulse wr_done; loop back or go WR_RESP
//   WR_RESP   — wait BVALID; update address; decrement remaining_beats
//
// Per-beat timing (no stall, assuming AWREADY/WREADY/BVALID immediate):
//   First beat of burst: WR_ADDR(1)+WR_DATA_L(1)+WR_BEAT(1) = 3 cycles
//   Each subsequent   : WR_LOAD_H(1)+WR_DATA_L(1)+WR_BEAT(1) = 3 cycles
//   Plus overhead     : WR_IDLE(1)+WR_RESP(1) amortised over burst_beats
//
// ============================================================================

module dma_write_engine #(
    parameter ADDR_WIDTH     = 32,
    parameter AXI_DATA_WIDTH = 64,
    parameter AXI_ID_WIDTH   = 4,
    parameter MAX_BURST_LEN  = 8'd15   // max AWLEN value (0-based); 15 → 16 beats
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // ── Control ───────────────────────────────────────────────────────────────
    input  wire [ADDR_WIDTH-1:0]       dst_addr,
    input  wire                        dma_start,      // pulse: reset addr + beat counter
    input  wire [28:0]                 total_wr_beats, // total beats = total_blocks + 2
    output reg                         wr_busy,
    output reg                         wr_done,        // pulsed once per W handshake
    output reg                         wr_error,
    output reg  [ADDR_WIDTH-1:0]       wr_err_addr,

    // ── WR FIFO registered path (kept for port compatibility, unused in logic) ─
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]                 fifo_dout,
    /* verilator lint_on UNUSEDSIGNAL */
    output reg                         fifo_pop,
    input  wire [5:0]                  fifo_count,

    // ── WR FIFO FWFT path (combinational — zero latency) ──────────────────────
    input  wire [31:0]                 fifo_fwft_dout,
    input  wire                        fifo_fwft_valid,

    // ── AXI4 Write Address Channel ────────────────────────────────────────────
    output reg  [AXI_ID_WIDTH-1:0]     M_AXI_AWID,
    output reg  [ADDR_WIDTH-1:0]       M_AXI_AWADDR,
    output wire [7:0]                  M_AXI_AWLEN,    // driven from awlen_reg
    output wire [2:0]                  M_AXI_AWSIZE,
    output wire [1:0]                  M_AXI_AWBURST,
    output wire [3:0]                  M_AXI_AWCACHE,
    output wire [2:0]                  M_AXI_AWPROT,
    output reg                         M_AXI_AWVALID,
    input  wire                        M_AXI_AWREADY,

    // ── AXI4 Write Data Channel ───────────────────────────────────────────────
    output reg  [AXI_DATA_WIDTH-1:0]   M_AXI_WDATA,
    output wire [AXI_DATA_WIDTH/8-1:0] M_AXI_WSTRB,
    output reg                         M_AXI_WLAST,
    output reg                         M_AXI_WVALID,
    input  wire                        M_AXI_WREADY,

    // ── AXI4 Write Response Channel ───────────────────────────────────────────
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [AXI_ID_WIDTH-1:0]     M_AXI_BID,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [1:0]                  M_AXI_BRESP,
    input  wire                        M_AXI_BVALID,
    output reg                         M_AXI_BREADY
);

    // ── Fixed AXI parameters (combinational assigns) ─────────────────────────
    reg [7:0] awlen_reg;
    assign M_AXI_AWLEN   = awlen_reg;
    assign M_AXI_AWSIZE  = 3'b011;         // 8 bytes/beat
    assign M_AXI_AWBURST = 2'b01;          // INCR
    assign M_AXI_AWCACHE = 4'b0010;        // Normal Non-cacheable Bufferable
    assign M_AXI_AWPROT  = 3'b000;
    assign M_AXI_WSTRB   = {(AXI_DATA_WIDTH/8){1'b1}};

    // ── FSM state encoding ────────────────────────────────────────────────────
    localparam [2:0]
        WR_IDLE   = 3'd0,
        WR_ADDR   = 3'd1,
        WR_LOAD_H = 3'd2,
        WR_DATA_L = 3'd3,
        WR_BEAT   = 3'd4,
        WR_RESP   = 3'd5;

    reg [2:0]            state;
    reg [31:0]           wdata_hi;          // latched high word of current beat
    reg [ADDR_WIDTH-1:0] cur_dst_addr;

    // ── Beat tracking registers ───────────────────────────────────────────────
    reg [28:0] remaining_beats;   // beats still to be written this DMA operation
    reg [7:0]  beats_in_burst;    // beats remaining in current AXI transaction
    reg [7:0]  burst_beats;       // = awlen_reg + 1; used for addr/counter update

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= WR_IDLE;
            wr_busy         <= 1'b0;
            wr_done         <= 1'b0;
            wr_error        <= 1'b0;
            wr_err_addr     <= {ADDR_WIDTH{1'b0}};
            M_AXI_AWVALID   <= 1'b0;
            M_AXI_AWID      <= {AXI_ID_WIDTH{1'b0}};
            M_AXI_AWADDR    <= {ADDR_WIDTH{1'b0}};
            awlen_reg       <= 8'd0;
            M_AXI_WVALID    <= 1'b0;
            M_AXI_WDATA     <= {AXI_DATA_WIDTH{1'b0}};
            M_AXI_WLAST     <= 1'b0;
            M_AXI_BREADY    <= 1'b0;
            fifo_pop        <= 1'b0;
            wdata_hi        <= 32'h0;
            cur_dst_addr    <= {ADDR_WIDTH{1'b0}};
            remaining_beats <= 29'd0;
            beats_in_burst  <= 8'd0;
            burst_beats     <= 8'd0;
        end else begin
            // ── Default: clear 1-cycle strobes ───────────────────────────────
            wr_done  <= 1'b0;
            fifo_pop <= 1'b0;

            // ── dma_start: highest priority — reinitialise for new DMA op ────
            if (dma_start) begin
                state           <= WR_IDLE;
                cur_dst_addr    <= dst_addr;
                remaining_beats <= total_wr_beats;
                wr_error        <= 1'b0;
                wr_busy         <= 1'b0;
                M_AXI_AWVALID   <= 1'b0;
                M_AXI_WVALID    <= 1'b0;
                M_AXI_BREADY    <= 1'b0;
                M_AXI_WLAST     <= 1'b0;
            end else begin

                case (state)

                    // ── WR_IDLE: wait for data; compute burst; issue AWVALID ──────
                    WR_IDLE: begin
                        M_AXI_AWVALID <= 1'b0;
                        M_AXI_WVALID  <= 1'b0;
                        M_AXI_BREADY  <= 1'b0;
                        M_AXI_WLAST   <= 1'b0;

                        if (remaining_beats > 29'd0 && fifo_count >= 6'd2) begin
                            // Compute AWLEN = min(remaining_beats - 1, MAX_BURST_LEN)
                            if (remaining_beats > ({21'd0, MAX_BURST_LEN} + 29'd1)) begin
                                // remaining > MAX_BURST_LEN+1: use maximum burst
                                awlen_reg      <= MAX_BURST_LEN;
                                burst_beats    <= MAX_BURST_LEN + 8'd1;
                                beats_in_burst <= MAX_BURST_LEN + 8'd1;
                            end else begin
                                // remaining <= MAX_BURST_LEN+1: use all remaining beats
                                awlen_reg      <= remaining_beats[7:0] - 8'd1;
                                burst_beats    <= remaining_beats[7:0];
                                beats_in_burst <= remaining_beats[7:0];
                            end
                            wr_busy       <= 1'b1;
                            M_AXI_AWID    <= {AXI_ID_WIDTH{1'b0}};
                            M_AXI_AWADDR  <= cur_dst_addr;
                            M_AXI_AWVALID <= 1'b1;
                            state         <= WR_ADDR;
                        end else begin
                            wr_busy <= 1'b0;
                        end
                    end

                    // ── WR_ADDR: wait AWREADY; inline H-latch from FWFT if ready ─
                    WR_ADDR: begin
                        if (M_AXI_AWREADY && M_AXI_AWVALID) begin
                            M_AXI_AWVALID <= 1'b0;
                            if (fifo_fwft_valid) begin
                                // Fast path: H word available now (pop → L becomes head)
                                wdata_hi <= fifo_fwft_dout;
                                fifo_pop <= 1'b1;
                                state    <= WR_DATA_L;
                            end else begin
                                // Slow path: FIFO temporarily empty, wait for H
                                state <= WR_LOAD_H;
                            end
                        end
                    end

                    // ── WR_LOAD_H: stall until H word arrives; pop it ─────────────
                    WR_LOAD_H: begin
                        if (fifo_fwft_valid) begin
                            wdata_hi <= fifo_fwft_dout;
                            fifo_pop <= 1'b1;   // pop H → L becomes fwft_dout next cycle
                            state    <= WR_DATA_L;
                        end
                    end

                    // ── WR_DATA_L: stall until L word ready; build WDATA; WVALID ──
                    // After pop of H in WR_ADDR/WR_LOAD_H, rd_ptr has advanced so
                    // fwft_dout now points to L word (combinationally).
                    WR_DATA_L: begin
                        if (fifo_fwft_valid) begin
                            M_AXI_WDATA  <= {fifo_fwft_dout, wdata_hi}; // {L[31:0], H[31:0]}
                            M_AXI_WVALID <= 1'b1;
                            M_AXI_WLAST  <= (beats_in_burst == 8'd1);   // last beat of burst
                            fifo_pop     <= 1'b1;   // pop L → next H becomes fwft_dout
                            state        <= WR_BEAT;
                        end
                        // else stall: WVALID stays 0; AXI4 allows de-assertion between beats
                    end

                    // ── WR_BEAT: wait WREADY; pulse wr_done; advance to next beat ──
                    WR_BEAT: begin
                        if (M_AXI_WVALID && M_AXI_WREADY) begin
                            wr_done      <= 1'b1;   // per-beat pulse (ctrl_fsm counts 18)
                            M_AXI_WVALID <= 1'b0;
                            M_AXI_WLAST  <= 1'b0;

                            if (beats_in_burst > 8'd1) begin
                                // More beats remain in this burst
                                beats_in_burst <= beats_in_burst - 8'd1;
                                // fwft_dout now has next H word (L was popped in WR_DATA_L)
                                if (fifo_fwft_valid) begin
                                    wdata_hi <= fifo_fwft_dout;
                                    fifo_pop <= 1'b1;
                                    state    <= WR_DATA_L;
                                end else begin
                                    state <= WR_LOAD_H;
                                end
                            end else begin
                                // Last beat of this burst: wait for B response
                                beats_in_burst <= 8'd0;
                                M_AXI_BREADY   <= 1'b1;
                                state          <= WR_RESP;
                            end
                        end
                    end

                    // ── WR_RESP: wait BVALID; update address + remaining counter ───
                    WR_RESP: begin
                        if (M_AXI_BVALID) begin
                            M_AXI_BREADY <= 1'b0;
                            if (M_AXI_BRESP != 2'b00) begin
                                wr_error <= 1'b1;
                                if (!wr_error) wr_err_addr <= M_AXI_AWADDR;
                            end
                            // Advance by burst_beats * 8 bytes
                            cur_dst_addr    <= cur_dst_addr + ({24'd0, burst_beats} << 3);
                            // Decrement remaining by burst_beats
                            remaining_beats <= remaining_beats - {21'd0, burst_beats};
                            state           <= WR_IDLE;
                        end
                    end

                    default: state <= WR_IDLE;
                endcase
            end
        end
    end

endmodule
