// ============================================================================
// Module  : ascon_dma
// Project : ASCON Crypto Accelerator IP
// Version : 1.1  (fixes: RTL-1 dma_error wire, RTL-2 FIFO soft-reset)
//
// Description:
//   ASCON-dedicated DMA engine. Orchestrates the full data movement pipeline:
//     DDR → (AXI4 read) → RD FIFO → ascon_CORE → WR FIFO → (AXI4 write) → DDR
//
//   Register map (offset within ascon_axi_slave space, base 0x2000_0000):
//     0x100  DMA_SRC_ADDR   R/W  Source address (plaintext in DDR)
//     0x104  DMA_DST_ADDR   R/W  Destination address (ctext+tag out)
//     0x108  DMA_BYTE_LEN   R/W  Bytes to read (Phase 1: always 8)
//     0x10C  DMA_CTRL       R/W  [0]=START [1]=SOFT_RST [2]=RD_ONLY [3]=WR_ONLY
//     0x110  DMA_STATUS     RO   [0]=BUSY [1]=DONE [2]=RD_DONE [3]=WR_DONE
//                                [4]=RD_ERROR [5]=WR_ERROR [6]=FIFO_OVERFLOW
//     0x114  DMA_BURST_LEN  R/W  [7:0] AXI burst length (0=1 beat)
//     0x118  DMA_ERR_ADDR   RO   Address that caused AXI error (debug)
//
//   Register values are driven from ascon_reg_bank via the control interface.
//   This module has NO internal AXI slave — all registers live in ascon_reg_bank.
//
// Hierarchy:
//   ascon_dma
//   ├── dma_ctrl_fsm         — top control FSM (sequences all phases)
//   ├── dma_read_engine      — AXI4 master read (fetch plaintext)
//   ├── dma_write_engine     — AXI4 master write (store ctext + tag)
//   ├── sync_fifo (rd_fifo)  — 4 × 64-bit RD FIFO
//   └── sync_fifo (wr_fifo)  — 8 × 32-bit WR FIFO
//
// External interfaces:
//   Control   : from ascon_axi_slave (reg_bank) — src_addr, dst_addr, byte_len,
//               dma_start, dma_soft_rst, burst_len, dma_busy, dma_done, dma_error
//   Core      : to/from ascon_CORE
//   AXI4 Full : M_AXI_* master interface (connects to crossbar / memory)
//
// Phase 1 constraints:
//   - Single 64-bit block (8 bytes plaintext)
//   - 1-beat AXI read (ARLEN=0), 3-beat AXI write (AWLEN=2)
//   - No concurrent read/write; strictly sequential
//   - No unaligned access handling (driver must ensure 8-byte alignment)
//
// Firmware driver notes (DO NOT CHANGE without updating firmware):
//   1. Write DMA_SRC_ADDR (0x100), DMA_DST_ADDR (0x104), DMA_BYTE_LEN (0x108)
//      each separated by a STATUS read fence (AXI ordering)
//   2. Write DMA_CTRL (0x10C) bit[0]=1 to START — this is SEPARATE from
//      ASCON core CTRL at offset 0x000. Driver MUST write 0x10C, NOT 0x000.
//   3. Poll DMA_STATUS (0x110) bit[1]=DONE or bit[4:5]=ERROR
//      NOTE: CPU-side STATUS mirror is at 0x004 bits[3:2] (dma_done/dma_busy)
//            in ascon_reg_bank. Firmware polls 0x20000004 bits[3:2].
//   4. Data coherency: CPU must ensure plaintext is in DMEM SRAM (not only
//      in DCache) before DMA start. Use non-cacheable writes (MMIO path) or
//      explicit cache flush. DMA bypasses DCache and reads SRAM directly.
// ============================================================================



module ascon_dma #(
    parameter ADDR_WIDTH     = 32,
    parameter AXI_DATA_WIDTH = 64,   // AXI4 Master data bus width
    parameter AXI_ID_WIDTH   = 4,
    parameter RD_FIFO_DEPTH  = 4,    // entries (64-bit each)
    parameter WR_FIFO_DEPTH  = 8     // entries (32-bit each)
) (
    input  wire  clk,
    input  wire  rst_n,

    // =========================================================================
    // Control interface (from ascon_reg_bank via ascon_axi_slave)
    // Registers: DMA_SRC_ADDR(0x100), DMA_DST_ADDR(0x104), DMA_BYTE_LEN(0x108)
    //            DMA_CTRL(0x10C), DMA_BURST_LEN(0x114)
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  src_addr,      // DMA_SRC_ADDR
    input  wire [ADDR_WIDTH-1:0]  dst_addr,      // DMA_DST_ADDR
    input  wire [31:0]            byte_len,      // DMA_BYTE_LEN
    input  wire [7:0]             burst_len,     // DMA_BURST_LEN[7:0]

    input  wire                   dma_start,     // from DMA_CTRL[0] pulse
    input  wire                   dma_soft_rst,  // from DMA_CTRL[1] pulse

    // Status outputs → to ascon_reg_bank for DMA_STATUS register (0x110)
    output wire                   dma_busy,      // DMA_STATUS[0]
    output wire                   dma_done,      // DMA_STATUS[1]  (sticky in reg_bank)
    output wire                   dma_error,     // DMA_STATUS[4|5] aggregate

    // Full status bits for DMA_STATUS register
    output wire                   status_rd_done,       // DMA_STATUS[2]
    output wire                   status_wr_done,       // DMA_STATUS[3]
    output wire                   status_rd_error,      // DMA_STATUS[4]
    output wire                   status_wr_error,      // DMA_STATUS[5]
    output wire                   status_fifo_overflow, // DMA_STATUS[6]

    // DMA_ERR_ADDR (0x118) — address that caused AXI error
    output wire [ADDR_WIDTH-1:0]  dma_err_addr,

    // =========================================================================
    // Interface to ascon_CORE
    // =========================================================================
    output wire [31:0]            core_ptext_0,
    output wire [31:0]            core_ptext_1,
    output wire                   core_data_valid,
    input  wire                   core_data_ready,
    output wire                   core_start,
    output wire                   core_data_last,
    input  wire                   core_busy,
    input  wire                   core_done,
    input  wire                   core_data_out_valid,
    input  wire                   core_tag_valid,

    // Results from core (captured by ctrl_fsm on core_done)
    input  wire [31:0]            core_ctext_0,
    input  wire [31:0]            core_ctext_1,
    input  wire [31:0]            core_tag_0,
    input  wire [31:0]            core_tag_1,
    input  wire [31:0]            core_tag_2,
    input  wire [31:0]            core_tag_3,

    // =========================================================================
    // AXI4-Full Master Interface (to crossbar / DMEM / external memory)
    // =========================================================================

    // Write Address Channel
    output wire [AXI_ID_WIDTH-1:0]       M_AXI_AWID,
    output wire [ADDR_WIDTH-1:0]         M_AXI_AWADDR,
    output wire [7:0]                    M_AXI_AWLEN,
    output wire [2:0]                    M_AXI_AWSIZE,
    output wire [1:0]                    M_AXI_AWBURST,
    output wire [3:0]                    M_AXI_AWCACHE,
    output wire [2:0]                    M_AXI_AWPROT,
    output wire                          M_AXI_AWVALID,
    input  wire                          M_AXI_AWREADY,

    // Write Data Channel
    output wire [AXI_DATA_WIDTH-1:0]     M_AXI_WDATA,
    output wire [AXI_DATA_WIDTH/8-1:0]   M_AXI_WSTRB,
    output wire                          M_AXI_WLAST,
    output wire                          M_AXI_WVALID,
    input  wire                          M_AXI_WREADY,

    // Write Response Channel
    input  wire [AXI_ID_WIDTH-1:0]       M_AXI_BID,
    input  wire [1:0]                    M_AXI_BRESP,
    input  wire                          M_AXI_BVALID,
    output wire                          M_AXI_BREADY,

    // Read Address Channel
    output wire [AXI_ID_WIDTH-1:0]       M_AXI_ARID,
    output wire [ADDR_WIDTH-1:0]         M_AXI_ARADDR,
    output wire [7:0]                    M_AXI_ARLEN,
    output wire [2:0]                    M_AXI_ARSIZE,
    output wire [1:0]                    M_AXI_ARBURST,
    output wire [3:0]                    M_AXI_ARCACHE,
    output wire [2:0]                    M_AXI_ARPROT,
    output wire                          M_AXI_ARVALID,
    input  wire                          M_AXI_ARREADY,

    // Read Data Channel
    input  wire [AXI_ID_WIDTH-1:0]       M_AXI_RID,
    input  wire [AXI_DATA_WIDTH-1:0]     M_AXI_RDATA,
    input  wire [1:0]                    M_AXI_RRESP,
    input  wire                          M_AXI_RLAST,
    input  wire                          M_AXI_RVALID,
    output wire                          M_AXI_RREADY
);

    // =========================================================================
    // Internal wires
    // =========================================================================

    // RD FIFO (64-bit wide, 4 deep)
    wire [63:0] rd_fifo_din;
    wire        rd_fifo_push;
    wire        rd_fifo_full;
    wire [63:0] rd_fifo_dout;
    wire        rd_fifo_pop;
    wire        rd_fifo_empty;

    wire [31:0] wr_fifo_din;
    wire        wr_fifo_push;
    wire        wr_fifo_full;
    wire [31:0] wr_fifo_dout;
    wire        wr_fifo_pop;
    wire        wr_fifo_empty;
    wire [3:0]  wr_fifo_count;

    // Read engine ↔ ctrl_fsm
    wire        rd_start_w;
    wire        rd_busy_w;
    wire        rd_done_w;
    wire        rd_error_w;
    wire [ADDR_WIDTH-1:0] rd_err_addr_w;

    // Write engine ↔ ctrl_fsm (wr_start removed — write engine auto-triggers on FIFO count)
    wire        wr_busy_w;
    wire        wr_done_w;
    wire        wr_error_w;
    wire [ADDR_WIDTH-1:0] wr_err_addr_w;

    // [FIX-RTL-1] dma_ctrl_fsm�?� dma_error output�?� 캡처할 wire
    // �?�전: .dma_error() → floating output�?�면 FSM error state가 �?위로 전달 안 �?�
    // Fix: wire로 캡처 후 최�?위 dma_error assign�? OR로 �?�함
    wire        dma_error_fsm_w;   // FSM internal error flag

    // Aggregate error address: whichever engine errored last
    assign dma_err_addr = rd_error_w ? rd_err_addr_w : wr_err_addr_w;

    // Aggregate status flags — [FIX-RTL-1] include FSM error
    assign status_rd_error = rd_error_w;
    assign status_wr_error = wr_error_w;
    assign dma_error       = rd_error_w | wr_error_w | dma_error_fsm_w;

    // =========================================================================
    // [FIX-RTL-2] Soft-reset: combined rst_n for FIFOs includes dma_soft_rst
    // Previous: FIFOs only used power-on rst_n → soft_rst pulse did NOT clear FIFOs
    // Fix: fifo_rst_n = rst_n AND NOT dma_soft_rst → soft_rst properly clears FIFOs
    // =========================================================================
    wire fifo_rst_n = rst_n & ~dma_soft_rst;

    // =========================================================================
    // RD FIFO — 64-bit × 4 deep
    // =========================================================================
    sync_fifo #(
        .WIDTH (64),
        .DEPTH (RD_FIFO_DEPTH)
    ) u_rd_fifo (
        .clk   (clk),
        .rst_n (fifo_rst_n),    // [FIX-RTL-2] soft-reset aware
        .din   (rd_fifo_din),
        .push  (rd_fifo_push),
        .full  (rd_fifo_full),
        .dout  (rd_fifo_dout),
        .pop   (rd_fifo_pop),
        .empty (rd_fifo_empty),
        /* verilator lint_off PINCONNECTEMPTY */
        .count ()
        /* verilator lint_on PINCONNECTEMPTY */
    );

    // =========================================================================
    // WR FIFO — 32-bit × 8 deep
    // =========================================================================
    sync_fifo #(
        .WIDTH (32),
        .DEPTH (WR_FIFO_DEPTH)
    ) u_wr_fifo (
        .clk   (clk),
        .rst_n (fifo_rst_n),    // [FIX-RTL-2] soft-reset aware
        .din   (wr_fifo_din),
        .push  (wr_fifo_push),
        .full  (wr_fifo_full),
        .dout  (wr_fifo_dout),
        .pop   (wr_fifo_pop),
        .empty (wr_fifo_empty),
        .count (wr_fifo_count)
    );

    // =========================================================================
    // DMA Control FSM
    // =========================================================================
    dma_ctrl_fsm u_ctrl_fsm (
        .clk                 (clk),
        .rst_n               (rst_n),
        .dma_start           (dma_start),
        .dma_soft_rst        (dma_soft_rst),
        .byte_len            (byte_len),
        .burst_len           (burst_len),
        // Status
        .dma_busy            (dma_busy),
        .dma_done            (dma_done),
        .dma_error           (dma_error_fsm_w),  // [FIX-RTL-1] capture FSM error output
        // Read engine
        .rd_start            (rd_start_w),
        .rd_busy             (rd_busy_w),
        .rd_done             (rd_done_w),
        .rd_error            (rd_error_w),
        // RD FIFO
        .rd_fifo_dout        (rd_fifo_dout),
        .rd_fifo_pop         (rd_fifo_pop),
        .rd_fifo_empty       (rd_fifo_empty),
        // ascon_CORE
        .core_ptext_0        (core_ptext_0),
        .core_ptext_1        (core_ptext_1),
        .core_data_valid     (core_data_valid),
        .core_data_ready     (core_data_ready),
        .core_start          (core_start),
        .core_data_last      (core_data_last),
        .core_busy           (core_busy),
        .core_done           (core_done),
        .core_data_out_valid (core_data_out_valid),
        .core_tag_valid      (core_tag_valid),
        // Results
        .core_ctext_0        (core_ctext_0),
        .core_ctext_1        (core_ctext_1),
        .core_tag_0          (core_tag_0),
        .core_tag_1          (core_tag_1),
        .core_tag_2          (core_tag_2),
        .core_tag_3          (core_tag_3),
        // WR FIFO
        .wr_fifo_din         (wr_fifo_din),
        .wr_fifo_push        (wr_fifo_push),
        .wr_fifo_full        (wr_fifo_full),
        // Write engine (auto-triggers on FIFO count — no wr_start needed)
        .wr_busy             (wr_busy_w),
        .wr_done             (wr_done_w),
        .wr_error            (wr_error_w),
        // Status bits
        .status_rd_done      (status_rd_done),
        .status_wr_done      (status_wr_done),
        .status_fifo_overflow(status_fifo_overflow)
    );

    // =========================================================================
    // DMA Read Engine
    // =========================================================================
    dma_read_engine #(
        .ADDR_WIDTH     (ADDR_WIDTH),
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .AXI_ID_WIDTH   (AXI_ID_WIDTH)
    ) u_rd_engine (
        .clk            (clk),
        .rst_n          (rst_n),
        // Control
        .src_addr       (src_addr),
        .burst_len      (burst_len),
        .rd_start       (rd_start_w),
        .rd_busy        (rd_busy_w),
        .rd_done        (rd_done_w),
        .rd_error       (rd_error_w),
        .rd_err_addr    (rd_err_addr_w),
        // RD FIFO push
        .fifo_din       (rd_fifo_din),
        .fifo_push      (rd_fifo_push),
        .fifo_full      (rd_fifo_full),
        // AXI4 AR channel
        .M_AXI_ARID     (M_AXI_ARID),
        .M_AXI_ARADDR   (M_AXI_ARADDR),
        .M_AXI_ARLEN    (M_AXI_ARLEN),
        .M_AXI_ARSIZE   (M_AXI_ARSIZE),
        .M_AXI_ARBURST  (M_AXI_ARBURST),
        .M_AXI_ARCACHE  (M_AXI_ARCACHE),
        .M_AXI_ARPROT   (M_AXI_ARPROT),
        .M_AXI_ARVALID  (M_AXI_ARVALID),
        .M_AXI_ARREADY  (M_AXI_ARREADY),
        // AXI4 R channel
        .M_AXI_RID      (M_AXI_RID),
        .M_AXI_RDATA    (M_AXI_RDATA),
        .M_AXI_RRESP    (M_AXI_RRESP),
        .M_AXI_RLAST    (M_AXI_RLAST),
        .M_AXI_RVALID   (M_AXI_RVALID),
        .M_AXI_RREADY   (M_AXI_RREADY)
    );

    // =========================================================================
    // DMA Write Engine
    // =========================================================================
    dma_write_engine #(
        .ADDR_WIDTH     (ADDR_WIDTH),
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .AXI_ID_WIDTH   (AXI_ID_WIDTH)
    ) u_wr_engine (
        .clk            (clk),
        .rst_n          (rst_n),
        .dst_addr       (dst_addr),
        .dma_start      (dma_start),
        .wr_busy        (wr_busy_w),
        .wr_done        (wr_done_w),
        .wr_error       (wr_error_w),
        .wr_err_addr    (wr_err_addr_w),
        // WR FIFO pop
        .fifo_dout      (wr_fifo_dout),
        .fifo_pop       (wr_fifo_pop),
        .fifo_count     (wr_fifo_count),
        // AXI4 AW channel
        .M_AXI_AWID     (M_AXI_AWID),
        .M_AXI_AWADDR   (M_AXI_AWADDR),
        .M_AXI_AWLEN    (M_AXI_AWLEN),
        .M_AXI_AWSIZE   (M_AXI_AWSIZE),
        .M_AXI_AWBURST  (M_AXI_AWBURST),
        .M_AXI_AWCACHE  (M_AXI_AWCACHE),
        .M_AXI_AWPROT   (M_AXI_AWPROT),
        .M_AXI_AWVALID  (M_AXI_AWVALID),
        .M_AXI_AWREADY  (M_AXI_AWREADY),
        // AXI4 W channel
        .M_AXI_WDATA    (M_AXI_WDATA),
        .M_AXI_WSTRB    (M_AXI_WSTRB),
        .M_AXI_WLAST    (M_AXI_WLAST),
        .M_AXI_WVALID   (M_AXI_WVALID),
        .M_AXI_WREADY   (M_AXI_WREADY),
        // AXI4 B channel
        .M_AXI_BID      (M_AXI_BID),
        .M_AXI_BRESP    (M_AXI_BRESP),
        .M_AXI_BVALID   (M_AXI_BVALID),
        .M_AXI_BREADY   (M_AXI_BREADY)
    );

endmodule