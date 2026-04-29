// =============================================================================
// FILE   : ascon_ip_dut_wrapper.v  (v2 - PYNQ-Z2 final fix)
// DEVICE : xc7z020clg400-1  (255 total I/O, 64 board-locked = 191 free)
//
// ROOT CAUSE l?i v1:
//   [Place 30-415]  201 ports > 191 available  ? quá nhi?u pins
//   [Vivado 12-1411] W18 dùng 2 l?n (WDATA[0] và AWID[2]) ? pin conflict
//   [Constraints 18-602] set_input_delay áp d?ng nh?m lên output port
//
// FIX v2:
//   1. Tie-off TOÀN B? S_AXI_* (slave) - không còn port v?t lý
//   2. Tie-off TOÀN B? M_AXI_* (master) - gi? nh? v1
//   3. Ch? 5 physical pins: clk, rst_n, o_tag_valid, o_busy, irq
//   4. o_tag[127:0] ? MARK_DEBUG ? ??c qua ILA/JTAG, không c?n pin
//   5. XDC s?ch: không conflict, không sai set_input_delay
//
// T?NG: 5 pins << 191 available ? Place & Route thành công
// =============================================================================

`default_nettype none

module ascon_ip_dut_wrapper #(
    parameter G_COMB_RND_128  = 6,
    parameter G_COMB_RND_128A = 4,
    parameter G_SBOX_PIPELINE = 0,
    parameter G_DUAL_RATE     = 1,
    parameter G_AXI_DATA_W    = 64,
    parameter S_ADDR_WIDTH    = 32,
    parameter S_DATA_WIDTH    = 32,
    parameter S_ID_WIDTH      = 4,
    parameter M_ADDR_WIDTH    = 32,
    parameter M_DATA_WIDTH    = 64,
    parameter M_ID_WIDTH      = 4,
    parameter RD_FIFO_DEPTH   = 4,
    parameter WR_FIFO_DEPTH   = 32
) (
    // =========================================================================
    // CH? 5 PINS V?T LÝ
    // =========================================================================
    input  wire  clk,         // PYNQ-Z2: H16, 125 MHz
    input  wire  rst_n,       // PYNQ-Z2: D19, BTN0 (nh?n = reset)

    output wire  o_tag_valid, // PYNQ-Z2: R14, LED[0]
    output wire  o_busy,      // PYNQ-Z2: P14, LED[1]
    output wire  irq          // PYNQ-Z2: N16, LED[2]
    // o_tag[127:0]: MARK_DEBUG ? xem qua Vivado Hardware Manager (ILA)
);

    // =========================================================================
    // S_AXI_* SLAVE TIE-OFF
    // Inputs ??n core: VALID = 0 (không có transaction nào ???c phát ra)
    // Outputs t? core: dây n?i b? _nc (not connected ra pad)
    // =========================================================================
    // --- inputs to core ---
    wire [S_ID_WIDTH-1:0]     s_awid    = {S_ID_WIDTH{1'b0}};
    wire [S_ADDR_WIDTH-1:0]   s_awaddr  = {S_ADDR_WIDTH{1'b0}};
    wire [7:0]                s_awlen   = 8'h00;
    wire [2:0]                s_awsize  = 3'b010;
    wire [1:0]                s_awburst = 2'b01;
    wire [2:0]                s_awprot  = 3'b000;
    wire                      s_awvalid = 1'b0;

    wire [S_DATA_WIDTH-1:0]   s_wdata   = {S_DATA_WIDTH{1'b0}};
    wire [S_DATA_WIDTH/8-1:0] s_wstrb   = {(S_DATA_WIDTH/8){1'b0}};
    wire                      s_wlast   = 1'b0;
    wire                      s_wvalid  = 1'b0;

    wire                      s_bready  = 1'b1;

    wire [S_ID_WIDTH-1:0]     s_arid    = {S_ID_WIDTH{1'b0}};
    wire [S_ADDR_WIDTH-1:0]   s_araddr  = {S_ADDR_WIDTH{1'b0}};
    wire [7:0]                s_arlen   = 8'h00;
    wire [2:0]                s_arsize  = 3'b010;
    wire [1:0]                s_arburst = 2'b01;
    wire [2:0]                s_arprot  = 3'b000;
    wire                      s_arvalid = 1'b0;

    wire                      s_rready  = 1'b1;

    // --- outputs from core (not connected to pads) ---
    wire                     s_awready_nc;
    wire                     s_wready_nc;
    wire [S_ID_WIDTH-1:0]    s_bid_nc;
    wire [1:0]               s_bresp_nc;
    wire                     s_bvalid_nc;
    wire                     s_arready_nc;
    wire [S_ID_WIDTH-1:0]    s_rid_nc;
    wire [S_DATA_WIDTH-1:0]  s_rdata_nc;
    wire [1:0]               s_rresp_nc;
    wire                     s_rlast_nc;
    wire                     s_rvalid_nc;

    // =========================================================================
    // M_AXI_* MASTER TIE-OFF
    // Core outputs: dây n?i b? _nc
    // Core inputs: memory luôn READY=1, không tr? d? li?u
    // =========================================================================
    wire [M_ID_WIDTH-1:0]     m_awid_nc;
    wire [M_ADDR_WIDTH-1:0]   m_awaddr_nc;
    wire [7:0]                m_awlen_nc;
    wire [2:0]                m_awsize_nc;
    wire [1:0]                m_awburst_nc;
    wire [3:0]                m_awcache_nc;
    wire [2:0]                m_awprot_nc;
    wire                      m_awvalid_nc;

    wire [M_DATA_WIDTH-1:0]   m_wdata_nc;
    wire [M_DATA_WIDTH/8-1:0] m_wstrb_nc;
    wire                      m_wlast_nc;
    wire                      m_wvalid_nc;
    wire                      m_bready_nc;

    wire [M_ID_WIDTH-1:0]     m_arid_nc;
    wire [M_ADDR_WIDTH-1:0]   m_araddr_nc;
    wire [7:0]                m_arlen_nc;
    wire [2:0]                m_arsize_nc;
    wire [1:0]                m_arburst_nc;
    wire [3:0]                m_arcache_nc;
    wire [2:0]                m_arprot_nc;
    wire                      m_arvalid_nc;
    wire                      m_rready_nc;

    // =========================================================================
    // o_tag: ILA capture - MARK_DEBUG, KHÔNG ra pad
    // =========================================================================
    (* MARK_DEBUG = "TRUE" *) wire [127:0] w_tag;
    (* MARK_DEBUG = "TRUE" *) wire         w_tag_valid;
    (* MARK_DEBUG = "TRUE" *) wire         w_busy;
    (* MARK_DEBUG = "TRUE" *) wire         w_irq;

    assign o_tag_valid = w_tag_valid;
    assign o_busy      = w_busy;
    assign irq         = w_irq;

    // =========================================================================
    // DUT
    // =========================================================================
    ascon_ip_top #(
        .G_COMB_RND_128  ( G_COMB_RND_128  ),
        .G_COMB_RND_128A ( G_COMB_RND_128A ),
        .G_SBOX_PIPELINE ( G_SBOX_PIPELINE ),
        .G_DUAL_RATE     ( G_DUAL_RATE     ),
        .G_AXI_DATA_W    ( G_AXI_DATA_W    ),
        .S_ADDR_WIDTH    ( S_ADDR_WIDTH    ),
        .S_DATA_WIDTH    ( S_DATA_WIDTH    ),
        .S_ID_WIDTH      ( S_ID_WIDTH      ),
        .M_ADDR_WIDTH    ( M_ADDR_WIDTH    ),
        .M_DATA_WIDTH    ( M_DATA_WIDTH    ),
        .M_ID_WIDTH      ( M_ID_WIDTH      ),
        .RD_FIFO_DEPTH   ( RD_FIFO_DEPTH   ),
        .WR_FIFO_DEPTH   ( WR_FIFO_DEPTH   )
    ) u_dut (
        .clk             ( clk        ),
        .rst_n           ( rst_n      ),

        // S_AXI - slave, toàn b? tie-off
        .S_AXI_AWID      ( s_awid     ), .S_AXI_AWADDR  ( s_awaddr  ),
        .S_AXI_AWLEN     ( s_awlen    ), .S_AXI_AWSIZE  ( s_awsize  ),
        .S_AXI_AWBURST   ( s_awburst  ), .S_AXI_AWPROT  ( s_awprot  ),
        .S_AXI_AWVALID   ( s_awvalid  ), .S_AXI_AWREADY ( s_awready_nc ),

        .S_AXI_WDATA     ( s_wdata    ), .S_AXI_WSTRB   ( s_wstrb   ),
        .S_AXI_WLAST     ( s_wlast    ), .S_AXI_WVALID  ( s_wvalid  ),
        .S_AXI_WREADY    ( s_wready_nc ),

        .S_AXI_BID       ( s_bid_nc   ), .S_AXI_BRESP   ( s_bresp_nc ),
        .S_AXI_BVALID    ( s_bvalid_nc ), .S_AXI_BREADY ( s_bready  ),

        .S_AXI_ARID      ( s_arid     ), .S_AXI_ARADDR  ( s_araddr  ),
        .S_AXI_ARLEN     ( s_arlen    ), .S_AXI_ARSIZE  ( s_arsize  ),
        .S_AXI_ARBURST   ( s_arburst  ), .S_AXI_ARPROT  ( s_arprot  ),
        .S_AXI_ARVALID   ( s_arvalid  ), .S_AXI_ARREADY ( s_arready_nc ),

        .S_AXI_RID       ( s_rid_nc   ), .S_AXI_RDATA   ( s_rdata_nc ),
        .S_AXI_RRESP     ( s_rresp_nc ), .S_AXI_RLAST   ( s_rlast_nc ),
        .S_AXI_RVALID    ( s_rvalid_nc ), .S_AXI_RREADY ( s_rready  ),

        // M_AXI - master, toàn b? tie-off
        .M_AXI_AWID      ( m_awid_nc   ), .M_AXI_AWADDR  ( m_awaddr_nc ),
        .M_AXI_AWLEN     ( m_awlen_nc  ), .M_AXI_AWSIZE  ( m_awsize_nc ),
        .M_AXI_AWBURST   ( m_awburst_nc), .M_AXI_AWCACHE ( m_awcache_nc),
        .M_AXI_AWPROT    ( m_awprot_nc ), .M_AXI_AWVALID ( m_awvalid_nc),
        .M_AXI_AWREADY   ( 1'b1        ),

        .M_AXI_WDATA     ( m_wdata_nc  ), .M_AXI_WSTRB   ( m_wstrb_nc ),
        .M_AXI_WLAST     ( m_wlast_nc  ), .M_AXI_WVALID  ( m_wvalid_nc),
        .M_AXI_WREADY    ( 1'b1        ),

        .M_AXI_BID       ( {M_ID_WIDTH{1'b0}} ),
        .M_AXI_BRESP     ( 2'b00       ),
        .M_AXI_BVALID    ( 1'b0        ),
        .M_AXI_BREADY    ( m_bready_nc ),

        .M_AXI_ARID      ( m_arid_nc   ), .M_AXI_ARADDR  ( m_araddr_nc ),
        .M_AXI_ARLEN     ( m_arlen_nc  ), .M_AXI_ARSIZE  ( m_arsize_nc ),
        .M_AXI_ARBURST   ( m_arburst_nc), .M_AXI_ARCACHE ( m_arcache_nc),
        .M_AXI_ARPROT    ( m_arprot_nc ), .M_AXI_ARVALID ( m_arvalid_nc),
        .M_AXI_ARREADY   ( 1'b1        ),

        .M_AXI_RID       ( {M_ID_WIDTH{1'b0}} ),
        .M_AXI_RDATA     ( {M_DATA_WIDTH{1'b0}} ),
        .M_AXI_RRESP     ( 2'b00       ),
        .M_AXI_RLAST     ( 1'b0        ),
        .M_AXI_RVALID    ( 1'b0        ),
        .M_AXI_RREADY    ( m_rready_nc ),

        // Status
        .o_tag           ( w_tag       ),
        .o_tag_valid     ( w_tag_valid ),
        .o_busy          ( w_busy      ),
        .irq             ( w_irq       )
    );

endmodule

`default_nettype wire