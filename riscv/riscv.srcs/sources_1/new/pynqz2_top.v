module pynqz2_top (
    input  wire       sys_clk,      // Chân R4 trên PYNQ-Z2 (125MHz)
    input  wire       sys_rst_n,    // Nút nh?n reset (th??ng dùng BTN0)
    
    // Ngo?i vi t?i thi?u ?? test
    output wire       uart_tx,      
    input  wire       uart_rx,
    output wire [3:0] leds,         // 4 LED có s?n trên board ?? hi?n th? tr?ng thái
    
    // JTAG pins n?u b?n dùng cáp r?i, n?u không hãy gi? k?t n?i n?i b?
    input  wire       tms,
    input  wire       tck,
    input  wire       tdi,
    output wire       tdo
);

    wire clk = sys_clk;
    wire rst = ~sys_rst_n; // PYNQ-Z2 nút nh?n th??ng tích c?c cao ho?c th?p tùy c?u hình

    // Tín hi?u k?t n?i n?i b? (Không ?i ra chân I/O)
    wire [31:0] imem_addr, imem_rdata;
    wire        imem_valid, imem_ready;
    wire [31:0] dcache_addr, dcache_wdata, dcache_rdata;
    wire [3:0]  dcache_wstrb;
    wire        dcache_req, dcache_we, dcache_ready;
    wire [1:0]  dcache_fence_type;

    // Instance nhân CPU c?a b?n
    (* DONT_TOUCH = "yes" *)
    riscv_cpu_core u_cpu (
        .clk(clk),
        .rst(rst),
        .imem_addr(imem_addr),
        .imem_valid(imem_valid),
        .imem_rdata(imem_rdata),
        .imem_ready(imem_ready),
        .dcache_addr(dcache_addr),
        .dcache_wdata(dcache_wdata),
        .dcache_wstrb(dcache_wstrb),
        .dcache_req(dcache_req),
        .dcache_we(dcache_we),
        .dcache_rdata(dcache_rdata),
        .dcache_ready(dcache_ready),
        .dcache_fence_type(dcache_fence_type),
        .external_irq(1'b0),
        .timer_irq(1'b0),
        .sw_irq(1'b0),
        .debug_haltreq(1'b0),    // T?m th?i disable n?u ch?a có kh?i JTAG DM
        .debug_resumereq(1'b0),
        .debug_halted(),
        .debug_running()
    );

    // --- GI?I PHÁP GI?M CHÂN: K?t n?i bus vào RAM n?i b? ---
    // ? ?ây b?n nên chèn m?t kh?i RAM (BRAM) 
    // M?i bus 32-bit s? d?ng l?i ? ?ây, không ch?y ra chân v?t lý c?a FPGA.
    
    assign imem_ready = 1'b1;
    assign dcache_ready = 1'b1;
    // (B?n c?n vi?t thêm logic cho Memory Controller ??n gi?n ? ?ây)

    assign leds = imem_addr[3:0]; // Hi?n th? 4 bit th?p c?a ??a ch? l?nh lên LED ?? bi?t CPU ?ang ch?y

endmodule