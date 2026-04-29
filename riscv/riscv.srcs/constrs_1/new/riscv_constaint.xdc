## Clock signal (125 MHz)
set_property -dict { PACKAGE_PIN H16   IOSTANDARD LVCMOS33 } [get_ports { sys_clk }];
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { sys_clk }];

## Reset (Su dung nut nhan BTN0)
set_property -dict { PACKAGE_PIN D19   IOSTANDARD LVCMOS33 } [get_ports { sys_rst_n }];

## LEDs (Dung de quan sat CPU hoat dong)
set_property -dict { PACKAGE_PIN R14   IOSTANDARD LVCMOS33 } [get_ports { leds[0] }];
set_property -dict { PACKAGE_PIN P14   IOSTANDARD LVCMOS33 } [get_ports { leds[1] }];
set_property -dict { PACKAGE_PIN N16   IOSTANDARD LVCMOS33 } [get_ports { leds[2] }];
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { leds[3] }];

## UART (Ket noi qua chip FTDI qua cong micro USB)
set_property -dict { PACKAGE_PIN W18   IOSTANDARD LVCMOS33 } [get_ports { uart_tx }];
set_property -dict { PACKAGE_PIN W19   IOSTANDARD LVCMOS33 } [get_ports { uart_rx }];

## Cau hinh dong dien va chuan I/O chung cho chip Zynq
set_property BITSTREAM.Config.UnusedPin Pullup [current_design]

# ===========================================================================
# [FIX-FANOUT] Timing / fanout constraints for regwrite_retire replicated reg
# Buoc 1: Gioi han fanout cua regwrite_retire_reg chinh (cap cho reg_file)
# Buoc 2: Cho phep Vivado tu dong replicate forward_a/forward_b neu can
# ===========================================================================

# Replicate regwrite_retire de giam fanout (~32 fan -> 2x16)
set_max_fanout 16 [get_nets {u_cpu/regwrite_retire_reg}]
set_max_fanout 16 [get_nets {u_cpu/regwrite_retire_fwd_reg}]

# Cho phep Vivado replicate forward_a/forward_b neu slack van am sau Fix 1
set_max_fanout 16 [get_nets {u_cpu/forward_a_reg[*]}]
set_max_fanout 16 [get_nets {u_cpu/forward_b_reg[*]}]

# Khuyen Vivado dung pipelining tren stall_any (fanout cao, tren critical path)
set_max_fanout 8  [get_nets {u_cpu/stall_any}]

# ===========================================================================
# [FIX-FANOUT v2] rd_wb la source cua critical path moi
# rd_wb -> forwarding_unit -> forward_a/b -> CE cua pipeline registers
# ===========================================================================

# Gioi han fanout cua rd_wb (mem_wb_reg output)
set_max_fanout 8 [get_nets {u_cpu/mem_wb_reg/rd_out_reg[*]}]

# Replicate stall_any_ifid va stall_any_idex (CE driver)
set_max_fanout 8 [get_nets {u_cpu/stall_any_ifid}]
set_max_fanout 8 [get_nets {u_cpu/stall_any_idex}]

# regwrite_wb cung co fanout cao tren path nay
set_max_fanout 8 [get_nets {u_cpu/mem_wb_reg/regwrite_out_reg}]

# ===========================================================================
# [FIX-FANOUT v3] rd_mem la source cua critical path moi (-2.613 ns)
# rd_mem -> forwarding_unit -> forward_a/b -> CE cua pipeline registers
# ===========================================================================

# Gioi han fanout cua rd_mem (ex_mem_reg output) - source cua path hien tai
set_max_fanout 8 [get_nets {u_cpu/ex_mem_reg/rd_out_reg[*]}]
set_max_fanout 8 [get_nets {u_cpu/rd_mem_fwd[*]}]

# forward_a/forward_b CE alias nets - dat gan id_ex_reg
set_max_fanout 4 [get_nets {u_cpu/forward_a_ce[*]}]
set_max_fanout 4 [get_nets {u_cpu/forward_b_ce[*]}]

# regwrite_mem cung co fanout cao tren path nay
set_max_fanout 8 [get_nets {u_cpu/ex_mem_reg/regwrite_out_reg}]

# ===========================================================================
# [FIX-TIMING v4] Branch redirect path: rd_mem -> ALU -> target_pc -> IFU PC/D
# pc_src_ex va target_pc_ex da duoc registered (pc_src_ex_r / target_pc_ex_r)
# nen IFU PC/D chi con nhan tu registered FF, Net Delay se giam manh.
# ===========================================================================

# Khuyen Vivado dat FF pc_src_ex_r / target_pc_ex_r gan IFU bang Pblock nhe
# (chi la suggestion, khong ep buoc - de Vivado tu quyet dinh placement)
set_max_fanout 4 [get_nets {u_cpu/pc_src_ex_r}]
set_max_fanout 4 [get_nets {u_cpu/target_pc_ex_r[*]}]

# ifu_pc_src co fanout thap nhung dat suggestion de router uu tien
set_max_fanout 4 [get_nets {u_cpu/ifu_pc_src}]

# ===========================================================================
# [FIX-FANOUT v5] regwrite_mem fanout=75~127, split thanh 3 ban alias
# Source moi: ex_mem_reg/regwrite_out_reg -> forwarding -> mux -> mul/pip D
# ===========================================================================

# 3 ban alias cua regwrite_mem
set_max_fanout 8  [get_nets {u_cpu/regwrite_mem_fwd}]
set_max_fanout 8  [get_nets {u_cpu/regwrite_mem_mul}]
set_max_fanout 8  [get_nets {u_cpu/regwrite_mem_pip}]

# Gioi han fanout goc cua regwrite_mem truoc khi no fan ra 3 alias
set_max_fanout 4  [get_nets {u_cpu/ex_mem_reg/regwrite_out_reg}]

# Multiplier operand alias nets - dat gan multiplier FF
set_max_fanout 4  [get_nets {u_cpu/mul_operand_a[*]}]
set_max_fanout 4  [get_nets {u_cpu/mul_operand_b[*]}]

# alu_in1_forwarded / alu_in2_pre_mux - drive ca ALU lan multiplier, fanout cao
set_max_fanout 16 [get_nets {u_cpu/alu_in1_forwarded[*]}]
set_max_fanout 16 [get_nets {u_cpu/alu_in2_pre_mux[*]}]

# forward_a/b decode wires - fwd_a_mem etc, van con tren critical path
set_max_fanout 8  [get_nets {u_cpu/fwd_a_mem}]
set_max_fanout 8  [get_nets {u_cpu/fwd_b_mem}]
set_max_fanout 8  [get_nets {u_cpu/fwd_a_wb}]
set_max_fanout 8  [get_nets {u_cpu/fwd_b_wb}]
