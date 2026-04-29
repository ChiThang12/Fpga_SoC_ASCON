# =============================================================================
# FILE   : ascon_ip_dut_wrapper_pynqz2.xdc  (v2 - clean)
# DEVICE : xc7z020clg400-1
# BOARD  : PYNQ-Z2 (TUL)
#
# FIX v2 so v?i v1:
#   - B? toàn b? S_AXI_* / M_AXI_* constraints (không còn port v?t lý)
#   - Xóa set_input_delay / set_output_delay cho output-only ports
#     (output ports không dùng set_INPUT_delay ? l?i [Constraints 18-602])
#   - Không còn pin conflict (W18 ch? dùng 1 l?n)
#   - T?ng: 5 pins v?t lý << 191 pins còn tr?ng
# =============================================================================

# -----------------------------------------------------------------------------
# 1. CLOCK - 125 MHz, Bank 35, MRCC (Multi-Region Clock Capable)
# -----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN H16  IOSTANDARD LVCMOS33 } [get_ports { clk }]
create_clock -period 8.000 -name sys_clk -waveform {0.000 4.000} [get_ports { clk }]

# B? qua jitter nh? t? onboard oscillator
set_property CLOCK_DEDICATED_ROUTE TRUE [get_nets { clk }]

# -----------------------------------------------------------------------------
# 2. RESET - BTN0 (D19, Bank 35)
#    Nh?n nút = GND ? rst_n = 0 ? reset active
# -----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN D19  IOSTANDARD LVCMOS33 } [get_ports { rst_n }]
set_false_path -from [get_ports { rst_n }]

# -----------------------------------------------------------------------------
# 3. STATUS OUTPUTS - LEDs (Bank 34)
#    o_tag_valid ? LED[0] R14
#    o_busy      ? LED[1] P14
#    irq         ? LED[2] N16
# -----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN R14  IOSTANDARD LVCMOS33 } [get_ports { o_tag_valid }]
set_property -dict { PACKAGE_PIN P14  IOSTANDARD LVCMOS33 } [get_ports { o_busy      }]
set_property -dict { PACKAGE_PIN N16  IOSTANDARD LVCMOS33 } [get_ports { irq         }]

# Output delay cho 3 LEDs (relaxed - ch? visual indicator)
set_output_delay -clock sys_clk -max 2.0 [get_ports { o_tag_valid o_busy irq }]
set_output_delay -clock sys_clk -min 0.0 [get_ports { o_tag_valid o_busy irq }]

# -----------------------------------------------------------------------------
# 4. INPUT DELAY - ch? cho input ports (clk ?ã có create_clock, rst_n ?ã false_path)
#    Không có input port nào khác ? không c?n set_input_delay
# -----------------------------------------------------------------------------
set_property KEEP_HIERARCHY TRUE [current_design]
# -----------------------------------------------------------------------------
# 5. TIMING EXCEPTIONS
# -----------------------------------------------------------------------------
# Multicycle path cho LED outputs (không quan tr?ng timing ch?t)
set_multicycle_path -setup 2 -to [get_ports { o_tag_valid o_busy irq }]
set_multicycle_path -hold  1 -to [get_ports { o_tag_valid o_busy irq }]

# =============================================================================
# H??NG D?N ??C o_tag[127:0] QUA ILA
# =============================================================================
# Sau khi Synthesize xong, trong Vivado:
#   1. Flow Navigator ? Open Synthesized Design
#   2. Menu: Tools ? Set Up Debug
#   3. Wizard s? t? detect wire w_tag[127:0] (?ã có MARK_DEBUG)
#   4. Thêm vào ILA core, ch?n clock = clk
#   5. Click "Finish" ? Generate Bitstream
#   6. Program board ? Hardware Manager ? Trigger ILA ? ??c giá tr? tag
# =============================================================================
