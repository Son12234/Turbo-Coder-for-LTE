##==============================================================================
## File    : basys3_turbo_demo.xdc
## Board   : Digilent Basys 3 (FPGA Artix-7 xc7a35tcpg236-1)
## Top     : turbo_demo_top
## Nguồn   : số chân (PACKAGE_PIN) lấy từ file Basys-3-Master.xdc chính thức của Digilent
##           (github.com/Digilent/digilent-xdc), chỉ đặt lại tên cổng cho khớp turbo_demo_top.
##
## Cách dùng trên board:
##   - sw[2:0]   chọn kích thước khối K : 0:40  1:104  2:256  3:512  4:1024  5:2048  6:4096  7:6144
##   - btnC      bắt đầu mã hóa một khối (btn_start)
##   - btnU      reset (btn_rst)
##   - led[0]=busy  led[1]=done  led[2]=lỗi K  led[3]=term_ok  led[15:4]=chữ ký MISR 12 bit
##
## Lưu ý: file XDC chỉ được dùng khi TỔNG HỢP / TRIỂN KHAI (synthesis, implementation) và khi
##        mô phỏng sau tổng hợp / sau triển khai (post-synthesis / post-implementation simulation).
##        Mô phỏng chức năng (behavioral simulation) KHÔNG đọc file XDC.
##==============================================================================

## ---------------- Xung nhịp 100 MHz ----------------
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## ---------------- Công tắc chọn K (sw[2:0]) ----------------
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports {sw[0]}]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports {sw[1]}]
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS33 } [get_ports {sw[2]}]

## ---------------- Nút bấm ----------------
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports btn_start]  ;# nút giữa btnC
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports btn_rst]    ;# nút trên btnU

## ---------------- 16 LED ----------------
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN E19 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN V19 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN W18 IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports {led[7]}]
set_property -dict { PACKAGE_PIN V13 IOSTANDARD LVCMOS33 } [get_ports {led[8]}]
set_property -dict { PACKAGE_PIN V3  IOSTANDARD LVCMOS33 } [get_ports {led[9]}]
set_property -dict { PACKAGE_PIN W3  IOSTANDARD LVCMOS33 } [get_ports {led[10]}]
set_property -dict { PACKAGE_PIN U3  IOSTANDARD LVCMOS33 } [get_ports {led[11]}]
set_property -dict { PACKAGE_PIN P3  IOSTANDARD LVCMOS33 } [get_ports {led[12]}]
set_property -dict { PACKAGE_PIN N3  IOSTANDARD LVCMOS33 } [get_ports {led[13]}]
set_property -dict { PACKAGE_PIN P1  IOSTANDARD LVCMOS33 } [get_ports {led[14]}]
set_property -dict { PACKAGE_PIN L1  IOSTANDARD LVCMOS33 } [get_ports {led[15]}]

## ---------------- Ràng buộc thời gian cho các tín hiệu không đồng bộ với clk ----------------
## Nút bấm và công tắc là tín hiệu chậm, đã qua mạch đồng bộ 2 tầng / chống rung trong thiết kế
## => không cần phân tích thời gian. LED chỉ để quan sát bằng mắt => cũng không cần.
set_false_path -from [get_ports {btn_start btn_rst}]
set_false_path -from [get_ports {sw[*]}]
set_false_path -to   [get_ports {led[*]}]

## ---------------- Cấu hình bitstream (theo file chính thức của Digilent) ----------------
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
