`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : turbo_demo_top.v
// Chức năng : Mạch demo chạy trên board FPGA (Basys 3, Nexys A7...) cho bộ mã hóa Turbo LTE.
//
// Nguyên lý: bộ mã hóa có ngõ vào/ra nối tiếp tốc độ 100 MHz nên không thể quan sát trực tiếp bằng
// mắt. Mạch demo vì vậy tự cấp dữ liệu và tự "ký" kết quả:
//   - Dữ liệu vào : bộ LFSR 32 bit (đa thức x^32 + x^22 + x^2 + x + 1), nạp lại hạt giống cố định
//                   mỗi lần bấm start => cùng K thì luôn cùng dữ liệu vào, kết quả lặp lại được.
//   - Chữ ký ra   : thanh ghi MISR 12 bit (đa thức CRC-12 của 3GPP) gom toàn bộ K+4 bộ ba {d2,d1,d0}.
//   - Hiển thị    : led[0]=busy, led[1]=done, led[2]=lỗi K, led[3]=term_ok, led[15:4]=chữ ký MISR.
// Chữ ký của 8 giá trị K có sẵn được ghi trong README.md và được testbench tb_turbo_demo_top kiểm tra.
//
// Cách dùng: sw[2:0] chọn K (0:40 1:104 2:256 3:512 4:1024 5:2048 6:4096 7:6144) ->
//            nhấn btn_start -> chờ led[1] sáng -> đọc chữ ký ở led[15:4]. btn_rst để reset.
//==============================================================================
module turbo_demo_top #(
    parameter integer DEBOUNCE_CYCLES = 1000000   // thời gian chống rung nút (nhịp clk); testbench đặt nhỏ để chạy nhanh
)(
    input  wire        clk,        // xung nhịp 100 MHz của board
    input  wire        btn_rst,    // nút reset (mức cao khi đang nhấn)
    input  wire        btn_start,  // nút bắt đầu mã hóa một khối
    input  wire [2:0]  sw,         // công tắc chọn kích thước khối K
    output wire [15:0] led         // 16 LED hiển thị trạng thái và chữ ký
);

    // ---------------- reset khi cấp nguồn và reset bằng nút ----------------
    reg [3:0] por_cnt;                         // bộ đếm reset khi bật nguồn (power-on reset)
    initial por_cnt = 4'd0;                    // bắt đầu từ 0 nên reset được giữ ngay khi nạp cấu hình
    always @(posedge clk)                      // đếm theo xung nhịp
        if (por_cnt != 4'hF) por_cnt <= por_cnt + 4'd1; // đếm đến 15 rồi dừng
    wire rst_btn;                              // mức nút reset đã chống rung
    btn_debounce #(.CNT_MAX(DEBOUNCE_CYCLES)) u_db_rst (
        .clk(clk), .btn_in(btn_rst), .btn_stable(rst_btn), .btn_rise()   // chỉ cần mức, không cần xung cạnh
    );
    wire rst = (por_cnt != 4'hF) | rst_btn;    // reset = (đang trong 16 nhịp đầu) hoặc (nút reset đang nhấn)

    // ---------------- nút start và công tắc chọn K ----------------
    wire start_rise;                           // xung 1 nhịp khi nút start vừa được nhấn
    wire start_lvl;                            // mức nút start đã chống rung (không dùng)
    btn_debounce #(.CNT_MAX(DEBOUNCE_CYCLES)) u_db_start (
        .clk(clk), .btn_in(btn_start), .btn_stable(start_lvl), .btn_rise(start_rise)
    );
    reg [2:0] sw_m;                            // tầng đồng bộ thứ nhất cho công tắc
    reg [2:0] sw_s;                            // tầng đồng bộ thứ hai cho công tắc
    initial begin sw_m = 3'd0; sw_s = 3'd0; end // giá trị ban đầu
    always @(posedge clk) begin                // đồng bộ công tắc sang miền clk
        sw_m <= sw;                            // tầng 1
        sw_s <= sw_m;                          // tầng 2
    end

    function [12:0] k_preset;                  // bảng chọn K theo công tắc
        input [2:0] sel;                       // giá trị công tắc
        begin
            case (sel)                         // 8 kích thước khối hợp lệ của LTE
                3'd0:    k_preset = 13'd40;    // nhỏ nhất
                3'd1:    k_preset = 13'd104;   // 104
                3'd2:    k_preset = 13'd256;   // 256
                3'd3:    k_preset = 13'd512;   // 512
                3'd4:    k_preset = 13'd1024;  // 1024
                3'd5:    k_preset = 13'd2048;  // 2048
                3'd6:    k_preset = 13'd4096;  // 4096
                default: k_preset = 13'd6144;  // lớn nhất của LTE
            endcase
        end
    endfunction

    // ---------------- bộ mã hóa Turbo LTE ----------------
    wire        in_ready;                      // bộ mã hóa sẵn sàng nhận bit
    wire        out_valid;                     // ngõ ra hợp lệ
    wire        d0, d1, d2;                    // ba luồng ngõ ra
    wire        out_tail;                      // đang phát bit đuôi (không dùng ở demo)
    wire        out_last;                      // nhịp cuối của khối
    wire        busy;                          // đang bận
    wire        err_k;                         // xung báo K sai
    wire        term_ok;                       // kết thúc trellis thành công
    wire        start_pulse = start_rise & ~busy; // chỉ nhận lệnh start khi bộ mã hóa đang rảnh
    wire        din;                           // bit dữ liệu từ LFSR

    turbo_encoder u_enc (
        .clk(clk), .rst(rst),
        .start(start_pulse), .k_in(k_preset(sw_s)),
        .in_ready(in_ready), .in_valid(in_ready), .din(din), // luôn có sẵn bit, nên in_valid = in_ready
        .out_valid(out_valid), .d0(d0), .d1(d1), .d2(d2),
        .out_tail(out_tail), .out_last(out_last),
        .busy(busy), .err_k(err_k), .term_ok(term_ok)
    );

    // ---------------- nguồn dữ liệu LFSR ----------------
    localparam [31:0] LFSR_SEED = 32'hACE12468;          // hạt giống cố định, khác 0
    reg  [31:0] lfsr;                                    // thanh ghi LFSR kiểu Galois
    initial lfsr = LFSR_SEED;                            // giá trị ban đầu
    wire [31:0] lfsr_nxt = lfsr[0] ? ((lfsr >> 1) ^ 32'h80200003) // bit 0 = 1: dịch phải rồi XOR mặt nạ đa thức
                                   :  (lfsr >> 1);               // bit 0 = 0: chỉ dịch phải
    assign din = lfsr[0];                                // bit dữ liệu hiện tại = bit thấp nhất của LFSR
    always @(posedge clk) begin                          // cập nhật LFSR
        if (rst || start_pulse) lfsr <= LFSR_SEED;       // nạp lại hạt giống khi reset hoặc bắt đầu khối mới
        else if (in_ready)      lfsr <= lfsr_nxt;        // mỗi bit được nhận thì LFSR tiến một bước
    end

    // ---------------- chữ ký MISR 12 bit và các cờ trạng thái ----------------
    reg [11:0] sig;                                      // chữ ký tích lũy
    reg        done_l;                                   // =1 sau khi phát xong khối, xóa khi bắt đầu khối mới
    reg        err_l;                                    // =1 nếu từng có lỗi K kể từ lần start gần nhất
    reg        term_l;                                   // lưu term_ok tại nhịp cuối của khối
    initial begin sig = 12'd0; done_l = 1'b0; err_l = 1'b0; term_l = 1'b0; end // giá trị ban đầu
    always @(posedge clk) begin                          // cập nhật chữ ký và cờ
        if (rst || start_pulse) begin                    // reset hoặc bắt đầu khối mới
            sig    <= 12'd0;                             // xóa chữ ký
            done_l <= 1'b0;                              // chưa xong
            err_l  <= 1'b0;                              // xóa cờ lỗi
            term_l <= 1'b0;                              // xóa cờ kết thúc trellis
        end else begin                                   // hoạt động bình thường
            if (out_valid)                               // mỗi nhịp có dữ liệu ra thì cập nhật chữ ký
                sig <= {sig[10:0], 1'b0} ^ (sig[11] ? 12'h80F : 12'h000) ^ {9'b0, d2, d1, d0}; // dịch trái, XOR đa thức CRC-12, XOR dữ liệu
            if (out_valid && out_last) begin             // nhịp cuối của khối
                done_l <= 1'b1;                          // báo xong
                term_l <= term_ok;                       // chốt trạng thái kết thúc trellis
            end
            if (err_k) err_l <= 1'b1;                    // ghi nhớ lỗi K
        end
    end

    assign led[0]    = busy;                             // LED0: đang bận
    assign led[1]    = done_l;                           // LED1: đã xong khối
    assign led[2]    = err_l;                            // LED2: có lỗi K
    assign led[3]    = term_l;                           // LED3: trellis kết thúc đúng (cả hai bộ RSC về 000)
    assign led[15:4] = sig;                              // LED4..15: chữ ký MISR

endmodule
