`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : btn_debounce.v
// Chức năng : Đồng bộ và chống rung cho một nút bấm cơ học (dùng trong turbo_demo_top).
//   - Hai D-FF nối tiếp đưa tín hiệu bất đồng bộ từ nút bấm sang miền xung clk (chống metastable).
//   - Bộ đếm: chỉ khi mức sau đồng bộ KHÁC mức ổn định hiện tại liên tục CNT_MAX nhịp thì mới
//     chấp nhận mức mới (loại bỏ rung cơ học khi nhấn/nhả).
//   - btn_rise = xung 1 nhịp clk ở thời điểm mức ổn định chuyển từ 0 lên 1 (lúc bắt đầu nhấn).
//==============================================================================
module btn_debounce #(
    parameter integer CNT_MAX = 1000000    // số nhịp clk cần ổn định (1.000.000 nhịp = 10 ms ở 100 MHz)
)(
    input  wire clk,                       // xung nhịp hệ thống
    input  wire btn_in,                    // nút bấm thô (bất đồng bộ, có rung)
    output reg  btn_stable,                // mức nút đã ổn định và đã chống rung
    output wire btn_rise                   // xung 1 nhịp khi nút vừa được nhấn
);

    reg        sync1;                      // D-FF đồng bộ thứ nhất
    reg        sync2;                      // D-FF đồng bộ thứ hai (đầu ra đã an toàn metastable)
    reg        prev;                       // btn_stable ở nhịp trước, dùng để tìm cạnh lên
    reg [23:0] cnt;                        // bộ đếm thời gian ổn định (24 bit đủ đến ~16,7 triệu nhịp)

    initial begin                          // giá trị ban đầu để dạng sóng không có X
        sync1      = 1'b0;                 // D-FF đồng bộ 1 bắt đầu ở 0
        sync2      = 1'b0;                 // D-FF đồng bộ 2 bắt đầu ở 0
        prev       = 1'b0;                 // nhịp trước = 0
        cnt        = 24'd0;                // bộ đếm = 0
        btn_stable = 1'b0;                 // nút ban đầu coi như chưa nhấn
    end

    always @(posedge clk) begin            // khối tuần tự theo sườn lên clk
        sync1 <= btn_in;                   // lấy mẫu nút bấm thô
        sync2 <= sync1;                    // chuyển qua tầng đồng bộ thứ hai
        prev  <= btn_stable;               // nhớ mức ổn định của nhịp này cho lần sau
        if (sync2 == btn_stable) begin     // mức mới trùng mức đang giữ: không có gì thay đổi
            cnt <= 24'd0;                  // xóa bộ đếm
        end else begin                     // mức mới khác mức đang giữ: bắt đầu đo thời gian ổn định
            if (cnt == CNT_MAX - 1) begin  // đã ổn định đủ lâu
                btn_stable <= sync2;       // chấp nhận mức mới
                cnt        <= 24'd0;       // xóa bộ đếm
            end else begin                 // chưa đủ lâu
                cnt <= cnt + 24'd1;        // đếm tiếp
            end
        end
    end

    assign btn_rise = btn_stable & ~prev;  // =1 đúng một nhịp khi btn_stable vừa lên 1

endmodule
