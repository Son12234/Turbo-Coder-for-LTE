`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : qpp_interleaver.v
// Chức năng : Bộ sinh địa chỉ xen QPP (Quadratic Permutation Polynomial) cho
//             Turbo code LTE:   Pi(i) = (f1*i + f2*i^2) mod K
//
// Để KHÔNG phải dùng bộ nhân và phép chia lấy dư, ta dùng công thức đệ quy
// (chỉ gồm các phép cộng và trừ có điều kiện):
//     Pi(0)   = 0
//     g(0)    = (f1 + f2) mod K
//     Pi(i+1) = (Pi(i) + g(i)) mod K
//     g(i+1)  = (g(i) + 2*f2)  mod K
// Chứng minh: Pi(i+1) - Pi(i) = f1 + f2*(2i+1) = g(i)  và  g(i+1) - g(i) = 2*f2.
//
// Cách dùng:
//   - Cấp k, f1, f2 ổn định (lấy từ qpp_param_rom), đưa xung init 1 nhịp  => addr = Pi(0) = 0.
//   - Mỗi nhịp có step = 1 thì addr chuyển sang Pi(i+1).
//==============================================================================
module qpp_interleaver (
    input  wire        clk,        // xung nhịp hệ thống
    input  wire        rst,        // reset đồng bộ, mức cao
    input  wire        init,       // xung nạp tham số, đưa bộ đếm về i = 0
    input  wire        step,       // =1: tiến sang chỉ số i+1 ở sườn lên tiếp theo
    input  wire [12:0] k,          // kích thước khối K (40..6144)
    input  wire [12:0] f1,         // hệ số f1 của QPP
    input  wire [12:0] f2,         // hệ số f2 của QPP
    output reg  [12:0] addr        // địa chỉ đã xen Pi(i) của chỉ số i hiện tại
);

    reg [12:0] g;                  // g(i) = (f1 + f2 + 2*f2*i) mod K : bước nhảy hiện tại của địa chỉ
    reg [12:0] inc;                // inc  = (2*f2) mod K             : độ tăng của g sau mỗi bước
    initial begin                  // giá trị khởi tạo để dạng sóng không có X trước khi reset
        addr = 13'd0;              // địa chỉ ban đầu = 0
        g    = 13'd0;              // bước nhảy ban đầu = 0
        inc  = 13'd0;              // độ tăng ban đầu = 0
    end

    // Hàm cộng modulo: trả về (a + b) mod m, với điều kiện a < m và b < m.
    function [12:0] mod_add;
        input [12:0] a;            // toán hạng thứ nhất (a < m)
        input [12:0] b;            // toán hạng thứ hai  (b < m)
        input [12:0] m;            // modulo (chính là K)
        reg   [13:0] s;            // tổng a + b, cần 14 bit vì có thể lớn hơn 8191
        reg   [13:0] d;            // hiệu s - m, bit 13 = 1 nghĩa là s < m (kết quả âm)
        begin
            s = {1'b0, a} + {1'b0, b};     // cộng hai số, mở rộng lên 14 bit
            d = s - {1'b0, m};             // trừ thử m
            if (d[13])                     // d âm => s < m => không cần trừ
                mod_add = s[12:0];         // giữ nguyên tổng
            else                           // d không âm => s >= m
                mod_add = d[12:0];         // dùng kết quả sau khi trừ m
        end
    endfunction

    always @(posedge clk) begin            // khối tuần tự theo sườn lên clk
        if (rst) begin                     // reset đồng bộ
            addr <= 13'd0;                 // địa chỉ về 0
            g    <= 13'd0;                 // bước nhảy về 0
            inc  <= 13'd0;                 // độ tăng về 0
        end else if (init) begin           // nạp tham số cho khối mới
            addr <= 13'd0;                 // Pi(0) = 0
            g    <= mod_add(f1, f2, k);    // g(0) = (f1 + f2) mod K
            inc  <= mod_add(f2, f2, k);    // 2*f2 mod K
        end else if (step) begin           // tiến một chỉ số
            addr <= mod_add(addr, g, k);   // Pi(i+1) = (Pi(i) + g(i)) mod K
            g    <= mod_add(g, inc, k);    // g(i+1) = (g(i) + 2*f2) mod K
        end                                // không init/step: giữ nguyên giá trị
    end

endmodule
