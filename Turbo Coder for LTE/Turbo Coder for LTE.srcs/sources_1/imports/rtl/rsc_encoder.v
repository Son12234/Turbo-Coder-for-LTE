`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : rsc_encoder.v
// Chức năng : Bộ mã hóa thành phần RSC 8 trạng thái của Turbo code LTE
//             (3GPP TS 36.212, mục 5.1.3.2.1, Hình 5.1.3-2)
//
// Hàm truyền đạt : G(D) = [ 1 , g1(D)/g0(D) ]
//     g0(D) = 1 + D^2 + D^3   (đa thức hồi tiếp)
//     g1(D) = 1 + D   + D^3   (đa thức tạo bit parity)
//
// Cấu trúc (s0, s1, s2 là 3 thanh ghi dịch, giá trị ban đầu = 0):
//     fb = s1 ^ s2          : hồi tiếp theo g0(D)   (lấy D^2 và D^3)
//     a  = x ^ fb           : đầu vào của chuỗi thanh ghi dịch
//     z  = a ^ s0 ^ s2      : bit parity theo g1(D) (lấy D^0, D^1 và D^3)
//     mỗi nhịp clk khi en=1 : s0 <= a ; s1 <= s0 ; s2 <= s1
//
// Kết thúc trellis (termination): đóng "công tắc" hồi tiếp => bit đưa vào bộ mã
// chính bằng fb = s1 ^ s2, nên a = fb ^ fb = 0. Sau 3 nhịp, cả 3 thanh ghi về 0.
//==============================================================================
module rsc_encoder (
    input  wire       clk,         // xung nhịp hệ thống
    input  wire       rst,         // reset đồng bộ, mức cao: đưa s0,s1,s2 về 0
    input  wire       clr,         // xóa đồng bộ trạng thái về 0 (dùng khi bắt đầu khối mới)
    input  wire       en,          // cho phép dịch: =1 thì trạng thái được cập nhật ở sườn lên clk
    input  wire       term,        // =1: chế độ kết thúc trellis (công tắc ở vị trí dưới)
    input  wire       din,         // bit thông tin đầu vào (dùng khi term = 0)
    output wire       x_out,       // bit thực sự đi vào bộ mã: din (bình thường) hoặc fb (khi term = 1)
    output wire       z_out,       // bit parity (tổ hợp, phụ thuộc trạng thái hiện tại và x_out)
    output wire [2:0] state,       // trạng thái {s0,s1,s2} để quan sát/gỡ lỗi
    output wire       state_zero   // =1 khi cả ba thanh ghi đều bằng 0 (trellis đã kết thúc)
);

    reg s0;                        // thanh ghi dịch thứ nhất  (a trễ 1 nhịp)
    reg s1;                        // thanh ghi dịch thứ hai   (a trễ 2 nhịp)
    reg s2;                        // thanh ghi dịch thứ ba    (a trễ 3 nhịp)
    initial begin                  // giá trị ban đầu = 0 (chuẩn LTE) ngay từ lúc cấp nguồn/bắt đầu mô phỏng
        s0 = 1'b0;                 // s0 khởi tạo 0
        s1 = 1'b0;                 // s1 khởi tạo 0
        s2 = 1'b0;                 // s2 khởi tạo 0
    end

    wire fb;                       // tín hiệu hồi tiếp theo g0(D) = 1 + D^2 + D^3
    wire a;                        // đầu vào của chuỗi thanh ghi dịch
    assign fb = s1 ^ s2;           // hồi tiếp = XOR của đầu ra thanh ghi 2 (D^2) và thanh ghi 3 (D^3)
    assign x_out = term ? fb : din;// công tắc kết thúc: term=1 thì lấy chính fb làm bit vào
    assign a = x_out ^ fb;         // bộ cộng đầu vào: bình thường = din ^ fb ; khi term = 0 luôn bằng 0
    assign z_out = a ^ s0 ^ s2;    // parity theo g1(D) = 1 + D + D^3 (lấy a, s0, s2)

    assign state = {s0, s1, s2};   // gom 3 thanh ghi thành bus 3 bit
    assign state_zero = (state == 3'b000); // cờ báo trạng thái không

    always @(posedge clk) begin    // khối tuần tự, kích theo sườn lên của clk
        if (rst || clr) begin      // reset hoặc xóa trạng thái
            s0 <= 1'b0;            // xóa s0
            s1 <= 1'b0;            // xóa s1
            s2 <= 1'b0;            // xóa s2
        end else if (en) begin     // khi được cho phép, dịch chuỗi thanh ghi
            s0 <= a;               // s0 nhận đầu vào mới a
            s1 <= s0;              // s1 nhận giá trị cũ của s0
            s2 <= s1;              // s2 nhận giá trị cũ của s1
        end                        // en = 0: giữ nguyên trạng thái
    end

endmodule
