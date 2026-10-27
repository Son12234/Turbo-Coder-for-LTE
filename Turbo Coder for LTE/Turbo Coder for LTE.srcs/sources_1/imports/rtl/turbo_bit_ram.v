`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : turbo_bit_ram.v
// Chức năng : RAM 1 bit x 6144 ô, 2 cổng, đọc đồng bộ (đọc có trễ 1 nhịp).
//             Lưu toàn bộ khối bit đầu vào để bộ mã hóa thứ 2 có thể đọc theo
//             thứ tự đã xen (truy cập ngẫu nhiên) trong khi bộ 1 đọc tuần tự.
//   - Cổng A : ghi (khi we_a = 1) và đọc ; dùng ghi lúc nạp, đọc tuần tự lúc mã hóa.
//   - Cổng B : chỉ đọc ; địa chỉ lấy từ bộ xen QPP.
// Viết theo cú pháp mà Vivado nhận diện thành Block RAM (RAMB36, chế độ true dual-port).
//==============================================================================
module turbo_bit_ram #(
    parameter DEPTH = 6144,        // số ô nhớ: bằng K lớn nhất của LTE
    parameter AW    = 13           // số bit địa chỉ: 2^13 = 8192 >= 6144
)(
    input  wire          clk,      // xung nhịp chung cho cả hai cổng
    input  wire          we_a,     // cho phép ghi ở cổng A
    input  wire [AW-1:0] addr_a,   // địa chỉ cổng A (ghi hoặc đọc)
    input  wire          din_a,    // dữ liệu ghi vào cổng A
    output reg           dout_a,   // dữ liệu đọc ra cổng A (trễ 1 nhịp)
    input  wire [AW-1:0] addr_b,   // địa chỉ cổng B (chỉ đọc)
    output reg           dout_b    // dữ liệu đọc ra cổng B (trễ 1 nhịp)
);

    reg mem [0:DEPTH-1];           // mảng nhớ DEPTH ô, mỗi ô 1 bit
    integer n;                     // biến vòng lặp dùng cho khởi tạo

    initial begin                  // khởi tạo lúc bắt đầu mô phỏng / nạp cấu hình FPGA
        for (n = 0; n < DEPTH; n = n + 1)  // duyệt tất cả các ô nhớ
            mem[n] = 1'b0;         // xóa về 0 để không xuất hiện giá trị X trong dạng sóng
        dout_a = 1'b0;             // xóa ngõ ra cổng A
        dout_b = 1'b0;             // xóa ngõ ra cổng B
    end

    always @(posedge clk) begin    // mọi truy cập đều đồng bộ theo sườn lên clk
        if (we_a)                  // nếu được phép ghi
            mem[addr_a] <= din_a;  // ghi bit vào ô nhớ addr_a
        dout_a <= mem[addr_a];     // đọc cổng A (read-first: trả về giá trị cũ nếu đang ghi cùng ô)
        dout_b <= mem[addr_b];     // đọc cổng B
    end

endmodule
