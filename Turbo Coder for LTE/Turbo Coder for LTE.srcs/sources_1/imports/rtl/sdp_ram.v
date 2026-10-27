`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : sdp_ram.v
// Chức năng : RAM hai cổng đơn giản (Simple Dual-Port): 1 cổng GHI + 1 cổng ĐỌC,
//             cùng một xung nhịp, đọc đồng bộ (dữ liệu ra trễ 1 nhịp so với địa chỉ).
//             Viết theo cú pháp mà Vivado nhận diện thành Block RAM (RAMB36/RAMB18).
//
// Hành vi khi vừa ghi vừa đọc cùng một địa chỉ ở cùng một nhịp: cổng đọc trả về
// giá trị CŨ (read-first). Thiết kế bộ giải mã không bao giờ phụ thuộc vào trường hợp này.
//
// Được dùng trong bộ giải mã Turbo cho: bộ nhớ LLR kênh, bộ nhớ thông tin ngoại
// (extrinsic) và bộ đệm α/γ bên trong siso_decoder.
//==============================================================================
module sdp_ram #(
    parameter WIDTH = 8,               // số bit của mỗi ô nhớ
    parameter DEPTH = 6144,            // số ô nhớ
    parameter AW    = 13               // số bit địa chỉ (2^AW >= DEPTH)
)(
    input  wire             clk,       // xung nhịp chung cho cả hai cổng
    input  wire             we,        // cho phép ghi
    input  wire [AW-1:0]    waddr,     // địa chỉ ghi
    input  wire [WIDTH-1:0] wdata,     // dữ liệu ghi
    input  wire [AW-1:0]    raddr,     // địa chỉ đọc
    output reg  [WIDTH-1:0] rdata      // dữ liệu đọc (trễ 1 nhịp)
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];   // mảng nhớ DEPTH ô, mỗi ô WIDTH bit
    integer n;                         // biến vòng lặp cho khởi tạo

    initial begin                      // khởi tạo lúc bắt đầu mô phỏng / nạp cấu hình FPGA
        for (n = 0; n < DEPTH; n = n + 1)      // duyệt mọi ô nhớ
            mem[n] = {WIDTH{1'b0}};            // xóa về 0 để dạng sóng không có giá trị X
        rdata = {WIDTH{1'b0}};                 // xóa ngõ ra đọc
    end

    always @(posedge clk) begin        // mọi truy cập đều đồng bộ theo sườn lên clk
        if (we)                        // nếu được phép ghi
            mem[waddr] <= wdata;       // ghi dữ liệu vào ô waddr
        rdata <= mem[raddr];           // đọc ô raddr (giá trị cũ nếu đang ghi cùng ô)
    end

endmodule
