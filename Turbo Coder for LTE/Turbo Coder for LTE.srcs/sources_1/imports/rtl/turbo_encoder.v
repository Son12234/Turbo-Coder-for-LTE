`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : turbo_encoder.v
// Chức năng : Bộ mã hóa Turbo LTE tốc độ 1/3 (3GPP TS 36.212, mục 5.1.3.2)
//             - 2 bộ mã hóa thành phần RSC 8 trạng thái (rsc_encoder)
//             - 1 bộ xen nội QPP (qpp_interleaver + qpp_param_rom)
//             - kết thúc trellis bằng 12 bit đuôi (tail bits)
//
// Ba luồng ngõ ra (mỗi luồng dài K + 4 bit, theo đúng chuẩn):
//     d0 : bit hệ thống x(i)  , i = 0..K-1  + 4 bit đuôi
//     d1 : parity z(i)  của bộ mã 1         + 4 bit đuôi
//     d2 : parity z'(i) của bộ mã 2 (đầu vào đã xen) + 4 bit đuôi
//
// 12 bit đuôi (x,z: bộ mã 1 ; x',z': bộ mã 2), đúng mục 5.1.3.2.2:
//     d0[K..K+3] = { x(K)   , z(K+1) , x'(K)   , z'(K+1) }
//     d1[K..K+3] = { z(K)   , x(K+2) , z'(K)   , x'(K+2) }
//     d2[K..K+3] = { x(K+1) , z(K+2) , x'(K+1) , z'(K+2) }
//
// Quy trình hoạt động (mỗi khối):
//   1) IDLE  : chờ xung start, chốt K.
//   2) CHECK : tra bảng QPP; K sai => báo err_k và quay về IDLE; K đúng => nạp bộ xen.
//   3) LOAD  : nhận K bit (1 bit/nhịp khi in_valid=1), ghi vào RAM.
//   4) ENC   : mỗi nhịp đọc đồng thời x(i) (tuần tự) và x(Pi(i)) (đã xen) từ RAM,
//              hai bộ RSC chạy song song => d0,d1,d2 ra thẳng hàng cùng chỉ số.
//   5) DRAIN : 1 nhịp để bit cuối đi hết đường ống.
//   6) TERM  : 3 nhịp, cả hai bộ RSC ở chế độ kết thúc => trạng thái về 000; lưu 12 bit đuôi.
//   7) TAIL  : 4 nhịp phát 12 bit đuôi trên d0,d1,d2 rồi trở về IDLE.
//==============================================================================
module turbo_encoder (
    input  wire        clk,        // xung nhịp hệ thống
    input  wire        rst,        // reset đồng bộ, mức cao
    // ---- điều khiển khối ----
    input  wire        start,      // xung 1 nhịp để bắt đầu khối mới (chỉ nhận khi busy = 0)
    input  wire [12:0] k_in,       // kích thước khối K (một trong 188 giá trị 40..6144), chốt khi start
    // ---- cổng nhận bit thông tin (bắt tay valid/ready) ----
    output wire        in_ready,   // =1 khi bộ mã sẵn sàng nhận bit
    input  wire        in_valid,   // =1 khi din hợp lệ ở nhịp này
    input  wire        din,        // bit thông tin
    // ---- cổng ngõ ra mã hóa (đã đăng ký, trễ so với đầu vào) ----
    output reg         out_valid,  // =1 khi d0,d1,d2 hợp lệ
    output reg         d0,         // luồng hệ thống
    output reg         d1,         // luồng parity 1
    output reg         d2,         // luồng parity 2
    output reg         out_tail,   // =1 khi đang phát 4 nhịp bit đuôi (chỉ số K..K+3)
    output reg         out_last,   // =1 ở nhịp phát cuối cùng của khối (chỉ số K+3)
    // ---- trạng thái ----
    output wire        busy,       // =1 từ lúc nhận start đến khi phát xong khối
    output reg         err_k,      // xung 1 nhịp báo K không hợp lệ
    output reg         term_ok     // =1 nếu cả hai bộ RSC đã về trạng thái 000 sau kết thúc trellis
);

    // ---------------- mã hóa trạng thái FSM ----------------
    localparam [2:0] S_IDLE  = 3'd0;   // chờ start
    localparam [2:0] S_CHECK = 3'd1;   // tra bảng QPP, nạp bộ xen, xóa trạng thái RSC
    localparam [2:0] S_LOAD  = 3'd2;   // nhận K bit vào RAM
    localparam [2:0] S_ENC   = 3'd3;   // mã hóa K bit (phát địa chỉ đọc RAM)
    localparam [2:0] S_DRAIN = 3'd4;   // xả đường ống 1 nhịp
    localparam [2:0] S_TERM  = 3'd5;   // 3 nhịp kết thúc trellis
    localparam [2:0] S_TAIL  = 3'd6;   // 4 nhịp phát bit đuôi

    reg  [2:0]  state;                 // trạng thái hiện tại của FSM
    reg  [12:0] k_reg;                 // K đã chốt
    reg  [12:0] load_cnt;              // đếm số bit đã nhận (cũng là địa chỉ ghi RAM)
    reg  [12:0] issue_cnt;             // chỉ số i đang đọc ở pha mã hóa (cũng là địa chỉ đọc tuần tự)
    reg  [1:0]  term_cnt;              // đếm 3 nhịp kết thúc trellis: 0,1,2
    reg  [1:0]  tail_cnt;              // đếm 4 nhịp phát đuôi: 0,1,2,3
    reg         v1;                    // cờ "dữ liệu từ RAM ở tầng 1 đường ống là hợp lệ"

    reg  [2:0]  tx1;                   // 3 bit x của bộ mã 1 trong lúc kết thúc: x(K), x(K+1), x(K+2)
    reg  [2:0]  tz1;                   // 3 bit z của bộ mã 1 trong lúc kết thúc: z(K), z(K+1), z(K+2)
    reg  [2:0]  tx2;                   // 3 bit x' của bộ mã 2 : x'(K), x'(K+1), x'(K+2)
    reg  [2:0]  tz2;                   // 3 bit z' của bộ mã 2 : z'(K), z'(K+1), z'(K+2)

    initial begin                      // giá trị khởi tạo cho mọi thanh ghi (dạng sóng sạch, không X)
        state = S_IDLE; k_reg = 13'd0; load_cnt = 13'd0; issue_cnt = 13'd0; // FSM và các bộ đếm
        term_cnt = 2'd0; tail_cnt = 2'd0; v1 = 1'b0;                        // bộ đếm đuôi và cờ đường ống
        tx1 = 3'd0; tz1 = 3'd0; tx2 = 3'd0; tz2 = 3'd0;                     // thanh ghi bit đuôi
        out_valid = 1'b0; d0 = 1'b0; d1 = 1'b0; d2 = 1'b0;                  // ngõ ra dữ liệu
        out_tail = 1'b0; out_last = 1'b0; err_k = 1'b0; term_ok = 1'b0;    // ngõ ra cờ
    end

    // ---------------- tra bảng tham số QPP ----------------
    wire [12:0] f1;                    // hệ số f1 tương ứng K
    wire [12:0] f2;                    // hệ số f2 tương ứng K
    wire        k_valid;               // K có nằm trong bảng LTE hay không
    qpp_param_rom u_rom (
        .k    (k_reg),                 // tra theo K đã chốt
        .f1   (f1),                    // trả về f1
        .f2   (f2),                    // trả về f2
        .valid(k_valid)                // trả về cờ hợp lệ
    );

    // ---------------- bộ xen QPP ----------------
    wire        qpp_init;              // xung nạp tham số cho bộ xen
    wire        qpp_step;              // xung tiến bộ xen sang chỉ số kế tiếp
    wire [12:0] qpp_addr;              // Pi(i): địa chỉ đã xen
    assign qpp_init = (state == S_CHECK) && k_valid; // nạp ngay khi K hợp lệ ở trạng thái CHECK
    assign qpp_step = (state == S_ENC);              // tiến 1 chỉ số mỗi nhịp ở pha mã hóa
    qpp_interleaver u_qpp (
        .clk (clk),                    // xung nhịp
        .rst (rst),                    // reset
        .init(qpp_init),               // nạp tham số
        .step(qpp_step),               // bước kế tiếp
        .k   (k_reg),                  // K
        .f1  (f1),                     // f1
        .f2  (f2),                     // f2
        .addr(qpp_addr)                // Pi(i)
    );

    // ---------------- RAM lưu khối bit vào ----------------
    wire        load_fire;             // =1 khi một bit hợp lệ được nhận ở pha LOAD
    wire [12:0] ram_addr_a;            // địa chỉ cổng A: ghi khi LOAD, đọc tuần tự khi ENC
    wire        ram_dout_a;            // bit x(i) đọc tuần tự (trễ 1 nhịp)
    wire        ram_dout_b;            // bit x(Pi(i)) đọc theo thứ tự xen (trễ 1 nhịp)
    assign load_fire  = (state == S_LOAD) && in_valid;             // nhận bit khi đang LOAD và in_valid = 1
    assign ram_addr_a = (state == S_LOAD) ? load_cnt : issue_cnt;  // chọn địa chỉ ghi hoặc đọc
    turbo_bit_ram u_ram (
        .clk   (clk),                  // xung nhịp
        .we_a  (load_fire),            // ghi khi nhận bit
        .addr_a(ram_addr_a),           // địa chỉ cổng A
        .din_a (din),                  // bit cần ghi
        .dout_a(ram_dout_a),           // bit đọc tuần tự
        .addr_b(qpp_addr),             // địa chỉ đã xen
        .dout_b(ram_dout_b)            // bit đọc theo thứ tự xen
    );

    // ---------------- hai bộ mã hóa thành phần ----------------
    wire        enc_clr;               // xóa trạng thái hai bộ RSC
    wire        enc_en;                // cho phép dịch hai bộ RSC
    wire        enc_term;              // chế độ kết thúc trellis
    wire        x1, z1, x2, z2;        // (x,z) của bộ 1 và (x',z') của bộ 2
    wire [2:0]  st1, st2;              // trạng thái của hai bộ RSC (để kiểm tra kết thúc)
    assign enc_clr  = (state == S_CHECK);                  // bắt đầu khối mới: xóa về 000
    assign enc_en   = v1 || (state == S_TERM);             // dịch khi có dữ liệu hợp lệ hoặc khi kết thúc trellis
    assign enc_term = (state == S_TERM);                   // công tắc kết thúc chỉ đóng ở pha TERM

    rsc_encoder u_enc1 (               // bộ mã hóa 1: đầu vào theo thứ tự gốc
        .clk(clk), .rst(rst), .clr(enc_clr), .en(enc_en), .term(enc_term),
        .din(ram_dout_a),              // bit x(i)
        .x_out(x1), .z_out(z1), .state(st1), .state_zero()
    );
    rsc_encoder u_enc2 (               // bộ mã hóa 2: đầu vào theo thứ tự đã xen
        .clk(clk), .rst(rst), .clr(enc_clr), .en(enc_en), .term(enc_term),
        .din(ram_dout_b),              // bit x(Pi(i))
        .x_out(x2), .z_out(z2), .state(st2), .state_zero()
    );

    // ---------------- các tín hiệu trạng thái ----------------
    wire [12:0] k_last;                // K - 1: chỉ số cuối cùng của khối
    assign k_last   = k_reg - 13'd1;   // tính K-1
    assign in_ready = (state == S_LOAD);   // chỉ nhận bit ở pha LOAD
    assign busy     = (state != S_IDLE);   // bận mọi lúc trừ IDLE

    // ---------------- FSM và các bộ đếm ----------------
    always @(posedge clk) begin        // khối tuần tự chính
        if (rst) begin                 // reset đồng bộ
            state     <= S_IDLE;       // về trạng thái chờ
            k_reg     <= 13'd0;        // xóa K
            load_cnt  <= 13'd0;        // xóa bộ đếm nạp
            issue_cnt <= 13'd0;        // xóa bộ đếm đọc
            term_cnt  <= 2'd0;         // xóa bộ đếm kết thúc
            tail_cnt  <= 2'd0;         // xóa bộ đếm đuôi
            v1        <= 1'b0;         // chưa có dữ liệu trong đường ống
            err_k     <= 1'b0;         // không lỗi
            term_ok   <= 1'b0;         // chưa kết thúc trellis
            tx1 <= 3'd0; tz1 <= 3'd0;  // xóa các thanh ghi bit đuôi bộ 1
            tx2 <= 3'd0; tz2 <= 3'd0;  // xóa các thanh ghi bit đuôi bộ 2
        end else begin                 // hoạt động bình thường
            err_k <= 1'b0;             // mặc định: xung lỗi chỉ kéo dài 1 nhịp
            v1    <= (state == S_ENC); // dữ liệu RAM ở nhịp sau hợp lệ nếu nhịp này đang phát địa chỉ đọc
            case (state)               // chuyển trạng thái
                S_IDLE: begin          // ---- chờ bắt đầu ----
                    if (start) begin   // có lệnh bắt đầu
                        k_reg <= k_in; // chốt kích thước khối
                        state <= S_CHECK; // sang bước kiểm tra
                    end
                end
                S_CHECK: begin         // ---- kiểm tra K, nạp bộ xen, xóa RSC ----
                    term_ok <= 1'b0;   // xóa cờ kết thúc của khối trước
                    if (k_valid) begin // K hợp lệ
                        load_cnt <= 13'd0; // bắt đầu ghi từ địa chỉ 0
                        state    <= S_LOAD;// sang pha nhận bit
                    end else begin     // K không hợp lệ
                        err_k <= 1'b1; // báo lỗi 1 nhịp
                        state <= S_IDLE; // bỏ qua lệnh start này
                    end
                end
                S_LOAD: begin          // ---- nhận K bit ----
                    if (in_valid) begin// chỉ khi bit hợp lệ
                        load_cnt <= load_cnt + 13'd1; // tăng địa chỉ ghi
                        if (load_cnt == k_last) begin // vừa nhận bit cuối
                            issue_cnt <= 13'd0;       // chuẩn bị đọc từ i = 0
                            state     <= S_ENC;       // sang pha mã hóa
                        end
                    end
                end
                S_ENC: begin           // ---- phát địa chỉ đọc cho K bit ----
                    if (issue_cnt == k_last)          // đã phát địa chỉ cuối (i = K-1)
                        state <= S_DRAIN;             // chờ bit cuối đi hết đường ống (giữ issue_cnt = K-1 để địa chỉ RAM luôn hợp lệ)
                    else                              // chưa phải chỉ số cuối
                        issue_cnt <= issue_cnt + 13'd1; // chuyển sang chỉ số kế tiếp
                end
                S_DRAIN: begin         // ---- xả đường ống (bộ RSC xử lý bit cuối ở nhịp này) ----
                    term_cnt <= 2'd0;  // chuẩn bị đếm 3 nhịp kết thúc
                    state    <= S_TERM;// sang pha kết thúc trellis
                end
                S_TERM: begin          // ---- kết thúc trellis: 3 nhịp ----
                    tx1[term_cnt] <= x1; // lưu x(K+term_cnt) của bộ 1
                    tz1[term_cnt] <= z1; // lưu z(K+term_cnt) của bộ 1
                    tx2[term_cnt] <= x2; // lưu x'(K+term_cnt) của bộ 2
                    tz2[term_cnt] <= z2; // lưu z'(K+term_cnt) của bộ 2
                    term_cnt <= term_cnt + 2'd1;      // nhịp kết thúc kế tiếp
                    if (term_cnt == 2'd2) begin       // đã xong 3 nhịp
                        tail_cnt <= 2'd0;             // chuẩn bị phát đuôi
                        state    <= S_TAIL;           // sang pha phát đuôi
                    end
                end
                S_TAIL: begin          // ---- phát 4 nhịp bit đuôi ----
                    term_ok  <= (st1 == 3'b000) && (st2 == 3'b000); // cả hai bộ RSC phải về 000
                    tail_cnt <= tail_cnt + 2'd1;      // nhịp đuôi kế tiếp
                    if (tail_cnt == 2'd3)             // đã phát nhịp cuối
                        state <= S_IDLE;              // xong khối, về chờ
                end
                default: state <= S_IDLE; // trạng thái không xác định: về IDLE cho an toàn
            endcase
        end
    end

    // ---------------- thanh ghi ngõ ra ----------------
    always @(posedge clk) begin        // ngõ ra được đăng ký để tránh glitch và dễ đạt tần số cao
        if (rst) begin                 // reset đồng bộ
            out_valid <= 1'b0;         // không có dữ liệu
            d0        <= 1'b0;         // xóa d0
            d1        <= 1'b0;         // xóa d1
            d2        <= 1'b0;         // xóa d2
            out_tail  <= 1'b0;         // không phải pha đuôi
            out_last  <= 1'b0;         // không phải nhịp cuối
        end else begin                 // hoạt động bình thường
            out_valid <= 1'b0;         // mặc định: ngõ ra không hợp lệ
            d0        <= 1'b0;         // mặc định: ép về 0 khi không hợp lệ (dạng sóng sạch)
            d1        <= 1'b0;         // mặc định như trên
            d2        <= 1'b0;         // mặc định như trên
            out_tail  <= 1'b0;         // mặc định: không phải đuôi
            out_last  <= 1'b0;         // mặc định: không phải nhịp cuối
            if (v1) begin              // tầng 1 có dữ liệu hợp lệ: phát bit thông tin
                out_valid <= 1'b1;     // đánh dấu hợp lệ
                d0        <= ram_dout_a; // bit hệ thống x(i)
                d1        <= z1;       // parity của bộ mã 1
                d2        <= z2;       // parity của bộ mã 2 (đầu vào đã xen)
            end else if (state == S_TAIL) begin // pha phát bit đuôi
                out_valid <= 1'b1;     // đánh dấu hợp lệ
                out_tail  <= 1'b1;     // đánh dấu đây là bit đuôi
                out_last  <= (tail_cnt == 2'd3); // nhịp thứ 4 là nhịp cuối của khối
                case (tail_cnt)        // chọn bit đuôi theo đúng bảng chuẩn
                    2'd0: begin d0 <= tx1[0]; d1 <= tz1[0]; d2 <= tx1[1]; end // chỉ số K  : x(K),   z(K),   x(K+1)
                    2'd1: begin d0 <= tz1[1]; d1 <= tx1[2]; d2 <= tz1[2]; end // chỉ số K+1: z(K+1), x(K+2), z(K+2)
                    2'd2: begin d0 <= tx2[0]; d1 <= tz2[0]; d2 <= tx2[1]; end // chỉ số K+2: x'(K),  z'(K),  x'(K+1)
                    2'd3: begin d0 <= tz2[1]; d1 <= tx2[2]; d2 <= tz2[2]; end // chỉ số K+3: z'(K+1),x'(K+2),z'(K+2)
                endcase
            end
        end
    end

endmodule
