`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : turbo_decoder.v
// Chức năng : Bộ giải mã Turbo LTE (tốc độ 1/3) lặp, thuật toán Max-Log-MAP.
//             Gồm: 1 bộ SISO dùng chung cho cả hai bộ giải mã thành phần (chạy lần lượt),
//             bộ xen QPP (trùng với bộ mã hóa), 5 bộ nhớ LLR/thông tin ngoại.
//
// QUY ƯỚC LLR : dương => bit 0 ; âm => bit 1  (bit 0 -> +A, bit 1 -> -A khi ánh xạ BPSK).
//
// GIAO TIẾP LLR VÀO (ngược lại với bộ mã hóa): sau start, khi in_ready = 1 bên ngoài đưa
// K+4 "bộ ba" LLR (in_l0, in_l1, in_l2) theo đúng thứ tự d0, d1, d2 do bộ mã hóa phát ra
// (K bộ ba thông tin rồi 4 bộ ba bit đuôi); có thể ngắt quãng bằng in_valid = 0.
//
// QUY TRÌNH:
//   IDLE -> CHECK (tra bảng QPP, K sai => err_k) -> LOAD (nhận K+4 bộ ba)
//        -> lặp  iters lần { SISO1 (thứ tự gốc) ; SISO2 (thứ tự đã xen QPP) }
//        -> OUT (phát K bit đã giải mã theo thứ tự gốc) -> IDLE.
//   Một nửa vòng lặp gồm: P_START (1 nhịp) -> P_READ (K nhịp phát địa chỉ đọc)
//   -> P_WAIT (chờ SISO phát xong thông tin ngoại).
//
// BỘ NHỚ (mỗi ô LW bit):
//   ls_mem  : LLR hệ thống d0[0..K-1]        lp1_mem : LLR parity 1 d1[0..K-1]
//   lp2_mem : LLR parity 2 d2[0..K-1]
//   ext1    : thông tin ngoại của SISO1 (chỉ số gốc)   ext2 : của SISO2 (đã giải xen, chỉ số gốc)
//   SISO1 : Ls = ls[k], Lp = lp1[k], La = ext2[k]      -> ghi ext1[k]
//   SISO2 : Ls = ls[Pi(j)], Lp = lp2[j], La = ext1[Pi(j)] -> ghi ext2[Pi(j)]  (giải xen bằng cách ghi theo Pi)
//   Quyết định cuối: bit(n) = [ ls[n] + ext1[n] + ext2[n] < 0 ].
//==============================================================================
module turbo_decoder #(
    parameter LW = 7,                  // số bit có dấu của LLR
    parameter SW = 10                  // số bit có dấu của metric trạng thái trong SISO
)(
    input  wire                 clk,       // xung nhịp hệ thống
    input  wire                 rst,       // reset đồng bộ, mức cao
    // ---- điều khiển khối ----
    input  wire                 start,     // xung 1 nhịp bắt đầu khối mới (chỉ nhận khi busy = 0)
    input  wire [12:0]          k_in,      // kích thước khối K (một trong 188 giá trị 40..6144)
    input  wire [3:0]           iter_in,   // số vòng lặp đầy đủ (1..15); 0 được hiểu là 1
    // ---- cổng LLR vào ----
    output wire                 in_ready,  // =1 khi bộ giải mã sẵn sàng nhận bộ ba LLR
    input  wire                 in_valid,  // =1 khi in_l0/in_l1/in_l2 hợp lệ
    input  wire signed [LW-1:0] in_l0,     // LLR của d0 (bit hệ thống)
    input  wire signed [LW-1:0] in_l1,     // LLR của d1 (parity 1)
    input  wire signed [LW-1:0] in_l2,     // LLR của d2 (parity 2)
    // ---- cổng bit giải mã ra ----
    output reg                  out_valid, // =1 khi out_bit hợp lệ
    output reg                  out_bit,   // bit đã giải mã
    output reg  [12:0]          out_idx,   // chỉ số của bit (0..K-1)
    output reg                  out_last,  // =1 ở bit cuối cùng của khối
    // ---- trạng thái ----
    output wire                 busy,      // =1 từ lúc nhận start đến khi phát xong bit cuối
    output reg                  err_k      // xung 1 nhịp báo K không hợp lệ
);

    localparam AW = 13;                    // số bit địa chỉ
    localparam [3:0] D_IDLE   = 4'd0;      // chờ start
    localparam [3:0] D_CHECK  = 4'd1;      // kiểm tra K, nạp tham số
    localparam [3:0] D_LOAD   = 4'd2;      // nhận K+4 bộ ba LLR
    localparam [3:0] D_PSTART = 4'd3;      // 1 nhịp: khởi động SISO và bộ xen
    localparam [3:0] D_PREAD  = 4'd4;      // K nhịp: phát địa chỉ đọc các bộ nhớ
    localparam [3:0] D_PWAIT  = 4'd5;      // chờ SISO xong
    localparam [3:0] D_OUT    = 4'd6;      // phát địa chỉ đọc để lấy quyết định cuối
    localparam [3:0] D_FLUSH  = 4'd7;      // chờ bit cuối ra khỏi đường ống

    reg  [3:0]  state;                     // trạng thái FSM
    reg  [12:0] k_reg;                     // K đã chốt
    reg  [3:0]  iters;                     // số vòng lặp cần chạy
    reg  [3:0]  iter_cnt;                  // số vòng lặp đã hoàn thành
    reg         ph;                        // pha: 0 = SISO1, 1 = SISO2
    reg  [12:0] load_cnt;                  // số bộ ba đã nhận (0..K+3)
    reg  [12:0] rd_cnt;                    // chỉ số phần tử đang phát địa chỉ (pha SISO) / bit đang phát (OUT)
    reg         rd_v;                      // =1: dữ liệu bộ nhớ ở nhịp này hợp lệ (địa chỉ đã phát nhịp trước)
    reg  [12:0] rd_tag;                    // nhãn (địa chỉ ghi lại thông tin ngoại) của phần tử đó
    reg         o_v1;                      // OUT: dữ liệu bộ nhớ ở nhịp này hợp lệ
    reg  [12:0] o_idx1;                    // OUT: chỉ số bit của dữ liệu đó
    reg         o_last1;                   // OUT: đó có phải bit cuối không

    // 4 bộ ba LLR đuôi: tl0[i] = d0[K+i], tl1[i] = d1[K+i], tl2[i] = d2[K+i]
    reg  [4*LW-1:0] tl0, tl1, tl2;         // dạng bus, phần tử i nằm ở [i*LW +: LW]

    initial begin                          // khởi tạo để dạng sóng không có X
        state = D_IDLE; k_reg = 0; iters = 4'd1; iter_cnt = 0; ph = 1'b0;      // FSM và tham số
        load_cnt = 0; rd_cnt = 0; rd_v = 1'b0; rd_tag = 0;                      // bộ đếm
        o_v1 = 1'b0; o_idx1 = 0; o_last1 = 1'b0;                                // đường ống OUT
        tl0 = 0; tl1 = 0; tl2 = 0;                                              // LLR đuôi
        out_valid = 1'b0; out_bit = 1'b0; out_idx = 0; out_last = 1'b0; err_k = 1'b0; // ngõ ra
    end

    // ---------------- tra bảng tham số QPP ----------------
    wire [12:0] f1, f2;                    // hệ số QPP
    wire        k_valid;                   // K có hợp lệ không
    qpp_param_rom u_rom (.k(k_reg), .f1(f1), .f2(f2), .valid(k_valid));

    // ---------------- bộ xen QPP (chỉ dùng ở pha SISO2) ----------------
    wire        qpp_init = (state == D_PSTART);    // nạp lại mỗi nửa vòng lặp, đưa về Pi(0) = 0
    wire        qpp_step = (state == D_PREAD);     // tiến 1 chỉ số mỗi nhịp phát địa chỉ
    wire [12:0] qpp_addr;                          // Pi(j)
    qpp_interleaver u_qpp (
        .clk(clk), .rst(rst), .init(qpp_init), .step(qpp_step),
        .k(k_reg), .f1(f1), .f2(f2), .addr(qpp_addr)
    );

    // ---------------- 5 bộ nhớ LLR ----------------
    wire load_beat = (state == D_LOAD) && in_valid;        // nhận một bộ ba ở pha LOAD
    wire load_info = load_beat && (load_cnt < k_reg);      // bộ ba đó thuộc phần thông tin (chỉ số < K)
    wire [LW-1:0] ls_rd, lp1_rd, lp2_rd, e1_rd, e2_rd;     // dữ liệu đọc (trễ 1 nhịp)
    wire [12:0] a_ls  = (state == D_OUT) ? rd_cnt : (ph ? qpp_addr : rd_cnt); // địa chỉ đọc ls: Pi(j) ở SISO2
    wire [12:0] a_e1  = (state == D_OUT) ? rd_cnt : qpp_addr;                 // ext1 đọc theo Pi(j) (SISO2)
    wire [12:0] a_e2  = rd_cnt;                                               // ext2 đọc theo j (SISO1)

    sdp_ram #(.WIDTH(LW), .DEPTH(6144), .AW(AW)) u_ls  (   // LLR hệ thống
        .clk(clk), .we(load_info), .waddr(load_cnt), .wdata(in_l0), .raddr(a_ls), .rdata(ls_rd));
    sdp_ram #(.WIDTH(LW), .DEPTH(6144), .AW(AW)) u_lp1 (   // LLR parity 1
        .clk(clk), .we(load_info), .waddr(load_cnt), .wdata(in_l1), .raddr(rd_cnt), .rdata(lp1_rd));
    sdp_ram #(.WIDTH(LW), .DEPTH(6144), .AW(AW)) u_lp2 (   // LLR parity 2
        .clk(clk), .we(load_info), .waddr(load_cnt), .wdata(in_l2), .raddr(rd_cnt), .rdata(lp2_rd));

    // ---------------- SISO dùng chung ----------------
    wire             s_ov, s_last;                 // out_valid, out_last của SISO
    wire signed [LW-1:0] s_le;                     // thông tin ngoại ra
    wire [12:0]      s_tag;                        // nhãn ra
    wire             siso_start = (state == D_PSTART); // khởi động SISO ở nhịp P_START

    // LLR đuôi cho từng SISO (xem 36.212 mục 5.1.3.2.2):
    //   SISO1: x(K)=d0[K], x(K+1)=d2[K], x(K+2)=d1[K+1] ; z(K)=d1[K], z(K+1)=d0[K+1], z(K+2)=d2[K+1]
    //   SISO2: x'(K)=d0[K+2], x'(K+1)=d2[K+2], x'(K+2)=d1[K+3] ; z'(K)=d1[K+2], z'(K+1)=d0[K+3], z'(K+2)=d2[K+3]
    wire [3*LW-1:0] tx1 = {tl1[1*LW +: LW], tl2[0*LW +: LW], tl0[0*LW +: LW]};  // {x(K+2),x(K+1),x(K)}
    wire [3*LW-1:0] tz1 = {tl2[1*LW +: LW], tl0[1*LW +: LW], tl1[0*LW +: LW]};  // {z(K+2),z(K+1),z(K)}
    wire [3*LW-1:0] tx2 = {tl1[3*LW +: LW], tl2[2*LW +: LW], tl0[2*LW +: LW]};  // {x'(K+2),x'(K+1),x'(K)}
    wire [3*LW-1:0] tz2 = {tl2[3*LW +: LW], tl0[3*LW +: LW], tl1[2*LW +: LW]};  // {z'(K+2),z'(K+1),z'(K)}

    siso_decoder #(.LW(LW), .SW(SW), .AW(AW), .DEPTH(6144)) u_siso (
        .clk(clk), .rst(rst), .start(siso_start), .k_len(k_reg),
        .in_valid(rd_v),                           // dữ liệu bộ nhớ hợp lệ ở nhịp sau khi phát địa chỉ
        .in_ls(ls_rd),                             // LLR hệ thống (đã đọc theo thứ tự gốc hoặc xen)
        .in_la(ph ? e1_rd : e2_rd),                // tiên nghiệm: ext1 (SISO2) hoặc ext2 (SISO1)
        .in_lp(ph ? lp2_rd : lp1_rd),              // parity của bộ thành phần tương ứng
        .in_tag(rd_tag),                           // nhãn để ghi lại thông tin ngoại
        .tail_x(ph ? tx2 : tx1), .tail_z(ph ? tz2 : tz1),
        .out_valid(s_ov), .out_le(s_le), .out_tag(s_tag), .out_last(s_last)
    );

    // ext1: ghi kết quả SISO1 ; ext2: xóa về 0 trong lúc LOAD, rồi ghi kết quả SISO2
    sdp_ram #(.WIDTH(LW), .DEPTH(6144), .AW(AW)) u_ext1 (
        .clk(clk), .we(s_ov && !ph), .waddr(s_tag), .wdata(s_le), .raddr(a_e1), .rdata(e1_rd));
    sdp_ram #(.WIDTH(LW), .DEPTH(6144), .AW(AW)) u_ext2 (
        .clk(clk),
        .we(load_info || (s_ov && ph)),                            // xóa khi LOAD hoặc ghi kết quả SISO2
        .waddr((state == D_LOAD) ? load_cnt : s_tag),              // địa chỉ ghi
        .wdata((state == D_LOAD) ? {LW{1'b0}} : s_le),             // dữ liệu ghi
        .raddr(a_e2), .rdata(e2_rd));

    // ---------------- quyết định cuối: ls + ext1 + ext2 ----------------
    wire signed [LW+1:0] post = $signed(ls_rd) + $signed(e1_rd) + $signed(e2_rd); // LLR hậu nghiệm xấp xỉ

    assign in_ready = (state == D_LOAD);           // chỉ nhận LLR ở pha LOAD
    assign busy     = (state != D_IDLE);           // bận mọi lúc trừ IDLE

    wire [12:0] k_last = k_reg - 13'd1;            // K-1

    // ---------------- FSM ----------------
    always @(posedge clk) begin
        if (rst) begin                             // reset đồng bộ
            state <= D_IDLE; k_reg <= 13'd0; iters <= 4'd1; iter_cnt <= 4'd0; ph <= 1'b0;
            load_cnt <= 13'd0; rd_cnt <= 13'd0; rd_v <= 1'b0; rd_tag <= 13'd0;
            o_v1 <= 1'b0; o_idx1 <= 13'd0; o_last1 <= 1'b0; err_k <= 1'b0;
            out_valid <= 1'b0; out_bit <= 1'b0; out_idx <= 13'd0; out_last <= 1'b0;
        end else begin
            err_k     <= 1'b0;                     // mặc định: xung lỗi chỉ kéo dài 1 nhịp
            rd_v      <= (state == D_PREAD);       // dữ liệu bộ nhớ hợp lệ ở nhịp sau khi phát địa chỉ
            rd_tag    <= ph ? qpp_addr : rd_cnt;   // nhãn = địa chỉ gốc của phần tử (Pi(j) hoặc j)
            o_v1      <= (state == D_OUT);         // OUT: dữ liệu hợp lệ ở nhịp sau
            o_idx1    <= rd_cnt;                   // OUT: chỉ số bit tương ứng
            o_last1   <= (rd_cnt == k_last);       // OUT: có phải bit cuối
            out_valid <= o_v1;                     // phát bit ra ngoài (trễ thêm 1 nhịp để có post)
            out_bit   <= (post < 0);               // LLR âm => bit 1
            out_idx   <= o_idx1;                   // chỉ số bit
            out_last  <= o_v1 && o_last1;          // bit cuối của khối
            case (state)
                D_IDLE: begin                      // ---- chờ bắt đầu ----
                    if (start) begin
                        k_reg <= k_in;                         // chốt K
                        iters <= (iter_in == 4'd0) ? 4'd1 : iter_in; // ít nhất 1 vòng lặp
                        state <= D_CHECK;
                    end
                end
                D_CHECK: begin                     // ---- kiểm tra K ----
                    if (k_valid) begin
                        load_cnt <= 13'd0; iter_cnt <= 4'd0; ph <= 1'b0;
                        state <= D_LOAD;
                    end else begin
                        err_k <= 1'b1;             // báo lỗi 1 nhịp
                        state <= D_IDLE;
                    end
                end
                D_LOAD: begin                      // ---- nhận K+4 bộ ba LLR ----
                    if (in_valid) begin
                        if (load_cnt >= k_reg) begin           // bộ ba đuôi: lưu vào thanh ghi
                            tl0[(load_cnt - k_reg)*LW +: LW] <= in_l0;
                            tl1[(load_cnt - k_reg)*LW +: LW] <= in_l1;
                            tl2[(load_cnt - k_reg)*LW +: LW] <= in_l2;
                        end
                        load_cnt <= load_cnt + 13'd1;
                        if (load_cnt == k_reg + 13'd3) state <= D_PSTART; // đã nhận đủ K+4 bộ ba
                    end
                end
                D_PSTART: begin                    // ---- khởi động một nửa vòng lặp ----
                    rd_cnt <= 13'd0;               // bắt đầu phát địa chỉ từ chỉ số 0
                    state  <= D_PREAD;
                end
                D_PREAD: begin                     // ---- phát K địa chỉ đọc ----
                    if (rd_cnt == k_last) state <= D_PWAIT; // địa chỉ cuối
                    else                  rd_cnt <= rd_cnt + 13'd1;
                end
                D_PWAIT: begin                     // ---- chờ SISO xong ----
                    if (s_last) begin
                        if (!ph) begin             // vừa xong SISO1 => chạy SISO2
                            ph <= 1'b1;
                            state <= D_PSTART;
                        end else begin             // vừa xong SISO2 => hết một vòng lặp
                            iter_cnt <= iter_cnt + 4'd1;
                            ph <= 1'b0;
                            if (iter_cnt + 4'd1 == iters) begin  // đủ số vòng lặp => xuất kết quả
                                rd_cnt <= 13'd0;
                                state  <= D_OUT;
                            end else state <= D_PSTART;
                        end
                    end
                end
                D_OUT: begin                       // ---- phát địa chỉ đọc cho K bit quyết định ----
                    if (rd_cnt == k_last) state <= D_FLUSH;
                    else                  rd_cnt <= rd_cnt + 13'd1;
                end
                D_FLUSH: begin                     // ---- chờ bit cuối ra khỏi đường ống ----
                    if (out_last) state <= D_IDLE;
                end
                default: state <= D_IDLE;          // trạng thái không xác định: về IDLE
            endcase
        end
    end

endmodule
