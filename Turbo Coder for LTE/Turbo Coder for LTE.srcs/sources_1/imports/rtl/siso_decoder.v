`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : siso_decoder.v
// Chức năng : Bộ giải mã thành phần SISO (Soft-In Soft-Out) thuật toán Max-Log-MAP
//             cho trellis 8 trạng thái của Turbo code LTE (g0 = 1+D^2+D^3, g1 = 1+D+D^3).
//             Mỗi lần chạy xử lý MỘT nửa vòng lặp (half-iteration) cho một khối K bit.
//
// QUY ƯỚC LLR : LLR dương => bit 0 có khả năng hơn ; LLR âm => bit 1 có khả năng hơn.
//
// THUẬT TOÁN (mỗi bước k):
//     lsa(k)   = Ls(k) + La(k)                       (hệ thống + tiên nghiệm)
//     gamma(u,z) = -( u*lsa + z*Lp )                 (u: bit thông tin, z: bit parity)
//     alpha(k+1) = max-ACS( alpha(k) + gamma )       (đệ quy tiến, chuẩn hóa theo trạng thái 0)
//     beta(k)    = max-ACS( beta(k+1) + gamma )      (đệ quy lùi,  chuẩn hóa theo trạng thái 0)
//     LLR(k)   = max_{u=0}(alpha+gamma+beta) - max_{u=1}(alpha+gamma+beta)
//     Le(k)    = 0.75 * ( LLR(k) - lsa(k) )          (thông tin ngoại, hệ số co 0.75, bão hòa LW bit)
//
// TRELLIS (trùng với rsc_encoder.v): trạng thái s = {s0,s1,s2}
//     đầu vào u : a = u ^ s1 ^ s2 ; parity z = a ^ s0 ^ s2 ; trạng thái kế = {a, s0, s1}
//
// LUỒNG HOẠT ĐỘNG:
//   1) Xung start (k_len = K) : khởi tạo alpha = beta = [0, -S, ..., -S] (bắt đầu từ trạng thái 0).
//   2) Pha TIẾN  : bên ngoài cấp ĐÚNG K nhịp in_valid liên tiếp, mỗi nhịp (Ls, La, Lp, tag).
//                  Mỗi phần tử tính 3 giá trị gamma, ghi {alpha(k), gamma, tag} vào bộ đệm, rồi cập nhật alpha.
//   3) Pha ĐUÔI  : 3 nhịp chạy đệ quy lùi qua 3 bước bit đuôi (tail) để beta kết thúc ở trạng thái 0.
//   4) Pha LÙI   : đọc bộ đệm từ k = K-1 về 0, cập nhật beta và tính LLR / thông tin ngoại qua
//                  đường ống 3 tầng. Ngõ ra (out_valid, out_le, out_tag) đi theo thứ tự k GIẢM DẦN;
//                  out_tag chính là in_tag đã cấp ở pha tiến (địa chỉ ghi thông tin ngoại).
//                  out_last = 1 ở phần tử cuối cùng (k = 0), cũng là cờ "xong".
//==============================================================================
module siso_decoder #(
    parameter LW    = 7,               // số bit có dấu của LLR vào/ra (kênh và thông tin ngoại)
    parameter SW    = 10,              // số bit có dấu của các metric trạng thái alpha/beta
    parameter AW    = 13,              // số bit chỉ số / tag (2^13 = 8192 >= 6144)
    parameter DEPTH = 6144             // độ sâu bộ đệm = K lớn nhất
)(
    input  wire                 clk,       // xung nhịp hệ thống
    input  wire                 rst,       // reset đồng bộ, mức cao
    input  wire                 start,     // xung 1 nhịp bắt đầu một nửa vòng lặp (chỉ nhận khi rảnh)
    input  wire [AW-1:0]        k_len,     // kích thước khối K (40..6144), chốt khi start
    // ---- luồng vào (pha tiến): đúng K nhịp in_valid ----
    input  wire                 in_valid,  // =1 khi (in_ls, in_la, in_lp, in_tag) hợp lệ
    input  wire signed [LW-1:0] in_ls,     // LLR hệ thống Ls(k)
    input  wire signed [LW-1:0] in_la,     // LLR tiên nghiệm La(k) (thông tin ngoại của bộ kia)
    input  wire signed [LW-1:0] in_lp,     // LLR parity Lp(k)
    input  wire [AW-1:0]        in_tag,    // nhãn đi kèm (địa chỉ để ghi lại thông tin ngoại)
    // ---- LLR của 3 bước đuôi: x(K),x(K+1),x(K+2) và z(K),z(K+1),z(K+2), mỗi giá trị LW bit ----
    input  wire [3*LW-1:0]      tail_x,    // {x(K+2), x(K+1), x(K)}  : LLR bit hệ thống ở đuôi
    input  wire [3*LW-1:0]      tail_z,    // {z(K+2), z(K+1), z(K)}  : LLR bit parity ở đuôi
    // ---- luồng ra (pha lùi): thứ tự k giảm dần ----
    output reg                  out_valid, // =1 khi (out_le, out_tag) hợp lệ
    output reg signed [LW-1:0]  out_le,    // thông tin ngoại Le(k)
    output reg [AW-1:0]         out_tag,   // nhãn tương ứng (lấy từ in_tag)
    output reg                  out_last   // =1 ở phần tử cuối (k = 0): xong một nửa vòng lặp
);

    // ---------------- hằng số dẫn xuất từ tham số ----------------
    localparam GW   = LW + 2;                      // số bit của một giá trị gamma (có dấu)
    localparam MW   = SW + 3;                      // số bit của tổng alpha + gamma + beta (có dấu)
    localparam WORD = 8*SW + 3*GW + AW;            // số bit một từ trong bộ đệm: alpha(8) + 3 gamma + tag
    localparam signed [SW-1:0] SMAX =  (1 << (SW-1)) - 1;     // metric lớn nhất khi bão hòa (+511 với SW=10)
    localparam signed [SW-1:0] SMIN = -((1 << (SW-1)) - 1);   // metric nhỏ nhất khi bão hòa (-511 với SW=10)
    localparam signed [LW-1:0] LMAX =  (1 << (LW-1)) - 1;     // LLR ngoại lớn nhất (+63 với LW=7)
    localparam signed [LW-1:0] LMIN = -((1 << (LW-1)) - 1);   // LLR ngoại nhỏ nhất (-63 với LW=7)

    localparam [1:0] S_IDLE = 2'd0;                // rảnh, chờ start
    localparam [1:0] S_FWD  = 2'd1;                // pha tiến: nhận K phần tử, tính alpha
    localparam [1:0] S_TAIL = 2'd2;                // pha đuôi: 3 nhịp khởi tạo beta
    localparam [1:0] S_BWD  = 2'd3;                // pha lùi: tính beta và LLR

    // ---------------- hàm tiện ích ----------------
    // Khởi tạo metric: trạng thái 0 = 0, bảy trạng thái còn lại = -2^(SW-2) (coi như "rất không có khả năng").
    function [8*SW-1:0] init_metrics;
        input dummy;                               // tham số giả (Verilog-2001 yêu cầu hàm có ít nhất 1 đầu vào)
        integer i;                                 // chỉ số trạng thái
        begin
            for (i = 0; i < 8; i = i + 1)          // duyệt 8 trạng thái
                init_metrics[i*SW +: SW] = (i == 0) ? {SW{1'b0}} : (-(1 << (SW-2))); // 0 hoặc -256
        end
    endfunction

    // Bão hòa một số có dấu SW+3 bit về khoảng [SMIN, SMAX] của metric SW bit.
    function signed [SW-1:0] sat_sw;
        input signed [SW+2:0] v;                   // giá trị cần bão hòa
        begin
            if (v > SMAX)      sat_sw = SMAX;      // quá lớn => cắt về SMAX
            else if (v < SMIN) sat_sw = SMIN;      // quá nhỏ => cắt về SMIN
            else               sat_sw = v[SW-1:0]; // trong khoảng => giữ nguyên
        end
    endfunction

    // Bão hòa thông tin ngoại về LW bit đối xứng [LMIN, LMAX].
    function signed [LW-1:0] sat_lw;
        input signed [MW+1:0] v;                   // thông tin ngoại đã nhân 0.75, độ rộng MW+2
        begin
            if (v > LMAX)      sat_lw = LMAX;      // quá lớn => cắt
            else if (v < LMIN) sat_lw = LMIN;      // quá nhỏ => cắt
            else               sat_lw = v[LW-1:0]; // giữ nguyên
        end
    endfunction

    // ---------------- thanh ghi điều khiển ----------------
    reg [1:0]    state;                            // trạng thái FSM
    reg [AW-1:0] k_last;                           // K-1: chỉ số phần tử cuối cùng
    reg [AW-1:0] wr_cnt;                           // chỉ số phần tử đang ghi vào bộ đệm (pha tiến)
    reg [AW-1:0] rd_addr;                          // địa chỉ đọc bộ đệm (pha lùi, giảm dần)
    reg [AW-1:0] bcnt;                             // chỉ số phần tử đang xử lý ở pha lùi (K-1 .. 0)
    reg [1:0]    tcnt;                             // đếm 3 nhịp pha đuôi: 0,1,2
    reg [8*SW-1:0] alpha;                          // 8 metric tiến alpha(k), trạng thái 0 ở bit thấp nhất
    reg [8*SW-1:0] beta;                           // 8 metric lùi beta(k+1)

    // ---------------- tầng 1 của pha tiến: tính gamma ----------------
    reg                  f_v;                      // =1: tầng 1 đang giữ một phần tử hợp lệ
    reg signed [GW-1:0]  f_g01;                    // gamma(u=0, z=1) = -Lp
    reg signed [GW-1:0]  f_g10;                    // gamma(u=1, z=0) = -lsa
    reg signed [GW-1:0]  f_g11;                    // gamma(u=1, z=1) = -lsa - Lp
    reg [AW-1:0]         f_tag;                    // nhãn của phần tử đó

    initial begin                                  // giá trị khởi tạo để dạng sóng không có X
        state = S_IDLE; k_last = {AW{1'b0}}; wr_cnt = {AW{1'b0}};          // FSM và bộ đếm
        rd_addr = {AW{1'b0}}; bcnt = {AW{1'b0}}; tcnt = 2'd0;              // bộ đếm pha lùi
        alpha = init_metrics(1'b0); beta = init_metrics(1'b0);             // metric ban đầu
        f_v = 1'b0; f_g01 = 0; f_g10 = 0; f_g11 = 0; f_tag = {AW{1'b0}};  // tầng 1 trống
        out_valid = 1'b0; out_le = 0; out_tag = {AW{1'b0}}; out_last = 1'b0; // ngõ ra
    end

    // Tổ hợp tầng 1: tính lsa và ba gamma từ ngõ vào hiện tại.
    wire signed [LW:0]    lsa_w  = in_ls + in_la;  // lsa = Ls + La, cần LW+1 bit để không tràn
    wire signed [GW-1:0]  lsa_x  = lsa_w;          // mở rộng dấu lsa lên GW bit
    wire signed [GW-1:0]  lp_x   = in_lp;          // mở rộng dấu Lp lên GW bit
    wire signed [GW-1:0]  g01_w  = -lp_x;          // gamma(0,1) = -Lp
    wire signed [GW-1:0]  g10_w  = -lsa_x;         // gamma(1,0) = -lsa
    wire signed [GW-1:0]  g11_w  = -lsa_x - lp_x;  // gamma(1,1) = -lsa - Lp

    // Bus gamma cho pha tiến, chỉ số = 2*u + z : [0]=gamma(0,0)=0, [1]=gamma(0,1), [2]=gamma(1,0), [3]=gamma(1,1).
    wire [4*GW-1:0] fwd_g = {f_g11, f_g10, f_g01, {GW{1'b0}}};

    // ---------------- bộ đệm alpha/gamma/tag ----------------
    wire [WORD-1:0] rd_word;                       // từ đọc ra từ bộ đệm (trễ 1 nhịp)
    wire            buf_we = f_v && (state == S_FWD); // ghi bộ đệm khi tầng 1 có phần tử hợp lệ ở pha tiến
    sdp_ram #(.WIDTH(WORD), .DEPTH(DEPTH), .AW(AW)) u_buf (
        .clk  (clk),                               // xung nhịp
        .we   (buf_we),                            // cho phép ghi
        .waddr(wr_cnt),                            // ghi tại chỉ số k
        .wdata({f_tag, f_g11, f_g10, f_g01, alpha}), // alpha(k) TRƯỚC khi cập nhật, 3 gamma, tag
        .raddr(rd_addr),                           // đọc theo địa chỉ giảm dần
        .rdata(rd_word)                            // dữ liệu đọc
    );
    wire [8*SW-1:0]      rd_alpha = rd_word[8*SW-1:0];                    // alpha(k) đã lưu
    wire signed [GW-1:0] rd_g01   = rd_word[8*SW +: GW];                  // gamma(0,1) đã lưu
    wire signed [GW-1:0] rd_g10   = rd_word[8*SW+GW +: GW];               // gamma(1,0) đã lưu
    wire signed [GW-1:0] rd_g11   = rd_word[8*SW+2*GW +: GW];             // gamma(1,1) đã lưu
    wire [AW-1:0]        rd_tag   = rd_word[8*SW+3*GW +: AW];             // tag đã lưu

    // ---------------- ACS tiến: 8 trạng thái kế tiếp ----------------
    // Mỗi trạng thái kế n = {a,s0,s1} có đúng 2 tiền nhiệm p = {n[1],n[0],s2}, s2 = 0 hoặc 1,
    // với u = n[2]^n[0]^s2 và z = n[2]^n[1]^s2.
    wire [8*(SW+1)-1:0] fwd_new;                   // tổng lớn nhất cho mỗi trạng thái kế (chưa chuẩn hóa)
    wire [8*SW-1:0]     alpha_nxt;                 // alpha(k+1) sau chuẩn hóa và bão hòa
    genvar n;                                      // chỉ số trạng thái kế
    generate
        for (n = 0; n < 8; n = n + 1) begin : g_fwd
            localparam integer P0 = (n % 4) * 2;              // tiền nhiệm với s2 = 0
            localparam integer P1 = (n % 4) * 2 + 1;          // tiền nhiệm với s2 = 1
            localparam integer U0 = ((n/4) + (n%2)) % 2;      // bit vào u của nhánh từ P0
            localparam integer U1 = ((n/4) + (n%2) + 1) % 2;  // bit vào u của nhánh từ P1
            localparam integer Z0 = ((n/4) + ((n/2)%2)) % 2;  // bit parity z của nhánh từ P0
            localparam integer Z1 = ((n/4) + ((n/2)%2) + 1) % 2; // bit parity z của nhánh từ P1
            wire signed [SW:0] c0 = $signed(alpha[P0*SW +: SW]) + $signed(fwd_g[(2*U0+Z0)*GW +: GW]); // alpha(P0)+gamma
            wire signed [SW:0] c1 = $signed(alpha[P1*SW +: SW]) + $signed(fwd_g[(2*U1+Z1)*GW +: GW]); // alpha(P1)+gamma
            assign fwd_new[n*(SW+1) +: (SW+1)] = (c1 > c0) ? c1 : c0;  // chọn nhánh lớn hơn (Max-Log)
            // chuẩn hóa: trừ giá trị của trạng thái 0 rồi bão hòa về SW bit
            wire signed [SW+1:0] d = $signed(fwd_new[n*(SW+1) +: (SW+1)]) - $signed(fwd_new[0 +: (SW+1)]);
            assign alpha_nxt[n*SW +: SW] = sat_sw(d);
        end
    endgenerate

    // ---------------- gamma cho đệ quy lùi: pha đuôi lấy từ LLR đuôi, pha lùi lấy từ bộ đệm ----------------
    wire [1:0]            t_idx = 2'd2 - tcnt;                 // bước đuôi đang xử lý: 2,1,0 (đi lùi)
    wire signed [LW-1:0]  t_x   = tail_x[t_idx*LW +: LW];      // LLR hệ thống của bước đuôi đó
    wire signed [LW-1:0]  t_z   = tail_z[t_idx*LW +: LW];      // LLR parity của bước đuôi đó
    wire signed [GW-1:0]  t_xx  = t_x;                         // mở rộng dấu lên GW bit
    wire signed [GW-1:0]  t_zz  = t_z;                         // mở rộng dấu lên GW bit
    wire signed [GW-1:0]  t_g01 = -t_zz;                       // gamma(0,1) đuôi = -z
    wire signed [GW-1:0]  t_g10 = -t_xx;                       // gamma(1,0) đuôi = -x
    wire signed [GW-1:0]  t_g11 = -t_xx - t_zz;                // gamma(1,1) đuôi = -x - z
    wire [4*GW-1:0] bwd_g = (state == S_TAIL) ? {t_g11, t_g10, t_g01, {GW{1'b0}}}      // pha đuôi
                                              : {rd_g11, rd_g10, rd_g01, {GW{1'b0}}};  // pha lùi

    // ---------------- ACS lùi + tổng nhánh cho LLR ----------------
    // Từ trạng thái s với bit vào u: a = u^s1^s2, z = a^s0^s2, trạng thái kế = {a,s0,s1}.
    wire [8*(SW+2)-1:0] bwd_new;                   // beta(k) chưa chuẩn hóa cho mỗi trạng thái
    wire [8*SW-1:0]     beta_nxt;                  // beta(k) sau chuẩn hóa và bão hòa
    wire [8*MW-1:0]     m0_bus;                    // 8 tổng alpha+gamma+beta của nhánh u=0 (đã đăng ký tầng A)
    wire [8*MW-1:0]     m1_bus;                    // 8 tổng của nhánh u=1 (đã đăng ký tầng A)
    genvar s;                                      // chỉ số trạng thái hiện tại
    generate
        for (s = 0; s < 8; s = s + 1) begin : g_bwd
            localparam integer S0 = (s/4) % 2;                 // bit s0 của trạng thái s
            localparam integer S1 = (s/2) % 2;                 // bit s1
            localparam integer S2 = s % 2;                     // bit s2
            localparam integer A0 = (S1 + S2) % 2;             // a khi u = 0
            localparam integer A1 = (1 + S1 + S2) % 2;         // a khi u = 1
            localparam integer N0 = A0*4 + S0*2 + S1;          // trạng thái kế khi u = 0
            localparam integer N1 = A1*4 + S0*2 + S1;          // trạng thái kế khi u = 1
            localparam integer Z0 = (A0 + S0 + S2) % 2;        // parity khi u = 0
            localparam integer Z1 = (A1 + S0 + S2) % 2;        // parity khi u = 1
            wire signed [SW+1:0] mb0 = $signed(bwd_g[(0*2+Z0)*GW +: GW]) + $signed(beta[N0*SW +: SW]); // gamma+beta(u=0)
            wire signed [SW+1:0] mb1 = $signed(bwd_g[(1*2+Z1)*GW +: GW]) + $signed(beta[N1*SW +: SW]); // gamma+beta(u=1)
            assign bwd_new[s*(SW+2) +: (SW+2)] = (mb1 > mb0) ? mb1 : mb0;   // beta(k)(s) = max theo u
            wire signed [SW+2:0] bd = $signed(bwd_new[s*(SW+2) +: (SW+2)]) - $signed(bwd_new[0 +: (SW+2)]); // trừ trạng thái 0
            assign beta_nxt[s*SW +: SW] = sat_sw(bd);              // bão hòa về SW bit
            // tổng nhánh cho LLR: alpha(k)(s) + gamma + beta(k+1)(kế), đăng ký ở tầng A của đường ống
            reg signed [MW-1:0] m0_q;                              // tổng nhánh u=0
            reg signed [MW-1:0] m1_q;                              // tổng nhánh u=1
            initial begin m0_q = 0; m1_q = 0; end                  // khởi tạo để không có X
            always @(posedge clk) begin
                m0_q <= $signed(rd_alpha[s*SW +: SW]) + mb0;       // alpha + gamma + beta (u = 0)
                m1_q <= $signed(rd_alpha[s*SW +: SW]) + mb1;       // alpha + gamma + beta (u = 1)
            end
            assign m0_bus[s*MW +: MW] = m0_q;                      // đưa ra bus chung
            assign m1_bus[s*MW +: MW] = m1_q;                      // đưa ra bus chung
        end
    endgenerate

    // ---------------- đường ống LLR: tầng A -> B -> ngõ ra ----------------
    reg                 va, vb;                    // cờ hợp lệ tầng A và tầng B
    reg                 last_a, last_b;            // cờ "phần tử cuối" tầng A và B
    reg [AW-1:0]        tag_a, tag_b;              // nhãn
    reg signed [GW-1:0] g10_a, g10_b;              // gamma(1,0) = -lsa, dùng để trừ tiên nghiệm + hệ thống
    reg signed [MW:0]   llr_b;                     // LLR toàn phần tầng B = max0 - max1
    initial begin                                  // khởi tạo
        va = 0; vb = 0; last_a = 0; last_b = 0; tag_a = 0; tag_b = 0; g10_a = 0; g10_b = 0; llr_b = 0;
    end

    // Tìm max của 8 tổng cho từng nhánh u (tổ hợp, nằm giữa tầng A và tầng B).
    reg signed [MW-1:0] max0, max1;                // giá trị lớn nhất của u=0 và u=1
    integer i;                                     // chỉ số vòng lặp
    always @(*) begin
        max0 = $signed(m0_bus[0 +: MW]);           // bắt đầu bằng trạng thái 0
        max1 = $signed(m1_bus[0 +: MW]);           // bắt đầu bằng trạng thái 0
        for (i = 1; i < 8; i = i + 1) begin        // so với 7 trạng thái còn lại
            if ($signed(m0_bus[i*MW +: MW]) > max0) max0 = $signed(m0_bus[i*MW +: MW]); // cập nhật max u=0
            if ($signed(m1_bus[i*MW +: MW]) > max1) max1 = $signed(m1_bus[i*MW +: MW]); // cập nhật max u=1
        end
    end

    // Tầng C (tổ hợp): Le = 0.75 * (LLR - lsa) ;  LLR - lsa = LLR + gamma(1,0) vì gamma(1,0) = -lsa.
    wire signed [MW+1:0] e_raw = llr_b + g10_b;                // LLR - lsa
    wire signed [MW+1:0] e_scl = e_raw - (e_raw >>> 2);        // nhân 0.75 = x - x/4 (dịch số học)

    // ---------------- FSM chính ----------------
    always @(posedge clk) begin
        if (rst) begin                             // reset đồng bộ
            state <= S_IDLE;                       // về rảnh
            f_v <= 1'b0;                           // xóa tầng 1
            wr_cnt <= {AW{1'b0}};                  // xóa bộ đếm ghi
            rd_addr <= {AW{1'b0}};                 // xóa địa chỉ đọc
            bcnt <= {AW{1'b0}};                    // xóa bộ đếm pha lùi
            tcnt <= 2'd0;                          // xóa bộ đếm đuôi
            alpha <= init_metrics(1'b0);           // alpha ban đầu
            beta  <= init_metrics(1'b0);           // beta ban đầu
        end else begin
            // tầng 1 của pha tiến: chốt gamma khi có phần tử vào (chỉ khi đang ở pha tiến)
            f_v <= in_valid && (state == S_FWD);   // phần tử hợp lệ nếu đang nhận
            if (in_valid && (state == S_FWD)) begin
                f_g01 <= g01_w;                    // chốt gamma(0,1)
                f_g10 <= g10_w;                    // chốt gamma(1,0)
                f_g11 <= g11_w;                    // chốt gamma(1,1)
                f_tag <= in_tag;                   // chốt nhãn
            end
            case (state)
                S_IDLE: begin                      // ---- rảnh ----
                    if (start) begin               // bắt đầu nửa vòng lặp mới
                        k_last <= k_len - 1'b1;    // chốt chỉ số cuối K-1
                        wr_cnt <= {AW{1'b0}};      // ghi từ chỉ số 0
                        alpha  <= init_metrics(1'b0); // alpha bắt đầu từ trạng thái 0
                        beta   <= init_metrics(1'b0); // beta kết thúc ở trạng thái 0
                        state  <= S_FWD;           // sang pha tiến
                    end
                end
                S_FWD: begin                       // ---- pha tiến ----
                    if (f_v) begin                 // có phần tử ở tầng 1: ghi bộ đệm (buf_we) và cập nhật alpha
                        alpha  <= alpha_nxt;       // alpha(k+1)
                        wr_cnt <= wr_cnt + 1'b1;   // sang chỉ số kế
                        if (wr_cnt == k_last) begin // vừa xử lý phần tử cuối
                            rd_addr <= k_last;     // chuẩn bị đọc phần tử K-1 đầu tiên
                            tcnt    <= 2'd0;       // bắt đầu đếm 3 nhịp đuôi
                            state   <= S_TAIL;     // sang pha đuôi
                        end
                    end
                end
                S_TAIL: begin                      // ---- pha đuôi: 3 nhịp ----
                    beta <= beta_nxt;              // cập nhật beta qua bước đuôi t_idx
                    tcnt <= tcnt + 2'd1;           // nhịp kế
                    if (tcnt == 2'd2) begin        // xong 3 nhịp
                        bcnt <= k_last;            // phần tử đầu tiên của pha lùi là K-1
                        if (rd_addr != 0) rd_addr <= rd_addr - 1'b1; // chuẩn bị đọc phần tử K-2
                        state <= S_BWD;            // sang pha lùi
                    end
                end
                S_BWD: begin                       // ---- pha lùi ----
                    beta <= beta_nxt;              // beta(k) từ beta(k+1)
                    if (rd_addr != 0) rd_addr <= rd_addr - 1'b1; // địa chỉ đọc giảm dần (dừng ở 0)
                    if (bcnt == 0) state <= S_IDLE;// xử lý xong phần tử k = 0
                    else           bcnt  <= bcnt - 1'b1; // sang phần tử kế
                end
                default: state <= S_IDLE;          // trạng thái không xác định: về rảnh
            endcase
        end
    end

    // ---------------- đường ống LLR (không cần reset dữ liệu, chỉ cần reset cờ hợp lệ) ----------------
    always @(posedge clk) begin
        if (rst) begin                             // reset các cờ
            va <= 1'b0; vb <= 1'b0; last_a <= 1'b0; last_b <= 1'b0;
            out_valid <= 1'b0; out_last <= 1'b0;
        end else begin
            // tầng A: chốt cờ và dữ liệu của phần tử đang ở pha lùi (m0_q/m1_q đã được chốt trong g_bwd)
            va     <= (state == S_BWD);                    // phần tử hợp lệ ở pha lùi
            last_a <= (state == S_BWD) && (bcnt == 0);     // phần tử cuối (k = 0)
            // tầng B: LLR toàn phần = max(u=0) - max(u=1)
            vb     <= va;                                  // chuyển cờ hợp lệ
            last_b <= last_a;                              // chuyển cờ cuối
            // ngõ ra: thông tin ngoại đã co và bão hòa
            out_valid <= vb;                               // chuyển cờ hợp lệ ra ngoài
            out_last  <= last_b;                           // chuyển cờ cuối ra ngoài
        end
        tag_a <= rd_tag;                           // nhãn của phần tử đọc từ bộ đệm
        g10_a <= rd_g10;                           // gamma(1,0) = -lsa của phần tử đó
        tag_b <= tag_a;                            // trượt nhãn sang tầng B
        g10_b <= g10_a;                            // trượt gamma(1,0) sang tầng B
        llr_b <= max0 - max1;                      // LLR toàn phần (tầng B)
        out_le  <= sat_lw(e_scl);                  // thông tin ngoại ra (bão hòa LW bit)
        out_tag <= tag_b;                          // nhãn ra
    end

endmodule
