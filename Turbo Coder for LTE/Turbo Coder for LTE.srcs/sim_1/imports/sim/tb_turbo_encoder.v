`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : tb_turbo_encoder.v
// Chức năng : Testbench tự kiểm tra cho turbo_encoder (Turbo code LTE, tốc độ 1/3).
//
// Cách kiểm tra:
//   * Mô hình tham chiếu viết ngay trong testbench theo dạng PHƯƠNG TRÌNH
//     (a(n) = x(n)^a(n-2)^a(n-3) ; z(n) = a(n)^a(n-1)^a(n-3) ; Pi(i) tính trực tiếp
//     bằng số học 64 bit) - khác hẳn cấu trúc thanh ghi dịch + công thức đệ quy của RTL.
//   * Hai vector cố định K = 40 và K = 48 được tính bằng một mô hình Python độc lập viết theo
//     TS 36.212. Bảng QPP (188 kích thước K) và cách ghép 12 bit đuôi đã được đối chiếu với mã nguồn
//     mở srsRAN và với nguyên văn mục 5.1.3.2.2 của chuẩn.
//   * Các ca thử: nhiều kích thước K (nhỏ nhất 40, lớn nhất 6144), toàn 0, toàn 1,
//     bên phát bị nghẽn (in_valid ngắt quãng), start giả khi đang bận, K sai,
//     reset giữa chừng (ở LOAD, ENC, TAIL), các khối nối đuôi nhau, và (FULL_SWEEP = 1)
//     quét TOÀN BỘ 188 kích thước K của LTE với dữ liệu ngẫu nhiên.
//   * Mỗi nhịp đều kiểm tra ngõ ra không có X/Z.
// Kết quả cuối: in "TEST PASSED" hoặc "TEST FAILED" rồi gọi $finish.
//==============================================================================
module tb_turbo_encoder #(
    parameter FULL_SWEEP = 1           // 1: chạy thêm bước quét toàn bộ 188 kích thước K (mất vài giây); 0: bỏ qua
);

    // ------------------------- tín hiệu nối với DUT -------------------------
    reg         clk;                   // xung nhịp 100 MHz
    reg         rst;                   // reset đồng bộ, mức cao
    reg         start;                 // xung bắt đầu khối
    reg  [12:0] k_in;                  // kích thước khối K
    reg         in_valid;              // bit vào hợp lệ
    reg         din;                   // bit vào
    wire        in_ready;              // DUT sẵn sàng nhận bit
    wire        out_valid;             // ngõ ra hợp lệ
    wire        d0;                    // luồng hệ thống
    wire        d1;                    // luồng parity 1
    wire        d2;                    // luồng parity 2
    wire        out_tail;              // đang phát bit đuôi
    wire        out_last;              // nhịp cuối của khối
    wire        busy;                  // DUT đang bận
    wire        err_k;                 // báo K sai
    wire        term_ok;               // kết thúc trellis thành công

    turbo_encoder dut (                // khối cần kiểm tra
        .clk(clk), .rst(rst), .start(start), .k_in(k_in),
        .in_ready(in_ready), .in_valid(in_valid), .din(din),
        .out_valid(out_valid), .d0(d0), .d1(d1), .d2(d2),
        .out_tail(out_tail), .out_last(out_last),
        .busy(busy), .err_k(err_k), .term_ok(term_ok)
    );

    initial clk = 1'b0;                // xung nhịp bắt đầu ở mức thấp
    always #5 clk = ~clk;              // chu kỳ 10 ns => 100 MHz

        // Bảng tham số QPP của RIÊNG testbench (không dùng ROM của DUT) - trả về {f1, f2} theo K.
    // Nguồn: 3GPP TS 36.212 Bảng 5.1.3-3; đã kiểm tra mọi dòng là hoán vị hợp lệ.
    function [25:0] tb_qpp_params;     // kết quả 26 bit = {f1[12:0], f2[12:0]}
        input integer kq;              // kích thước khối cần tra
        begin
            case (kq)                  // tra bảng
                40   : tb_qpp_params = {13'd3, 13'd10};
                48   : tb_qpp_params = {13'd7, 13'd12};
                56   : tb_qpp_params = {13'd19, 13'd42};
                64   : tb_qpp_params = {13'd7, 13'd16};
                72   : tb_qpp_params = {13'd7, 13'd18};
                80   : tb_qpp_params = {13'd11, 13'd20};
                88   : tb_qpp_params = {13'd5, 13'd22};
                96   : tb_qpp_params = {13'd11, 13'd24};
                104  : tb_qpp_params = {13'd7, 13'd26};
                112  : tb_qpp_params = {13'd41, 13'd84};
                120  : tb_qpp_params = {13'd103, 13'd90};
                128  : tb_qpp_params = {13'd15, 13'd32};
                136  : tb_qpp_params = {13'd9, 13'd34};
                144  : tb_qpp_params = {13'd17, 13'd108};
                152  : tb_qpp_params = {13'd9, 13'd38};
                160  : tb_qpp_params = {13'd21, 13'd120};
                168  : tb_qpp_params = {13'd101, 13'd84};
                176  : tb_qpp_params = {13'd21, 13'd44};
                184  : tb_qpp_params = {13'd57, 13'd46};
                192  : tb_qpp_params = {13'd23, 13'd48};
                200  : tb_qpp_params = {13'd13, 13'd50};
                208  : tb_qpp_params = {13'd27, 13'd52};
                216  : tb_qpp_params = {13'd11, 13'd36};
                224  : tb_qpp_params = {13'd27, 13'd56};
                232  : tb_qpp_params = {13'd85, 13'd58};
                240  : tb_qpp_params = {13'd29, 13'd60};
                248  : tb_qpp_params = {13'd33, 13'd62};
                256  : tb_qpp_params = {13'd15, 13'd32};
                264  : tb_qpp_params = {13'd17, 13'd198};
                272  : tb_qpp_params = {13'd33, 13'd68};
                280  : tb_qpp_params = {13'd103, 13'd210};
                288  : tb_qpp_params = {13'd19, 13'd36};
                296  : tb_qpp_params = {13'd19, 13'd74};
                304  : tb_qpp_params = {13'd37, 13'd76};
                312  : tb_qpp_params = {13'd19, 13'd78};
                320  : tb_qpp_params = {13'd21, 13'd120};
                328  : tb_qpp_params = {13'd21, 13'd82};
                336  : tb_qpp_params = {13'd115, 13'd84};
                344  : tb_qpp_params = {13'd193, 13'd86};
                352  : tb_qpp_params = {13'd21, 13'd44};
                360  : tb_qpp_params = {13'd133, 13'd90};
                368  : tb_qpp_params = {13'd81, 13'd46};
                376  : tb_qpp_params = {13'd45, 13'd94};
                384  : tb_qpp_params = {13'd23, 13'd48};
                392  : tb_qpp_params = {13'd243, 13'd98};
                400  : tb_qpp_params = {13'd151, 13'd40};
                408  : tb_qpp_params = {13'd155, 13'd102};
                416  : tb_qpp_params = {13'd25, 13'd52};
                424  : tb_qpp_params = {13'd51, 13'd106};
                432  : tb_qpp_params = {13'd47, 13'd72};
                440  : tb_qpp_params = {13'd91, 13'd110};
                448  : tb_qpp_params = {13'd29, 13'd168};
                456  : tb_qpp_params = {13'd29, 13'd114};
                464  : tb_qpp_params = {13'd247, 13'd58};
                472  : tb_qpp_params = {13'd29, 13'd118};
                480  : tb_qpp_params = {13'd89, 13'd180};
                488  : tb_qpp_params = {13'd91, 13'd122};
                496  : tb_qpp_params = {13'd157, 13'd62};
                504  : tb_qpp_params = {13'd55, 13'd84};
                512  : tb_qpp_params = {13'd31, 13'd64};
                528  : tb_qpp_params = {13'd17, 13'd66};
                544  : tb_qpp_params = {13'd35, 13'd68};
                560  : tb_qpp_params = {13'd227, 13'd420};
                576  : tb_qpp_params = {13'd65, 13'd96};
                592  : tb_qpp_params = {13'd19, 13'd74};
                608  : tb_qpp_params = {13'd37, 13'd76};
                624  : tb_qpp_params = {13'd41, 13'd234};
                640  : tb_qpp_params = {13'd39, 13'd80};
                656  : tb_qpp_params = {13'd185, 13'd82};
                672  : tb_qpp_params = {13'd43, 13'd252};
                688  : tb_qpp_params = {13'd21, 13'd86};
                704  : tb_qpp_params = {13'd155, 13'd44};
                720  : tb_qpp_params = {13'd79, 13'd120};
                736  : tb_qpp_params = {13'd139, 13'd92};
                752  : tb_qpp_params = {13'd23, 13'd94};
                768  : tb_qpp_params = {13'd217, 13'd48};
                784  : tb_qpp_params = {13'd25, 13'd98};
                800  : tb_qpp_params = {13'd17, 13'd80};
                816  : tb_qpp_params = {13'd127, 13'd102};
                832  : tb_qpp_params = {13'd25, 13'd52};
                848  : tb_qpp_params = {13'd239, 13'd106};
                864  : tb_qpp_params = {13'd17, 13'd48};
                880  : tb_qpp_params = {13'd137, 13'd110};
                896  : tb_qpp_params = {13'd215, 13'd112};
                912  : tb_qpp_params = {13'd29, 13'd114};
                928  : tb_qpp_params = {13'd15, 13'd58};
                944  : tb_qpp_params = {13'd147, 13'd118};
                960  : tb_qpp_params = {13'd29, 13'd60};
                976  : tb_qpp_params = {13'd59, 13'd122};
                992  : tb_qpp_params = {13'd65, 13'd124};
                1008 : tb_qpp_params = {13'd55, 13'd84};
                1024 : tb_qpp_params = {13'd31, 13'd64};
                1056 : tb_qpp_params = {13'd17, 13'd66};
                1088 : tb_qpp_params = {13'd171, 13'd204};
                1120 : tb_qpp_params = {13'd67, 13'd140};
                1152 : tb_qpp_params = {13'd35, 13'd72};
                1184 : tb_qpp_params = {13'd19, 13'd74};
                1216 : tb_qpp_params = {13'd39, 13'd76};
                1248 : tb_qpp_params = {13'd19, 13'd78};
                1280 : tb_qpp_params = {13'd199, 13'd240};
                1312 : tb_qpp_params = {13'd21, 13'd82};
                1344 : tb_qpp_params = {13'd211, 13'd252};
                1376 : tb_qpp_params = {13'd21, 13'd86};
                1408 : tb_qpp_params = {13'd43, 13'd88};
                1440 : tb_qpp_params = {13'd149, 13'd60};
                1472 : tb_qpp_params = {13'd45, 13'd92};
                1504 : tb_qpp_params = {13'd49, 13'd846};
                1536 : tb_qpp_params = {13'd71, 13'd48};
                1568 : tb_qpp_params = {13'd13, 13'd28};
                1600 : tb_qpp_params = {13'd17, 13'd80};
                1632 : tb_qpp_params = {13'd25, 13'd102};
                1664 : tb_qpp_params = {13'd183, 13'd104};
                1696 : tb_qpp_params = {13'd55, 13'd954};
                1728 : tb_qpp_params = {13'd127, 13'd96};
                1760 : tb_qpp_params = {13'd27, 13'd110};
                1792 : tb_qpp_params = {13'd29, 13'd112};
                1824 : tb_qpp_params = {13'd29, 13'd114};
                1856 : tb_qpp_params = {13'd57, 13'd116};
                1888 : tb_qpp_params = {13'd45, 13'd354};
                1920 : tb_qpp_params = {13'd31, 13'd120};
                1952 : tb_qpp_params = {13'd59, 13'd610};
                1984 : tb_qpp_params = {13'd185, 13'd124};
                2016 : tb_qpp_params = {13'd113, 13'd420};
                2048 : tb_qpp_params = {13'd31, 13'd64};
                2112 : tb_qpp_params = {13'd17, 13'd66};
                2176 : tb_qpp_params = {13'd171, 13'd136};
                2240 : tb_qpp_params = {13'd209, 13'd420};
                2304 : tb_qpp_params = {13'd253, 13'd216};
                2368 : tb_qpp_params = {13'd367, 13'd444};
                2432 : tb_qpp_params = {13'd265, 13'd456};
                2496 : tb_qpp_params = {13'd181, 13'd468};
                2560 : tb_qpp_params = {13'd39, 13'd80};
                2624 : tb_qpp_params = {13'd27, 13'd164};
                2688 : tb_qpp_params = {13'd127, 13'd504};
                2752 : tb_qpp_params = {13'd143, 13'd172};
                2816 : tb_qpp_params = {13'd43, 13'd88};
                2880 : tb_qpp_params = {13'd29, 13'd300};
                2944 : tb_qpp_params = {13'd45, 13'd92};
                3008 : tb_qpp_params = {13'd157, 13'd188};
                3072 : tb_qpp_params = {13'd47, 13'd96};
                3136 : tb_qpp_params = {13'd13, 13'd28};
                3200 : tb_qpp_params = {13'd111, 13'd240};
                3264 : tb_qpp_params = {13'd443, 13'd204};
                3328 : tb_qpp_params = {13'd51, 13'd104};
                3392 : tb_qpp_params = {13'd51, 13'd212};
                3456 : tb_qpp_params = {13'd451, 13'd192};
                3520 : tb_qpp_params = {13'd257, 13'd220};
                3584 : tb_qpp_params = {13'd57, 13'd336};
                3648 : tb_qpp_params = {13'd313, 13'd228};
                3712 : tb_qpp_params = {13'd271, 13'd232};
                3776 : tb_qpp_params = {13'd179, 13'd236};
                3840 : tb_qpp_params = {13'd331, 13'd120};
                3904 : tb_qpp_params = {13'd363, 13'd244};
                3968 : tb_qpp_params = {13'd375, 13'd248};
                4032 : tb_qpp_params = {13'd127, 13'd168};
                4096 : tb_qpp_params = {13'd31, 13'd64};
                4160 : tb_qpp_params = {13'd33, 13'd130};
                4224 : tb_qpp_params = {13'd43, 13'd264};
                4288 : tb_qpp_params = {13'd33, 13'd134};
                4352 : tb_qpp_params = {13'd477, 13'd408};
                4416 : tb_qpp_params = {13'd35, 13'd138};
                4480 : tb_qpp_params = {13'd233, 13'd280};
                4544 : tb_qpp_params = {13'd357, 13'd142};
                4608 : tb_qpp_params = {13'd337, 13'd480};
                4672 : tb_qpp_params = {13'd37, 13'd146};
                4736 : tb_qpp_params = {13'd71, 13'd444};
                4800 : tb_qpp_params = {13'd71, 13'd120};
                4864 : tb_qpp_params = {13'd37, 13'd152};
                4928 : tb_qpp_params = {13'd39, 13'd462};
                4992 : tb_qpp_params = {13'd127, 13'd234};
                5056 : tb_qpp_params = {13'd39, 13'd158};
                5120 : tb_qpp_params = {13'd39, 13'd80};
                5184 : tb_qpp_params = {13'd31, 13'd96};
                5248 : tb_qpp_params = {13'd113, 13'd902};
                5312 : tb_qpp_params = {13'd41, 13'd166};
                5376 : tb_qpp_params = {13'd251, 13'd336};
                5440 : tb_qpp_params = {13'd43, 13'd170};
                5504 : tb_qpp_params = {13'd21, 13'd86};
                5568 : tb_qpp_params = {13'd43, 13'd174};
                5632 : tb_qpp_params = {13'd45, 13'd176};
                5696 : tb_qpp_params = {13'd45, 13'd178};
                5760 : tb_qpp_params = {13'd161, 13'd120};
                5824 : tb_qpp_params = {13'd89, 13'd182};
                5888 : tb_qpp_params = {13'd323, 13'd184};
                5952 : tb_qpp_params = {13'd47, 13'd186};
                6016 : tb_qpp_params = {13'd23, 13'd94};
                6080 : tb_qpp_params = {13'd47, 13'd190};
                6144 : tb_qpp_params = {13'd263, 13'd480};
                default : tb_qpp_params = 26'd0; // K không hợp lệ
            endcase
        end
    endfunction

    // ------------------------- bộ nhớ của testbench -------------------------
    reg in_bits  [0:6143];             // khối bit đầu vào đang thử
    reg ref0     [0:6147];             // luồng d0 chuẩn  (K + 4 phần tử)
    reg ref1     [0:6147];             // luồng d1 chuẩn
    reg ref2     [0:6147];             // luồng d2 chuẩn
    reg cap0     [0:6147];             // luồng d0 thu được từ DUT
    reg cap1     [0:6147];             // luồng d1 thu được từ DUT
    reg cap2     [0:6147];             // luồng d2 thu được từ DUT
    reg cap_tail [0:6147];             // cờ out_tail thu được kèm mỗi phần tử

    integer cap_cnt;                   // số phần tử đã thu (mỗi nhịp out_valid = 1 thu 1 phần tử)
    reg     got_last;                  // đã thấy out_last của khối hiện tại
    integer err_cnt;                   // số xung err_k đã quan sát
    integer errors;                    // tổng số lỗi so sánh
    integer x_errors;                  // số lần phát hiện X/Z trên ngõ ra
    integer n_tests;                   // số ca thử đã chạy
    integer klist [0:16];              // danh sách K cho phép quét ngẫu nhiên
    integer ti;                        // biến vòng lặp của chuỗi kiểm tra chính
    reg     quiet;                     // 1: chỉ in khi có lỗi (dùng trong bước quét 188 K để log gọn)

    // --------------------- bộ giám sát chạy mỗi nhịp clk ---------------------
    always @(posedge clk) begin
        if ((^{in_ready, out_valid, d0, d1, d2, out_tail, out_last, busy, err_k, term_ok}) === 1'bx) begin
            x_errors = x_errors + 1;   // phát hiện X hoặc Z ở ít nhất một ngõ ra
            if (x_errors <= 5) $display("[ERR] t=%0t : X/Z tren ngo ra DUT", $time);
        end
        if (out_valid === 1'b1) begin  // ngõ ra hợp lệ => thu một phần tử
            if (cap_cnt < 6148) begin  // chống tràn mảng nếu DUT phát thừa
                cap0[cap_cnt]     = d0;        // lưu d0
                cap1[cap_cnt]     = d1;        // lưu d1
                cap2[cap_cnt]     = d2;        // lưu d2
                cap_tail[cap_cnt] = out_tail;  // lưu cờ đuôi
            end
            cap_cnt = cap_cnt + 1;     // tăng số phần tử đã thu
            if (out_last === 1'b1) got_last = 1'b1; // đánh dấu đã nhận nhịp cuối
        end else begin                 // ngõ ra không hợp lệ: các tín hiệu dữ liệu phải bằng 0
            if ((d0 | d1 | d2 | out_tail | out_last) !== 1'b0) begin
                errors = errors + 1;   // dữ liệu/cờ không về 0 khi out_valid = 0
                $display("[ERR] t=%0t : d0/d1/d2/out_tail/out_last khac 0 khi out_valid = 0", $time);
            end
        end
        if (err_k === 1'b1) err_cnt = err_cnt + 1; // đếm xung báo lỗi K
    end

    // ---------------- mô hình tham chiếu (dạng phương trình) ----------------
    task make_reference;               // tính ref0/ref1/ref2 từ in_bits cho khối dài kk
        input integer kk;              // kích thước khối
        reg [25:0] pr;                 // {f1,f2} tra từ bảng riêng của testbench
        reg [63:0] k64, f1_64, f2_64, i64, p64; // số học 64 bit tránh tràn khi tính f2*i*i
        integer    i, t, pidx;         // biến vòng lặp và chỉ số xen
        reg        a1, a2, a3;         // lịch sử a(n-1), a(n-2), a(n-3) của bộ mã 1
        reg        b1, b2, b3;         // lịch sử của bộ mã 2
        reg        xn, an, zn;         // bit vào, nút a và parity ở nhịp hiện tại
        reg [2:0]  ex1, ez1, ex2, ez2; // x,z (bộ 1) và x',z' (bộ 2) của 3 nhịp kết thúc
        begin
            pr    = tb_qpp_params(kk); // tra f1, f2
            k64   = kk;                // K ở dạng 64 bit
            f1_64 = pr[25:13];         // f1
            f2_64 = pr[12:0];          // f2
            a1 = 1'b0; a2 = 1'b0; a3 = 1'b0;  // bộ 1 bắt đầu ở trạng thái 0
            b1 = 1'b0; b2 = 1'b0; b3 = 1'b0;  // bộ 2 bắt đầu ở trạng thái 0
            for (i = 0; i < kk; i = i + 1) begin
                // ---- bộ mã 1: đầu vào theo thứ tự gốc ----
                xn = in_bits[i];               // bit x(i)
                an = xn ^ a2 ^ a3;             // a(n) = x(n) + a(n-2) + a(n-3)   (g0 = 1 + D^2 + D^3)
                zn = an ^ a1 ^ a3;             // z(n) = a(n) + a(n-1) + a(n-3)   (g1 = 1 + D + D^3)
                ref0[i] = xn;                  // luồng hệ thống
                ref1[i] = zn;                  // parity của bộ 1
                a3 = a2; a2 = a1; a1 = an;     // dịch lịch sử
                // ---- bộ mã 2: đầu vào đã xen Pi(i) = (f1*i + f2*i^2) mod K ----
                i64  = i;                                      // i dạng 64 bit
                p64  = (f1_64 * i64 + f2_64 * i64 * i64) % k64; // công thức QPP trực tiếp
                pidx = p64[31:0];                              // chỉ số xen (nhỏ hơn K)
                xn = in_bits[pidx];            // bit x(Pi(i))
                an = xn ^ b2 ^ b3;             // a'(n)
                zn = an ^ b1 ^ b3;             // z'(n)
                ref2[i] = zn;                  // parity của bộ 2
                b3 = b2; b2 = b1; b1 = an;     // dịch lịch sử
            end
            for (t = 0; t < 3; t = t + 1) begin // 3 nhịp kết thúc trellis của cả hai bộ
                xn = a2 ^ a3;                  // bit vào = hồi tiếp để a(n) = 0
                an = xn ^ a2 ^ a3;             // = 0
                zn = an ^ a1 ^ a3;             // parity trong lúc kết thúc
                ex1[t] = xn; ez1[t] = zn;      // lưu x và z của bộ 1
                a3 = a2; a2 = a1; a1 = an;     // dịch
                xn = b2 ^ b3;                  // tương tự cho bộ 2
                an = xn ^ b2 ^ b3;             // = 0
                zn = an ^ b1 ^ b3;             // parity bộ 2
                ex2[t] = xn; ez2[t] = zn;      // lưu x' và z'
                b3 = b2; b2 = b1; b1 = an;     // dịch
            end
            if ((a1 | a2 | a3 | b1 | b2 | b3) !== 1'b0) begin // mô hình tham chiếu tự kiểm tra
                errors = errors + 1;
                $display("[ERR] mo hinh tham chieu khong ket thuc trellis (K=%0d)", kk);
            end
            // ---- xếp 12 bit đuôi theo TS 36.212 mục 5.1.3.2.2 ----
            ref0[kk]   = ex1[0]; ref1[kk]   = ez1[0]; ref2[kk]   = ex1[1]; // chỉ số K
            ref0[kk+1] = ez1[1]; ref1[kk+1] = ex1[2]; ref2[kk+1] = ez1[2]; // chỉ số K+1
            ref0[kk+2] = ex2[0]; ref1[kk+2] = ez2[0]; ref2[kk+2] = ex2[1]; // chỉ số K+2
            ref0[kk+3] = ez2[1]; ref1[kk+3] = ex2[2]; ref2[kk+3] = ez2[2]; // chỉ số K+3
        end
    endtask

    // ------------------------ các task tạo dữ liệu vào ------------------------
    task fill_random;                  // điền ngẫu nhiên (hạt giống mặc định, lặp lại được)
        input integer kk;              // số bit
        integer i;                     // biến vòng lặp
        begin
            for (i = 0; i < kk; i = i + 1) in_bits[i] = $random; // lấy bit thấp của $random
        end
    endtask

    task fill_const;                   // điền cùng một giá trị
        input integer kk;              // số bit
        input         v;               // giá trị 0 hoặc 1
        integer i;                     // biến vòng lặp
        begin
            for (i = 0; i < kk; i = i + 1) in_bits[i] = v;
        end
    endtask

    // ----------- gửi một khối vào DUT (gọi tại sườn xuống của clk) -----------
    task send_block;
        input integer kk;              // kích thước khối
        input integer stall_pct;       // % nhịp bên phát không gửi bit (0 = gửi liên tục)
        input integer inject_at;       // -1: không; n>=0: tại bit thứ n phát thêm 1 xung start giả (DUT phải bỏ qua)
        integer sent;                  // số bit đã gửi
        integer guard;                 // bộ đếm chống treo vòng gửi
        reg     injected;              // đã chèn start giả chưa
        begin
            guard    = 0;              // chưa quá nhịp nào
            cap_cnt  = 0;              // xóa bộ thu
            got_last = 1'b0;           // chưa thấy nhịp cuối
            k_in     = kk;             // đặt K
            start    = 1'b1;           // phát xung start
            in_valid = 1'b0;           // chưa gửi bit
            din      = 1'b0;           // bit vào = 0
            @(negedge clk);            // giữ start 1 nhịp
            start    = 1'b0;           // hạ start
            sent     = 0;              // chưa gửi bit nào
            injected = 1'b0;           // chưa chèn start giả
            while (sent < kk && guard < (30 * kk + 500)) begin // gửi đủ kk bit (có chặn treo nếu DUT không bao giờ sẵn sàng)
                @(negedge clk);        // mỗi vòng ứng với một nhịp clk
                guard = guard + 1;     // đếm nhịp
                if (inject_at >= 0 && sent == inject_at && !injected) begin
                    start = 1'b1; k_in = 13'd48; injected = 1'b1; // start giả với K khác: DUT phải bỏ qua
                end else begin
                    start = 1'b0; k_in = kk;                      // bình thường: không có start
                end
                if (in_ready && (({$random} % 100) >= stall_pct)) begin // sẵn sàng và không bị nghẽn
                    in_valid = 1'b1;   // bit hợp lệ
                    din      = in_bits[sent]; // bit thật
                    sent     = sent + 1;      // đã gửi thêm một bit
                end else begin         // nghẽn hoặc DUT chưa sẵn sàng
                    in_valid = (!in_ready) ? $random : 1'b0; // chưa sẵn sàng mà vẫn bật valid ngẫu nhiên: DUT phải bỏ qua
                    din      = $random;                      // bit rác: không được lọt vào khối
                end
            end
            if (sent < kk) begin       // DUT không nhận đủ bit trong thời hạn
                errors = errors + 1;
                $display("[ERR] DUT khong nhan du bit: moi gui %0d / %0d (in_ready khong len)", sent, kk);
            end
            @(negedge clk);            // sau bit cuối
            in_valid = 1'b0;           // ngừng gửi
            din      = 1'b0;           // bit vào = 0
            start    = 1'b0;           // không còn start
            k_in     = kk;             // giữ K
        end
    endtask

    task wait_done;                    // chờ DUT phát xong khối (có chặn treo)
        input integer kk;              // kích thước khối
        integer guard;                 // bộ đếm chống treo
        begin
            guard = 0;                 // bắt đầu đếm
            while (!got_last && guard < (4 * kk + 200)) begin
                @(negedge clk);        // chờ thêm một nhịp
                in_valid = $random;    // bus vào bị "nhiễu": DUT không ở pha LOAD nên phải bỏ qua hoàn toàn
                din      = $random;    // bit rác kèm theo
                guard = guard + 1;     // tăng bộ đếm
            end
            in_valid = 1'b0;           // trả bus vào về trạng thái nghỉ
            din      = 1'b0;           // bit vào = 0
            if (!got_last) begin       // quá hạn mà chưa thấy out_last
                errors = errors + 1;
                $display("[ERR] het gio cho out_last (K=%0d)", kk);
            end
        end
    endtask

    // --------------------- so sánh kết quả với mô hình chuẩn ---------------------
    task check_block;
        input integer kk;              // kích thước khối
        input [8*24-1:0] name;         // nhãn ca thử (chuỗi ASCII 24 ký tự)
        integer i, le;                 // biến lặp và số lỗi cục bộ
        begin
            le = 0;                    // chưa có lỗi
            if (cap_cnt != kk + 4) begin // phải thu đúng K + 4 phần tử
                le = le + 1;
                $display("[ERR] %0s K=%0d : so phan tu thu = %0d, mong doi %0d", name, kk, cap_cnt, kk + 4);
            end
            for (i = 0; i < kk + 4 && i < cap_cnt; i = i + 1) begin
                if (cap0[i] !== ref0[i] || cap1[i] !== ref1[i] || cap2[i] !== ref2[i]) begin
                    le = le + 1;       // sai khác ở chỉ số i
                    if (le <= 5) $display("[ERR] %0s K=%0d i=%0d : DUT={%b%b%b} chuan={%b%b%b}", name, kk, i,
                                          cap0[i], cap1[i], cap2[i], ref0[i], ref1[i], ref2[i]);
                end
                if (cap_tail[i] !== (i >= kk)) begin // out_tail chỉ được bật ở 4 phần tử cuối
                    le = le + 1;
                    if (le <= 5) $display("[ERR] %0s K=%0d i=%0d : out_tail sai", name, kk, i);
                end
            end
            if (term_ok !== 1'b1) begin // trellis phải kết thúc về trạng thái 0
                le = le + 1;
                $display("[ERR] %0s K=%0d : term_ok != 1", name, kk);
            end
            if (busy !== 1'b0) begin    // sau out_last DUT phải rảnh
                le = le + 1;
                $display("[ERR] %0s K=%0d : busy != 0 sau khi xong", name, kk);
            end
            errors  = errors + le;      // cộng dồn lỗi
            n_tests = n_tests + 1;      // thêm một ca thử
            if (le == 0) begin
                if (!quiet) $display("[PASS] %0s K=%0d", name, kk); // ca đạt: chỉ in khi không ở chế độ gọn
            end else $display("[FAIL] %0s K=%0d (%0d loi)", name, kk, le);
        end
    endtask

    // So sánh với vector cố định (dạng hex, bit i = phần tử thứ i)
    task check_literal;
        input integer kk;              // kích thước khối
        input [63:0]  e0;              // d0 mong đợi
        input [63:0]  e1;              // d1 mong đợi
        input [63:0]  e2;              // d2 mong đợi
        integer i, le;                 // biến lặp, số lỗi
        begin
            le = 0;                    // chưa có lỗi
            for (i = 0; i < kk + 4; i = i + 1)
                if (cap0[i] !== e0[i] || cap1[i] !== e1[i] || cap2[i] !== e2[i]) le = le + 1;
            errors  = errors + le;     // cộng dồn
            n_tests = n_tests + 1;     // thêm ca thử
            if (le == 0) $display("[PASS] vector co dinh K=%0d khop 100%%", kk);
            else         $display("[FAIL] vector co dinh K=%0d (%0d loi)", kk, le);
        end
    endtask

    // Chạy một ca thử đầy đủ: tạo chuẩn -> gửi -> chờ -> so sánh. Gọi tại sườn xuống clk.
    task run_test;
        input integer kk;              // kích thước khối
        input integer stall_pct;       // % nghẽn
        input integer inject_at;       // vị trí chèn start giả (-1: không)
        input integer settle;          // số nhịp chờ thêm sau out_last để phát hiện ngõ ra thừa
        input [8*24-1:0] name;         // nhãn
        begin
            make_reference(kk);        // tính luồng chuẩn
            send_block(kk, stall_pct, inject_at); // đẩy dữ liệu vào DUT
            wait_done(kk);             // chờ phát xong
            repeat (settle) @(negedge clk); // chờ thêm cho chắc
            check_block(kk, name);     // so sánh
        end
    endtask

    // ------------------------- ca thử K không hợp lệ -------------------------
    task invalid_k_test;
        input integer kbad;            // K sai cần thử
        integer err_prev;              // số xung err_k trước khi thử
        begin
            err_prev = err_cnt;       // nhớ giá trị cũ
            cap_cnt  = 0;              // xóa bộ thu
            k_in     = kbad;           // đặt K sai
            start    = 1'b1;           // phát start
            @(negedge clk);            // giữ 1 nhịp
            start    = 1'b0;           // hạ start
            repeat (5) @(negedge clk); // chờ DUT xử lý
            n_tests = n_tests + 1;     // thêm ca thử
            if (err_cnt == err_prev + 1 && busy === 1'b0 && in_ready === 1'b0 && cap_cnt == 0)
                $display("[PASS] K=%0d bi tu choi dung (err_k 1 xung, khong phat du lieu)", kbad);
            else begin
                errors = errors + 1;   // sai hành vi
                $display("[FAIL] K=%0d : err_k=%0d xung, busy=%b, in_ready=%b, cap=%0d", kbad,
                         err_cnt - err_prev, busy, in_ready, cap_cnt);
            end
        end
    endtask

    // ---------------- reset giữa chừng rồi kiểm tra khôi phục ----------------
    task abort_test;
        input integer kk;              // kích thước khối
        input integer nbits;           // số bit gửi trước khi reset (>= kk nghĩa là gửi đủ)
        input integer extra;           // số nhịp chờ thêm trước khi reset
        input integer wait_tail;       // 1: chờ đến khi DUT bắt đầu phát bit đuôi rồi mới reset
        integer sent, guard;           // số bit đã gửi, bộ đếm chống treo
        begin
            cap_cnt = 0; got_last = 1'b0;      // xóa bộ thu
            k_in = kk; start = 1'b1; in_valid = 1'b0; din = 1'b0;
            @(negedge clk);                    // giữ start 1 nhịp
            start = 1'b0; sent = 0; guard = 0; // hạ start, xóa bộ đếm chống treo
            while (sent < nbits && sent < kk && guard < (4 * kk + 200)) begin // gửi nbits bit đầu (có chặn treo)
                @(negedge clk);
                guard = guard + 1;
                if (in_ready) begin in_valid = 1'b1; din = $random; sent = sent + 1; end
                else begin in_valid = 1'b0; din = 1'b0; end
            end
            @(negedge clk); in_valid = 1'b0; din = 1'b0; // ngừng gửi
            repeat (extra) @(negedge clk);     // để DUT chạy thêm một lúc
            if (wait_tail) begin               // chờ đến pha phát đuôi
                guard = 0;
                while (out_tail !== 1'b1 && guard < (4 * kk + 200)) begin @(negedge clk); guard = guard + 1; end
            end
            rst = 1'b1;                        // reset giữa chừng
            @(negedge clk); @(negedge clk);    // giữ reset 2 nhịp
            rst = 1'b0;                        // nhả reset
            @(negedge clk);                    // 1 nhịp sau đó kiểm tra
            n_tests = n_tests + 1;             // thêm ca thử
            if (busy === 1'b0 && in_ready === 1'b0 && out_valid === 1'b0)
                $display("[PASS] reset giua chung (K=%0d, gui %0d bit, wait_tail=%0d): DUT ve IDLE", kk, nbits, wait_tail);
            else begin
                errors = errors + 1;           // DUT không về IDLE
                $display("[FAIL] reset giua chung (K=%0d): busy=%b in_ready=%b out_valid=%b", kk, busy, in_ready, out_valid);
            end
        end
    endtask

    // ============================ CHUỖI KIỂM TRA CHÍNH ============================
    reg [39:0] lit_in40;               // đầu vào cố định K = 40
    reg [47:0] lit_in48;               // đầu vào cố định K = 48
    integer    j;                      // biến lặp phụ
    integer    kk_sw;                  // K đang thử trong bước quét 188 kích thước
    integer    err_before;             // số lỗi trước bước quét
    integer    tests_before;           // số ca thử trước bước quét

    initial begin
        // ---- khởi tạo biến và reset ----
        rst = 1'b1; start = 1'b0; k_in = 13'd0; in_valid = 1'b0; din = 1'b0; // giữ reset từ đầu
        quiet = 1'b0; errors = 0; x_errors = 0; n_tests = 0; cap_cnt = 0; got_last = 1'b0; err_cnt = 0;
        klist[0]  = 40;   klist[1]  = 48;   klist[2]  = 56;   klist[3]  = 64;
        klist[4]  = 88;   klist[5]  = 104;  klist[6]  = 128;  klist[7]  = 256;
        klist[8]  = 504;  klist[9]  = 512;  klist[10] = 528;  klist[11] = 1024;
        klist[12] = 1056; klist[13] = 2048; klist[14] = 2112; klist[15] = 4032;
        klist[16] = 6144;              // kích thước tối đa của LTE
        repeat (4) @(negedge clk);     // giữ reset vài nhịp
        rst = 1'b0;                    // nhả reset
        @(negedge clk);                // sang sườn xuống tiếp theo

        $display("=== TB turbo_encoder: bat dau ===");

        // ---- 1) trạng thái sau reset ----
        n_tests = n_tests + 1;
        if (busy === 1'b0 && in_ready === 1'b0 && out_valid === 1'b0 && term_ok === 1'b0 && err_k === 1'b0)
            $display("[PASS] trang thai sau reset: busy=0 in_ready=0 out_valid=0");
        else begin
            errors = errors + 1;
            $display("[FAIL] trang thai sau reset sai");
        end

        // ---- 2) vector cố định K = 40 và K = 48 ----
        lit_in40 = 40'hB2179057FC;         // đầu vào cố định K = 40
        for (j = 0; j < 40; j = j + 1) in_bits[j] = lit_in40[j];
        run_test(40, 0, -1, 3, "vector K=40");
        check_literal(40, 44'hDB2179057FC, 44'h17AF4049314, 44'h470B3BE792C);
        lit_in48 = 48'h5E5F7866EFD1;         // đầu vào cố định K = 48
        for (j = 0; j < 48; j = j + 1) in_bits[j] = lit_in48[j];
        run_test(48, 0, -1, 3, "vector K=48");
        check_literal(48, 52'hE5E5F7866EFD1, 52'h203A5418292FF, 52'h6C545EA8605BB);

        // ---- 3) quét nhiều kích thước K với dữ liệu ngẫu nhiên ----
        for (ti = 0; ti < 17; ti = ti + 1) begin
            fill_random(klist[ti]);
            run_test(klist[ti], 0, -1, 3, "ngau nhien");
        end

        // ---- 4) dữ liệu đặc biệt: toàn 0 và toàn 1 ----
        fill_const(40, 1'b0);   run_test(40,   0, -1, 3, "toan 0");
        fill_const(40, 1'b1);   run_test(40,   0, -1, 3, "toan 1");
        fill_const(6144, 1'b0); run_test(6144, 0, -1, 3, "toan 0");
        fill_const(6144, 1'b1); run_test(6144, 0, -1, 3, "toan 1");

        // ---- 5) bên phát bị nghẽn (in_valid ngắt quãng, bit rác khi valid = 0) ----
        fill_random(104);  run_test(104, 30, -1, 3, "nghen 30%");
        fill_random(528);  run_test(528, 70, -1, 3, "nghen 70%");
        fill_random(40);   run_test(40,  90, -1, 3, "nghen 90%");

        // ---- 6) start giả khi DUT đang bận phải bị bỏ qua ----
        fill_random(56);   run_test(56,  0, 10, 3, "start gia @10");
        fill_random(56);   run_test(56, 20,  0, 3, "start gia @0");

        // ---- 7) K không hợp lệ ----
        invalid_k_test(41);
        invalid_k_test(0);
        invalid_k_test(39);
        invalid_k_test(520);           // nằm giữa 512 và 528 nên không có trong bảng
        invalid_k_test(6143);
        invalid_k_test(6145);
        invalid_k_test(8191);
        fill_random(64);   run_test(64, 0, -1, 3, "sau K sai");

        // ---- 8) reset giữa chừng rồi chạy lại khối đầy đủ ----
        abort_test(104, 50, 0, 0);     // reset khi đang nhận bit (LOAD)
        fill_random(104);  run_test(104, 0, -1, 3, "sau reset LOAD");
        abort_test(64, 64, 20, 0);     // reset khi đang mã hóa (ENC)
        fill_random(64);   run_test(64,  0, -1, 3, "sau reset ENC");
        abort_test(48, 48, 0, 1);      // reset khi đang phát bit đuôi (TAIL)
        fill_random(48);   run_test(48,  0, -1, 3, "sau reset TAIL");

        // ---- 9) các khối nối đuôi nhau, không chờ thêm nhịp nào ----
        fill_random(40);   run_test(40,  0, -1, 0, "noi duoi #1");
        fill_random(48);   run_test(48,  0, -1, 0, "noi duoi #2");
        fill_random(56);   run_test(56,  0, -1, 0, "noi duoi #3");
        repeat (10) @(negedge clk);    // chờ thêm để chắc chắn không có ngõ ra thừa

        // ---- 10) quét toàn bộ 188 kích thước K (TS 36.212 Bảng 5.1.3-3) ----
        if (FULL_SWEEP != 0) begin
            $display("--- quet toan bo 188 kich thuoc K (chi in khi co loi) ---");
            quiet        = 1'b1;       // chế độ gọn
            err_before   = errors;     // nhớ số lỗi trước khi quét
            tests_before = n_tests;    // nhớ số ca trước khi quét
            for (ti = 0; ti < 188; ti = ti + 1) begin
                if (ti < 60)       kk_sw = 40   + 8  * ti;          // K = 40..512  bước 8   (60 giá trị)
                else if (ti < 92)  kk_sw = 528  + 16 * (ti - 60);   // K = 528..1024 bước 16 (32 giá trị)
                else if (ti < 124) kk_sw = 1056 + 32 * (ti - 92);   // K = 1056..2048 bước 32 (32 giá trị)
                else               kk_sw = 2112 + 64 * (ti - 124);  // K = 2112..6144 bước 64 (64 giá trị)
                fill_random(kk_sw);                                 // dữ liệu ngẫu nhiên
                run_test(kk_sw, 0, -1, 1, "quet 188 K");            // chạy và so sánh với mô hình chuẩn
            end
            quiet = 1'b0;              // trở lại chế độ in đầy đủ
            if (errors == err_before)
                $display("[PASS] quet du %0d kich thuoc K cua LTE: tat ca khop mo hinh chuan", n_tests - tests_before);
            else
                $display("[FAIL] quet 188 K: %0d loi", errors - err_before);
        end

        // ---- tổng kết ----
        n_tests = n_tests + 1;
        if (x_errors == 0) $display("[PASS] khong co X/Z tren ngo ra trong toan bo mo phong");
        else begin errors = errors + 1; $display("[FAIL] co %0d lan X/Z tren ngo ra", x_errors); end

        $display("=== Tong ket: %0d ca thu, %0d loi, %0d lan X/Z ===", n_tests, errors, x_errors);
        if (errors == 0 && x_errors == 0) $display("TEST PASSED");
        else                              $display("TEST FAILED");
        $finish;                       // kết thúc mô phỏng
    end

    initial begin                      // bộ canh giờ toàn cục phòng khi mô phỏng bị treo
        #400_000_000;                  // 0,4 giây thời gian mô phỏng
        $display("TEST FAILED (TIMEOUT)");
        $finish;
    end

endmodule
