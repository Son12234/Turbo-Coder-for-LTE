`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : tb_turbo_decoder.v
// Chức năng : Testbench tự kiểm tra VÒNG KÍN  bộ mã hóa -> kênh nhiễu -> bộ giải mã
//             (turbo_encoder + turbo_decoder, Turbo code LTE tốc độ 1/3).
//
// Cách kiểm tra:
//   * Bit thông tin ngẫu nhiên (PRNG xorshift32 viết ngay trong testbench, nên kết quả GIỐNG HỆT
//     trên mọi trình mô phỏng: Vivado xsim, ModelSim, Icarus...) được đưa vào bộ mã hóa thật.
//     Ba luồng d0,d1,d2 (K+4 bộ ba) được thu vào mảng.
//   * Kênh được mô hình hóa bằng bộ tạo LLR:  chế độ 0 = không nhiễu (LLR = +-A),
//     chế độ 1 = kênh nhị phân đối xứng (lật CHÍNH XÁC n bit tại vị trí ngẫu nhiên, kể cả bit đuôi),
//     chế độ 2 = kênh cộng nhiễu xấp xỉ Gauss (A + tổng 4 số ngẫu nhiên đều).
//   * Bộ giải mã chạy với số vòng lặp chọn trước; mọi bit ra phải trùng bit gốc.
//   * Kiểm tra thêm: thứ tự chỉ số ra 0..K-1, đúng K bit, out_last đúng chỗ, K sai bị từ chối (err_k),
//     LLR vào bị ngắt quãng (in_valid = 0 xen kẽ, kèm dữ liệu rác), reset giữa chừng (ở LOAD và ở giữa
//     lúc giải mã), các khối nối đuôi nhau với K khác nhau, lợi ích của việc tăng số vòng lặp,
//     và ca thử "chỉ có LLR bit đuôi mới cứu được 3 bit cuối" (xem task tail_only_test) cho cả hai SISO.
//   * Mỗi nhịp đều kiểm tra các ngõ ra không có X/Z.
//   * FULL_SWEEP = 1 : giải mã không nhiễu TOÀN BỘ 188 kích thước K của LTE (lâu hơn).
// Kết quả cuối: in "TEST PASSED" hoặc "TEST FAILED" rồi gọi $finish.
//==============================================================================
module tb_turbo_decoder #(
    parameter FULL_SWEEP = 0,              // 1: quét thêm tất cả 188 kích thước K (mất vài phút); 0: chỉ quét mẫu
    parameter [31:0] SEED = 32'h1BADB002,  // hạt giống của PRNG (khác 0); đổi để thử kênh khác
    parameter integer TAIL_BLOCKS = 60     // số khối nhiễu dùng cho ca thử độ nhạy với LLR bit đuôi (mục 6b); tăng nếu đổi SEED
);

    localparam LW = 7;                     // độ rộng LLR của bộ giải mã (phải trùng tham số mặc định của DUT)
    localparam integer LMAXC = (1 << (LW-1)) - 1;   // LLR lớn nhất (+63)

    // ------------------------- xung nhịp và reset -------------------------
    reg clk;                               // xung nhịp 100 MHz
    reg rst_e;                             // reset bộ mã hóa
    reg rst_d;                             // reset bộ giải mã (tách riêng để thử reset giữa chừng)
    initial clk = 1'b0;                    // bắt đầu ở mức thấp
    always #5 clk = ~clk;                  // chu kỳ 10 ns

    // ------------------------- bộ mã hóa -------------------------
    reg         e_start;                   // xung bắt đầu khối
    reg  [12:0] e_k;                       // kích thước khối
    reg         e_in_valid;                // bit vào hợp lệ
    reg         e_din;                     // bit vào
    wire        e_in_ready, e_out_valid, e_d0, e_d1, e_d2, e_out_tail, e_out_last, e_busy, e_err_k, e_term_ok;
    turbo_encoder u_enc (
        .clk(clk), .rst(rst_e), .start(e_start), .k_in(e_k),
        .in_ready(e_in_ready), .in_valid(e_in_valid), .din(e_din),
        .out_valid(e_out_valid), .d0(e_d0), .d1(e_d1), .d2(e_d2),
        .out_tail(e_out_tail), .out_last(e_out_last),
        .busy(e_busy), .err_k(e_err_k), .term_ok(e_term_ok)
    );

    // ------------------------- bộ giải mã -------------------------
    reg         d_start;                   // xung bắt đầu khối
    reg  [12:0] d_k;                       // kích thước khối
    reg  [3:0]  d_iters;                   // số vòng lặp
    reg         d_in_valid;                // LLR vào hợp lệ
    reg  signed [LW-1:0] d_l0, d_l1, d_l2; // ba LLR vào
    wire        d_in_ready, d_out_valid, d_out_bit, d_out_last, d_busy, d_err_k;
    wire [12:0] d_out_idx;
    turbo_decoder #(.LW(LW)) u_dec (
        .clk(clk), .rst(rst_d), .start(d_start), .k_in(d_k), .iter_in(d_iters),
        .in_ready(d_in_ready), .in_valid(d_in_valid),
        .in_l0(d_l0), .in_l1(d_l1), .in_l2(d_l2),
        .out_valid(d_out_valid), .out_bit(d_out_bit), .out_idx(d_out_idx), .out_last(d_out_last),
        .busy(d_busy), .err_k(d_err_k)
    );

    // ------------------------- PRNG xorshift32 -------------------------
    reg [31:0] rng;                        // trạng thái bộ tạo số ngẫu nhiên
    function [31:0] xorshift;              // một bước xorshift32 (chu kỳ 2^32 - 1)
        input [31:0] s;
        reg   [31:0] t;
        begin
            t = s;                         // sao chép trạng thái
            t = t ^ (t << 13);             // trộn bit
            t = t ^ (t >> 17);             // trộn bit
            t = t ^ (t << 5);              // trộn bit
            xorshift = t;                  // trả về trạng thái mới
        end
    endfunction
    task next_rand;                        // tiến PRNG một bước (kết quả nằm trong biến rng)
        begin rng = xorshift(rng); end
    endtask

    // ------------------------- dữ liệu của khối đang thử -------------------------
    reg         in_bits  [0:6143];         // bit thông tin gốc
    reg         cd0 [0:6147];              // luồng mã hóa d0 (K+4 bit)
    reg         cd1 [0:6147];              // luồng mã hóa d1
    reg         cd2 [0:6147];              // luồng mã hóa d2
    reg  signed [LW-1:0] ll0 [0:6147];     // LLR đưa vào bộ giải mã (sau kênh)
    reg  signed [LW-1:0] ll1 [0:6147];
    reg  signed [LW-1:0] ll2 [0:6147];
    reg         fmask [0:18443];           // mặt nạ lật bit cho kênh nhị phân đối xứng (3*(K+4) vị trí)
    reg         dec_bits [0:6143];         // bit giải mã thu được
    integer     ovr_n;                     // số bit thông tin bị ghi đè trong encode_block (0 = không ghi đè)
    integer     ovr_pos [0:2];             // chỉ số (thứ tự gốc) của các bit bị ghi đè
    reg         ovr_val [0:2];             // giá trị gán cho các bit đó

    // ------------------------- bộ đếm kiểm tra -------------------------
    integer total_checks;                  // tổng số ca kiểm tra
    integer total_fail;                    // số ca sai
    integer x_errors;                      // số lần thấy X/Z trên ngõ ra
    integer last_errs;                     // số bit sai của lần giải mã gần nhất

    // ------------------------- thu ngõ ra bộ mã hóa -------------------------
    integer enc_cnt;                       // số bộ ba đã thu
    reg     enc_cap_en;                    // =1: đang thu
    always @(posedge clk) begin
        if (e_out_valid === 1'b1 && enc_cap_en) begin     // mỗi nhịp hợp lệ lưu một bộ ba
            if (enc_cnt < 6148) begin
                cd0[enc_cnt] <= e_d0;                     // lưu d0
                cd1[enc_cnt] <= e_d1;                     // lưu d1
                cd2[enc_cnt] <= e_d2;                     // lưu d2
            end
            enc_cnt <= enc_cnt + 1;                       // đếm
        end
    end

    // ------------------------- thu ngõ ra bộ giải mã -------------------------
    integer dec_cnt;                       // số bit ra đã nhận
    reg     dec_cap_en;                    // =1: đang chờ kết quả
    reg     dec_done;                      // =1: đã nhận out_last
    integer idx_err;                       // số lần chỉ số ra không đúng thứ tự
    integer last_err;                      // số lần out_last sai chỗ
    integer spurious;                      // số lần có out_valid khi không chờ kết quả
    always @(posedge clk) begin
        if (d_out_valid === 1'b1) begin
            if (!dec_cap_en) spurious <= spurious + 1;    // ngõ ra thừa
            else begin
                if (d_out_idx !== dec_cnt[12:0]) idx_err <= idx_err + 1;  // phải đúng 0,1,2,...
                if (d_out_idx < 6144) dec_bits[d_out_idx] <= d_out_bit;   // lưu bit
                dec_cnt <= dec_cnt + 1;
                if (d_out_last === 1'b1) begin
                    dec_done <= 1'b1;                      // bit cuối
                    if (d_out_idx !== (d_k - 13'd1)) last_err <= last_err + 1; // phải ở chỉ số K-1
                end else if (d_out_idx === (d_k - 13'd1)) last_err <= last_err + 1; // chỉ số K-1 mà thiếu out_last
            end
        end
    end

    // ------------------------- kiểm tra X/Z mỗi nhịp -------------------------
    always @(posedge clk) begin
        if ((^{e_in_ready, e_out_valid, e_busy, e_err_k, d_in_ready, d_out_valid, d_busy, d_err_k}) === 1'bx) begin
            x_errors <= x_errors + 1;
            if (x_errors < 5) $display("[ERR] t=%0t : X/Z tren tin hieu dieu khien", $time);
        end
        if (d_out_valid === 1'b1 && (^{d_out_bit, d_out_idx, d_out_last}) === 1'bx) begin
            x_errors <= x_errors + 1;
            if (x_errors < 5) $display("[ERR] t=%0t : X/Z tren bit giai ma", $time);
        end
        if (e_out_valid === 1'b1 && (^{e_d0, e_d1, e_d2}) === 1'bx) begin
            x_errors <= x_errors + 1;
            if (x_errors < 5) $display("[ERR] t=%0t : X/Z tren du lieu bo ma hoa", $time);
        end
    end

    // ------------------------- tiện ích -------------------------
    task check;                            // ghi nhận một ca kiểm tra
        input         cond;                // điều kiện phải đúng
        input [8*60-1:0] name;             // tên ca (chuỗi ASCII)
        begin
            total_checks = total_checks + 1;
            if (cond) $display("[PASS] %0s", name);
            else begin
                total_fail = total_fail + 1;
                $display("[FAIL] %0s", name);
            end
        end
    endtask

    task wait_cycles;                      // chờ n nhịp (đồng bộ theo sườn xuống)
        input integer n;
        integer w;
        begin for (w = 0; w < n; w = w + 1) @(negedge clk); end
    endtask

    task wait_idle;                        // chờ cả hai khối rảnh
        integer g;
        begin
            g = 0;
            while ((e_busy !== 1'b0 || d_busy !== 1'b0) && g < 200000) begin @(negedge clk); g = g + 1; end
        end
    endtask

    // ------------------------- bước 1: mã hóa một khối -------------------------
    task encode_block;                     // tạo bit ngẫu nhiên và mã hóa bằng bộ mã hóa thật
        input integer kk;
        integer i, g;
        begin
            wait_idle;                     // bảo đảm rảnh
            for (i = 0; i < kk; i = i + 1) begin           // sinh bit thông tin
                next_rand;
                in_bits[i] = rng[16];
            end
            for (i = 0; i < ovr_n; i = i + 1) in_bits[ovr_pos[i]] = ovr_val[i];   // ghi đè vài bit (chỉ dùng trong tail_only_test)
            enc_cnt = 0; enc_cap_en = 1'b1;                // bắt đầu thu
            @(negedge clk); e_k = kk; e_start = 1'b1;      // xung start
            @(negedge clk); e_start = 1'b0;
            g = 0;
            while (e_in_ready !== 1'b1 && g < 100) begin @(negedge clk); g = g + 1; end  // chờ sẵn sàng
            i = 0;
            while (i < kk) begin                           // đưa K bit vào
                @(negedge clk);
                if (e_in_ready === 1'b1) begin e_in_valid = 1'b1; e_din = in_bits[i]; i = i + 1; end
                else begin e_in_valid = 1'b0; e_din = 1'b0; end
            end
            @(negedge clk); e_in_valid = 1'b0; e_din = 1'b0;
            g = 0;
            while (enc_cnt < kk + 4 && g < 20 * kk + 1000) begin @(negedge clk); g = g + 1; end // chờ đủ K+4 bộ ba
            @(negedge clk); enc_cap_en = 1'b0;
        end
    endtask

    // ------------------------- bước 2: kênh -> LLR -------------------------
    // ch_mode: 0 = không nhiễu ; 1 = lật đúng n_flip bit ; 2 = nhiễu cộng xấp xỉ Gauss
    // amp: biên độ LLR (+-amp) ; nw: nửa độ rộng của mỗi số ngẫu nhiên đều (chế độ 2)
    task build_llr;
        input integer kk;
        input integer ch_mode;
        input integer amp;
        input integer n_flip;
        input integer nw;
        integer t, s, placed, pos, v, nz, q, rr;
        reg b;
        begin
            if (ch_mode == 1) begin                         // lập mặt nạ lật bit
                for (t = 0; t < 3 * (kk + 4); t = t + 1) fmask[t] = 1'b0;
                placed = 0;
                while (placed < n_flip) begin
                    next_rand;
                    pos = rng[27:8];                        // số nguyên không âm
                    pos = pos % (3 * (kk + 4));             // vị trí ngẫu nhiên trong 3*(K+4)
                    if (!fmask[pos]) begin fmask[pos] = 1'b1; placed = placed + 1; end
                end
            end
            for (t = 0; t < kk + 4; t = t + 1) begin        // mỗi bộ ba
                for (s = 0; s < 3; s = s + 1) begin         // mỗi luồng d0,d1,d2
                    b = (s == 0) ? cd0[t] : ((s == 1) ? cd1[t] : cd2[t]);
                    if (ch_mode == 1) b = b ^ fmask[3 * t + s];   // lật bit nếu nằm trong mặt nạ
                    v = b ? -amp : amp;                     // bit 0 -> +amp ; bit 1 -> -amp
                    if (ch_mode == 2) begin                 // cộng nhiễu: tổng 4 số đều trong [-nw, nw]
                        nz = 0;
                        for (q = 0; q < 4; q = q + 1) begin
                            next_rand;
                            rr = rng[23:8];                 // số nguyên không âm 0..65535
                            rr = rr % (2 * nw + 1);         // 0 .. 2*nw
                            nz = nz + rr - nw;              // đều trong [-nw, nw]
                        end
                        v = v + nz;
                    end
                    if (v > LMAXC) v = LMAXC;               // bão hòa về LW bit
                    if (v < -LMAXC) v = -LMAXC;
                    if (s == 0) ll0[t] = v; else if (s == 1) ll1[t] = v; else ll2[t] = v;
                end
            end
        end
    endtask

    // ------------------------- bước 3: giải mã và so sánh -------------------------
    // gap_pct: xác suất (%) chèn khoảng trống (in_valid = 0 kèm dữ liệu rác) trước mỗi bộ ba LLR
    task decode_block;
        input integer kk;
        input integer iters;
        input integer gap_pct;
        integer i, g, lim;
        begin
            wait_idle;
            dec_cnt = 0; dec_done = 1'b0; idx_err = 0; last_err = 0; dec_cap_en = 1'b1;
            @(negedge clk); d_k = kk; d_iters = iters; d_start = 1'b1;     // xung start
            @(negedge clk); d_start = 1'b0;
            g = 0;
            while (d_in_ready !== 1'b1 && g < 100) begin @(negedge clk); g = g + 1; end
            for (i = 0; i < kk + 4; i = i + 1) begin                       // đưa K+4 bộ ba LLR
                next_rand;
                if ((rng[15:8] % 100) < gap_pct) begin                      // chèn khoảng trống
                    d_in_valid = 1'b0; d_l0 = rng[6:0]; d_l1 = rng[13:7]; d_l2 = rng[20:14]; // dữ liệu rác
                    @(negedge clk);
                end
                d_in_valid = 1'b1; d_l0 = ll0[i]; d_l1 = ll1[i]; d_l2 = ll2[i];
                @(negedge clk);
            end
            d_in_valid = 1'b0; d_l0 = 0; d_l1 = 0; d_l2 = 0;
            lim = kk * (4 * iters + 8) + 5000;                              // giới hạn chờ (tránh treo)
            g = 0;
            while (!dec_done && g < lim) begin @(negedge clk); g = g + 1; end
            wait_cycles(4);
            dec_cap_en = 1'b0;
            last_errs = 0;                                                  // đếm bit sai
            for (i = 0; i < kk; i = i + 1) if (dec_bits[i] !== in_bits[i]) last_errs = last_errs + 1;
        end
    endtask

    // Ghép cả ba bước + kiểm tra chung; trả về số bit sai trong last_errs.
    task full_test;
        input integer kk;
        input integer iters;
        input integer ch_mode;
        input integer amp;
        input integer n_flip;
        input integer nw;
        input integer gap_pct;
        input integer expect_zero;         // 1: bắt buộc 0 bit sai ; 0: chỉ thông báo
        input [8*60-1:0] name;
        reg ok;                            // kết quả tổng hợp của ca thử
        begin
            encode_block(kk);
            build_llr(kk, ch_mode, amp, n_flip, nw);
            decode_block(kk, iters, gap_pct);
            ok = (dec_done === 1'b1) && (dec_cnt == kk) && (idx_err == 0) && (last_err == 0); // giao thức đúng
            if (expect_zero) ok = ok && (last_errs == 0);                                      // và không sai bit nào
            check(ok, name);
            if (!ok || !expect_zero)
                $display("[INFO] %0s : %0d bit sai / %0d (nhan %0d bit, loi chi so %0d, loi out_last %0d)",
                         name, last_errs, kk, dec_cnt, idx_err, last_err);
        end
    endtask

    // ------------------------- thử K sai ở bộ giải mã -------------------------
    task invalid_k_test;
        input integer kk;
        integer g, errs_seen, busy_seen;
        begin
            wait_idle;
            dec_cnt = 0; dec_done = 1'b0; spurious = 0; dec_cap_en = 1'b0; // không chờ kết quả: bit ra nào cũng bị tính là thừa
            errs_seen = 0; busy_seen = 0;
            @(negedge clk); d_k = kk; d_iters = 3; d_start = 1'b1;
            @(negedge clk); d_start = 1'b0;
            for (g = 0; g < 12; g = g + 1) begin             // quan sát 12 nhịp
                if (d_err_k === 1'b1) errs_seen = errs_seen + 1;
                @(negedge clk);
            end
            busy_seen = (d_busy === 1'b1);
            check(errs_seen == 1 && busy_seen == 0 && spurious == 0 && d_in_ready === 1'b0, "K sai bi tu choi (err_k 1 xung, khong busy)");
        end
    endtask

    // ------------------------- thử reset giữa chừng -------------------------
    // phase 0: reset khi đang LOAD ; phase 1: reset khi đang chạy SISO
    task reset_mid_test;
        input integer phase;
        input integer kk;
        integer i, g;
        begin
            encode_block(kk);
            build_llr(kk, 0, 20, 0, 0);
            wait_idle;
            dec_cnt = 0; dec_done = 1'b0; spurious = 0; dec_cap_en = 1'b1;
            @(negedge clk); d_k = kk; d_iters = 4; d_start = 1'b1;
            @(negedge clk); d_start = 1'b0;
            g = 0;
            while (d_in_ready !== 1'b1 && g < 100) begin @(negedge clk); g = g + 1; end
            for (i = 0; i < ((phase == 0) ? kk / 2 : kk + 4); i = i + 1) begin   // gửi một phần hoặc toàn bộ LLR
                d_in_valid = 1'b1; d_l0 = ll0[i]; d_l1 = ll1[i]; d_l2 = ll2[i];
                @(negedge clk);
            end
            d_in_valid = 1'b0;
            if (phase == 1) wait_cycles(kk + 50);             // để SISO chạy được một đoạn
            rst_d = 1'b1; wait_cycles(3); rst_d = 1'b0;       // reset bộ giải mã 3 nhịp
            wait_cycles(2);
            check(d_busy === 1'b0 && d_in_ready === 1'b0, "sau reset giua chung: bo giai ma ve IDLE");
            wait_cycles(8 * kk + 200);                        // không được có bit ra nào
            check(spurious == 0 && dec_cnt == 0, "sau reset giua chung: khong phat bit thua");
            dec_cap_en = 1'b0;
            full_test(kk, 4, 0, 20, 0, 0, 0, 1, "giai ma binh thuong sau reset giua chung");
        end
    endtask

    // ------------------------- ca thử "chỉ có bit đuôi mới cứu được 3 bit cuối" -------------------------
    // Ý tưởng: giấu hoàn toàn bằng chứng về 3 bit thông tin cuối của một bộ mã thành phần (xóa LLR về 0), xóa luôn
    // thông tin của bộ mã còn lại (để nó không "chữa" hộ), nhưng giữ nguyên LLR KHÔNG nhiễu của các bit đuôi của
    // bộ mã đang thử. Khi đó chỉ có đường LLR đuôi của SISO tương ứng mới xác định được 3 bit này, nên giải mã
    // đúng hay sai là kết quả XÁC ĐỊNH (không dính may rủi của nhiễu). Bắt được các lỗi: bỏ qua đuôi, nối nhầm
    // hoặc đảo thứ tự các LLR đuôi, đảo dấu LLR đuôi.
    //   which = 0 : thử SISO1. Ba bit bị giấu là bit số K-1, K-2, K-3 (thứ tự gốc). Xóa toàn bộ thông tin của
    //               bộ mã 2: luồng d2 ở các vị trí thông tin và ba bộ ba đuôi của bộ mã 2 (chỉ số K+2, K+3).
    //   which = 1 : thử SISO2. Ba bit bị giấu là các bit nằm ở 3 vị trí cuối SAU khi xen, tức bit số Pi(K-1),
    //               Pi(K-2), Pi(K-3). Xóa toàn bộ thông tin của bộ mã 1: luồng d1 ở các vị trí thông tin và hai
    //               bộ ba đuôi của bộ mã 1 (chỉ số K, K+1).
    // Chỉ dùng K = 40 (bảng LTE: f1 = 3, f2 = 10 => Pi(i) = (3*i + 10*i*i) mod 40).
    task tail_only_test;
        input integer which;               // 0: thử SISO1 ; 1: thử SISO2
        input [2:0]   pat;                 // giá trị 3 bit bị giấu: pat[j] là bit bị giấu thứ j (j = 0 ứng với vị trí cuối)
        input [8*60-1:0] name;
        integer j, t, pj;
        begin
            for (j = 0; j < 3; j = j + 1) begin
                pj = 40 - 1 - j;                                          // vị trí cuối thứ j trong thứ tự của bộ mã đang thử
                if (which == 0) ovr_pos[j] = pj;                          // SISO1: bit số K-1-j
                else            ovr_pos[j] = (3 * pj + 10 * pj * pj) % 40;// SISO2: bit số Pi(K-1-j)
                ovr_val[j] = pat[j];                                      // giá trị cần ép cho bit đó
            end
            ovr_n = 3;                                                    // bật ghi đè 3 bit trong encode_block
            encode_block(40);
            ovr_n = 0;                                                    // tắt ghi đè
            build_llr(40, 0, 20, 0, 0);                                   // kênh không nhiễu: LLR = +-20
            for (j = 0; j < 3; j = j + 1) begin                           // giấu bằng chứng của 3 bit đang thử
                ll0[ovr_pos[j]] = 0;                                      // LLR hệ thống của bit bị giấu
                if (which == 0) ll1[39 - j] = 0;                          // SISO1: parity 1 ở 3 vị trí cuối
                else            ll2[39 - j] = 0;                          // SISO2: parity 2 ở 3 vị trí cuối
            end
            for (t = 0; t < 44; t = t + 1) begin                          // xóa thông tin của bộ mã còn lại
                if (which == 0) begin
                    if (t < 40 || t >= 42) begin ll2[t] = 0; end          // luồng d2 ở vị trí thông tin
                    if (t >= 42)           begin ll0[t] = 0; ll1[t] = 0; end // đuôi bộ mã 2 (K+2, K+3): xóa cả d0, d1 (d2 đã xóa)
                end else begin
                    if (t < 42) begin ll1[t] = 0; end                     // luồng d1 ở vị trí thông tin và đuôi bộ mã 1
                    if (t >= 40 && t < 42) begin ll0[t] = 0; ll2[t] = 0; end // đuôi bộ mã 1 (K, K+1): xóa cả d0, d2
                end
            end
            decode_block(40, 2, 0);                                       // 2 vòng lặp
            check(dec_done === 1'b1 && dec_cnt == 40 && idx_err == 0 && last_err == 0 && last_errs == 0, name);
            if (last_errs != 0) $display("[INFO] %0s : %0d bit sai / 40", name, last_errs);
        end
    endtask

    // ------------------------- chương trình chính -------------------------
    integer ki, ks, e1, e8, nk;
    integer sweep_k [0:11];                // danh sách K mẫu cho bước quét nhanh
    initial begin
        // khởi tạo tín hiệu
        rst_e = 1'b1; rst_d = 1'b1;
        e_start = 1'b0; e_k = 13'd0; e_in_valid = 1'b0; e_din = 1'b0;
        d_start = 1'b0; d_k = 13'd0; d_iters = 4'd1; d_in_valid = 1'b0; d_l0 = 0; d_l1 = 0; d_l2 = 0;
        total_checks = 0; total_fail = 0; x_errors = 0; last_errs = 0;
        enc_cnt = 0; enc_cap_en = 1'b0; dec_cnt = 0; dec_cap_en = 1'b0; dec_done = 1'b0;
        idx_err = 0; last_err = 0; spurious = 0;
        rng = SEED;
        ovr_n = 0;
        $display("=== TB turbo_decoder (vong kin ma hoa -> kenh -> giai ma): bat dau ===");
        repeat (4) @(negedge clk);
        rst_e = 1'b0; rst_d = 1'b0;
        repeat (2) @(negedge clk);

        // ---- 1. không nhiễu, các kích thước nhỏ, 1 vòng lặp ----
        full_test(40,  1, 0, 20, 0, 0, 0, 1, "K=40 khong nhieu, 1 vong lap");
        full_test(48,  1, 0, 20, 0, 0, 0, 1, "K=48 khong nhieu, 1 vong lap");
        full_test(104, 2, 0, 20, 0, 0, 0, 1, "K=104 khong nhieu, 2 vong lap");

        // ---- 2. quét mẫu 12 kích thước K trải khắp 4 dải bước nhảy của bảng LTE ----
        sweep_k[0] = 56;   sweep_k[1] = 64;   sweep_k[2] = 128;  sweep_k[3] = 256;
        sweep_k[4] = 512;  sweep_k[5] = 528;  sweep_k[6] = 1024; sweep_k[7] = 1056;
        sweep_k[8] = 2048; sweep_k[9] = 2112; sweep_k[10] = 4096; sweep_k[11] = 6144;
        for (ki = 0; ki < 12; ki = ki + 1) begin
            nk = sweep_k[ki];
            $display("--- quet mau: K = %0d ---", nk);
            full_test(nk, 1, 0, 20, 0, 0, 0, 1, "quet mau khong nhieu 1 vong lap");
        end

        // ---- 3. kênh nhị phân đối xứng: lật chính xác n bit, 6 vòng lặp ----
        full_test(104,  6, 1, 16, 6,  0, 0, 1, "K=104 lat 6 bit (kenh BSC), 6 vong lap");
        full_test(512,  6, 1, 16, 20, 0, 0, 1, "K=512 lat 20 bit (kenh BSC), 6 vong lap");
        full_test(1024, 8, 1, 16, 40, 0, 0, 1, "K=1024 lat 40 bit (kenh BSC), 8 vong lap");

        // ---- 4. kênh nhiễu cộng xấp xỉ Gauss (K=1024: ~2 dB Eb/N0 ; K=512 và K=6144: ~4 dB Eb/N0) ----
        full_test(1024, 8, 2, 10, 0, 8, 0, 1, "K=1024 nhieu Gauss ~2dB, 8 vong lap");
        full_test(512,  8, 2, 10, 0, 6, 0, 1, "K=512 nhieu Gauss ~4dB, 8 vong lap");
        full_test(6144, 4, 2, 10, 0, 6, 0, 1, "K=6144 nhieu Gauss ~4dB, 4 vong lap");

        // ---- 5. LLR vào bị ngắt quãng (in_valid = 0 xen kẽ, kèm dữ liệu rác) ----
        full_test(256, 6, 1, 16, 8, 0, 35, 1, "K=256 lat 8 bit, LLR vao bi ngat quang 35%");

        // ---- 6. lợi ích của việc tăng số vòng lặp: cùng một khối nhiễu nặng, 1 vòng so với 8 vòng ----
        encode_block(1024);
        build_llr(1024, 2, 10, 0, 9);                      // nhiễu nặng hơn mục 4 (~1 dB Eb/N0): 1 vòng còn lỗi, 8 vòng sửa hết (đã thử 8 hạt giống)
        decode_block(1024, 1, 0); e1 = last_errs;
        decode_block(1024, 8, 0); e8 = last_errs;
        $display("[INFO] K=1024 nhieu nang: 1 vong lap -> %0d bit sai ; 8 vong lap -> %0d bit sai", e1, e8);
        check(e8 < e1, "tang vong lap lam giam so bit sai");
        check(e8 == 0, "K=1024 nhieu nang: 8 vong lap sua het loi");

        // ---- 6b. chỉ có LLR bit đuôi mới cứu được 3 bit cuối (ca thử XÁC ĐỊNH, không phụ thuộc may rủi của nhiễu) ----
        // which = 0 thử đường đuôi của SISO1 ; which = 1 thử đường đuôi của SISO2. Ba mẫu bit khác nhau để mọi vị trí đều bị thử.
        tail_only_test(0, 3'b101, "duoi SISO1 cuu 3 bit cuoi, mau 101");
        tail_only_test(0, 3'b110, "duoi SISO1 cuu 3 bit cuoi, mau 110");
        tail_only_test(0, 3'b011, "duoi SISO1 cuu 3 bit cuoi, mau 011");
        tail_only_test(1, 3'b101, "duoi SISO2 cuu 3 bit cuoi, mau 101");
        tail_only_test(1, 3'b110, "duoi SISO2 cuu 3 bit cuoi, mau 110");
        tail_only_test(1, 3'b011, "duoi SISO2 cuu 3 bit cuoi, mau 011");

        // ---- 7. K sai ----
        invalid_k_test(41);
        invalid_k_test(0);
        invalid_k_test(6145);
        full_test(56, 3, 0, 20, 0, 0, 0, 1, "giai ma binh thuong sau K sai");

        // ---- 8. reset giữa chừng ----
        reset_mid_test(0, 104);
        reset_mid_test(1, 104);

        // ---- 9. các khối nối đuôi nhau với K khác nhau (kiểm tra xóa sạch bộ nhớ thông tin ngoại) ----
        full_test(200, 5, 1, 16, 5, 0, 0, 1, "khoi noi duoi #1 K=200");
        full_test(40,  5, 1, 16, 2, 0, 0, 1, "khoi noi duoi #2 K=40");
        full_test(136, 5, 1, 16, 4, 0, 0, 1, "khoi noi duoi #3 K=136");

        // ---- 10. quét toàn bộ 188 kích thước K (tùy chọn) ----
        if (FULL_SWEEP != 0) begin
            $display("--- quet toan bo 188 kich thuoc K (khong nhieu, 1 vong lap) ---");
            for (ks = 40; ks <= 512; ks = ks + 8)    begin encode_block(ks); build_llr(ks,0,20,0,0); decode_block(ks,1,0); check(last_errs == 0 && dec_cnt == ks && idx_err == 0, "quet K"); end
            for (ks = 528; ks <= 1024; ks = ks + 16) begin encode_block(ks); build_llr(ks,0,20,0,0); decode_block(ks,1,0); check(last_errs == 0 && dec_cnt == ks && idx_err == 0, "quet K"); end
            for (ks = 1056; ks <= 2048; ks = ks + 32) begin encode_block(ks); build_llr(ks,0,20,0,0); decode_block(ks,1,0); check(last_errs == 0 && dec_cnt == ks && idx_err == 0, "quet K"); end
            for (ks = 2112; ks <= 6144; ks = ks + 64) begin encode_block(ks); build_llr(ks,0,20,0,0); decode_block(ks,1,0); check(last_errs == 0 && dec_cnt == ks && idx_err == 0, "quet K"); end
        end

        repeat (10) @(negedge clk);
        check(x_errors == 0, "khong co X/Z tren ngo ra trong toan bo mo phong");
        check(spurious == 0, "khong co bit ra thua ngoai khoi");
        $display("=== Tong ket: %0d ca thu, %0d loi, %0d lan X/Z ===", total_checks, total_fail, x_errors);
        if (total_fail == 0 && x_errors == 0) $display("TEST PASSED");
        else                                  $display("TEST FAILED");
        $finish;                           // kết thúc mô phỏng
    end

    initial begin                          // chốt chặn: tránh mô phỏng treo vô hạn
        #2_000_000_000;                    // 2 giây thời gian mô phỏng (200 triệu nhịp)
        $display("TEST FAILED (het thoi gian cho phep)");
        $finish;
    end

endmodule
