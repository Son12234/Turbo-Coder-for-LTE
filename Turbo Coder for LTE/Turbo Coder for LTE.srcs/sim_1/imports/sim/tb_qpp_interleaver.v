`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : tb_qpp_interleaver.v
// Chức năng : Testbench tự kiểm tra cho qpp_param_rom + qpp_interleaver.
//   1) Quét mọi K từ 0 đến 8191: cờ valid của ROM phải bằng 1 đúng với 188 kích thước hợp lệ
//      (40..512 bước 8, 528..1024 bước 16, 1056..2048 bước 32, 2112..6144 bước 64).
//   2) Với từng K hợp lệ: f1, f2 của ROM phải trùng bảng riêng của testbench (và f1,f2 < K, f2 chẵn).
//   3) Với từng K hợp lệ: chuỗi địa chỉ do bộ sinh đệ quy tạo ra phải đúng bằng
//      Pi(i) = (f1*i + f2*i^2) mod K  tính trực tiếp bằng số học 64 bit, ĐÚNG TỪNG PHẦN TỬ.
//   4) Chuỗi đó phải là một hoán vị: mỗi địa chỉ 0..K-1 xuất hiện đúng một lần.
//   5) Sau K bước địa chỉ quay về 0 (tính tuần hoàn), giữ nguyên khi step = 0, và init nạp lại được giữa chừng.
//==============================================================================
module tb_qpp_interleaver;

    reg         clk;                   // xung nhịp
    reg         rst;                   // reset đồng bộ
    reg         init;                  // xung nạp tham số
    reg         step;                  // tiến một chỉ số
    reg  [12:0] k;                     // K đang thử
    wire [12:0] f1;                    // f1 do ROM trả về
    wire [12:0] f2;                    // f2 do ROM trả về
    wire        valid;                 // K hợp lệ
    wire [12:0] addr;                  // địa chỉ đã xen do DUT sinh ra

    qpp_param_rom u_rom (.k(k), .f1(f1), .f2(f2), .valid(valid));      // ROM tham số
    qpp_interleaver dut (                                              // bộ sinh địa chỉ cần kiểm tra
        .clk(clk), .rst(rst), .init(init), .step(step),
        .k(k), .f1(f1), .f2(f2), .addr(addr)
    );

    initial clk = 1'b0;                // xung nhịp bắt đầu ở mức thấp
    always #5 clk = ~clk;              // chu kỳ 10 ns

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

    reg seen [0:6143];                 // bản đồ các địa chỉ đã xuất hiện (kiểm tra hoán vị)
    integer errors;                    // tổng số lỗi
    integer n_valid;                   // số K hợp lệ đã thấy trong ROM
    integer n_k_checked;               // số K đã kiểm tra chuỗi địa chỉ
    integer kk, i, c;                  // biến lặp
    reg     exp_valid;                 // giá trị valid mong đợi
    reg [25:0] pr;                     // {f1,f2} từ bảng của testbench
    reg [63:0] k64, f1_64, f2_64, i64, p64; // số học 64 bit
    integer    k_err;                  // số lỗi của K hiện tại

    // Vị từ: K có nằm trong 188 giá trị hợp lệ của LTE không
    function is_lte_k;
        input integer kv;              // K cần xét
        begin
            is_lte_k = ((kv >= 40   && kv <= 512  && ((kv - 40)   % 8)  == 0) ||
                        (kv >= 528  && kv <= 1024 && ((kv - 528)  % 16) == 0) ||
                        (kv >= 1056 && kv <= 2048 && ((kv - 1056) % 32) == 0) ||
                        (kv >= 2112 && kv <= 6144 && ((kv - 2112) % 64) == 0));
        end
    endfunction

    initial begin
        errors = 0; n_valid = 0; n_k_checked = 0;      // khởi tạo
        rst = 1'b1; init = 1'b0; step = 1'b0; k = 13'd0; // giữ reset từ đầu
        repeat (3) @(negedge clk);     // giữ reset vài nhịp
        rst = 1'b0;                    // nhả reset
        $display("=== TB qpp_interleaver: bat dau ===");

        // ---- 1) quét cờ valid của ROM cho mọi K từ 0 đến 8191 ----
        for (kk = 0; kk < 8192; kk = kk + 1) begin
            k = kk; #1;                // đặt K, chờ ROM (tổ hợp) ổn định
            exp_valid = is_lte_k(kk);  // giá trị mong đợi theo quy tắc khoảng
            if (valid !== exp_valid) begin
                errors = errors + 1;
                if (errors <= 10) $display("[ERR] K=%0d : valid ROM=%b mong doi=%b", kk, valid, exp_valid);
            end
            if (valid === 1'b1) n_valid = n_valid + 1;
            if (valid !== 1'b1 && (f1 !== 13'd0 || f2 !== 13'd0)) begin
                errors = errors + 1;   // K sai thì f1, f2 phải bằng 0
                if (errors <= 10) $display("[ERR] K=%0d khong hop le nhung f1/f2 khac 0", kk);
            end
        end
        if (n_valid == 188) $display("[PASS] ROM co dung 188 kich thuoc K hop le, cac K con lai bi tu choi");
        else begin errors = errors + 1; $display("[FAIL] ROM co %0d kich thuoc hop le (can 188)", n_valid); end

        // ---- 2) + 3) + 4) + 5) kiểm tra từng K hợp lệ ----
        for (kk = 40; kk <= 6144; kk = kk + 1) begin
            if (is_lte_k(kk)) begin
                k_err = 0;                         // chưa có lỗi cho K này
                @(negedge clk); k = kk;            // đặt K (ROM trả f1,f2 ngay)
                #1;
                pr = tb_qpp_params(kk);            // f1,f2 của bảng riêng testbench
                if (f1 !== pr[25:13] || f2 !== pr[12:0]) begin
                    k_err = k_err + 1;
                    $display("[ERR] K=%0d : ROM f1,f2=%0d,%0d ; bang TB=%0d,%0d", kk, f1, f2, pr[25:13], pr[12:0]);
                end
                if (!(f1 < kk && f2 < kk && f2[0] == 1'b0)) begin
                    k_err = k_err + 1;             // f1,f2 phải nhỏ hơn K và f2 chẵn
                    $display("[ERR] K=%0d : f1/f2 vi pham dieu kien (f1<K, f2<K, f2 chan)", kk);
                end
                for (i = 0; i < kk; i = i + 1) seen[i] = 1'b0;   // xóa bản đồ hoán vị
                k64 = kk; f1_64 = pr[25:13]; f2_64 = pr[12:0];   // số học 64 bit
                @(negedge clk); init = 1'b1;       // xung nạp tham số
                @(negedge clk); init = 1'b0;       // hạ init: addr = Pi(0) = 0
                step = 1'b1;                       // giữ step = 1 để mỗi nhịp tiến một chỉ số
                for (i = 0; i < kk; i = i + 1) begin
                    i64 = i;
                    p64 = (f1_64 * i64 + f2_64 * i64 * i64) % k64;  // công thức QPP trực tiếp
                    if (addr !== p64[12:0]) begin  // so từng phần tử
                        k_err = k_err + 1;
                        if (k_err <= 5) $display("[ERR] K=%0d i=%0d : addr DUT=%0d  cong thuc=%0d", kk, i, addr, p64[12:0]);
                    end
                    if (addr < kk) begin           // đánh dấu địa chỉ đã gặp
                        if (seen[addr] === 1'b1) begin k_err = k_err + 1; if (k_err <= 5) $display("[ERR] K=%0d : dia chi %0d lap lai", kk, addr); end
                        seen[addr] = 1'b1;
                    end else begin
                        k_err = k_err + 1;
                        if (k_err <= 5) $display("[ERR] K=%0d : dia chi %0d >= K", kk, addr);
                    end
                    @(negedge clk);                // sang chỉ số kế tiếp
                end
                step = 1'b0;                       // dừng
                if (addr !== 13'd0) begin          // sau K bước phải quay về Pi(K) mod K = 0
                    k_err = k_err + 1;
                    $display("[ERR] K=%0d : sau K buoc addr=%0d (can 0)", kk, addr);
                end
                c = 0;                             // đếm số địa chỉ đã gặp
                for (i = 0; i < kk; i = i + 1) if (seen[i] === 1'b1) c = c + 1;
                if (c != kk) begin k_err = k_err + 1; $display("[ERR] K=%0d : chi co %0d/%0d dia chi khac nhau", kk, c, kk); end
                errors = errors + k_err;           // cộng dồn
                n_k_checked = n_k_checked + 1;     // thêm một K đã kiểm tra
            end
        end
        if (n_k_checked == 188) $display("[PASS] 188 chuoi dia chi QPP khop cong thuc tung phan tu va la hoan vi day du");
        else begin errors = errors + 1; $display("[FAIL] chi kiem tra %0d/188 kich thuoc", n_k_checked); end

        // ---- 5) giữ nguyên khi step = 0; init giữa chừng nạp lại từ đầu ----
        @(negedge clk); k = 13'd40; #1;            // K = 40 : f1=3, f2=10
        @(negedge clk); init = 1'b1;
        @(negedge clk); init = 1'b0; step = 1'b1;
        repeat (7) @(negedge clk);                 // tiến 7 bước
        step = 1'b0;
        c = addr;                                  // nhớ địa chỉ
        repeat (5) @(negedge clk);                 // 5 nhịp không step
        if (addr !== c[12:0]) begin errors = errors + 1; $display("[FAIL] addr doi khi step = 0"); end
        else $display("[PASS] step = 0 giu nguyen dia chi (Pi(7) = %0d)", addr);
        init = 1'b1; @(negedge clk); init = 1'b0;  // init giữa chừng
        if (addr !== 13'd0) begin errors = errors + 1; $display("[FAIL] init giua chung khong dua addr ve 0"); end
        else $display("[PASS] init giua chung nap lai Pi(0) = 0");

        $display("=== Tong ket: %0d loi ===", errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $finish;                       // kết thúc mô phỏng
    end

    initial begin                      // bộ canh giờ phòng treo
        #500_000_000;
        $display("TEST FAILED (TIMEOUT)");
        $finish;
    end

endmodule
