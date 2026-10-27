`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : tb_rsc_encoder.v
// Chức năng : Testbench tự kiểm tra cho rsc_encoder (bộ mã hóa thành phần 8 trạng thái).
//   1) Trạng thái ban đầu = 000.
//   2) Đáp ứng xung: đầu vào 1,0,0,0... phải cho parity 1 1 1 1 0 0 1 0 1 1 1 0 0 1 0 ...
//      (chuỗi này tính tay từ g1(D)/g0(D), có chu kỳ 7 vì g0 là đa thức nguyên thủy).
//   3) en = 0 giữ nguyên trạng thái; clr đưa về 000.
//   4) 2000 bit ngẫu nhiên so với mô hình phương trình a(n) = x(n)^a(n-2)^a(n-3).
//   5) Kết thúc trellis từ cả 8 trạng thái: x,z đúng công thức đóng và trạng thái về 000 sau 3 nhịp.
//   6) Kết thúc trellis sau các chuỗi ngẫu nhiên có độ dài khác nhau.
//==============================================================================
module tb_rsc_encoder;

    reg        clk;                    // xung nhịp
    reg        rst;                    // reset đồng bộ
    reg        clr;                    // xóa trạng thái
    reg        en;                     // cho phép dịch
    reg        term;                   // chế độ kết thúc trellis
    reg        din;                    // bit vào
    wire       x_out;                  // bit thực sự vào bộ mã
    wire       z_out;                  // parity
    wire [2:0] state;                  // {s0,s1,s2}
    wire       state_zero;             // cờ trạng thái không

    rsc_encoder dut (                  // khối cần kiểm tra
        .clk(clk), .rst(rst), .clr(clr), .en(en), .term(term), .din(din),
        .x_out(x_out), .z_out(z_out), .state(state), .state_zero(state_zero)
    );

    initial clk = 1'b0;                // xung nhịp bắt đầu ở mức thấp
    always #5 clk = ~clk;              // chu kỳ 10 ns

    integer errors;                    // tổng số lỗi
    integer n_tests;                   // số ca thử
    integer i, j, n, len;              // biến lặp
    reg [2:0] m_state;                 // trạng thái của mô hình tham chiếu {a(n-1), a(n-2), a(n-3)}
    reg [14:0] exp_imp;                // parity đáp ứng xung mong đợi, bit i = z(i)
    reg [14:0] got_imp;                // parity thu được
    reg        m_a, m_z, m_x;          // biến trung gian của mô hình
    reg        ex_x [0:2];             // x mong đợi ở 3 nhịp kết thúc
    reg        ex_z [0:2];             // z mong đợi ở 3 nhịp kết thúc
    reg [2:0]  st_before;              // trạng thái ngay trước khi kết thúc

    // Một nhịp mã hóa: đặt đầu vào ở sườn xuống, chờ tổ hợp ổn định, so sánh x,z với mô hình, rồi để sườn lên cập nhật.
    task step;
        input d;                       // bit vào
        input t;                       // 1 = kết thúc trellis
        begin
            @(negedge clk);            // đặt đầu vào ở sườn xuống để tránh đua với sườn lên
            din = d; term = t; en = 1'b1; // cấp bit, chế độ và cho phép dịch
            #1;                        // chờ logic tổ hợp ổn định
            m_x = t ? (m_state[1] ^ m_state[0]) : d;       // mô hình: x = fb (a(n-2)^a(n-3)) khi kết thúc, ngược lại = d
            m_a = m_x ^ m_state[1] ^ m_state[0];           // a(n) = x ^ a(n-2) ^ a(n-3)
            m_z = m_a ^ m_state[2] ^ m_state[0];           // z(n) = a(n) ^ a(n-1) ^ a(n-3)
            if (x_out !== m_x || z_out !== m_z) begin      // so sánh ngõ ra tổ hợp
                errors = errors + 1;
                $display("[ERR] t=%0t : x/z DUT=%b%b mo hinh=%b%b (state=%b term=%b)", $time, x_out, z_out, m_x, m_z, state, t);
            end
            m_state = {m_a, m_state[2], m_state[1]};       // mô hình dịch: {a(n-1),a(n-2),a(n-3)} <= {a, a(n-1), a(n-2)}
        end
    endtask

    task check_state;                  // so sánh trạng thái DUT với mô hình (gọi sau một sườn lên)
        input [8*20-1:0] tag;          // nhãn hiển thị
        begin
            if (state !== m_state) begin
                errors = errors + 1;
                $display("[ERR] %0s : state DUT=%b mo hinh=%b", tag, state, m_state);
            end
        end
    endtask

    task do_clear;                     // đưa DUT và mô hình về trạng thái 000 bằng clr
        begin
            @(negedge clk); clr = 1'b1; en = 1'b0; term = 1'b0; din = 1'b0; // đặt clr
            @(negedge clk); clr = 1'b0;                                     // hạ clr sau 1 nhịp
            m_state = 3'b000;                                               // mô hình về 0
        end
    endtask

    initial begin
        errors = 0; n_tests = 0; m_state = 3'b000;     // khởi tạo biến
        rst = 1'b1; clr = 1'b0; en = 1'b0; term = 1'b0; din = 1'b0; // giữ reset từ đầu
        repeat (3) @(negedge clk);     // giữ reset vài nhịp
        rst = 1'b0;                    // nhả reset
        $display("=== TB rsc_encoder: bat dau ===");

        // ---- 1) trạng thái ban đầu ----
        @(negedge clk);
        n_tests = n_tests + 1;
        if (state === 3'b000 && state_zero === 1'b1) $display("[PASS] trang thai ban dau = 000");
        else begin errors = errors + 1; $display("[FAIL] trang thai ban dau sai: %b", state); end

        // ---- 2) đáp ứng xung ----
        exp_imp = 15'b010011101001111; // z(14..0) = 0 1 0 0 1 1 1 0 1 0 0 1 1 1 1 -> z(0..14) = 1 1 1 1 0 0 1 0 1 1 1 0 0 1 0
        got_imp = 15'd0;               // xóa kết quả
        for (i = 0; i < 15; i = i + 1) begin
            step((i == 0) ? 1'b1 : 1'b0, 1'b0); // bit đầu = 1, các bit sau = 0
            got_imp[i] = z_out;        // thu parity z(i) (đã ổn định trước sườn lên)
            @(posedge clk); #1;        // chờ sườn lên cập nhật trạng thái
            check_state("dap ung xung");
        end
        n_tests = n_tests + 1;
        if (got_imp === exp_imp) $display("[PASS] dap ung xung khop chuoi tinh tay (z = 111100101110010)");
        else begin errors = errors + 1; $display("[FAIL] dap ung xung: DUT=%b mong doi=%b", got_imp, exp_imp); end

        // ---- 3) en = 0 giữ nguyên trạng thái; clr xóa về 000 ----
        @(negedge clk); en = 1'b0; din = 1'b1;     // tắt cho phép dịch, vẫn cấp bit 1
        st_before = state;                         // nhớ trạng thái hiện tại
        repeat (5) @(negedge clk);                 // chờ 5 nhịp
        n_tests = n_tests + 1;
        if (state === st_before) $display("[PASS] en=0 giu nguyen trang thai (%b)", state);
        else begin errors = errors + 1; $display("[FAIL] en=0 nhung trang thai doi %b -> %b", st_before, state); end
        do_clear;                                  // xóa bằng clr
        n_tests = n_tests + 1;
        if (state === 3'b000 && state_zero === 1'b1) $display("[PASS] clr dua trang thai ve 000");
        else begin errors = errors + 1; $display("[FAIL] clr khong xoa duoc: %b", state); end

        // ---- 4) 2000 bit ngẫu nhiên so với mô hình phương trình ----
        for (i = 0; i < 2000; i = i + 1) begin
            step($random, 1'b0);       // bit ngẫu nhiên
            @(posedge clk); #1;        // chờ cập nhật
            check_state("ngau nhien");
        end
        n_tests = n_tests + 1;
        $display("[INFO] 2000 bit ngau nhien da so voi mo hinh, tong loi hien tai = %0d", errors);

        // ---- 5) kết thúc trellis từ cả 8 trạng thái ----
        for (n = 0; n < 8; n = n + 1) begin
            do_clear;                                  // bắt đầu từ trạng thái 000
            step(n[0], 1'b0); @(posedge clk); #1;      // nạp 3 bit để tới trạng thái bất kỳ (cả 8 trạng thái đều đạt được)
            step(n[1], 1'b0); @(posedge clk); #1;
            step(n[2], 1'b0); @(posedge clk); #1;
            st_before = state;                         // trạng thái trước khi kết thúc: {s0,s1,s2}
            // công thức đóng suy ra từ cấu trúc: x0=s1^s2, z0=s0^s2, x1=s0^s1, z1=s1, x2=s0, z2=s0
            ex_x[0] = st_before[1] ^ st_before[0]; ex_z[0] = st_before[2] ^ st_before[0];
            ex_x[1] = st_before[2] ^ st_before[1]; ex_z[1] = st_before[1];
            ex_x[2] = st_before[2];                ex_z[2] = st_before[2];
            for (j = 0; j < 3; j = j + 1) begin
                step(1'b1, 1'b1);                      // din = 1 bị bỏ qua khi term = 1
                if (x_out !== ex_x[j] || z_out !== ex_z[j]) begin
                    errors = errors + 1;
                    $display("[ERR] ket thuc tu %b nhip %0d: x/z=%b%b, cong thuc dong=%b%b", st_before, j, x_out, z_out, ex_x[j], ex_z[j]);
                end
                @(posedge clk); #1;
            end
            n_tests = n_tests + 1;
            if (state === 3'b000 && state_zero === 1'b1)
                $display("[PASS] ket thuc trellis tu trang thai %b ve 000 sau 3 nhip", st_before);
            else begin errors = errors + 1; $display("[FAIL] tu %b khong ve 000: %b", st_before, state); end
        end

        // ---- 6) kết thúc sau các chuỗi ngẫu nhiên độ dài 1..60 ----
        for (n = 1; n <= 60; n = n + 1) begin
            do_clear;                                  // bắt đầu từ 000
            for (i = 0; i < n; i = i + 1) begin step($random, 1'b0); @(posedge clk); #1; end
            for (i = 0; i < 3; i = i + 1) begin step(1'b0, 1'b1); @(posedge clk); #1; end
            if (state !== 3'b000 || m_state !== 3'b000) begin
                errors = errors + 1;
                $display("[ERR] chuoi dai %0d khong ket thuc ve 000: DUT=%b mo hinh=%b", n, state, m_state);
            end
        end
        n_tests = n_tests + 1;
        $display("[INFO] 60 chuoi ngau nhien da kiem tra ket thuc trellis, tong loi hien tai = %0d", errors);

        // ---- 7) reset đồng bộ xóa trạng thái đang có ----
        @(negedge clk); en = 1'b1; term = 1'b0; din = 1'b1; // nạp vài bit 1
        repeat (4) @(negedge clk);
        rst = 1'b1; en = 1'b0;                              // reset 1 nhịp, đồng thời tắt cho phép dịch
        @(negedge clk); rst = 1'b0;                         // nhả reset
        @(negedge clk);
        n_tests = n_tests + 1;
        if (state === 3'b000) $display("[PASS] reset dong bo xoa trang thai ve 000");
        else begin errors = errors + 1; $display("[FAIL] reset khong xoa trang thai: %b", state); end

        $display("=== Tong ket: %0d ca thu, %0d loi ===", n_tests, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $finish;                       // kết thúc mô phỏng
    end

    initial begin                      // bộ canh giờ phòng treo
        #10_000_000;
        $display("TEST FAILED (TIMEOUT)");
        $finish;
    end

endmodule
