`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : tb_turbo_demo_top.v
// Chức năng : Testbench tự kiểm tra cho mạch demo turbo_demo_top (LFSR -> bộ mã hóa -> MISR -> LED).
//
// Cách kiểm tra:
//   * "Bấm nút" start / reset bằng cách giữ tín hiệu đủ lâu hơn thời gian chống rung (đặt nhỏ để chạy nhanh).
//   * Với 8 giá trị K của công tắc sw[2:0], chữ ký MISR trên led[15:4] phải bằng giá trị chuẩn được tính
//     bằng một mô hình Python độc lập (LFSR -> mã hóa Turbo theo TS 36.212 -> MISR).
//   * Lặp lại cùng K phải cho cùng chữ ký; nhấn start khi đang bận phải bị bỏ qua;
//     nhấn reset giữa chừng phải đưa mạch về trạng thái nghỉ rồi chạy lại bình thường.
//   * Mỗi nhịp đều kiểm tra led không có X/Z.
// Kết quả cuối: in "TEST PASSED" hoặc "TEST FAILED" rồi gọi $finish.
//==============================================================================
module tb_turbo_demo_top;

    localparam integer DB = 8;         // thời gian chống rung dùng trong mô phỏng (nhịp clk)

    reg         clk;                   // xung nhịp 100 MHz
    reg         btn_rst;               // nút reset
    reg         btn_start;             // nút start
    reg  [2:0]  sw;                    // công tắc chọn K
    wire [15:0] led;                   // LED ra

    turbo_demo_top #(.DEBOUNCE_CYCLES(DB)) dut (   // khối cần kiểm tra
        .clk(clk), .btn_rst(btn_rst), .btn_start(btn_start), .sw(sw), .led(led)
    );

    initial clk = 1'b0;                // xung nhịp bắt đầu ở mức thấp
    always #5 clk = ~clk;              // chu kỳ 10 ns => 100 MHz

    // Tín hiệu quan sát cho dễ đọc
    wire        led_busy = led[0];     // đang bận
    wire        led_done = led[1];     // đã xong
    wire        led_err  = led[2];     // lỗi K
    wire        led_term = led[3];     // trellis kết thúc đúng
    wire [11:0] led_sig  = led[15:4];  // chữ ký MISR

    // Chữ ký chuẩn theo sw[2:0] (tính bằng mô hình Python độc lập)
    function [11:0] expected_sig;
        input [2:0] sel;               // giá trị công tắc
        begin
            case (sel)
                3'd0:    expected_sig = 12'h371; // K = 40
                3'd1:    expected_sig = 12'hE3D; // K = 104
                3'd2:    expected_sig = 12'h16E; // K = 256
                3'd3:    expected_sig = 12'hCA2; // K = 512
                3'd4:    expected_sig = 12'hEC8; // K = 1024
                3'd5:    expected_sig = 12'h05E; // K = 2048
                3'd6:    expected_sig = 12'hDD8; // K = 4096
                default: expected_sig = 12'h8B3; // K = 6144
            endcase
        end
    endfunction

    integer errors;                    // tổng số lỗi
    integer n_tests;                   // số ca thử
    integer x_errors;                  // số lần thấy X/Z trên led
    integer ci;                        // biến lặp chính
    integer guard;                     // bộ đếm chống treo

    // Bộ giám sát X/Z chạy mỗi nhịp
    always @(posedge clk) begin
        if ((^led) === 1'bx) begin     // có bit nào của led là X hoặc Z
            x_errors = x_errors + 1;
            if (x_errors <= 5) $display("[ERR] t=%0t : X/Z tren led", $time);
        end
    end

    // "Nhấn" nút start trong một số nhịp (gọi tại sườn xuống clk)
    task press_start;
        input integer cycles;          // thời gian giữ nút (nhịp)
        begin
            btn_start = 1'b1;          // nhấn
            repeat (cycles) @(negedge clk);
            btn_start = 1'b0;          // nhả
        end
    endtask

    // Chờ led_done = 1 (có chặn treo); trả về 1 nếu thành công
    task wait_done;
        output ok;                     // 1: thấy done trong thời hạn
        begin
            guard = 0;                 // bắt đầu đếm
            while (!led_done && guard < 200000) begin
                @(negedge clk);        // chờ một nhịp
                guard = guard + 1;
            end
            ok = led_done;             // kết quả
        end
    endtask

    // Chạy một khối với công tắc sel và so chữ ký
    task run_and_check;
        input [2:0] sel;               // giá trị công tắc
        input [8*16-1:0] name;         // nhãn ca thử
        reg ok;                        // kết quả chờ
        begin
            sw = sel;                  // đặt công tắc
            repeat (4) @(negedge clk); // chờ bộ đồng bộ công tắc ổn định
            press_start(DB + 6);       // nhấn start lâu hơn thời gian chống rung
            wait_done(ok);             // chờ xong
            @(negedge clk);            // sang nhịp sau để mọi LED ổn định
            n_tests = n_tests + 1;
            if (!ok) begin
                errors = errors + 1;
                $display("[FAIL] %0s sw=%0d : het gio cho done", name, sel);
            end else if (led_sig !== expected_sig(sel) || led_term !== 1'b1 || led_err !== 1'b0 || led_busy !== 1'b0) begin
                errors = errors + 1;
                $display("[FAIL] %0s sw=%0d : sig=%03h (chuan %03h) term=%b err=%b busy=%b",
                         name, sel, led_sig, expected_sig(sel), led_term, led_err, led_busy);
            end else
                $display("[PASS] %0s sw=%0d : chu ky = %03h, term_ok=1, khong loi", name, sel, led_sig);
            repeat (3) @(negedge clk); // nghỉ vài nhịp trước ca sau
        end
    endtask

    reg ok_w;                          // biến phụ cho wait_done

    initial begin
        btn_rst = 1'b0; btn_start = 1'b0; sw = 3'd0;  // trạng thái nghỉ của nút và công tắc
        errors = 0; n_tests = 0; x_errors = 0;         // xóa bộ đếm
        repeat (30) @(negedge clk);                    // chờ qua giai đoạn reset khi cấp nguồn (16 nhịp)
        $display("=== TB turbo_demo_top: bat dau ===");

        // ---- 1) trạng thái sau khi cấp nguồn ----
        n_tests = n_tests + 1;
        if (led === 16'h0000) $display("[PASS] sau cap nguon: tat ca led = 0");
        else begin errors = errors + 1; $display("[FAIL] sau cap nguon led = %04h", led); end

        // ---- 2) tám giá trị K của công tắc ----
        for (ci = 0; ci < 8; ci = ci + 1) run_and_check(ci[2:0], "chu ky K");

        // ---- 3) lặp lại cùng K phải cho cùng chữ ký ----
        run_and_check(3'd0, "lap lai K=40");
        run_and_check(3'd4, "lap lai K=1024");

        // ---- 4) nhấn start khi đang bận phải bị bỏ qua (chữ ký vẫn đúng) ----
        sw = 3'd4;                                     // K = 1024
        repeat (4) @(negedge clk);
        press_start(DB + 6);                           // bắt đầu khối
        repeat (300) @(negedge clk);                   // chạy được một lúc
        press_start(DB + 6);                           // nhấn start lần nữa khi đang bận
        wait_done(ok_w);
        @(negedge clk);
        n_tests = n_tests + 1;
        if (ok_w && led_sig === expected_sig(3'd4) && led_term === 1'b1)
            $display("[PASS] nhan start khi dang ban bi bo qua, chu ky van dung (%03h)", led_sig);
        else begin errors = errors + 1; $display("[FAIL] start khi dang ban: done=%b sig=%03h", ok_w, led_sig); end
        repeat (3) @(negedge clk);

        // ---- 5) reset giữa chừng rồi chạy lại ----
        sw = 3'd7;                                     // K = 6144
        repeat (4) @(negedge clk);
        press_start(DB + 6);                           // bắt đầu khối lớn
        repeat (500) @(negedge clk);                   // đang ở giữa pha nhận bit
        btn_rst = 1'b1; repeat (DB + 6) @(negedge clk); btn_rst = 1'b0; // nhấn reset
        repeat (6) @(negedge clk);
        n_tests = n_tests + 1;
        if (led === 16'h0000) $display("[PASS] reset giua chung: mach ve trang thai nghi, led = 0");
        else begin errors = errors + 1; $display("[FAIL] sau reset giua chung led = %04h", led); end
        run_and_check(3'd1, "sau reset K=104");

        // ---- tổng kết ----
        n_tests = n_tests + 1;
        if (x_errors == 0) $display("[PASS] khong co X/Z tren led trong toan bo mo phong");
        else begin errors = errors + 1; $display("[FAIL] co %0d lan X/Z tren led", x_errors); end
        $display("=== Tong ket: %0d ca thu, %0d loi, %0d lan X/Z ===", n_tests, errors, x_errors);
        if (errors == 0 && x_errors == 0) $display("TEST PASSED");
        else                              $display("TEST FAILED");
        $finish;                                       // kết thúc mô phỏng
    end

    initial begin                      // bộ canh giờ toàn cục phòng khi mô phỏng bị treo
        #200_000_000;                  // 0,2 giây thời gian mô phỏng
        $display("TEST FAILED (TIMEOUT)");
        $finish;
    end

endmodule
