`timescale 1ns / 1ps                                   // đơn vị thời gian mô phỏng: 1 ns, độ phân giải 1 ps
//==============================================================================
// File   : qpp_param_rom.v
// Chức năng : ROM tổ hợp tra tham số bộ xen QPP (Quadratic Permutation Polynomial)
//             theo 3GPP TS 36.212 Bảng 5.1.3-3 (188 kích thước khối K từ 40 đến 6144).
//
// Bộ xen nội của Turbo code LTE:  Pi(i) = (f1*i + f2*i^2) mod K ,  i = 0..K-1
//   - Ngõ vào k    : kích thước khối K (số bit thông tin)
//   - Ngõ ra f1,f2 : hai hệ số của đa thức bậc hai tương ứng với K
//   - Ngõ ra valid : =1 nếu K nằm trong bảng, =0 nếu K không hợp lệ (khi đó f1=f2=0)
// Bảng được sinh tự động từ dữ liệu chuẩn và đã được kiểm tra: mọi dòng đều là một hoán vị đầy đủ của 0..K-1.
//==============================================================================
module qpp_param_rom (
    input  wire [12:0] k,      // kích thước khối K cần tra (13 bit đủ cho tối đa 6144)
    output reg  [12:0] f1,     // hệ số bậc 1 của bộ xen QPP
    output reg  [12:0] f2,     // hệ số bậc 2 của bộ xen QPP
    output reg         valid   // cờ báo K hợp lệ
);

    always @(*) begin          // khối tổ hợp: mọi ngõ ra được gán trong mọi nhánh nên không tạo latch
        f1    = 13'd0;         // giá trị mặc định của f1
        f2    = 13'd0;         // giá trị mặc định của f2
        valid = 1'b0;          // mặc định: K không hợp lệ
        case (k)               // tra bảng theo K
            13'd40   : begin f1 = 13'd3  ; f2 = 13'd10 ; valid = 1'b1; end // i=1   K=40
            13'd48   : begin f1 = 13'd7  ; f2 = 13'd12 ; valid = 1'b1; end // i=2   K=48
            13'd56   : begin f1 = 13'd19 ; f2 = 13'd42 ; valid = 1'b1; end // i=3   K=56
            13'd64   : begin f1 = 13'd7  ; f2 = 13'd16 ; valid = 1'b1; end // i=4   K=64
            13'd72   : begin f1 = 13'd7  ; f2 = 13'd18 ; valid = 1'b1; end // i=5   K=72
            13'd80   : begin f1 = 13'd11 ; f2 = 13'd20 ; valid = 1'b1; end // i=6   K=80
            13'd88   : begin f1 = 13'd5  ; f2 = 13'd22 ; valid = 1'b1; end // i=7   K=88
            13'd96   : begin f1 = 13'd11 ; f2 = 13'd24 ; valid = 1'b1; end // i=8   K=96
            13'd104  : begin f1 = 13'd7  ; f2 = 13'd26 ; valid = 1'b1; end // i=9   K=104
            13'd112  : begin f1 = 13'd41 ; f2 = 13'd84 ; valid = 1'b1; end // i=10  K=112
            13'd120  : begin f1 = 13'd103; f2 = 13'd90 ; valid = 1'b1; end // i=11  K=120
            13'd128  : begin f1 = 13'd15 ; f2 = 13'd32 ; valid = 1'b1; end // i=12  K=128
            13'd136  : begin f1 = 13'd9  ; f2 = 13'd34 ; valid = 1'b1; end // i=13  K=136
            13'd144  : begin f1 = 13'd17 ; f2 = 13'd108; valid = 1'b1; end // i=14  K=144
            13'd152  : begin f1 = 13'd9  ; f2 = 13'd38 ; valid = 1'b1; end // i=15  K=152
            13'd160  : begin f1 = 13'd21 ; f2 = 13'd120; valid = 1'b1; end // i=16  K=160
            13'd168  : begin f1 = 13'd101; f2 = 13'd84 ; valid = 1'b1; end // i=17  K=168
            13'd176  : begin f1 = 13'd21 ; f2 = 13'd44 ; valid = 1'b1; end // i=18  K=176
            13'd184  : begin f1 = 13'd57 ; f2 = 13'd46 ; valid = 1'b1; end // i=19  K=184
            13'd192  : begin f1 = 13'd23 ; f2 = 13'd48 ; valid = 1'b1; end // i=20  K=192
            13'd200  : begin f1 = 13'd13 ; f2 = 13'd50 ; valid = 1'b1; end // i=21  K=200
            13'd208  : begin f1 = 13'd27 ; f2 = 13'd52 ; valid = 1'b1; end // i=22  K=208
            13'd216  : begin f1 = 13'd11 ; f2 = 13'd36 ; valid = 1'b1; end // i=23  K=216
            13'd224  : begin f1 = 13'd27 ; f2 = 13'd56 ; valid = 1'b1; end // i=24  K=224
            13'd232  : begin f1 = 13'd85 ; f2 = 13'd58 ; valid = 1'b1; end // i=25  K=232
            13'd240  : begin f1 = 13'd29 ; f2 = 13'd60 ; valid = 1'b1; end // i=26  K=240
            13'd248  : begin f1 = 13'd33 ; f2 = 13'd62 ; valid = 1'b1; end // i=27  K=248
            13'd256  : begin f1 = 13'd15 ; f2 = 13'd32 ; valid = 1'b1; end // i=28  K=256
            13'd264  : begin f1 = 13'd17 ; f2 = 13'd198; valid = 1'b1; end // i=29  K=264
            13'd272  : begin f1 = 13'd33 ; f2 = 13'd68 ; valid = 1'b1; end // i=30  K=272
            13'd280  : begin f1 = 13'd103; f2 = 13'd210; valid = 1'b1; end // i=31  K=280
            13'd288  : begin f1 = 13'd19 ; f2 = 13'd36 ; valid = 1'b1; end // i=32  K=288
            13'd296  : begin f1 = 13'd19 ; f2 = 13'd74 ; valid = 1'b1; end // i=33  K=296
            13'd304  : begin f1 = 13'd37 ; f2 = 13'd76 ; valid = 1'b1; end // i=34  K=304
            13'd312  : begin f1 = 13'd19 ; f2 = 13'd78 ; valid = 1'b1; end // i=35  K=312
            13'd320  : begin f1 = 13'd21 ; f2 = 13'd120; valid = 1'b1; end // i=36  K=320
            13'd328  : begin f1 = 13'd21 ; f2 = 13'd82 ; valid = 1'b1; end // i=37  K=328
            13'd336  : begin f1 = 13'd115; f2 = 13'd84 ; valid = 1'b1; end // i=38  K=336
            13'd344  : begin f1 = 13'd193; f2 = 13'd86 ; valid = 1'b1; end // i=39  K=344
            13'd352  : begin f1 = 13'd21 ; f2 = 13'd44 ; valid = 1'b1; end // i=40  K=352
            13'd360  : begin f1 = 13'd133; f2 = 13'd90 ; valid = 1'b1; end // i=41  K=360
            13'd368  : begin f1 = 13'd81 ; f2 = 13'd46 ; valid = 1'b1; end // i=42  K=368
            13'd376  : begin f1 = 13'd45 ; f2 = 13'd94 ; valid = 1'b1; end // i=43  K=376
            13'd384  : begin f1 = 13'd23 ; f2 = 13'd48 ; valid = 1'b1; end // i=44  K=384
            13'd392  : begin f1 = 13'd243; f2 = 13'd98 ; valid = 1'b1; end // i=45  K=392
            13'd400  : begin f1 = 13'd151; f2 = 13'd40 ; valid = 1'b1; end // i=46  K=400
            13'd408  : begin f1 = 13'd155; f2 = 13'd102; valid = 1'b1; end // i=47  K=408
            13'd416  : begin f1 = 13'd25 ; f2 = 13'd52 ; valid = 1'b1; end // i=48  K=416
            13'd424  : begin f1 = 13'd51 ; f2 = 13'd106; valid = 1'b1; end // i=49  K=424
            13'd432  : begin f1 = 13'd47 ; f2 = 13'd72 ; valid = 1'b1; end // i=50  K=432
            13'd440  : begin f1 = 13'd91 ; f2 = 13'd110; valid = 1'b1; end // i=51  K=440
            13'd448  : begin f1 = 13'd29 ; f2 = 13'd168; valid = 1'b1; end // i=52  K=448
            13'd456  : begin f1 = 13'd29 ; f2 = 13'd114; valid = 1'b1; end // i=53  K=456
            13'd464  : begin f1 = 13'd247; f2 = 13'd58 ; valid = 1'b1; end // i=54  K=464
            13'd472  : begin f1 = 13'd29 ; f2 = 13'd118; valid = 1'b1; end // i=55  K=472
            13'd480  : begin f1 = 13'd89 ; f2 = 13'd180; valid = 1'b1; end // i=56  K=480
            13'd488  : begin f1 = 13'd91 ; f2 = 13'd122; valid = 1'b1; end // i=57  K=488
            13'd496  : begin f1 = 13'd157; f2 = 13'd62 ; valid = 1'b1; end // i=58  K=496
            13'd504  : begin f1 = 13'd55 ; f2 = 13'd84 ; valid = 1'b1; end // i=59  K=504
            13'd512  : begin f1 = 13'd31 ; f2 = 13'd64 ; valid = 1'b1; end // i=60  K=512
            13'd528  : begin f1 = 13'd17 ; f2 = 13'd66 ; valid = 1'b1; end // i=61  K=528
            13'd544  : begin f1 = 13'd35 ; f2 = 13'd68 ; valid = 1'b1; end // i=62  K=544
            13'd560  : begin f1 = 13'd227; f2 = 13'd420; valid = 1'b1; end // i=63  K=560
            13'd576  : begin f1 = 13'd65 ; f2 = 13'd96 ; valid = 1'b1; end // i=64  K=576
            13'd592  : begin f1 = 13'd19 ; f2 = 13'd74 ; valid = 1'b1; end // i=65  K=592
            13'd608  : begin f1 = 13'd37 ; f2 = 13'd76 ; valid = 1'b1; end // i=66  K=608
            13'd624  : begin f1 = 13'd41 ; f2 = 13'd234; valid = 1'b1; end // i=67  K=624
            13'd640  : begin f1 = 13'd39 ; f2 = 13'd80 ; valid = 1'b1; end // i=68  K=640
            13'd656  : begin f1 = 13'd185; f2 = 13'd82 ; valid = 1'b1; end // i=69  K=656
            13'd672  : begin f1 = 13'd43 ; f2 = 13'd252; valid = 1'b1; end // i=70  K=672
            13'd688  : begin f1 = 13'd21 ; f2 = 13'd86 ; valid = 1'b1; end // i=71  K=688
            13'd704  : begin f1 = 13'd155; f2 = 13'd44 ; valid = 1'b1; end // i=72  K=704
            13'd720  : begin f1 = 13'd79 ; f2 = 13'd120; valid = 1'b1; end // i=73  K=720
            13'd736  : begin f1 = 13'd139; f2 = 13'd92 ; valid = 1'b1; end // i=74  K=736
            13'd752  : begin f1 = 13'd23 ; f2 = 13'd94 ; valid = 1'b1; end // i=75  K=752
            13'd768  : begin f1 = 13'd217; f2 = 13'd48 ; valid = 1'b1; end // i=76  K=768
            13'd784  : begin f1 = 13'd25 ; f2 = 13'd98 ; valid = 1'b1; end // i=77  K=784
            13'd800  : begin f1 = 13'd17 ; f2 = 13'd80 ; valid = 1'b1; end // i=78  K=800
            13'd816  : begin f1 = 13'd127; f2 = 13'd102; valid = 1'b1; end // i=79  K=816
            13'd832  : begin f1 = 13'd25 ; f2 = 13'd52 ; valid = 1'b1; end // i=80  K=832
            13'd848  : begin f1 = 13'd239; f2 = 13'd106; valid = 1'b1; end // i=81  K=848
            13'd864  : begin f1 = 13'd17 ; f2 = 13'd48 ; valid = 1'b1; end // i=82  K=864
            13'd880  : begin f1 = 13'd137; f2 = 13'd110; valid = 1'b1; end // i=83  K=880
            13'd896  : begin f1 = 13'd215; f2 = 13'd112; valid = 1'b1; end // i=84  K=896
            13'd912  : begin f1 = 13'd29 ; f2 = 13'd114; valid = 1'b1; end // i=85  K=912
            13'd928  : begin f1 = 13'd15 ; f2 = 13'd58 ; valid = 1'b1; end // i=86  K=928
            13'd944  : begin f1 = 13'd147; f2 = 13'd118; valid = 1'b1; end // i=87  K=944
            13'd960  : begin f1 = 13'd29 ; f2 = 13'd60 ; valid = 1'b1; end // i=88  K=960
            13'd976  : begin f1 = 13'd59 ; f2 = 13'd122; valid = 1'b1; end // i=89  K=976
            13'd992  : begin f1 = 13'd65 ; f2 = 13'd124; valid = 1'b1; end // i=90  K=992
            13'd1008 : begin f1 = 13'd55 ; f2 = 13'd84 ; valid = 1'b1; end // i=91  K=1008
            13'd1024 : begin f1 = 13'd31 ; f2 = 13'd64 ; valid = 1'b1; end // i=92  K=1024
            13'd1056 : begin f1 = 13'd17 ; f2 = 13'd66 ; valid = 1'b1; end // i=93  K=1056
            13'd1088 : begin f1 = 13'd171; f2 = 13'd204; valid = 1'b1; end // i=94  K=1088
            13'd1120 : begin f1 = 13'd67 ; f2 = 13'd140; valid = 1'b1; end // i=95  K=1120
            13'd1152 : begin f1 = 13'd35 ; f2 = 13'd72 ; valid = 1'b1; end // i=96  K=1152
            13'd1184 : begin f1 = 13'd19 ; f2 = 13'd74 ; valid = 1'b1; end // i=97  K=1184
            13'd1216 : begin f1 = 13'd39 ; f2 = 13'd76 ; valid = 1'b1; end // i=98  K=1216
            13'd1248 : begin f1 = 13'd19 ; f2 = 13'd78 ; valid = 1'b1; end // i=99  K=1248
            13'd1280 : begin f1 = 13'd199; f2 = 13'd240; valid = 1'b1; end // i=100 K=1280
            13'd1312 : begin f1 = 13'd21 ; f2 = 13'd82 ; valid = 1'b1; end // i=101 K=1312
            13'd1344 : begin f1 = 13'd211; f2 = 13'd252; valid = 1'b1; end // i=102 K=1344
            13'd1376 : begin f1 = 13'd21 ; f2 = 13'd86 ; valid = 1'b1; end // i=103 K=1376
            13'd1408 : begin f1 = 13'd43 ; f2 = 13'd88 ; valid = 1'b1; end // i=104 K=1408
            13'd1440 : begin f1 = 13'd149; f2 = 13'd60 ; valid = 1'b1; end // i=105 K=1440
            13'd1472 : begin f1 = 13'd45 ; f2 = 13'd92 ; valid = 1'b1; end // i=106 K=1472
            13'd1504 : begin f1 = 13'd49 ; f2 = 13'd846; valid = 1'b1; end // i=107 K=1504
            13'd1536 : begin f1 = 13'd71 ; f2 = 13'd48 ; valid = 1'b1; end // i=108 K=1536
            13'd1568 : begin f1 = 13'd13 ; f2 = 13'd28 ; valid = 1'b1; end // i=109 K=1568
            13'd1600 : begin f1 = 13'd17 ; f2 = 13'd80 ; valid = 1'b1; end // i=110 K=1600
            13'd1632 : begin f1 = 13'd25 ; f2 = 13'd102; valid = 1'b1; end // i=111 K=1632
            13'd1664 : begin f1 = 13'd183; f2 = 13'd104; valid = 1'b1; end // i=112 K=1664
            13'd1696 : begin f1 = 13'd55 ; f2 = 13'd954; valid = 1'b1; end // i=113 K=1696
            13'd1728 : begin f1 = 13'd127; f2 = 13'd96 ; valid = 1'b1; end // i=114 K=1728
            13'd1760 : begin f1 = 13'd27 ; f2 = 13'd110; valid = 1'b1; end // i=115 K=1760
            13'd1792 : begin f1 = 13'd29 ; f2 = 13'd112; valid = 1'b1; end // i=116 K=1792
            13'd1824 : begin f1 = 13'd29 ; f2 = 13'd114; valid = 1'b1; end // i=117 K=1824
            13'd1856 : begin f1 = 13'd57 ; f2 = 13'd116; valid = 1'b1; end // i=118 K=1856
            13'd1888 : begin f1 = 13'd45 ; f2 = 13'd354; valid = 1'b1; end // i=119 K=1888
            13'd1920 : begin f1 = 13'd31 ; f2 = 13'd120; valid = 1'b1; end // i=120 K=1920
            13'd1952 : begin f1 = 13'd59 ; f2 = 13'd610; valid = 1'b1; end // i=121 K=1952
            13'd1984 : begin f1 = 13'd185; f2 = 13'd124; valid = 1'b1; end // i=122 K=1984
            13'd2016 : begin f1 = 13'd113; f2 = 13'd420; valid = 1'b1; end // i=123 K=2016
            13'd2048 : begin f1 = 13'd31 ; f2 = 13'd64 ; valid = 1'b1; end // i=124 K=2048
            13'd2112 : begin f1 = 13'd17 ; f2 = 13'd66 ; valid = 1'b1; end // i=125 K=2112
            13'd2176 : begin f1 = 13'd171; f2 = 13'd136; valid = 1'b1; end // i=126 K=2176
            13'd2240 : begin f1 = 13'd209; f2 = 13'd420; valid = 1'b1; end // i=127 K=2240
            13'd2304 : begin f1 = 13'd253; f2 = 13'd216; valid = 1'b1; end // i=128 K=2304
            13'd2368 : begin f1 = 13'd367; f2 = 13'd444; valid = 1'b1; end // i=129 K=2368
            13'd2432 : begin f1 = 13'd265; f2 = 13'd456; valid = 1'b1; end // i=130 K=2432
            13'd2496 : begin f1 = 13'd181; f2 = 13'd468; valid = 1'b1; end // i=131 K=2496
            13'd2560 : begin f1 = 13'd39 ; f2 = 13'd80 ; valid = 1'b1; end // i=132 K=2560
            13'd2624 : begin f1 = 13'd27 ; f2 = 13'd164; valid = 1'b1; end // i=133 K=2624
            13'd2688 : begin f1 = 13'd127; f2 = 13'd504; valid = 1'b1; end // i=134 K=2688
            13'd2752 : begin f1 = 13'd143; f2 = 13'd172; valid = 1'b1; end // i=135 K=2752
            13'd2816 : begin f1 = 13'd43 ; f2 = 13'd88 ; valid = 1'b1; end // i=136 K=2816
            13'd2880 : begin f1 = 13'd29 ; f2 = 13'd300; valid = 1'b1; end // i=137 K=2880
            13'd2944 : begin f1 = 13'd45 ; f2 = 13'd92 ; valid = 1'b1; end // i=138 K=2944
            13'd3008 : begin f1 = 13'd157; f2 = 13'd188; valid = 1'b1; end // i=139 K=3008
            13'd3072 : begin f1 = 13'd47 ; f2 = 13'd96 ; valid = 1'b1; end // i=140 K=3072
            13'd3136 : begin f1 = 13'd13 ; f2 = 13'd28 ; valid = 1'b1; end // i=141 K=3136
            13'd3200 : begin f1 = 13'd111; f2 = 13'd240; valid = 1'b1; end // i=142 K=3200
            13'd3264 : begin f1 = 13'd443; f2 = 13'd204; valid = 1'b1; end // i=143 K=3264
            13'd3328 : begin f1 = 13'd51 ; f2 = 13'd104; valid = 1'b1; end // i=144 K=3328
            13'd3392 : begin f1 = 13'd51 ; f2 = 13'd212; valid = 1'b1; end // i=145 K=3392
            13'd3456 : begin f1 = 13'd451; f2 = 13'd192; valid = 1'b1; end // i=146 K=3456
            13'd3520 : begin f1 = 13'd257; f2 = 13'd220; valid = 1'b1; end // i=147 K=3520
            13'd3584 : begin f1 = 13'd57 ; f2 = 13'd336; valid = 1'b1; end // i=148 K=3584
            13'd3648 : begin f1 = 13'd313; f2 = 13'd228; valid = 1'b1; end // i=149 K=3648
            13'd3712 : begin f1 = 13'd271; f2 = 13'd232; valid = 1'b1; end // i=150 K=3712
            13'd3776 : begin f1 = 13'd179; f2 = 13'd236; valid = 1'b1; end // i=151 K=3776
            13'd3840 : begin f1 = 13'd331; f2 = 13'd120; valid = 1'b1; end // i=152 K=3840
            13'd3904 : begin f1 = 13'd363; f2 = 13'd244; valid = 1'b1; end // i=153 K=3904
            13'd3968 : begin f1 = 13'd375; f2 = 13'd248; valid = 1'b1; end // i=154 K=3968
            13'd4032 : begin f1 = 13'd127; f2 = 13'd168; valid = 1'b1; end // i=155 K=4032
            13'd4096 : begin f1 = 13'd31 ; f2 = 13'd64 ; valid = 1'b1; end // i=156 K=4096
            13'd4160 : begin f1 = 13'd33 ; f2 = 13'd130; valid = 1'b1; end // i=157 K=4160
            13'd4224 : begin f1 = 13'd43 ; f2 = 13'd264; valid = 1'b1; end // i=158 K=4224
            13'd4288 : begin f1 = 13'd33 ; f2 = 13'd134; valid = 1'b1; end // i=159 K=4288
            13'd4352 : begin f1 = 13'd477; f2 = 13'd408; valid = 1'b1; end // i=160 K=4352
            13'd4416 : begin f1 = 13'd35 ; f2 = 13'd138; valid = 1'b1; end // i=161 K=4416
            13'd4480 : begin f1 = 13'd233; f2 = 13'd280; valid = 1'b1; end // i=162 K=4480
            13'd4544 : begin f1 = 13'd357; f2 = 13'd142; valid = 1'b1; end // i=163 K=4544
            13'd4608 : begin f1 = 13'd337; f2 = 13'd480; valid = 1'b1; end // i=164 K=4608
            13'd4672 : begin f1 = 13'd37 ; f2 = 13'd146; valid = 1'b1; end // i=165 K=4672
            13'd4736 : begin f1 = 13'd71 ; f2 = 13'd444; valid = 1'b1; end // i=166 K=4736
            13'd4800 : begin f1 = 13'd71 ; f2 = 13'd120; valid = 1'b1; end // i=167 K=4800
            13'd4864 : begin f1 = 13'd37 ; f2 = 13'd152; valid = 1'b1; end // i=168 K=4864
            13'd4928 : begin f1 = 13'd39 ; f2 = 13'd462; valid = 1'b1; end // i=169 K=4928
            13'd4992 : begin f1 = 13'd127; f2 = 13'd234; valid = 1'b1; end // i=170 K=4992
            13'd5056 : begin f1 = 13'd39 ; f2 = 13'd158; valid = 1'b1; end // i=171 K=5056
            13'd5120 : begin f1 = 13'd39 ; f2 = 13'd80 ; valid = 1'b1; end // i=172 K=5120
            13'd5184 : begin f1 = 13'd31 ; f2 = 13'd96 ; valid = 1'b1; end // i=173 K=5184
            13'd5248 : begin f1 = 13'd113; f2 = 13'd902; valid = 1'b1; end // i=174 K=5248
            13'd5312 : begin f1 = 13'd41 ; f2 = 13'd166; valid = 1'b1; end // i=175 K=5312
            13'd5376 : begin f1 = 13'd251; f2 = 13'd336; valid = 1'b1; end // i=176 K=5376
            13'd5440 : begin f1 = 13'd43 ; f2 = 13'd170; valid = 1'b1; end // i=177 K=5440
            13'd5504 : begin f1 = 13'd21 ; f2 = 13'd86 ; valid = 1'b1; end // i=178 K=5504
            13'd5568 : begin f1 = 13'd43 ; f2 = 13'd174; valid = 1'b1; end // i=179 K=5568
            13'd5632 : begin f1 = 13'd45 ; f2 = 13'd176; valid = 1'b1; end // i=180 K=5632
            13'd5696 : begin f1 = 13'd45 ; f2 = 13'd178; valid = 1'b1; end // i=181 K=5696
            13'd5760 : begin f1 = 13'd161; f2 = 13'd120; valid = 1'b1; end // i=182 K=5760
            13'd5824 : begin f1 = 13'd89 ; f2 = 13'd182; valid = 1'b1; end // i=183 K=5824
            13'd5888 : begin f1 = 13'd323; f2 = 13'd184; valid = 1'b1; end // i=184 K=5888
            13'd5952 : begin f1 = 13'd47 ; f2 = 13'd186; valid = 1'b1; end // i=185 K=5952
            13'd6016 : begin f1 = 13'd23 ; f2 = 13'd94 ; valid = 1'b1; end // i=186 K=6016
            13'd6080 : begin f1 = 13'd47 ; f2 = 13'd190; valid = 1'b1; end // i=187 K=6080
            13'd6144 : begin f1 = 13'd263; f2 = 13'd480; valid = 1'b1; end // i=188 K=6144
            default : begin f1 = 13'd0; f2 = 13'd0; valid = 1'b0; end // K không có trong bảng
        endcase                // hết bảng tra
    end                        // hết khối tổ hợp

endmodule
