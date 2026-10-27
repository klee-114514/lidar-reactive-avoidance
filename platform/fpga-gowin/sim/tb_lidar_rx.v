//=============================================================================
// tb_lidar_rx.v —— 验证 lidar_rx（115200、16× 过采样取中点 + 严格 7 字节头）【final_leida v1.4】
//-----------------------------------------------------------------------------
// 要证明六件事：
//   ① 正常波特率下，7 字节应答头 A5 5A 05 00 00 40 81 能一个不差地收下来
//   ② 字节计数 byte_cnt、应答头计数 hdr_cnt、应答类型 last_type 都对
//   ③ 波特率偏差 ±3% 时仍然能收对（这就是 16× 过采样取中点存在的意义）
//   ④ 垃圾/连续 A5 时头识别能重新对齐（不会卡死）
//   ⑤ ★头中间 4 字节错（A5 5A 05 00 00 00 81）⇒ 不认头★
//   ⑥ ★type 错（…40 80）⇒ 不认头★
//   ⑦ ★★核心回归：合法头之后的 payload 里出现 A5 5A ⇒ 绝不再算一次头★★
//      —— 这正是 v1.3 上板"T 每约 5 秒 +1、同拍 df/L/R 变 FFFF"的根因
//
// 运行（在 final_leida/sim/ms 下，路径按实际工程填）：
//   vlib work
//   vlog -work work <rtl 的绝对路径>\lidar_rx.v tb_lidar_rx.v
//   vsim -c -do "run -all; quit -f" work.tb_lidar_rx
//=============================================================================
`timescale 1ns/1ps

module tb_lidar_rx;

    localparam real BIT_115200 = 1000000000.0 / 115200.0;   // 8680.56 ns

    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50MHz

    reg  rst_n = 1'b0;
    reg  rx    = 1'b1;

    wire [7:0]  data;
    wire        valid;
    wire [15:0] byte_cnt, hdr_cnt;
    wire [7:0]  last_type;
    wire        hdr_seen, hdr_done;

    lidar_rx #(.CLK_FREQ(50_000_000), .BAUD(115200)) dut (
        .clk(clk), .rst_n(rst_n), .rx(rx),
        .data(data), .valid(valid),
        .byte_cnt(byte_cnt), .hdr_cnt(hdr_cnt),
        .last_type(last_type), .hdr_done(hdr_done), .hdr_seen(hdr_seen)
    );

    // ---------------- 收到的字节存档（valid 是单拍脉冲）----------------
    reg [7:0] cap [0:127];
    integer   ncap = 0;
    always @(posedge clk) begin
        if (valid && ncap < 128) begin
            cap[ncap] = data;
            ncap      = ncap + 1;
        end
    end

    // ---------------- hdr_done 脉冲计数（要和 hdr_cnt 始终相等）----------------
    integer hdr_pulses = 0;
    always @(posedge clk) if (hdr_done) hdr_pulses = hdr_pulses + 1;

    // ---------------- 按给定比特时间发一个字节 ----------------
    integer k;
    task send_byte(input [7:0] b, input real bit_ns);
        begin
            rx = 1'b0;            #(bit_ns);        // 起始位
            for (k = 0; k < 8; k = k + 1) begin
                rx = b[k];        #(bit_ns);        // 数据位（低位先发）
            end
            rx = 1'b1;            #(bit_ns);        // 停止位
        end
    endtask

    task send_hdr(input real bit_ns);
        begin
            send_byte(8'hA5, bit_ns);
            send_byte(8'h5A, bit_ns);
            send_byte(8'h05, bit_ns);
            send_byte(8'h00, bit_ns);
            send_byte(8'h00, bit_ns);
            send_byte(8'h40, bit_ns);
            send_byte(8'h81, bit_ns);
        end
    endtask

    integer errors = 0;

    // ---------------- 断言 ----------------
    task chk_cnt(input [15:0] e_byte, input [15:0] e_hdr, input [7:0] e_type, input [8*32:1] tag);
        begin
            if (byte_cnt !== e_byte) begin
                $display("  FAIL %0s: byte_cnt=%0d 期望 %0d", tag, byte_cnt, e_byte); errors = errors + 1;
            end
            if (hdr_cnt !== e_hdr) begin
                $display("  FAIL %0s: hdr_cnt=%0d 期望 %0d", tag, hdr_cnt, e_hdr); errors = errors + 1;
            end
            if (e_hdr != 0 && last_type !== e_type) begin
                $display("  FAIL %0s: last_type=%02h 期望 %02h", tag, last_type, e_type); errors = errors + 1;
            end
            if (hdr_pulses !== hdr_cnt) begin
                $display("  FAIL %0s: hdr_done 脉冲数 %0d != hdr_cnt %0d", tag, hdr_pulses, hdr_cnt);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        #200;
        rst_n = 1'b1;
        #200;

        // ---- ① 正常波特率：真实抓到的 7 字节应答头 ----
        $display("\n---- ① 正常 115200：A5 5A 05 00 00 40 81 ----");
        send_hdr(BIT_115200);
        #2000;
        chk_cnt(16'd7, 16'd1, 8'h81, "T1");
        $display("  ncap=%0d  byte_cnt=%0d  hdr_cnt=%0d  last_type=%02h  hdr_seen=%b",
                 ncap, byte_cnt, hdr_cnt, last_type, hdr_seen);
        if (cap[0]!==8'hA5 || cap[1]!==8'h5A || cap[2]!==8'h05 || cap[3]!==8'h00 ||
            cap[4]!==8'h00 || cap[5]!==8'h40 || cap[6]!==8'h81) begin
            $display("  FAIL T1: 收到的字节内容不对"); errors = errors + 1;
        end
        if (hdr_seen !== 1'b1) begin $display("  FAIL T1: hdr_seen 没置起"); errors = errors + 1; end

        // ---- ② 连续两个 A5 也要能重新对齐 ----
        $display("\n---- ② 重对齐：A5 A5 5A 05 00 00 40 81 ----");
        send_byte(8'hA5, BIT_115200);
        send_hdr(BIT_115200);
        #2000;
        chk_cnt(16'd15, 16'd2, 8'h81, "T2");

        // ---- ③ 波特率偏差 ±3%（这就是过采样要证明的东西）----
        $display("\n---- ③ 波特率 +3%%（8941ns/位）----");
        send_hdr(BIT_115200 * 1.03);
        #2000;
        chk_cnt(16'd22, 16'd3, 8'h81, "T3");

        $display("\n---- ③ 波特率 -3%%（8420ns/位）----");
        send_hdr(BIT_115200 * 0.97);
        #2000;
        chk_cnt(16'd29, 16'd4, 8'h81, "T4");

        // ---- ④ 垃圾字节不该破坏任何东西 ----
        $display("\n---- ④ 垃圾字节 0x00 ----");
        send_byte(8'h00, BIT_115200);
        #2000;
        chk_cnt(16'd30, 16'd4, 8'h81, "T5");

        // ---- ⑤ ★头中间 4 字节错 ⇒ 不认头（v1.4 新判据）★ ----
        $display("\n---- ⑤ 中间字节错：A5 5A 05 00 00 00 81 ⇒ 不算头 ----");
        send_byte(8'hA5, BIT_115200);
        send_byte(8'h5A, BIT_115200);
        send_byte(8'h05, BIT_115200);
        send_byte(8'h00, BIT_115200);
        send_byte(8'h00, BIT_115200);
        send_byte(8'h00, BIT_115200);   // ← 这里应该是 0x40
        send_byte(8'h81, BIT_115200);
        #2000;
        chk_cnt(16'd37, 16'd4, 8'h81, "T6");

        // ---- ⑥ ★type 错 ⇒ 不认头★ ----
        $display("\n---- ⑥ type 错：A5 5A 05 00 00 40 80 ⇒ 不算头 ----");
        send_byte(8'hA5, BIT_115200);
        send_byte(8'h5A, BIT_115200);
        send_byte(8'h05, BIT_115200);
        send_byte(8'h00, BIT_115200);
        send_byte(8'h00, BIT_115200);
        send_byte(8'h40, BIT_115200);
        send_byte(8'h80, BIT_115200);   // ← type 不是 0x81
        #2000;
        chk_cnt(16'd44, 16'd4, 8'h81, "T7");

        // ---- ⑦ ★★核心回归：payload 里的 A5 5A 不许再算头★★ ----
        //   发一个合法头，再发一个"字节里带 A5 5A"的数据应答报文：
        //     01 04 A5 5A 00
        //   v1.3 的旧逻辑会在 A5 5A 处再 +1 并清累加器 ⇒ 这一圈变成"整圈无点"
        $display("\n---- ⑦ payload 里带 A5 5A ⇒ hdr_cnt 必须只 +1 ----");
        send_hdr(BIT_115200);                       // 合法头：hdr_cnt 4→5
        #2000;
        chk_cnt(16'd51, 16'd5, 8'h81, "T8a");
        send_byte(8'h01, BIT_115200);               // 该"节点"里含 A5 5A
        send_byte(8'h04, BIT_115200);
        send_byte(8'hA5, BIT_115200);
        send_byte(8'h5A, BIT_115200);
        send_byte(8'h00, BIT_115200);
        #2000;
        chk_cnt(16'd56, 16'd5, 8'h81, "T8b");       // ★hdr_cnt 仍是 5★

        $display("\n==== tb_lidar_rx 结果 ====");
        $display("  收到字节 ncap = %0d（期望 56）", ncap);
        $display("  byte_cnt = %0d  hdr_cnt = %0d  hdr_done 脉冲 = %0d  last_type = %02h",
                 byte_cnt, hdr_cnt, hdr_pulses, last_type);
        if (errors == 0 && ncap == 56) $display("==== tb_lidar_rx PASS ====");
        else                           $display("==== tb_lidar_rx FAIL: errors=%0d ncap=%0d ====", errors, ncap);
        $finish;
    end

    initial begin
        #20_000_000;                 // 20ms 上限
        $display("==== tb_lidar_rx TIMEOUT ====");
        $finish;
    end

endmodule
