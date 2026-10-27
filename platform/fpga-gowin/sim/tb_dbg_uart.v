//=============================================================================
// tb_dbg_uart.v —— 顶层调试串口验证（115200-8-N-1）【final_leida v1.0】
//-----------------------------------------------------------------------------
// ★v1.0 与 final_servo 版的差别★：
//   · 顶层不再是 final_servo_top，而是 final_leida_top，且**没有超声/舵机端口**
//   · 原来的 HC-SR04 Echo 模型、Trig 计数整块删掉（超声已停用）
//   · 感知输入是常量桩 ⇒ 期望值变成固定值：
//         df = 0FA0 (4000mm 恒定)   L=1 R=1 (剖面桩 4000 ≥ 450)
//         a=0  s=0  i=1             f=88 (servo_init_done + scan_done)
//         V = FFFF (dist_valid 恒 1，计数饱和——"没有测距链路"的正常表现)
//         T = 0000
//   · 本 tb 仍然要守住的核心目标没变：把 uart_tx 按 115200 收下来还原成文本，
//     检查 V5.0 四行 323 字节、CR/LF 齐全、不隔一个丢一个。
//
// 激励：编码器 A/B 给慢速正交方波 → 期望 SL/SR/CL/CR 非 0
//
// 运行：
//   vlib work
//   vlog -work work ../rtl/avoid_fsm.v ../rtl/encoder_decode.v ../rtl/final_leida_top.v ... tb_dbg_uart.v
//   vsim -c -do "run -all; quit -f" work.tb_dbg_uart
//=============================================================================
`timescale 1ns/1ps

module tb_dbg_uart;

    // ---------------- 50MHz 主时钟 ----------------
    reg clk = 1'b0;
    always #10 clk = ~clk;

    // ---------------- 激励 ----------------
    reg  [7:0] sw        = 8'hFF;   // 全不拨 = 原逻辑
    reg        key_stop  = 1'b1;    // 低有效：1 = 没按
    reg        key_start = 1'b1;
    reg        key_fwd   = 1'b1;
    reg        key_left  = 1'b1;
    reg        key_r     = 1'b1;
    reg        imu_rx    = 1'b1;    // 串口空闲高
    reg        ir_rx     = 1'b1;    // 红外空闲高
    reg        lidar_rx  = 1'b1;    // ★v1.4：雷达串口空闲高（本 tb 不灌雷达数据）★

    // ---------------- DUT 端口网线 ----------------
    wire        uart_tx;
    wire        ain1_l, ain2_l, pwma, ain1_r, ain2_r, pwmb, stby;
    wire [7:0]  led;
    wire        beep;                    // ★v1.6：无源蜂鸣器（4kHz 载波门控）
    wire        bt_tx;                   // ★v1.7：蓝牙调试串口（日志镜像）

    // 编码器：两路正交方波（A、B 错开 1/4 周期）
    reg  [9:0] qdiv = 10'd0;
    reg        qa   = 1'b0;
    reg        qb   = 1'b0;
    always @(posedge clk) begin
        qdiv <= qdiv + 10'd1;
        if (qdiv == 10'd0)        qa <= ~qa;
        else if (qdiv == 10'd512) qb <= ~qb;
    end

    // ---------------- DUT ----------------
    // ★v1.0：超声/舵机端口已从顶层删除，这里也不再连★
    // ★2026-10-15：把蜂鸣器的时间参数压小，好在 2.2s 的仿真里看到
    //   "就绪 2 短声 + 断流 2 短声一次 + 之后完全安静"★
    final_leida_top #(
        .BEEP_DEAD_DLY_MS (900),
        .BEEP_REP_MS      (800),
        // ★v1.8：顶层默认 BT_BAUD 已改成 9600，这里【显式覆盖回 115200】★ ——
        //   下面"bt_tx 与 uart_tx 逐拍完全一致"那条断言只有在两路同速时才成立，
        //   而本 tb 的接收机也只按 115200 解有线路，不覆盖仿真必然 FAIL 喵。
        .BT_BAUD          (115200)
    ) dut (
        .clk        (clk),
        .key_stop   (key_stop),
        .key_start  (key_start),
        .key_fwd    (key_fwd),
        .key_left   (key_left),
        .key_r      (key_r),
        .sw         (sw),
        .ain1_l     (ain1_l),
        .ain2_l     (ain2_l),
        .pwma       (pwma),
        .ain1_r     (ain1_r),
        .ain2_r     (ain2_r),
        .pwmb       (pwmb),
        .stby       (stby),
        .enc_al     (qa),
        .enc_bl     (qb),
        .enc_ar     (qa),
        .enc_br     (qb),
        .imu_rx     (imu_rx),
        .lidar_rx   (lidar_rx),          // ★v1.4：必须驱动（空闲高），否则收不到确定电平★
        .ir_rx      (ir_rx),
        .uart_tx    (uart_tx),
        .beep       (beep),
        .bt_tx      (bt_tx),
        .led        (led)
    );

    // ---------------- ★v1.7 蓝牙那一路必须和有线那一路【逐拍完全一致】★ ----------------
    //   ★v1.8：顶层默认已改 9600，这里靠上面的 .BT_BAUD(115200) 覆盖成两路同速★
    //   ⇒ 输出应当一位不差；有任何不一致都说明有一路被漏推/被覆盖。
    integer bt_mismatch = 0;
    always @(posedge clk) if (bt_tx !== uart_tx) bt_mismatch = bt_mismatch + 1;

    // ---------------- ★v1.6 调试指示：雷达健康灯 + 蜂鸣器边沿计数★ ----------------
    //   本 tb 不灌雷达数据 ⇒ ld_scan_fresh 永远 0 ⇒ led_dbg 应恒 0（雷达断流）
    reg        beep_d  = 1'b0;
    integer    beep_edges = 0;
    integer    errors = 0;               // ★必须先声明：下面的 chk_edges/行长度断言都用它★
    always @(posedge clk) begin
        beep_d <= beep;
        if (beep !== beep_d) beep_edges = beep_edges + 1;   // 载波翻转次数
    end
    task chk_edges(input integer e0, input integer lo, input integer hi, input [8*44:1] msg);
        integer d;
        begin
            d = beep_edges - e0;
            if (d < lo || d > hi) begin
                $display("\n  FAIL: %0s 边沿数 %0d，期望 %0d~%0d", msg, d, lo, hi);
                errors = errors + 1;
            end
        end
    endtask

    // ---------------- 逐行长度断言（V5.0：29 + 48 + 175 + 71 = 323 字节）----------------
    //   阶段0 在行3 尾追加了 A/S/t/x（电机段动作/状态/转向命令/绕障握手）；
    //   2a 又追加了 E=(障碍段两端方位+距离) / D=(段内最近距离) / FL / FR：
    //     行1 = 29 字节（27 字符 + CRLF）
    //     行2 = 48 字节（46 字符 + CRLF）
    //     行3 = 175 字节（173 字符 + CRLF；★V4.0d★ 末尾加 S= 决策标志 + V=4.00d）
    //     V5.0 追加行4 = 71 字节；整帧 = 29 + 48 + 175 + 71 = 323 字节
    //   这个断言就是为了守住"改打印时又隔一个丢一个"那个老 bug。
    integer last_lf  = 0;
    integer line_no  = 0;
    reg [15:0] exp_len;
    reg [7:0]  exp_head;
    task chk_line(input integer len);
        begin
            case (line_no % 4)
                0: begin exp_len = 16'd29; exp_head = "d"; end
                1: begin exp_len = 16'd48; exp_head = "P"; end
                2: begin exp_len = 16'd175; exp_head = "M"; end
                default: begin exp_len = 16'd71; exp_head = "Z"; end
            endcase
            if (len !== exp_len) begin
                $display("\n  FAIL: 第 %0d 行长度 %0d，期望 %0d", line_no + 1, len, exp_len);
                errors = errors + 1;
            end
            line_no = line_no + 1;
        end
    endtask

    // ---------------- 115200 接收机（逐字节还原文本）----------------
    localparam real BIT_NS = 1000000000.0 / 115200.0;   // 8680.56ns

    integer   i;
    integer   nbyte = 0;          // 收到的总字节数
    reg [7:0] rx;

    initial begin
        forever begin
            @(negedge uart_tx);                 // 起始位下降沿
            #(BIT_NS * 1.5);                    // 对到第 0 位中心
            for (i = 0; i < 8; i = i + 1) begin
                rx[i] = uart_tx;
                #(BIT_NS);
            end
            nbyte = nbyte + 1;
            if (rx == 8'h0D)      $write("<CR>");
            else if (rx == 8'h0A) begin
                $write("<LF>\n");
                $fflush();
                chk_line(nbyte - last_lf);      // ★v1.4：逐行长度断言★
                last_lf = nbyte;
            end
            else if (rx >= 8'h20 && rx < 8'h7F) $write("%c", rx);
            else                  $write("<%02X>", rx);
        end
    end

    // ---------------- ★蜂鸣器 / 雷达健康灯 时序检查（2026-10-15 静音化后）★ ----------------
    //   tb 里把 BEEP_DEAD_DLY_MS 压到 900ms，于是时间线是：
    //     POR 在 40ms ⇒ 就绪两短声 [140,260) + [380,500)
    //     dead_dly 900ms ⇒ 断流【响 2 短声一次】：[900,1020) + [1140,1260)
    //     ★之后不再复读★（旧的"每 BEEP_REP_MS 一长声"已取消）
    integer eA, eB, eC, eD, eE;
    initial begin
        #150_000_000;  eA = beep_edges;      // 150ms
        #100_000_000;
        chk_edges(eA, 400, 1400, "就绪第 1 短声（应听到 4kHz 载波）");
        #20_000_000;   eB = beep_edges;      // 270ms（两短声之间的静音段内）
        #100_000_000;
        chk_edges(eB, 0, 0, "两短声之间应静音");   // 270→370ms
        #230_000_000;  eC = beep_edges;      // 600ms
        #200_000_000;
        chk_edges(eC, 0, 0, "断流延时期内不该响");  // 600→800ms
        // 雷达健康指示（led[1] = B15 扩展口，板上没灯，所以只能在这里核电平）：
        // 本 tb 不灌雷达数据 ⇒ 必须是灭的
        if (led[1] !== 1'b0) begin
            $display("\n  FAIL: led[1] 应为 0（本 tb 没有雷达数据）"); errors = errors + 1;
        end
        #160_000_000;  eD = beep_edges;      // 960ms（已进入断流 2 短声的第一声）
        #400_000_000;
        chk_edges(eD, 800, 4000, "断流应响 2 短声（不是长鸣）");        // 960→1360ms
        #400_000_000;  eE = beep_edges;      // 1760ms
        #300_000_000;
        chk_edges(eE, 0, 0, "断流不再复读（应完全安静）");              // 1760→2060ms
        $display("\n---- 指示检查完成：beep 边沿总数=%0d，led[1]=%b ----",
                 beep_edges, led[1]);
        // ★v1.7：蓝牙那一路和有线那一路的逐拍一致性★
        if (bt_mismatch !== 0) begin
            $display("\n  FAIL: bt_tx 与 uart_tx 有 %0d 拍不一致（两路应完全同步）", bt_mismatch);
            errors = errors + 1;
        end else begin
            $display("---- v1.7 蓝牙镜像检查：bt_tx 与 uart_tx 逐拍一致（不一致拍数=0）----");
        end
    end

    initial begin
        // ★v1.4：跑到 700ms —— 超过 SENS_STALE_CYC(0.5s)，可以看到"雷达断流
        //   被当成正前被堵"这条安全路径真的生效：f 的 bit5(front_blk)/bit0(estop_lat)
        //   会置 1（f=80 → f=A1），也就是上板时 LED4 8Hz 快闪的那个状态★
        // ★2026-10-15：延长到 2.1s，把"就绪两短声 + 断流 2 短声一次 + 之后安静"检查完★
        //   （注意：#2_200_000_000 会溢出 32 位有符号整数，别再写回 2.2e9）
        #2_100_000_000;
        $display("\n==== tb_dbg_uart 结束 ====");
        $display("总收到字节 = %0d（%0d 行）", nbyte, line_no);
        $display("（本 tb 不灌雷达数据 ⇒ df=0000、f 先 80 后 A1、led_dbg=0、");
        $display("       蜂鸣器应能听到：上电 2 短声 + 断流 2 短声（只一次），之后安静）");
        if (errors == 0 && line_no >= 4) $display("==== tb_dbg_uart PASS ====");
        else                             $display("==== tb_dbg_uart FAIL: errors=%0d line_no=%0d ====", errors, line_no);
        $finish;
    end

endmodule
