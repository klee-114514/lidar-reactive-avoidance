//=============================================================================
// tb_lidar_cmd.v —— 验证雷达命令时序与发送握手【final_leida v1.2】
//-----------------------------------------------------------------------------
// v1.1 里 lidar_rx 有 tb、lidar_cmd 没有，这是验证缺口。本 tb 把它补上，断言五件事：
//   ① 上电后不是马上发，而是等够 PWR_ON_MS 才发第一个字节
//   ② 发的字节序列正确：A5 25（停止）→ 停 GAP_MS → A5 20（开始扫描）
//   ③ ★不会"同一个字节被推两次"★（他们踩过的"隔一个丢一个"那个坑：
//      uart_tx 的 tx_busy 要下一拍才拉高，只看 !tx_busy 就会重复发）
//   ④ 监视期：GAP 之后 RETRY_MS 内没有新数据 ⇒ 自动重发一次 A5 20
//   ⑤ 一旦有数据进来（rx_byte_cnt 变化）⇒ 不再重发
//
// 判时序一律用【时钟沿计数 cyc】（不用 $time，避免 timescale 口径坑）：
//   cyc × 20ns = ns
//
// 运行（final_leida/sim/ms 下）：
//   vlib work
//   vlog -work work <rtl路径>/lidar_cmd.v <rtl路径>/uart_tx.v tb_lidar_cmd.v
//   vsim -c -do "run -all; quit -f" work.tb_lidar_cmd
//=============================================================================
`timescale 1ns/1ps

module tb_lidar_cmd;

    localparam real BIT_NS = 1000000000.0 / 115200.0;   // 8680.56ns

    // 参数故意取小值，仿真才跑得快（1ms 节拍仍然是真实的 50000 拍）
    localparam [31:0] PWR_ON_MS = 32'd5;
    localparam [31:0] GAP_MS    = 32'd3;
    localparam [31:0] RETRY_MS  = 32'd15;

    reg clk = 1'b0;
    always #10 clk = ~clk;              // 50MHz

    reg         rst_n = 1'b0;
    reg         rx_activity = 1'b0;      // ★v1.3：断流判据改用"每字节翻转位"（永不饱和）★

    wire        tx_start;
    wire [7:0]  tx_data;
    wire        tx_busy;
    wire        tx;

    // 时钟沿计数（判时序用这个，不用 $time）
    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    // ---------------- DUT + 真实 uart_tx ----------------
    lidar_cmd #(
        .CLK_FREQ (50_000_000),
        .PWR_ON_MS(PWR_ON_MS),
        .GAP_MS   (GAP_MS),
        .RETRY_MS (RETRY_MS)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .rx_activity(rx_activity),
        .tx_start(tx_start), .tx_data(tx_data), .tx_busy(tx_busy)
    );

    uart_tx #(.CLK_FREQ(50_000_000), .BAUD(115200)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_start(tx_start), .tx_data(tx_data),
        .tx_busy(tx_busy), .tx(tx)
    );

    // ---------------- 115200 接收机：把 tx 上的字节还原出来 ----------------
    reg  [7:0]  rec [0:15];
    integer     rcyc  [0:15];
    integer     nrec = 0;
    integer     kk;
    reg  [7:0]  bsh;

    initial begin
        forever begin
            @(negedge tx);                  // 起始位下降沿
            #(BIT_NS * 1.5);                // 对到第 0 位中心
            for (kk = 0; kk < 8; kk = kk + 1) begin
                bsh[kk] = tx;
                #(BIT_NS);
            end
            if (nrec < 16) begin
                rec[nrec]  = bsh;
                rcyc[nrec] = cyc;
                nrec       = nrec + 1;
            end
        end
    end

    // ---------------- 断言 ----------------
    integer errors = 0;
    task chk(input cond, input [8*40:1] msg);
        begin
            if (!cond) begin
                $display("  FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    integer gap_cyc, gap_expect;

    initial begin
        #200;                       // POR（顶层是 40ms，这里只是 tb 自己的复位）
        rst_n = 1'b1;

        // ---- ① 上电 5ms 内不该有任何字节 ----
        #4_000_000;                 // 4ms（<5ms）
        chk(nrec == 0, "T1: PWR_ON_MS 之前就发字节了");

        // ---- ② 等到 9ms：应当是 A5 25 / A5 20 ----
        #5_000_000;                 // 累计 9ms
        $display("\n[cyc=%0d] 收到 %0d 字节", cyc, nrec);
        for (kk = 0; kk < nrec; kk = kk + 1) $display("    rec[%0d] = %02h  @cyc=%0d", kk, rec[kk], rcyc[kk]);
        chk(nrec == 4, "T2: 9ms 后应当正好 4 字节（A5 25 A5 20）");
        if (nrec >= 4) begin
            chk(rec[0] === 8'hA5, "T2: rec[0] 应为 A5");
            chk(rec[1] === 8'h25, "T2: rec[1] 应为 25（停止）");
            chk(rec[2] === 8'hA5, "T2: rec[2] 应为 A5");
            chk(rec[3] === 8'h20, "T2: rec[3] 应为 20（开始扫描）");
            // ③ 同一对里的两个字节之间不应有额外字节（防重复发送）
            chk(rec[0] === rec[2], "T3: 头两字节应一致（A5）");
            gap_cyc    = rcyc[2] - rcyc[1];              // 25 与 A5 之间（含 25 那个字节本身）
            gap_expect = GAP_MS * 50000;                 // 3ms = 150000 拍
            // ★注意★ 1ms 节拍分频器是【自由运行】的，清零 ms_cnt 时并不复位分频器，
            //   所以"等 3 个 tick"实际可能是"等到接下来的第 3 个整毫秒边界"，
            //   误差最多 ±1ms（对 10ms 的命令间隔、2s 的重发间隔都无所谓）。
            //   所以这里判：不小于 (GAP_MS-1) 个毫秒、也不超过 GAP_MS 个毫秒 + 一个字节。
            $display("  25→A5 间隔 = %0d 拍（期望 %0d~%0d 拍，含 25 字节本身 4344 拍）",
                     gap_cyc, (GAP_MS-1)*50000, gap_expect + 6000);
            chk((gap_cyc > (GAP_MS-1)*50000) && (gap_cyc < gap_expect + 6000),
                "T3: A5 25 与 A5 20 之间的间隔不在 GAP_MS ±1ms 内");
        end

        // ---- ④ 监视期：RETRY_MS(15ms) 内没数据 ⇒ 应当重发 A5 20 ----
        #10_000_000;                // 累计 19ms（监视从 ~8.4ms 起，还没到 15ms）
        chk(nrec == 4, "T4: 监视期没到 RETRY_MS 就重发了");
        #10_000_000;                // 累计 29ms（已过 RETRY 点 ~23.4ms）
        $display("\n[cyc=%0d] 重发后共 %0d 字节", cyc, nrec);
        for (kk = 4; kk < nrec; kk = kk + 1) $display("    rec[%0d] = %02h  @cyc=%0d", kk, rec[kk], rcyc[kk]);
        chk(nrec == 6, "T4: 断流后应当重发一对（A5 20）");
        if (nrec >= 6) begin
            chk(rec[4] === 8'hA5, "T4: 重发第一个字节应为 A5");
            chk(rec[5] === 8'h20, "T4: 重发第二个字节应为 20");
        end

        // ---- ⑤ 有数据在动时不应再重发 ----
        //   ★注意★ 必须"持续有新数据"才算数据流：只把翻转位翻一次，
        //   之后又静默超过 RETRY_MS，重发是【正确】行为（那样才测不出问题）。
        $display("\n[cyc=%0d] 开始模拟数据流（每 1ms 翻转一次 activity，持续 20ms）", cyc);
        for (kk = 0; kk < 20; kk = kk + 1) begin
            rx_activity = ~rx_activity;
            #1_000_000;
        end
        chk(nrec == 6, "T5: 数据一直在动却还在重发");
        $display("[cyc=%0d] 数据流停止，再等 20ms（此时重发才是对的）", cyc);
        #20_000_000;
        chk(nrec == 8, "T5: 数据停了却没有重发");

        $display("\n==== tb_lidar_cmd 结果 ====");
        $display("  共收到字节 = %0d", nrec);
        if (errors == 0) $display("==== tb_lidar_cmd PASS ====");
        else             $display("==== tb_lidar_cmd FAIL: errors=%0d ====", errors);
        $finish;
    end

    initial begin
        #200_000_000;               // 200ms 上限
        $display("==== tb_lidar_cmd TIMEOUT (nrec=%0d, errors=%0d) ====", nrec, errors);
        $finish;
    end

endmodule
