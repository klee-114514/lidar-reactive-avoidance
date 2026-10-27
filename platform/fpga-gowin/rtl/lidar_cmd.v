//=============================================================================
// lidar_cmd.v —— RPLIDAR A1M8 上电命令序列 + 断流重发【final_leida v1.1】
//-----------------------------------------------------------------------------
// 为什么必须发命令：
//   RPLIDAR A1M8 不是"上电自动吐数据"的雷达。主机必须先发 A5 25（停止、
//   把状态清干净）再发 A5 20（开始扫描），它才会持续输出数据节点。
//   电机是 MOTOCTL 接 5V 常转（硬件已解决），但"出不出数据"由这两条命令决定。
//
// 时序（都可用参数改）：
//   上电 → 等 PWR_ON_MS(默认 1000ms，等雷达内部就绪)
//        → A5 → 25（停止）
//        → 等 GAP_MS(默认 10ms)
//        → A5 → 20（开始扫描）
//        → 进入监视：只要 rx_byte_cnt 在 RETRY_MS(默认 2000ms) 内没变过，
//          就重发一次 A5 20（现场省事：万一命令没发进去，不用重新上电）
//
// ★发送握手纪律（他们踩过的坑）★：uart_tx 的 tx_busy 要等被 tx_start 唤醒的
//   【下一拍】才拉高、而且要到【整字节发完】才落下。所以一个字节必须走完四步：
//      置 tx_data/tx_start → 拉低 tx_start → 等 busy 拉高 → ★等 busy 拉低★ = 发送完成
//   ★v1.2 修的 bug★：原来第 4 步漏了"等 busy 拉低"，于是第二个字节的 tx_start
//   在 uart_tx 还在发第一个字节时就被打出去（会被忽略）⇒ 字节被吞。
//   实测症状：发出去的是 A5 A5（0x25 丢了）⇒ 雷达永远收不到 A5 20 ⇒ 一个字节都不吐。
//   tb_lidar_cmd 就是为这条断言写的。
//=============================================================================
module lidar_cmd #(
    parameter CLK_FREQ        = 50_000_000,
    parameter [31:0] PWR_ON_MS = 32'd1000,
    parameter [31:0] GAP_MS    = 32'd10,
    parameter [31:0] RETRY_MS  = 32'd2000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx_activity,   // 来自 lidar_rx：每收到一个字节翻转一次（永不饱和）
    output reg         tx_start,
    output reg  [7:0]  tx_data,
    input  wire        tx_busy
);

    localparam [7:0] SYNC     = 8'hA5;
    localparam [7:0] CMD_STOP = 8'h25;
    localparam [7:0] CMD_SCAN = 8'h20;

    // ---------------- 1ms 节拍 ----------------
    // ★注意★ 这个分频器是【自由运行】的：清零 ms_cnt 时并不同时复位 ms_div，
    //   所以"等 N 个 tick"= "等到接下来的第 N 个整毫秒边界"，实际间隔会比 N ms
    //   少最多 1ms。对本模块的用途（10ms 命令间隔、2s 断流重发）完全无所谓，
    //   但如果以后要拿它做精确计时，记得连分频器一起复位（注意别造成多驱动）。
    localparam integer CLK_PER_MS = CLK_FREQ / 1000;
    reg [15:0] ms_div;
    reg        ms_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_div  <= 16'd0;
            ms_tick <= 1'b0;
        end else if (ms_div >= CLK_PER_MS - 1) begin
            ms_div  <= 16'd0;
            ms_tick <= 1'b1;
        end else begin
            ms_div  <= ms_div + 16'd1;
            ms_tick <= 1'b0;
        end
    end

    // ---------------- 主状态机 ----------------
    localparam [2:0] ST_PWR     = 3'd0,
                     ST_STOP_A5 = 3'd1,
                     ST_STOP_25 = 3'd2,
                     ST_GAP     = 3'd3,
                     ST_SCAN_A5 = 3'd4,
                     ST_SCAN_20 = 3'd5,
                     ST_MON     = 3'd6;

    reg [2:0]  st;
    reg [1:0]  sp;          // 0=没在发 1=已发 tx_start 2=等 busy 起 3=等 busy 落
    reg [31:0] ms_cnt;
    reg        last_act;    // ★用翻转位判"有没有新数据"，不能用会饱和的计数器★

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= ST_PWR;
            sp       <= 2'd0;
            ms_cnt   <= 32'd0;
            last_act <= 1'b0;
            tx_start <= 1'b0;
            tx_data  <= 8'd0;
        end else begin
            if (ms_tick) ms_cnt <= ms_cnt + 32'd1;

            case (st)
                // ---- 等雷达上电就绪 ----
                ST_PWR: begin
                    if (ms_cnt >= PWR_ON_MS) begin
                        ms_cnt <= 32'd0;
                        st     <= ST_STOP_A5;
                    end
                end

                // ---- A5 25：停止 ----
                ST_STOP_A5: begin
                    if      (sp == 2'd0) begin tx_data <= SYNC;     tx_start <= 1'b1; sp <= 2'd1; end
                    else if (sp == 2'd1) begin tx_start <= 1'b0;    sp <= 2'd2; end
                    else if (sp == 2'd2) begin if (tx_busy)  sp <= 2'd3; end
                    else                 begin if (!tx_busy) begin sp <= 2'd0; st <= ST_STOP_25; end end
                end
                ST_STOP_25: begin
                    if      (sp == 2'd0) begin tx_data <= CMD_STOP; tx_start <= 1'b1; sp <= 2'd1; end
                    else if (sp == 2'd1) begin tx_start <= 1'b0;    sp <= 2'd2; end
                    else if (sp == 2'd2) begin if (tx_busy)  sp <= 2'd3; end
                    else begin
                        if (!tx_busy) begin sp <= 2'd0; ms_cnt <= 32'd0; st <= ST_GAP; end
                    end
                end

                // ---- 间隔 ----
                ST_GAP: begin
                    if (ms_cnt >= GAP_MS) begin
                        ms_cnt <= 32'd0;
                        st     <= ST_SCAN_A5;
                    end
                end

                // ---- A5 20：开始扫描 ----
                ST_SCAN_A5: begin
                    if      (sp == 2'd0) begin tx_data <= SYNC;     tx_start <= 1'b1; sp <= 2'd1; end
                    else if (sp == 2'd1) begin tx_start <= 1'b0;    sp <= 2'd2; end
                    else if (sp == 2'd2) begin if (tx_busy)  sp <= 2'd3; end
                    else                 begin if (!tx_busy) begin sp <= 2'd0; st <= ST_SCAN_20; end end
                end
                ST_SCAN_20: begin
                    if      (sp == 2'd0) begin tx_data <= CMD_SCAN; tx_start <= 1'b1; sp <= 2'd1; end
                    else if (sp == 2'd1) begin tx_start <= 1'b0;    sp <= 2'd2; end
                    else if (sp == 2'd2) begin if (tx_busy)  sp <= 2'd3; end
                    else begin
                        if (!tx_busy) begin
                            sp       <= 2'd0;
                            ms_cnt   <= 32'd0;
                            last_act <= rx_activity;
                            st       <= ST_MON;
                        end
                    end
                end

                // ---- 监视：断流就重发 A5 20 ----
                ST_MON: begin
                    if (rx_activity != last_act) begin
                        last_act <= rx_activity;   // 有新数据 ⇒ 计时清零
                        ms_cnt   <= 32'd0;
                    end else if (ms_cnt >= RETRY_MS) begin
                        ms_cnt <= 32'd0;
                        st     <= ST_SCAN_A5;
                    end
                end

                default: st <= ST_PWR;
            endcase
        end
    end

endmodule
