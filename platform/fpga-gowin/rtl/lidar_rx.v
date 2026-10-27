//=============================================================================
// lidar_rx.v —— RPLIDAR A1M8 串口接收（115200-8N1，16× 过采样取中点）【final_leida v1.1】
//-----------------------------------------------------------------------------
// 为什么不用现成的 uart_rx.v：
//   · uart_rx 是按 9600 写的（50M/9600 = 5208 拍/位，单点采样也够）；
//     115200 只有 434 拍/位，而且雷达串口长期跑在 ~85~90% 占用率上，
//     单点采样遇到发送端时钟偏差就会误码 —— 而误码在地图上表现为"零星假点"，
//     现场极难排查。所以这里用【16× 过采样 + 每比特中点采样】。
//
// 16× tick 用【小数分频累加器】生成（不是 CLK_FREQ/BAUD/16 的整数除法）：
//   每个 clk 累加 16×BAUD，溢出 CLK_FREQ 就出一个 tick ⇒ 长期平均正好 16×BAUD，
//   没有累积误差（50M、115200 时平均 27.13 拍/tick）。
//
// 除字节流本身，还顺带产出几个"给串口调试看"的量：
//   byte_cnt   收到的字节数（★16 位自由回绕，不饱和★——饱和了就看不出"还在不在动"）
//   hdr_cnt    ★通过完整校验的应答头次数★（16 位自由回绕）——
//              v1.4 起只有 7 字节全对才 +1，所以它现在就是"雷达应答了几次"，
//              正常应该只在发命令时 +1（长时间自己涨 = 有假头，见下）
//   last_type  应答头第 7 个字节（type）：0x81 = 标准扫描数据
//   hdr_done   ★完整 7 字节头通过后的单拍脉冲★（给解析模块做字节对齐/清累加器用）
//   activity   ★每收到一个字节翻转一次★（1 位、永不饱和）
//              —— lidar_cmd 的"断流重发"判据必须用它：用会饱和的计数器判断，
//                 计数器一饱和就被误判成断流，于是每 2s 重发一次命令、把扫描打断
//                 （2026-10-14 上板实测：T 每 2 秒 +1 就是这个 bug）
//   hdr_seen   曾经收到过完整合法头（锁存）
//
// ★v1.4 修的头识别 bug（2026-10-14 定位）★
//   旧版只要看到 A5 5A 两个字节就认头。但 A5 5A 只是 2 字节同步字，在 ~10KB/s 的
//   数据流里"恰好撞上"的概率约 1/65536 ⇒ 期望约 0.15 次/秒。撞上时：
//     · hdr_cnt（串口 T）自己 +1（实测 0.19 次/秒，吻合）
//     · hdr_done 误触发 ⇒ lidar_parse 把本圈三扇区累加器清成 FAR
//     · 那一圈锁存出去就是 FFFF / F=0，现场看着像"整圈一个有效点都没有"
//   官方 SDK（sl_lidarprotocol_codec.cpp）的办法是进 loop 后只按长度取 payload、
//   全程不再找同步字。这里是等价的最简做法：★认头必须 7 字节全对★
//     A5 5A 05 00 00 40 81
//     └┬─┘ └────┬────┘ └┬─┘
//     同步字   4 字节小端 = 0x40000005：    type
//              低 30 位 size = 5（单条数据应答 5 字节）
//              高 2 位 subType = 01b（多次应答）
//   假头概率因此降到 1/2^56 量级，实际不可能再发生。
//=============================================================================
module lidar_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200,
    parameter [7:0] EXPECT_TYPE = 8'h81    // ★v1.4：SCAN 应答的数据类型（标准扫描）★
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx,
    output reg  [7:0]  data,
    output reg         valid,
    output reg  [15:0] byte_cnt,
    output reg  [15:0] hdr_cnt,
    output reg  [7:0]  last_type,
    output reg         hdr_done,
    output reg         activity,
    output reg         hdr_seen
);

    // ---------------- 16× 波特率 tick（小数分频）----------------
    localparam integer ACC_STEP = 16 * BAUD;     // 1_843_200
    localparam integer ACC_MOD  = CLK_FREQ;      // 50_000_000
    reg [31:0] acc;
    reg        tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc  <= 32'd0;
            tick <= 1'b0;
        end else if (acc + ACC_STEP >= ACC_MOD) begin
            acc  <= acc + ACC_STEP - ACC_MOD;
            tick <= 1'b1;
        end else begin
            acc  <= acc + ACC_STEP;
            tick <= 1'b0;
        end
    end

    // ---------------- 输入同步 + 边沿检测 ----------------
    reg rx_s1, rx_s2, rx_s3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s1 <= 1'b1; rx_s2 <= 1'b1; rx_s3 <= 1'b1;
        end else begin
            rx_s1 <= rx;
            rx_s2 <= rx_s1;
            rx_s3 <= rx_s2;
        end
    end
    wire rx_fall = rx_s3 & (~rx_s2);

    // ---------------- 位状态机（tick 驱动，os 0..15）----------------
    localparam [1:0] S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;
    reg [1:0] bs;
    reg [3:0] os;
    reg [2:0] bit_idx;
    reg [7:0] shreg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bs <= S_IDLE; os <= 4'd0; bit_idx <= 3'd0; shreg <= 8'd0;
            data <= 8'd0; valid <= 1'b0;
        end else begin
            valid <= 1'b0;
            case (bs)
                S_IDLE: begin
                    if (rx_fall) begin        // 起始位下降沿：对齐 os
                        os <= 4'd0;
                        bs <= S_START;
                    end
                end
                S_START: begin
                    if (tick) begin
                        if (os == 4'd7) begin     // 8 个 tick = 半位 ⇒ 起始位中点
                            os <= os + 4'd1;
                            if (!rx_s2) begin     // 仍为低 ⇒ 真起始位
                                bs      <= S_DATA;
                                bit_idx <= 3'd0;
                                shreg   <= 8'd0;
                            end else begin
                                bs <= S_IDLE;     // 假起始
                            end
                        end else begin
                            os <= os + 4'd1;
                        end
                    end
                end
                S_DATA: begin
                    if (tick) begin
                        if (os == 4'd7) begin     // 每 16 个 tick 采一次（位中点）
                            shreg   <= {rx_s2, shreg[7:1]};   // 低位先到
                            os      <= os + 4'd1;
                            if (bit_idx == 3'd7) bs <= S_STOP;
                            else                 bit_idx <= bit_idx + 3'd1;
                        end else begin
                            os <= os + 4'd1;
                        end
                    end
                end
                S_STOP: begin
                    if (tick) begin
                        if (os == 4'd7) begin     // 停止位中点应为 1
                            os <= os + 4'd1;
                            if (rx_s2) begin
                                data  <= shreg;
                                valid <= 1'b1;
                            end
                            bs <= S_IDLE;
                        end else begin
                            os <= os + 4'd1;
                        end
                    end
                end
                default: bs <= S_IDLE;
            endcase
        end
    end

    // ---------------- 字节级：计数 + ★严格 7 字节应答头识别（v1.4）★ ----------------
    //   A5 5A | 05 00 00 40 | 81
    //   任何一字节不符 ⇒ 回 H_IDLE，★不产生 hdr_done、不涨 hdr_cnt★（这就是假头修法）
    localparam [2:0] H_IDLE = 3'd0, H_A5 = 3'd1, H_5A = 3'd2,
                     H_SIZE = 3'd3, H_TYPE = 3'd4;
    reg [2:0] hst;
    reg [1:0] size_i;

    // 期望的 size/subType 4 字节（小端 0x40000005：size=5、subType=1 多次应答）
    function [7:0] size_byte;
        input [1:0] i;
        begin
            case (i)
                2'd0:    size_byte = 8'h05;
                2'd1:    size_byte = 8'h00;
                2'd2:    size_byte = 8'h00;
                default: size_byte = 8'h40;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt  <= 16'd0;
            hdr_cnt   <= 16'd0;
            last_type <= 8'd0;
            hdr_done  <= 1'b0;
            activity  <= 1'b0;
            hdr_seen  <= 1'b0;
            hst       <= H_IDLE;
            size_i    <= 2'd0;
        end else begin
            hdr_done <= 1'b0;
            if (valid) begin
                byte_cnt <= byte_cnt + 16'd1;      // ★自由回绕，不饱和★
                activity <= ~activity;             // ★每字节翻转，永不饱和★
                case (hst)
                    H_IDLE: begin
                        if (data == 8'hA5) hst <= H_A5;
                    end
                    H_A5: begin
                        if (data == 8'h5A) begin
                            hst    <= H_SIZE;
                            size_i <= 2'd0;
                        end else if (data == 8'hA5) begin
                            hst <= H_A5;           // A5 A5 5A… 也能重新对齐
                        end else begin
                            hst <= H_IDLE;
                        end
                    end
                    H_SIZE: begin
                        if (data != size_byte(size_i)) begin
                            hst <= H_IDLE;         // ★假头拦在这里：不算头、不清累加器★
                        end else if (size_i == 2'd3) begin
                            hst <= H_TYPE;
                        end else begin
                            size_i <= size_i + 2'd1;
                        end
                    end
                    H_TYPE: begin
                        hst <= H_IDLE;
                        if (data == EXPECT_TYPE) begin
                            last_type <= data;
                            hdr_done  <= 1'b1;     // 7 字节全对
                            hdr_cnt   <= hdr_cnt + 16'd1;
                            hdr_seen  <= 1'b1;
                        end
                    end
                    default: hst <= H_IDLE;
                endcase
            end
        end
    end

endmodule
