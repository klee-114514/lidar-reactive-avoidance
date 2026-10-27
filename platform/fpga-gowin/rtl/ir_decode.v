//=============================================================================
// ir_decode.v —— HT6221 / NEC 红外遥控接收解码
//-----------------------------------------------------------------------------
// 来源：《第22章 HT6221红外遥控解码设计与验证》（小梅哥）时序与阈值
// 时钟：50MHz（与 final 工程一致）
// 接收头输出与发射波形反相：
//   完整帧头 = 9ms 低 + 4.5ms 高
//   bit0     = 560us 低 + 560us 高
//   bit1     = 560us 低 + 1690us 高
//   帧体     = 16 位地址 + 8 位数据 + 8 位数据反码（各 LSB 先传）
//   结束     = 560us 低（stop），其上升沿打 dec_done
//   重复帧   = 9ms 低 + 2.25ms 高 + 560us 低（长按，无键码）→ ir_repeat
//=============================================================================
module ir_decode (
    input  wire        clk,      // 50MHz
    input  wire        rst_n,
    input  wire        iIR,      // 一体化接收头输出
    output reg  [15:0] ir_addr,  // 地址码（LSB 先入，对应 data_tmp[15:0]）
    output reg  [15:0] ir_data,  // {数据反码, 数据}，低 8 位为键码
    output reg         dec_done, // 完整帧解码完成，单周期脉冲
    output reg         ir_repeat // 长按重复帧，单周期脉冲
);

    // ---------------- 异步输入同步 + 边沿检测 ----------------
    reg iIR_sync1, iIR_sync2;
    reg iIR_reg1,  iIR_reg2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            iIR_sync1 <= 1'b1;
            iIR_sync2 <= 1'b1;
            iIR_reg1  <= 1'b1;
            iIR_reg2  <= 1'b1;
        end else begin
            iIR_sync1 <= iIR;
            iIR_sync2 <= iIR_sync1;
            iIR_reg1  <= iIR_sync2;
            iIR_reg2  <= iIR_reg1;
        end
    end

    wire iIR_pedge = ~iIR_reg2 &  iIR_reg1;
    wire iIR_nedge =  iIR_reg2 & ~iIR_reg1;

    // ---------------- 时间计数（50MHz）----------------
    reg        time_cnt_en;
    reg [18:0] time_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            time_cnt <= 19'd0;
        else if (!time_cnt_en)
            time_cnt <= 19'd0;
        else
            time_cnt <= time_cnt + 19'd1;
    end

    // 手册容差窗（计数值 @50MHz）
    wire T9ms_ok   = (time_cnt > 19'd325000) && (time_cnt < 19'd495000); // ~6.5~9.9ms
    wire T4_5ms_ok = (time_cnt > 19'd152500) && (time_cnt < 19'd277500); // ~3.05~5.55ms
    wire T_rep_ok  = (time_cnt > 19'd90000)  && (time_cnt < 19'd140000); // ~1.8~2.8ms 重复帧
    wire T_56ms_ok = (time_cnt > 19'd20000)  && (time_cnt < 19'd35000);  // ~0.4~0.7ms
    wire T1_69ms_ok= (time_cnt > 19'd75000)  && (time_cnt < 19'd90000);  // ~1.5~1.8ms
    wire timeout   = (time_cnt >= 19'd500000);                           // 10ms

    // ---------------- 状态机（独热码）----------------
    localparam [4:0]
        IDLE        = 5'b00001,
        LEADER_T9   = 5'b00010,
        LEADER_T4_5 = 5'b00100, // 4.5ms 完整帧头 或 2.25ms 重复帧头
        DATE_GET    = 5'b01000,
        REP_TAIL    = 5'b10000; // 重复帧的 560us 尾部

    reg [4:0] state;
    reg [5:0] data_cnt;
    reg [31:0] data_tmp;

    // 主状态机
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            time_cnt_en <= 1'b0;
        end else if (timeout) begin
            state      <= IDLE;
            time_cnt_en <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (iIR_nedge) begin
                        time_cnt_en <= 1'b1;
                        state      <= LEADER_T9;
                    end else begin
                        time_cnt_en <= 1'b0;
                        state      <= IDLE;
                    end
                end

                LEADER_T9: begin
                    if (iIR_pedge) begin
                        if (T9ms_ok) begin
                            time_cnt_en <= 1'b0;
                            state      <= LEADER_T4_5;
                        end else begin
                            time_cnt_en <= 1'b0;
                            state      <= IDLE;
                        end
                    end else begin
                        time_cnt_en <= 1'b1;
                        state      <= LEADER_T9;
                    end
                end

                LEADER_T4_5: begin
                    if (iIR_nedge) begin
                        if (T4_5ms_ok) begin
                            time_cnt_en <= 1'b0;
                            state      <= DATE_GET;
                        end else if (T_rep_ok) begin
                            time_cnt_en <= 1'b0;
                            state      <= REP_TAIL;
                        end else begin
                            time_cnt_en <= 1'b0;
                            state      <= IDLE;
                        end
                    end else begin
                        time_cnt_en <= 1'b1;
                        state      <= LEADER_T4_5;
                    end
                end

                REP_TAIL: begin
                    // 2.25ms 高之后的 560us 低结束 → 重复帧有效
                    if (iIR_pedge) begin
                        time_cnt_en <= 1'b0;
                        state      <= IDLE;
                    end else if (iIR_nedge) begin
                        time_cnt_en <= 1'b0;
                        state      <= IDLE;
                    end else begin
                        time_cnt_en <= 1'b1;
                        state      <= REP_TAIL;
                    end
                end

                DATE_GET: begin
                    // 异常退出必须同时关计数使能，避免污染下一帧的 T9 测量
                    if (iIR_pedge && !T_56ms_ok) begin
                        time_cnt_en <= 1'b0;
                        state      <= IDLE;
                    end else if (iIR_nedge && !T_56ms_ok && !T1_69ms_ok) begin
                        time_cnt_en <= 1'b0;
                        state      <= IDLE;
                    end else if (dec_done) begin
                        time_cnt_en <= 1'b0;
                        state      <= IDLE;
                    end else if (iIR_pedge && T_56ms_ok) begin
                        time_cnt_en <= 1'b0;
                    end else if (iIR_nedge && (T_56ms_ok || T1_69ms_ok)) begin
                        time_cnt_en <= 1'b0;
                    end else begin
                        time_cnt_en <= 1'b1;
                        state      <= DATE_GET;
                    end
                end

                default: begin
                    state      <= IDLE;
                    time_cnt_en <= 1'b0;
                end
            endcase
        end
    end

    // ---------------- 32bit 数据采集（下降沿判 0/1）----------------
    // 离开 DATE_GET（timeout/坏帧/完成）一律清零，防残帧污染下一帧
    wire frame_done_pulse = (state == DATE_GET) && iIR_pedge &&
                            (data_cnt == 6'd32) && T_56ms_ok;
    wire frame_abort = timeout ||
                       (state == DATE_GET && (
                           (iIR_pedge && !T_56ms_ok) ||
                           (iIR_nedge && !T_56ms_ok && !T1_69ms_ok)
                       ));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_cnt <= 6'd0;
            data_tmp <= 32'd0;
        end else if (frame_abort || state != DATE_GET) begin
            data_cnt <= 6'd0;
            data_tmp <= 32'd0;
        end else if (frame_done_pulse) begin
            data_cnt <= 6'd0;
        end else begin
            if (iIR_nedge)
                data_cnt <= data_cnt + 6'd1;
            if (iIR_nedge && T_56ms_ok)
                data_tmp[data_cnt] <= 1'b0;
            else if (iIR_nedge && T1_69ms_ok)
                data_tmp[data_cnt] <= 1'b1;
        end
    end

    // ---------------- 解码完成 / 重复帧 ----------------
    // 32bit 收齐且 stop 低电平落入 560us 窗，其上升沿才打 dec_done
    wire repeat_pulse = (state == REP_TAIL) && iIR_pedge && T_56ms_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dec_done  <= 1'b0;
            ir_repeat <= 1'b0;
            ir_addr   <= 16'd0;
            ir_data   <= 16'd0;
        end else begin
            dec_done  <= 1'b0;
            ir_repeat <= 1'b0;
            if (frame_done_pulse) begin
                dec_done <= 1'b1;
                ir_addr  <= data_tmp[15:0];   // 16 位地址
                ir_data  <= data_tmp[31:16];  // {反码, 数据}
            end
            if (repeat_pulse)
                ir_repeat <= 1'b1;
        end
    end

endmodule
