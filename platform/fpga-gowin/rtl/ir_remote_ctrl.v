//=============================================================================
// ir_remote_ctrl.v —— HT6221 键码 → 小车按键语义映射
//-----------------------------------------------------------------------------
// 按「车载 MP3 21 键」丝印映射，物理键 S0~S4 仍独立存在。
// 键码为 HT6221 标准值，实测不符可改参数。
//
// 丝印 → 动作：
//   【队列任务】
//   CH+ (47h)        入队前进 50cm     ↔ S2 单击
//   PREV (44h)       入队左转          ↔ S3
//   NEXT (40h)       入队右转          ↔ S4
//   PLAY/PAUSE (43h) 执行队列          ↔ S1 单击
//   CH- (45h)        全局急停；长按=hold ↔ S0 按住急停
//   EQ  (09h)        清空队列          ↔ S1 双击
//   1 (0Ch)          前进 10cm
//   2 (18h)          前进 20cm
//   5 (1Ch)          前进 50cm
//   0 (16h)          取消当前行程
//   【手柄点动】按住=1，松手（或超时）=0；与队列键物理分离，无模式键
//   VOL+ (15h)       前进
//   VOL- (07h)       后退
//   100+ (19h)       原地左转
//   200+ (0Dh)       原地右转
//
// 长按：ir_repeat 刷新 stop_hold / 手柄电平
//=============================================================================
module ir_remote_ctrl #(
    parameter [15:0] EXP_ADDR = 16'hFF00, // 期望地址（ADDR_CHK=1 时生效）
    parameter        ADDR_CHK = 0,        // 1=校验地址，0=只认键码
    // 车载 MP3 / HT6221 标准键码
    parameter [7:0] KEY_FWD    = 8'h47, // CH+  50cm
    parameter [7:0] KEY_LEFT   = 8'h44, // PREV
    parameter [7:0] KEY_RIGHT  = 8'h40, // NEXT
    parameter [7:0] KEY_EXEC   = 8'h43, // PLAY/PAUSE
    parameter [7:0] KEY_STOP   = 8'h45, // CH-
    parameter [7:0] KEY_CLEAR  = 8'h09, // EQ
    parameter [7:0] KEY_D1     = 8'h0C, // 1 → 10cm
    parameter [7:0] KEY_D2     = 8'h18, // 2 → 20cm
    parameter [7:0] KEY_D5     = 8'h1C, // 5 → 50cm
    parameter [7:0] KEY_CANCEL = 8'h16, // 0 → 取消行程
    parameter [7:0] KEY_GP_FWD = 8'h15, // VOL+
    parameter [7:0] KEY_GP_BAK = 8'h07, // VOL-
    parameter [7:0] KEY_GP_L   = 8'h19, // 100+
    parameter [7:0] KEY_GP_R   = 8'h0D, // 200+
    // 电平保持时间（松开后/无 repeat 时超时清零）
    parameter [23:0] HOLD_CYC  = 24'd10_000_000  // ~200ms @50MHz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] ir_addr,
    input  wire [15:0] ir_data,
    input  wire        dec_done,
    input  wire        ir_repeat,
    // 单周期命令脉冲（队列任务）
    output reg         cmd_fwd,     // 前进（dist_sel 指出长度）
    output reg  [1:0]  dist_sel,    // 0:50cm  1:20cm  2:10cm
    output reg         cmd_left,
    output reg         cmd_right,
    output reg         cmd_exec,
    output reg         cmd_stop,
    output reg         cmd_clear,
    output reg         cmd_cancel,
    // CH- 长按电平：repeat 期间为 1，松开后保持 HOLD_CYC
    output reg         stop_hold,
    // 手柄点动电平：按住=1，松开/超时=0（互斥由上层仲裁）
    output reg         gp_fwd,
    output reg         gp_back,
    output reg         gp_left,
    output reg         gp_right
);

    wire [7:0] key_code = ir_data[7:0];
    wire       addr_ok  = ADDR_CHK ? (ir_addr == EXP_ADDR) : 1'b1;
    wire       key_ok   = dec_done && addr_ok;

    // 锁存最近一次完整键码，供 repeat 沿用
    reg [7:0] last_key;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            last_key <= 8'h00;
        else if (key_ok)
            last_key <= key_code;
    end

    // stop_hold：仅长按 repeat 维持；单按 CH- 只出 cmd_stop（estop 锁存）
    //           ir_clear 同步清除 hold，避免清急停后仍被按住
    reg [23:0] hold_cnt;
    wire       rep_stop = ir_repeat && (last_key == KEY_STOP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hold_cnt  <= 24'd0;
            stop_hold <= 1'b0;
        end else if (key_ok && (key_code == KEY_CLEAR || key_code == KEY_CANCEL)) begin
            hold_cnt  <= 24'd0;
            stop_hold <= 1'b0;
        end else if (rep_stop) begin
            hold_cnt  <= HOLD_CYC;
            stop_hold <= 1'b1;
        end else if (stop_hold) begin
            if (hold_cnt == 24'd0)
                stop_hold <= 1'b0;
            else
                hold_cnt <= hold_cnt - 24'd1;
        end
    end

    // 手柄点动：完整帧/重复帧刷新对应电平，HOLD_CYC 超时松开
    // 四路单热，同一时刻只认最近一键
    reg        gp_any;
    reg [23:0] gp_hold;

    wire gp_hit = key_ok && (key_code == KEY_GP_FWD || key_code == KEY_GP_BAK ||
                             key_code == KEY_GP_L   || key_code == KEY_GP_R);
    wire gp_rep = ir_repeat && (last_key == KEY_GP_FWD || last_key == KEY_GP_BAK ||
                                last_key == KEY_GP_L   || last_key == KEY_GP_R);

    wire gp_is_fwd  = (gp_hit && (key_code == KEY_GP_FWD)) || (gp_rep && (last_key == KEY_GP_FWD));
    wire gp_is_bak  = (gp_hit && (key_code == KEY_GP_BAK)) || (gp_rep && (last_key == KEY_GP_BAK));
    wire gp_is_l    = (gp_hit && (key_code == KEY_GP_L))   || (gp_rep && (last_key == KEY_GP_L));
    wire gp_is_r    = (gp_hit && (key_code == KEY_GP_R))   || (gp_rep && (last_key == KEY_GP_R));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gp_any   <= 1'b0;
            gp_hold  <= 24'd0;
            gp_fwd   <= 1'b0;
            gp_back  <= 1'b0;
            gp_left  <= 1'b0;
            gp_right <= 1'b0;
        end else if (gp_hit || gp_rep) begin
            gp_any   <= 1'b1;
            gp_hold  <= HOLD_CYC;
            gp_fwd   <= gp_is_fwd;
            gp_back  <= gp_is_bak;
            gp_left  <= gp_is_l;
            gp_right <= gp_is_r;
        end else if (gp_any) begin
            if (gp_hold == 24'd0) begin
                gp_any   <= 1'b0;
                gp_fwd   <= 1'b0;
                gp_back  <= 1'b0;
                gp_left  <= 1'b0;
                gp_right <= 1'b0;
            end else
                gp_hold <= gp_hold - 24'd1;
        end
    end

    // 命令脉冲：完整帧才出；repeat 只维持 stop_hold / 手柄电平
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_fwd    <= 1'b0;
            dist_sel   <= 2'd0;
            cmd_left   <= 1'b0;
            cmd_right  <= 1'b0;
            cmd_exec   <= 1'b0;
            cmd_stop   <= 1'b0;
            cmd_clear  <= 1'b0;
            cmd_cancel <= 1'b0;
        end else begin
            cmd_fwd    <= 1'b0;
            cmd_left   <= 1'b0;
            cmd_right  <= 1'b0;
            cmd_exec   <= 1'b0;
            cmd_stop   <= 1'b0;
            cmd_clear  <= 1'b0;
            cmd_cancel <= 1'b0;

            if (key_ok) begin
                if (key_code == KEY_FWD || key_code == KEY_D5) begin
                    cmd_fwd  <= 1'b1;
                    dist_sel <= 2'd0; // 50cm
                end else if (key_code == KEY_D2) begin
                    cmd_fwd  <= 1'b1;
                    dist_sel <= 2'd1; // 20cm
                end else if (key_code == KEY_D1) begin
                    cmd_fwd  <= 1'b1;
                    dist_sel <= 2'd2; // 10cm
                end else if (key_code == KEY_LEFT) begin
                    cmd_left <= 1'b1;
                end else if (key_code == KEY_RIGHT) begin
                    cmd_right <= 1'b1;
                end else if (key_code == KEY_EXEC) begin
                    cmd_exec <= 1'b1;
                end else if (key_code == KEY_STOP) begin
                    cmd_stop <= 1'b1;
                end else if (key_code == KEY_CLEAR) begin
                    cmd_clear <= 1'b1;
                end else if (key_code == KEY_CANCEL) begin
                    cmd_cancel <= 1'b1;
                end
            end
        end
    end

endmodule
