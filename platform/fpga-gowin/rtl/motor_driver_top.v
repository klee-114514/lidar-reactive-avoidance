//=============================================================================
// motor_driver_top.v —— 顶层：动作队列执行（前进/差速转向）+ 纯闭环
//-----------------------------------------------------------------------------
// 【按键总表】（2026-09-26 定稿）
//   S0 (F15)  单击 = 软复位（清空序列、回待机、清急停锁存）
//             双击 = 急停锁存（松手仍停，直到 S0 单击解除）
//             按住 = 急停（立即断电，保底；松手不续跑）
//   S1 (A20)  单击 = 执行当前序列
//             双击 = 清空序列
//   S2 (B20)  单击 = 入队 前进 10cm
//             双击 = 入队 前进 20cm
//             长按 = 入队 前进 5cm
//   S3 (A21)  单击 = 入队 原地左转 90°
//             双击 = 入队 弧线左转
//             长按 = 入队 原地左转 45°
//   S4 (B21)  单击 = 入队 原地右转 90°
//             双击 = 入队 弧线右转
//             长按 = 入队 原地右转 45°
//   ★ S4 已从硬复位改为模式键；硬复位仅上电（内部 POR）★
//
// 【序列】最多 4 步，按入队顺序执行；跑完一段自动下一段，队列空回待机。
//   例：S2 单击 → S3 单击 → S1 单击 = 前进 10cm 后原地左转 90°
//
// 【定量里程】无需标定，按 500 线 / 1:60 / 轮径 66.5mm / 轮距 220mm：
//   COUNTS_PER_MM = 500×4×60 / (π×66.5) ≈ 574
//   前进 5/20cm = 28700 / 114800（final 里单击 50cm=287000）
//   原地 45°/90°  = 50800 / 101600   （每轮弧长 = 轮距/2 × θ）
//   弧线段默认 58800，角度需按转弯半径另标
//
// 【差速】
//   前进     +V / +V
//   原地左   -V / +V     原地右   +V / -V
//   弧线左   +V/2 / +V   弧线右   +V / +V/2
//   再套 LEFT_NEG/RIGHT_NEG（实测 1/1，同序安装）
//
// 【安全】
//   S0 按住或双击 → pwm_enable=0；状态机踢回 IDLE，不续跑
//   SAFE_MS 超时：编码器坏了也必停
//
// 【LED】
//   led[2:0] 队列剩余步数 0~4
//   led[3]   正在执行
//   led[4]   急停锁存
//   led[5]/[6] 左/右轮在转
//   led[7]   心跳
//=============================================================================
module motor_driver_top #(
    parameter CLK_FREQ = 50_000_000,
    parameter PWM_FREQ = 20_000,
    parameter DUTY_MAX = 1000,
    parameter SPEED_MS = 5,
    // ★ 2026-09-28 提速度：原来 40（≈14mm/s，满速 2%）⇒ 电机工作在静摩擦死区边上，
    //   表现为"软绵无力"，且速度环在死区附近容易抖。
    //   仿真（sim/tb_speed.v，按实测极性建模）：400 配 KP=128/KI=4 稳稳跟住（≈130mm/s），
    //   占空比 ~300-400‰，远离死区；KP 降到 32 就够不到目标了。
    // ★V2.13★：360 → 500；★V2.14★：**500 → 420**（≈146mm/s，比原来 360 快一档就够）。
    //   依据（用户 00:02 日志）：提速后原地转冲过头最多 ~90°（航向 -13°→+74.6°，目标 +16.9°），
    //   收不进到位窗口 ⇒ 3s 看门狗把好转身砍成 w=31 失败 ⇒ "一开始就来回转 90 度"。所以回退。
    //   换算：1 计数/5ms ≈ 0.347mm/s。
    // ★V2.15★：420 → 560（≈194mm/s）。巡航/直行不吃转身收角，可以再提。
    // ★V4.35j★ 用户要求：未进入避障时的巡航速度提到原来的 1.5 倍，并且要能看出
    //   "触发避障时有速度变化"。原来 560 ≈ 194mm/s（现场 SL≈0x0160），×1.5 ⇒ 840。
    //   只动这一处：`sel_spd = run_slow ? (ext_active?AVOID_SPD:SLOW_SPD) : tgt_eff`
    //   ⇒ 减速区(SLOW_SPD=320)、绕障腿(AVOID_SPD=360)、原地转(TURN_SPD=520) 全部不受影响。
    parameter signed [15:0] TARGET_SPD = 16'sd840,
    // ★2026-10-10 降速 40%（用户要求 ×0.6）★
    //   编码器 574 counts/mm ⇒ 360 counts/5ms = 72000 counts/s ≈ 125mm/s
    //   原来的 600 ≈ 209mm/s 对 30×25cm 的车来说太快，避障时来不及反应。
    //   注意死区：稳态占空比 ≈ 死区(~150‰) + 速度/2.35 ⇒ 360 约需 303‰，仍有余量；
    //   若串口看到 SL/SR 明显跟不上 360，就把 KP 从 64 提到 96。
    // ★V2.13★：216→300、288→420；★V2.14★ 回退：**300→260、420→288**（≈90 与 100mm/s）
    // ★V2.15★：260→320（≈111mm/s）、288→360（≈125mm/s）
    parameter signed [15:0] SLOW_SPD  = 16'sd320,   // 减速档（前方进减速区）≈111mm/s
    parameter signed [15:0] AVOID_SPD = 16'sd360,   // 绕障腿（ext_slow=1）≈125mm/s
    // 原地转：上限 450（原 750 的 60%）。只在【最后约 30°】压到 TURN_SPD_MIN（=70%），
    //   前段保持全速 —— 减速不能太肉、太明显；到位用 yaw 提前量硬停，不爬行。
    // ★V2.13★：450→600、315→420；★V2.14★ 回退/再收紧：**600→450、420→280**。
    //   收角速度比原来(315)还低一档：末端慢下来才能收进 ±3.85° 的到位窗口，不再冲过头来回摆。
    // ★V2.15★：**两段式原地转**——快段 340~520，最后 CRAWL_ANG_CNT(20°) 降到 CRAWL_TURN_SPD(130) 爬行收角。
    //   为什么：00:17 日志里"末端还在 ~97mm/s 上硬刹"⇒ 每次冲过头 13~18° ⇒ 补转/摆头。
    //   注意 CRAWL_ANG_CNT 是【收角用的伺服参数】，不是策略角度。
    parameter signed [15:0] TURN_SPD   = 16'sd520,
    parameter signed [15:0] TURN_SPD_MIN = 16'sd340,
    parameter signed [15:0] CRAWL_TURN_SPD = 16'sd130,
    parameter [15:0] CRAWL_ANG_CNT = 16'd3640,      // 20°（1°=182 计数）
    // ★V5.0e★ S3/S4 串级角度环参数。
    //   外环输入是“编码器 5ms 角速度积分 + IMU 10Hz 绝对角校正”，
    //   输出单位与轮速目标一致（counts/5ms），再交给现有左右轮速度 PID。
    //   Kp/Ki/Kd 都按 Q8 定点计算：
    //     p_term = err × ANG_KP >> 8
    //   err=16384(90°)、ANG_KP=8 时 p_term=512，正好接近 TURN_SPD。
    parameter signed [15:0] ANG_KP = 16'sd8,
    parameter signed [15:0] ANG_KI = 16'sd1,
    parameter signed [15:0] ANG_KD = 16'sd32,       // 0.125，对角速度做弱阻尼
    parameter signed [15:0] ANG_SLEW = 16'sd40,     // 每 5ms 最大变化量
    parameter [15:0] ANG_TOL_CNT = 16'd182,         // 到位带 ±1°
    parameter [15:0] ANG_STILL_RATE = 16'd2,        // 约 <=1.1°/s
    parameter [15:0] ANG_STILL_MS = 16'd20,
    parameter signed [31:0] ANG_I_LIM = 32'sd8192,  // 积分限幅
    // 绕障时的原地转压到 75%（turn_base×3/4）：90° 约 1.7s，既不甩也不肉
    parameter [15:0] AVOID_TURN_NUM = 16'd3,
    parameter [15:0] AVOID_TURN_SH  = 16'd2,
    // ★2b-1（2026-10-15）★ yaw 提前量：2800(15.4°) → 700(≈3.8°)、1024(5.6°) → 400(≈2.2°)
    //   理由：避障改成"按测量算一个小角度"（可能只有 20~30°）之后，15.4° 的提前量会让
    //   "要 27° 只到 12°"。改小后：命令多少就是多少（IMU 闭环会一直修到残差 ≤ 700 才收工）。
    //   非末段（队列还有后续）仍用更小的 400：到角度直接切下一段，不做减速急停。
    // ★V2.28a★：900(4.9°) → 400(2.2°)。FSM 的 YAW_TOL_CNT=4°，
    //   电机侧必须比它更早收住，否则每次都会判“没到位”再补转。
    parameter [15:0] YAW_STOP_M = 16'd400,
    parameter [15:0] YAW_STOP_S = 16'd400,
    // ★2026-10-26★ A_YAW_ABS 的【方向滞回】门限（raw 计数，182/度）。
    //   问题：A_YAW_ABS 的转向方向由 ya_pos（误差符号）直接决定，车快到位时
    //   IMU 在目标附近抖动 ⇒ 符号来回翻 ⇒ v_l/v_r 正负互换 ⇒ 原地左右摆。
    //   现场证据（22:22 日志）：S_T_END 本该转 30° 收工，实际转了 3.2s，
    //   SL/SR 正负翻转 4 次（FEC2/0192 → 013A/FED5 → FF82/007C → 006D/FF8C）。
    //   对策：误差 > YA_HYST_CNT 才允许换向，否则保持上次方向。
    //   5° = 910 计数：大于 IMU 抖动，小于正常转向的初始误差（≥15°）。
    parameter [15:0] YA_HYST_CNT = 16'd910,
    // ★V2.7（清单7）★：原地转之前先【停稳】多久（两轮命令压 0，PID 主动刹住）。
    //   现场依据：静止起转能到 90°，边跑边转只到 ~60% ⇒ 先停住再转。
    //   ★不能太大★：顶层堵转检测是"ext 段里两轮不动 400ms"⇒ 200ms 留 200ms 余量
    //   （avoid_fsm 另有"段头 600ms 不认 stall"的配套改动）。
    // ★V2.9 改语义★：这 200ms 不再是"两轮压 0 停死"，而是【极低速起转】的软化窗口
    //   （用户 2026-10-15 原话：「不要停，停顿非常明显，可以用降低速度来代替」）。
    //   窗口内两轮按转向方向对着给 ±SOFT_TURN_SPD（很低速）；窗口过后照原来的
    //   角度曲线加速转。低速下轮胎拖拽小 ⇒ 精度仍接近"静止起转"，
    //   再叠加"转完 IMU 回读、超 4° 重发"（avoid_fsm 侧）作双保险。
    parameter [15:0] DWELL_MS   = 16'd150,   // ★V2.15★ 200 → 150ms（每转身省 50ms）
    // ★V2.9★：软化窗口里的"起转速度"（计数/5ms）。40 ≈ 15mm/s（tgt_eff=360 的 1/9）。
    //   要更稳就调小（但不能为 0，否则又变成"停死"）。
    parameter signed [15:0] SOFT_TURN_SPD = 16'sd40,   // 起转软化（仍不停死）
    // ---- ★2b-1：弧长判据的 IMU 门控 + A_FWD_HOLD（前进并保持指定绝对航向）★ ----
    //   IMU_ARC_GATE=1：IMU 在线时"两轮平均弧长"不再参与【原地转】的段完成判据
    //     （只保留作 IMU 掉线时的兜底）。现场实测：车还在跑时下令原地转，轮子被地面拖着
    //     多滚了十几万计数，弧长判据 1.4s 就满足、而车身只转了 34°（命令 90°）。
    //     置 0 = 回到旧行为（只给回归对拍用，正常都保持 1）。
    //   ★直行段（非 pivot）的"弧长=走了多远"完全不动，仍以弧长为主判据★
    parameter IMU_ARC_GATE = 1'b1,
    //   A_FWD_HOLD 的航向保持修正：err>>>HOLD_SH，死区 HOLD_GATE(raw)，限幅 ±HOLD_LIM
    //     它只是"微调"：限幅后最大差速 ±120 计数/5ms（≈±21mm/s），绝不会变成原地转。
    parameter HOLD_SH     = 4,
    // ★V2.13★：16(0.09°) → 120(0.66°)：原来几乎无死区，轮子在追 IMU 噪声 ⇒ 斜行时一直微摆。
    parameter [15:0] HOLD_GATE = 16'd250,   // ★V2.15★ 120(0.66°)→250(1.4°)：别追最后 1°（摆动源之一）
    // ★V2.18y★：120 → 80。现场斜行航向在 33°→53°→22°→35° 来回摆，
    //   A_FWD_HOLD 的差速修正过强、欠阻尼；先降限幅看摆幅是否收窄。
    parameter signed [15:0] HOLD_LIM = 16'sd80,
    // 1=原地左转时 yaw 递增（按 IMU 安装）；若左右转完成判据反了就取反
    parameter YAW_LEFT_POS = 1'b1,
    // 直线航向辅助（IMU 外环）：corr = (yaw_err>>>HEAD_SH)，限幅 ±HEAD_LIM
    //   2026-09-29 实车仍偏：5→3、80→120，加大纠偏
    parameter HEAD_SH      = 3,
    parameter signed [15:0] HEAD_LIM = 16'sd150,   // ★V2.15★ 120→150：提速后保持力度补一档
    // Yaw 原始值：90°/45°（raw/32768*180°）
    parameter [15:0] YAW_90_RAW = 16'd16384,
    parameter [15:0] YAW_45_RAW = 16'd8192,
    parameter [15:0] KP = 16'd64,
    parameter [15:0] KI = 16'd4,
    parameter [15:0] KD = 16'd0,
    // ---- 理论段长（4 倍频计数），见文件头换算 ----
    parameter N_FWD_5   = 32'd28700,
    parameter N_FWD_10  = 32'd57400,
    parameter N_FWD_20  = 32'd114800,
    parameter N_TURN_45 = 32'd50800,
    parameter N_TURN_90 = 32'd101600,
    parameter N_ARC     = 32'd57400,
    parameter SAFE_MS   = 10000,
    // 原地转的保底超时（yaw 坏掉/掉帧时兜底）—— 标称 90° 只要 ~1 秒，给 5 秒足够
    parameter PIVOT_MS  = 5000,
    // ---- 机械参数（文档/预估，不进运行时乘除）----
    parameter ENC_LINES    = 500,
    parameter GEAR_RATIO   = 60,
    parameter WHEEL_DIA_MM = 66.5,
    parameter TRACK_MM     = 220,
    parameter DOUBLE_MS    = 400,
    parameter LONG_MS      = 800
)(
    input  wire       clk,
    input  wire       force_stop,      // 1=避障急停：冻结 PID 并切断 STBY（融合用，原工程无此脚）
    input  wire       slow_req,        // 1=前方进入减速区/正前样本失效 → 用 SLOW_SPD 跑
    input  wire       ext_slow,        // 1=当前 ext 段（绕障腿）→ 用 AVOID_SPD + 半速转
    input  wire [15:0] yaw_raw,        // IMU Yaw 原始值（0x53 帧 int16）
    input  wire        yaw_valid,
    input  wire       key_stop,        // S0 复位/急停，按下=0
    input  wire       key_start,       // S1 执行，按下=0
    input  wire       key_fwd,         // S2 前进入队，按下=0
    input  wire       key_left,        // S3 左转入队，按下=0
    input  wire       key_r,           // S4 右转入队，按下=0
    // 红外遥控命令（与物理键并联；单周期脉冲，来自 ir_remote_ctrl）
    input  wire       ir_fwd,          // 前进入队（长度看 ir_dist）
    input  wire [1:0] ir_dist,         // 0:50cm  1:20cm  2:10cm
    input  wire       ir_left,         // 左转入队 90°
    input  wire       ir_right,        // 右转入队 90°
    input  wire       ir_exec,         // 执行队列
    input  wire       ir_stop,         // 急停锁存（=S0 双击）
    input  wire       ir_clear,        // 清队列/软复位（=S1 双击 / S0 单击）
    input  wire       ir_cancel,       // 取消当前行程（清队 + 急停解除后再跑需重新入队）
    input  wire       ir_stop_hold,    // 长按 CH-：强制断电（=S0 按住）
    // 手柄点动电平（VOL+/VOL-/100+/200+ 按住=1）；S_RUN 时本模块忽略
    input  wire       gp_fwd,
    input  wire       gp_back,
    input  wire       gp_left,
    input  wire       gp_right,
    input  wire [7:0] sw,
    output wire       ain1_l,
    output wire       ain2_l,
    output wire       pwma,
    output wire       ain1_r,
    output wire       ain2_r,
    output wire       pwmb,
    output wire       stby,
    input  wire       enc_al,
    input  wire       enc_bl,
    input  wire       enc_ar,
    input  wire       enc_br,
    output wire [7:0] led,
    // 融合调试：编码器计数 / 5ms 速度（供长度、车速换算）
    output wire [31:0] dbg_cnt_l,
    output wire [31:0] dbg_cnt_r,
    output wire [15:0] dbg_spd_l,
    output wire [15:0] dbg_spd_r,
    // 诊断：相对【本段起点】的 |yaw 变化|（raw）。原地转 90° 时应到 16384。
    // 打印出来就能判断"转弯差角度"到底是陀螺仪标度不对还是编码器判据不对。
    output wire [15:0] dbg_yaw,
    // ★2026-10-15 阶段0 临时调试（只加输出，不改任何逻辑）★
    //   目的：定位"避障里的原地转为什么变成边走边转"。
    //   t 打印出来如果是"一正一负"才说明真的在下原地转命令；若两轮同向就是命令没生效。
    output wire [2:0]  dbg_act,        // cur_act：0=前进 1=左转 2=右转 3=左弧 4=右弧 5=绝对航向原地转
    output wire [1:0]  dbg_st,         // st：0=IDLE 1=S_RUN 2=S_EXT_RUN 3=S_EXT_WAIT
    output wire [15:0] dbg_turn,       // turn_cmd（补码；原地转/弧行的转向命令）
    // 软复位/清队列脉冲：final_top 用来一并清避障锁存
    output wire        clr_out,
    // ---- L 形绕障外部动作口（2026-10-02）----
    input  wire        ext_load,
    input  wire [2:0]  ext_act,        // 0=FWD 1=L 2=R 5=YAW_ABS
    input  wire [31:0] ext_n,
    input  wire [15:0] ext_yaw_tgt,    // A_YAW_ABS 目标航向
    input  wire        ext_to_queue,
    output wire        ext_done,
    output wire        ext_active,
    output wire [31:0] remain_out,     // 被打断 FWD 剩余（偏保守）
    output wire        remain_cap,
    // ★V2.7（清单6）★：本段【实测弧长】（编码器计数，段内清零）——
    //   就是段完成判据用的那个量（rot_avg_r）⇒ 避障记账与"段走完没"用同一把尺子。
    output wire [31:0] ext_arc_cnt
);

    // =========================================================================
    // 声明
    // =========================================================================
    reg  [7:0]  por_cnt  = 8'd0;
    reg         por_done = 1'b0;
    wire        rst_n;

    wire        pwm_enable;
    reg         estop_lat;
    wire        s0_single, s0_double, s0_long, s0_level;
    wire        s1_single, s1_double, s1_long, s1_level;
    wire        s2_single, s2_double, s2_long, s2_level;
    wire        s3_single, s3_double, s3_long, s3_level;
    wire        s4_single, s4_double, s4_long, s4_level;
    wire        run_pulse;
    wire        soft_clr;

    localparam Q_MAX = 4;
    reg  [2:0]  q_act [0:Q_MAX-1];
    reg  [31:0] q_n   [0:Q_MAX-1];
    reg  [1:0]  q_rd, q_wr;
    reg  [2:0]  q_cnt;

    reg  [1:0]  st;              // 0=IDLE 1=RUN 2=EXT_RUN 3=EXT_WAIT
    reg  [2:0]  cur_act;
    reg  [31:0] cur_n;
    reg         seg_last;   // 1=队列里最后一段：段末减速急停；0=直接衔接下一段
    reg  [31:0] run_cnt;
    reg  [31:0] cnt_l0, cnt_r0;
    wire [31:0] dist_l, dist_r;
    wire        seg_done;
    wire        pop_req;
    // external avoid
    reg  [31:0] remain_saved;
    reg         remain_val_r, ext_done_r, remain_cap_r, ext_to_q_r;
    reg  [15:0] yaw_ext_tgt_r;
    wire [31:0] prog_min = (dist_l < dist_r) ? dist_l : dist_r;
    wire [31:0] remain_now = (cur_n > prog_min) ? (cur_n - prog_min) : 32'd0;
    assign remain_out = remain_val_r ? remain_saved : remain_now;
    assign remain_cap = remain_cap_r;
    assign ext_done   = ext_done_r;
    wire [2:0]  head_act;
    wire [31:0] head_n;

    wire signed [15:0] duty_pid_l, duty_pid_r;
    wire signed [15:0] target_l, target_r;
    wire        dir_l, dir_r;
    wire [31:0] cnt_l, cnt_r;
    wire [15:0] spd_l, spd_r;
    wire        strobe_l, strobe_r;
    // ---- 反馈反向（等效于把该路编码器 A/B 对调）：声明+赋值见 §9 例化之后的 iv) 段 ----
    wire signed [15:0] spd_l_eff, spd_r_eff;
    reg  [31:0] cnt_l_eff, cnt_r_eff;
    reg  signed [15:0] spd_l_prt, spd_r_prt;
    wire        pid_run;
    reg  [24:0] hb_cnt;
    reg         heartbeat;
    reg signed [15:0] v_l, v_r;

    localparam A_FWD       = 3'd0;
    localparam A_LEFT_PIV  = 3'd1;
    localparam A_RIGHT_PIV = 3'd2;
    localparam A_LEFT_ARC  = 3'd3;
    localparam A_RIGHT_ARC = 3'd4;
    localparam A_YAW_ABS   = 3'd5;   // 绝对航向原地转
    localparam A_FWD_HOLD  = 3'd6;   // ★2b-1：前进 + 保持指定绝对航向（边走边修，不原地转）★

    localparam S_IDLE    = 2'd0;
    localparam S_RUN     = 2'd1;
    localparam S_EXT_RUN = 2'd2;
    localparam S_EXT_WAIT= 2'd3;
    assign ext_active = (st == S_EXT_RUN);

    localparam RUN_CYC    = CLK_FREQ / 1000 * SAFE_MS;
    localparam PIVOT_CYC  = CLK_FREQ / 1000 * PIVOT_MS;
    localparam [31:0] DWELL_CYC = (CLK_FREQ / 1000) * DWELL_MS;   // ★V2.7（清单7）★
    // =========================================================================
    // 【方向三件套】—— 调车时最容易互相抵消、越调越乱的地方，务必分清：
    //   ① 这里的 LEFT_NEG / RIGHT_NEG ：只决定【目标速度的符号】= 车往哪边走。
    //        改它【不会】动速度环反馈极性，所以方向不对时改这里最安全。
    //   ② TB6612 → 电机的两根线（AO1/AO2、BO1/BO2）：决定"正占空比"驱动轮子往哪转。
    //   ③ 编码器 A/B 两根线（或 encoder_decode 的 INVERT=1）：决定"轮子正转"时计数正负。
    //   速度环稳定的充要条件是 ②③ 的符号同号（ε·σ > 0），与 ① 无关；
    //   车往哪边走取决于 ① 与 ③ 的搭配。
    //   ⇒ 所以：方向反了改 ①；车"窜出去停不下来/速度不受控"才是 ②③ 反了。
    //     千万不要用"对调电机两根线"来修方向 —— 那会把本来正确的反馈极性一起翻掉。
    //
    // ★ 2026-09-28：实测 A_FWD 时车往后走 ⇒ 编码器计数正向与车体前进同向，
    //   正确的搭配是 0/0（原来写 1/1）。同时新增 sw[6]/sw[7] 在线单轮反向。
    // =========================================================================
    //   左路：ε=+1（轮子往前 → 计数正）、σ=−1（正占空比 → 轮子往后）⇒ 反馈要反向
    //   右路：实测反向时满速冲（≈316mm/s）⇒ 它本身 ε·σ 就是 +1 ⇒ 反馈【不】反向
    //   配对：A 路驱动左轮、左轮的运动出现在 enc_al/bl ⇒ 默认要【不交叉】
    //   方向：ε'=−1 时前进要求 target 为负 ⇒ LEFT_NEG/RIGHT_NEG = 1
    //   ⇒ 于是"所有拨码都不拨"就是正确的整车配置，开关只作现场微调。
    // ★恢复《工程经验》§3.1 / 约束手册编译默认（全拨码不拨=正确）★
    // ENC_SWAP=0，INV_L=1/INV_R=0（左右不对称），LEFT_NEG/RIGHT_NEG=1/1
    // 方向不对只改 LEFT_NEG/RIGHT_NEG；禁止对调电机线
    localparam ENC_SWAP_DEF = 1'b0;
    localparam INV_L_DEF    = 1'b1;
    localparam INV_R_DEF    = 1'b0;
    localparam LEFT_NEG     = 1'b1;
    localparam RIGHT_NEG    = 1'b1;

    // =========================================================================
    // 0) 上电复位（S4 不再硬复位）
    // =========================================================================
    always @(posedge clk) begin
        if (!por_done) begin
            if (por_cnt == 8'hFF)
                por_done <= 1'b1;
            else
                por_cnt <= por_cnt + 8'd1;
        end
    end
    assign rst_n = por_done;

    // =========================================================================
    // 1) 安全联锁
    // =========================================================================
    assign pwm_enable = (~s0_level) & (~estop_lat) & (~ir_stop_hold);
    assign stby       = pwm_enable;

    // =========================================================================
    // 2) 按键：S0~S4
    // =========================================================================
    key_triple #(.CLK_FREQ (CLK_FREQ), .DEBOUNCE_MS (20), .DOUBLE_MS (DOUBLE_MS),
                 .LONG_MS (LONG_MS), .ACTIVE_LOW (1)) u_k0 (
        .clk (clk), .rst_n (rst_n), .key_in (key_stop),
        .click_single (s0_single), .click_double (s0_double),
        .click_long (s0_long), .key_level (s0_level)
    );
    key_triple #(.CLK_FREQ (CLK_FREQ), .DEBOUNCE_MS (20), .DOUBLE_MS (DOUBLE_MS),
                 .LONG_MS (LONG_MS), .ACTIVE_LOW (1)) u_k1 (
        .clk (clk), .rst_n (rst_n), .key_in (key_start),
        .click_single (s1_single), .click_double (s1_double),
        .click_long (s1_long), .key_level (s1_level)
    );
    key_triple #(.CLK_FREQ (CLK_FREQ), .DEBOUNCE_MS (20), .DOUBLE_MS (DOUBLE_MS),
                 .LONG_MS (LONG_MS), .ACTIVE_LOW (1)) u_k2 (
        .clk (clk), .rst_n (rst_n), .key_in (key_fwd),
        .click_single (s2_single), .click_double (s2_double),
        .click_long (s2_long), .key_level (s2_level)
    );
    key_triple #(.CLK_FREQ (CLK_FREQ), .DEBOUNCE_MS (20), .DOUBLE_MS (DOUBLE_MS),
                 .LONG_MS (LONG_MS), .ACTIVE_LOW (1)) u_k3 (
        .clk (clk), .rst_n (rst_n), .key_in (key_left),
        .click_single (s3_single), .click_double (s3_double),
        .click_long (s3_long), .key_level (s3_level)
    );
    key_triple #(.CLK_FREQ (CLK_FREQ), .DEBOUNCE_MS (20), .DOUBLE_MS (DOUBLE_MS),
                 .LONG_MS (LONG_MS), .ACTIVE_LOW (1)) u_k4 (
        .clk (clk), .rst_n (rst_n), .key_in (key_r),
        .click_single (s4_single), .click_double (s4_double),
        .click_long (s4_long), .key_level (s4_level)
    );

    assign soft_clr  = s0_single | s1_double | ir_clear | ir_cancel;
    assign clr_out   = soft_clr;

    // =========================================================================
    // 2b) 调试 / 整定开关（拨码 sw，约束里 PULL_MODE=UP → 【什么都不拨 = 全 1 = 原逻辑】）
    //-----------------------------------------------------------------------------
    //   sw[0] = 1（默认）      → 完全正常，下面这些一个都不生效 ★
    //   sw[0] = 0（0 号拨到 ON）→ 进入调试（LED7 会从 1s 慢闪变成 0.125s 快闪）：
    //       sw[1] = 1（不拨，默认）→ 【闭环整定】KP、KI 同时右移 sw[3:2] 位：
    //                   00 = ×1 (KP=256 KI=8)   01 = ÷2 (128 / 4)
    //                   10 = ÷4 ( 64 / 2)       11 = ÷8 ( 32 / 1)
    //           5号(sw[4]) 拨到 ON(=0) → 目标速度 ×5（40 → 200 counts/5ms）
    //          按键照常工作（S2 入队 → S1 跑）。LED6 灭。
    //       sw[1] = 0（2 号拨到 ON）→ 【开环点动】按住 S1 出固定占空比，松手就停。
    //          幅值：★把开关往 ON 方向拨，就把它标的权重加上去★
    //                  3 号(sw[2]) = 100‰   4 号(sw[3]) = 200‰   5 号(sw[4]) = 400‰
    //                ⇒ 全部不拨 = 0‰（安全）；三个都拨到 ON = 700‰
    //          6 号(sw[5]) 拨到 ON → 只驱动左通道（按住 S1 看哪个轮子转，
    //               用来确定"通道 ↔ 轮子"的对应，配对排查用）
    //          方向：8 号 拨 ON = 反向（和闭环同一个旋钮，避免两处符号互相抵消）
    //          LED6 常亮表示处在开环点动模式。
    //          用途：
    //            a) 转速读数是真还是噪声 —— 固定占空比下 sl/sr 应该是恒定值；
    //               如果还在乱摆 ⇒ 编码器链路/接线/干扰问题，跟 PID 无关。
    //            b) 正占空比 = 哪个方向、对应哪一路计数 —— 一次就能把
    //               LEFT_NEG / 编码器左右交叉 / 电机端口互换 这三件事对齐。
    //            c) 从 100‰ 往上加，看多大占空比轮子才开始转 —— 直接量出真实死区。
    //   注意：开环点动模式下 S1 不再触发动作队列，必须【按住】S1 才出力；
    //         按住 S0 立刻断电（stby=0）。整定模式下 S1 照常执行队列。
    // =========================================================================
    wire        dbg_en   = ~sw[0];
    wire        dbg_open = dbg_en & (~sw[1]);
    wire        dbg_tune = dbg_en &   sw[1];
    // 幅值：开关拨到 ON(=0) 就把对应权重加进来；全不拨 = 0
    wire [15:0] dbg_mag  = (sw[4] ? 16'd0 : 16'd400) +
                           (sw[3] ? 16'd0 : 16'd200) +
                           (sw[2] ? 16'd0 : 16'd100);
    // 开环方向：统一由 8号(dir_rev，与闭环同一个旋钮)决定，这里只给幅值
    //   （以前这里还有 6号 再乘一个负号，和 8号 会互相抵消，害人，已去掉）
    wire signed [15:0] dbg_duty = $signed(dbg_mag);
    // 6号(sw[5]) 拨到 ON(=0) → 开环点动【只驱动左通道(A 路)】
    //   用途：按住 S1 看【哪个物理轮子】转 ⇒ 直接确定"通道 ↔ 轮子"的对应关系，
    //         再配合"手转轮子看 dl/dr 哪个变"，就能确定编码器配对是否交叉。
    wire        only_l   = ~sw[5];
    wire        dbg_jog  = dbg_open & s1_level & pwm_enable;    // 按住 S1 才出力
    // ★2026-10-08 时序修复★ kp/ki 以前是"拨码 → 变量移位"的组合链，直接接在 DSP
    //   乘法器输入端；PnR 实测它和 16×16 乘法串在同一拍 ⇒ 50MHz setup 违例。
    //   增益只在按按键/拨码时才变，打一拍（20ns）对控制没有任何影响。
    wire [15:0] kp_w = dbg_tune ? (KP >> sw[3:2]) : KP;
    wire [15:0] ki_w = dbg_tune ? (KI >> sw[3:2]) : KI;
    reg  [15:0] kp_eff, ki_eff;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_eff <= KP;
            ki_eff <= KI;
        end else begin
            kp_eff <= kp_w;
            ki_eff <= ki_w;
        end
    end
    // 整定模式第二个旋钮：SW4(sw[4]) 拨到 ON(=0) → 目标速度 ×1.5（700 → 1050 counts/5ms）
    //   用途：现场试"再快一点"够不够、以及快了之后稳不稳，不用重新综合。
    //   （×1.5 之后需要的稳态占空比 ≈ 150 + 1050/2.35 ≈ 600‰，仍在积分限幅 700‰ 之内）
    // ★V4.35j★ 原来这里在整定模式(SW4 拨 ON)会把目标速度再 ×1.5。既然巡航已常驻 ×1.5，
    //   这个调试分支必须去掉，否则同时满足时是 ×2.25（840→1260），现场很容易误拨出事。
    //   整定模式从此只调 PID（kp/ki 仍由 SW3:2 切档），不再改速度。
    wire signed [15:0] tgt_eff  = TARGET_SPD;
    // 方向整定 + 两个"接线事实"开关（所有模式生效）
    //   7号(sw[6]) 拨到 ON(=0) → 编码器左右配对与编译默认【相反】
    //        用途：如果实测"左轮往后、右轮往前、来回抽"（= 交叉耦合：左 PID 实际在管右轮），
    //              拨一下就能把配对换过来，不用重新综合。
    //   8号(sw[7]) 拨到 ON(=0) → 两轮方向一起取反（兜底）
    wire        enc_swap = ENC_SWAP_DEF ^ (~sw[6]);   // SW6 拨ON → 与默认相反
    wire        dir_rev  = ~sw[7];                    // SW7 拨ON → 方向反向
    wire        neg_l    = LEFT_NEG  ^ dir_rev;
    wire        neg_r    = RIGHT_NEG ^ dir_rev;
    // 送给两路 encoder_decode 的 A/B（默认 = 不交叉，见 §9 例化处实测说明）
    wire        enc_a_l  = enc_swap ? enc_ar : enc_al;
    wire        enc_b_l  = enc_swap ? enc_br : enc_bl;
    wire        enc_a_r  = enc_swap ? enc_al : enc_ar;
    wire        enc_b_r  = enc_swap ? enc_bl : enc_br;
    // 开环点动时直接给占空比（经方向整定）；否则走 PID（原逻辑）
    wire signed [15:0] dbg_duty_l = neg_l ? (-dbg_duty) : dbg_duty;
    wire signed [15:0] dbg_duty_r = neg_r ? (-dbg_duty) : dbg_duty;
    wire [15:0] duty_cmd_l = dbg_open ? (dbg_jog ? dbg_duty_l : 16'sd0) : duty_pid_l[15:0];
    wire [15:0] duty_cmd_r = dbg_open ? (dbg_jog ? (only_l ? 16'sd0 : dbg_duty_r) : 16'sd0)
                                      : duty_pid_r[15:0];

    // 只有【开环点动】模式下 S1 才不当"执行"用（那时 S1 = 按住出力）；
    // 闭环整定模式下 S1 照常执行动作队列。
    assign run_pulse = (s1_single | ir_exec) & (~estop_lat) & (~s0_level) & (~ir_stop_hold) & (~dbg_open);

    // =========================================================================
    // 3) 急停锁存：S0 双击 / 遥控 CH- 置位，S0 单击 / 遥控 EQ 清除
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            estop_lat <= 1'b0;
        else if (s0_double | ir_stop)
            estop_lat <= 1'b1;
        else if (s0_single | ir_clear)
            estop_lat <= 1'b0;
    end

    // =========================================================================
    // 4) 动作队列（最多 4）
    // =========================================================================
    wire        push_req;
    wire [2:0]  push_act;
    wire [31:0] push_n;

    // 10cm 遥控行程（574 counts/mm × 100mm）；50/20cm 复用 N_FWD_10 / N_FWD_20
    localparam [31:0] N_FWD_10CM = 32'd57400;

    assign push_req = (s2_single | s2_double | s2_long |
                       s3_single | s3_double | s3_long |
                       s4_single | s4_double | s4_long |
                       ir_fwd | ir_left | ir_right) & (~soft_clr);

    assign push_act = (s2_single | s2_double | s2_long | ir_fwd) ? A_FWD       :
                      (s3_single | s3_long   | ir_left)         ? A_LEFT_PIV  :
                      (s3_double)                               ? A_LEFT_ARC  :
                      (s4_single | s4_long   | ir_right)        ? A_RIGHT_PIV :
                      (s4_double)                               ? A_RIGHT_ARC : A_FWD;

    assign push_n   = (s2_single) ? N_FWD_10  :
                      (s2_double) ? N_FWD_20  :
                      (s2_long)   ? N_FWD_5   :
                      (ir_fwd && (ir_dist == 2'd0)) ? N_FWD_10  :
                      (ir_fwd && (ir_dist == 2'd1)) ? N_FWD_20  :
                      (ir_fwd && (ir_dist == 2'd2)) ? N_FWD_10CM :
                      (s3_single) ? N_TURN_90 :
                      (s3_double) ? N_ARC     :
                      (s3_long)   ? N_TURN_45 :
                      (ir_left)   ? N_TURN_90 :
                      (s4_single) ? N_TURN_90 :
                      (s4_double) ? N_ARC     :
                      (s4_long)   ? N_TURN_45 :
                      (ir_right)  ? N_TURN_90 : 32'd0;

    // 出队 = 开始执行一步 或 本段跑完接下一步
    assign pop_req  = ((st == S_IDLE) && run_pulse && (q_cnt != 3'd0)) ||
                      ((st == S_RUN)  && seg_done  && (q_cnt != 3'd0)) ||
                      ((st == S_EXT_WAIT) && ext_to_q_r && ext_done_r && (q_cnt != 3'd0));

    assign head_act = q_act[q_rd];
    assign head_n   = q_n[q_rd];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_rd  <= 2'd0;
            q_wr  <= 2'd0;
            q_cnt <= 3'd0;
        end
        else if (soft_clr) begin
            q_rd  <= 2'd0;
            q_wr  <= 2'd0;
            q_cnt <= 3'd0;
        end
        else begin
            case ({push_req, pop_req})
                2'b10: begin
                    if (q_cnt < Q_MAX) begin
                        q_act[q_wr] <= push_act;
                        q_n[q_wr]   <= push_n;
                        q_wr        <= q_wr + 2'd1;
                        q_cnt       <= q_cnt + 3'd1;
                    end
                end
                2'b01: begin
                    if (q_cnt != 3'd0) begin
                        q_rd  <= q_rd + 2'd1;
                        q_cnt <= q_cnt - 3'd1;
                    end
                end
                2'b11: begin
                    if (q_cnt != 3'd0) begin
                        q_rd        <= q_rd + 2'd1;
                        q_act[q_wr] <= push_act;
                        q_n[q_wr]   <= push_n;
                        q_wr        <= q_wr + 2'd1;
                    end
                    else begin
                        q_act[q_wr] <= push_act;
                        q_n[q_wr]   <= push_n;
                        q_wr        <= q_wr + 2'd1;
                        q_cnt       <= 3'd1;
                    end
                end
                default: ;
            endcase
        end
    end

    // =========================================================================
    // 5) 执行状态机（装载当前段 + 距离/超时停）
    // =========================================================================
    // 有符号绝对值：计数增/减都正确（无符号 abs 在计数变负时会算成巨大值）
    wire signed [31:0] d_l = $signed(cnt_l) - $signed(cnt_l0);
    wire signed [31:0] d_r = $signed(cnt_r) - $signed(cnt_r0);
    assign dist_l   = d_l[31] ? (32'd0 - d_l) : d_l;
    assign dist_r   = d_r[31] ? (32'd0 - d_r) : d_r;
    // 原地转：IMU 在线用 Yaw；IMU 不在线回退编码器弧长（保证上电可跑）
    reg  [15:0] yaw0;
    reg  [31:0] yaw_age;      // 距上次 yaw_valid 的拍数
    wire        imu_ok = (yaw_age < 32'd50_000_000); // 1s 内有帧
    wire signed [15:0] yaw_delta = $signed(yaw_raw) - $signed(yaw0);
    // 有向进度：只累计【本段转向】方向上的角度；反向惯性/漂移不计
    //   （旧写法 yaw_abs=|Δ| 会把上一段残余转动算成本段进度 → 衔接后只偏左）
    wire signed [15:0] yaw_dir   = (cur_act == A_LEFT_PIV) == YAW_LEFT_POS ?
                                   yaw_delta : -yaw_delta;
    wire [15:0] yaw_abs = yaw_dir[15] ? 16'd0 : yaw_dir;  // 负=还没开始转够
    // ★2026-10-08 时序修复★ IMU yaw → yaw_abs 本身是"减法 + 符号选择 + 钳位"，
    //   后面还接着"到位判据"和"转角速度曲线"两串；修完前两条长链之后，
    //   它成了新的最差路径（19.9ns / 20ns）。这里打一拍切成两段：
    //     cnt: yaw_raw → yaw_abs  |  yaw_abs_r → 到位/速度曲线
    //   yaw 帧率是 10ms 级，晚 20ns 完全无感。
    reg [15:0] yaw_abs_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) yaw_abs_r <= 16'd0;
        else        yaw_abs_r <= yaw_abs;
    end
    // 直线航向误差（有向）：head_e>0 = 相对本段起点偏左（YAW_LEFT_POS=1 时）
    //   corr>0 ⇒ 左轮稍快/右轮稍慢 ⇒ 车右修；IMU 掉线则 0（退回纯编码器速度环）
    wire signed [15:0] head_e  = YAW_LEFT_POS ? yaw_delta : -yaw_delta;
    wire signed [15:0] corr_r  = head_e >>> HEAD_SH;
    wire signed [15:0] head_corr = (~imu_ok) ? 16'sd0 :
                                  (corr_r >  HEAD_LIM) ?  HEAD_LIM :
                                  (corr_r < -HEAD_LIM) ? -HEAD_LIM : corr_r;
    wire        is_yaw_abs = (cur_act == A_YAW_ABS);
    wire        is_pivot   = (cur_act == A_LEFT_PIV) || (cur_act == A_RIGHT_PIV) || is_yaw_abs;
    wire        is_hold    = (cur_act == A_FWD_HOLD);
    // ★V2.7（清单7）★ 原地转段开头的"停稳"窗口；★V2.9★ 改成"极低速起转"软化窗口
    //   （只作用于 A_YAW_ABS；A_LEFT_PIV/A_RIGHT_PIV 从来没有这个窗口，不受影响）
    wire        pivot_settle = is_pivot && ((st == S_RUN) || (st == S_EXT_RUN)) &&
                               (run_cnt < DWELL_CYC);
    // 队列 cur_n 仍是编码器计数；Yaw 目标由段长反推 90°/45°
    wire [15:0] yaw_tgt = (cur_n >= N_TURN_45 + N_TURN_45) ? YAW_90_RAW : YAW_45_RAW;
    // 末段 YAW_STOP_M；非末段轻提前量 YAW_STOP_S（都不爬行）
    wire [15:0] yaw_stop_m = seg_last ? YAW_STOP_M : YAW_STOP_S;
    // =========================================================================
    // ★2026-10-26★ 连续原地转【欠转量结转】—— 修"转一次准、连转就偏"
    //   现象（用户 2026-10-26 报）：只转一次 90° 很准；连转 3~4 次最后整体偏。
    //   机制（可逐行核对）：
    //     ① 到位判据是 `yaw_abs_r + yaw_stop_m >= yaw_tgt`，也就是**离目标
    //        YAW_STOP_M(400=2.2°) 就收工**，剩下那 2.2° 交给停车滑行补完。
    //        单段时滑行正好补上 ⇒ 一次转 90° 很准（与用户观察一致）。
    //     ② 但 seg_done 那一拍，下面执行状态机（S_RUN 分支）**立刻**把
    //        `yaw0 <= yaw_raw` 重锁到【滑行前】的位置，而滑行还会再转 2.2°。
    //      ⇒ 滑行转的那 2.2° 对下一段等于白给：下一段的 0 点比真实静止位置早 2.2°。
    //      ⇒ 每多连一段就少 2.2°，4 段连转累计 ≈ 8.8°。
    //   对策：把上一段"没转完的余量"记下来，加到下一段的目标上。
    //     · 只在【同方向】连转时加 —— 反向转时两段的偏差自然抵消，加了反而错。
    //     · 只在【紧接的下一段】加 —— 中间隔了停顿的话，车已经停稳、下一段的 yaw0
    //       锁在真实静止位置，此时再补就是【多转】，反而错。用 150ms 时间窗判"紧接"。
    //     · 上限 YAW_RESID_MAX：超时/打滑导致的异常大残差不许结转（防目标翻倍）。
    //   注：只作用于 A_LEFT_PIV / A_RIGHT_PIV（相对转向）；A_YAW_ABS 用绝对目标，
    //       它的目标由 FSM 给，不吃这一套。
    //   ★2026-10-26 实测（sim/tb_turn_multi.v）★：无结转时 4 段连转误差
    //     -329 → -688 → -1051 → -1414（每段 −360，线性增长）；
    //     结转生效后恒在 −300 量级，不再随段数增长。
    // =========================================================================
    localparam [23:0] RESID_WIN_CYC = 24'd7_500_000;  // "紧接下一段"的时间窗 150ms @50MHz
    localparam [15:0] YAW_RESID_MAX = 16'd1820;       // 结转上限 = 10°
    // 本段转向是否让 yaw 增大（与下面 yaw_dir 的约定完全一致）
    wire        seg_pos = ((cur_act == A_LEFT_PIV) == YAW_LEFT_POS);
    reg  [15:0] yaw_resid;       // 上一段的欠转量（候选）
    reg         yaw_resid_pos;   // 该余量对应的方向（1 = yaw 增大）
    reg  [23:0] resid_age;       // 距上次 seg_done 的拍数（判"紧接"用）
    reg  [15:0] yaw_carry_r;     // 本段【实际生效】的结转量（新段第一拍锁定，段内不变）
    reg         seg_rebase_d;
    // 实际用的目标（只有【相对原地转】才带结转；A_YAW_ABS 与直行段原样）
    wire [15:0] yaw_tgt_eff = (is_pivot && (~is_yaw_abs)) ? (yaw_tgt + yaw_carry_r) : yaw_tgt;
    // ★时序★ 这个加法器正好插在 yaw_remain → 乘法器 这条链的前面，
    //   而 V5.0c 的 Setup 余量只有 +0.505ns，不能白加一级。它只在段边界变，
    //   打一拍（20ns）对"按剩余角度收油"没有任何可测影响。
    //   新段第一拍 yaw_tgt_eff_r 还是上一段的值 —— 那一拍被 seg_rebase 挡住不判完成，
    //   而且起转期走 pivot_settle 的 SOFT_TURN_SPD，turn_cmd 不被采信，安全。
    reg  [15:0] yaw_tgt_eff_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) yaw_tgt_eff_r <= YAW_45_RAW;
        else        yaw_tgt_eff_r <= yaw_tgt_eff;
    end
    // 绝对航向：最短角误差 + 提前量
    wire signed [15:0] ya_err = $signed(yaw_ext_tgt_r) - $signed(yaw_raw);
    wire [15:0] ya_mag = ya_err[15] ? (16'd0 - ya_err) : ya_err;
    // ★2b-1 修一处隐患★：旧写法把 |误差|>90° 的角"镜像"了一下（32768−|e|），那是错的：
    //   16 位补码减法给出的误差本身就已经在 ±180° 内，最短角 = 直接用它。旧写法在
    //   误差 −110° 时会算成 +70°，车会朝反方向转 180°（本工程转向都 ≤60°，平时不发作，
    //   但 2b 的"回线/保持航向"要用到它，所以这里一并改对）。
    wire [15:0] ya_min = ya_mag;              // 最短角误差幅值（≤180°）
    wire        ya_pos = ~ya_err[15];         // 1 = 需要 yaw 增大
    // ★2026-10-26★ 方向滞回：误差小的时候【保持上次转向方向】。
    //   没有这一条时，车在目标附近被 IMU 抖动带着来回翻向（见 YA_HYST_CNT 注释），
    //   表现为 A_YAW_ABS 永远判不到"到位"，原地左右摆好几秒。
    //   只作用于 A_YAW_ABS；A_LEFT_PIV/A_RIGHT_PIV 是固定方向，不受影响。
    reg ya_pos_lat;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                    ya_pos_lat <= 1'b0;
        else if (!is_yaw_abs)          ya_pos_lat <= ya_pos;            // 非绝对航向转：跟随
        else if (ya_mag > YA_HYST_CNT) ya_pos_lat <= ya_pos;            // 误差大：允许换向
        // 误差 ≤ YA_HYST_CNT：保持上次方向（防抖）
    end
    // ---- ★2b-1：A_FWD_HOLD 的有向航向保持修正★ ----
    //   head_corr 的符号约定与 A_FWD 的 head_corr 完全一致：正 = 左轮快 ⇒ 车往右修。
    //   ya_pos=1（需要 yaw 增大）+ YAW_LEFT_POS=1（左转增 yaw）⇒ 需要往左 ⇒ 修正取负。
    wire        hold_need_left = (ya_pos == YAW_LEFT_POS);
    wire [15:0] hold_mag   = (ya_min > HOLD_GATE) ? (ya_min - HOLD_GATE) : 16'd0;
    wire [15:0] hold_sh16  = hold_mag >> HOLD_SH;
    wire signed [15:0] hold_raw  = hold_need_left ? (-$signed(hold_sh16)) : $signed(hold_sh16);
    wire signed [15:0] hold_corr = (~imu_ok) ? 16'sd0 :
                                   (hold_raw >  HOLD_LIM) ?  HOLD_LIM :
                                   (hold_raw < -HOLD_LIM) ? -HOLD_LIM : hold_raw;
    wire        yaw_done_abs = is_yaw_abs && imu_ok && (ya_min <= YAW_STOP_M);
    // =========================================================================
    // 【2026-09-29 修 bug】段完成判据（原地转角度不准 + 卡死两个问题一起修）
    //   旧写法1：(dist_l >= cur_n) || (dist_r >= cur_n)  ← 取"先到的那个轮子"，
    //            两轮打滑不对称时车体没转够就结束 ⇒ 实测偏 20~30°。
    //   旧写法2（我上一版）：原地转且有 IMU 时【只用 yaw】⇒ 一旦 yaw 掉帧/冻住，
    //            原地转永远不完成，只能等 60 秒超时 ⇒ 表现为"按键没反应、D3 一直亮"。
    //   现在三层，谁都兜得住：
    //     ① yaw（IMU 在线时最准；前段全速，最后 ~30° 压到 70%，提前量硬停）
    //     ② 两轮【平均】弧长 ≥ cur_n×1.1（仅卡死/掉线保险；陀螺仪可靠时不会先触发）
    //     ③ 超时：原地转 PIVOT_MS=5s，其它动作 SAFE_MS=10s
    // =========================================================================
    // 原地转速度（IMU 在线）：
    //   remain ≥ yaw_tgt/3（90° 约剩 30°）→ 保持 TURN_SPD=500
    //   之后线性压到 TURN_SPD_MIN=350（70%），到位即停，不爬行
    //   公式：turn = MIN + 150×remain×3/yaw_tgt = MIN + remain×450/yaw_tgt
    //   yaw_tgt=2^14/2^13，用移位代替除法。IMU 不在线：固定 MIN，靠编码器/超时停。
    wire [15:0] yaw_remain = (yaw_abs_r >= yaw_tgt_eff_r) ? 16'd0 : (yaw_tgt_eff_r - yaw_abs_r);
    // 末段从 MIN(315) 加到 450，增量 135（= 原 675/225 的 60%，收油形状不变）
    wire [31:0] turn_mul   = {16'd0, yaw_remain} * 32'd405;
    wire [15:0] turn_add   = (yaw_tgt_eff_r[14]) ? turn_mul[29:14] : turn_mul[28:13];
    // ★时序：×675 + 变位右移再打一拍，turn_clp 之后的钳位/求和只剩 3~4 级逻辑★
    reg  [15:0] turn_add_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) turn_add_r <= 16'd0;
        else        turn_add_r <= turn_add;
    end
    // ★V2.15★：增量上限 135 → 180（快段才够到 340+180=520）
    wire [15:0] turn_clp   = (turn_add_r > 16'd180) ? 16'd180 : turn_add_r;
    // 非末段不掉到爬行速，保持衔接；末段压到 TURN_SPD_MIN 收角
    wire [15:0] turn_min_e = seg_last ? TURN_SPD_MIN : 16'd360;
    // 钳位到 [TURN_SPD_MIN, TURN_SPD]
    wire [15:0] turn_sum   = turn_min_e + turn_clp;
    wire [15:0] turn_lvl   = (turn_sum > TURN_SPD) ? TURN_SPD[15:0] : turn_sum;
    // ★V2.15★：**两段式**——剩余角度还大于 CRAWL_ANG_CNT(20°) 时走上面那条快曲线
    //   （340~520）；最后 20° 直接降到 CRAWL_TURN_SPD(130≈45mm/s) 爬行收角，
    //   冲量小 ⇒ 一次收住，不再"冲过头→补转→再冲"。
    // ★V2.28a★：A_YAW_ABS 的“剩余角度”必须用 ya_min（到目标的误差），
    //   不能用 yaw_abs（A_YAW_ABS 的 yaw_dir 取反后左转恒被截成 0 ⇒ 永远满速）。
    wire [15:0] rem_eff = is_yaw_abs ? ya_min : yaw_remain;
    wire [15:0] turn_lvl_2s = (rem_eff > CRAWL_ANG_CNT) ? turn_lvl : CRAWL_TURN_SPD;
    wire signed [15:0] turn_base = imu_ok ? $signed(turn_lvl_2s) : TURN_SPD_MIN;
    // ★绕障时的原地转压到 75%：90° 约 1.7s（用户要"别那么甩"，但也不要肉）★
    wire signed [15:0] turn_base_av = (turn_base * AVOID_TURN_NUM) >>> AVOID_TURN_SH;
    // ★2026-10-08 时序修复★
    //   原写法把 turn_cmd 做成组合量，于是
    //      yaw_remain ×675 → 移位 → 钳位 → 求和 → 钳位 →(×1.5)→ v_l/v_r → PID 乘法
    //   全挤在同一拍里：PnR 实测这条链路的端点在 60K 器件上 setup 违例。
    //   turn 命令本身变化很慢（IMU 帧率级，最快 10ms 一次），打一拍只晚 20ns，
    //   对"按剩余角度提前收油"这种功能没有任何可测影响。
    reg signed [15:0] turn_cmd;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            turn_cmd <= 16'sd0;
        else if (ext_slow)
            turn_cmd <= (dbg_tune & (~sw[4])) ? ((turn_base_av * 16'sd3) >> 1) : turn_base_av;
        else
            turn_cmd <= (dbg_tune & (~sw[4])) ? ((turn_base * 16'sd3) >> 1) : turn_base;
    end
    wire [31:0] rot_avg = (dist_l + dist_r) >> 1;
    // ★2026-10-08 时序修复★ 旧写法 rot_lim = cur_n + (cur_n/32'd10) 是 32 位【变量除法】，
    //   PnR 实测从 cur_n 到 cnt_l0/yaw0/run_cnt 的 CE 共 34.76ns（周期只有 20ns），
    //   而且是 312 个违例端点的最大来源（cur_n 一位扇出到一大片 CE）。
    //   两边同乘 10 把除法消掉：
    //       rot_avg ≥ cur_n + cur_n/10  ⇔  rot_avg×10 ≥ cur_n×11
    //   与原式最多差 1 个编码器计数（这是"两轮弧长兜底"，1 计数无意义）。
    //   再打一拍：cnt → |Δ| → rot_avg 的组合本身就接近 20ns。

    // =====================================================================
    // ★★ 段完成判据必须"新段第一拍作废"（2026-10-08 踩的坑，下面这段是修它）★★
    //   段装载时状态机把 cnt_l0 / yaw0 / cur_n 一次性换成新段基准，
    //   但下面这些流水线寄存器里还留着【上一段】的弧长/角度 ⇒ 新段第一拍就满足
    //       rot_avg ≥ cur_n  （甚至 0 ≥ 0）
    //   ⇒ seg_done 立刻为 1 ⇒ 状态机认为"这一段已经跑完"，把队列跳到下一步。
    //   现象就是：按 S3/S4 原地转，车几乎不动（或只走一点/转一点），
    //             队列里的动作被一个个吃掉。
    //   修法：用"基准刚被改写"检出【新段第一拍】seg_rebase，
    //         这一拍强行禁止 seg_done，并清掉完成流水线。
    // =====================================================================
    reg [31:0] cnt_l0_d;
    reg [15:0] yaw0_d;
    reg [31:0] cur_n_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_l0_d <= 32'd0;
            yaw0_d   <= 16'd0;
            cur_n_d  <= 32'd0;
        end else begin
            cnt_l0_d <= cnt_l0;
            yaw0_d   <= yaw0;
            cur_n_d  <= cur_n;
        end
    end
    // 本拍就是"新段第一拍"（段基准刚被状态机换掉）
    //   三个一起看：cnt_l0 / yaw0 换了，或者段长 cur_n 换了（上电时 cnt_l0 恰好是 0，
    //   只盯 cnt_l0 会漏掉"第一次启动"那一拍 —— 见下面 rot_done_c 的 cur_n!=0 门控）
    wire seg_rebase = (cnt_l0 != cnt_l0_d) || (yaw0 != yaw0_d) || (cur_n != cur_n_d);

    // =========================================================================
    // ★V5.0e：S3/S4 串级角度闭环★
    //   外层：目标 90° - 融合航向 -> 角度 PID -> 有符号转向量
    //   内层：现有左右轮速度 PID（不变）
    //
    //   10Hz IMU 不能直接当 5ms 控制反馈：
    //     · 0x53 每帧约 100ms；56°/s 时两帧相差约 5.6°
    //     · 两帧之间用编码器左右轮差速估算 yaw 增量，保持 5ms 连续反馈
    //     · IMU 帧到达时用绝对角重锁融合航向，修掉原地转打滑误差
    //
    //   完成条件不是“提前 2.2°”：
    //     误差 <= ANG_TOL_CNT(约1°)
    //     且编码器角速度 <= ANG_STILL_RATE
    //     且至少收到一帧落在这条误差带内的新 IMU 数据
    //     且连续 ANG_STILL_MS 保持
    // =========================================================================
    localparam [15:0] ANG_STILL_SAMPLES =
        (ANG_STILL_MS + SPEED_MS - 1) / SPEED_MS;

    // imu_rel16 的 16 位减法会自然按 ±180° 回绕，正是最短相对角口径。
    wire signed [15:0] imu_rel16 = $signed(yaw_raw) - $signed(yaw0);
    // 编码器差速 -> yaw counts/5ms。
    // 8192 / 101600 ≈ 165 / 2048 = 0.080566，误差约 0.08%。
    wire signed [31:0] enc_diff32 =
        $signed(spd_r_eff) - $signed(spd_l_eff);
    wire signed [31:0] enc_yaw_rate =
        (enc_diff32 * 32'sd165) >>> 11;
    // 当前段希望转到的有符号相对角：左转为正、右转为负。
    wire signed [17:0] pivot_tgt =
        seg_pos ? $signed({2'b00, yaw_tgt})
                : -$signed({2'b00, yaw_tgt});

    reg signed [23:0] yaw_fused;
    reg signed [31:0] angle_i_acc;
    reg signed [31:0] angle_u_raw;
    reg signed [15:0] turn_pid_cmd;
    reg        pid_calc_d;
    reg [15:0] angle_still_cnt;
    reg        settle_imu_seen;

    wire signed [23:0] yaw_fused_ext = yaw_fused;
    wire signed [23:0] pivot_tgt_ext =
        {{6{pivot_tgt[17]}}, pivot_tgt};
    wire signed [23:0] angle_err_now =
        pivot_tgt_ext - yaw_fused_ext;
    wire [23:0] angle_err_abs =
        angle_err_now[23] ? -angle_err_now : angle_err_now;
    wire [15:0] omega_abs_now =
        enc_yaw_rate[31] ? -enc_yaw_rate[15:0] : enc_yaw_rate[15:0];
    wire signed [31:0] angle_i_next_raw =
        angle_i_acc + angle_err_now;
    wire signed [31:0] angle_i_next =
        (angle_i_next_raw >  ANG_I_LIM) ? ANG_I_LIM :
        (angle_i_next_raw < -ANG_I_LIM) ? -ANG_I_LIM :
        angle_i_next_raw;
    wire signed [31:0] angle_i_use =
        (angle_err_abs > {8'd0, ANG_TOL_CNT}) ? angle_i_next : 32'sd0;
    wire signed [31:0] angle_u_lim =
        (angle_u_raw > TURN_SPD) ? TURN_SPD :
        (angle_u_raw < -TURN_SPD) ? -TURN_SPD :
        angle_u_raw;
    wire signed [23:0] imu_err_now =
        pivot_tgt_ext - {{8{imu_rel16[15]}}, imu_rel16};
    wire [23:0] imu_err_abs =
        imu_err_now[23] ? -imu_err_now : imu_err_now;
    wire rel_pivot = is_pivot && (~is_yaw_abs);
    wire rel_pivot_start = seg_rebase && rel_pivot;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_fused       <= 24'sd0;
            settle_imu_seen <= 1'b0;
        end else if (!rel_pivot || rel_pivot_start) begin
            yaw_fused       <= 24'sd0;
            settle_imu_seen <= 1'b0;
        end else begin
            if (yaw_valid) begin
                // 10Hz 绝对角重锁融合航向；两帧之间由编码器负责连续性。
                yaw_fused <= {{8{imu_rel16[15]}}, imu_rel16};
                settle_imu_seen <= (imu_err_abs <= ANG_TOL_CNT);
            end else if (strobe_l) begin
                yaw_fused <= yaw_fused + $signed(enc_yaw_rate[23:0]);
            end
        end
    end

    // 角度 PID 分两级寄存：第一级算三项，第二级限幅+斜率限制。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_i_acc  <= 32'sd0;
            angle_u_raw  <= 32'sd0;
            pid_calc_d   <= 1'b0;
        end else if (!rel_pivot || rel_pivot_start) begin
            angle_i_acc  <= 32'sd0;
            angle_u_raw  <= 32'sd0;
            pid_calc_d   <= 1'b0;
        end else begin
            pid_calc_d <= 1'b0;
            if (strobe_l) begin
                // 离目标较远时正常积分；带内清掉积分，避免到点后积分顶着车继续走。
                if (angle_err_abs > {8'd0, ANG_TOL_CNT})
                    angle_i_acc <= angle_i_next;
                else
                    angle_i_acc <= 32'sd0;
                angle_u_raw <=
                    ((angle_err_now * ANG_KP) >>> 8) +
                    ((angle_i_use   * ANG_KI) >>> 8) -
                    ((enc_yaw_rate * ANG_KD) >>> 8);
                pid_calc_d <= 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            turn_pid_cmd <= 16'sd0;
        else if (!rel_pivot || rel_pivot_start)
            turn_pid_cmd <= 16'sd0;
        else if (pid_calc_d) begin
            if (angle_u_lim > (turn_pid_cmd + ANG_SLEW))
                turn_pid_cmd <= turn_pid_cmd + ANG_SLEW;
            else if (angle_u_lim < (turn_pid_cmd - ANG_SLEW))
                turn_pid_cmd <= turn_pid_cmd - ANG_SLEW;
            else
                turn_pid_cmd <= angle_u_lim[15:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            angle_still_cnt <= 16'd0;
        else if (!rel_pivot || rel_pivot_start)
            angle_still_cnt <= 16'd0;
        else if (strobe_l) begin
            if ((angle_err_abs <= ANG_TOL_CNT) &&
                (omega_abs_now <= ANG_STILL_RATE) &&
                settle_imu_seen) begin
                if (angle_still_cnt < ANG_STILL_SAMPLES)
                    angle_still_cnt <= angle_still_cnt + 16'd1;
            end else
                angle_still_cnt <= 16'd0;
        end
    end

    wire yaw_done_rel = rel_pivot && imu_ok &&
                        settle_imu_seen &&
                        (angle_err_abs <= ANG_TOL_CNT) &&
                        (omega_abs_now <= ANG_STILL_RATE) &&
                        (angle_still_cnt >= ANG_STILL_SAMPLES);
    wire yaw_done = yaw_done_rel || yaw_done_abs;

    reg [31:0] rot_avg_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          rot_avg_r <= 32'd0;
        else if (seg_rebase) rot_avg_r <= 32'd0;      // 新段从 0 弧长开始
        else                 rot_avg_r <= rot_avg;
    end
    wire [35:0] rot_avg_x10 = {4'd0, rot_avg_r} * 6'd10;
    wire [35:0] cur_n_x11   = {4'd0, cur_n}    * 6'd11;
    // ★"只在段运行中才判弧长"门控★
    //   空闲时（上电 / 段跑完队列空）cur_n=0 ⇒ rot_avg_r(0) >= cur_n(0) 恒为真，
    //   会把"段完成"锁成 1；等 S1 一按，新段第一拍就被判"已跑完" ⇒ 按键功能全废
    //   （2026-10-08 实测：按 S3/S4 原地转几乎不动）。这里要求 st 处在运行态才比较。
    wire        seg_active  = (st == S_RUN) || (st == S_EXT_RUN);
    // ★2b-1：IMU_ARC_GATE —— IMU 在线时禁止"两轮平均弧长"给【原地转】判完成★
    //   （车在跑的时候下令转，轮子被拖着多滚，弧长会提前满足 ⇒ 段被截断只转 34°）
    //   直行段（非 pivot）仍以弧长为主判据（弧长就是"走了多远"），不动。
    wire        rot_done_c  = seg_active &&
                              (is_pivot ? ((~IMU_ARC_GATE | ~imu_ok) && (rot_avg_x10 >= cur_n_x11))
                                        : (rot_avg_r >= cur_n));
    // 段完成判据也打一拍：seg_done 扇出到 q_act RAM / run_cnt / cnt_l0 / yaw0 一大批 CE，
    //   组合直连时 PnR 只给到 +0.015ns 裕量（等于没裕量）；同样在新段第一拍清掉。
    reg         yaw_done_r, rot_done_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_done_r <= 1'b0;
            rot_done_r <= 1'b0;
        end else if (seg_rebase) begin
            yaw_done_r <= 1'b0;
            rot_done_r <= 1'b0;
        end else begin
            yaw_done_r <= yaw_done;
            rot_done_r <= rot_done_c;
        end
    end
    wire [31:0] to_cyc  = is_pivot ? PIVOT_CYC : RUN_CYC;
    // 避障暂停时必须压住 seg_done：否则编码器抖动/IMU 漂移会误判段完成、
    // 弹出下一段并重置 cnt_*/yaw0，把「剩余里程」丢掉（review B1）
    // （~seg_rebase：新段第一拍不许判完成 —— 见上面的说明）
    assign seg_done = ((st == S_RUN) || (st == S_EXT_RUN)) && (~force_stop) &&
                      (~seg_rebase) &&
                      (yaw_done_r || rot_done_r ||
                       ((run_cnt + 32'd1) >= to_cyc));

    // ★2026-10-26★ 欠转量结转的记账（机制说明见 yaw_tgt_eff 上方那段注释）
    //   ① resid_age：距上次 seg_done 的拍数，用来判"下一段是不是紧接的连段"。
    //   ② yaw_resid：本段结束时【还欠多少度】。只在"相对原地转 + 靠 yaw 判到位
    //      （不是超时/弧长兜底）"时才记；超时/打滑的异常大残差不记（上限兜住）。
    //   ③ yaw_carry_r：新段第一拍（seg_rebase 后一拍，此时 cur_act 已稳定）
    //      决定本段要不要带结转 —— 同方向 + 在时间窗内 才带，段内保持不变。
    //      （段内不能跟着 resid_age 变：一段转 0.3~0.9s，早超过 150ms 窗了。）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) seg_rebase_d <= 1'b0;
        else        seg_rebase_d <= seg_rebase;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yaw_resid     <= 16'd0;
            yaw_resid_pos <= 1'b0;
            resid_age     <= RESID_WIN_CYC;       // 上电第一段不算连段
        end else begin
            if (resid_age != RESID_WIN_CYC) resid_age <= resid_age + 24'd1;
            if (seg_done) begin
                resid_age <= 24'd0;
                if (is_pivot && (~is_yaw_abs) && yaw_done_r &&
                    (yaw_tgt_eff > yaw_abs_r) &&
                    ((yaw_tgt_eff - yaw_abs_r) <= YAW_RESID_MAX)) begin
                    // ★必须记 yaw_tgt_eff（带结转的目标）而不是 yaw_tgt★：
                    //   本段的目标已经被上一段的余量抬高过，用裸 yaw_tgt 减会把这部分
                    //   抵掉 ⇒ 余量每段衰减一次，连转到第三段就回到老样子。
                    yaw_resid     <= yaw_tgt_eff - yaw_abs_r;   // 正常 ≈ yaw_stop_m(400)
                    yaw_resid_pos <= seg_pos;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) yaw_carry_r <= 16'd0;
        else if (seg_rebase_d)
            yaw_carry_r <= ((resid_age < RESID_WIN_CYC) && (yaw_resid_pos == seg_pos) &&
                            is_pivot && (~is_yaw_abs)) ? yaw_resid : 16'd0;
    end

    // IMU 存活：1 秒内有帧才用 Yaw 转向
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            yaw_age <= 32'hFFFF_FFFF;
        else if (yaw_valid)
            yaw_age <= 32'd0;
        else if (yaw_age != 32'hFFFF_FFFF)
            yaw_age <= yaw_age + 32'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            cur_act <= A_FWD;
            cur_n   <= 32'd0;
            run_cnt <= 32'd0;
            cnt_l0  <= 32'd0;
            cnt_r0  <= 32'd0;
            yaw0    <= 16'd0;
            seg_last <= 1'b1;
            remain_saved <= 32'd0;
            remain_val_r <= 1'b0;
            ext_done_r   <= 1'b0;
            remain_cap_r <= 1'b0;
            ext_to_q_r   <= 1'b0;
            yaw_ext_tgt_r<= 16'd0;
        end
        else if (soft_clr || (!pwm_enable)) begin
            st      <= S_IDLE;
            run_cnt <= 32'd0;
            seg_last <= 1'b1;
            ext_done_r <= 1'b0;
            remain_val_r <= 1'b0;
            remain_cap_r <= 1'b0;
            ext_to_q_r   <= 1'b0;
        end
        else begin
            ext_done_r   <= 1'b0;
            remain_cap_r <= 1'b0;

            if (ext_load) begin
                if ((st == S_RUN) && (cur_act == A_FWD)) begin
                    remain_saved <= remain_now;
                    remain_val_r <= 1'b1;
                    remain_cap_r <= 1'b1;
                end
                st       <= S_EXT_RUN;
                cur_act  <= ext_act;
                cur_n    <= ext_n;
                yaw_ext_tgt_r <= ext_yaw_tgt;
                cnt_l0   <= cnt_l;
                cnt_r0   <= cnt_r;
                yaw0     <= yaw_raw;
                run_cnt  <= 32'd0;
                seg_last <= 1'b1;
                ext_to_q_r <= ext_to_queue;
            end
            else if (st == S_IDLE) begin
                run_cnt <= 32'd0;
                if (run_pulse && (q_cnt != 3'd0)) begin
                    st      <= S_RUN;
                    cur_act <= head_act;
                    cur_n   <= head_n;
                    cnt_l0  <= cnt_l;
                    cnt_r0  <= cnt_r;
                    yaw0    <= yaw_raw;
                    run_cnt <= 32'd0;
                    remain_val_r <= 1'b0;
                    seg_last <= (~push_req) && (q_cnt == 3'd1);
                end
            end
            else if (st == S_RUN) begin
                if (seg_done) begin
                    run_cnt <= 32'd0;
                    remain_val_r <= 1'b0;
                    if (q_cnt != 3'd0) begin
                        st      <= S_RUN;
                        cur_act <= head_act;
                        cur_n   <= head_n;
                        cnt_l0  <= cnt_l;
                        cnt_r0  <= cnt_r;
                        yaw0    <= yaw_raw;
                        seg_last <= (~push_req) && (q_cnt == 3'd1);
                    end
                    else begin
                        st <= S_IDLE;
                        seg_last <= 1'b1;
                    end
                end
                else if (!force_stop)
                    run_cnt <= run_cnt + 32'd1;
            end
            else if (st == S_EXT_RUN) begin
                if (seg_done) begin
                    run_cnt    <= 32'd0;
                    ext_done_r <= 1'b1;
                    st         <= S_EXT_WAIT;
                end
                else if (!force_stop)
                    run_cnt <= run_cnt + 32'd1;
            end
            else begin // S_EXT_WAIT
                run_cnt <= 32'd0;
                if (ext_to_q_r && ext_done_r) begin
                    if (q_cnt != 3'd0) begin
                        st      <= S_RUN;
                        cur_act <= head_act;
                        cur_n   <= head_n;
                        cnt_l0  <= cnt_l;
                        cnt_r0  <= cnt_r;
                        yaw0    <= yaw_raw;
                        remain_val_r <= 1'b0;
                        ext_to_q_r <= 1'b0;
                        seg_last <= (~push_req) && (q_cnt == 3'd1);
                    end
                    else begin
                        st <= S_IDLE;
                        ext_to_q_r <= 1'b0;
                        seg_last <= 1'b1;
                    end
                end
                else if (ext_done_r) begin
                    // ★修复：ext 不入队时也必须退出 S_EXT_WAIT★
                    // 否则队列灯一直闪、S1 无法启动（st 卡在 EXT_WAIT）
                    if (remain_val_r) begin
                        // 被绕障打断的直行：用剩余里程继续
                        st      <= S_RUN;
                        cur_act <= A_FWD;
                        cur_n   <= remain_saved;
                        cnt_l0  <= cnt_l;
                        cnt_r0  <= cnt_r;
                        yaw0    <= yaw_raw;
                        run_cnt <= 32'd0;
                        remain_val_r <= 1'b0;
                        ext_to_q_r <= 1'b0;
                    end
                    else begin
                        if (q_cnt != 3'd0) begin
                            st      <= S_RUN;
                            cur_act <= head_act;
                            cur_n   <= head_n;
                            cnt_l0  <= cnt_l;
                            cnt_r0  <= cnt_r;
                            yaw0    <= yaw_raw;
                            run_cnt <= 32'd0;
                            ext_to_q_r <= 1'b0;
                            seg_last <= (~push_req) && (q_cnt == 3'd1);
                        end
                        else begin
                            st <= S_IDLE;
                            ext_to_q_r <= 1'b0;
                            seg_last <= 1'b1;
                        end
                    end
                end
            end // S_EXT_WAIT
        end
    end

    // =========================================================================
    // 6) 差速目标（车体坐标系）→ NEG → PID
    // =========================================================================
    // 避障急停时冻结 PID，防止积分饱和后解除时猛冲
    // 手柄点动：仅队列未在跑段时有效（S_RUN 时忽略手柄，避免抢自动任务）
    // ★V2.28★：绕障 ext 段进行中禁止手柄/红外抢动作。
    //   gp_mode 在顶层还会屏蔽 obstacle_stop，若这里不拦住，手柄一直接收
    //   就会把 A_YAW_ABS/A_FWD_HOLD 整个盖掉（现场两轮同号/反号的怪现象）。
    wire gp_req = (gp_fwd | gp_back | gp_left | gp_right) &
                  (~estop_lat) & (~s0_level) & (~ir_stop_hold) &
                  (~force_stop) & (~ext_active);
    // ★final_servo：手柄随时可接管（含导航段进行中）★
    //   原版 (st != S_RUN) & (st != S_EXT_RUN) 会让“导航中遇障停车后想用手柄倒车”失效，
    //   而且松手瞬间导航段会自己接着跑。这里去掉状态限制：只要手柄有键，就由手柄驱动；
    //   顶层在检测到接管后锁存 mission_hold，松手后车保持停住，直到 S0/S1。
    wire gp_active = gp_req & (~dbg_open);
    // 手柄速度：与 TARGET_SPD 同量级；左转=左负右正（与 A_LEFT_PIV 一致）
    wire signed [15:0] gp_spd = tgt_eff;
    wire signed [15:0] gp_v_l = gp_fwd ?  gp_spd :
                                gp_back ? -gp_spd :
                                gp_left ? -gp_spd :
                                gp_right ? gp_spd : 16'sd0;
    wire signed [15:0] gp_v_r = gp_fwd ?  gp_spd :
                                gp_back ? -gp_spd :
                                gp_left ?  gp_spd :
                                gp_right ? -gp_spd : 16'sd0;

    // ★慢速档（v3.0）★
    //   绕障腿（ext_active & ext_slow）= AVOID_SPD；
    //   前方进减速区/正前样本失效（slow_req，来自顶层）= SLOW_SPD；
    //   其余巡航 = tgt_eff。★不再用 1kHz 斩波 gate 来"减速"★（斩波会让 PID 积分顶满、
    //   解除瞬间窜车，而且实际转速非线性）
    wire run_slow = ext_active ? ext_slow : slow_req;
    wire signed [15:0] sel_spd = run_slow ? (ext_active ? AVOID_SPD : SLOW_SPD) : tgt_eff;

    assign pid_run = ((st == S_RUN) || (st == S_EXT_RUN) || gp_active) && pwm_enable && (~force_stop);

    always @(*) begin
        if (gp_active) begin
            v_l = gp_v_l;
            v_r = gp_v_r;
        end else case (cur_act)
            A_FWD: begin
                // IMU 航向辅助：相对本段起点的偏差做微差速，内环仍是编码器速度 PID
                v_l = sel_spd + head_corr;
                v_r = sel_spd - head_corr;
            end
            A_FWD_HOLD: begin
                // ★2b-1：前进 + 保持【指定的绝对航向】（ext_yaw_tgt）：只做小幅差速微调★
                //   2b 的"小步斜行 / 贴着障碍走 / 回线"都用它；
                //   力矩不足以把车扭住时也只是"跟不上"，不会变成原地打转。
                v_l = sel_spd + hold_corr;
                v_r = sel_spd - hold_corr;
            end
            // ★V5.0e★ S3/S4 使用有符号角度 PID 输出。
            //   过冲后 PID 自然变号，低速反向修正；IMU 掉线时退回原固定方向。
            A_LEFT_PIV, A_RIGHT_PIV: begin
                if (imu_ok) begin
                    v_l = -turn_pid_cmd;
                    v_r =  turn_pid_cmd;
                end else if (cur_act == A_LEFT_PIV) begin
                    v_l = -turn_cmd;
                    v_r =  turn_cmd;
                end else begin
                    v_l =  turn_cmd;
                    v_r = -turn_cmd;
                end
            end
            A_YAW_ABS: begin
                // ★V2.7（清单7）★ 先停稳 DWELL_MS 再转；★V2.9★ 改成【极低速起转】
                //   —— 用户要求"不要停"：低速起转既没有硬停的顿感，
                //   又保留"低速下转向不被地面拖"的精度（转完仍由 IMU 到位判据兜底）。
                if (pivot_settle) begin
                    // ★2026-10-26★ 方向一律取【滞回后】的 ya_pos_lat（见 YA_HYST_CNT 注释）：
                    //   目标附近 IMU 抖动会让 ya_pos 来回翻，v_l/v_r 正负互换 ⇒ 原地打摆子。
                    if (ya_pos_lat == YAW_LEFT_POS) begin
                        v_l = -SOFT_TURN_SPD;      // 很低速地朝"左转"方向起步
                        v_r =  SOFT_TURN_SPD;
                    end else begin
                        v_l =  SOFT_TURN_SPD;
                        v_r = -SOFT_TURN_SPD;
                    end
                end
                // ya_pos_lat=1 需 yaw 增大；YAW_LEFT_POS=1 时左转增 yaw
                else if (ya_pos_lat == YAW_LEFT_POS) begin
                    v_l = -turn_cmd;
                    v_r =  turn_cmd;
                end
                else begin
                    v_l =  turn_cmd;
                    v_r = -turn_cmd;
                end
            end
            A_LEFT_ARC: begin
                v_l = sel_spd / 16'sd2;
                v_r = sel_spd;
            end
            A_RIGHT_ARC: begin
                v_l = sel_spd;
                v_r = sel_spd / 16'sd2;
            end
            default: begin
                v_l = 16'sd0;
                v_r = 16'sd0;
            end
        endcase
    end

    assign target_l = neg_l ? (-v_l) : v_l;
    assign target_r = neg_r ? (-v_r) : v_r;

    // 段装载：只在【动作类型变了】时清 PID 积分。
    //   直线→直线不清：稳态速度靠积分顶着，一清就会「停一下再走」。
    //   转↔转（含反向）、直线↔转弯才清：防上一段积分拖尾把转速打乱。
    wire seg_load = ((st == S_IDLE) && run_pulse && (q_cnt != 3'd0)) ||
                    ((st == S_RUN)  && seg_done && (q_cnt != 3'd0)) ||
                    ext_load;
    // ★V2.7（清单3 配套）★：ext 段"同动作接同动作"也不许清积分。
    //   旧写法只认 A_FWD/A_FWD，而绕障腿是 A_FWD_HOLD ⇒ 每装一个新小步都清一次积分，
    //   步长缩到 40mm 后就是 ~3Hz 的"顿一下再起步"，现场看着像走走停停。
    wire skip_iclr = ((cur_act == A_FWD) && (head_act == A_FWD)) ||
                     (ext_load && (ext_act == cur_act));
    reg  i_clr_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            i_clr_r <= 1'b0;
        else
            i_clr_r <= seg_load && (~skip_iclr);
    end
    wire i_clr = i_clr_r;

    // =========================================================================
    // 7) PID × 2
    // =========================================================================
    // ★V2.9★：CLK_FREQ_HZ 用于"段间短停不清积分"算时长（必须写在 #(...) 参数表里——
    //   写进端口连接表 ModelSim 会报 (vopt-2912) Port not found）
    motor_pid #(.CLK_FREQ_HZ (CLK_FREQ)) u_pid_l (
        .clk (clk), .rst_n (rst_n), .run (pid_run), .i_clr (i_clr),
        .target (target_l), .actual (spd_l_eff), .actual_strobe (strobe_l),
        .kp (kp_eff), .ki (ki_eff), .kd (KD),
        .duty (duty_pid_l), .err_out (), .p_term_out ()
    );
    motor_pid #(.CLK_FREQ_HZ (CLK_FREQ)) u_pid_r (
        .clk (clk), .rst_n (rst_n), .run (pid_run), .i_clr (i_clr),
        .target (target_r), .actual (spd_r_eff), .actual_strobe (strobe_r),
        .kp (kp_eff), .ki (ki_eff), .kd (KD),
        .duty (duty_pid_r), .err_out (), .p_term_out ()
    );

    // =========================================================================
    // 8) TB6612 × 2
    // =========================================================================
    tb6612_driver #(.CLK_FREQ (CLK_FREQ), .PWM_FREQ (PWM_FREQ), .DUTY_MAX (DUTY_MAX)) u_drv_l (
        .clk (clk), .rst_n (rst_n), .pwm_enable (pwm_enable), .stop_mode (1'b1),
        .duty_cmd (duty_cmd_l),
        .ain1 (ain1_l), .ain2 (ain2_l), .pwm (pwma)
    );
    tb6612_driver #(.CLK_FREQ (CLK_FREQ), .PWM_FREQ (PWM_FREQ), .DUTY_MAX (DUTY_MAX)) u_drv_r (
        .clk (clk), .rst_n (rst_n), .pwm_enable (pwm_enable), .stop_mode (1'b1),
        .duty_cmd (duty_cmd_r),
        .ain1 (ain1_r), .ain2 (ain2_r), .pwm (pwmb)
    );

    // =========================================================================
    // 9) 编码器 × 2
    // =========================================================================
    // 【左右配对】默认（7号不拨）：u_enc_l 读 enc_ar/enc_br、u_enc_r 读 enc_al/enc_bl，
    //   即"交叉接法"——这是原作者手转轮子实测后的结论（左轮的运动出现在 ar/br 上）。
    //   后来驱动板上的两个电机输出端口被互换过，这个配对可能就不再成立了，
    //   于是加了 7号(sw[6]) 现场切换：拨到 ON 就变成不交叉（各读各的）。
    encoder_decode #(.CLK_FREQ (CLK_FREQ), .SPEED_MS (SPEED_MS), .INVERT (0)) u_enc_l (
        .clk (clk), .rst_n (rst_n),
        .enc_a (enc_a_l), .enc_b (enc_b_l),
        .count (cnt_l), .speed (spd_l), .dir (dir_l), .speed_strobe (strobe_l)
    );
    encoder_decode #(.CLK_FREQ (CLK_FREQ), .SPEED_MS (SPEED_MS), .INVERT (0)) u_enc_r (
        .clk (clk), .rst_n (rst_n),
        .enc_a (enc_a_r), .enc_b (enc_b_r),
        .count (cnt_r), .speed (spd_r), .dir (dir_r), .speed_strobe (strobe_r)
    );

    // iv) 反馈反向（等效于把该路编码器 A/B 对调）：
    //     默认值 INV_L_DEF/INV_R_DEF 是 2026-09-28 实测出来的（见文件头注释）；
    //     正常模式(1号/SW0 不拨)下 SW2/SW3 拨ON 可以把默认再翻一次（现场微调用）。
    //     注意：默认值在【所有模式】都生效，所以调试/整定模式下极性也是对的。
    wire inv_l = INV_L_DEF ^ ((~dbg_en) & (~sw[2]));
    wire inv_r = INV_R_DEF ^ ((~dbg_en) & (~sw[3]));
    // 速度取负：组合实现（16 位，很短；PID 的 actual 不能被打一拍，否则相当于多一个控制周期延迟）
    assign spd_l_eff = inv_l ? ((~spd_l) + 16'd1) : spd_l;
    assign spd_r_eff = inv_r ? ((~spd_r) + 16'd1) : spd_r;
    // 计数/速度取负：★打印用的打一拍★ —— 32 位加法 + 串口/mm 换算的组合路径
    // PnR 实测 23.7ns（超 20ns 周期），打一拍后打印路径从寄存器出发，裕量正常。
    // 注意：PID 用的 spd_?_eff 必须保持组合（不能打拍），否则等于给控制环多一个周期延迟。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_l_eff <= 32'd0;
            cnt_r_eff <= 32'd0;
            spd_l_prt <= 16'sd0;
            spd_r_prt <= 16'sd0;
        end
        else begin
            cnt_l_eff <= inv_l ? ((~cnt_l) + 32'd1) : cnt_l;
            cnt_r_eff <= inv_r ? ((~cnt_r) + 32'd1) : cnt_r;
            spd_l_prt <= inv_l ? ((~spd_l) + 16'd1) : spd_l;
            spd_r_prt <= inv_r ? ((~spd_r) + 16'd1) : spd_r;
        end
    end

    // =========================================================================
    // 10) 心跳 + 调试模式指示
    //     led[7]：正常模式 = 1s 慢闪（心跳）；调试模式 = 0.125s 快闪（一眼看出）
    //     led[6]：调试模式下 = 开环点动模式指示（亮 = 开环点动，灭 = 闭环整定）
    //            正常模式下仍是"右轮在转"
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt    <= 25'd0;
            heartbeat <= 1'b0;
        end
        else if (hb_cnt >= (CLK_FREQ / 2) - 1) begin
            hb_cnt    <= 25'd0;
            heartbeat <= ~heartbeat;
        end
        else
            hb_cnt <= hb_cnt + 25'd1;
    end

    reg  [23:0] dbg_hb_cnt;
    reg         dbg_hb;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_hb_cnt <= 24'd0;
            dbg_hb    <= 1'b0;
        end
        else if (dbg_hb_cnt >= (CLK_FREQ / 8) - 1) begin
            dbg_hb_cnt <= 24'd0;
            dbg_hb    <= ~dbg_hb;
        end
        else
            dbg_hb_cnt <= dbg_hb_cnt + 24'd1;
    end

    // =========================================================================
    // 11) LED
    // =========================================================================
    // LED（丝印 0..7 = led[0..7]；注意丝印 0/1 被挪到扩展口 C15/B15，板上没有灯）
    //   丝印 2 (led[2]) = 入队闪 1 下确认，随后按队列深度 N 循环慢闪
    //   丝印 3 (led[3]) = 正在执行一段
    //   丝印 4 (led[4]) = 按键急停锁存（避障状态由 final_top 覆盖到这根线）
    //   丝印 5/6       = 左/右轮在转（调试模式下 6 = 开环点动指示）
    //   丝印 7         = 心跳（1s 慢闪；调试 0.125s 快闪）
    // ---- LED2 闪码：入队确认闪 1 下，随后按队列深度慢闪 N 下循环（N=1~4）----
    reg  [24:0] l2_div;
    reg  [4:0]  l2_left;   // 剩余闪烁次数
    reg         l2_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l2_div <= 25'd0; l2_left <= 5'd0; l2_q <= 1'b0;
        end
        else if (push_req) begin
            // 确认 1 下；灭完后再按 q_cnt（含本下）循环 N 下
            l2_left <= 5'd1;
            l2_div  <= 25'd0;
            l2_q    <= 1'b1;
        end
        else if (l2_left != 5'd0) begin
            if (l2_div >= 25'd12_500_000) begin  // 0.25s 半周期 → 2Hz
                l2_div <= 25'd0;
                l2_q   <= ~l2_q;
                if (l2_q)
                    l2_left <= l2_left - 5'd1;
            end
            else
                l2_div <= l2_div + 25'd1;
        end
        else if (q_cnt != 3'd0) begin
            // 队列非空：灭一段时间后重播 N 下（N=q_cnt）
            l2_q <= 1'b0;
            if (l2_div >= 25'd25_000_000) begin  // 0.5s 间隔
                l2_div  <= 25'd0;
                l2_left <= {2'b0, q_cnt};
                l2_q    <= 1'b1;
            end
            else
                l2_div <= l2_div + 25'd1;
        end
        else begin
            l2_q   <= 1'b0;
            l2_div <= 25'd0;
        end
    end
    assign led[2]   = l2_q;
    assign led[3]   = (st == S_RUN) || (st == S_EXT_RUN);
    assign led[4]   = estop_lat;
    // 串口打印统一成"车体坐标系"：前进 = 正（两轮一致，便于看现象）
    //   PID 用的 spd_?_eff 是各轮硬件极性修正后的量；这里打印它的相反数，
    //   于是"dl/dr 递增、sl/sr 为正"就等价于"该轮往前转"。
    //   ★再打一拍★：取负 + 串口 mm 换算的组合路径太长（PnR 会掉到 1.9ns 裕量）
    reg  signed [15:0] spd_l_out, spd_r_out;
    reg  [31:0]        cnt_l_out, cnt_r_out;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spd_l_out <= 16'sd0;  spd_r_out <= 16'sd0;
            cnt_l_out <= 32'd0;   cnt_r_out <= 32'd0;
        end
        else begin
            spd_l_out <= (~spd_l_prt) + 16'sd1;
            spd_r_out <= (~spd_r_prt) + 16'sd1;
            cnt_l_out <= (~cnt_l_eff) + 32'd1;
            cnt_r_out <= (~cnt_r_eff) + 32'd1;
        end
    end

    assign led[5]   = (spd_l_eff != 16'sd0);
    assign dbg_cnt_l = cnt_l_out;
    assign dbg_cnt_r = cnt_r_out;
    assign ext_arc_cnt = rot_avg_r;   // ★V2.7（清单6）★：本段实测弧长（编码器计数）
    assign dbg_spd_l = spd_l_out;
    assign dbg_spd_r = spd_r_out;
    assign dbg_yaw   = yaw_abs;      // 诊断用：本段起点的 yaw 变化量
    // ★2026-10-15 阶段0：把电机内部的"段动作/状态/转向命令"引出来给串口打印★
    assign dbg_act   = cur_act;
    assign dbg_st    = st;
    // ★V2.18y★ 调试：A_FWD_HOLD 段 t= 显示 hold_corr（航向保持差速），
    //   其它段仍显示 turn_cmd；不增加串口行长。
    assign dbg_turn  = (cur_act == A_FWD_HOLD) ? hold_corr[15:0] :
                       (rel_pivot && imu_ok) ? turn_pid_cmd[15:0] :
                       turn_cmd[15:0];
    assign led[6]   = dbg_en ? dbg_open : (spd_r != 16'd0);   // 调试时=开环点动指示
    assign led[7]   = dbg_en ? dbg_hb   : heartbeat;          // 调试时=快闪

endmodule
