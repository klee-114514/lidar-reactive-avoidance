//=============================================================================
// final_leida_top.v —— 车尾=新车头 + RPLIDAR A1 激光雷达避障（v1.1 雷达链路第一段）
//-----------------------------------------------------------------------------
// v1.0（2026-10-12）从 final_servo 完全拷贝而来，只做三件事：
//   ① 工程/顶层/约束改名为 final_leida
//   ② ★超声波 + 舵机 + scan_ctrl 全部停用★：三个模块不进 gprj、顶层不例化、
//      端口也不声明；感知输入改用"永远有效、前方 4000mm 空旷"的常量桩，
//      所以本版【没有任何避障能力】，只会按队列跑路（有意为之，方便先跑通骨架）。
//   ③ 其余（电机/队列/PID/IMU/按键/红外/手柄模式/串口三行打印/avoid_fsm）原样保留
// ---------------------------------------------------------------------------
// v1.3（2026-10-14）① 修"断流重发误判" ② 加帧解析（★车的行为不变★）
//   ① 断流判据原来用【16 位饱和】的字节计数器：它一饱和就不再变化 ⇒ 被误判成断流
//      ⇒ 每 2s 重发一次 A5 25+A5 20 ⇒ 雷达每 2 秒被 STOP/SCAN 一次、扫描被打断。
//      上板证据：T 每 2 秒 +1。现在改用 lidar_rx 的【activity 翻转位】（永不饱和）。
//   ② 新增 lidar_parse.v：把 5 字节节点拆成角度/距离，归约成三个扇区的最小距离
//      （正前 90°±10°、车左前 135°±10°、车右前 45°±10°），每圈锁存一次，
//      三行打印的 df/L/R/W/F 改用这些真实值（W=最近一个点的角度）。
//      ★avoid_fsm 仍然吃 v1.0 的常量桩（前方永远 4000mm 空旷）★
//      ⇒ 本版【车仍然不会避障】，只是让我们先把雷达看准。v1.4 才接进 avoid_fsm。
// ---------------------------------------------------------------------------
// v1.1（2026-10-13）加雷达链路第一段：命令 + 接收（★不做解析★）
//   · 新增 lidar_rx.v：115200、16× 过采样取中点（现有 uart_rx 是 9600 单点采样）
//   · 新增 lidar_cmd.v：上电等 1s → A5 25（停止）→ 10ms → A5 20（开始扫描）；
//     之后每 2s 收不到新字节就重发一次 A5 20
//   · 新引脚：lidar_rx = D14（雷达 TX → FPGA）、lidar_tx = D15（FPGA → 雷达 RX）
//   · 三行串口打印的 V/T 临时改口径：V = 雷达收到的字节数、T = 命中 A5 5A 头的次数
//     ★上板判据：T ≥ 1 ⇒ "命令发出 + 雷达应答 + 接收采对"三件事同时成立★
//   下一步：v1.2 加帧解析（角度/距离）、v1.3 扇区表接入 avoid_fsm
//          （avoid_fsm 端口一行不改，只换 front/prof/watch 的数据来源）
//-----------------------------------------------------------------------------
// 下面这段是 final_servo（v3.1）的原说明，对电机/绕障几何部分仍然有效：
// final_servo_top.v —— 车尾=新车头 + HC-SR04 舵机扫描避障（v3.1：90° Z 形绕障）
//-----------------------------------------------------------------------------
// 复用：motor_driver / keys / IR / IMU / uart 打印
// 新增：hcsr04_ranger + servo_pan + scan_ctrl + avoid_fsm
// 方向：仅调转车头（LEFT_NEG/RIGHT_NEG=0/0），左右通道/编码器不动
//
// v3.1（2026-10-10）相对 v3.0（70° 梯形慢速绕障）：
//   ① 形状换回 90° 垂直侧移的 Z 形（θ=90°）——侧移腿完全不朝障碍推进，
//      所以触发距离能从 600mm 退回 280mm，而且对任何宽度的障碍都安全
//   ② 绕障直行腿不再压慢（用巡航速度），只有转弯腿压到 75% 转速
//   ③ 新增 RETURN_TO_LINE：1=4 次转向回原航线(~25s)；0=2 次转向留平行线(~10s)
//   ④ v3.0 修好的东西全部保留：want_avoid 锁存 + 定时重扫（修"永远不避障"）、
//      舵机角度镜像修正、LED4 分码、串口 f=XX 状态字节
// 阈值：D_SLOW=450 减速+扫剖面；D_STOP=280 触发避障
//=============================================================================
module final_leida_top #(
    parameter CLK_FREQ = 50_000_000,
    parameter REAR_AS_FRONT = 1'b1,
    parameter [15:0] D_DEAD_MM  = 16'd30,
    // ★★2026-10-11：D_STOP 必须【等于】D_SLOW★★
    //   旧版 D_STOP=280 < D_SLOW=450，中间 280~450mm 是一条【死区】：
    //   既不急停（要 ≤280）也不进绕障（要 ≤280），只是用减速档慢慢顶着走。
    //   实车实测：20cm 纸箱被一路推着走、LED4 一直 1Hz 闪（=slow_lat）、永不触发。
    //   所以"进入减速区"就等同于"被堵" —— 停车 + 进绕障。
    //   注：θ=90° 的侧移不朝障碍推进，450mm 触发的余量很宽裕（最坏角点还有 ~325mm）。
    // ★V2.15★：450 → 550；★V2.17b★：**按用户要求改回 450**——
    //   用户原话「不需要550mm，车还远着呢」⇒ 恢复他熟悉/实测合适的触发距离。
    //   （速度提到巡航 560≈194mm/s 后，急停是在 D_STOP 触发、PWM 直接切断，450 仍留得住余量。）
    // ★V4.2j★ 按用户 2026-10-21 明确要求：触发距离不要提前，恢复 450。
    parameter [15:0] D_STOP_MM  = 16'd450,
    parameter [15:0] D_SLOW_MM  = 16'd450,   // 减速 + 开始扫剖面（与 D_STOP 保持一致）
    parameter [15:0] D_HYST_MM  = 16'd50,
    // ================= 绕障几何（★v1.4 按实车 240×170 重算★）=================
    // 单位 mm；量法见 README「几何实测」
    parameter integer CAR_L_MM   = 240,   // 车长（前后最外缘之间）
    parameter integer CAR_W_MM   = 170,   // 车宽（左右最外缘之间）
    // ★v1.5：雷达装在【前进方向偏后 150mm】处（实测：雷达中心距车头前缘 150mm、
    //   距侧面 85mm ⇒ 横向正好在车体正中）★
    //   · 正前距离要减掉它，D_STOP=450 才恢复成"车头前缘前 450mm"的字面含义；
    //   · M1 公式里它是【减】项（射线扫过障碍末端那一刻，车尾离末端只剩
    //     CAR_L−SEN_FWD = 90mm）。SEN_FWD_MM=0 时两条公式与旧超声版逐字相同。
    //   · 扫描面高于车体上表面 ⇒ 车角（最远 172mm）不会被自己看见。
    parameter integer SEN_FWD_MM = 150,   // 探头面到车体最前缘（前进方向那一端）
    // ============ ★2b：绕障不再有任何"死角度/死距离"★ ============
    //   老的 THETA_DEG / SIN_Q10 / COS_Q10 / RETURN_TO_LINE / L_MAX_MM 已随
    //   "90° Z 形"整套作废：转角由 avoid_fsm 按【实测量】算出来
    //   （障碍段两端横向位置 + 含轮车宽 + 余量 + 障碍近面到后轮轴的距离）。
    //   下面这些都只是"车的物理常数 / 安全余量 / 兜底阈值"，不是策略角度。
    parameter integer AXLE_FWD_MM = 175,   // 后轮轴 → 车头前缘（实测 17.5cm）
    parameter integer CARW_ALL_MM = 220,   // ★含轮最外缘总宽★（车体170 + 两侧各25）
    parameter [15:0] D_PASS_MM   = 16'd900,   // 盯障碍侧判"看不见了"的门槛（lidar_parse 用）
    // ★V2.8（清单8）★：60 → 90mm（给"蹭到"留余量；同时收紧"那一侧还剩多少地方"）。
    // ⚠️ 代价：能绕过去的场合会变少（更容易判"两侧都不够"⇒ 停车 + 2 短声）。
    parameter [15:0] M_SIDE_MM   = 16'd90,    // 侧向安全余量（半宽+余量 = 110+90 = 200mm）
    // ★V2.9b★：220 → 300，语义明确为【回头扫掠净空】——把"能不能回头"交给实测：
    //   障碍侧 0°/180° 窗口的实测垂距 ≥ 这个值 且 正前连续 3 圈看不到障碍 ⇒ 回头。
    //   300 = 车体扫掠半径(240×220 车：半对角163、工程取211) + 侧向余量 M_SIDE(90)。
    //   这是【距离】(由车体尺寸+余量推出)，不是策略角度；车体尺寸变了要跟着改。
    parameter [15:0] SIDE_MIN_MM = 16'd300,   // 回头扫掠净空（障碍侧实测垂距门槛）
    parameter [15:0] PHI_MAX_DEG = 16'd60,    // 允许的最大转角（几何上一般 20~45°）
    parameter [15:0] ADV_MIN_MM  = 16'd360,   // 末端判据的"最少前进量"（车长240 + 120）
    parameter [15:0] STEP_MM     = 16'd80,    // 斜行/回线的小步长
    // ★V2.7（清单3）：80 → 40mm★ 判定粒度翻倍（一步一测），每步 ≈0.32s@巡航
    parameter [31:0] EXTRA_MAX_MM= 32'd800,   // 绕障【额外】里程上限（80cm，用户定）
    // ★V2.9★：12000 → 18000ms；★V2.13★：18000 → 25000ms。
    //   ⚠️ 这里必须和 `avoid_fsm` 的默认值一致——V2.12 我只改了 avoid_fsm 的默认值，
    //      顶层这个数把它盖掉了，现场仍是 18.0s ⇒ 回线被砍、车停在偏离 264mm 处
    //      （用户 23:45 日志：进绕障 28.7s → 46.7s 正好 18.0s 判 w=35）。
    //      提速后一趟绕障估 ≈12~15s，25s 留足余量。
    // ★V4.2e★ 回线角放平后回线腿更长（横移 300mm 约 1.4m 路径 ≈11s），25s 会误触 w=35
    parameter [31:0] AVOID_TO_MS = 32'd32000, // 绕障总时长上限
    parameter [3:0]  RETRY_MAX   = 4'd1,      // 允许"再绕一次"
    // ================= ★v1.6 板载调试指示（LED + 无源蜂鸣器）★ =================
    parameter integer BEEP_HZ      = 4000,    // 无源蜂鸣器驱动方波频率（2~5kHz 才响）
    parameter integer BEEP_SHORT_MS= 120,     // 短声时长
    parameter integer BEEP_LONG_MS = 600,     // 长声时长
    parameter integer BEEP_REP_MS  = 3000,    // 断流复读周期
    parameter integer BEEP_GAP_MS  = 120,     // 短声之间的间隔
    parameter integer BEEP_DEAD_DLY_MS = 2000,// 断流多久才开始长鸣（给雷达上电留时间）
    parameter        BEEP_INVERT   = 1'b0,    // 若板上是低电平触发，把它改成 1'b1
    // ================= ★v1.8 蓝牙调试串口（镜像同一条日志）★ =================
    //   把 M15 那三行调试日志【同时】送到 bt_tx（D19，扩展口 GPIO0_31），
    //   手机连上蓝牙串口模块就能无线看日志，不用一直拖着 USB 线。
    //   ★v1.8（2026-10-14）：默认由 115200 改为 9600★ —— 现场用的 HC-05
    //     （ZS-040 底板）出厂 UART 就是 9600；而蓝牙 SPP 是透明传输，
    //     手机串口助手 App 里的"波特率"根本不参与传输，所以唯一必须对上的
    //     就是这一段：FPGA 115200 撞模块 9600 ⇒ 蓝牙收到的全是乱码喵。
    //   ★波特率必须和你蓝牙模块的一致★：HC-05/HC-06/JDY-31 这类默认 9600；
    //     若换成 115200 的模块（部分 BLE），把这里改回 115200 即可。
    //   打印节拍要等慢的那一路发完：9600 下周期从 ~134ms 变成 ~285ms
    //   （日志刷新变慢，内容一字不少）喵。
    parameter integer BT_BAUD     = 9600
)(
    input  wire       clk,
    input  wire       key_stop,
    input  wire       key_start,
    input  wire       key_fwd,
    input  wire       key_left,
    input  wire       key_r,
    input  wire [7:0] sw,
    // 电机（物理脚：A=原左，B=原右；本层做车尾=车头变换）
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
    // ★v1.0：HC-SR04 / 舵机全部停用，这三个端口连声明都不留★
    //   雷达接口在 v1.1 加进来（见下）
    input  wire       imu_rx,
    // ★v1.1 雷达 RPLIDAR A1M8★：核心接口的 TX/RX 交叉接 FPGA 的 D14 / D15
    //   手册规格：115200 8N1；输出高 2.9~3.5V、输入高 ≥1.6V ⇒ 3.3V CMOS，直连、不用电平转换
    input  wire       lidar_rx,
    output wire       lidar_tx,
    input  wire       ir_rx,
    output wire       uart_tx,
    // ★v1.6 板载调试指示（不接电脑也能听/看状态）★
    //   beep = N19（板上【无源】蜂鸣器，经三极管驱动）：要 2~5kHz 方波才响
    //          上电就绪 2 短声 / 雷达断流每 3s 一长声 / 恢复 1 短声 /
    //          进绕障 3 短声 / 绕障结束 1 短声 / 绕障失败长鸣不止
    //   ★不做 led_dbg★：板上 LED0/LED1 的管脚 M22/N22 在本封装是专用脚（SSPI），
    //     约束上去 P&R 直接报 PR2017 —— 这就是原来"不用 M22/N22"的原因。
    //     "雷达健康"改由 led[1]（B15 扩展口）输出，想看得见就在扩展口接一颗 LED 喵。
    output wire       beep,
    // ★v1.7 蓝牙调试串口（只有输出：把日志镜像过去）★
    //   bt_tx → 蓝牙模块的 RX；蓝牙模块的 TX 暂时不用接
    //   （以后要做"手机发指令给车"，再加一个 bt_rx 输入脚就行，见 docs/接线说明.md）
    output wire       bt_tx,
    output wire [7:0] led
);

    // ★v1.4 雷达感知口径参数★（M1/D4/L_MAX/RETURN_TO_LINE 等 90° 时代的量已删除）
    localparam [15:0] SENS_FAR_MM   = 16'd4000;   // 扇区无点 ⇒ 折算成这个"远"距离
    localparam integer SENS_STALE_CYC = CLK_FREQ/2;  // 0.5s 没成圈 ⇒ 判断流
    localparam [7:0]  WATCH_PASS_K  = 8'd3;       // 盯视"连续几圈很远"才算末端过去
    localparam [15:0] HALF_F_Q6     = 16'd1280;   // 正前扇区半宽 ±20°
    // ★V2.28★：与 lidar_parse 的默认值和 V2.17 的现场结论对齐（±20°）。
    //   旧顶层一直用 640（±10°）覆盖掉了模块默认的 1280，导致 110~125°/145~170°
    //   仍有盲缝，d_obs 会在“看得到/看不到”之间跳。
    localparam [15:0] HALF_S_Q6     = 16'd1280;   // 其余扇区半宽 ±20°
    // 传给 avoid_fsm 的 16 位版本（integer 参数做位选，避免宽度不匹配告警）
    localparam [15:0] CARW16   = CARW_ALL_MM[15:0];   // ★含轮总宽★
    localparam [15:0] CARL16   = CAR_L_MM[15:0];
    localparam [15:0] AXLE16   = AXLE_FWD_MM[15:0];
    localparam [15:0] SENS16   = SEN_FWD_MM[15:0];


    // ---------------- 上电复位 ----------------
    reg [24:0] por_cnt = 25'd0;
    reg        por_done = 1'b0;
    always @(posedge clk) begin
        if (!por_done) begin
            if (por_cnt >= 25'd2_000_000) por_done <= 1'b1;
            else por_cnt <= por_cnt + 25'd1;
        end
    end
    wire rst_n = por_done;

    // ------------- ★v1.0 感知桩：超声/舵机/扫描停用，语义量全部固定★ -------------
    //  这些名字原来由 scan_ctrl 产生，v1.4 起由 lidar_parse 的"控制口径"输出顶替
    //  （同名同义），所以 avoid_fsm 及以下所有逻辑【一行都不用改】。
    //  ★v1.4：常量桩已删除，全部改成真实雷达量★
    //    · 扇区无点 ⇒ 折算 4000mm"远"（= 超声超时值，语义都是"那个方向没有东西"）
    //    · 0.5s 没成圈 ⇒ 正前强制 0mm 且 echo_to=1 ⇒ 顶层 sensor_fault ⇒
    //      停车 + LED4 8Hz —— 与超声版"探头失灵被当成被堵"同一条路径同一个指示
    // 术语：导航模式=按键/遥控规划入队后自动跑；手柄模式=遥控点动
    // ---- ★v1.4 雷达感知层网线声明★（必须在下面引用之前；例化在文件后半段）----
    wire [15:0] ld_front_mm, ld_left_mm, ld_right_mm, ld_sl_mm, ld_sr_mm;
    wire        ld_front_ok, ld_left_ok, ld_right_ok, ld_sl_ok, ld_sr_ok;
    wire [15:0] ld_scan_pts, ld_last_ang, ld_last_dist;
    wire [15:0] ld_node_cnt, ld_scan_cnt, ld_bad_cnt;
    wire        ld_scan_done_p, ld_rev_no_pt, ld_scan_fresh;
    wire [15:0] ld_dist_f_mm, ld_prof_l_mm, ld_prof_r_mm, ld_watch_mm;
    wire        ld_dist_f_v, ld_echo_to, ld_prof_l_v, ld_prof_r_v, ld_watch_v;
    wire [7:0]  ld_watch_far_cnt;
    // ---- ★2a：障碍物角向连续段（前半球最近的那一段）＋ 正前左右半区★ ----
    wire [15:0] ld_obs_a0, ld_obs_d0, ld_obs_a1, ld_obs_d1;
    wire [15:0] ld_obs_dmin, ld_obs_amin;
    wire        ld_obs_valid;
    wire [15:0] ld_hist_data;         // ★V4.0a★ 72 扇区直方图读数据（暂未接控制）
    wire [15:0] ld_hist_side_l, ld_hist_side_r; // ★V4.0c★ 直方图侧距
    wire [6:0]  av_hist_addr;         // ★V4.0b★ avoid_fsm 读直方图地址
    wire [15:0] ld_front_l, ld_front_r;
    wire        ld_front_l_ok, ld_front_r_ok;
    wire        dist_valid      = ld_scan_done_p;   // 每完成一圈一个脉冲（喂 echo_age 判链路死活）
    wire        echo_to         = ld_echo_to;       // 断流电平（0.5s 没成圈就一直是 1）
    wire        servo_init_done = 1'b1;             // 无舵机：恒 1（导航门控要用）
    wire [15:0] dist_f          = ld_dist_f_mm;     // 正前（已折算到车头前缘；断流时 0）
    wire        dist_f_v        = ld_dist_f_v;      // 正前样本有效（= 链路新鲜）
    wire        scan_done       = ld_scan_done_p;   // 剖面可用（avoid_fsm 的启动门控之一）
    wire [15:0] prof_l_mm       = ld_prof_l_mm;
    wire        prof_l_v        = ld_prof_l_v;
    wire [15:0] prof_r_mm       = ld_prof_r_mm;
    wire        prof_r_v        = ld_prof_r_v;
    wire [15:0] watch_mm        = ld_watch_mm;
    wire        watch_v         = ld_watch_v;
    wire [7:0]  watch_far_cnt   = ld_watch_far_cnt;
    wire [2:0]  scan_st         = 3'd0;    // 串口 s= 字段，暂无用途
    wire        watch_en_w;           // avoid_fsm → lidar_parse：盯视使能
    wire [7:0]  watch_ang_w;          // avoid_fsm → lidar_parse：盯视角度（0/180）
    wire        av_ext_slow;          // avoid_fsm → 仲裁器：绕障腿走慢速档
    wire        avoid_active, avoid_fail, avoid_done;
    wire [3:0]  avoid_st;
    wire [7:0]  avoid_why;
    wire        avoid_side;
    wire [15:0] avoid_lat;
    wire [15:0] avoid_phi;   // ★V2.2：本次算出的角度（度）★
    wire [15:0] avoid_fwd_acc; // ★V2.18w★ 临时调试：绕障前进投影
    wire [15:0] avoid_g;       // ★V2.18w★ 临时调试：stall/ret/aim/retry/busy
    wire [7:0]  avoid_flags;   // ★V4.0d★ VFH/侧距/正前/crush 决策标志
    wire [15:0] avoid_shadow;  // ★V4.1a★ 影子几何规划器：左右最小可行档
    wire        ext_active;   // 须在 force_stop 逻辑前声明
    // ★V5.0★ 迷宫层与动作仲裁。迷宫只借用现有 ext_* 接口，不改电机协议。
    wire        maze_mode = sw[0] & (~sw[1]);   // SW0=正常，SW1 ON=迷宫
    wire [6:0]  maze_hist_addr;
    wire        maze_ext_load, maze_ext_to_queue, maze_ext_slow;
    wire [2:0]  maze_ext_act;
    wire [31:0] maze_ext_n;
    wire [15:0] maze_ext_yaw_tgt;
    wire        maze_active, maze_done_p, maze_fail;
    wire        maze_allow_avoid, maze_heading_changed;
    wire [3:0]  maze_st;
    wire [7:0]  maze_why;
    wire        maze_inside;
    wire [15:0] maze_right_mm, maze_left_mm;
    wire [15:0] maze_right_depth, maze_straight_depth, maze_left_depth;
    wire [15:0] maze_turn_min;
    wire [7:0]  maze_turn_cnt, maze_scan_cnt;
    // 导航模式：规划入队后自动跑（含 ext 绕障）；手柄模式：遥控点动
    wire        nav_mode, gp_mode;

    // ★v1.0：hcsr04_ranger / servo_pan / scan_ctrl 三个例化整体删除★
    //   文件仍在 rtl/ 目录里（作参考、将来可恢复），但**不在 gprj 里**——
    //   这一点很重要：孤儿模块进 gprj 会被综合器当成顶层，报 PA2024 端口超限。
    // （原 scan_ctrl 例化已随超声/舵机一起停用，v1.3 由 lidar_sector 顶替）

    // 正前失效解锁：
    //   · 正前样本超过有效期（舵机在盯侧面/扫描节拍慢）→ 悬空
    //   · 或 2s 内一次有效回波都没有 → 认为测距链路坏了（断线/虚焊）
    //     （超时本身代表“>4m 空旷”，所以不能把超时当故障，否则开阔场地会一直 25% 慢速）
    reg [31:0] range_age, echo_age;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) range_age <= 32'd0;
        else if (dist_f_v) range_age <= 32'd0;
        else if (range_age < 32'd75_000_000) range_age <= range_age + 32'd1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) echo_age <= 32'd0;
        else if (dist_valid) echo_age <= 32'd0;
        else if (echo_age < 32'd100_000_000) echo_age <= echo_age + 32'd1;
    end
    wire sensor_dead = (echo_age >= 32'd100_000_000);
    // ★连续回波超时 ⇒ 当【传感器故障】，不再当成"空旷 4m"★
    //   旧设计：超时 = 有效样本 = 4000mm = 前方空旷 ⇒ 探头一失灵/线一松，车就全速冲。
    //   室内地板/墙一定有回波（17cm 高时地面约 0.63m），连续 10 次（≈0.7s）全丢
    //   基本就是链路坏了 ⇒ 按"被堵"处理：停车（+可进绕障）。
    reg [4:0] to_run;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) to_run <= 5'd0;
        else if (dist_valid) to_run <= 5'd0;
        else if (echo_to && (to_run < 5'd31)) to_run <= to_run + 5'd1;
    end
    wire sensor_fault = (to_run >= 5'd10);
    wire range_stale = sensor_dead | (~dist_f_v);

    // ---------------- 分级：减速 / 急停 ----------------
    // dist_f 连续 2 次有效才更新（去毛刺）
    reg estop_lat, slow_lat;
    reg [1:0] zone_st;
    reg [3:0] zone_cnt;
    localparam [3:0] CONF_N = 4'd2;   // 去抖加快，灯与门控同步

    wire [1:0] zone_raw = (dist_f <= D_DEAD_MM)  ? 2'd3 :
                          (dist_f <= D_STOP_MM)  ? 2'd2 :
                          (dist_f <= D_SLOW_MM)  ? 2'd1 : 2'd0;
    wire frame_p = dist_valid && dist_f_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zone_st <= 2'd0; zone_cnt <= 4'd0;
        end else if (range_stale) begin
            // ★样本失效时【保持最后判定】，只清去抖计数★
            //   旧版在这里把 zone_st 清 0 ⇒ 一失效就"解锁"⇒ 车闭着眼往前蹭
            zone_cnt <= 4'd0;
        end else if (frame_p) begin
            if (zone_raw == zone_st) zone_cnt <= 4'd0;
            // ★V4.1g★ 进入更近的区【立即生效】，只对"离开"去抖：
            //   旧的统一 2 帧去抖让触发实际晚了 1~2 圈（450 设定、397 才触发），
            //   直接把要绕的角度顶大了 10~15°。
            else if (zone_raw > zone_st) begin
                zone_st <= zone_raw; zone_cnt <= 4'd0;
            end
            else if (zone_cnt >= CONF_N - 4'd1) begin
                zone_st <= zone_raw; zone_cnt <= 4'd0;
            end else zone_cnt <= zone_cnt + 4'd1;
        end
    end

    wire motor_clr;
    // 前向空旷 0.5s 且已松手柄 → 自动恢复（E3）
    reg [31:0] clear_cnt;
    reg        recover_d;
    wire front_ok = dist_f_v && (dist_f > D_STOP_MM);
    wire recover_arm = (avoid_fail | estop_lat) & (~avoid_active) & (~gp_mode);
    wire recover_pulse;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) clear_cnt <= 32'd0;
        else if (front_ok && recover_arm) begin
            if (clear_cnt < 32'd25_000_010) clear_cnt <= clear_cnt + 32'd1;
        end else
            clear_cnt <= 32'd0;
    end
    wire recover_reach = (clear_cnt >= 32'd25_000_000);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) recover_d <= 1'b0;
        else recover_d <= recover_reach;
    end
    assign recover_pulse = recover_reach & ~recover_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            estop_lat <= 1'b0; slow_lat <= 1'b0;
        end else if (motor_clr || avoid_done || recover_pulse) begin
            // ★注意：这里【不再】因为 range_stale 就清 estop_lat★
            //   旧版"样本一失效就解锁"是"闭眼往前蹭"的第二个来源
            estop_lat <= 1'b0; slow_lat <= 1'b0;
        end else if (avoid_fail) begin
            estop_lat <= 1'b1;
        end else begin
            // A2：避障/迷宫监督运动期间不锁 estop，避免 force_stop 死锁；
            //     雷达断流和迷宫失败仍由下方独立硬停车路径处理。
            if (~(avoid_active | maze_active))
                estop_lat <= (zone_st == 2'd2) || (zone_st == 2'd3) || sensor_fault;
            else
                estop_lat <= 1'b0;
            slow_lat <= ((zone_st != 2'd0) | sensor_fault) & ~gp_mode;
        end
    end

    // ★前向被堵：zone2（≤D_STOP）与 zone3（≤D_DEAD 贴太近）都算★
    //   v3.2：传感器故障/彻底没回波也算"被堵"（宁可停，不可盲冲）
    wire front_blk = (zone_st == 2'd2) || (zone_st == 2'd3) || sensor_fault || sensor_dead;
    wire mission_hold;      // 手柄接管后锁存的“任务已中止、保持停车”（定义在下面）
    // 绕障进行中放行 force_stop；手柄模式完全绕过避障
    wire avoid_running = avoid_active | maze_active | ext_active;
    // ★v3.2：avoid_fail（绕障失败锁存）【直接】压住停车★
    //   旧版只通过 estop_lat 压，而 estop_lat 会被别的路径清掉 ⇒
    //   失败锁存后车还能继续跑（而且再也无法避障）⇒ 直冲障碍物
    wire obstacle_stop_raw = (estop_lat | (front_blk & ~avoid_running) |
                              mission_hold | avoid_fail | maze_fail);
    // ★V2.28★：avoid_fail 必须无条件压住电机。
    //   旧式最后再 & ~avoid_running，失败时若当前 ext 段还没结束，
    //   obstacle_stop 仍为 0 ⇒ 车会带着失败状态继续跑完这一段。
    wire maze_safety_stop = maze_active & (sensor_fault | sensor_dead | (~dist_f_v));
    wire obstacle_stop = avoid_fail | maze_fail | maze_safety_stop |
                         (obstacle_stop_raw & ~gp_mode & ~avoid_running);

    // ★v3.0：减速不再靠 1kHz 斩波，改成给电机模块一个"限速"电平★
    //   进减速区（zone≠0）/正前样本失效/传感器故障时按 SLOW_SPD 跑。
    wire slow_req = (zone_st != 2'd0) | (~dist_f_v) | sensor_fault;

    // ---------------- IR / 电机 ----------------
    wire [7:0]  ir_addr_w, ir_data_w;
    wire        ir_dec_done, ir_rep;
    wire        ir_fwd_w, ir_left_w, ir_right_w, ir_exec_w;
    wire        ir_stop_w, ir_clear_w, ir_cancel_w, ir_hold_w;
    wire [1:0]  ir_dist_w;
    wire        gp_fwd_w, gp_back_w, gp_left_w, gp_right_w;

    ir_decode u_ird (
        .clk(clk), .rst_n(rst_n), .iIR(ir_rx),
        .ir_addr(ir_addr_w), .ir_data(ir_data_w),
        .dec_done(ir_dec_done), .ir_repeat(ir_rep)
    );
    ir_remote_ctrl u_ir_map (
        .clk(clk), .rst_n(rst_n),
        .ir_addr(ir_addr_w), .ir_data(ir_data_w),
        .dec_done(ir_dec_done), .ir_repeat(ir_rep),
        .cmd_fwd(ir_fwd_w), .dist_sel(ir_dist_w),
        .cmd_left(ir_left_w), .cmd_right(ir_right_w),
        .cmd_exec(ir_exec_w), .cmd_stop(ir_stop_w),
        .cmd_clear(ir_clear_w), .cmd_cancel(ir_cancel_w),
        .stop_hold(ir_hold_w),
        .gp_fwd(gp_fwd_w), .gp_back(gp_back_w),
        .gp_left(gp_left_w), .gp_right(gp_right_w)
    );

    wire [7:0]  imu_byte;
    wire        imu_bv;
    wire [15:0] imu_yaw;
    wire        imu_yaw_v;

    wire        motor_stby;
    wire [7:0]  led_m;
    wire        m_pwma, m_pwmb;
    wire        m_ain1_l, m_ain2_l, m_ain1_r, m_ain2_r;
    wire [31:0] cnt_l, cnt_r;
    wire [15:0] spd_l, spd_r;
    wire [15:0] yaw_dbg;

    // ---- ★2026-10-15 阶段0 临时调试：电机内部"段动作/状态/转向命令"★ ----
    wire [2:0]  m_dbg_act;
    wire [1:0]  m_dbg_st;
    wire [15:0] m_dbg_turn;

    // ★V4.2b-dbg★ 诊断用：A_FWD_HOLD 的航向修正命令 vs 两轮实测差速
    //   高字节 = m_dbg_turn[7:0]（A_FWD_HOLD 时就是 hold_corr）
    //   低字节 = clamp(spd_l - spd_r, -128..127)
    wire signed [15:0] spd_diff_w = $signed(spd_l) - $signed(spd_r);
    wire [7:0] spd_diff8 = (spd_diff_w > 16'sd127) ? 8'd127 :
                           (spd_diff_w < -16'sd128) ? 8'h80 : spd_diff_w[7:0];
    wire [7:0] corr8 = m_dbg_turn[7:0];

    // avoid_fsm 与 maze_nav_fsm 各自产生一套 ext_*，顶层只允许一套到达电机。
    wire        av_ext_load, av_ext_to_queue, ext_done, remain_cap;
    wire [2:0]  av_ext_act;
    wire [31:0] av_ext_n, remain_out;
    wire [15:0] av_ext_yaw_tgt;
    wire        ext_load, ext_to_queue;
    wire [2:0]  ext_act;
    wire [31:0] ext_n;
    wire [15:0] ext_yaw_tgt;
    wire        ext_slow_w;

    // 避障拥有绝对优先权；迷宫只在自身 active 且避障未接管时发段。
    assign ext_load     = avoid_active ? av_ext_load :
                          (maze_active ? maze_ext_load : 1'b0);
    assign ext_act      = avoid_active ? av_ext_act  : maze_ext_act;
    assign ext_n        = avoid_active ? av_ext_n    : maze_ext_n;
    assign ext_yaw_tgt  = avoid_active ? av_ext_yaw_tgt : maze_ext_yaw_tgt;
    assign ext_to_queue = avoid_active ? av_ext_to_queue : maze_ext_to_queue;
    assign ext_slow_w   = avoid_active ? av_ext_slow : maze_ext_slow;
    wire [31:0] ext_arc_cnt;     // ★V2.7（清单6）★：电机给的"本段实测弧长"

    wire dbg_mode = (~sw[0]);
    // 导航模式 = 电机在跑导航段（led_m[3]）或绕障接管中
    // 手柄模式 = 遥控点动有效（优先，绕过避障）
    assign gp_mode  = (gp_fwd_w | gp_back_w | gp_left_w | gp_right_w);
    // 手柄接管后（mission_hold）不再扫描/不入队，避免舵机在障碍前反复来回扫
    assign nav_mode = (led_m[3] | avoid_active | ext_active) & (~gp_mode) &
                      (~mission_hold) & servo_init_done;

    // ================= ★v3.0：避障启动 = "锁存意愿 + 等剖面"★ =================
    // 旧版要求 front_blk 与 scan_done 【同一拍】成立，而剖面数据 1.5s 就过期、
    // ST_HOLD 又不会重扫 ⇒ 车慢慢爬到 D_STOP 时剖面早废了 ⇒ 永远不进避障。
    // 现象就是：LED4 常亮、车原地不动、不打舵、重新设定路线没用、按 S0 也没用
    //（S0 只是把锁存清掉，正前还在 D_STOP 内，下一拍又立刻锁上）。
    // 现在：正前被堵就把"想绕"这件事锁住，扫描端每 RESCAN_MS 保证给一份新剖面，
    //       两者什么时候凑齐什么时候开绕。
    reg want_avoid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) want_avoid <= 1'b0;
        else if (motor_clr | recover_pulse | gp_mode | avoid_done | avoid_fail)
            want_avoid <= 1'b0;
        else if (front_blk & led_m[3] & (~dbg_mode) & (~mission_hold))
            want_avoid <= 1'b1;
        else if (dist_f_v && (dist_f > (D_STOP_MM + D_HYST_MM)) && (~avoid_active))
            want_avoid <= 1'b0;      // 正前确实空了才撤意愿
    end

    // ★V4.35j★ 用户要求：按 S3/S4 原地左右转的时候不触发避障。
    //   顶层拿得到当前段动作 m_dbg_act（0=前进 1=左转 2=右转 3=左弧 4=右弧 5=绝对航向转），
    //   正在 A_LEFT_PIV/A_RIGHT_PIV 时不 arm。转完回 A_FWD 后恢复判定。
    //   避障自己的 S_T_OUT 用的是 A_YAW_ABS(act=5)，不会被这条挡住。
    wire pivoting = (m_dbg_act == 3'd1) || (m_dbg_act == 3'd2);
    wire avoid_arm_raw = want_avoid & (~avoid_active) & (~avoid_fail) & dist_f_v &
                         scan_done & (~gp_mode) & (~mission_hold) & (~dbg_mode) &
                         (~motor_clr) & servo_init_done & (~pivoting);
    // 迷宫模式下只有“确认是可绕小箱子”才放行既有避障；墙模式绝不能误接管。
    wire avoid_arm = avoid_arm_raw & ((~maze_mode) | maze_allow_avoid);

    // ================= 手柄接管 = 一次复位 =================
    // 手柄一动：立刻复位绕障状态机与扫描（avoid_abort），当前 100mm 小步走完即停；
    // 松手后车保持停住（mission_hold），直到按 S0（软复位）或 S1（重跑队列）。
    reg  ks_d, ks_dd;
    always @(posedge clk) begin
        ks_d  <= key_start;
        ks_dd <= ks_d;
    end
    wire key_start_evt = ks_dd & (~ks_d);          // 按键低有效：按下 = 下降沿
    reg  takeover_lat;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) takeover_lat <= 1'b0;
        else if (gp_mode) takeover_lat <= 1'b1;
        else if (motor_clr | ir_exec_w | recover_pulse | key_start_evt)
            takeover_lat <= 1'b0;
    end
    assign mission_hold = takeover_lat & (~gp_mode);
    wire avoid_abort  = gp_mode;

    // ★V5.0★ 迷宫启动与前级消抖：
    //   SW1 ON 后，S1 / 遥控 PLAY 只启动迷宫，不再启动旧动作队列；
    //   启动脉冲借用现有 ir_clear 清空队列，并用 hold 窗口避免 motor_clr 把刚启动的迷宫立即中止。
    wire maze_start_req = maze_mode & (~maze_active) &
                          (key_start_evt | ir_exec_w) & (~gp_mode);
    wire maze_start_clr = maze_start_req;
    reg  [3:0] maze_start_hold;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) maze_start_hold <= 4'd0;
        else if (maze_start_req) maze_start_hold <= 4'd8;
        else if (maze_start_hold != 4'd0) maze_start_hold <= maze_start_hold - 4'd1;
    end
    wire maze_abort = gp_mode | (~maze_mode) |
                      ((motor_clr | ir_stop_w) &
                       (maze_start_hold == 4'd0) & (~maze_start_req));
    reg maze_done_lat;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) maze_done_lat <= 1'b0;
        else if (maze_start_req || motor_clr || ir_stop_w) maze_done_lat <= 1'b0;
        else if (maze_done_p) maze_done_lat <= 1'b1;
    end

    motor_driver_top #(
        .CLK_FREQ (CLK_FREQ),
        .N_FWD_10 (32'd287000),
        .N_TURN_90 (32'd101600),
        .N_TURN_45 (32'd50800),
        .SAFE_MS (32'd20000)
    ) u_motor (
        .clk(clk),
        .force_stop(obstacle_stop),
        .slow_req(slow_req), .ext_slow(ext_slow_w),
        .key_stop(key_stop), .key_start(maze_mode ? 1'b0 : key_start),
        .key_fwd(key_fwd), .key_left(key_left), .key_r(key_r),
        .ir_fwd(ir_fwd_w), .ir_dist(ir_dist_w),
        .ir_left(ir_left_w), .ir_right(ir_right_w),
        .ir_exec(maze_mode ? 1'b0 : ir_exec_w), .ir_stop(ir_stop_w),
        .ir_clear(ir_clear_w | maze_start_clr), .ir_cancel(ir_cancel_w),
        .ir_stop_hold(ir_hold_w),
        .gp_fwd(gp_fwd_w), .gp_back(gp_back_w),
        .gp_left(gp_left_w), .gp_right(gp_right_w),
        .sw(sw),
        // 电机通道 1:1（仅车头反向在 LEFT/RIGHT_NEG）
        .ain1_l(m_ain1_l), .ain2_l(m_ain2_l), .pwma(m_pwma),
        .ain1_r(m_ain1_r), .ain2_r(m_ain2_r), .pwmb(m_pwmb),
        .stby(motor_stby),
        .enc_al(enc_al), .enc_bl(enc_bl),
        .enc_ar(enc_ar), .enc_br(enc_br),
        .led(led_m),
        .dbg_cnt_l(cnt_l), .dbg_cnt_r(cnt_r),
        .dbg_spd_l(spd_l), .dbg_spd_r(spd_r),
        .dbg_yaw(yaw_dbg),
        // ★阶段0 调试★：只读电机的段动作/状态机/转向命令
        .dbg_act(m_dbg_act), .dbg_st(m_dbg_st), .dbg_turn(m_dbg_turn),
        .ext_arc_cnt(ext_arc_cnt),   // ★V2.7（清单6）★ 本段实测弧长（计数）
        .yaw_raw(imu_yaw), .yaw_valid(imu_yaw_v),
        .clr_out(motor_clr),
        .ext_load(ext_load), .ext_act(ext_act), .ext_n(ext_n),
        .ext_yaw_tgt(ext_yaw_tgt), .ext_to_queue(ext_to_queue),
        .ext_done(ext_done), .ext_active(ext_active),
        .remain_out(remain_out), .remain_cap(remain_cap)
    );

    // ---- 电机通道 1:1（与约束手册一致，不交换左右）----
    // 前进/后退符号只在 motor_driver 的 LEFT_NEG/RIGHT_NEG
    // ★v3.0：这里只保留"急停切断"，不再兼做减速★（减速由 SLOW_SPD/AVOID_SPD 负责）
    wire gate = ~obstacle_stop;

    assign pwma   = m_pwma & gate;
    assign ain1_l = m_ain1_l;
    assign ain2_l = m_ain2_l;
    assign pwmb   = m_pwmb & gate;
    assign ain1_r = m_ain1_r;
    assign ain2_r = m_ain2_r;

    assign stby = motor_stby & ~obstacle_stop;

    // ---------------- 堵转检测（绕障腿在跑但两轮都不动）----------------
    // 车顶住墙/障碍时，100mm 小步永远走不完（要等 motor 的 SAFE_MS 兜底），
    // 这里 400ms 就判“堵转”，交给 avoid_fsm 立即认输，避免电机长时间堵转。
    // ★V2.18v★：25 位最大 33,554,431，永远到不了 40,000,000；
    //   800ms 堵转保护实际是死的。加宽到 26 位。
    reg [25:0] stall_cnt;
    reg        avoid_stall;
    // ★V2.28★：speed 是补码。旧式无符号比较把 -1 当成 65535 ⇒ 轮子明明不动
    //   （只是编码器抖一下负值）也被判成“在动”，stall_cnt 永远清零。
    wire [15:0] spd_l_mag = spd_l[15] ? (16'd0 - spd_l) : spd_l;
    wire [15:0] spd_r_mag = spd_r[15] ? (16'd0 - spd_r) : spd_r;
    wire       wheels_moving = (spd_l_mag > 16'd3) || (spd_r_mag > 16'd3);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stall_cnt   <= 26'd0;
            avoid_stall <= 1'b0;
        end else if ((!ext_active) || wheels_moving) begin
            stall_cnt   <= 26'd0;
            avoid_stall <= 1'b0;
        // ★V2.7（H3）★：400ms → 800ms。转向段开头有 DWELL_MS=200 的"停稳"
        //   （两轮本来就不转），400ms 的窗口只给它剩 200ms，很容易误报堵转。
        end else if (stall_cnt >= 26'd40_000_000) begin   // 800ms @50MHz
            avoid_stall <= 1'b1;
        end else
            stall_cnt <= stall_cnt + 26'd1;
    end

    avoid_fsm #(
        .CAR_W_MM    (CARW16),          // ★含轮最外缘总宽 220★
        .CAR_L_MM    (CARL16),
        .AXLE_FWD_MM (AXLE16),
        .SENS_FWD_MM (SENS16),
        .D_ARM_MM   (D_STOP_MM),
        .M_SIDE_MM  (M_SIDE_MM),
        .STEP_MM    (STEP_MM),
        .SIDE_MIN_MM(SIDE_MIN_MM),
        .PHI_MAX_DEG(PHI_MAX_DEG),
        .ADV_MIN_MM (ADV_MIN_MM),
        .PASS_K     (WATCH_PASS_K),
        .EXTRA_MAX_MM(EXTRA_MAX_MM),
        .AVOID_TO_MS(AVOID_TO_MS),
        .RETRY_MAX  (RETRY_MAX),
        .CNT_PER_MM (32'd574),
        .N_TURN_90  (32'd101600),
        .CLK_FREQ_HZ(CLK_FREQ)
    ) u_avoid (
        .clk(clk), .rst_n(rst_n),
        .front_mm(dist_f), .front_v(dist_f_v),
        .prof_l_mm(prof_l_mm), .prof_l_v(prof_l_v),
        .prof_r_mm(prof_r_mm), .prof_r_v(prof_r_v),
        .scan_done(scan_done),
        .watch_mm(watch_mm), .watch_v(watch_v), .watch_far_cnt(watch_far_cnt),
        .watch_en(watch_en_w), .watch_ang(watch_ang_w),
        // ★2a：障碍段两端 + 正前左右半区 + 正左/正右可用空间★
        //   注意命名：lidar_parse 的 sl=180°窗口=车正右、sr=0°窗口=车正左（历史命名是反的）
        .obs_a0(ld_obs_a0), .obs_d0(ld_obs_d0), .obs_a1(ld_obs_a1), .obs_d1(ld_obs_d1),
        .obs_dmin(ld_obs_dmin), .obs_amin(ld_obs_amin), .obs_valid(ld_obs_valid),
        .hist_rd_addr(av_hist_addr), .hist_rd_data(ld_hist_data),
        .hist_side_l_mm(ld_hist_side_l), .hist_side_r_mm(ld_hist_side_r),
        .front_l_mm(ld_front_l), .front_r_mm(ld_front_r),
        .front_l_ok(ld_front_l_ok), .front_r_ok(ld_front_r_ok),
        // ★V2.15★：35°~55° / 125°~145° 两格也接进去——"障碍侧实测最近距离"要取三格最小，
        //   否则车斜行(车身偏 20~30°)时箱子落在 135°~170° 空档里，两格都看不见。
        .wide_l_mm(ld_left_mm), .wide_r_mm(ld_right_mm),
        .left_mm(ld_sr_mm), .left_ok(ld_sr_ok),      // 0°  窗口 = 车正左
        .right_mm(ld_sl_mm), .right_ok(ld_sl_ok),    // 180°窗口 = 车正右
        .arm(avoid_arm),
        .force_clear(motor_clr | maze_heading_changed),
        .recover(recover_pulse),
        .abort(avoid_abort),
        .stall(avoid_stall),
        .yaw_now(imu_yaw), .yaw_v(imu_yaw_v),
        .remain_in(remain_out),
        .step_arc_cnt(ext_arc_cnt),
        .ext_load(av_ext_load), .ext_act(av_ext_act), .ext_n(av_ext_n),
        .ext_yaw_tgt(av_ext_yaw_tgt), .ext_to_queue(av_ext_to_queue),
        .ext_slow(av_ext_slow),
        .ext_done(ext_done), .ext_active(ext_active),
        .avoid_active(avoid_active), .avoid_fail(avoid_fail), .avoid_done(avoid_done),
        .st_code(avoid_st), .dbg_why(avoid_why), .dbg_side(avoid_side),
        .dbg_lat(avoid_lat), .dbg_phi(avoid_phi),
        .dbg_fwd_acc(avoid_fwd_acc), .dbg_g(avoid_g), .dbg_flags(avoid_flags),
        .dbg_shadow(avoid_shadow)
    );

    // IMU yaw_valid 是单拍脉冲；迷宫转向门控需要电平，这里与 avoid_fsm 同口径
    // 做 50ms 存活超时，禁止拿脉冲直接当使能。
    reg [21:0] maze_imu_to;
    reg        maze_yaw_ok;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            maze_imu_to <= 22'd0;
            maze_yaw_ok <= 1'b0;
        end else if (imu_yaw_v) begin
            maze_imu_to <= 22'd0;
            maze_yaw_ok <= 1'b1;
        end else if (maze_imu_to < 22'd2_500_000) begin
            maze_imu_to <= maze_imu_to + 22'd1;
        end else begin
            maze_yaw_ok <= 1'b0;
        end
    end

    maze_nav_fsm #(
        .CAR_HALF_W_MM(CARW16 >> 1),
        .SIDE_MARGIN_MM(M_SIDE_MM),
        .CORRIDOR_HALF_MM(M_SIDE_MM + (CARW16 >> 1)),
        .STEP_MM(16'd80),
        .CNT_PER_MM(32'd574),
        .N_TURN_90(32'd101600),
        .PASS_K(8'd3),
        .MOVE_TO_CYC(32'd275_000_000),
        .STALE_TO_CYC(32'd25_000_000)
    ) u_maze (
        .clk(clk), .rst_n(rst_n),
        .start(maze_start_req), .abort(maze_abort),
        .scan_done(scan_done), .scan_fresh(ld_scan_fresh),
        .front_mm(dist_f), .yaw_now(imu_yaw), .yaw_ok(maze_yaw_ok),
        .avoid_active(avoid_active), .avoid_done(avoid_done), .avoid_fail(avoid_fail),
        .hist_rd_addr(maze_hist_addr), .hist_rd_data(ld_hist_data),
        .ext_load(maze_ext_load), .ext_act(maze_ext_act), .ext_n(maze_ext_n),
        .ext_yaw_tgt(maze_ext_yaw_tgt), .ext_to_queue(maze_ext_to_queue),
        .ext_slow(maze_ext_slow), .ext_done(ext_done), .ext_active(ext_active),
        .active(maze_active), .done(maze_done_p), .fail(maze_fail),
        .allow_avoid(maze_allow_avoid), .heading_changed(maze_heading_changed),
        .st_code(maze_st), .dbg_why(maze_why), .dbg_inside(maze_inside),
        .dbg_right_mm(maze_right_mm), .dbg_left_mm(maze_left_mm),
        .dbg_right_depth(maze_right_depth),
        .dbg_straight_depth(maze_straight_depth),
        .dbg_left_depth(maze_left_depth),
        .dbg_turn_min(maze_turn_min),
        .dbg_turn_cnt(maze_turn_cnt), .dbg_scan_cnt(maze_scan_cnt)
    );

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(9600)) u_imu_uart (
        .clk(clk), .rst_n(rst_n),
        .rx(imu_rx), .data(imu_byte), .valid(imu_bv)
    );
    imu_parse u_imu (
        .clk(clk), .rst_n(rst_n),
        .rx_data(imu_byte), .rx_valid(imu_bv),
        .yaw_raw(imu_yaw), .yaw_valid(imu_yaw_v), .frame_ok()
    );

    // ---------------- ★v1.1/v1.3 雷达链路：接收 + 命令 + 解析★ ----------------
    // 接收：115200、16× 过采样取中点；顺带统计字节数 / 命中 A5 5A 头的次数
    wire [7:0]  ld_data;
    wire        ld_valid;
    wire [15:0] ld_byte_cnt, ld_hdr_cnt;
    wire [7:0]  ld_type;
    wire        ld_hdr_seen, ld_hdr_done, ld_activity;

    lidar_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(115200)) u_lidar_rx (
        .clk(clk), .rst_n(rst_n), .rx(lidar_rx),
        .data(ld_data), .valid(ld_valid),
        .byte_cnt(ld_byte_cnt), .hdr_cnt(ld_hdr_cnt),
        .last_type(ld_type), .hdr_done(ld_hdr_done),
        .activity(ld_activity), .hdr_seen(ld_hdr_seen)
    );

    // 命令：上电 1s → A5 25（停止）→ 10ms → A5 20（开始扫描），断流 2s 重发
    //   ★断流判据用 activity 翻转位（v1.3 修）：用会饱和的字节计数器会误判成断流，
    //     导致每 2s 重发一次命令、把扫描打断（2026-10-14 上板 T 每 2 秒 +1 即此）★
    wire       lc_tx_start;
    wire [7:0] lc_tx_data;
    wire       lc_tx_busy;

    lidar_cmd #(.CLK_FREQ(CLK_FREQ)) u_lidar_cmd (
        .clk(clk), .rst_n(rst_n),
        .rx_activity(ld_activity),
        .tx_start(lc_tx_start), .tx_data(lc_tx_data), .tx_busy(lc_tx_busy)
    );

    // ★注意：调试打印已经占了一个 uart_tx（M15），雷达要另起一个实例★
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(115200)) u_lidar_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_start(lc_tx_start), .tx_data(lc_tx_data),
        .tx_busy(lc_tx_busy), .tx(lidar_tx)
    );

    // ---------------- ★v1.4 雷达感知层（五扇区 + 控制口径）★ ----------------
    // 原始量给串口显示；控制口径（dist_f/prof_*/watch_*）直接顶替上面那些常量桩
    //   （这些网线在上面 164 行附近就被引用了，所以声明放在那里，这里只放例化）
    lidar_parse #(
        .SENS_FWD_MM (SEN_FWD_MM[15:0]),   // ★按实车：雷达装几何中心 ⇒ 车长/2★
        .SENS_FAR_MM (SENS_FAR_MM),
        .D_PASS_MM   (D_PASS_MM),
        .PASS_K      (WATCH_PASS_K),
        .STALE_CYC   (SENS_STALE_CYC),
        .HALF_F_Q6   (HALF_F_Q6),
        .HALF_S_Q6   (HALF_S_Q6)
    ) u_lidar_parse (
        .clk(clk), .rst_n(rst_n),
        .b_data(ld_data), .b_valid(ld_valid), .hdr_done(ld_hdr_done),
        .watch_en(watch_en_w), .watch_ang(watch_ang_w),
        .front_mm(ld_front_mm), .left_mm(ld_left_mm), .right_mm(ld_right_mm),
        .side_l_mm(ld_sl_mm), .side_r_mm(ld_sr_mm),
        .front_ok(ld_front_ok), .left_ok(ld_left_ok), .right_ok(ld_right_ok),
        .side_l_ok(ld_sl_ok), .side_r_ok(ld_sr_ok),
        .scan_pts(ld_scan_pts),
        .last_ang_deg(ld_last_ang), .last_dist_mm(ld_last_dist),
        .node_cnt(ld_node_cnt), .scan_cnt(ld_scan_cnt), .bad_cnt(ld_bad_cnt),
        .scan_done_p(ld_scan_done_p), .rev_no_pt(ld_rev_no_pt), .scan_fresh(ld_scan_fresh),
        .dist_f_mm(ld_dist_f_mm), .dist_f_v(ld_dist_f_v), .echo_to(ld_echo_to),
        .prof_l_mm(ld_prof_l_mm), .prof_r_mm(ld_prof_r_mm),
        .prof_l_v(ld_prof_l_v), .prof_r_v(ld_prof_r_v),
        .watch_mm(ld_watch_mm), .watch_v(ld_watch_v), .watch_far_cnt(ld_watch_far_cnt),
        // ★2a：障碍角向连续段（前半球最近那一段）＋ 正前左右半区
        .obs_a0_deg(ld_obs_a0), .obs_d0_mm(ld_obs_d0),
        .obs_a1_deg(ld_obs_a1), .obs_d1_mm(ld_obs_d1),
        .obs_dmin_mm(ld_obs_dmin), .obs_amin_deg(ld_obs_amin), .obs_valid(ld_obs_valid),
        .front_l_mm(ld_front_l), .front_r_mm(ld_front_r),
        .front_l_ok(ld_front_l_ok), .front_r_ok(ld_front_r_ok),
        .hist_rd_addr(avoid_active ? av_hist_addr : maze_hist_addr),
        .hist_rd_data(ld_hist_data),
        .hist_side_l_mm(ld_hist_side_l), .hist_side_r_mm(ld_hist_side_r)
    );

    // "本圈点数 / 32"（0..F），塞进行2 的 F 字段：本圈有几百个点时可读出个大概
    wire [3:0] ld_fcnt = ld_scan_pts[8:5];

    // ---------------- 串口调试（115200，每 ~300ms 四行，共 323 字节）----------------
    // 行1: df=HHHH L=x R=x a=X s=X i=x
    //      ★v1.4 口径★ df=雷达正前扇区(90°±20°)最小距离 mm，
    //        ★已折算到车头前缘★（减掉 SEN_FWD_MM=150），所以可以直接和卷尺对照：
    //          · 有障碍：df = 车头前缘到障碍的真实距离
    //          · 0FA0(4000)：本圈正前扇区【一个有效点都没有】或 >12m（= 前方空旷）
    //          · 0000：雷达断流（0.5s 没成圈）⇒ 被当成"贴障碍"急停，同时 LED4 8Hz
    //        （更细的"本圈有几个有效点"看行3 的 F 字段：F=0 就是整圈几乎没点）
    //      L/R=车左前/车右前扇区最小距离是否 ≥ D_SLOW（1=较空；FFFF 也会判 1）
    //        ★v1.9：左右已按实测方位修正（45°=车左前、135°=车右前，见 lidar_parse.v 头部约定）★
    //      a=avoid_fsm 状态 s=（暂无用途，恒 0）i=常量 1（无舵机）
    // 行2: P L=HHHH R=HHHH W=HHHH F=x Y=HHHH d=x n=x w=XX
    //      L=车左前扇区(45°±10°)原始距离 R=车右前(135°±10°)原始距离（FFFF=本圈无点）
    //        ★v1.9 起 L/R 两格与物理左右一致（旧版是镜像的：135° 印在 L、45° 印在 R）★
    //      W=【最后收到的那个点的角度】(hex，度：90°=0x5A；一圈内循环变化，不是距离)
    //        F=本圈有效点数/32
    //      Y=IMU yaw d=绕障方向(0左/1右) n=侧移步数 w=avoid_fsm 原因码
    // 行3: M SL=HHHH SR=HHHH CL=HHHH CR=HHHH V=HHHH T=HHHH f=XX N=HHHH B=HHHH Q=HHHH R=HHHH
    //      SL/SR=两轮 5ms 编码器速度（车体坐标，前进为正）
    //      CL/CR=两轮累计计数低 16 位
    //      V=雷达收到的字节数（自由回绕）T=★通过完整 7 字节校验的应答头次数★
    //        ⇒ v1.4 起 T 正常只在发命令时 +1；它自己慢慢涨 = 还有假头（不该再发生）
    //      ★v1.4 新增★
    //        N=累计完成圈数（用来数真实圈频：约 5~8 圈/秒）
    //        B=丢掉的错位字节数（应当基本不涨；一涨就说明对齐/C 位有问题）
    //        Q=盯视扇区距离（折算后；0FA0=该扇区无点或 >12m）。
    //          watch_ang 复位值是 90（bit7=0）⇒ 不动车时 Q 就是【正右 0°】扇区
    //        R=车正左(180°)扇区原始距离（FFFF=本圈无点）⇒ 和 Q 一起把两侧都核一遍
    //      ★2026-10-15 阶段0 追加（行3 尾，临时调试用，定位"避障里的转")★
    //        A=电机当前段动作（0前进 1左转 2右转 3左弧 4右弧 5绝对航向原地转）
    //        S=电机状态机（0空闲 1跑队列段 2跑ext段 3ext收尾）
    //        t=转向命令 turn_cmd（补码 hex）★一正一负才说明在下"原地转"命令★
    //        x=握手三位 {ext_load, ext_done, ext_active}
    //          ⇒ 判断"FSM 发的绕障段电机到底收到没有、什么时候收尾"
    //      ★2a 追加（行3 尾，障碍角向连续段；方位约定 0°=车正左、90°=正前、180°=车正右）★
    //        E=<a0><d0><a1><d1>  a0/a1=段起/止方位角（两位 hex=度，最大 0xB4=180°）
    //                            d0/d1=段两端点的原始距离（四位 hex=mm）
    //                            坐标换算（2b 用）：y(前)=d·sin(a)、x(右)=-d·cos(a)
    //        D=<dmin>            段内最近距离（四位 hex=mm；FFFF=本圈前半球没有段）
    //        FL=<fl> FR=<fr>     正前偏车左[70°,90°)/偏车右[90°,110°] 最小距离（选边平局时用）
    //      ★f=XX 状态标志字节★
    //        bit7=servo_init_done(恒1)  bit6=avoid_active  bit5=front_blk  bit4=want_avoid
    //        bit3=★scan_fresh（电平：雷达在出数据）★  bit2=mission_hold
    //        bit1=key_estop  bit0=estop_lat
    //        例：f=88 = 无舵机桩(1)+雷达在出数据(1) ⇒ ★final_leida v1.4 的正常值★
    //            f=80 = 雷达 0.5s 没成圈（数据流断了）
    //            f=F0 = 正前被堵+想绕+雷达在出数据（下一拍就该进绕障）
    reg [7:0]  tx_byte;
    reg        tx_start;
    wire       tx_busy;
    // ★v1.7：蓝牙那一路的 busy（打印节拍要同时等两路都发完）★
    wire       bt_tx_busy;
    // V5.0 追加独立迷宫诊断行，缓冲区扩到 324 字节。
    reg [7:0]  line [0:323];
    reg [8:0]  widx, ridx;
    reg [1:0]  print_st;
    reg [23:0] print_div;
    // ★v1.4：行1 的 L/R 用【雷达扇区原始读数】判"这一侧空不空"★
    wire       side_l_ok = (ld_left_mm  >= D_SLOW_MM);
    wire       side_r_ok = (ld_right_mm >= D_SLOW_MM);
    // ★状态标志字节（行3 的 f=XX）★ —— 以前"车不动"到底是被谁按住完全看不出来
    //   key_estop = 电机模块自己的急停锁存（S0 双击 / 红外 STOP）。★必须在这里声明★
    //   （Verilog 的网线要先声明后用，放到文件末尾会被 vlog 判 undefined）
    wire       key_estop = led_m[4];
    // ★v1.4：bit3 从"scan_done 脉冲"改成"scan_fresh 电平"★
    //   scan_done 现在是每圈 1 拍，占空比极小 ⇒ f 会长期显示 80，把
    //   "f≥88 才是 final_leida"这条位流识别判据作废；改成电平后判据继续有效，
    //   而且链路一断 bit3 就会掉（新的现场指示）。
    wire [7:0] dbg_flags = {servo_init_done, avoid_active, front_blk, want_avoid,
                            ld_scan_fresh, mission_hold, key_estop, estop_lat};

    // ★v1.1：V/T 两个字段临时改口径（超声已停用，这两个字段本来就空着）★
    //   V = 雷达收到的字节数（16 位饱和）
    //   T = 命中应答头 A5 5A 的次数（16 位饱和）
    //   ⇒ ★T ≥ 1 就同时证明了：命令发出去了 + 雷达应答了 + 接收采对了★
    //     （雷达收不到 A5 20 是不会吐数据的，所以这一条能当"链路总闸"）
    wire [15:0] echo_ok_cnt = ld_byte_cnt;
    wire [15:0] echo_to_cnt = ld_hdr_cnt;

    function [7:0] hexchar;
        input [3:0] n;
        begin
            hexchar = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h41 + {4'd0, n} - 8'd10);
        end
    endfunction

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(115200)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_start(tx_start), .tx_data(tx_byte),
        .tx_busy(tx_busy), .tx(uart_tx)
    );

    // ★v1.7 蓝牙调试串口：和有线日志【同一个 tx_start/tx_data】，只换波特率★
    //   两路负载很轻（一个 uart_tx ≈ 50 个寄存器），互不影响；
    //   打印状态机同时等 tx_busy 和 bt_tx_busy 都落下，所以即使蓝牙那路是 9600
    //   （比 115200 慢 12 倍）也不会丢字节，只是整体刷新变慢喵。
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BT_BAUD)) u_bt_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_start(tx_start), .tx_data(tx_byte),
        .tx_busy(bt_tx_busy), .tx(bt_tx)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            print_st <= 2'd0; widx <= 9'd0; ridx <= 9'd0;
            tx_byte <= 8'd0; tx_start <= 1'b0;
            print_div <= 24'd0;
        end else begin
            tx_start <= 1'b0;
            case (print_st)
                2'd0: begin
                    if (print_div >= 24'd6_000_000) begin
                        print_div <= 24'd0;
                        // ---- 行1 ----
                        // ★v1.4：df 打印【控制口径】的正前距离（已折算到车头前缘），
                        //   所以可以直接和卷尺对照；链路断了它会被强制成 0000★
                        line[0] <= "d"; line[1] <= "f"; line[2] <= "=";
                        line[3] <= hexchar(ld_dist_f_mm[15:12]);
                        line[4] <= hexchar(ld_dist_f_mm[11:8]);
                        line[5] <= hexchar(ld_dist_f_mm[7:4]);
                        line[6] <= hexchar(ld_dist_f_mm[3:0]);
                        line[7] <= " "; line[8] <= "L"; line[9] <= "=";
                        line[10]<= side_l_ok ? "1" : "0";
                        line[11]<= " "; line[12]<= "R"; line[13]<= "=";
                        line[14]<= side_r_ok ? "1" : "0";
                        line[15]<= " "; line[16]<= "a"; line[17]<= "=";
                        line[18]<= hexchar(avoid_st);
                        line[19]<= " "; line[20]<= "s"; line[21]<= "=";
                        line[22]<= hexchar({1'b0, scan_st});
                        line[23]<= " "; line[24]<= "i"; line[25]<= "=";
                        line[26]<= servo_init_done ? "1" : "0";
                        line[27]<= 8'h0D; line[28]<= 8'h0A;
                        // ---- 行2 ----
                        line[29]<= "P"; line[30]<= " ";
                        line[31]<= "L"; line[32]<= "=";
                        line[33]<= hexchar(ld_left_mm[15:12]);
                        line[34]<= hexchar(ld_left_mm[11:8]);
                        line[35]<= hexchar(ld_left_mm[7:4]);
                        line[36]<= hexchar(ld_left_mm[3:0]);
                        line[37]<= " "; line[38]<= "R"; line[39]<= "=";
                        line[40]<= hexchar(ld_right_mm[15:12]);
                        line[41]<= hexchar(ld_right_mm[11:8]);
                        line[42]<= hexchar(ld_right_mm[7:4]);
                        line[43]<= hexchar(ld_right_mm[3:0]);
                        line[44]<= " "; line[45]<= "W"; line[46]<= "=";
                        line[47]<= hexchar(ld_last_ang[15:12]);
                        line[48]<= hexchar(ld_last_ang[11:8]);
                        line[49]<= hexchar(ld_last_ang[7:4]);
                        line[50]<= hexchar(ld_last_ang[3:0]);
                        line[51]<= " "; line[52]<= "F"; line[53]<= "=";
                        line[54]<= hexchar(ld_fcnt);
                        line[55]<= " "; line[56]<= "Y"; line[57]<= "=";
                        line[58]<= hexchar(imu_yaw[15:12]);
                        line[59]<= hexchar(imu_yaw[11:8]);
                        line[60]<= hexchar(imu_yaw[7:4]);
                        line[61]<= hexchar(imu_yaw[3:0]);
                        line[62]<= " "; line[63]<= "d"; line[64]<= "=";
                        line[65]<= avoid_side ? "1" : "0";
                        line[66]<= " "; line[67]<= "n"; line[68]<= "=";
                        line[69]<= hexchar(avoid_lat[3:0]);
                        line[70]<= " "; line[71]<= "w"; line[72]<= "=";
                        line[73]<= hexchar(avoid_why[7:4]);
                        line[74]<= hexchar(avoid_why[3:0]);
                        line[75]<= 8'h0D; line[76]<= 8'h0A;
                        // ---- 行3（53字节）----
                        line[77]<= "M"; line[78]<= " ";
                        line[79]<= "S"; line[80]<= "L"; line[81]<= "=";
                        line[82]<= hexchar(spd_l[15:12]);
                        line[83]<= hexchar(spd_l[11:8]);
                        line[84]<= hexchar(spd_l[7:4]);
                        line[85]<= hexchar(spd_l[3:0]);
                        line[86]<= " "; line[87]<= "S"; line[88]<= "R"; line[89]<= "=";
                        line[90]<= hexchar(spd_r[15:12]);
                        line[91]<= hexchar(spd_r[11:8]);
                        line[92]<= hexchar(spd_r[7:4]);
                        line[93]<= hexchar(spd_r[3:0]);
                        line[94]<= " "; line[95]<= "C"; line[96]<= "L"; line[97]<= "=";
                        line[98]<= hexchar(cnt_l[15:12]);
                        line[99]<= hexchar(cnt_l[11:8]);
                        line[100]<=hexchar(cnt_l[7:4]);
                        line[101]<=hexchar(cnt_l[3:0]);
                        line[102]<= " "; line[103]<= "C"; line[104]<= "R"; line[105]<= "=";
                        line[106]<=hexchar(cnt_r[15:12]);
                        line[107]<=hexchar(cnt_r[11:8]);
                        line[108]<=hexchar(cnt_r[7:4]);
                        line[109]<=hexchar(cnt_r[3:0]);
                        line[110]<= " "; line[111]<= "V"; line[112]<= "=";
                        line[113]<=hexchar(echo_ok_cnt[15:12]);
                        line[114]<=hexchar(echo_ok_cnt[11:8]);
                        line[115]<=hexchar(echo_ok_cnt[7:4]);
                        line[116]<=hexchar(echo_ok_cnt[3:0]);
                        line[117]<= " "; line[118]<= "T"; line[119]<= "=";
                        line[120]<=hexchar(echo_to_cnt[15:12]);
                        line[121]<=hexchar(echo_to_cnt[11:8]);
                        line[122]<=hexchar(echo_to_cnt[7:4]);
                        line[123]<=hexchar(echo_to_cnt[3:0]);
                        line[124]<= " "; line[125]<= "f"; line[126]<= "=";
                        line[127]<= hexchar(dbg_flags[7:4]);
                        line[128]<= hexchar(dbg_flags[3:0]);
                        // ---- ★v1.4 追加：N=累计圈数 B=错位字节数 Q=盯视扇区距离★ ----
                        line[129]<= " "; line[130]<= "N"; line[131]<= "=";
                        line[132]<= hexchar(ld_scan_cnt[15:12]);
                        line[133]<= hexchar(ld_scan_cnt[11:8]);
                        line[134]<= hexchar(ld_scan_cnt[7:4]);
                        line[135]<= hexchar(ld_scan_cnt[3:0]);
                        line[136]<= " "; line[137]<= "B"; line[138]<= "=";
                        line[139]<= hexchar(ld_bad_cnt[15:12]);
                        line[140]<= hexchar(ld_bad_cnt[11:8]);
                        line[141]<= hexchar(ld_bad_cnt[7:4]);
                        line[142]<= hexchar(ld_bad_cnt[3:0]);
                        line[143]<= " "; line[144]<= "Q"; line[145]<= "=";
                        line[146]<= hexchar(ld_watch_mm[15:12]);
                        line[147]<= hexchar(ld_watch_mm[11:8]);
                        line[148]<= hexchar(ld_watch_mm[7:4]);
                        line[149]<= hexchar(ld_watch_mm[3:0]);
                        // R = 车正左扇区(180°)原始距离（FFFF=本圈无点）；配合 Q（默认盯正右）
                        //   就能在不动车的情况下把两侧 0°/180° 扇区都核一遍
                        line[150]<= " "; line[151]<= "R"; line[152]<= "=";
                        line[153]<= hexchar(ld_sl_mm[15:12]);
                        line[154]<= hexchar(ld_sl_mm[11:8]);
                        line[155]<= hexchar(ld_sl_mm[7:4]);
                        line[156]<= hexchar(ld_sl_mm[3:0]);
                        // ---- ★2026-10-15 阶段0 临时调试（只加字段，不改逻辑）★ ----
                        //   A=当前段动作 S=电机状态机 t=转向命令 x=绕障握手三位
                        line[157]<= " "; line[158]<= "A"; line[159]<= "=";
                        line[160]<= hexchar({1'b0, m_dbg_act});
                        line[161]<= " "; line[162]<= "S"; line[163]<= "=";
                        line[164]<= hexchar({2'b0, m_dbg_st});
                        line[165]<= " "; line[166]<= "t"; line[167]<= "=";
                        line[168]<= hexchar(m_dbg_turn[15:12]);
                        line[169]<= hexchar(m_dbg_turn[11:8]);
                        line[170]<= hexchar(m_dbg_turn[7:4]);
                        line[171]<= hexchar(m_dbg_turn[3:0]);
                        line[172]<= " "; line[173]<= "x"; line[174]<= "=";
                        line[175]<= hexchar({1'b0, ext_load, ext_done, ext_active});
                        // ---- ★2a：障碍角向连续段（前半球最近的那一段）＋正前左右半区★ ----
                        //   E=<a0><d0><a1><d1>  a=方位角(两位hex=度) d=原始距离(四位hex=mm)
                        //   D=<dmin>  段内最近距离           FL/FR=正前左右半区最小值
                        line[176]<= " "; line[177]<= "E"; line[178]<= "=";
                        line[179]<= hexchar(ld_obs_a0[7:4]);
                        line[180]<= hexchar(ld_obs_a0[3:0]);
                        line[181]<= hexchar(ld_obs_d0[15:12]);
                        line[182]<= hexchar(ld_obs_d0[11:8]);
                        line[183]<= hexchar(ld_obs_d0[7:4]);
                        line[184]<= hexchar(ld_obs_d0[3:0]);
                        line[185]<= hexchar(ld_obs_a1[7:4]);
                        line[186]<= hexchar(ld_obs_a1[3:0]);
                        line[187]<= hexchar(ld_obs_d1[15:12]);
                        line[188]<= hexchar(ld_obs_d1[11:8]);
                        line[189]<= hexchar(ld_obs_d1[7:4]);
                        line[190]<= hexchar(ld_obs_d1[3:0]);
                        line[191]<= " "; line[192]<= "D"; line[193]<= "=";
                        line[194]<= hexchar(ld_obs_dmin[15:12]);
                        line[195]<= hexchar(ld_obs_dmin[11:8]);
                        line[196]<= hexchar(ld_obs_dmin[7:4]);
                        line[197]<= hexchar(ld_obs_dmin[3:0]);
                        line[198]<= " "; line[199]<= "F"; line[200]<= "L"; line[201]<= "=";
                        line[202]<= hexchar(ld_front_l[15:12]);
                        line[203]<= hexchar(ld_front_l[11:8]);
                        line[204]<= hexchar(ld_front_l[7:4]);
                        line[205]<= hexchar(ld_front_l[3:0]);
                        line[206]<= " "; line[207]<= "F"; line[208]<= "R"; line[209]<= "=";
                        line[210]<= hexchar(ld_front_r[15:12]);
                        line[211]<= hexchar(ld_front_r[11:8]);
                        line[212]<= hexchar(ld_front_r[7:4]);
                        line[213]<= hexchar(ld_front_r[3:0]);
                        // ---- ★V2.2：本次"算出的角度"和"已累计横移"★ ----
                        //   P=<φ 两位hex=度><lat 四位hex=mm>
                        line[214]<= " "; line[215]<= "P"; line[216]<= "=";
                        line[217]<= hexchar(avoid_phi[7:4]);
                        line[218]<= hexchar(avoid_phi[3:0]);
                        line[219]<= hexchar(avoid_lat[15:12]);
                        line[220]<= hexchar(avoid_lat[11:8]);
                        line[221]<= hexchar(avoid_lat[7:4]);
                        line[222]<= hexchar(avoid_lat[3:0]);
                        // ★V4.2b-dbg★ F=<corr><diff>：corr=A_FWD_HOLD 航向修正命令低字节，
                        //   diff=clamp(左轮速-右轮速)。用来判断"命令有没有落到轮子上"。
                        //   本版临时占用原 fwd_acc 的位置，诊断完就换回。
                        line[223]<= " "; line[224]<= "F"; line[225]<= "=";
                        line[226]<= hexchar(corr8[7:4]);
                        line[227]<= hexchar(corr8[3:0]);
                        line[228]<= hexchar(spd_diff8[7:4]);
                        line[229]<= hexchar(spd_diff8[3:0]);
                        line[230]<= " "; line[231]<= "G"; line[232]<= "=";
                        line[233]<= hexchar(avoid_shadow[15:12]);
                        line[234]<= hexchar(avoid_shadow[11:8]);
                        line[235]<= hexchar(avoid_shadow[7:4]);
                        line[236]<= hexchar(avoid_shadow[3:0]);
                        line[237]<= " "; line[238]<= "V"; line[239]<= "=";
                        line[240]<= "5"; line[241]<= "."; line[242]<= "0";
                        line[243]<= "e"; line[244]<= " ";
                        // ★V4.0d★ 紧凑决策标志：bit7 gap_ok / bit6 side /
                        //   bit5 side_win=FFFF / bit4 side_win>=300 / bit3 front_ok /
                        //   bit2 hist_l=FFFF / bit1 hist_r=FFFF / bit0 crush
                        line[245]<= " "; line[246]<= "S"; line[247]<= "=";
                        line[248]<= hexchar(avoid_flags[7:4]);
                        line[249]<= hexchar(avoid_flags[3:0]);
                        line[250]<= 8'h0D; line[251]<= 8'h0A;
                        // ---- V5.0 迷宫诊断行：状态/原因/入迷宫/可让避障/走廊深度 ----
                        line[252]<= "Z"; line[253]<= " ";
                        line[254]<= "s"; line[255]<= "t"; line[256]<= "=";
                        line[257]<= hexchar(maze_st);
                        line[258]<= " "; line[259]<= "w"; line[260]<= "=";
                        line[261]<= hexchar(maze_why[7:4]);
                        line[262]<= hexchar(maze_why[3:0]);
                        line[263]<= " "; line[264]<= "i"; line[265]<= "=";
                        line[266]<= maze_inside ? "1" : "0";
                        line[267]<= " "; line[268]<= "a"; line[269]<= "=";
                        line[270]<= maze_allow_avoid ? "1" : "0";
                        line[271]<= " "; line[272]<= "R"; line[273]<= "=";
                        line[274]<= hexchar(maze_right_mm[15:12]);
                        line[275]<= hexchar(maze_right_mm[11:8]);
                        line[276]<= hexchar(maze_right_mm[7:4]);
                        line[277]<= hexchar(maze_right_mm[3:0]);
                        line[278]<= " "; line[279]<= "L"; line[280]<= "=";
                        line[281]<= hexchar(maze_left_mm[15:12]);
                        line[282]<= hexchar(maze_left_mm[11:8]);
                        line[283]<= hexchar(maze_left_mm[7:4]);
                        line[284]<= hexchar(maze_left_mm[3:0]);
                        line[285]<= " "; line[286]<= "D"; line[287]<= "R"; line[288]<= "=";
                        line[289]<= hexchar(maze_right_depth[15:12]);
                        line[290]<= hexchar(maze_right_depth[11:8]);
                        line[291]<= hexchar(maze_right_depth[7:4]);
                        line[292]<= hexchar(maze_right_depth[3:0]);
                        line[293]<= " "; line[294]<= "D"; line[295]<= "S"; line[296]<= "=";
                        line[297]<= hexchar(maze_straight_depth[15:12]);
                        line[298]<= hexchar(maze_straight_depth[11:8]);
                        line[299]<= hexchar(maze_straight_depth[7:4]);
                        line[300]<= hexchar(maze_straight_depth[3:0]);
                        line[301]<= " "; line[302]<= "D"; line[303]<= "L"; line[304]<= "=";
                        line[305]<= hexchar(maze_left_depth[15:12]);
                        line[306]<= hexchar(maze_left_depth[11:8]);
                        line[307]<= hexchar(maze_left_depth[7:4]);
                        line[308]<= hexchar(maze_left_depth[3:0]);
                        line[309]<= " "; line[310]<= "T"; line[311]<= "=";
                        line[312]<= hexchar(maze_turn_min[15:12]);
                        line[313]<= hexchar(maze_turn_min[11:8]);
                        line[314]<= hexchar(maze_turn_min[7:4]);
                        line[315]<= hexchar(maze_turn_min[3:0]);
                        line[316]<= " "; line[317]<= "N"; line[318]<= "=";
                        line[319]<= hexchar(maze_turn_cnt[7:4]);
                        line[320]<= hexchar(maze_turn_cnt[3:0]);
                        line[321]<= 8'h0D; line[322]<= 8'h0A;
                        widx <= 9'd323; ridx <= 9'd0;
                        print_st <= 2'd1;
                    end else print_div <= print_div + 24'd1;
                end
                2'd1: begin
                    // ★握手（2026-10-08 修）★
                    //   uart_tx 的 tx_busy 要等"被 tx_start 唤醒的下一拍"才拉高，
                    //   旧写法只看 !tx_busy ⇒ 同一字节被推两次 ridx，而 uart_tx 锁存的是
                    //   前一次的 tx_byte ⇒ 【隔一个字节丢一个】（77 字节只发出 39 个）。
                    //   必须同时看 tx_start（与 uart_line_print.v 的写法一致）。
                    // ★v1.7：蓝牙那一路 busy 也要等★ —— 蓝牙若是 9600（比 115200 慢 12 倍），
                    //   只等有线那路会让蓝牙丢字节；等两路都空再推下一个字节就稳了喵。
                    if (!tx_busy && !bt_tx_busy && !tx_start && (ridx < widx)) begin
                        tx_byte  <= line[ridx];
                        tx_start <= 1'b1;
                        ridx     <= ridx + 9'd1;
                    end else if (!tx_busy && !bt_tx_busy && (ridx >= widx)) begin
                        print_st <= 2'd0;
                    end
                end
                default: print_st <= 2'd0;
            endcase
        end
    end

    // LED：与 final 一致——直接透传 motor_driver 的按键/队列/急停灯
    //   led[0..2] 入队深度确认灯  led[3] 执行/手柄  led[4] 急停/绕障
    //   led[5]/[6] 左右轮  led[7] 心跳
    wire gp_any_w  = gp_fwd_w | gp_back_w | gp_left_w | gp_right_w;
    wire queue_run = led_m[3];
    reg  led3_hb = 1'b0;
    reg [22:0] led3_div = 23'd0;
    always @(posedge clk) begin
        if (led3_div >= 23'd5_000_000) begin
            led3_div <= 23'd0;
            led3_hb  <= ~led3_hb;
        end else
            led3_div <= led3_div + 23'd1;
    end
    // ---- ★LED4 分码（v3.0）★ ----
    //   常亮     = 绕障失败锁存（avoid_fail）
    //   4Hz 闪   = 电机模块自己的急停（key_estop：S0 双击 / 红外 STOP 码或噪声）
    //   2Hz 闪   = 手柄接管锁存（mission_hold：动过手柄/红外点动线毛刺）
    //   8Hz 快闪 = 顶层急停：正前被堵停住（等剖面 / 等绕障启动）★卡死回归指示★
    //   1Hz 闪   = 绕障进行中 或 减速接近
    //   灭       = 正常
    //   （旧版这三种"车完全不动"都是常亮，现场根本分不出来）
    reg [23:0] l4_d1 = 24'd0, l4_d2 = 24'd0;   // 12_500_000 需要 24 位，别写成 23 位
    reg [21:0] l4_d4 = 22'd0, l4_d8 = 22'd0;
    reg        l4_b1 = 1'b0, l4_b2 = 1'b0, l4_b4 = 1'b0, l4_b8 = 1'b0;
    always @(posedge clk) begin
        if (l4_d1 >= 24'd12_500_000) begin l4_d1 <= 24'd0; l4_b1 <= ~l4_b1; end
        else                               l4_d1 <= l4_d1 + 24'd1;
        if (l4_d2 >= 24'd6_250_000)  begin l4_d2 <= 24'd0; l4_b2 <= ~l4_b2; end
        else                               l4_d2 <= l4_d2 + 24'd1;
        if (l4_d4 >= 22'd3_125_000)  begin l4_d4 <= 22'd0; l4_b4 <= ~l4_b4; end
        else                               l4_d4 <= l4_d4 + 22'd1;
        if (l4_d8 >= 22'd1_562_500)  begin l4_d8 <= 22'd0; l4_b8 <= ~l4_b8; end
        else                               l4_d8 <= l4_d8 + 22'd1;
    end

    // ================= ★v1.6 板载调试指示：雷达健康灯 + 蜂鸣器状态播报★ =================
    //   动机：现场跑车时不想一直拖着电脑看串口助手，所以把"雷达死活"和"避障事件"
    //   搬到板载 LED + 蜂鸣器上；不改任何避障逻辑，只读现成信号。
    //
    //   led[1]  （B15，扩展口）：雷达健康指示（原来恒 0）；想看得见就在扩展口接 LED
    //   beep  （N19，无源蜂鸣器，经三极管）：★必须给 2~5kHz 方波才响★
    //       ★2026-10-15 静音化★：全部改成"事件短声"，取消长鸣与复读——
    //         上电就绪 2 短声 / 进绕障 1 短声 / 绕障结束 1 短声 /
    //         雷达断流 2 短声（只响一次）/ 雷达恢复 1 短声 / 绕障失败 2 短声（只响一次）
    //       绕障失败仍然用 LED4 常亮指示；雷达死活仍然用 led[1] 指示（B15 扩展口）。
    assign led[1]  = ld_scan_fresh;          // ★v1.6：从"恒 0"改成雷达健康指示★

    // ---- 4kHz 载波（无源蜂鸣器要方波，直流不响）----
    localparam integer BEEP_DIV = CLK_FREQ / (2 * BEEP_HZ);   // 50M/(2*4000) = 6250
    localparam integer MS_DIV   = CLK_FREQ / 1000;            // 1ms = 50000 拍

    reg [15:0] bp_div;
    reg        bp_car;                                        // 载波电平
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin bp_div <= 16'd0; bp_car <= 1'b0; end
        else if (bp_div >= BEEP_DIV[15:0] - 16'd1) begin
            bp_div <= 16'd0; bp_car <= ~bp_car;
        end else bp_div <= bp_div + 16'd1;
    end

    // ---- 1ms 时基 ----
    reg [15:0] ms_cnt;
    wire       ms_tick = (ms_cnt >= MS_DIV[15:0] - 16'd1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ms_cnt <= 16'd0;
        else if (ms_tick) ms_cnt <= 16'd0;
        else ms_cnt <= ms_cnt + 16'd1;
    end

    // ---- 事件检测：雷达断流/恢复、进绕障/绕障结束/失败 ----
    reg sf_d, av_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sf_d <= 1'b0; av_d <= 1'b0; end
        else begin sf_d <= ld_scan_fresh; av_d <= avoid_active; end
    end
    wire link_lost  = sf_d & ~ld_scan_fresh;      // 1→0
    wire link_recov = ~sf_d & ld_scan_fresh;      // 0→1
    wire avoid_start= avoid_active & ~av_d;       // 0→1
    wire avoid_end  = avoid_done;                 // 单拍脉冲

    // ---- 断流复读：断流超过 BEEP_DEAD_DLY_MS 后，每 BEEP_REP_MS 响 BEEP_LONG_MS ----
    //   为什么要有 dead_dly：上电后雷达要 1~1.2s 才开始吐数据，一上来就长鸣会把
    //   "上电就绪"的两短声盖掉，听着很糊涂。
    reg [15:0] rep_ms;
    reg [15:0] dead_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rep_ms <= 16'd0; dead_ms <= 16'd0; end
        else if (!ld_scan_fresh) begin
            if (ms_tick) begin
                if (dead_ms < 16'd65535) dead_ms <= dead_ms + 16'd1;
                rep_ms <= (rep_ms >= BEEP_REP_MS[15:0]) ? 16'd0 : rep_ms + 16'd1;
            end
        end else begin
            rep_ms <= 16'd0; dead_ms <= 16'd0;
        end
    end
    // ★2026-10-15：取消"长鸣不止"与"每 BEEP_REP_MS 复读"（用户要求别吵）★
    //   断流改成"刚判出断流那一刻响 2 短声，只响一次"；恢复仍是 1 短声；
    //   绕障失败从"常鸣"改成"2 短声只响一次"（失败仍由 LED4 常亮指示）。
    //   rep_ms / BEEP_LONG_MS / BEEP_REP_MS 保留作历史参数，不再产生响声。
    reg  dead_lat;                       // 断流已经响过了
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                 dead_lat <= 1'b0;
        else if (ld_scan_fresh)                     dead_lat <= 1'b0;
        else if (dead_ms >= BEEP_DEAD_DLY_MS[15:0]) dead_lat <= 1'b1;
    end
    wire dead_evt = (~dead_lat) && (dead_ms >= BEEP_DEAD_DLY_MS[15:0]);
    wire rep_tone = 1'b0;                // 已停用（保留网线名，避免漏改引用）

    // ---- 事件音播放器：响 ev_n 声短音，每声 BEEP_SHORT_MS，间隔 BEEP_GAP_MS ----
    localparam integer EV_UNIT = BEEP_SHORT_MS + BEEP_GAP_MS;
    reg [3:0]  ev_n, ev_k;
    reg [15:0] ev_sub;
    wire       ev_busy = (ev_k < ev_n) || (ev_sub != 16'd0);
    wire       ev_on   = (ev_k < ev_n) && (ev_sub < BEEP_SHORT_MS[15:0]);

    // 上电就绪：rst_n 释放后 100ms 响 2 短声（用独立计数器，别和事件播放器抢资源）
    reg [15:0] por_ms;
    reg        ready_done;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin por_ms <= 16'd0; ready_done <= 1'b0; end
        else if (!ready_done) begin
            if (ms_tick && (por_ms >= 16'd100)) ready_done <= 1'b1;
            else if (ms_tick) por_ms <= por_ms + 16'd1;
        end
    end

    // ★事件必须锁存★：avoid_done 只有 1 拍宽（20ns），而播放器每 1ms 才看一次，
    //   不锁存的话 99.998% 的脉冲会被漏掉。
    reg ev_recov_req, ev_start_req, ev_end_req;
    reg ev_dead_req,  ev_fail_req;   // ★2026-10-15：断流/失败"只响一次"的请求锁存
    reg fail_lat, maze_fail_lat;     // 失败电平的上一拍（用于取上升沿）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ev_n <= 4'd0; ev_k <= 4'd0; ev_sub <= 16'd0;
            ev_recov_req <= 1'b0; ev_start_req <= 1'b0; ev_end_req <= 1'b0;
            ev_dead_req <= 1'b0; ev_fail_req <= 1'b0;
            fail_lat <= 1'b0; maze_fail_lat <= 1'b0;
        end else begin
            if (link_recov)  ev_recov_req <= 1'b1;
            if (avoid_start) ev_start_req <= 1'b1;
            if (avoid_done)  ev_end_req   <= 1'b1;
            if (dead_evt)               ev_dead_req <= 1'b1;   // 刚断流
            if ((avoid_fail & ~fail_lat) ||
                (maze_fail & ~maze_fail_lat)) ev_fail_req <= 1'b1;
            fail_lat <= avoid_fail;
            maze_fail_lat <= maze_fail;

            if (ms_tick) begin
                if (ev_fail_req) begin                    // 绕障失败：2 短声（只响一次）
                    ev_n <= 4'd2; ev_k <= 4'd0; ev_sub <= 16'd0; ev_fail_req <= 1'b0;
                end else if (ev_dead_req) begin            // 雷达断流：2 短声（只响一次）
                    ev_n <= 4'd2; ev_k <= 4'd0; ev_sub <= 16'd0; ev_dead_req <= 1'b0;
                end else if (ev_end_req) begin             // 绕障结束：1 短声
                    ev_n <= 4'd1; ev_k <= 4'd0; ev_sub <= 16'd0; ev_end_req <= 1'b0;
                end else if (ev_start_req) begin          // 进绕障：1 短声（原 3 短声，静音化）
                    ev_n <= 4'd1; ev_k <= 4'd0; ev_sub <= 16'd0; ev_start_req <= 1'b0;
                end else if (ev_recov_req) begin          // 雷达恢复：1 短声
                    ev_n <= 4'd1; ev_k <= 4'd0; ev_sub <= 16'd0; ev_recov_req <= 1'b0;
                end else if (!ready_done && (por_ms >= 16'd100)) begin   // 上电就绪：2 短声
                    ev_n <= 4'd2; ev_k <= 4'd0; ev_sub <= 16'd0;
                end else if (ev_busy) begin               // 没有新事件就推进当前序列
                    if (ev_sub >= EV_UNIT[15:0] - 16'd1) begin
                        ev_sub <= 16'd0;
                        if (ev_k < ev_n) ev_k <= ev_k + 4'd1;
                    end else ev_sub <= ev_sub + 16'd1;
                end
            end
        end
    end

    // ---- 门控 + 载波 + 极性 ----
    // ★2026-10-15：所有响声都走事件播放器（短声），不再有"常鸣/复读"★
    // 迷宫成功：保持 LED 完成指示，并在停车后响一次长声。
    reg [15:0] maze_long_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || maze_start_req || motor_clr || ir_stop_w)
            maze_long_ms <= 16'd0;
        else if (maze_done_lat && ms_tick && (maze_long_ms < BEEP_LONG_MS[15:0]))
            maze_long_ms <= maze_long_ms + 16'd1;
    end
    wire maze_long_beep = maze_done_lat && (maze_long_ms < BEEP_LONG_MS[15:0]);

    wire beep_gate = rep_tone | ev_on | maze_long_beep;
    assign beep = BEEP_INVERT ? ~(beep_gate & bp_car) : (beep_gate & bp_car);

    assign led[0] = led_m[0];
    assign led[2] = led_m[2];
    assign led[3] = queue_run ? led3_hb : (gp_any_w & ~key_estop & ~obstacle_stop);
    assign led[4] = maze_done_lat ? l4_b2 :        // 迷宫完成：2Hz
                    maze_fail    ? l4_b4 :          // 迷宫失败：4Hz
                    avoid_fail   ? 1'b1  :          // 绕障失败锁存
                    key_estop    ? l4_b4 :          // 电机模块急停（S0 双击/红外 STOP）
                    mission_hold ? l4_b2 :          // 手柄接管锁存
                    estop_lat    ? l4_b8 :          // 正前被堵停住（等剖面/等绕障启动）
                    (avoid_active | slow_lat) ? l4_b1 : 1'b0;
    assign led[5] = led_m[5];
    assign led[6] = led_m[6];
    assign led[7] = led_m[7];

endmodule
