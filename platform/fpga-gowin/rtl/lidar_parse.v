//=============================================================================
// lidar_parse.v —— RPLIDAR A1M8 标准扫描解析 + 五扇区归约 + 控制口径【final_leida v1.4】
//-----------------------------------------------------------------------------
// ★本模块在 v1.4 起同时承担两件事★（原计划拆成 lidar_sector.v，为避免动
//   final_leida.gprj 的文件列表——Gowin IDE 会缓存文件列表、改完不重开工程会报
//   EX3937 unknown module——所以合并在本文件里）：
//   ① 解析 5 字节数据应答报文，把点归到 5 个扇区，每圈锁存一次最小距离；
//   ② 把扇区结果折算成【与超声 + 舵机版同名同义】的控制量，直接喂顶层/avoid_fsm。
//
// 节点格式（官方 SDK sl_lidar_cmd.h 与协议规范 LR001 v2.4 图表 4-4/4-5）：
//   b0 : bit0 = S（1 = 新一圈 360° 扫描的第一个点）
//        bit1 = ~S（必须与 bit0 反相，否则字节错位 ⇒ 丢这一字节重找）★官方同判据★
//        bit7:2 = 信号质量
//   b1 : bit7:1 = angle_q6[6:0]；★bit0 = C 校验位，恒为 1★
//        （官方 handler_normalnode.cpp 在 pos==1 时就是查这一位）
//   b2 : bit7:0 = angle_q6[14:7]
//        ⇒ ang_q6 = ({b2,b1} >> 1)（丢掉 C 位）；1/64°：45°=2880、90°=5760、
//          135°=8640、180°=11520、360°=23040
//   b3/b4 : distance_q2（小端 16 位），实际距离 = distance_q2/4 mm；
//        ★按规范，采集到无效点时该字段为 0★（不是 0xFFFF）
//
// ★★方向约定（全工程统一；2026-10-14 用户定义 + 实测定案）★★
//   基准 = 以【车前进方向为"前"】的驾驶员视角：
//     前 = 车往前开的方向；左 = 驾驶员左手边；右 = 驾驶员右手边。
//   提问、注释、串口字段判读一律用这个基准，不用"站在车头看"的视角。
//   ★实测（2026-10-14：车停着不动，箱子分别放车的左前 / 右前 45°，两组数据互为镜像）★：
//     雷达角 45° 在【车左前】、135° 在【车右前】、0° 在【车正左】、180° 在【车正右】
//     ⇒ 角度增大 = 往车的【左】边。★旧注释"角度增大 = 往车的右边"是错的★。
//   ⇒ v1.9 起把 ANG_L_Q6/ANG_R_Q6 的【数值】按实测对调，名字与实际方位一致；
//     v1.8 及以前是镜像的 ⇒ prof_l/prof_r 左右颠倒 ⇒ avoid_fsm 的 side_pick
//     反着选边（哪边近就往哪边转 = 朝障碍那侧转）。
//   ★注意★：0°/180° 那一对【变量名】同样是历史遗留的反的（sr_acc=0°=车正左、
//     sl_acc=180°=车正右），但 watch 路径的【用法是对的】（side=0 向左绕 ⇒ wang=0
//     ⇒ 选的正是车正左；side=1 向右绕 ⇒ wang=180 ⇒ 车正右）⇒ 别再去"修"它。
// 五个扇区（名字已按实测方位）：
//   正前 90°±20°、车左前 45°±10°、车右前 135°±10°、
//   0°±10° 窗口（实测车正左）、180°±10° 窗口（实测车正右）
//   正前半窗放大到 ±20° 的原因：±10° 在 450mm 停车距离处只有 16cm 宽，比车宽 170mm 还窄。
//   0° 扇区跨 360/0 边界 ⇒ 两段判。
//
// ★控制口径（v1.4 关键设计）★
//   · 扇区本圈无有效点（FFFF）⇒ 按"那个方向 12m 内没有东西"理解 ⇒ 折算成 SENS_FAR_MM
//     并算作有效。这与超声版"超时 = 4000mm = 前方空旷"口径一致；只在链路还活着时成立。
//   · 链路死了（STALE_CYC 内一圈都没完成）⇒ fresh=0 ⇒ 正前【强制 0mm】、dist_f_v=0，
//     并拉高 echo_to ⇒ 顶层 to_run 计数到 10 ⇒ sensor_fault ⇒ 停车 + LED4 8Hz。
//     ★与超声版"探头失灵被当成被堵"完全同一条路径、同一个指示★
//   · 正前距离【折算到车头前缘】：雷达装在车头之后 150mm（实测），读数比"车头前缘
//     到障碍"多 SENS_FWD_MM(=150mm)。不减掉的话 D_STOP=450 会晚 150mm 才动作。
//     侧向/盯视【不减】：0°/180° 的读数本来就算"车体纵向中心线到障碍的垂距"，
//     与超声版同一物理量（原注释"读数 − 车宽/2 = 车体到障碍的实际间隙"仍成立）。
//
// ★v1.4 修的假头 bug★：hdr_done 现在只在 lidar_rx 收到【完整 7 字节头】时才来，
//   所以真头清累加器、假头不清（旧版只要看到 A5 5A 就清 ⇒ 每约 5 秒出现一次"整圈 FFFF"）。
//
// 全整数、无除法（角度窗比较在 Q6 域，度数→Q6 用移位）。
//=============================================================================
module lidar_parse #(
    // ---- 扇区角度（Q6 = 1/64°）----
    parameter [15:0] ANG_F_Q6   = 16'd5760,    // 正前 90°
    // ★v1.9：ANG_L/ANG_R 的数值按 2026-10-14 实测对调（45° 才是车的左边）★
    //   旧的 135°=左前 / 45°=右前 是镜像的，会让 side_pick 朝障碍那侧选边
    parameter [15:0] ANG_L_Q6   = 16'd2880,    // 车左前 45°（实测）
    parameter [15:0] ANG_R_Q6   = 16'd8640,    // 车右前 135°（实测）
    // ★下面这一对是【变量名】与实测方位反的（0° 实测在车正左、180° 在车正右），
    //   但 watch 路径的用法是对的（见文件头"方向约定"末段）⇒ 不要动★
    parameter [15:0] ANG_SL_Q6  = 16'd11520,   // 180° 窗口：实测【车正右】
    parameter [15:0] ANG_SR_Q6  = 16'd0,       // 0°   窗口：实测【车正左】
    parameter [15:0] HALF_F_Q6  = 16'd1280,    // 正前半窗 ±20°
    // ★V2.17★：±10° → **±20°**。原因（用户 00:48 日志）：绕障判"能不能回头"要看"车侧面离障碍多远"，
    //   而几个 ±10° 窗拼起来在 110°~125°、145°~170° 留下**缝隙**——箱子正好落在缝里时那几格
    //   读不到它，判据就被骗（车头刚过箱子就回头、把箱子推开）。
    //   放宽到 ±20° 后：车右/车左那一侧从 90° 到 200° 基本连成一片（只剩两条 5° 细缝）。
    parameter [15:0] HALF_S_Q6  = 16'd1280,    // 其余扇区半窗 ±20°
    parameter [15:0] A_MAX_Q6   = 16'd23040,   // 360°（0° 扇区跨界的另一半用）
    parameter [15:0] D_MIN_MM   = 16'd150,     // 小于此值当无效（雷达盲区）
    parameter [15:0] D_MAX_MM   = 16'd12000,   // 大于此值当无效
    // ---- 控制口径 ----
    parameter [15:0] SENS_FWD_MM = 16'd150,    // 雷达中心到车头前缘（实测 150mm，见顶层）
    parameter [15:0] SENS_FAR_MM = 16'd4000,   // "无点"折算成的距离（与超声超时值同量级）
    parameter [15:0] D_PASS_MM   = 16'd900,    // 盯视判"障碍末端已过去"的门槛
    parameter [7:0]  PASS_K      = 8'd3,       // 连续多少圈才算确认
    // ---- ★2a：障碍物角向连续段（相对判据，无绝对距离门限）★ ----
    //   这些数只是"尺子的精度"，不是策略参数（策略在 2b 里由测量算）：
    // ★V2.7（清单5 / H1）★：250→400mm、6°→10°。
    //   现象"H1"：有时进避障就原地不动（a=9 w=22）——触发那一圈 2a 根本没有段。
    //   真雷达到处是点但会抖，250mm 跳变 + 6° 间隔太严 ⇒ 箱子被切成碎段、甚至一段都不成立。
    //   放宽后仍能分清"箱子 600mm vs 旁边墙 2000mm"（tb_lidar_parse T8：76°/104° 不变，
    //   因为 40°→70° 那道缝是 30°、跳变 2382mm，照样断开）。
    parameter [15:0] RUN_JUMP_MM = 16'd400,    // 相邻两点距离跳变 ≤ 此值 ⇒ 视为同一段
    parameter [15:0] RUN_GAP_Q6  = 16'd640,    // 相邻两点方位间隔 ≥ 10°(640/64) ⇒ 断开
    // ★V2.5★：障碍段窗口改成【正前 ±45°】(45°~135°)——原来取"前半球 0~180° 里最近的东西"，
    //   结果车右侧 388mm 的椅子(落在 ~170°~179°)经常顶替正前方的箱子 ⇒ FSM 算出
    //   "往左只需挪 0mm"、角度取最小档 15°、收工判据立刻满足 ⇒ 现场表现为"转一点点就直冲"。
    //   现场日志铁证：P=0F0014（φ=15°、需求横移 ≤100mm，几何上应是 30°/272mm）。
    parameter [15:0] A_LO_Q6     = 16'd2880,   // 窗口下界 45°
    parameter [15:0] A_HI_Q6     = 16'd8640,   // 窗口上界 135°
    parameter integer STALE_CYC  = 25_000_000  // 断流判定：多少拍没成圈算断流（0.5s@50MHz）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  b_data,        // 来自 lidar_rx 的字节流
    input  wire        b_valid,       // 单拍有效
    input  wire        hdr_done,      // 完整应答头通过的单拍脉冲 ⇒ 按 5 字节重新对齐
    // ---- 盯视（来自 avoid_fsm）----
    input  wire        watch_en,      // 1 = 正在盯视（只在 S_LAT/S_NORTH/S_MERGE 为 1）
    input  wire [7:0]  watch_ang,     // 度：bit7=1 ⇒ 180°(正左)，=0 ⇒ 0°(正右)
    // ---- 原始量（给串口显示；FFFF = 本圈该扇区无有效点）----
    output reg  [15:0] front_mm,
    output reg  [15:0] left_mm,
    output reg  [15:0] right_mm,
    output reg  [15:0] side_l_mm,
    output reg  [15:0] side_r_mm,
    output reg         front_ok,
    output reg         left_ok,
    output reg         right_ok,
    output reg         side_l_ok,
    output reg         side_r_ok,
    output reg  [15:0] scan_pts,      // 上一整圈的有效点数
    output reg  [15:0] last_ang_deg,  // 最近一个点的角度（度）
    output reg  [15:0] last_dist_mm,  // 最近一个点的距离（mm）
    output reg  [15:0] node_cnt,      // 累计有效点（自由回绕）
    output reg  [15:0] scan_cnt,      // 累计完成圈数（自由回绕）
    output reg  [15:0] bad_cnt,       // 丢掉的字节数（错位/不合格，自由回绕）
    // ---- 状态 / 脉冲 ----
    output reg         scan_done_p,   // 每完成一圈 1 拍（给 avoid_fsm 启动门控 + 顶层 dist_valid）
    output reg         rev_no_pt,     // 本圈 360° 一个有效点都没有（1 拍，只作诊断）
    output reg         scan_fresh,    // 电平：已有过成圈且离上次成圈 < STALE_CYC
    // ---- 控制口径（同名同义，直接喂顶层/avoid_fsm）----
    output reg  [15:0] dist_f_mm,     // 正前（已折算到车头前缘；断流时强制 0）
    output reg         dist_f_v,      // 正前样本有效（= scan_fresh）
    output wire        echo_to,       // 回波/数据超时（电平：断流期间一直为 1）
    output reg  [15:0] prof_l_mm,     // 车左前（无点⇒SENS_FAR_MM）
    output reg  [15:0] prof_r_mm,     // 车右前
    output reg         prof_l_v,
    output reg         prof_r_v,
    output reg  [15:0] watch_mm,      // 盯视扇区（无点⇒SENS_FAR_MM）
    output reg         watch_v,       // 盯视样本有效（= scan_fresh & watch_en）
    output reg  [7:0]  watch_far_cnt, // 盯视方向连续 >= D_PASS 的圈数（饱和 PASS_K）
    // ================= ★2a：障碍物角向连续段（前半球"最近的那一个连通段"）★ =================
    //   语义：前半球（方位 0°~180°）里，最近点所在的那个连通段。
    //   段 = 相邻两点"距离跳变 ≤ RUN_JUMP_MM 且方位间隔 < RUN_GAP_Q6"的连通点集
    //   ⇒ 判据全部是"和上一个点比"，不含任何绝对距离门限。
    //   方位约定：0°=车正左、90°=正前、180°=车正右（角度增大＝往车的右边，实测）。
    //   坐标换算（2b 策略层用）：y(前) = d·sin(a)、x(右) = -d·cos(a)
    //   ⇒ 往左绕需要的横向净空 = |x_左边缘| + 含轮半宽 + 余量；
    //      往右绕需要的横向净空 =  x_右边缘  + 含轮半宽 + 余量
    output reg  [15:0] obs_a0_deg,    // 段起始方位角（度）
    output reg  [15:0] obs_d0_mm,     // 段起始端点的距离（mm，原始未折算）
    output reg  [15:0] obs_a1_deg,    // 段结束方位角（度）
    output reg  [15:0] obs_d1_mm,     // 段结束端点的距离（mm）
    output reg  [15:0] obs_dmin_mm,   // 段内最近距离（mm；FFFF = 本圈前半球没有段）
    output reg  [15:0] obs_amin_deg,  // 最近点所在方位角（度）
    output reg         obs_valid,     // 本圈前半球有段
    // ---- ★2a：正前扇区切成左右两半（2b 打破"两边一样空"时的平局用）★ ----
    output reg  [15:0] front_l_mm,    // A∈[70°,90°) 最小距离（偏车左）
    output reg  [15:0] front_r_mm,    // A∈[90°,110°] 最小距离（偏车右）
    output reg         front_l_ok,
    output reg         front_r_ok,
    // ---- ★V4.0a★ 72 扇区 × 5° 最小距离直方图（读端口）----
    //   hist_rd_addr=0..71；hist_rd_data 为上一整圈锁存值（1 拍读延迟）
    input  wire [6:0]  hist_rd_addr,
    output reg  [15:0] hist_rd_data,
    // ★V4.0c★ 直方图侧距：0°±15° / 180°±15° 的最小值（Bug2 恒距跟随用）
    output reg  [15:0] hist_side_l_mm,
    output reg  [15:0] hist_side_r_mm
);

    localparam [15:0] FAR = 16'hFFFF;

    reg [2:0]  idx;
    reg [7:0]  b0, b1, b2, b3;
    reg [15:0] f_acc, l_acc, r_acc, sl_acc, sr_acc, pts_acc;
    reg        started;               // 已经见过一次 S 位（第一圈只当"从此开始"）

    // ---- ★2a：正前左右半区累加 + 前半球角向连续段（流式、单遍、不用数组）★ ----
    reg [15:0] fl_acc, fr_acc;         // 正前偏车左/偏车右半区本圈最小值
    reg        r_open;                 // 当前段还在延续
    reg [15:0] r_a0q, r_a1q;           // 当前段：起始方位(Q6) / 结束方位(Q6)
    reg [15:0] r_d0,  r_d1;            // 当前段：起始端距离 / 结束端距离
    reg [15:0] r_min, r_last, r_amin;  // 当前段：最近距离 / 上一点距离 / 最近点方位
    reg        b_have;                 // 本圈已经有过"最优段"
    reg [15:0] b_a0q, b_a1q, b_d0, b_d1, b_min, b_amin;

    // ★V4.0a★ 72×5° 最小距离直方图：acc=本圈累加，lat=上一整圈锁存
    // ★V4.2g★ 开到 74 项：a_cur 最大 23039 时 (a*13)>>12 ≤ 73，
    //   省掉写地址上的 ">71" 比较器（PnR 最后 -0.125ns 就在这条路径上）。
    reg [15:0] hist_acc [0:73];
    reg [15:0] hist_lat [0:73];
    // ★V4.2h★ 直方图写入流水：把 "b1/b2 → 扇区译码 → 74 路写使能" 拆成两级。
    //   两个字节之间隔 ~87µs，晚一拍完全无影响。
    reg        hw_v;              // 上一拍有点待写
    reg [6:0]  hw_s;              // 待写扇区
    reg [15:0] hw_d;              // 待写距离
    integer    hi;                     // 直方图循环变量

    // 折算结果（在成圈那一拍算好并锁存）
    reg [15:0] front_ctrl, prof_l_ctrl, prof_r_ctrl, watch_ctrl;

    // ---- 组合：由已存的 b1/b2/b3 + 当前 b_data(=b4) 算角度与距离 ----
    wire [15:0] a_cur = {b2, b1} >> 1;          // 角度，单位 1/64°
    wire [15:0] d_cur = {b_data, b3} >> 2;      // 距离，mm
    wire        d_ok  = (d_cur >= D_MIN_MM) && (d_cur <= D_MAX_MM);
    // ★V4.2g★ 时序：原来是 a_cur*205 的 16×16 乘法（PnR slack -1.0ns）。
    //   205/65536 ≈ 13/4096，用三次移位相加代替乘法：a/512 + a/1024 + a/4096。
    //   等效扇区宽 4.92°（原 5.0°），对 5° 量化体系等价。
    //   (a*13)>>12 保留精度（先乘后移，避免逐项截断把 18 算成 17）；
    //   乘 13 = 8+4+1，综合成两级加法，不用 DSP。
    wire [18:0] hist_sec_mul = a_cur * 16'd13;
    wire [6:0]  hist_sec = hist_sec_mul[18:12];

    // ---- 扇区命中（全在 Q6 整数域比较）----
    wire in_f  = (a_cur >= (ANG_F_Q6  - HALF_F_Q6)) && (a_cur <= (ANG_F_Q6  + HALF_F_Q6));
    wire in_l  = (a_cur >= (ANG_L_Q6  - HALF_S_Q6)) && (a_cur <= (ANG_L_Q6  + HALF_S_Q6));
    wire in_r  = (a_cur >= (ANG_R_Q6  - HALF_S_Q6)) && (a_cur <= (ANG_R_Q6  + HALF_S_Q6));
    wire in_sl = (a_cur >= (ANG_SL_Q6 - HALF_S_Q6)) && (a_cur <= (ANG_SL_Q6 + HALF_S_Q6));
    wire in_sr = (a_cur <= (ANG_SR_Q6 + HALF_S_Q6)) || (a_cur >= (A_MAX_Q6 - HALF_S_Q6));

    // ---- ★2a：正前扇区左右半区（方位增大＝往车的右边）----
    wire in_fl = (a_cur >= (ANG_F_Q6 - HALF_F_Q6)) && (a_cur <  ANG_F_Q6);              // [70°,90°)
    wire in_fr = (a_cur >=  ANG_F_Q6             ) && (a_cur <= (ANG_F_Q6 + HALF_F_Q6)); // [90°,110°]

    // ---- ★2a：前半球连续段判据（与"上一个点"比，无绝对门限）----
    wire in_half  = (a_cur >= A_LO_Q6) && (a_cur <= A_HI_Q6); // ★V2.5：只看正前 ±45°★
    wire run_go   = d_ok && in_half;                         // 这个点参与分段
    wire cont_d   = (d_cur <= (r_last + RUN_JUMP_MM)) &&
                    ((d_cur + RUN_JUMP_MM) >= r_last);        // 距离与上一点连续
    wire cont_a   = (a_cur >= r_a1q) && (a_cur <= (r_a1q + RUN_GAP_Q6));  // 方位连续且不回跳
    wire run_join = r_open && cont_d && cont_a;
    // 本圈"最优段"的当前值（把正在延续的段也算进去）——供 S 位组合锁存
    wire        take_cur = r_open && ((!b_have) || (r_min < b_min));
    wire [15:0] cur_min  = take_cur ? r_min  : (b_have ? b_min  : FAR);
    wire [15:0] cur_a0q  = take_cur ? r_a0q  : b_a0q;
    wire [15:0] cur_d0   = take_cur ? r_d0   : b_d0;
    wire [15:0] cur_a1q  = take_cur ? r_a1q  : b_a1q;
    wire [15:0] cur_d1   = take_cur ? r_d1   : b_d1;
    wire [15:0] cur_amin = take_cur ? r_amin : b_amin;
    wire        cur_ok   = b_have | r_open;

    // ---- 断流计数（每成圈清零，饱和在 STALE_CYC 不溢出）----
    reg [27:0] stale_cnt;
    wire       fresh_raw = (stale_cnt < STALE_CYC);
    wire       fresh     = started & fresh_raw;      // 还没成过圈 ⇒ 不算新鲜
    // ★断流电平★：没数据就一直是 1 ⇒ 顶层 to_run 计数到 10 ⇒ sensor_fault ⇒ 停车 + LED4 8Hz
    wire       link_stale = ~fresh_raw;

    assign echo_to = link_stale;

    // ---- 折算函数：FFFF ⇒ SENS_FAR_MM（"那个方向没有东西"，不再折算）；否则
    //      fwd_adj=1 时折算到车头前缘（减 SENS_FWD_MM，下溢钳 0）----
    function [15:0] map_far;
        input [15:0] raw;
        input        fwd_adj;
        reg   [15:0] v;
        begin
            if (raw == FAR) begin
                v = SENS_FAR_MM;
            end else if (fwd_adj) begin
                if (raw > SENS_FWD_MM) v = raw - SENS_FWD_MM;
                else                   v = 16'd0;
            end else begin
                v = raw;
            end
            map_far = v;
        end
    endfunction

    // ★V4.0c★ 5 路取最小（FFFF=无点，天然不参与取小）
    function [15:0] min5;
        input [15:0] a, b, c, d, e;
        begin
            min5 = a;
            if (b < min5) min5 = b;
            if (c < min5) min5 = c;
            if (d < min5) min5 = d;
            if (e < min5) min5 = e;
        end
    endfunction

    // ★V4.0c★ 9 路取最小（侧距窗口 0°/180°±20°，含边界扇区）
    function [15:0] min9;
        input [15:0] a, b, c, d, e, f, g, h, i;
        begin
            min9 = a;
            if (b < min9) min9 = b;
            if (c < min9) min9 = c;
            if (d < min9) min9 = d;
            if (e < min9) min9 = e;
            if (f < min9) min9 = f;
            if (g < min9) min9 = g;
            if (h < min9) min9 = h;
            if (i < min9) min9 = i;
        end
    endfunction

    // 盯视扇区选择：watch_ang 的 bit7 = 1 ⇒ 180°(正左)，= 0 ⇒ 0°(正右)
    wire        watch_left = watch_ang[7];
    wire [15:0] watch_acc  = watch_left ? sl_acc : sr_acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx <= 3'd0;
            b0 <= 8'd0; b1 <= 8'd0; b2 <= 8'd0; b3 <= 8'd0;
            f_acc <= FAR; l_acc <= FAR; r_acc <= FAR; sl_acc <= FAR; sr_acc <= FAR;
            pts_acc <= 16'd0;
            front_ctrl <= FAR; prof_l_ctrl <= FAR; prof_r_ctrl <= FAR; watch_ctrl <= FAR;
            front_mm <= FAR; left_mm <= FAR; right_mm <= FAR;
            side_l_mm <= FAR; side_r_mm <= FAR;
            front_ok <= 1'b0; left_ok <= 1'b0; right_ok <= 1'b0;
            side_l_ok <= 1'b0; side_r_ok <= 1'b0;
            scan_pts <= 16'd0; last_ang_deg <= 16'd0; last_dist_mm <= 16'd0;
            node_cnt <= 16'd0; scan_cnt <= 16'd0; bad_cnt <= 16'd0;
            scan_done_p <= 1'b0; rev_no_pt <= 1'b0; scan_fresh <= 1'b0;
            dist_f_mm <= 16'd0; dist_f_v <= 1'b0;
            prof_l_mm <= FAR; prof_r_mm <= FAR; prof_l_v <= 1'b0; prof_r_v <= 1'b0;
            watch_mm <= FAR; watch_v <= 1'b0; watch_far_cnt <= 8'd0;
            // ★V4.35f★ hist_acc/hist_lat 是 [0:73]（V4.2g 扩的，为了覆盖 360/0 度那个扇区），
            //   原来三处循环都只做 72 项 ⇒ 扇区 72/73(355~360 度) 永不清零、
            //   hist_lat 也永不更新 ⇒ 那两格会一直留着历史最近距离（假障碍源）。
            for (hi = 0; hi < 74; hi = hi + 1) begin
                hist_acc[hi] <= FAR; hist_lat[hi] <= FAR;
            end
            hist_side_l_mm <= FAR; hist_side_r_mm <= FAR;
            // ★2a★
            obs_a0_deg <= 16'd0; obs_d0_mm <= FAR; obs_a1_deg <= 16'd0; obs_d1_mm <= FAR;
            obs_dmin_mm <= FAR; obs_amin_deg <= 16'd0; obs_valid <= 1'b0;
            front_l_mm <= FAR; front_r_mm <= FAR; front_l_ok <= 1'b0; front_r_ok <= 1'b0;
            fl_acc <= FAR; fr_acc <= FAR;
            r_open <= 1'b0; r_a0q <= 16'd0; r_a1q <= 16'd0; r_d0 <= FAR; r_d1 <= FAR;
            r_min <= FAR; r_last <= FAR; r_amin <= 16'd0;
            b_have <= 1'b0; b_a0q <= 16'd0; b_a1q <= 16'd0; b_d0 <= FAR; b_d1 <= FAR;
            b_min <= FAR; b_amin <= 16'd0;
            started <= 1'b0; stale_cnt <= 28'd0;
            hw_v <= 1'b0; hw_s <= 7'd0; hw_d <= FAR;
        end else begin
            // 默认：脉冲类信号每拍回落
            // ★V4.2h★ 直方图 pending 写：上一拍的点在这一拍落到 hist_acc。
            //   若本拍正好是 S 位清数组，clear 在下面、赋值更晚 ⇒ clear 优先，符合语义。
            if (hw_v) begin
                if (hw_d < hist_acc[hw_s]) hist_acc[hw_s] <= hw_d;
            end
            scan_done_p <= 1'b0;
            rev_no_pt   <= 1'b0;

            // ---- 链路新鲜度（电平）----
            if (stale_cnt < STALE_CYC) stale_cnt <= stale_cnt + 28'd1;
            scan_fresh <= fresh;
            dist_f_v   <= fresh;
            dist_f_mm  <= fresh ? front_ctrl : 16'd0;   // ★断流 ⇒ 强制 0mm（被堵）★
            prof_l_v   <= fresh;
            prof_r_v   <= fresh;
            prof_l_mm  <= prof_l_ctrl;
            prof_r_mm  <= prof_r_ctrl;
            watch_mm   <= watch_ctrl;
            watch_v    <= fresh & watch_en;

            if (b_valid) begin
                case (idx)
                    // ---- node byte0：bit1 必须是 bit0 的反相，否则就是错位 ----
                    3'd0: begin
                        if (b_data[1] == ~b_data[0]) begin
                            b0  <= b_data;
                            idx <= 3'd1;
                        end else begin
                            bad_cnt <= bad_cnt + 16'd1;   // 留在 idx=0 继续找
                        end
                    end
                    // ---- node byte1：★bit0 是 C 校验位，必须为 1★（官方同判据）----
                    3'd1: begin
                        if (b_data[0] == 1'b1) begin
                            b1  <= b_data;
                            idx <= 3'd2;
                        end else begin
                            bad_cnt <= bad_cnt + 16'd1;   // 错位：丢掉这一字节，回 idx=0 重找
                            idx     <= 3'd0;
                        end
                    end
                    3'd2: begin b2 <= b_data; idx <= 3'd3; end
                    3'd3: begin b3 <= b_data; idx <= 3'd4; end

                    // ---- 第 5 个字节到齐：本点成立，归约 ----
                    3'd4: begin
                        idx          <= 3'd0;
                        node_cnt     <= node_cnt + 16'd1;
                        last_ang_deg <= a_cur >> 6;     // 度（整数）
                        last_dist_mm <= d_cur;

                        if (b0[0]) begin
                            // ★S=1：新一圈开始 ⇒ 先把上一圈结果锁存出去，再从本点重新累加★
                            //   · 第一圈（started=0）只当"从此开始"，不锁存（否则会把
                            //     半圈的累加器当成一整圈，可能出现"假空旷"）
                            //   · scan_cnt 只数【已完成的圈】
                            if (started) begin
                                front_mm   <= f_acc;
                                left_mm    <= l_acc;
                                right_mm   <= r_acc;
                                side_l_mm  <= sl_acc;
                                side_r_mm  <= sr_acc;
                                front_ok   <= (f_acc  != FAR);
                                left_ok    <= (l_acc  != FAR);
                                right_ok   <= (r_acc  != FAR);
                                side_l_ok  <= (sl_acc != FAR);
                                side_r_ok  <= (sr_acc != FAR);
                                scan_pts   <= pts_acc;
                                scan_cnt   <= scan_cnt + 16'd1;
                                scan_done_p<= 1'b1;
                                rev_no_pt  <= (pts_acc == 16'd0);

                                // ★2a：本圈障碍段（含正在延续的那一段）锁存出去★
                                //   坐标：0°=车正左、90°=正前、180°=车正右
                                obs_a0_deg  <= cur_a0q >> 6;
                                obs_d0_mm   <= cur_d0;
                                obs_a1_deg  <= cur_a1q >> 6;
                                obs_d1_mm   <= cur_d1;
                                obs_dmin_mm <= cur_ok ? cur_min : FAR;
                                obs_amin_deg<= cur_amin >> 6;
                                obs_valid   <= cur_ok;
                                front_l_mm  <= fl_acc;
                                front_r_mm  <= fr_acc;
                                front_l_ok  <= (fl_acc != FAR);
                                front_r_ok  <= (fr_acc != FAR);
                                // ★V4.0a★ 锁存上一整圈直方图，再从本点开始新一圈
            // ★V4.35f★ hist_acc/hist_lat 是 [0:73]（V4.2g 扩的，为了覆盖 360/0 度那个扇区），
            //   原来三处循环都只做 72 项 ⇒ 扇区 72/73(355~360 度) 永不清零、
            //   hist_lat 也永不更新 ⇒ 那两格会一直留着历史最近距离（假障碍源）。
                                for (hi = 0; hi < 74; hi = hi + 1) begin
                                    hist_lat[hi] <= hist_acc[hi];
                                    hist_acc[hi] <= FAR;
                                end
                                // ★V4.2g★ 不再用 9 路组合取小（PnR 实测 slack -15.4ns、1558 个端点违规）。
                                //   这两个"直方图侧距"的窗口与现有流式累加器【完全相同】：
                                //     in_sr = 0°±20°（车正左）  ⇒ sr_acc 就是 hist_side_l
                                //     in_sl = 180°±20°（车正右）⇒ sl_acc 就是 hist_side_r
                                //   流式累加每来一个点只比一次，组合深度 O(1)。
                                hist_side_l_mm <= sr_acc;
                                hist_side_r_mm <= sl_acc;

                                // 控制口径（成圈这一拍算好）
                                front_ctrl  <= map_far(f_acc,  1'b1);
                                prof_l_ctrl <= map_far(l_acc,  1'b0);
                                prof_r_ctrl <= map_far(r_acc,  1'b0);
                                watch_ctrl  <= map_far(watch_acc, 1'b0);

                                // 盯视"末端已过去"计数：★只在 watch_en=1 时数★
                                //   （不门控的话侧向开阔时计数会在进腿前就满 ⇒
                                //     S_LAT 会提前把 lat_target 截到 M1_STEPS ⇒ 撞墙）
                                if (!watch_en) begin
                                    watch_far_cnt <= 8'd0;
                                end else if ((watch_acc != FAR) && (watch_acc < D_PASS_MM)) begin
                                    watch_far_cnt <= 8'd0;      // 这一圈还看见障碍
                                end else if (watch_far_cnt < PASS_K) begin
                                    watch_far_cnt <= watch_far_cnt + 8'd1;
                                end
                            end else begin
                                started <= 1'b1;
                            end

                            stale_cnt <= 28'd0;             // 成圈 ⇒ 链路新鲜

                            // 清累加器，从本点重新累加
                            f_acc   <= (in_f  && d_ok) ? d_cur : FAR;
                            l_acc   <= (in_l  && d_ok) ? d_cur : FAR;
                            r_acc   <= (in_r  && d_ok) ? d_cur : FAR;
                            sl_acc  <= (in_sl && d_ok) ? d_cur : FAR;
                            sr_acc  <= (in_sr && d_ok) ? d_cur : FAR;
                            fl_acc  <= (in_fl && d_ok) ? d_cur : FAR;
                            fr_acc  <= (in_fr && d_ok) ? d_cur : FAR;
                            pts_acc <= d_ok ? 16'd1 : 16'd0;
                            // ★V4.2h★ 直方图写入寄存一拍（见下方 pending 写）
                            hw_v <= d_ok; hw_s <= hist_sec; hw_d <= d_cur;
                        end else begin
                            if (in_f  && d_ok && (d_cur < f_acc))  f_acc  <= d_cur;
                            if (in_l  && d_ok && (d_cur < l_acc))  l_acc  <= d_cur;
                            if (in_r  && d_ok && (d_cur < r_acc))  r_acc  <= d_cur;
                            if (in_sl && d_ok && (d_cur < sl_acc)) sl_acc <= d_cur;
                            if (in_sr && d_ok && (d_cur < sr_acc)) sr_acc <= d_cur;
                            if (in_fl && d_ok && (d_cur < fl_acc)) fl_acc <= d_cur;
                            if (in_fr && d_ok && (d_cur < fr_acc)) fr_acc <= d_cur;
                            if (d_ok) pts_acc <= pts_acc + 16'd1;
                            hw_v <= d_ok; hw_s <= hist_sec; hw_d <= d_cur;
                        end

                        // ================= ★2a：前半球角向连续段（流式单遍）★ =================
                        //   段不跨圈：S 位（新一圈第一个点）先把上一圈还没收尾的段收进"本圈最优"
                        //   （其贡献已在上面 obs_* 锁存里算过），再清"本圈最优"、从本点重新开段。
                        if (b0[0]) begin
                            r_open <= 1'b0;
                            b_have <= 1'b0; b_a0q <= 16'd0; b_a1q <= 16'd0;
                            b_d0   <= FAR;  b_d1  <= FAR;
                            b_min  <= FAR;  b_amin<= 16'd0;
                        end
                        if (run_go) begin
                            if ((!r_open) || b0[0] || (!run_join)) begin
                                // ① 收尾旧段（若在延续）⇒ 参与"本圈最优段"比较
                                if (r_open && (!b0[0]) && ((!b_have) || (r_min < b_min))) begin
                                    b_have <= 1'b1; b_min <= r_min; b_amin <= r_amin;
                                    b_a0q  <= r_a0q; b_d0 <= r_d0;
                                    b_a1q  <= r_a1q; b_d1 <= r_d1;
                                end
                                // ② 从本点开新段
                                r_open <= 1'b1;
                                r_a0q  <= a_cur; r_d0 <= d_cur;
                                r_a1q  <= a_cur; r_d1 <= d_cur;
                                r_min  <= d_cur; r_last <= d_cur; r_amin <= a_cur;
                            end else begin
                                // ③ 延续本段
                                r_a1q  <= a_cur; r_d1 <= d_cur; r_last <= d_cur;
                                if (d_cur < r_min) begin r_min <= d_cur; r_amin <= a_cur; end
                            end
                        end
                    end
                    default: idx <= 3'd0;
                endcase
            end

            // ---- 收到新的【完整】应答头 ⇒ 按 5 字节重新对齐、并清空本圈累加器 ----
            //   （放在后面：同一拍上它的 idx 赋值优先级更高）
            if (hdr_done) begin
                idx     <= 3'd0;
                f_acc   <= FAR;
                l_acc   <= FAR;
                r_acc   <= FAR;
                sl_acc  <= FAR;
                sr_acc  <= FAR;
                fl_acc  <= FAR;
                fr_acc  <= FAR;
            // ★V4.35f★ hist_acc/hist_lat 是 [0:73]（V4.2g 扩的，为了覆盖 360/0 度那个扇区），
            //   原来三处循环都只做 72 项 ⇒ 扇区 72/73(355~360 度) 永不清零、
            //   hist_lat 也永不更新 ⇒ 那两格会一直留着历史最近距离（假障碍源）。
                for (hi = 0; hi < 74; hi = hi + 1) hist_acc[hi] <= FAR;
                pts_acc <= 16'd0;
                // ★2a：重新对齐也从"无段"重来，避免把半圈数据当一整段
                r_open <= 1'b0; b_have <= 1'b0; b_min <= FAR;
                started <= 1'b0;     // 收到新命令 = 扫描要重来 ⇒ "第一圈"重新计
                stale_cnt <= 28'd0;
            end
        end
    end

    // ★V4.0a★ 72 扇区直方图读端口（组合读；地址 0..71）
    // ★V4.2h★ 改成寄存读：74:1 的 16 位组合 mux 是全设计最慢的一条路
    //   （V4.2g PnR 余量仅 +0.010ns）。寄存后 mux 只驱动一个寄存器，
    //   比较/使能逻辑在下一拍用寄存值，路径被拆成两段。
    //   注意：读协议从"地址下一拍有效"变成"地址后第 2 拍有效"，
    //   avoid_fsm 的三个扫描循环已同步插入一拍等待。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hist_rd_data <= FAR;
        else        hist_rd_data <= hist_lat[hist_rd_addr];
    end

endmodule
