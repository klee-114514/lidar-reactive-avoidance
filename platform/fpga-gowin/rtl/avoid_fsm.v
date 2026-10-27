//=============================================================================
// avoid_fsm.v —— 2b：按测量算角度的"边走边测"绕障（2026-10-15）
//-----------------------------------------------------------------------------
// ★本版把老的"90° Z 形"整套作废★：THETA_DEG/SIN_Q10/COS_Q10/D4_MM/M1_MM/
//   L_MAX_MM/RETURN_TO_LINE 的 4 次转向、以及"原地转 90° + 垂直侧移"都不再存在。
//
// 思路（用户方案：边走边测边算，闭环反馈；除故障处理外不写死角度和距离）：
//   1) 触发：正前折算距离 front_mm ≤ D_ARM ⇒ 进避障（触发仍用"正前"口径）；
//   2) 选边：用 2a 的障碍段两端 (a0,d0)/(a1,d1) 算两个端点的横向位置
//        x(右) = -d·cos(a)，y(前) = d·sin(a)   （a：0°=车正左、90°=正前、180°=车正右）
//      往左绕要的横移  Δx_L = (车宽/2 + 余量) - x_L
//      往右绕要的横移  Δx_R = (车宽/2 + 余量) + x_R
//      再各自跟"那一侧实测可用空间"（正左/正右窗口 - 车宽/2）比 ⇒ 取"能过去且最省"的一侧；
//   3) 算角度：tanφ ≥ Δx / D（D = 障碍近面到【后轮轴】的距离 = 实测距离 + (轴-雷达)）
//      用 5° 一档【递增搜索】（tan 表 + 乘法比较，不做除法）⇒ 没有任何写死的角度；
//   4) 斜行：每 STEP_MM 一小段用 A_FWD_HOLD 保持航向 "yaw_base ± φ"（边走边修），
//      每小段结束检查障碍侧间隙（< SIDE_MIN_MM ⇒ 判失败停车）；
//   5) 末端判据：障碍侧窗口连续看不清了（watch_far_cnt 饱和）且 累计前进 ≥ ADV_MIN_MM；
//   6) 回线：用同一个 φ 斜着把横向位移走回来 ⇒ 再转回 yaw_base（保证"角度平行"）；
//   7) 续跑：把触发瞬间冻结的任务剩余里程（remain_at_arm）交回队列
//      ⇒ 避障这段里程不计入任务（车最终走的总里程 = 任务里程 + 绕障额外里程）；
//   8) 兜底：绕障额外里程 > EXTRA_MAX_MM（=80cm）或 时长 > AVOID_TO_MS ⇒ 停车锁存；
//      连续绕障次数超过 RETRY_MAX+1 次也判失败（对应"再绕一次"的约定）。
//
// 说明：x/y 换算只用到 sin/cos/tan 的 5° 档小表（case 实现，不引 ROM 核），
//       全部整数运算；乘法每拍最多一次（顺序推进），不会堆出长组合路径。
//=============================================================================
// ★V2.7（2026-10-15，按分账「V2.6 待做清单」施工）★
//   1) 收工判据改成"障碍侧实测净空"（清单1+2）：min(障碍侧 0°/180° 窗口,
//      障碍侧正前半区 FL/FR, obs_dmin) ≥ SIDE_MIN_MM 才算绕过去。
//      V2.5 把侧窗换成 FL/FR 是倒退——箱子贴到车身侧面后 FL/FR 看不见它，
//      车会"贴着障碍就收工"，用户原话"你还是没有边走边测"。
//   2) 小步长 80mm → 40mm（清单3）：判定频率翻倍，一步一测。
//   3) 出航向/回正/回线这几处【原地转】改用全速（清单3）：pend_slow=0。
//   4) 回线横向残差门 40mm → RET_TOL_MM(15mm)（清单4）。
//   5) 触发时本圈没测到障碍段（obs_valid=0）不再立刻认输（清单5/H1）：
//      进 S_REWAIT 等下一圈（最多 CALC_RETRY_MAX 圈）重算。
//   6) 横移/里程记账改用【编码器实测弧长】step_arc_cnt（清单6），不用命令步长。
//   7) 段头 HONOR_STALL_CYC(600ms) 内忽略 stall（H3）：电机侧"停稳"窗口里
//      两轮本来就不转，顶层的堵转检测会误报。
//=============================================================================
// ★V2.8（2026-10-15，V2.6 清单第 8/9 项）★
//   8) 侧向余量 M_SIDE_MM 60 → 90mm（顶层参数 / 本模块默认值 / tb 例化 三处一起改）。
//   9) 选边判死：可行性判据改用【车实际要挪的量】dxL_use/dxR_use（含物理下限
//      HALF_W_M）；"名义上能过、实际挪不过去" ⇒ 新原因码 0x2A 判死停车（不许硬走）。
//=============================================================================
// ★V2.9（2026-10-15，用户：「不要停，停顿非常明显，可以用降低速度来代替」）★
//   10) 斜行腿 / 回线腿：巡航全速 → 绕障降速档 AVOID_SPD(288≈100mm/s)，宁慢不停；
//       AVOID_TO_MS 12000 → 18000ms（降速后一次典型绕障 ≈14.5s，12s 会误判 0x35）。
//   11) 配套改动在电机侧：段间"短停"不再清 PID 积分（motor_pid）、原地转前
//       不再停死而是极低速起转 SOFT_TURN_SPD（motor_driver_top）。
//=============================================================================
// ★V2.9b（2026-10-15，用户原话）★「走到障碍物侧面的时候看侧面离障碍物多远来判断车到底
//   能回头了没有」「说了不能写死角度」「选了往左绕就只关注雷达前面和右边的数据」
//   12) **回头判据改成全实测**（这是本次最关键的一条）：
//       ① 正前 ±45° 扇区【连续 FRONT_GONE_K(3) 圈】都看不到障碍段（obs_valid=0）
//          ⇒ 车已经走到障碍的侧面/身后（这是"走到侧面了"的实测判据，不是角度也不是里程）；
//       ② 障碍侧那一格 0°/180° 窗口的**实测垂距** ≥ 回头扫掠净空 SIDE_MIN_MM
//          （该窗口读数在 lidar_parse 里就是"车体纵向中心线到障碍侧面的垂距"）；
//       两条都满足 ⇒ 立刻回头（S_T_BACK），用时不超过一小步。
//   13) 原来那两条【推算量】判据（`lat_acc ≥ dx_use`、`fwd_acc ≥ ADV_MIN_MM`）**不再参与
//       "能不能回头"**——它们正是"侧窗早就看见障碍过去了、车还一直往前斜着走"的成因；
//       `dx_sel/dx_use` 只保留给"斜行角度 φ 怎么算"用。
//   14) `SIDE_MIN_MM` 220 → 300mm，并明确它现在就是【回头扫掠净空】：
//       = 车体扫掠半径(240×220 的车：半对角 163、工程上取的 211)+ 侧向余量 M_SIDE(90) ≈ 300。
//=============================================================================
// ★V2.10（2026-10-15，用户：「这次依旧走到障碍物侧面然后一直直走」）★
//   15) **修正 V2.9b 的错误假设**：V2.9b 只认"正前 ±45° 扇区连续 3 圈看不到障碍段"，
//       而那个现象只在我自造的 tb 激励里成立；现场箱子贴着车走时，它的近端/侧翼常常仍在
//       正前扇区里 ⇒ `front_gone` 永不成立 ⇒ 车一直往前开（"一直直走"）。
//       现在回头判据 = 【两条实测证据取"或"】：A 障碍侧窗口"先贴过(≤300)再变远(≥300)"
//       （`side_close_lat`）；B 正前连续 3 圈无段（`front_gone`）。
//       两条都仍要求"当前侧距 ≥ SIDE_MIN_MM"，且**都不含角度、不含里程推算**。
//=============================================================================
// ★V2.11（2026-10-15 23:2x，用户现场日志）★
//   16) **把旧兜底加回来**：用户日志 `R=047B`(车正右 1147mm，永远 ≥300 ⇒ A 路永不置位)
//       + `E=520283650…`(正前 82~101° 一直有段 ⇒ B 路不成立) ⇒ A/B 同时死 ⇒ 车不回头。
//       现在回头 = **三条取"或"**：A/B 实测路（原因码 0x33）+ C 兜底（= V2.7/V2.8 旧判据：
//       推算横移 ≥ dx_use 且 gap_obs ≥ `TURN_FALLBACK_MM(220)` 且 前进 ≥ ADV_MIN，原因码 **0x3D**）。
//       两个码分开打，现场一眼看出是哪条放行的。
//=============================================================================
module avoid_fsm #(
    // ---- 车体几何（顶层按实测传入）----
    parameter [15:0] CAR_W_MM    = 16'd220,   // ★含轮最外缘宽度★（车体170 + 两侧各25）
    parameter [15:0] CAR_L_MM    = 16'd240,   // 车长
    parameter [15:0] AXLE_FWD_MM = 16'd175,   // 后轮轴 → 车头前缘
    parameter [15:0] SENS_FWD_MM = 16'd150,   // 雷达中心 → 车头前缘
    // ---- 策略参数（都是"余量/步长"这类几何常数；角度一律算出来）----
    parameter [15:0] D_ARM_MM    = 16'd450,   // 触发距离（= D_STOP，正前折算口径）
    // ★V2.18★：90 → **60mm**。用户 2026-10-16 01:0x：「这轮做的还可以…依旧老问题，还是离障碍物有点远了才回来」
    //   ⇒ 把侧向余量收回来（他最初要 90 是为了"防蹭"，现在"绕得远"是主要抱怨，这两者本来就对立）。
    //   它参与 HALF_W_M = 110 + M_SIDE ⇒ 200→**170mm**（"至少要挪开多少"和"那一侧剩多少"同时松 30mm）。
    //   ⚠️ 若上板又出现"蹭到"，把它加回 75/90 即可（一个数）。
    parameter [15:0] M_SIDE_MM   = 16'd60,    // 侧向安全余量
    // ★V2.7（清单3）：80 → 40mm。判定粒度翻倍（一步一测），每步 ≈0.32s@巡航★
    parameter [15:0] STEP_MM     = 16'd40,    // 斜行/回线小步长
    // ★V2.15★：回头净空由车体尺寸算出；★V2.18★：改用**真实扫掠半径（半对角线）**：
    //   车原地转时车体扫过的半径 = √((车长/2)² + (含轮车宽/2)²) = √(120²+110²) ≈ 163mm，取 **170**；
    //   + 侧向余量 M_SIDE_MM(60) ⇒ 240×220 的车 = **230mm**（原来按"车长/2+车宽/2=230"算是 320，
    //   比真实需要大近一倍 ⇒ 车要多走 ~90mm 才回头，这正是用户说的"离障碍物有点远了才回来"）。
    parameter [15:0] SIDE_MIN_MM = 16'd170 + M_SIDE_MM,
    // ★V2.17★：删掉 PASS_GROW_MM（"障碍段自己变远 ≥100mm"）——它也会被"段换对象"骗：
    //   用户 00:48 日志里 `D=`(段内最近点)从 608 跳到 734（箱子离开正前扇区、段换成了别的东西），
    //   被判成"已经走过它"⇒ 车头刚过箱子就回头、把箱子推开。现在改用"跨过门槛"的过程判据：
    //   本次曾经贴到过 ≤ 回头净空，且**现在**已经 ≥ 回头净空 ⇒ 才算走过它（见 turn_now_ok）。
    parameter [15:0] PHI_MAX_DEG = 16'd60,    // 允许的最大转角（几何上一般 20~45°）
    // ★V2.1★：末端判据的"最少前进量"。
    // ★V2.9b★：它（和 lat_acc 一样）**不再参与"能不能回头"**——回头改看实测两条
    //   （正前连续 3 圈无段 + 障碍侧实测垂距 ≥ SIDE_MIN_MM）。参数保留备查/备用。
    parameter [15:0] ADV_MIN_MM  = 16'd300,
    parameter [15:0] D_SAFE_MM   = 16'd50,    // 算角度时 D 先扣掉的安全量
    // ★V2.28a★：回头不仅要“前进够 ADV_MIN”，还要真正越过障碍近面
    //   一个保守进深，否则车头刚过 20~30mm 就切回原航线，回线会扫进箱子。
    parameter [15:0] OBST_DEPTH_GUARD_MM = 16'd350,
    parameter [7:0]  PASS_K      = 8'd3,      // 盯视"没障碍了"要连续几圈
    // ★V2.2：转向后的"回读校验"——实测车常只转出目标的 60%（边跑边转被拖着走）★
    //   每次转完用 IMU 核 |实际 − 目标|：超过 YAW_TOL_CNT(≈8°) 就重发该转向，最多 TURN_RETRY 次
    // ★V2.3：收紧到 4°（728 计数）——IMU 实测很准（静止不乱跳、手转跟手），
    //   所以"转到位"可以判得更严；编码器打滑不再参与这个判据★
    parameter [15:0] YAW_TOL_CNT = 16'd728,   // 4°（1° = 182 计数）
    // ★V2.13★：斜行期间的"补转"门限单独放宽到 8°（用户现场"一直在摆头"）。
    //   依据（23:45 日志）：斜行命令 φ=30°，而实测航向只有 ~17° 且抖动 ±7°
    //   ⇒ 用 4° 的 YAW_TOL_CNT 去卡，斜行一趟要补转 3 次（每次一个原地转身）＝ 看得见的摆头。
    //   8° 只用于"斜行中要不要再补一次"，出航向/回正的校验仍是 YAW_TOL_CNT(4°)。
    // ★V2.18u★：8° → 15°。斜行中航向偏 8° 就插一次补转，现场
    //   09:08~09:13 连续 w=3C / w=39，Y 在 34°→16°→45°→38° 来回摆。
    //   15° 以内交给 A_FWD_HOLD 连续修，减少原地补转形成的摆头。
    // ★V4.35g★ 8° → 5°：14:36 现场入口转身目标 31.5°、实际最低 25.4°（过冲 6.1°），
    //   8° 的宽容度放过了它 ⇒ 车带 6° 偏差开走、靠 A_FWD_HOLD 慢慢修，
    //   观感就是"一边往前一边往外转"。收紧到 5° 让 ret_fix_cnt 补航向更早介入。
    //   不取 3°：原地转本身有惯性过冲，阈值太紧会变成来回摆头。
    parameter [15:0] RET_AIM_TOL = 16'd910,   // ★V4.35g★ 5°：回线每步允许的航向偏差
    parameter [15:0] AIM_TOL_CNT = 16'd2730,  // 15°
    parameter [3:0]  TURN_RETRY  = 4'd3,
    parameter [15:0] ADJ_STEP_DEG= 16'd5,     // 侧向不够时每次加大的角度
    // ★V2.30★：只允许一次 +5° 修正；连续重瞄会把车摇成 V2.29 那样。
    parameter [3:0]  ADJ_MAX     = 4'd1,
    parameter [31:0] EXTRA_MAX_MM= 32'd800,   // 绕障【额外】里程上限（用户定：80cm）
    // ★V2.9★：12000 → 18000ms；★V2.12★：18000 → 25000ms。
    //   依据（用户 23:35 现场日志）：进绕障 23:35:26.7 → 回头 37.3 → 回线 38.0 →
    //   44.7 判 `w=35` 超时，正好 18.0s；即"降速后的斜行(10.6s)+回线(6.6s)+几次起转"
    //   已经吃满 18s，回线差一点点没走完就被砍。放宽到 25s 留出余量。
    parameter [31:0] AVOID_TO_MS = 32'd25000, // 绕障总时长上限
    parameter [3:0]  RETRY_MAX   = 4'd1,      // 允许的额外绕障次数（=1 ⇒ 最多绕 2 次）
    // ★V2.14★：3000 → 4500ms。用户 00:02 日志：一次"正在收敛的原地转"转了 3.2s 就被
    //   3s 看门狗砍成 `w=31` 判失败 ⇒ 自动恢复后又重转一次（"来回转 90 度"）。
    //   斜行小步只要 ~0.4s，放宽到 4.5s 不影响它们，只给原地转留收敛余量。
    parameter [31:0] STEP_TO_MS  = 32'd4500,  // 单个小步卡死判据（兼作原地转的收敛余量）
    // ★V2.28★：原地转的 FSM 看门狗必须比电机侧 PIVOT_MS(5000ms) 长，
    //   否则电机还没到自己的兜底时间，FSM 先把这一段判死。
    parameter [31:0] PIVOT_TO_MS = 32'd5500,
    // ★V2.28★：S_RESUME 交回的是“任务剩余里程”，不是一个小步；
    //   用 4.5s 会在一米以上的任务上误报 w=51。
    parameter [31:0] RESUME_TO_MS= 32'd60000,
    parameter [31:0] CNT_PER_MM  = 32'd574,   // 编码器计数/mm
    parameter [31:0] N_TURN_90   = 32'd101600,// 90° 转向的编码器弧长（IMU 掉线兜底用）
    parameter CLK_FREQ_HZ        = 50_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    // ---- 感知（lidar_parse）----
    input  wire [15:0] front_mm,
    input  wire        front_v,
    input  wire [15:0] prof_l_mm,
    input  wire        prof_l_v,
    input  wire [15:0] prof_r_mm,
    input  wire        prof_r_v,
    input  wire        scan_done,
    input  wire [15:0] watch_mm,
    input  wire        watch_v,
    input  wire [7:0]  watch_far_cnt,
    output reg         watch_en,
    output reg  [7:0]  watch_ang,
    // ---- ★2a：障碍段两端 + 正前左右半区 + 正左/正右可用空间★ ----
    input  wire [15:0] obs_a0, obs_d0, obs_a1, obs_d1, obs_dmin, obs_amin,
    input  wire        obs_valid,
    // ★V4.0b★ 72扇区×5°直方图读端口（来自 lidar_parse）
    output reg  [6:0]  hist_rd_addr,
    input  wire [15:0] hist_rd_data,
    input  wire [15:0] hist_side_l_mm,
    input  wire [15:0] hist_side_r_mm,
    input  wire [15:0] front_l_mm, front_r_mm,
    input  wire        front_l_ok, front_r_ok,
    // ★V2.15★：35°~55° / 125°~145° 两格（"障碍侧实测最近距离"要三格取最小）
    input  wire [15:0] wide_l_mm, wide_r_mm,
    input  wire [15:0] left_mm,  input wire left_ok,    // 0°  窗口 = 车正左
    input  wire [15:0] right_mm, input wire right_ok,   // 180°窗口 = 车正右
    // ---- 触发/复位 ----
    input  wire        arm,
    input  wire        force_clear,
    input  wire        recover,
    input  wire        abort,
    input  wire        stall,
    input  wire [15:0] yaw_now,
    input  wire        yaw_v,
    input  wire [31:0] remain_in,
    // ★V2.7（清单6）★：本段【实测弧长】（电机 ext_arc_cnt，编码器计数，段内清零）
    input  wire [31:0] step_arc_cnt,
    // ---- 电机外部动作口 ----
    output reg         ext_load,
    output reg  [2:0]  ext_act,
    output reg  [31:0] ext_n,
    output reg  [15:0] ext_yaw_tgt,
    output reg         ext_to_queue,
    output reg         ext_slow,
    input  wire        ext_done,
    input  wire        ext_active,
    // ---- 状态 ----
    output reg         avoid_active,
    output reg         avoid_fail,
    output reg         avoid_done,
    output wire [3:0]  st_code,
    output reg  [7:0]  dbg_why,
    output wire        dbg_side,
    output wire [15:0] dbg_lat,     // ★V2.28a★ 连续输出 lat_acc，回线阶段也能看
    output wire [15:0] dbg_phi,     // ★V2.2：本次算出的角度（度）——给串口读，方便核对★
    output wire [15:0] dbg_fwd_acc, // ★V2.18w★ 临时调试：绕障前进投影（mm）
    output wire [15:0] dbg_g,       // ★V2.18w★ 临时调试：stall/ret_piv/aim/retry/busy
    output wire [7:0]  dbg_flags,   // ★V4.0d★ VFH/侧距/正前/crush 决策标志
    // ★V4.1a★ 影子几何规划器结果（只送串口，不参与控制）
    //   [7:4] = kL 档位（1..12，F=该侧不可行），[3:0] = kR
    output wire [15:0] dbg_shadow
);

    localparam A_FWD      = 3'd0;
    localparam A_YAW_ABS  = 3'd5;
    localparam A_FWD_HOLD = 3'd6;      // ★2b-1 新增：前进 + 保持指定绝对航向★

    localparam [3:0] S_IDLE   = 4'd0,
                     S_CALC   = 4'd1,   // 选边 + 算角度（多拍顺序计算）
                     S_T_OUT  = 4'd2,   // 转到 yaw_base ± φ
                     S_DIAG   = 4'd3,   // 小步斜行（保持航向 + 侧向间隙 + 末端判据）
                     S_T_BACK = 4'd4,   // 转回 yaw_base
                     S_RETURN = 4'd5,   // 斜着把横移走回来
                     S_T_END  = 4'd6,   // 最后一次转回 yaw_base
                     S_RESUME = 4'd7,   // 交回任务剩余里程
                     S_REWAIT = 4'd8,   // ★V2.7★ 本圈没测到障碍段 ⇒ 等下一圈重算
                     S_FAIL   = 4'd9,
                     S_FOLLOW = 4'd10,  // ★V3.0★ Bug2 边界跟随（侧距 PD）
                     S_RET2   = 4'd11,  // ★V3.0★ m-line 回线 PD
                     S_GAP    = 4'd12;  // ★V4.0b★ 72扇区 FGM/VFH 选谷

    // ---- sin/tan 的 5° 档小表（Q10 / Q8）；k = 角度/5 ----
    //   sin 表覆盖 0°..90°（k=0..18）；其它象限用下面的对称/镜像处理，不扩表。
    function [15:0] sin90_q10;                     // sin(k*5°)*1024，k = 0..18
        input [15:0] k;
        begin
            case (k)
                16'd0:  sin90_q10 = 16'd0;      16'd1:  sin90_q10 = 16'd89;
                16'd2:  sin90_q10 = 16'd178;    16'd3:  sin90_q10 = 16'd265;
                16'd4:  sin90_q10 = 16'd350;    16'd5:  sin90_q10 = 16'd433;
                16'd6:  sin90_q10 = 16'd512;    16'd7:  sin90_q10 = 16'd587;
                16'd8:  sin90_q10 = 16'd658;    16'd9:  sin90_q10 = 16'd724;
                16'd10: sin90_q10 = 16'd784;    16'd11: sin90_q10 = 16'd839;
                16'd12: sin90_q10 = 16'd887;    16'd13: sin90_q10 = 16'd928;
                16'd14: sin90_q10 = 16'd962;    16'd15: sin90_q10 = 16'd989;
                16'd16: sin90_q10 = 16'd1008;   16'd17: sin90_q10 = 16'd1020;
                default: sin90_q10 = 16'd1024;  // 90°
            endcase
        end
    endfunction

    function [15:0] cos90_q10;                     // cos(k*5°)*1024（k = 0..18）= sin((18-k)*5°)
        input [15:0] k;
        begin
            cos90_q10 = sin90_q10(16'd18 - k);
        end
    endfunction

    // 方位角 a ∈ [0°,180°] ⇒ sin/cos（用 0..90° 表 + 镜像，避免 37 项大表）
    //   a ≤ 90° : sin=sin90(k)，cos=+cos90(k)
    //   a > 90° : sin=sin90(36-k)，cos=-cos90(36-k)
    function signed [15:0] sinA_q10;
        input [15:0] k;                            // k = a/5 ∈ 0..36
        begin
            sinA_q10 = (k <= 16'd18) ? $signed({1'b0, sin90_q10(k)})
                                     : $signed({1'b0, sin90_q10(16'd36 - k)});
        end
    endfunction

    function signed [15:0] cosA_q10;
        input [15:0] k;
        begin
            cosA_q10 = (k <= 16'd18) ? $signed({1'b0, cos90_q10(k)})
                                     : (-$signed({1'b0, cos90_q10(16'd36 - k)}));
        end
    endfunction

    function [15:0] tan_q8;                        // tan(k*5°) * 256
        input [15:0] k;
        begin
            case (k)
                16'd0:  tan_q8 = 16'd0;      16'd1:  tan_q8 = 16'd22;
                16'd2:  tan_q8 = 16'd45;     16'd3:  tan_q8 = 16'd69;
                16'd4:  tan_q8 = 16'd93;     16'd5:  tan_q8 = 16'd119;
                16'd6:  tan_q8 = 16'd148;    16'd7:  tan_q8 = 16'd179;
                16'd8:  tan_q8 = 16'd215;    16'd9:  tan_q8 = 16'd256;
                16'd10: tan_q8 = 16'd305;    16'd11: tan_q8 = 16'd366;
                default: tan_q8 = 16'd443;    // 60°
            endcase
        end
    endfunction

    // ★V2.30★ 回线走廊长度用的 csc 表：csc(k*5°)*1024，k=3..12（15°..60°）
    function [15:0] csc_q10;
        input [15:0] k;
        begin
            case (k)
                16'd3:  csc_q10 = 16'd3957;   // 15°
                16'd4:  csc_q10 = 16'd2994;   // 20°
                16'd5:  csc_q10 = 16'd2423;   // 25°
                16'd6:  csc_q10 = 16'd2048;   // 30°
                16'd7:  csc_q10 = 16'd1785;   // 35°
                16'd8:  csc_q10 = 16'd1593;   // 40°
                16'd9:  csc_q10 = 16'd1448;   // 45°
                16'd10: csc_q10 = 16'd1337;   // 50°
                16'd11: csc_q10 = 16'd1250;   // 55°
                default:csc_q10 = 16'd1182;   // 60°
            endcase
        end
    endfunction

    // ---- 常数 ----
    localparam [15:0] HALF_W_M   = (CAR_W_MM >> 1) + M_SIDE_MM;   // 半宽 + 侧向余量
    localparam [15:0] HALF_W_CHK = (CAR_W_MM >> 1) + 16'd20;      // 回线走廊检查：真半宽+20mm
    localparam [31:0] R_PIVOT2   = 32'd51529;                     // (207+20)^2 ≈ 227mm 枢轴圆
    localparam [15:0] GAP_BLOCK_MM = 16'd600;                     // 直方图扇区 <600mm 视为障碍
    localparam [7:0]  GAP_MIN_SECT = 8'd8;                        // 最小自由 gap = 8×5° = 40°
    localparam [15:0] AXLE_OFF   = AXLE_FWD_MM - SENS_FWD_MM;     // 后轮轴在雷达【后方】多少
    localparam [31:0] STEP_CNT   = STEP_MM * CNT_PER_MM;
    localparam [31:0] STEP_TO_CYC= (CLK_FREQ_HZ/1000) * STEP_TO_MS;
    localparam [31:0] PIVOT_TO_CYC=(CLK_FREQ_HZ/1000) * PIVOT_TO_MS;
    // ★V4.0d★ 转向卡死：1.5s 内 yaw 还没追上目标 ⇒ 判失败回正，不再原地转 5.5s
    localparam [31:0] YAW_STUCK_CYC=(CLK_FREQ_HZ/1000) * 1500;
    // ★V4.1c★ 判"卡死"改成看窗口内航向有没有真的在动：1.5s 内变化 ≤3° 才算卡死。
    //   19:29 现场：目标 +41.4°，车转到 54° 过冲后正在回收到 41.7°，
    //   旧的"绝对值超时"把它误判成 w=53 直接放弃。
    localparam [15:0] YAW_STUCK_DELTA = 16'd546;   // 3°
    localparam [31:0] RESUME_TO_CYC=(CLK_FREQ_HZ/1000) * RESUME_TO_MS;
    localparam [31:0] TO_CYC     = (CLK_FREQ_HZ/1000) * AVOID_TO_MS;
    // ★V2.28★：车体包络的硬停车线。侧向窗口 <180mm（含轮半宽 110 + 70 余量）
    //   或正前折算距离 <200mm ⇒ 已经不靠规划能救，立即停车。
    // ★V3.0e★ 180→155：目标侧距 200mm 时，170/163 这种擦边不再一碰就停；
    //   155 离雷达 D_MIN=150 仍留 5mm，正前 crush 200 不动。
    localparam [15:0] CRUSH_SIDE_MM  = 16'd155;   // 正侧 0°/180°±20：沿射线碰车体侧边 = 110+余量
    // ★V4.33a★ 车体包络表：斜前 45°/135°±10 单独一档。
    //   沿该射线最先碰到的不是车头角(207mm)，而是车体侧边：a=135° 时 r=110/cos45°=156，
    //   a=125° 时 r=110/0.574=192，a=145° 时 134 ⇒ 门槛应取上端 190（原来和正侧共用 155，
    //   在 125~135° 方向少报约 35mm，车头角会先蹭上去）。
    localparam [15:0] CRUSH_WIDE_MM  = 16'd190;   // 斜前那一档（见上）
    localparam [15:0] CRUSH_FRONT_MM = 16'd200;
    // ★V2.7★
    // ★V4.35i★ 回线残差门 15 → 60：实测横向记账只有真实位移的约 1/5
    //   （15:50 现场 lat 327→30 花了 8.4 秒、实际走了近 3000mm），15mm 永远等不到
    //   ⇒ 车一直斜着走、不回平。放宽到 60 让 S_T_END 早点接手把航向对回 yaw_base。
    //   代价是最终横向残差可能留几十 mm（用户明确"不能准就算了"，平行优先）。
    localparam [15:0] RET_TOL_MM     = 16'd60;   // ★V4.35i★ 回线横向残差门（15 → 60）
    localparam [1:0]  CALC_RETRY_MAX = 2'd3;     // obs_valid=0 最多等几圈（清单5/H1）
    localparam [31:0] HONOR_STALL_CYC= (CLK_FREQ_HZ/1000) * 600; // 段头 600ms 内不认堵转
    // ★V2.9b★：正前"连续几圈看不到障碍段"才算"车已走到它侧面/身后"。
    //   3 圈 ≈ 0.5s@5.5Hz（RPLIDAR A1 实测圈率）。单圈误判（H1 的丢帧/碎段）不算数。
    localparam [1:0]  FRONT_GONE_K   = 2'd3;
    // ★V2.17b★：兜底净空**改成和"回头净空"SIDE_MIN_MM 同一个值**（不再单独写 220）。
    //   原因（用户 00:56 现场日志）：`00:56:25.604 a=4 w=3D` 那次是**兜底先放行**的，
    //   当时障碍侧实测只有 **293mm**（实测路要求 ≥320，正拦着）⇒ 车还没真让开就回正、撞箱子。
    //   两条判据门槛不一致是我留下的坑。现在全场只有**一个**净空门槛（由车体尺寸算出），
    //   兜底与实测路的区别只剩"怎么算已经走过它"（推算横移 vs 先贴近后变远）。
    localparam [15:0] TURN_FALLBACK_MM= SIDE_MIN_MM;
    // ★V2.7（时序）★：原地转的编码器弧长（IMU 掉线兜底）= 每度计数 × 角度。
    //   旧写法 `(N_TURN_90 × 角度) / 32'd90` 是【可变除数除法】⇒ 综合出除法器，
    //   PnR 实测它就是最差路径（u_avoid/srch_k → u_avoid/turn_n：35.6ns / 周期 20ns，
    //   slack -15.6）。改成"先用常量除法算出每度计数、再乘一个小角度（≤3 位宽）"，
    //   数据通路只剩一个小乘法 ⇒ 不引除法器，值只差 1/10000（1129 vs 1128.9）。
    localparam [31:0] TURN_CNT_PER_DEG = (N_TURN_90 + 32'd45) / 32'd90;  // ≈1129 计数/度

    // ---- 状态寄存器 ----
    reg [3:0]  st;
    reg        side;              // 0 = 往左绕，1 = 往右绕
    reg [15:0] yaw_base;
    reg        yaw_ok;
    reg [31:0] remain_at_arm;
    reg [3:0]  calc_st;
    reg [15:0] k0, k1;            // a/5
    reg signed [15:0] c0, c1, s0, s1;    // cos/sin Q10（端点方位可能 >90°，cos 会变负）
    reg signed [31:0] xL, xR;     // 端点横向位置（右为正）
    reg signed [31:0] yL, yR;     // 端点纵向位置（前为正）
    reg [15:0] dxL, dxR;          // 两侧需要的横移量
    reg [15:0] capL, capR;        // 两侧实测可用横移（0 = 不够）
    reg        okL, okR;
    reg [15:0] dx_sel;            // 选中侧的横移量
    reg [15:0] Dm;                // 障碍近面到后轮轴的距离（已扣安全量）
    reg [3:0]  srch_k;            // 递增搜索：角度档 k
    reg [15:0] phi_k;             // 选中的角度档
    reg [15:0] phi_deg;
    // ★V2.30★ 两侧各算一个最小 φ，再按“φ 更小且回线走廊能过”选边
    reg [3:0]  phiL_k, phiR_k;    // 两侧的最小 5° 档；13 = 该侧不可行
    reg        chk_side;          // 走廊检查中的侧
    reg [3:0]  chk_k;             // 走廊检查中的候选档
    reg        other_tried;       // 已经换到另一侧试过
    reg [31:0] chk_L;             // 回线走廊长度
    reg signed [31:0] chk_u, chk_v;
    reg        chk_hit;
    // ★V3.0★ Bug2 边界跟随：侧距 PD / m-line 回线 PD 的临时量
    reg signed [31:0] e_side;
    reg signed [31:0] psi_s;
    reg signed [31:0] tmp_s;
    reg [15:0] side_win;          // ★V3.0★ 障碍侧 0°/180° 窗口的实测侧距
    // ★V3.0★ 符号-幅值：避免 Gowin 把同一加法器同时配成加/减（PA2114）
    reg [15:0] psi_mag, base_mag, mag_l, mag_r, safe_l, safe_r, diff_mag;
    reg [15:0] cur_mag;           // ★V3.0c★ 当前实测航向幅值（5° 档）
    reg [15:0] off_s, off_neg;
    reg        psi_right;
    // ★V4.0b★ S_GAP：72扇区直方图选谷
    reg [3:0]  gap_st;
    reg [6:0]  gap_idx, gap_run_start, gap_best_start;
    reg [7:0]  gap_run_len, gap_best_len, gap_best_dist;
    reg [71:0] gap_blk;
    reg        gap_any_blk, gap_ok, gap_side;
    reg [15:0] gap_phi;
    reg [3:0]  gap_k;
    reg [6:0]  gap_center_c;
    reg [7:0]  gap_dist_c;

    // ================= ★V4.1a★ 影子几何规划器（只算不动）=================
    //   触发那一圈，把 72 扇区里的前半球点换算到【后轮轴车体系】，
    //   对 side∈{左,右} × k=1..12（φ=5k°）逐个做"斜行走廊"矩形扫掠：
    //     障碍点是否落在 |v|<=HALF_W_M 且 u∈[-后悬, 前悬+L] 的带内
    //   取每侧最小的可行 k。结果只送串口，完全不参与 S_T_OUT/S_FOLLOW。
    //   L 由本圈最远障碍点算出，不是写死距离。
    localparam [15:0] SW_HORIZON_MM = 16'd1500;   // 只统计 1.5m 内的点
    localparam [15:0] SW_L_EXTRA_MM = 16'd165;    // 无当前段时的后备：车尾 65 + 余量 100
    localparam [15:0] SW_L_MAX_MM   = 16'd1200;
    localparam [15:0] SW_LOCAL_EXTRA_MM = CAR_L_MM + STEP_MM; // 当前段之外再留整车长+一步
    localparam [15:0] SW_U_BACK_MM  = CAR_L_MM - AXLE_FWD_MM;   // 后悬 65
    reg [2:0]  sw_ph;             // 单点流水相位 0..3
    reg [6:0]  sw_idx;            // 采样序号 0..72（每扇区两个边界角）
    reg [4:0]  sw_cand;           // 候选 0..23
    reg [3:0]  sw_k;              // 当前候选档 1..12
    reg        sw_side;           // 0=往左绕(+φ)，1=往右绕(−φ)
    reg signed [7:0] sw_cur_k_s;  // 当前车体相对基准航向的实测档位（左正右负）
    reg [3:0]  sw_kL, sw_kR;      // 每侧最小可行档；F=不可行
    reg [15:0] sw_ymax;           // 本圈最远前向坐标（后轮轴系）
    reg [15:0] sw_umax;           // 走廊前边界 = 前悬 + L
    reg [15:0] sw_vhalf;          // 走廊半宽（随候选航向变化）
    reg        sw_hit;            // 本候选已命中
    reg        sw_ok;             // 本点读数有效
    reg        sw_any;            // 前半球至少有一个有效点
    reg        sw_done;           // 影子扫掠至少跑过一次（区分"没算过"与"算出 0 档"）
    // ★V4.2f★ S_FOLLOW 每步重算（模型验证规格）
    reg [2:0]  fol_st;            // ★V4.31a★ S_FOLLOW 的四相位：0 判侧距/1 算角/2 算偏移/3 发段
    reg        psip_keep, psip_far; // ★V4.31a★ 贴边 P 控制的两拍中间量
    reg        side_seen;         // ★V4.32a★ 本次绕障中是否已经进过侧面窗
    // ★V4.33c★ 障碍前端与车轴平齐（V4.33e 起 a1_pass/计数器已删除，不再需要）
    reg        a0_pass;
    reg [5:0]  psi_r;             // ★V4.32a★ 拆拍中间量：本步指令角（≤60）
    reg        lat_fix, lat_fix_used; // ★V4.32a★ 回线残差补偿（只补一次）
    // ★V4.2g★ S_RET2 拆两拍：先算 psi（存 phi_deg），下一拍再用寄存值发段，
    //   把 "hist_side → safe → psi → ×182 → pend_yaw" 的长链打断。
    reg        ret_st;
    reg        ret_right, ret_need;
    reg signed [17:0] sw_x, sw_y; // 点在后轮轴车体系的坐标
    reg signed [17:0] sw_u, sw_v; // 点在该候选航向下的纵/横坐标
    reg [31:0] turn_n;            // 转向段的编码器弧长（IMU 掉线兜底）
    reg [31:0] trav_acc;          // 绕障累计里程（mm）
    reg [31:0] fwd_acc;           // 朝原任务方向的前进投影（mm）
    reg [31:0] lat_acc;           // 累计横移（mm，右为正）
    reg [31:0] lat_rem;           // 回线时还剩多少横移要走
    reg [3:0]  avoid_cnt;         // 连续绕障次数（"再绕一次"的限额）
    reg [3:0]  turn_retry;        // 本次转向重发次数
    reg [3:0]  adj_cnt;           // 斜行中"加大角度"的次数
    reg [3:0]  end_retry;         // 续跑前"补转航向"的次数
    reg [3:0]  aim_cnt;           // ★V2.3：斜行中"航向偏了就补转"的次数（独立计数，别和 adj_cnt 抢）★
    // ★V4.2c★ S_FOLLOW 里的航向补转计数（A_FWD_HOLD 在地上只能修掉约 1/6 的误差，
    //   必须由 FSM 自己按 IMU 实测把车拉回目标航向）
    reg [3:0]  hdg_fix_cnt;
    reg        ret_piv;           // ★V2.5：回线阶段"是否已经转到斜行回线航向"★
    reg        ret_yaw_ok;        // ★V2.18x★：回线转身后实测航向是否已到回线目标
    // ★V4.34c★ 基准航向锁存：原来每次 arm 都取"当前航向"，一旦上一轮撞停时车头是歪的
    //   （15:30 现场：车还没动 Y 就已经 −22.8°），下一轮就把歪头当成"原航线"，
    //   导致越绕越歪、怎么转都回不到真正的原航线。改成整段任务只取一次。
    reg [15:0] yaw_home;
    reg        yaw_home_v;
    // ★V4.34a★ 回线段的航向闭环：A_FWD_HOLD 在地上只修得掉约 1/6 误差（V4.2c 记载），
    //   回线目标 −14° 实际只到 −4° ⇒ 横向几乎不收敛（14:37/14:38 两次现场都是这样卡住）
    reg [15:0] ret_tgt;           // 本次回线的目标航向
    reg        ret_corr;          // 当前这一段是"补航向的原地转"，不计平移
    reg [3:0]  ret_fix_cnt;
    // ★V4.35★ 回线受阻计数（见 S_RET2 第二拍）
    reg [3:0]  ret_block_cnt;
    // ★V4.35b★ 滞回：回线受阻被弹回 S_FOLLOW 后，必须等"前面和障碍侧都清空"
    //   才允许再次回线，否则会在 S_FOLLOW↔S_RET2 之间以每步一次的节奏抖动。
    reg        ret_hold;
    reg [2:0]  ret_try;           // ★V2.28★：回线转身重发次数（3 次也要能表示）
    reg [1:0]  calc_retry;        // ★V2.7★ obs_valid=0 已等了几圈（清单5/H1）
    reg [1:0]  front_gone_cnt;    // ★V2.9b★ 正前"看不到障碍段"已连续几圈
    // ★V2.17★：本次绕障期间"障碍侧那一片的实测最近距离 d_obs"的历史最小（每成圈更新）——
    //   用来判"我是不是贴身走过它"（"曾经贴过 → 现在够远"这个过程）。
    reg [15:0] side_min;
    reg        fail_want;         // ★V2.12★ 这次是"失败"在走流程：先转回 yaw_base 再认输
    reg [31:0] wd;                // 单段/单步看门狗
    // ★V4.1c★ 转向"有没有在动"的窗口检测（替代"1.5s 没到就判卡死"）
    reg [31:0] yaw_wd;            // 1.5s 窗口计数
    reg [15:0] yaw_ref;           // 窗口起点航向
    reg [31:0] to_cnt;            // 绕障总时长
    integer    t;                 // k = a/5 的循环变量（只在 S_CALC 第 0 拍用）
    reg        busy;
    reg        load_pend;
    reg [2:0]  pend_act;
    reg [31:0] pend_n;
    reg [15:0] pend_yaw;
    reg        pend_to_q;
    reg        pend_slow;

    assign st_code  = st;
    assign dbg_side = side;
    assign dbg_phi  = phi_deg;
    assign dbg_fwd_acc = fwd_acc[15:0];
    assign dbg_lat  = lat_acc[15:0];

    wire front_block = front_v && (front_mm <= D_ARM_MM);
    wire watch_pass  = (watch_far_cnt >= PASS_K);
    // ★V2.7（H3）★：段头 HONOR_STALL_CYC(600ms) 内不认 stall——
    //   电机侧的"先停稳 200ms 再转"里两轮本来就不转，顶层堵转检测(400ms)会误报。
    //   真堵住还有段看门狗(STEP_TO_CYC=3s)和电机侧 PIVOT_MS/SAFE_MS 兜底。
    wire stall_hit   = stall && (wd >= HONOR_STALL_CYC);
    // ★V2.18x★ 临时调试字：bit15=stall, bit14=stall_hit, bit13=ret_piv,
    //   bit12=busy, bit11..8=aim_cnt, bit7..4=turn_retry,
    //   bit3..2=ret_try, bit1=ret_yaw_ok, bit0=0
    assign dbg_g = {stall, stall_hit, ret_piv, busy, aim_cnt, turn_retry,
                    ret_try[1:0], ret_yaw_ok, 1'b0};

    // 障碍侧窗口：往右绕(side=1) 时障碍最后落在【车正左】⇒ 盯 0°；往左绕盯 180°
    //   （lidar_parse 里 watch_ang 的 bit7 = 1 ⇒ 180°(车正右) 累加器，= 0 ⇒ 0°(车正左)）
    wire [7:0] wang = side ? 8'd0 : 8'd180;

    // ★V2.2：|实际航向 − yaw_base|（按 16 位角取最短，用于"回读校验"）★
    wire [15:0] yb_d    = yaw_now - yaw_base;
    wire [15:0] yb_mag  = yb_d[15] ? (16'd0 - yb_d) : yb_d;
    wire [15:0] yb_err  = (yb_mag > 16'd16384) ? (16'd32768 - yb_mag) : yb_mag;
    // 出航向的目标航向（S_T_OUT / 斜行 / 回线都用它）
    wire [15:0] yaw_out_tgt = side ? (yaw_base - (phi_deg * 16'd182))
                                   : (yaw_base + (phi_deg * 16'd182));
    // ★V2.18x★：回线阶段目标航向（从斜行 +φ 直接转到 −φ）
    wire [15:0] ret_yaw_tgt = side ? (yaw_base + (phi_deg * 16'd182))
                                   : (yaw_base - (phi_deg * 16'd182));

    // ★V2.3：把【IMU 实测航向】换算成"5° 档索引"，用于按实测航向记账横移★
    //   实测偏移（相对 yaw_base，取幅值）⇒ 度数 ⇒ 档位（0..12）
    wire [15:0] act_raw = yaw_now - yaw_base;
    wire [15:0] act_mag = act_raw[15] ? (16'd0 - act_raw) : act_raw;
    wire [31:0] act_deg = ({16'd0, act_mag} * 32'd45) >> 13;      // 计数→度（≈360/65536）
    wire [31:0] act_k32 = (act_deg * 32'd205) >> 10;              // ≈ 度/5
    wire [15:0] act_k   = (act_k32 > 32'd12) ? 16'd12 : act_k32[15:0];
    // 点云来自【当前车体坐标系】，规划候选角相对 yaw_base；
    // S_FOLLOW 扫掠时必须用“目标绝对档位 − 当前实测档位”。
    wire signed [7:0] act_k_s8 = act_raw[15] ? (-$signed({4'd0, act_k[3:0]}))
                                             :  $signed({4'd0, act_k[3:0]});

    // ★V2.5（问题2）★：横移需求必须有物理下限——"绕过障碍"至少要挪开一个含轮半宽+余量
    //   （否则 dx 被算成 0 时，收工判据 lat_acc≥dx 立刻满足 ⇒ 转两步就"绕完"）
    wire [15:0] dx_use = (dx_sel < HALF_W_M) ? HALF_W_M : dx_sel;
    // ★V2.8（清单9：选边判死）★：选边的"能不能过去"也必须用【车实际要挪的量】。
    //   旧写法用 dxL/dxR 去比 capL/capR，而车实际至少挪 HALF_W_M：
    //   箱子几乎整段落在车中线左边（x_R<0）时 dxR<HALF_W_M ⇒ 判据说"能过"、
    //   车却要挪更多 ⇒ 只能贴着箱子和旁边那把椅子过去（现场"右轮蹭箱"）。
    //   这两个量同时用于"可行性判定"、"取更省的一侧"和 dx_sel/rhs256。
    // ★V4.33a★ 回线角律用 Stanley 形式：atan(横向残差 / 前视距离)
    //   前视距离 = 出航阶段已经攒下的纵向前进 fwd_acc（下限 300，上限钳 16 位）
    wire [15:0] fwd16 = (fwd_acc[31:16] != 16'd0) ? 16'hFFFF : fwd_acc[15:0];
    wire [15:0] L_look16 = (fwd16 < 16'd300) ? 16'd300 : fwd16;
    // 用移位加法代替乘常数（10 = 8+2，9 = 8+1）：这两个量在 S_RET2 的关键路径上，
    // 见 PnR 里 base_mag→mag→psi→off_s→pend_yaw 这条链，不能再放乘法器
    wire [31:0] lat_m10  = ({16'd0, lat_acc[15:0]} << 3) + ({16'd0, lat_acc[15:0]} << 1);
    wire [31:0] look_m9  = ({16'd0, L_look16} << 3) + {16'd0, L_look16};
    wire [15:0] dxL_use = (dxL < HALF_W_M) ? HALF_W_M : dxL;
    wire [15:0] dxR_use = (dxR < HALF_W_M) ? HALF_W_M : dxR;

    // ★V2.7（清单1+2）★：收工判据改看"障碍侧实测净空"——三路取更紧的那个。
    //   三路读数口径相同：雷达中心(≈车纵向中线) → 障碍的垂距。
    //     ① 障碍侧 0°/180° 窗口：贴着障碍走时那个"侧向间隙"（V2.5 换掉了它 ⇒ 加回来；
    //        箱子贴到车身侧面后只剩它看得见，这正是用户说的"边走边测"）
    //     ② 障碍侧正前半区 FL/FR：箱子还在正前偏左/偏右时它才看得见
    //     ③ obs_dmin：本圈障碍段（正前 ±45°）内的最近点
    //   障碍侧：side=0 往左绕 ⇒ 障碍留在车【正右】⇒ 用 right/FR；side=1 反之。
    //   三路都是 FFFF（该方向没点 = 开阔）时 gap_obs=FFFF ⇒ 不拦着收工（与旧口径一致）。
    wire [15:0] gap_side_win  = side ? left_mm    : right_mm;
    wire [15:0] gap_side_half = side ? front_l_mm : front_r_mm;
    wire [15:0] gap_a   = (gap_side_win < gap_side_half) ? gap_side_win : gap_side_half;
    wire [15:0] gap_obs = (gap_a < obs_dmin) ? gap_a : obs_dmin;

    // ★V2.9b★：回头判据的两条实测条件（用户："走到障碍物侧面的时候看侧面离障碍物多远
    //   来判断车到底能回头了没有"；"不能写死角度"）：
    //     ① front_gone：正前 ±45° 扇区连续 FRONT_GONE_K 圈都没有障碍段
    //        ⇒ 用【雷达正前的实测】证明"车已经走到它侧面/身后"；
    //     ② gap_side_win：障碍侧那一格 0°/180° 窗口的实测垂距（就是"侧面离障碍物多远"）。
    //   两条都不含任何写死的角度、也不含里程推算。
    wire front_gone = (front_gone_cnt >= FRONT_GONE_K);
    // ★V2.9c★：回头判据改成【两条实测证据取"或"】，谁先成立谁能触发——
    //   A. 障碍侧"先贴过再变远"：本次斜行期间障碍侧窗口读到过 ≤ SIDE_MIN_MM，
    //      现在又 ≥ SIDE_MIN_MM ⇒ 车确实贴着它走过、现在离它够远
    //      （用户原话："走到障碍物侧面的时候看侧面离障碍物多远来判断车到底能回头了没有"）。
    //   B. 正前连续 3 圈看不到障碍段（front_gone）⇒ 车已经走到它侧面/身后。
    //   ⚠️ 教训：V2.9b 只写了 B，结果把"正前那片一定会消失"当成事实——那只是 tb 里
    //   我自造的激励，现场不成立时就永远不满足 ⇒ 车沿侧面一直开。所以现在两路取"或"。
    //   两条都仍然要求"当前侧距 ≥ 回头扫掠净空"，避免回头时车体扫到它。
    // ★V2.11★：**三条取"或"**——前两条是"更早回头"的实测路，第三条是"一定会回头"的兜底：
    //   A/B（实测，优先）：当前侧距 ≥ 回头扫掠净空(300) 且（贴过再变远 或 正前 3 圈无段）；
    //   C（兜底，= V2.7/V2.8 旧判据）：推算横移 ≥ dx_use 且 gap_obs ≥ 220 且 前进投影 ≥ ADV_MIN。
    //   教训（用户 23:15 现场日志）：只有 A/B 时，`R=047B`(车正右 1147mm，永远 ≥300 ⇒ 贴过
    //   这条路永不置位) + `E=520283650…`(正前 82~101° 一直有段 ⇒ 正前这条路也不成立)
    //   ⇒ 两条同时死 ⇒ 车永远不回头。所以 C 必须留着。
    // ★V2.15★：**障碍侧实测最近距离 = 三格取最小**（往左绕 = 车正右那一带）：
    //   车右前 90~110°(front_r) + 车右偏前 125~145°(wide_r) + 车正右 170~190°(right)
    //   ⇒ 消掉"车斜行时箱子落在 135°~170° 空档、两格都看不见"的老漏洞
    //   （00:17 日志里读数在 316/1886/2358/无点 之间乱跳就是它造成的）。
    //   没有点的那格是 FFFF，天然不参与取最小。
    wire [15:0] d_obs_r = (front_r_mm < wide_r_mm) ? front_r_mm : wide_r_mm;
    wire [15:0] d_obs_l = (front_l_mm < wide_l_mm) ? front_l_mm : wide_l_mm;
    wire [15:0] d_obs   = side ? ((d_obs_l < left_mm)  ? d_obs_l : left_mm)
                               : ((d_obs_r < right_mm) ? d_obs_r : right_mm);
    // ★V2.15★：回头判据 = **实时距离**当主判据（用户三条：实时检测 / 不写死角度距离 / 检测到就反应）：
    // ★V2.17★：回头 = 【跨过门槛】的两条实测——都用"障碍侧**那一片**的实测最近距离" d_obs：
    //   ① 现在：`d_obs ≥ SIDE_MIN_MM`（回头时车体扫不到它；SIDE_MIN 由车体尺寸算出，不是常数）；
    //   ② 过程：本次绕障期间**曾经** `d_obs ≤ SIDE_MIN_MM`（说明确实贴身走过它）。
    //   为什么用"曾经贴过 → 现在够远"：这是"先近后远"这个**过程**，一进门时②不成立（那时它还在正前、
    //   读数大于净空），所以不会一进门就回头；而"窗口/段换对象"也不会让它成立（换对象只会让读数变大，
    //   ② 需要的是**先小**）。
    //   注意：②靠的是"曾经读到过贴近的值"，所以障碍侧的格子必须没有缝——见 V2.17 把 `HALF_S_Q6`
    //   从 ±10° 放宽到 ±20° 的那条（否则箱子落在 110~125°/145~170° 缝里时读不到）。
    wire side_was_close = (side_min <= SIDE_MIN_MM);
    wire passed_ok  = side_was_close;
    // ★V2.18s★：侧窗“变空/变远”不能单独放行回头。
    //   23:45 现场：只横移 99mm 时箱子滑出侧窗（R=FFFF、FR=1111），
    //   旧判据误判“已绕过”，回线只补 99mm，车最后平行停在障碍旁边。
    //   ⇒ 实测窗口还必须配上“车真的横移够了”的门槛。
    // ★V2.18w★：实测侧窗放行也必须已经前进够，防止车头刚过、车尾还没过
    //   就做 2φ 回线转身，转的时候扫到箱子。
    wire [31:0] fwd_gate = {16'd0, Dm} + {16'd0, OBST_DEPTH_GUARD_MM};
    wire        fwd_clear = (fwd_acc >= ADV_MIN_MM) && (fwd_acc >= fwd_gate);
    wire turn_now_ok = (d_obs >= SIDE_MIN_MM) && passed_ok &&
                       (lat_acc >= {16'd0, dx_use}) && fwd_clear;
    wire turn_lat_ok = (lat_acc >= {16'd0, dx_use}) && (gap_obs >= TURN_FALLBACK_MM) &&
                       fwd_clear;
    wire turn_back_ok = turn_now_ok || turn_lat_ok;

    // ★V4.1d★ S_FOLLOW 的离开门槛改成【几何量】：后轮轴越过障碍近面(Dm)
    //   再走掉后悬 65 + 余量 20，才算整车过完。
    //   19:47 现场：触发时 Dm≈498，旧判据固定 360 ⇒ F=389 就回线，
    //   车头前还有 496mm 就转身，回线腿正中箱子（df 1675→162，w=52）。
    wire [31:0] fwd_need = (Dm == 16'd0) ? {16'd0, ADV_MIN_MM}
                                         // ★V4.34f★ 撤销 V4.34d 的"提前 185mm"：17:31 现场证明
                                         //   提前后 w=33 出现在 lat=200、箱子还没过去，回线右转时
                                         //   车头右前角正好扫到箱子（连续三次撞角）。宁可晚，不能撞。
                                         : ({16'd0, Dm} + 32'd85);

    // ★V2.28★：障碍段“近面”的前向投影。obs_dmin 是斜距，直接拿它算 Dm
    //   会在斜前方障碍上把余量算大（φ 算小）⇒ 偶尔让不开、擦到箱子。
    //   yL/yR 是雷达系下的前向坐标，取较小者 + 后轮轴偏移，才是后轮轴到近面的距离。
    wire signed [31:0] y_near = (yL < yR) ? yL : yR;
    wire signed [31:0] dm_proj = y_near + $signed({16'd0, AXLE_OFF}) -
                                 $signed({16'd0, D_SAFE_MM});

    // ★V2.28★：连续车体包络。绕障/回线/回正期间不等到“小步结束”才看，
    //   侧窗或正前已经进入车体安全包络就立即停车；否则侧擦时轮子还在转，
    //   stall_cnt 根本抓不到。
    // ★V4.0d★ 侧向包络改用直方图侧距（0/180±20），不再用单窗口
    wire [15:0] near_side = (hist_side_l_mm < hist_side_r_mm) ? hist_side_l_mm : hist_side_r_mm;
    wire [15:0] near_wide = (wide_l_mm < wide_r_mm) ? wide_l_mm : wide_r_mm;
    wire crush_side  = ((near_side != 16'hFFFF) && (near_side < CRUSH_SIDE_MM)) ||
                       ((near_wide != 16'hFFFF) && (near_wide < CRUSH_WIDE_MM));
    // ★V4.35b★ 障碍侧的【斜前窗】距离（45°/135°±10）＝ 车头角方向。
    //   ★关键修正★：原来"我离障碍多远"只看【正侧窗 160~200°】，可障碍经常停在
    //   125~145°（斜前窗）—— 于是 ①贴边 P 控制看不见它、不加角 ②side_seen 不置位、
    //   收角门控被永久锁死。现在把两个窗口取小，统一成一个"侧距"。
    wire [15:0] side_wide_mm = side ? wide_l_mm : wide_r_mm;
    // ★V4.35e★ side_prox 必须做成【寄存器】：它要进 base_mag 的"减 HALF_W → 乘 102 → 移位"
    //   那条链，再串一级 min 的话 PnR 最差路径只剩 +0.011ns（V4.35c 实测）。
    //   改成在 S_FOLLOW 的 !busy 里先算好，min 只吃一拍。
    reg  [15:0] side_prox_r;
    // ★V4.35c★ "进过侧窗"用 ≤600（与原 side_win 口径一致，只把窗口从"正侧"扩到"正侧∪斜前"）；
    //   ≤300 是"很近"，是另一回事，别混用（混了会让收角门控过早锁死）。
    wire        side_prox_seen = (side_prox_r != 16'hFFFF) && (side_prox_r <= 16'd600);
    wire        side_prox_near = (side_prox_r != 16'hFFFF) && (side_prox_r <= 16'd300);
    // ★V4.35c★ "车头角很近"判据：直接用统一的 side_prox_r（正侧窗 ∪ 障碍侧斜前窗）。
    //   回线是往障碍那一侧转的 ⇒ 障碍侧斜前窗就是车头角方向。
    //   原来按 ret_right 选窗，而 ret_right 可能被安全项翻向，会选错侧。
    wire        ret_corner_near = (side_prox_r != 16'hFFFF) && (side_prox_r < 16'd260);
    wire crush_front = front_v && (front_mm < CRUSH_FRONT_MM);
    wire crush_stop  = avoid_active && (st != S_IDLE) && (st != S_FAIL) &&
                       (crush_side || crush_front);
    assign dbg_flags = {gap_ok, side,
                        (side_win == 16'hFFFF),
                        (side_win >= 16'd300),
                        (front_v && (front_mm > (D_ARM_MM + 16'd50))),
                        (hist_side_l_mm == 16'hFFFF),
                        (hist_side_r_mm == 16'hFFFF),
                        crush_stop};

    // ---------------- ★V4.1a★ 影子几何规划器的组合部分 ----------------
    // ★V4.1e★ 还没跑过影子时打 00EE（不再用 00FF，免得和"两侧都不可行"混）
    assign dbg_shadow = sw_done ? {8'd0, sw_kL, sw_kR} : 16'h00EE;

    // ★V4.3a★ 候选绝对航向先用有符号档位表示（左正右负），再减当前实测档位。
    //   S_GAP 时当前档位为 0，等价于原绝对角；S_FOLLOW 时点云已随车体一起转，
    //   只能旋转“尚未转完的剩余角”，不能再拿相对 yaw_base 的整角重复旋转一次。
    wire signed [7:0] sw_cand_k_s = sw_side ? (-$signed({4'd0, sw_k}))
                                            :  $signed({4'd0, sw_k});
    wire signed [7:0] sw_rel_k_s  = sw_cand_k_s - sw_cur_k_s;
    wire [7:0] sw_rel_k_abs = sw_rel_k_s[7] ? (-sw_rel_k_s) : sw_rel_k_s;
    wire [3:0] sw_rel_k = (sw_rel_k_abs > 8'd12) ? 4'd12 : sw_rel_k_abs[3:0];
    wire       sw_rel_side = sw_rel_k_s[7];
    wire [15:0] sw_kx16  = {12'd0, sw_rel_k};
    wire signed [15:0] sw_sin_m = $signed({1'b0, sin90_q10(sw_kx16)});
    wire signed [15:0] sw_cos_m = $signed({1'b0, cos90_q10(sw_kx16)});
    wire signed [15:0] sw_sin = sw_rel_side ? sw_sin_m : (-sw_sin_m);
    wire signed [15:0] sw_nsin = sw_rel_side ? (-sw_sin_m) : sw_sin_m;

    // ★V4.1b★ 每扇区取【两个边界角】各一个点：把"扇区最小距离"当成扇区中心一个点，
    //   会把窄障碍的角向宽度吃掉（100mm 箱在 600mm 处只占 2 个扇区）。
    wire [5:0] sw_sec = sw_idx[6:1];                    // 扇区 0..36
    wire [5:0] sw_ang = sw_sec + {5'd0, sw_idx[0]};     // 角度档 0..36（0..180°）
    wire signed [15:0] sw_sa = sinA_q10({10'd0, sw_ang});
    wire signed [15:0] sw_ca = cosA_q10({10'd0, sw_ang});

    // ★V4.3a★ 候选航向确定后，车身已经对齐该航向；直线扫掠半宽就是
    //   “含轮半宽 + 侧向余量”，不能再把前悬 175·sinψ 加到整条走廊上。
    //   起点原地转造成的额外扫掠由下面的 R_PIVOT2 独立检查。
    wire [15:0] sw_vhalf_n = HALF_W_M;

    // ★V4.1b★ 5° 扇区 ⇒ 角度不确定度 ±2.5°，选完再加一档余量
    wire [3:0] sw_k_bias = (sw_k >= 4'd12) ? 4'd12 : (sw_k + 4'd1);
    wire [3:0] sw_best_k    = (sw_kL <= sw_kR) ? sw_kL : sw_kR;
    wire       sw_best_side = (sw_kL <= sw_kR) ? 1'b0 : 1'b1;
    wire [15:0] sw_best_phi = {12'd0, sw_best_k} * 16'd5;

    wire [15:0] sw_r = hist_rd_data;
    // FFFF=无回波、<150mm=雷达盲区，都不参与扫掠（盲区问题留给 V4.1e）
    wire sw_rok = (sw_r != 16'hFFFF) && (sw_r >= 16'd150) && (sw_r < SW_HORIZON_MM);

    // x(右正) = -r·cosθ，y(前正) = r·sinθ；再换算到后轮轴（雷达在其前 25mm）
    wire signed [31:0] sw_xc = -(($signed({16'd0, sw_r}) * sw_ca) >>> 10);
    wire signed [31:0] sw_yc =  (($signed({16'd0, sw_r}) * sw_sa) >>> 10);
    wire signed [31:0] sw_yax = sw_yc + 32'sd25;

    // 点在该候选航向车体系下的前向 u / 右向 v
    //   u = x·sinφ + y·cosφ
    //   v = x·cosφ − y·sinφ = x·cosφ + y·(−sinφ)   ← 两个乘法相加，不再用减法
    //   x,y 已钳在 ±1500 内 ⇒ 用 16 位有符号，乘积正好 32 位，和 33 位，不丢位
    wire signed [15:0] sw_x16 = sw_x[15:0];
    wire signed [15:0] sw_y16 = sw_y[15:0];
    wire signed [31:0] sw_pu1 = sw_x16 * sw_sin;
    wire signed [31:0] sw_pu2 = sw_y16 * sw_cos_m;
    wire signed [31:0] sw_pv1 = sw_x16 * sw_cos_m;
    wire signed [31:0] sw_pv2 = sw_y16 * sw_nsin;
    wire signed [32:0] sw_uc = (sw_pu1 + sw_pu2) >>> 10;
    wire signed [32:0] sw_vc = (sw_pv1 + sw_pv2) >>> 10;

    wire signed [17:0] sw_vhalfs = $signed({2'b00, sw_vhalf});
    wire signed [17:0] sw_uback = -$signed({2'b00, SW_U_BACK_MM});
    wire signed [17:0] sw_umaxs =  $signed({2'b00, sw_umax});

    wire sw_in_v = (sw_v <= sw_vhalfs) && (sw_v >= -sw_vhalfs);
    wire sw_in_u = (sw_u >= sw_uback) && (sw_u <= sw_umaxs);
    wire signed [35:0] sw_x2 = sw_x * sw_x;
    wire signed [35:0] sw_y2 = sw_y * sw_y;
    wire [36:0] sw_rad2 = {1'b0, sw_x2} + {1'b0, sw_y2};
    wire sw_pivot_hit = (sw_rad2 <= {5'd0, R_PIVOT2});
    wire sw_coll = sw_ok && ((sw_in_v && sw_in_u) || sw_pivot_hit);

    // ★V4.3a★ 初始扫掠的走廊终点由当前障碍段最近距离推出来；
    //   每个跟随小步也用最新 obs_dmin 重算。找不到当前段时才回退到整圈最远点。
    wire sw_near_valid = obs_valid && (obs_dmin != 16'hFFFF);
    wire [31:0] sw_local_l32 = {16'd0, obs_dmin} + {16'd0, SW_LOCAL_EXTRA_MM};
    wire [15:0] sw_local_l16 = (sw_local_l32 > {16'd0, SW_L_MAX_MM})
                               ? SW_L_MAX_MM : sw_local_l32[15:0];
    wire [15:0] sw_umax_local = AXLE_FWD_MM + sw_local_l16;

    // ★V2.7（清单6）★：本段实测弧长（编码器计数）→ mm。
    //   1/574 = 0.0017422 用 Q20 常数乘（1827/1048576），不做除法、也不引除法器；
    //   误差 0.02%（80mm 档读出 80.01mm）。乘法结果在 32 位内（1e6 计数×1827 = 1.83e9）。
    wire [31:0] arc_mm_c = (step_arc_cnt * 32'd1827) >> 20;
    // ★V3.0e★：避障斜行的前进投影 fwd_acc 不再从任务剩余里扣。
    //   日志 1.0 明确“避障这段里程不计入任务”，扣 fwd_acc 会让 resume_n 提前变 0，
    //   车停在障碍物正侧面却 avoid_done=1（用户现场“假完成”）。
    wire [31:0] resume_n = remain_at_arm;
    // ★V2.7（时序）★：打一拍再用——本步的弧长在整个小步里几乎不变（125mm/s 时 20ns 只走 0.0025mm），
    //   打拍只为把"乘法+移位"从后续的"sin/cos 乘法 + 32 位累加"长链里切出来。
    reg  [31:0] arc_mm;              // 打拍后的【本步实测弧长(mm)】，下面记账统一用它
    // ★V4.2i★ 时序：S_FOLLOW/S_RET2 的 ext_done 分支里，(arc_mm*sin)>>10
    //   在同一个比较里被算两次 ⇒ 一拍 4 个乘法。提成公共项，只算一次 sin/一次 cos。
    wire signed [47:0] arc_sin_w = (arc_mm * sin90_q10(act_k_r)) >>> 10;
    wire signed [47:0] arc_cos_w = (arc_mm * cos90_q10(act_k_r)) >>> 10;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) arc_mm <= 32'd0;
        else        arc_mm <= arc_mm_c;
    end
    // ★V2.7（时序）★：把 act_k（实测航向→5° 档索引）也打一拍再用。
    //   它的换算是"两次乘法 + 两次移位 + 钳位"，后面又接 sin/cos 表、×弧长、32 位累加，
    //   全挤在一拍里 PnR 报 23.8ns；IMU 帧率是 10ms 级，晚 20ns 取档位没有任何影响。
    reg [15:0] act_k_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) act_k_r <= 16'd0;
        else        act_k_r <= act_k;
    end

    // ★V4.34b★ IMU 存活标志：yaw_v 是单拍脉冲，不能直接当电平用（V2.20 的老坑，
    //   arm 那一拍采样又把坑挖回来了）。收到任意一帧就置 1，50ms 收不到就置 0。
    reg [21:0] imu_to;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imu_to <= 22'd0; yaw_ok <= 1'b0;
        end else if (yaw_v) begin
            imu_to <= 22'd0;
            yaw_ok <= 1'b1;
        end else if (imu_to < 22'd2_500_000) begin
            imu_to <= imu_to + 22'd1;
        end else begin
            yaw_ok <= 1'b0;                 // 50ms 没帧 ⇒ IMU 掉线
        end
    end

    // ★V2.2：两个 16 位角之间的最短差（用于"转向后回读校验"）★
    function [15:0] yaw_diff;
        input [15:0] a, b;
        reg [15:0] d, m;
        begin
            d = a - b;
            m = d[15] ? (16'd0 - d) : d;
            yaw_diff = (m > 16'd16384) ? (16'd32768 - m) : m;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ext_load <= 1'b0; ext_act <= A_FWD; ext_n <= 32'd0; ext_yaw_tgt <= 16'd0;
            ext_to_queue <= 1'b0; ext_slow <= 1'b0;
            load_pend <= 1'b0; pend_act <= A_FWD; pend_n <= 32'd0;
            pend_yaw <= 16'd0; pend_to_q <= 1'b0; pend_slow <= 1'b1;
            // ★V4.34b★ 这里不再赋 yaw_ok：它由独立的 IMU 存活 always 驱动，
            //   两处都写会变成多驱动（Gowin EX2000）。
            st <= S_IDLE; side <= 1'b1; yaw_base <= 16'd0;
            remain_at_arm <= 32'd0; calc_st <= 4'd0;
            k0 <= 16'd0; k1 <= 16'd0; c0 <= 16'd0; c1 <= 16'd0; s0 <= 16'd0; s1 <= 16'd0;
            xL <= 32'sd0; xR <= 32'sd0; yL <= 32'sd0; yR <= 32'sd0;
            dxL <= 16'd0; dxR <= 16'd0; capL <= 16'd0; capR <= 16'd0;
            okL <= 1'b0; okR <= 1'b0; dx_sel <= 16'd0; Dm <= 16'd0;
            srch_k <= 4'd3; phi_k <= 16'd0; phi_deg <= 16'd0; turn_n <= 32'd0;
            phiL_k <= 4'd13; phiR_k <= 4'd13;
            chk_side <= 1'b0; chk_k <= 4'd3; other_tried <= 1'b0;
            chk_L <= 32'd0; chk_u <= 32'sd0; chk_v <= 32'sd0; chk_hit <= 1'b0;
            e_side <= 32'sd0; psi_s <= 32'sd0; tmp_s <= 32'sd0; side_win <= 16'hFFFF;
            psi_mag <= 16'd0; base_mag <= 16'd0; mag_l <= 16'd0; mag_r <= 16'd0;
            safe_l <= 16'd0; safe_r <= 16'd0; diff_mag <= 16'd0; cur_mag <= 16'd0;
            off_s <= 16'd0; off_neg <= 16'd0; psi_right <= 1'b0;
            side_seen <= 1'b0; lat_fix <= 1'b0; lat_fix_used <= 1'b0; psi_r <= 6'd0;
            psip_keep <= 1'b0; psip_far <= 1'b0; side_prox_r <= 16'hFFFF;
            // ★V4.35f★ 补齐"上电为 X"的寄存器：这些原先只在"进入对应状态"时被初始化，
            //   上电时是 X（仿真里会传播，实机是随机值）。补上复位，纯增赋值、不改逻辑。
            fol_st       <= 3'd0;  gap_center_c <= 7'd0;  gap_dist_c <= 8'd0;
            hdg_fix_cnt  <= 4'd0;  ret_right      <= 1'b0;  ret_st        <= 1'b0;
            sw_cand      <= 5'd0;  sw_hit         <= 1'b0;  sw_idx        <= 7'd0;
            sw_k         <= 4'd0;  sw_ok          <= 1'b0;  sw_ph         <= 3'd0;
            sw_side      <= 1'b0;  sw_ymax        <= 16'd0;
            yaw_ref      <= 16'd0; yaw_wd         <= 32'd0;   // ★V4.33d★ 补复位
            ret_tgt <= 16'd0; ret_corr <= 1'b0; ret_fix_cnt <= 4'd0;
            yaw_home <= 16'd0; yaw_home_v <= 1'b0;
            ret_block_cnt <= 4'd0; ret_hold <= 1'b0;
            a0_pass <= 1'b0;
            gap_st <= 4'd0; gap_idx <= 7'd0; gap_run_start <= 7'd0; gap_best_start <= 7'd0;
    // ★V4.1c★ 复位后 G=00FF 表示"影子还没算过"，与"算出来 0 档"区分开
            sw_kL <= 4'd15; sw_kR <= 4'd15; sw_any <= 1'b0; sw_vhalf <= HALF_W_M;
            sw_cur_k_s <= 8'sd0; sw_umax <= AXLE_FWD_MM + SW_L_MAX_MM;
    sw_done <= 1'b0;
            gap_run_len <= 8'd0; gap_best_len <= 8'd0; gap_best_dist <= 8'd255;
            gap_blk <= 72'd0; gap_any_blk <= 1'b0; gap_ok <= 1'b0; gap_side <= 1'b0;
            gap_phi <= 16'd0; gap_k <= 4'd0; hist_rd_addr <= 7'd0;
            trav_acc <= 32'd0; fwd_acc <= 32'd0; lat_acc <= 32'sd0; lat_rem <= 32'sd0;
            avoid_cnt <= 4'd0; wd <= 32'd0; to_cnt <= 32'd0; busy <= 1'b0;
            turn_retry <= 4'd0; adj_cnt <= 4'd0; end_retry <= 4'd0; aim_cnt <= 4'd0;
            ret_piv <= 1'b0;
            ret_yaw_ok <= 1'b0; ret_try <= 2'd0;
            calc_retry <= 2'd0;
            front_gone_cnt <= 2'd0;
            side_min       <= 16'hFFFF;    // ★V2.17★ 障碍侧实测最近距离的历史最小（上电先给"看不到"）
            fail_want      <= 1'b0;
            avoid_active <= 1'b0; avoid_fail <= 1'b0; avoid_done <= 1'b0;
            watch_en <= 1'b0; watch_ang <= 8'd90; dbg_why <= 8'h00;
        end else begin
            avoid_done <= 1'b0;
            ext_load   <= 1'b0;
            watch_en   <= 1'b0;
            watch_ang  <= wang;

            // ---- ext 握手：pend 就绪且电机空闲 ⇒ 发一次 load ----
            if (load_pend && !ext_active) begin
                ext_load     <= 1'b1;
                ext_act      <= pend_act;
                ext_n        <= pend_n;
                ext_yaw_tgt  <= pend_yaw;
                ext_to_queue <= pend_to_q;
                ext_slow     <= pend_slow;
                load_pend    <= 1'b0;
            end

            // ★V2.9b★：每成圈更新一次"正前看不到障碍段"的连续圈数（只在绕障期间用它）
            //   obs_valid 是 lidar_parse 每圈锁存的"正前 ±45° 扇区里有没有段"⇒ 全实测。
            if (scan_done) begin
                if (!obs_valid)
                    front_gone_cnt <= (front_gone_cnt >= FRONT_GONE_K) ?
                                      FRONT_GONE_K : (front_gone_cnt + 2'd1);
                else
                    front_gone_cnt <= 2'd0;
                // ★V4.33c★ 里程碑改用【侧窗实测】，不再用 obs_a0/a1：
                //   obs_* 的段被 lidar_parse 的 in_half 限在 45°~135°（A_LO_Q6/A_HI_Q6），
                //   方位永远到不了 165°/15° ⇒ 用 obs_a0 判"平齐"永远是 0，
                //   收角门控会被永久锁死（V4.33a 的致命 bug，台架因直接驱动 obs_* 而漏掉）。
                //   改用：进过侧窗 = 车轴已到障碍前端附近；侧窗又变远 = 已过后端。
                // ★V2.15b★：每成圈更新"**障碍段自己**最近点"的历史最小（边走边测的"过程"记忆）。
                //   只记 <4000 的有效读数（>4000 视为"那个方向没东西"，不参与）。
                // ★V2.17★：每成圈更新"障碍侧那一片"最近距离的历史最小（只记 <4000 的有效读数）
                // ★V2.18★：**只在已经开始沿侧面走（S_DIAG）之后才记**——否则"进门那一圈/原地转身期间"
                //   的旧读数（比如旁边那把椅子 250mm）会把"曾经贴过"提前成立 ⇒ 回头会偏早。
                if (avoid_active && (st == S_DIAG) && (d_obs < 16'd4000) && (d_obs < side_min))
                    side_min <= d_obs;
            end

            if (force_clear || abort) begin
                avoid_fail   <= 1'b0;
                avoid_active <= 1'b0;
                load_pend    <= 1'b0;
                busy         <= 1'b0;
                ret_yaw_ok   <= 1'b0; ret_try <= 2'd0;
                avoid_cnt    <= 4'd0;
                yaw_home_v   <= 1'b0;   // ★V4.34c★ 新任务/清障 ⇒ 基准重新取
                st           <= S_IDLE;
                dbg_why      <= force_clear ? 8'h01 : 8'h02;
            end else if (recover) begin
                avoid_fail   <= 1'b0;
                avoid_active <= 1'b0;
                load_pend    <= 1'b0;
                busy         <= 1'b0;
                ret_yaw_ok   <= 1'b0; ret_try <= 2'd0;
                // ★V4.2a★ 清单第3项：恢复时把绕障计数清零。
                //   旧版只在"绕障成功"时清，失败两次后 avoid_cnt>RETRY_MAX ⇒
                //   之后每次触发都直接 w=24 进 S_FAIL，车彻底不动（21:18 第三次现场）。
                avoid_cnt    <= 4'd0;
                // ★V4.32a★ 回线残差补偿：绕障失败/碰撞恢复时若横移还没走回来，
                //   先把回线补完再交任务。旧版 recover 直接回 S_IDLE，lat_acc/lat_rem
                //   谁都不碰 ⇒ 330mm 横移被永久留着，任务跑到结束车还是歪的（02:36 现场）。
                //   只补一次（lat_fix_used），避免 crush→recover→crush 死循环。
                if (!lat_fix_used && (lat_acc > RET_TOL_MM) &&
                    front_v && (front_mm >= (D_ARM_MM + 16'd50))) begin
                    lat_fix      <= 1'b1;
                    lat_fix_used <= 1'b1;
                    lat_rem      <= lat_acc;
                    ret_st       <= 1'b0;          // 走正常回线入口（先原地转到回线角）
                    ret_piv      <= 1'b0;          // ★V4.35a★ 同样要重置回线阶段状态
                    ret_block_cnt<= 4'd0;
                    dbg_why      <= 8'h55;
                    st           <= S_RET2;
                end else begin
                    st           <= S_IDLE;
                    dbg_why      <= 8'h03;
                end
            end else if (crush_stop) begin
                // ★V2.28★ 车体包络已侵入：不再等小步结束/看门狗，立即锁失败并停车。
                //   顶层会把 avoid_fail 无条件并入 obstacle_stop（不再被 avoid_running 屏蔽）。
                avoid_fail   <= 1'b1;
                avoid_active <= 1'b0;
                load_pend    <= 1'b0;
                busy         <= 1'b0;
                dbg_why      <= 8'h52;
                st           <= S_FAIL;
            end else if (st == S_FAIL) begin
                avoid_fail   <= 1'b1;
                avoid_active <= 1'b0;
                load_pend    <= 1'b0;
            end else begin
                if (avoid_active) to_cnt <= to_cnt + 32'd1;
                a0_pass <= side_seen;   // ★V4.33c★ 前端平齐 = 进过侧窗（每拍跟随）

                case (st)
                //==============================================================
                S_IDLE: begin
                    avoid_active <= 1'b0;
                    if (arm && front_block && scan_done && !avoid_fail &&
                        (avoid_cnt <= RETRY_MAX)) begin
                        // 触发瞬间：冻结任务剩余里程、记下原航向
                        // ★V4.34c★ 基准航向：整段任务只取一次，之后每轮都用它
                        yaw_base      <= yaw_home_v ? yaw_home : yaw_now;
                        if (!yaw_home_v) yaw_home <= yaw_now;
                        yaw_home_v    <= 1'b1;
                        // ★V4.34b★ 原来这里写 yaw_ok<=yaw_v，而 yaw_v 是单拍脉冲
                        //   ⇒ yaw_ok 几乎恒为 0 ⇒ S_FOLLOW 的 w=3C、S_RET2 的补航向
                        //   全是死代码（三份现场日志里从没出现 3C）。改由下面的
                        //   "IMU 存活" 逻辑维护，这里不再动它。
                        remain_at_arm <= remain_in;
                        trav_acc      <= 32'd0;
                        fwd_acc       <= 32'd0;
                        lat_acc       <= 32'd0;
                        lat_rem       <= 32'd0;
                        to_cnt        <= 32'd0;
                        side          <= 1'b1;      // 先用"最省的一侧为空"的默认，S_CALC 里定
                        calc_st       <= 4'd0;
                        // ★V4.1a★ 修 V4.0b 遗留：gap_st 原来只在复位清 0，
                        //   第二次绕障会从 4'd4 直接跳回 S_CALC，沿用上一次的空当结果。
                        gap_st        <= 4'd0;
                        gap_ok        <= 1'b0;
                        srch_k        <= 4'd3;
                        phi_k         <= 16'd0;
                        phi_deg       <= 16'd0;
                        turn_retry    <= 4'd0;
                        adj_cnt       <= 4'd0;
                        end_retry     <= 4'd0;
                        aim_cnt       <= 4'd0;
                        hdg_fix_cnt   <= 4'd0;
                        fol_st        <= 3'd0;
                        ret_st        <= 1'b0;
                        ret_yaw_ok    <= 1'b0;
                        ret_try       <= 2'd0;
                        calc_retry    <= 2'd0;
                        front_gone_cnt<= 2'd0;      // ★V2.9b★ 新一次绕障重新数"正前看不到"的圈数
                        side_seen     <= 1'b0;      // ★V4.32a★ 新一次绕障重新记"进没进过侧窗"
                        a0_pass       <= 1'b0;      // ★V4.33c★ 前端平齐里程碑重新记
                        ret_tgt       <= 16'd0;     // ★V4.34a★ 回线航向闭环重新记
                        ret_corr      <= 1'b0;
                        ret_fix_cnt   <= 4'd0;
                        ret_block_cnt <= 4'd0; ret_hold <= 1'b0;
                        side_prox_r  <= 16'hFFFF;
                        lat_fix       <= 1'b0;
                        lat_fix_used  <= 1'b0;      // 新一次绕障允许再补一次回线
                        side_min      <= 16'hFFFF;  // ★V2.17★ 新一次绕障重新记"侧面最贴近多近"
                        fail_want     <= 1'b0;      // ★V2.12★
                        avoid_active  <= 1'b1;
                        busy          <= 1'b0;
                        dbg_why       <= 8'h10;
                        st            <= S_GAP;   // ★V4.0b★ 先读直方图选谷，再进 S_CALC
                    end else if (arm && front_block && (avoid_cnt > RETRY_MAX)) begin
                        dbg_why <= 8'h24;               // 绕障次数超限 ⇒ 停车锁存
                        st      <= S_FAIL;
                    end
                end

                //==============================================================
                // S_CALC：选边 + 算角度（顺序推进，每拍最多一次乘法）
                //   calc_st 0 : k0/k1 = a/5
                //          1 : 查 sin/cos 表
                //          2 : xL/yL
                //          3 : xR/yR
                //          4 : dxL/dxR/capL/capR ⇒ okL/okR ⇒ 选边、算 Dm/rhs256
                //          5 : 递增搜索角度（每拍一档）
                //==============================================================
                //==============================================================
                // ★V4.0b★ S_GAP：72 扇区直方图选谷（FGM 近似）
                //   1) 读 72 扇区，<600mm 记 blocked
                //   2) 只看前半球 0..35（0..180°），找 ≥8 扇区(40°) 的自由段
                //   3) 选中心离正前(扇区18)最近的段；center<18 左、>18 右
                //==============================================================
                S_GAP: begin
                    avoid_active <= 1'b1;
                    case (gap_st)
                    4'd0: begin
                        hist_rd_addr <= 7'd0;
                        gap_idx <= 7'd0; gap_blk <= 72'd0;
                        gap_any_blk <= 1'b0; gap_ok <= 1'b0;
                        sw_ph <= 3'd0;
                        gap_st <= 4'd1;
                    end
                    4'd1: begin
                        // ★V4.2j★ 直方图改成寄存读后，这里也必须"出地址 → 等一拍 → 再判"。
                        //   原来少等这一拍，读到的是上一个扇区的距离 ⇒ gap_blk 整体错位，
                        //   兜底会给出莫名其妙的 5° 小角。
                        if (sw_ph == 3'd0) begin
                            hist_rd_addr <= {1'b0, gap_idx};
                            sw_ph <= 3'd1;
                        end else begin
                            if ((hist_rd_data != 16'hFFFF) && (hist_rd_data < GAP_BLOCK_MM)) begin
                                gap_blk[gap_idx] <= 1'b1;
                                gap_any_blk <= 1'b1;
                            end
                            sw_ph <= 3'd0;
                            if (gap_idx == 7'd71) begin
                                gap_st <= 4'd2;
                            end else begin
                                gap_idx <= gap_idx + 7'd1;
                            end
                        end
                    end
                    4'd2: begin
                        if (!gap_any_blk) begin
                            gap_ok <= 1'b0;
                            gap_st <= 4'd5;            // ★V4.1a★ 直方图全空也先做影子扫掠
                        end else begin
                            gap_idx <= 7'd0; gap_run_len <= 8'd0; gap_run_start <= 7'd0;
                            gap_best_len <= 8'd0; gap_best_start <= 7'd0; gap_best_dist <= 8'd255;
                            gap_st <= 4'd3;
                        end
                    end
                    4'd3: begin
                        if ((gap_idx > 7'd35) || gap_blk[gap_idx]) begin
                            if (gap_run_len >= GAP_MIN_SECT) begin
                                gap_center_c = gap_run_start + ((gap_run_len - 8'd1) >> 1);
                                gap_dist_c = (gap_center_c > 7'd18) ? (gap_center_c - 7'd18)
                                                                    : (7'd18 - gap_center_c);
                                if ((gap_dist_c < gap_best_dist) ||
                                    ((gap_dist_c == gap_best_dist) && (gap_run_len > gap_best_len))) begin
                                    gap_best_start <= gap_run_start;
                                    gap_best_len <= gap_run_len;
                                    gap_best_dist <= gap_dist_c;
                                end
                            end
                            gap_run_len <= 8'd0;
                            gap_run_start <= gap_idx + 7'd1;
                        end else begin
                            if (gap_run_len == 8'd0) gap_run_start <= gap_idx;
                            gap_run_len <= gap_run_len + 8'd1;
                        end
                        if (gap_idx == 7'd71) gap_st <= 4'd4;
                        else gap_idx <= gap_idx + 7'd1;
                    end
                    4'd4: begin
                        if (gap_best_len >= GAP_MIN_SECT) begin
                            gap_center_c = gap_best_start + ((gap_best_len - 8'd1) >> 1);
                            gap_side <= (gap_center_c > 7'd18) ? 1'b1 : 1'b0;
                            gap_phi  <= (gap_center_c > 7'd18)
                                        ? ((gap_center_c - 7'd18) * 16'd5)
                                        : ((7'd18 - gap_center_c) * 16'd5);
                            if (gap_phi > PHI_MAX_DEG) gap_phi <= PHI_MAX_DEG;
                            gap_k    <= (gap_phi * 16'd205) >> 10;
                            gap_ok   <= 1'b1;
                        end else begin
                            gap_ok <= 1'b0;
                        end
                        gap_st <= 4'd5;                // ★V4.1a★ 影子扫掠 → 再进 S_CALC
                    end
                    //----------------------------------------------------------
                    // ★V4.1a★ 影子几何规划器：只算不动，结果送 dbg_shadow
                    //   4'd5 初始化 → 4'd6 找最远前向点 → 4'd7 定走廊长度
                    //   → 4'd8 候选角扫掠 → 4'd9 交回 S_CALC
                    //----------------------------------------------------------
                    4'd5: begin
                        sw_idx  <= 7'd0;
                        sw_ymax <= 16'd0;
                        sw_vhalf<= HALF_W_M;
                        sw_any  <= 1'b0;
                        sw_ph   <= 3'd0;
                        sw_kL   <= 4'd15;
                        sw_kR   <= 4'd15;
                        sw_hit  <= 1'b0;
                        sw_cand <= 5'd0;
                        sw_k    <= 4'd0;      // k=0 也参与：确实不需要转时要能报 0
                        sw_side <= 1'b0;
                        sw_cur_k_s <= 8'sd0;  // 初始点云就是 yaw_base 车体系
                        hist_rd_addr <= 7'd0;
                        gap_st  <= 4'd6;
                    end
                    4'd6: begin
                        // 两拍一点：ph0 出地址，ph1 收数据
                        if (sw_ph == 3'd0) begin
                            hist_rd_addr <= {1'b0, sw_sec};
                            sw_ph <= 3'd1;
                        end else if (sw_ph == 3'd1) begin
                            // ★V4.2h★ 直方图改成寄存读 ⇒ 地址后第 2 拍数据才有效
                            sw_ph <= 3'd2;
                        end else begin
                            if (sw_rok) begin
                                sw_any <= 1'b1;
                                if (sw_yax > $signed({16'd0, sw_ymax}))
                                    sw_ymax <= sw_yax[15:0];
                            end
                            sw_ph <= 3'd0;
                            if (sw_idx == 7'd72) gap_st <= 4'd7;
                            else                 sw_idx <= sw_idx + 7'd1;
                        end
                    end
                    4'd7: begin
                        if (!sw_any) begin
                            // 前半球一个有效点都没有 ⇒ 影子不可信，交回旧路径
                            sw_kL  <= 4'd15;
                            sw_kR  <= 4'd15;
                            gap_st <= 4'd9;
                        end else begin
                            // ★V4.3a★ 优先只扫当前障碍段前方“整车长 + 一步”；
                            //   这样远处柜脚/墙边不能把局部绕行路线整体判死。
                            if (sw_near_valid) begin
                                sw_umax <= sw_umax_local;
                            end else if (sw_ymax > (SW_L_MAX_MM - SW_L_EXTRA_MM))
                                sw_umax <= AXLE_FWD_MM + SW_L_MAX_MM;
                            else begin
                                sw_umax <= AXLE_FWD_MM + sw_ymax + SW_L_EXTRA_MM;
                            end
                            sw_idx  <= 7'd0;
                            sw_ph   <= 3'd0;
                            sw_k    <= 4'd0;
                            sw_side <= 1'b0;
                            sw_cand <= 5'd0;
                            sw_hit  <= 1'b0;
                            hist_rd_addr <= 7'd0;
                            gap_st  <= 4'd8;
                        end
                    end
                    4'd8: begin
                        // 每个点 4 拍：出地址 / 算x,y / 算u,v / 判命中
                        case (sw_ph)
                        3'd0: begin
                            hist_rd_addr <= {1'b0, sw_sec};
                            if (sw_idx == 7'd0) sw_vhalf <= sw_vhalf_n;
                            sw_ph <= 3'd1;
                        end
                        3'd1: begin
                            // ★V4.2h★ 寄存读的第二拍：等数据寄存器更新
                            sw_ph <= 3'd2;
                        end
                        3'd2: begin
                            sw_ok <= sw_rok;
                            sw_x  <= sw_xc[17:0];
                            sw_y  <= sw_yax[17:0];
                            sw_ph <= 3'd3;
                        end
                        3'd3: begin
                            sw_u  <= sw_uc[17:0];
                            sw_v  <= sw_vc[17:0];
                            sw_ph <= 3'd4;
                        end
                        default: begin
                            if (sw_coll) sw_hit <= 1'b1;
                            sw_ph <= 3'd0;
                            if (sw_idx == 7'd72) begin
                                // 本候选判完：没命中就记下该侧最小可行档
                                if (!sw_hit && !sw_coll) begin
                                    if (!sw_side && (sw_kL == 4'd15)) sw_kL <= sw_k_bias;
                                    if ( sw_side && (sw_kR == 4'd15)) sw_kR <= sw_k_bias;
                                end
                                sw_hit <= 1'b0;
                                sw_idx <= 7'd0;
                                if (sw_cand == 5'd25) begin
                                    gap_st <= 4'd9;
                                end else begin
                                    sw_cand <= sw_cand + 5'd1;
                                    if (!sw_side) sw_side <= 1'b1;
                                    else begin
                                        sw_side <= 1'b0;
                                        sw_k    <= sw_k + 4'd1;
                                    end
                                end
                            end else begin
                                sw_idx <= sw_idx + 7'd1;
                            end
                        end
                        endcase
                    end
                    4'd9: begin
                        sw_done <= 1'b1;               // 影子跑完（哪怕两侧都不可行）
                        st <= S_CALC;                  // 影子算完，行为完全不变
                    end
                    default: begin gap_ok <= 1'b0; st <= S_CALC; end
                    endcase
                end

                S_CALC: begin
                    avoid_active <= 1'b1;
                    case (calc_st)
                    4'd0: begin
                        // k = a/5（顺序比较；只在进避障时算一次。方向约定：0°=车正左、
                        //   90°=正前、180°=车正右 ⇒ k=0..36）
                        k0 = 16'd0;
                        for (t = 5; t <= 180; t = t + 5)
                            if (obs_a0 >= t[15:0]) k0 = k0 + 16'd1;
                        k1 = 16'd0;
                        for (t = 5; t <= 180; t = t + 5)
                            if (obs_a1 >= t[15:0]) k1 = k1 + 16'd1;
                        calc_st <= 4'd1;
                    end
                    4'd1: begin
                        // 端点方位可能在 90° 两侧 ⇒ 用带镜像的 sinA/cosA（cos 会为负）
                        c0 <= cosA_q10(k0); s0 <= sinA_q10(k0);
                        c1 <= cosA_q10(k1); s1 <= sinA_q10(k1);
                        calc_st <= 4'd2;
                    end
                    4'd2: begin
                        // x(右) = -d·cos(a)、y(前) = d·sin(a)（相对雷达中心）
                        //   ★c0/s0 是有符号的（方位 >90° 时 cos<0）⇒ 必须 $signed(c0)，
                        //     写成 {16'd0,c0} 会把符号位丢掉、乘出天文数字★
                        xL <= -(($signed({16'd0, obs_d0}) * $signed(c0)) >>> 10);
                        yL <=  (($signed({16'd0, obs_d0}) * $signed(s0)) >>> 10);
                        calc_st <= 4'd3;
                    end
                    4'd3: begin
                        xR <= -(($signed({16'd0, obs_d1}) * $signed(c1)) >>> 10);
                        yR <=  (($signed({16'd0, obs_d1}) * $signed(s1)) >>> 10);
                        calc_st <= 4'd4;
                    end
                    4'd4: begin
                        // 需要的横移（右为正）：
                        //   往左绕 Δx_L = (半宽+余量) - x_L（x_L 是箱子左端横向位置，通常为负）
                        //   往右绕 Δx_R = (半宽+余量) + x_R
                        dxL <= (xL <= 32'sd0) ? (HALF_W_M + (16'd0 - xL[15:0])) :
                                                ((HALF_W_M > xL[15:0]) ? (HALF_W_M - xL[15:0]) : 16'd0);
                        dxR <= (xR >= 32'sd0) ? (HALF_W_M + xR[15:0]) :
                                                ((HALF_W_M > (16'd0 - xR[15:0])) ?
                                                 (HALF_W_M - (16'd0 - xR[15:0])) : 16'd0);
                        // 那一侧实测可用横移 = 正左/正右窗口 - (半宽+余量)。
                        //   ★V2.28★ 不能再把 front_l/front_r 取 min 进来：
                        //   12:12 现场 L=1188、R=1467、FL=FR=540，箱子在正前，
                        //   旧式 min(1188,540)-200=340 < dxL=374 ⇒ 误判“两侧都不够”w=25。
                        capL <= ((left_mm  < wide_l_mm ? left_mm  : wide_l_mm ) > HALF_W_M) ?
                                ((left_mm  < wide_l_mm ? left_mm  : wide_l_mm ) - HALF_W_M) : 16'd0;
                        capR <= ((right_mm < wide_r_mm ? right_mm : wide_r_mm) > HALF_W_M) ?
                                ((right_mm < wide_r_mm ? right_mm : wide_r_mm) - HALF_W_M) : 16'd0;
                        // 障碍近面到【后轮轴】的距离：
                        //   obs_dmin 是斜距，不能直接当 y 用；yL/yR 已经由端点方位换算成前向坐标。
                        //   后轮轴在雷达后方 AXLE_OFF(25mm) ⇒ 加回来，再扣掉安全量 D_SAFE。
                        Dm   <= (dm_proj > 0) ? dm_proj[15:0] : 16'd0;
                        calc_st <= 4'd5;
                    end
                    4'd5: begin
                        // ---- 选边：先看"能不能过去"，再比"哪个更省" ----
                        // ★V2.28★ front_l_ok/front_r_ok 是“该半区有没有点”，
                        //   没有点 = 开阔，不能当成“不可走”。
                        //   只有确实有点、且该点已经进入车体半宽时才否决这一侧。
                        okL <= obs_valid &&
                               ((!front_l_ok) || (front_l_mm > HALF_W_M)) &&
                               (dxL <= capL);
                        okR <= obs_valid &&
                               ((!front_r_ok) || (front_r_mm > HALF_W_M)) &&
                               (dxR <= capR);
                        // ★V2.7（清单5/H1）★："没测到段"不再立刻认输——真雷达到处是点，
                        //   一帧没段多半是抖动/遮挡 ⇒ 等下一圈新数据重算（最多 CALC_RETRY_MAX 圈）
                        if (!obs_valid) begin
                            if (calc_retry < CALC_RETRY_MAX) begin
                                calc_retry <= calc_retry + 2'd1;
                                dbg_why    <= 8'h28;           // 在等下一圈（不是卡死）
                                wd         <= 32'd0;
                                st         <= S_REWAIT;
                            end else begin
                                dbg_why <= 8'h22; st <= S_FAIL;   // 等满 3 圈还是没段 ⇒ 认输
                            end
                        end else if (Dm == 16'd0) begin
                            dbg_why <= 8'h22; st <= S_FAIL;        // 太近 ⇒ 认输
                        // ★V2.8（清单9）★：下面一律用 dxL_use/dxR_use（含物理下限），
                        //   "能不能过去"和"哪边更省"都按车【实际要挪的量】算。
                        end else begin
                            // ★V2.30★ 两侧各搜一个最小 φ；先不算选边，等 4'd8 统一比
                            phiL_k <= 4'd13; phiR_k <= 4'd13;
                            srch_k <= 4'd3;
                            calc_st<= 4'd6;
                        end
                    end
                    4'd6: begin
                        // 左侧最小 φ：tanφ ≥ dxL_use/Dm
                        if ((!okL) || (dxL_use > capL)) begin
                            phiL_k <= 4'd13; srch_k <= 4'd3; calc_st <= 4'd7;
                        end else if (srch_k > 4'd12) begin
                            phiL_k <= 4'd13; srch_k <= 4'd3; calc_st <= 4'd7;
                        end else if (({16'd0, Dm} * tan_q8(srch_k)) >= ({dxL_use, 16'd0} >> 8)) begin
                            phiL_k <= srch_k; srch_k <= 4'd3; calc_st <= 4'd7;
                        end else begin
                            srch_k <= srch_k + 4'd1;
                        end
                    end
                    4'd7: begin
                        // 右侧最小 φ
                        if ((!okR) || (dxR_use > capR)) begin
                            phiR_k <= 4'd13; calc_st <= 4'd8;
                        end else if (srch_k > 4'd12) begin
                            phiR_k <= 4'd13; calc_st <= 4'd8;
                        end else if (({16'd0, Dm} * tan_q8(srch_k)) >= ({dxR_use, 16'd0} >> 8)) begin
                            phiR_k <= srch_k; calc_st <= 4'd8;
                        end else begin
                            srch_k <= srch_k + 4'd1;
                        end
                    end
                    4'd8: begin
                        // ★V4.1b★ 影子几何规划器优先：用真实车体矩形扫掠算出的
                        //   左右最小可行档，取代"自由段中点"这个与障碍尺寸无关的角度。
                        if (sw_best_k != 4'd15) begin
                            side     <= sw_best_side;
                            phi_deg  <= sw_best_phi;
                            phi_k    <= {12'd0, sw_best_k};
                            turn_n   <= TURN_CNT_PER_DEG * sw_best_phi;
                            dx_sel   <= sw_best_side ? dxR_use : dxL_use;
                            avoid_cnt<= avoid_cnt + 4'd1;
                            busy     <= 1'b0;
                            dbg_why  <= 8'h21;
                            st       <= S_T_OUT;
                        // ★V4.0b★ 回退：影子两侧都不可行时才用直方图谷中心
                        end else if (gap_ok && (Dm != 16'd0)) begin
                            side     <= gap_side;
                            phi_deg  <= gap_phi;
                            phi_k    <= gap_k;
                            turn_n   <= TURN_CNT_PER_DEG * gap_phi;
                            dx_sel   <= gap_side ? dxR_use : dxL_use;
                            avoid_cnt<= avoid_cnt + 4'd1;
                            busy     <= 1'b0;
                            dbg_why  <= 8'h21;
                            st       <= S_T_OUT;
                        // 选“φ 更小且可行”的一侧；φ 相同再比 dx_use，最后偏左
                        end else if ((phiL_k <= 4'd12) && (phiR_k <= 4'd12)) begin
                            if ((phiL_k * 16'd5) < (phiR_k * 16'd5)) begin
                                side<=1'b0; dx_sel<=dxL_use; chk_k<=phiL_k; chk_side<=1'b0;
                            end else if ((phiR_k * 16'd5) < (phiL_k * 16'd5)) begin
                                side<=1'b1; dx_sel<=dxR_use; chk_k<=phiR_k; chk_side<=1'b1;
                            end else if (dxL_use <= dxR_use) begin
                                side<=1'b0; dx_sel<=dxL_use; chk_k<=phiL_k; chk_side<=1'b0;
                            end else begin
                                side<=1'b1; dx_sel<=dxR_use; chk_k<=phiR_k; chk_side<=1'b1;
                            end
                            other_tried<=1'b0; calc_st<=4'd9;
                        end else if (phiL_k <= 4'd12) begin
                            side<=1'b0; dx_sel<=dxL_use; chk_k<=phiL_k; chk_side<=1'b0;
                            other_tried<=1'b0; calc_st<=4'd9;
                        end else if (phiR_k <= 4'd12) begin
                            side<=1'b1; dx_sel<=dxR_use; chk_k<=phiR_k; chk_side<=1'b1;
                            other_tried<=1'b0; calc_st<=4'd9;
                        end else if ((dxL <= capL) || (dxR <= capR)) begin
                            dbg_why <= 8'h2A; st <= S_FAIL;
                        end else begin
                            dbg_why <= 8'h25; st <= S_FAIL;
                        end
                    end
                    4'd9: begin
                        // 回线走廊长度 L=dx_use/sinφ；先清命中
                        chk_L <= (({16'd0, dx_use} * csc_q10(chk_k)) >> 10);
                        if (chk_L > 32'd2000) chk_L <= 32'd2000;
                        chk_hit <= 1'b0;
                        calc_st <= 4'd10;
                    end
                    4'd10: begin
                        // 端点0 转到回线坐标系
                        chk_u <= ((xL * sinA_q10(chk_k * 16'd2)) >>> 10) +
                                 (((yL + 32'sd25) * cosA_q10(chk_k * 16'd2)) >>> 10);
                        chk_v <= ((xL * cosA_q10(chk_k * 16'd2)) >>> 10) -
                                 (((yL + 32'sd25) * sinA_q10(chk_k * 16'd2)) >>> 10);
                        calc_st <= 4'd11;
                    end
                    4'd11: begin
                        // 端点0：227mm 枢轴圆 或 回线走廊 命中
                        chk_hit <= ( (xL*xL + (yL+32'sd25)*(yL+32'sd25)) < R_PIVOT2 ) ||
                                   ( ((chk_v[31] ? (32'd0 - chk_v) : chk_v) <= HALF_W_CHK) &&
                                     (chk_u >= -32'sd65) &&
                                     (chk_u <= $signed(chk_L + 32'd175)) );
                        calc_st <= 4'd12;
                    end
                    4'd12: begin
                        chk_u <= ((xR * sinA_q10(chk_k * 16'd2)) >>> 10) +
                                 (((yR + 32'sd25) * cosA_q10(chk_k * 16'd2)) >>> 10);
                        chk_v <= ((xR * cosA_q10(chk_k * 16'd2)) >>> 10) -
                                 (((yR + 32'sd25) * sinA_q10(chk_k * 16'd2)) >>> 10);
                        calc_st <= 4'd13;
                    end
                    4'd13: begin
                        chk_hit <= chk_hit ||
                                   ( (xR*xR + (yR+32'sd25)*(yR+32'sd25)) < R_PIVOT2 ) ||
                                   ( ((chk_v[31] ? (32'd0 - chk_v) : chk_v) <= HALF_W_CHK) &&
                                     (chk_u >= -32'sd65) &&
                                     (chk_u <= $signed(chk_L + 32'd175)) );
                        calc_st <= 4'd14;
                    end
                    4'd14: begin
                        if (!chk_hit) begin
                            phi_k   <= chk_k;
                            phi_deg <= chk_k * 16'd5;
                            turn_n  <= TURN_CNT_PER_DEG * (chk_k * 16'd5);
                            avoid_cnt <= avoid_cnt + 4'd1;
                            busy    <= 1'b0;
                            dbg_why <= 8'h21;
                            st      <= S_T_OUT;
                        end else if (chk_k < 4'd12) begin
                            chk_k <= chk_k + 4'd1;
                            calc_st <= 4'd9;
                        end else if (!other_tried) begin
                            if ((chk_side == 1'b0) && (phiR_k <= 4'd12)) begin
                                side<=1'b1; dx_sel<=dxR_use; chk_k<=phiR_k; chk_side<=1'b1;
                                other_tried<=1'b1; calc_st<=4'd9;
                            end else if ((chk_side == 1'b1) && (phiL_k <= 4'd12)) begin
                                side<=1'b0; dx_sel<=dxL_use; chk_k<=phiL_k; chk_side<=1'b0;
                                other_tried<=1'b1; calc_st<=4'd9;
                            end else begin
                                dbg_why <= 8'h2A; st <= S_FAIL;
                            end
                        end else begin
                            dbg_why <= 8'h2A; st <= S_FAIL;
                        end
                    end
                    default: begin
                        dbg_why <= 8'h26; st <= S_FAIL;             // 不该到这里
                    end
                    endcase
                end

                //==============================================================
                // ★V2.7（清单5/H1）★ S_REWAIT：触发那一圈没测到障碍段，等下一圈重算。
                //   scan_done 是"每成圈 1 拍"的脉冲 ⇒ 它一来就回 S_CALC 重跑一遍。
                //   ★avoid_active 必须保持 1★：掉下来会放行 obstacle_stop，
                //     车会被急停掐一下（force_stop 还会压住电机侧的 seg_done）。
                //==============================================================
                S_REWAIT: begin
                    avoid_active <= 1'b1;
                    if (scan_done) begin
                        calc_st <= 4'd0;
                        st      <= S_CALC;
                    end else if (wd >= STEP_TO_CYC) begin
                        dbg_why <= 8'h29; st <= S_FAIL;    // 等圈超时 ⇒ 真失败
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                //==============================================================
                S_T_OUT: begin                      // 转到 yaw_base ± φ（IMU 闭环）
                    avoid_active <= 1'b1;
                    if (!busy) begin
                        pend_act <= A_YAW_ABS; pend_n <= turn_n;
                        // 182 = 每 1° 的 yaw 计数；必须用【角度】phi_deg（V2.1 修的 bug）
                        pend_yaw <= yaw_out_tgt;
                        // ★V2.7（清单3）★：出航向转改【全速】——原来 75% 慢得能看出来
                        // ★V4.1c★ 回到 75% 速（ext_slow=1 同时给 AVOID_SPD）：全速实测过冲 12.6°
                        pend_to_q<= 1'b0; pend_slow <= 1'b1;
                        load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                        yaw_ref <= yaw_now; yaw_wd <= 32'd0;
                    end else if (ext_done) begin
                        busy <= 1'b0; wd <= 32'd0;
                        // ★V2.2：回读校验——实车常只转出目标的 60%（边跑边转被拖着走）★
                        if ((yaw_diff(yaw_now, yaw_out_tgt) > YAW_TOL_CNT) &&
                            (turn_retry < TURN_RETRY)) begin
                            turn_retry <= turn_retry + 4'd1;
                            dbg_why    <= 8'h38;              // 没转到位 ⇒ 重发同一个转向
                            st         <= S_T_OUT;
                        end else begin
                            turn_retry <= 4'd0;
                            if (yaw_diff(yaw_now, yaw_out_tgt) > YAW_TOL_CNT) begin
                                // ★V4.2a★ 清单第3项：重试用尽仍没转到位 ⇒ 先回正再认输，
                                //   不再带着错误航向进 S_FOLLOW（旧版这里直接放行）。
                                dbg_why  <= 8'h39;
                                fail_want<= 1'b1;
                                st       <= S_T_BACK;
                            end else begin
                                dbg_why  <= 8'h30;
                                // ★V4.35d★ 补航向成功 ⇒ 把 3C 预算重置。
                                //   原来 hdg_fix_cnt 只在 arm 清零，一趟绕障里最多补 8 次就永久失效
                                //   （17:31 现场已用到 6 次，再差 2 次这趟就再也不补航向 ⇒
                                //   车带着 15°+ 偏差贴边走 ⇒ 更容易撞）。
                                hdg_fix_cnt <= 4'd0;
                                st       <= S_FOLLOW;   // 初始转到位 ⇒ 进入边界跟随
                            end
                        end
                    // ★V4.1c★ 卡死判据改成"1.5s 窗口内航向没动"；正在过冲/回收都不算
                    end else if (yaw_wd >= YAW_STUCK_CYC) begin
                        if ((yaw_diff(yaw_now, yaw_out_tgt) > YAW_TOL_CNT) &&
                            (yaw_diff(yaw_now, yaw_ref) <= YAW_STUCK_DELTA)) begin
                            dbg_why <= 8'h53; fail_want <= 1'b1; st <= S_T_BACK;
                        end else begin
                            yaw_ref <= yaw_now;
                            yaw_wd  <= 32'd0;
                            wd      <= wd + 32'd1;
                        end
                    // ★V2.7（H3）★：段头 600ms 内不认 stall（见 stall_hit 定义）
                    end else if (stall_hit || (wd >= PIVOT_TO_CYC)) begin
                        dbg_why <= 8'h31; fail_want <= 1'b1; st <= S_T_BACK;   // ★V2.12★ 先回正
                    end else begin
                        wd <= wd + 32'd1;
                        yaw_wd <= yaw_wd + 32'd1;
                    end
                end

                //==============================================================
                S_DIAG: begin                       // 小步斜行：保持航向 + 侧向间隙 + 末端判据
                    avoid_active <= 1'b1;
                    watch_en     <= 1'b1;
                    watch_ang    <= wang;
                    // ★V2.7（H3）★：段头 600ms 内不认 stall（见 stall_hit 定义）
                    if (stall_hit) begin
                        dbg_why <= 8'h32; fail_want <= 1'b1; st <= S_T_BACK;   // ★V2.12★ 先回正再认输
                    end else if ((trav_acc - fwd_acc) > EXTRA_MAX_MM) begin
                        dbg_why <= 8'h34; fail_want <= 1'b1; st <= S_T_BACK;   // 额外里程超限
                    end else if (to_cnt > TO_CYC) begin
                        dbg_why <= 8'h35; fail_want <= 1'b1; st <= S_T_BACK;   // 总时长超限
                    end else if (!busy) begin
                        // ★末端判据（V2.1 换观测量：不再用"侧窗看不见了"——那会被 1m 外的墙
                        //   满足，实测导致"箱子还在正前 19cm 就收工回头"）★
                        //   ① 横向已经挪够：lat_acc ≥ 本次需要的 dx_sel ⇒ 真的绕到旁边了
                        //   ② 前半球最近的东西不再贴脸：obs_dmin ≥ SIDE_MIN_MM（或没段）
                        //      车体回转半径 ≈211mm ⇒ 这一条等价于"此时回头扫不到它"
                        //   ③ 至少前进 ADV_MIN_MM（防"原地打转就收工"）
                        // ★V2.3：先补航向，再判收工——航向没到 φ 时"实测横移"没意义★
                        // ★V2.13★：门限用 AIM_TOL_CNT(8°)（原来 YAW_TOL_CNT=4° 太紧 ⇒ 反复补转=摆头）
                        if (yaw_ok && (yaw_diff(yaw_now, yaw_out_tgt) > AIM_TOL_CNT) &&
                            (aim_cnt < ADJ_MAX)) begin
                            aim_cnt    <= aim_cnt + 4'd1;
                            turn_retry <= 4'd0;
                            dbg_why    <= 8'h3C;          // 斜行航向偏离 φ ⇒ 先补转
                            st         <= S_T_OUT;
                        // ★V2.9b★：能不能回头 = 【两条实测】——
                        //   ① front_gone：正前连续 3 圈看不到障碍段（车已经走到它侧面/身后）
                        //   ② gap_side_win ≥ SIDE_MIN_MM：障碍侧那格的实测垂距 ≥ 回头扫掠净空
                        //   ★V2.18s★：侧窗实测必须再加“已横移够”的门槛（lat_acc ≥ dx_use）。
                        //   23:45 现场只横移 99mm，箱子滑出侧窗（R=FFFF、FR=1111）就被
                        //   误判成“已绕过”，回线只补 99mm，车平行停在障碍旁边。
                        //   ⇒ 保留实测窗口，但不再允许“窗口变空”单独放行回头。
                        //   dx_use/dx_sel 只留给"斜行角度 φ 怎么算"，不再决定什么时候回头。
                        // ★V2.9c/V2.11★：实测两条（0x33）或兜底一条（0x3D）任一条成立即可回头。
                        //   两个原因码分开，现场日志一看就知道是哪条路放行的。
                        end else if (turn_now_ok) begin
                            // ★V2.18★：回头后**直接进回线**（跳过"先转回原航向"那次多余转身）——
                            //   回线航向 = 原航向−φ，从斜行航向 +φ 直接转过去只有 2φ；
                            //   原来"转回原航向再转出去"要转两次，现场看就是"原地摆两下头"。
                            dbg_why <= 8'h33; wd <= 32'd0;
                            lat_rem <= lat_acc;
                            ret_yaw_ok <= 1'b0; ret_try <= 2'd0;
                            st      <= (lat_acc > RET_TOL_MM) ? S_RETURN : S_T_BACK;
                        end else if (turn_lat_ok) begin
                            dbg_why <= 8'h3D; wd <= 32'd0;   // ★V2.11★ 旧兜底（同上：直接进回线）
                            lat_rem <= lat_acc;
                            ret_yaw_ok <= 1'b0; ret_try <= 2'd0;
                            st      <= (lat_acc > RET_TOL_MM) ? S_RETURN : S_T_BACK;
                        end else begin
                            pend_act <= A_FWD_HOLD; pend_n <= STEP_CNT;
                            // 斜行/回线都朝"yaw_base ± φ"这个航向走（边走边修）
                            pend_yaw <= side ? (yaw_base - (phi_deg * 16'd182))
                                             : (yaw_base + (phi_deg * 16'd182));
                            // ★V2.9★：斜行腿改用【绕障降速档】AVOID_SPD(288≈100mm/s)。
                            //   用户 2026-10-15：「不要停…可以用降低速度来代替」⇒ 宁慢不停；
                            //   顺带每 40mm 一步从 ~0.32s 拉长到 ~0.40s，边走边测的点更多。
                            pend_to_q<= 1'b0; pend_slow <= 1'b1;
                            load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                        end
                    end else if (ext_done) begin
                        busy     <= 1'b0;
                        wd       <= 32'd0;
                        // 横移/里程记账（Q10 表算，全整数）
                        // ★V2.7（清单6）★：按【实测弧长】记账（不再用命令步长 STEP_MM）
                        trav_acc <= trav_acc + arc_mm;
                        // 记账：前进投影用 cos、横移用 sin（phi_k ≤ 12 ⇒ 表内 0..60° 够用）
                        // ★V2.3：改用【IMU 实测航向】记账（不再用命令行进的 phi_k）★
                        //   车没真转够 ⇒ 实测航向就小 ⇒ lat_acc 也小 ⇒ 不会再"以为挪够"
                        fwd_acc  <= fwd_acc + arc_cos_w[31:0];
                        lat_acc  <= lat_acc + arc_sin_w[31:0];
                    // ★V2.5（问题5）★：判"贴太近"改用【障碍所在侧的正前半区】(FL/FR)，
                    //   不再用 0°/180° 侧窗——那两格会被旁边家具长期占住（现场实测 388~1000mm）
                    // ★V2.15★：**检测到就反应**——"贴太近"改用障碍侧实测最近距离 d_obs 判：
                    //   d_obs < HALF_W_M（半宽+侧向余量，由车体尺寸算出）⇒ 真的让不开了
                    //   ⇒ 立刻加大绕行角度重走（不再是"盯正前半区 250mm"那种只看一格的写法）。
                    end else if (d_obs < HALF_W_M) begin
                        // ★V2.2：贴太近不再直接判失败——先把 φ 加大一档重走（最多 ADJ_MAX 次）★
                        if ((adj_cnt < ADJ_MAX) &&
                            ((phi_deg + ADJ_STEP_DEG) <= PHI_MAX_DEG)) begin
                            adj_cnt <= adj_cnt + 4'd1;
                            phi_k   <= phi_k + 4'd1;
                            phi_deg <= phi_deg + ADJ_STEP_DEG;
                            // ★V2.7（时序）★：同上，不做除法
                            turn_n  <= TURN_CNT_PER_DEG * (phi_deg + ADJ_STEP_DEG);
                            turn_retry <= 4'd0;
                            busy    <= 1'b0;
                            dbg_why <= 8'h3A;                 // 间隙不够 ⇒ 加大角度重走
                            st      <= S_T_OUT;
                        end else begin
                            dbg_why <= 8'h36; fail_want <= 1'b1; st <= S_T_BACK;  // ★V2.12★ 先回正
                        end
                    end else if (wd >= STEP_TO_CYC) begin
                        dbg_why <= 8'h37; st <= S_FAIL;              // 单步卡死
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                //==============================================================
                // ★V2.12★：S_T_BACK 现在有两个用途——①正常回正；②【失败前先回正】
                //   （fail_want=1）。后者是用户 23:35 日志暴露的缺陷：绕障判失败后，
                //   车带着 25° 的偏航向被"自动恢复"放回任务里 ⇒ 航向偏移被下一段锁死 ⇒
                //   "走出去了没有往回拐"。先转回 yaw_base 再认输，续跑就沿着原航向。
                S_T_BACK: begin
                    avoid_active <= 1'b1;
                    // ---- ★V2.12★ 失败流程：只做一件事——把车头转回 yaw_base，然后认输 ----
                    //   （必须放在最前面：进 S_T_BACK 时可能还有一小步斜行在跑，
                    //     那一步的 ext_done 不能当成"已经回正"，否则就会带着偏航向认输。）
                    if (fail_want) begin
                        if (!busy) begin
                            if (yaw_diff(yaw_now, yaw_base) <= YAW_TOL_CNT) begin
                                st <= S_FAIL;            // 已经正了 ⇒ 认输（停车 + 2 短声）
                            end else begin
                                pend_act <= A_YAW_ABS;  pend_n <= turn_n;
                                pend_yaw <= yaw_base;   pend_to_q <= 1'b0; pend_slow <= 1'b0;
                                load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                            end
                        end else if (ext_done) begin
                            busy <= 1'b0; wd <= 32'd0;   // 转完/小步跑完 ⇒ 下一拍再判
                        end else if (stall_hit || (wd >= PIVOT_TO_CYC)) begin
                            st <= S_FAIL;                // 转不动 ⇒ 别卡着，直接认输
                        end else begin
                            wd <= wd + 32'd1;
                        end
                    // ---- 正常流程：回正 ⇒ 回读校验 ⇒ 回线/续跑 ----
                    end else if (!busy) begin
                        pend_act <= A_YAW_ABS; pend_n <= turn_n;
                        // ★V2.7（清单3）★：回正转改【全速】（原来 75% ⇒ "慢慢回头"）
                        pend_yaw <= yaw_base; pend_to_q <= 1'b0; pend_slow <= 1'b0;
                        load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                        // 回线要走的横移量 = 出腿累计的横移
                        lat_rem  <= lat_acc;
                    end else if (ext_done) begin
                        busy <= 1'b0;
                        wd   <= 32'd0;
                        if ((yaw_diff(yaw_now, yaw_base) > YAW_TOL_CNT) &&
                            (turn_retry < TURN_RETRY)) begin
                        // ★V2.2：转回后回读校验；没回正就重发一次（最多 3 次）★
                            turn_retry <= turn_retry + 4'd1;
                            dbg_why    <= 8'h4A;
                            st         <= S_T_BACK;
                        end else begin
                            turn_retry <= 4'd0;
                            ret_piv    <= 1'b0;      // ★V2.5：进回线阶段，先"未转到位"★
                            ret_yaw_ok <= 1'b0; ret_try <= 2'd0;
                            dbg_why    <= 8'h40;
                            // ★V2.7（清单4）★：残差门 40mm → RET_TOL_MM(15mm)
                            st   <= (lat_rem > RET_TOL_MM) ? S_RETURN : S_RESUME;
                        end
                    // ★V2.7（H3）★：段头 600ms 内不认 stall（见 stall_hit 定义）
                    end else if (stall_hit || (wd >= PIVOT_TO_CYC)) begin
                        dbg_why <= 8'h41; st <= S_FAIL;
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                //==============================================================
                S_RETURN: begin                     // 用同一个 φ 斜着把横移走回来（回原航线）
                    avoid_active <= 1'b1;
                    // ★V2.7（H3）★：段头 600ms 内不认 stall（见 stall_hit 定义）
                    if (stall_hit) begin
                        dbg_why <= 8'h42; fail_want <= 1'b1; st <= S_T_BACK;   // ★V2.12★ 先回正
                    end else if (!busy) begin
                        // ★V2.18x★：2φ 转身结束后先核对实测航向，确认真的转到
                        //   yaw_base−φ 再开始回线腿；否则打滑/超时会带着浅角度走远。
                        if (ret_piv && !ret_yaw_ok) begin
                            if (yaw_diff(yaw_now, ret_yaw_tgt) <= YAW_TOL_CNT) begin
                                ret_yaw_ok <= 1'b1;
                                dbg_why    <= 8'h49;      // 回线航向已确认
                            end else if (ret_try < 3'd3) begin
                                ret_try <= ret_try + 3'd1;
                                ret_piv <= 1'b0;          // 重新回到“转身”阶段
                                pend_act <= A_YAW_ABS; pend_n <= turn_n + turn_n;
                                pend_yaw <= ret_yaw_tgt;
                                pend_to_q<= 1'b0; pend_slow <= 1'b0;
                                load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                                dbg_why   <= 8'h47;       // 回线转身没到位，重发
                            end else begin
                                // ★V2.28★ 两次重发仍不到 4°，不能再“假装到位”走回线腿：
                                //   22:42 现场就是带着 7.6° 误差走，越走越偏。
                                if (yaw_diff(yaw_now, ret_yaw_tgt) <= YAW_TOL_CNT) begin
                                    ret_yaw_ok <= 1'b1;
                                    dbg_why    <= 8'h48;
                                end else begin
                                    dbg_why    <= 8'h4C;
                                    fail_want  <= 1'b1;
                                    st         <= S_T_BACK;
                                end
                            end
                        // ★V2.7（清单4）★：残差门 40mm → RET_TOL_MM(15mm)
                        end else if (lat_rem <= RET_TOL_MM) begin
                            dbg_why <= 8'h43; st <= S_T_END; wd <= 32'd0;
                            turn_retry <= 4'd0;          // ★V2.28★ 回正阶段单独计数
                        end else if (!ret_piv) begin
                            // ★V2.5（问题3）★：回线必须先【真正转到】"斜行回线航向"——
                            //   A_FWD_HOLD 只能微调 ±几度，靠它转 30° 是转不过去的
                            //   （现场证据：RETURN 期间 Y 一直停在基准航向 ⇒ "路线不往回走"）
                            // ★V2.18★：这条腿是从**斜行航向(+φ)直接转到回线航向(−φ)**，转角 = 2φ
                            //   ⇒ 编码器兜底弧长按 2× 给（IMU 到位判据不受影响）。
                            pend_act <= A_YAW_ABS; pend_n <= turn_n + turn_n;
                            pend_yaw <= ret_yaw_tgt;
                            // ★V2.7（清单3）★：回线前那一次转保持【全速】——
                            //   用户要的是"回头别太慢"；V2.9 只降斜行/回线【腿】的速度，
                            //   原地转仍是全速（但不再停死，见 motor_driver_top 的低速起转）。
                            pend_to_q<= 1'b0; pend_slow <= 1'b0;
                            load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                        end else begin
                            pend_act <= A_FWD_HOLD; pend_n <= STEP_CNT;
                            // 朝反向斜着走：往右绕过 ⇒ 现在要朝左回（yaw_base + φ）
                            pend_yaw <= ret_yaw_tgt;
                            // ★V2.9★：回线腿同样用绕障降速档（AVOID_SPD≈100mm/s，宁慢不停）
                            pend_to_q<= 1'b0; pend_slow <= 1'b1;
                            load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                        end
                    end else if (ext_done) begin
                        busy    <= 1'b0;
                        wd      <= 32'd0;
                        if (!ret_piv) begin
                            ret_piv <= 1'b1;            // 转向那一段结束 ⇒ 后面才是斜行腿
                        end else begin
                            // ★V2.5（问题7）★：按【实测航向】记账，并把"净横移"减回去
                            //   ⇒ 回线结束后 lat_acc 就是真正的横向残差，可以直接校验
                            // ★V2.7（清单6）★：按【实测弧长】记账（回线腿同理）
                            trav_acc<= trav_acc + arc_mm;
                            // ★必须同时累加"前进投影"★：否则回线整段里程都被算成"白走"，
                            //   额外里程上限(80cm)会在回线途中误触发 → 判失败(现场 T4 的 why=34)
                            fwd_acc  <= fwd_acc + arc_cos_w[31:0];
                            lat_rem <= (lat_rem > arc_sin_w[31:0]) ?
                                        (lat_rem - arc_sin_w[31:0]) : 32'd0;
                            lat_acc <= (lat_acc > arc_sin_w[31:0]) ?
                                        (lat_acc - arc_sin_w[31:0]) : 32'd0;
                        end
                    end else if ((trav_acc - fwd_acc) > EXTRA_MAX_MM) begin
                        dbg_why <= 8'h34; fail_want <= 1'b1; st <= S_T_BACK;   // ★V2.12★ 先回正
                    end else if (to_cnt > TO_CYC) begin
                        dbg_why <= 8'h35; fail_want <= 1'b1; st <= S_T_BACK;   // ★V2.12★ 先回正
                    end else if (wd >= PIVOT_TO_CYC) begin
                        dbg_why <= 8'h44; fail_want <= 1'b1; st <= S_T_BACK;   // ★V2.12★ 先回正
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                //==============================================================
                S_T_END: begin                      // 最后一次转回 yaw_base（保证角度平行）
                    avoid_active <= 1'b1;
                    if (!busy) begin
                        pend_act <= A_YAW_ABS; pend_n <= turn_n;
                        // ★V2.7（清单3）★：最后一次转回 yaw_base 也改【全速】
                        pend_yaw <= yaw_base; pend_to_q <= 1'b0; pend_slow <= 1'b0;
                        load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                    end else if (ext_done) begin
                        busy <= 1'b0; wd <= 32'd0;
                        // ★V2.28★ 最后一次回正不能只看“段跑完”：
                        //   12:13 现场 ext_done 后航向还在 18.6°，S_RESUME 才发现
                        //   正前 143mm 判 w=27。这里先核对实测航向，超差就重发。
                        if ((yaw_diff(yaw_now, yaw_base) > YAW_TOL_CNT) &&
                            (turn_retry < TURN_RETRY)) begin
                            turn_retry <= turn_retry + 4'd1;
                            dbg_why    <= 8'h4B;
                            st         <= S_T_END;
                        end else begin
                            turn_retry <= 4'd0;
                            dbg_why    <= 8'h45;
                            st         <= S_RESUME;
                        end
                    // ★V2.7（H3）★：段头 600ms 内不认 stall（见 stall_hit 定义）
                    end else if (stall_hit || (wd >= PIVOT_TO_CYC)) begin
                        dbg_why <= 8'h46; st <= S_FAIL;
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                //==============================================================
                S_RESUME: begin                     // 把冻结的任务剩余里程交回队列（避障不计入任务）
                    avoid_active <= 1'b1;
                    if (!busy) begin
                        // ★V2.1：续跑前必须确认【正前通畅】★
                        //   否则就是"车顶着障碍物往前推"（2026-10-15 实测现象）⇒ 判失败停车
                        if (!(front_v && (front_mm >= (D_ARM_MM + 16'd50)))) begin
                            dbg_why <= 8'h27; st <= S_FAIL;
                        end else if ((yaw_diff(yaw_now, yaw_base) > YAW_TOL_CNT) &&
                                     (end_retry < TURN_RETRY)) begin
                            // ★V2.2：航向还是歪的 ⇒ 先补转，绝不歪着往前开★
                            end_retry  <= end_retry + 4'd1;
                            turn_retry <= 4'd0;
                            dbg_why    <= 8'h3B;
                            st         <= S_T_END;
                        end else begin
                            pend_act <= A_FWD; pend_n <= resume_n;
                            pend_yaw <= 16'd0; pend_to_q <= 1'b1; pend_slow <= 1'b0;
                            load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                        end
                    end else if (ext_done) begin
                        busy         <= 1'b0;
                        avoid_done   <= 1'b1;
                        avoid_active <= 1'b0;
                        lat_fix      <= 1'b0;
                        // ★V2.28★ 成功绕完后清“连续尝试”计数。
                        //   旧版只在 force_clear 才清，第三个障碍会直接 w=24 停车。
                        avoid_cnt    <= 4'd0;
                        dbg_why      <= 8'h50;
                        st           <= S_IDLE;
                    end else if (stall || (wd >= RESUME_TO_CYC)) begin
                        dbg_why <= 8'h51; st <= S_FAIL;
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                //==============================================================
                // ★V3.0★ S_FOLLOW：Bug2 边界跟随（实测侧距 PD，不再算固定 φ）
                //   side=0 障碍在右：e>0(太远) ⇒ 往右修(负角)；e<0(太近) ⇒ 往左修
                //==============================================================
                S_FOLLOW: begin
                    avoid_active <= 1'b1;
                    watch_en     <= 1'b1;
                    watch_ang    <= wang;
                    if (stall_hit) begin
                        dbg_why <= 8'h32; fail_want <= 1'b1; st <= S_T_BACK;
                    end else if ((trav_acc - fwd_acc) > EXTRA_MAX_MM) begin
                        dbg_why <= 8'h34; fail_want <= 1'b1; st <= S_T_BACK;
                    end else if (to_cnt > TO_CYC) begin
                        dbg_why <= 8'h35; fail_want <= 1'b1; st <= S_T_BACK;
                    end else if (!busy) begin
                        // ★V4.0c★ 直方图侧距（0°±15° / 180°±15° 最小），
                        //   不再用单个 0/180±20° 窗口。
                        side_win = side ? hist_side_l_mm : hist_side_r_mm;
                        // ★V4.35e★ 障碍侧最近距离（正侧窗 ∪ 障碍侧斜前窗），先算好再用
                        side_prox_r = (side_win == 16'hFFFF) ? side_wide_mm :
                                     ((side_wide_mm == 16'hFFFF) ? side_win :
                                      ((side_wide_mm < side_win) ? side_wide_mm : side_win));
                        // 通过判据：车轴越过近面 + 后悬(65) + 余量(20)，
                        //   且正前空（或段消失）⇒ 进入 m-line 回线
                        // ★V4.1d★ ①离开门槛用几何量 fwd_need（Dm+85），不再固定 360
                        //   ②正前"看不见"必须连续 3 圈才算，不接受单帧丢段放行
                        // ★V4.1e★ 回线前必须确认【障碍侧那一整片】都够远：
                        //   d_obs = min(正前该半区, 斜侧窗, 正侧窗)。
                        //   20:09 现场：side_win(正侧 160~200°) 已 ≥300 就放行，
                        //   可是箱子还在 135~155° 的斜侧窗里贴着，
                        //   紧接着 S_RET2 的 2φ 甩头把车头扫进箱子（w=52）。
                        // ★V4.2c★ 先补航向：A_FWD_HOLD 在地上只修得掉约 1/6 的误差，
                        //   实测偏出目标 >8° 就由 FSM 原地补转到目标（最多 8 次）。
                        if (yaw_ok &&
                            (yaw_diff(yaw_now, yaw_out_tgt) > AIM_TOL_CNT) &&
                            (hdg_fix_cnt < 4'd8)) begin
                            hdg_fix_cnt <= hdg_fix_cnt + 4'd1;
                            turn_retry  <= 4'd0;
                            dbg_why     <= 8'h3C;
                            st          <= S_T_OUT;
                        // ★V4.33e★ 这里【不再】有 a1_pass：
                        //   14:07 现场 w=33 出现在 lat=158、箱子还在 127~134°/375mm，
                        //   车压根没走过去。原因是 df=740>500（车转了40°箱子出了正前窗）
                        //   + a1_pass 假成立（正侧160~200本来就空）⇒ 两个条件合起来骗过了门。
                        //   回到纯几何判据：后轮轴越过障碍近面(Dm+85) 才许回线。
                        end else if ((fwd_acc >= fwd_need) && !ret_hold &&
                            ((side_prox_r == 16'hFFFF) || (side_prox_r >= 16'd300)) &&
                            ((d_obs == 16'hFFFF) || (d_obs >= SIDE_MIN_MM)) &&
                            ((front_gone_cnt >= FRONT_GONE_K) ||
                             (front_mm > (D_ARM_MM + 16'd50)))) begin
                            dbg_why <= 8'h33;
                            lat_rem <= lat_acc;
                            // ★V4.35a★ 每次进回线都要重置"回线阶段状态"：
                            //   ret_piv 只在 S_T_OUT 里清过，而 S_FOLLOW 是直接跳 S_RET2 的
                            //   ⇒ 第二个障碍会带着上一轮的 ret_piv=1 进来，
                            //   于是 ret_need=0、不做入口转身，车头还挂在 +35° 就开始回线 ⇒ 撞角。
                            ret_piv       <= 1'b0;
                            ret_block_cnt <= 4'd0;
                            st <= (lat_acc > RET_TOL_MM) ? S_RET2 : S_T_END;
                        end else begin
                            // ★V4.35b★ 滞回解除：只有"正前 >500 且障碍侧（含斜前窗）>300"
                            //   才算真的清空，这时才允许下一次回线。
                            if (front_v && (front_mm > 16'd500) &&
                                ((side_prox_r == 16'hFFFF) || (side_prox_r >= 16'd300)))
                                ret_hold <= 1'b0;
                            // ★V4.31a★ 贴边 + 侧距 P 控制（拆两拍：一拍里放两个乘法会把 pend_yaw 拉爆）
                            //   目标侧距 = HALF_W_M(200)；只在 side 这一侧调角（0..60°），不翻向
                            case (fol_st)
                            3'd0: begin
                                // ★V4.35h★ 侧距 P 控制回退到 V4.31a 的【纯正侧窗】口径。
                                //   原因：V4.35c/e 把 side_prox_r 改成 min(正侧窗, 障碍侧斜前窗)，
                                //   而车斜着走时斜前窗 45/135°±10 看到的是箱子【远端】，
                                //   读数天然 300~350 ⇒ 永远走"离得远就收角"分支，φ 从 30 一路收到 5~0，
                                //   "太近就加角"这条一次都进不去。
                                //   而 2026-10-21 02:39 用户点名「侧面即将撞到时会自己往外拐一下，
                                //   这个确实做的不错」指的正是这一条 ⇒ 必须让它能重新触发。
                                //   side_prox_r（含斜前窗）仍保留给 side_seen / ret_corner_near / 回线判据用。
                                if ((side_win == 16'hFFFF) || (side_win > 16'd600)) begin
                                    psip_keep <= 1'b1;                // 未进侧面窗 ⇒ 保持触发角
                                    psip_far  <= 1'b0;
                                    base_mag  <= 16'd0;
                                end else if (side_win >= HALF_W_M) begin
                                    psip_keep <= 1'b0;
                                    psip_far  <= 1'b1;               // 离得远 ⇒ 收角
                                    base_mag  <= ((side_win - HALF_W_M) * 16'd102) >> 10;
                                end else begin
                                    psip_keep <= 1'b0;
                                    psip_far  <= 1'b0;               // 太近 ⇒ 加角
                                    base_mag  <= ((HALF_W_M - side_win) * 16'd102) >> 10;
                                end
                                // ★V4.33c★ 用当前 side_win 直接判，不要用 psip_keep——
                                //   那是本拍才写进寄存器的，用它会让 side_seen 慢一整步
                                if (side_prox_seen) side_seen <= 1'b1;  // ★V4.35c★ 进过侧窗（含斜前窗）了
                                fol_st <= 3'd1;
                            end
                            3'd1: begin
                                cur_mag = act_k_r * 16'd5;           // 当前实测航向幅值（度）
                                if (psip_keep) begin
                                    // ★V4.32a★ 侧窗失效有两种意思：①还没接近到障碍侧面
                                    //   ⇒ 保持触发角；②已经绕过它、侧窗空了 ⇒ 保持【当前角】，
                                    //   绝不能拽回触发角（旧版把 0° 一把拽到 35°，车随即拐进左侧障碍）
                                    psi_mag = side_seen ? cur_mag : (phi_k * 16'd5);
                                end else if (psip_far) begin
                                    psi_mag = (cur_mag > base_mag) ? (cur_mag - base_mag) : 16'd0;
                                end else begin
                                    psi_mag = cur_mag + base_mag;
                                end
                                // ★V4.33a★ 收角门控：车轴没过障碍前端之前不许收角（保持当前角），
                                //   过了之后每步最多收 5°。旧版侧距一变大就一次收 30°，车头
                                //   会扫回障碍正面（02:36 日志 53.8→55.4s，phi 35→0 只用 1.6s）
                                if (!a0_pass) begin
                                    if (psi_mag < cur_mag) psi_mag = cur_mag;
                                end else if ((psi_mag + 16'd5) < cur_mag) begin
                                    psi_mag = cur_mag - 16'd5;
                                end
                                if (psi_mag > 16'd60) psi_mag = 16'd60;
                                psi_r   <= psi_mag[5:0];
                                fol_st  <= 3'd2;
                            end
                            3'd2: begin
                                // ★V4.32a★ 第三拍：只做 6×常数乘法，和 pend_yaw 分开
                                off_s   <= {10'd0, psi_r} * 16'd182;
                                off_neg <= (~({10'd0, psi_r} * 16'd182)) + 16'd1;
                                fol_st  <= 3'd3;
                            end
                            default: begin
                                phi_deg  <= {10'd0, psi_r};
                                pend_act <= A_FWD_HOLD; pend_n <= STEP_CNT;
                                pend_yaw <= yaw_base + (side ? off_neg : off_s);
                                pend_to_q<= 1'b0; pend_slow <= 1'b1;
                                load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                                fol_st   <= 3'd0;
                            end
                            endcase
                        end
                    end else if (ext_done) begin
                        busy <= 1'b0; wd <= 32'd0;
                        trav_acc <= trav_acc + arc_mm;
                        fwd_acc  <= fwd_acc + arc_cos_w[31:0];
                        lat_acc  <= lat_acc + arc_sin_w[31:0];
                    end else if (wd >= STEP_TO_CYC) begin
                        dbg_why <= 8'h37; st <= S_FAIL;
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                //==============================================================
                // ★V3.0★ S_RET2：m-line 回线 PD + 左右两侧安全项
                //   side=0 车在左 ⇒ 往右回(负角)；side=1 反之
                //   左侧太近 ⇒ 往右(负)；右侧太近 ⇒ 往左(正)
                //==============================================================
                S_RET2: begin
                    avoid_active <= 1'b1;
                    if (stall_hit) begin
                        dbg_why <= 8'h42; fail_want <= 1'b1; st <= S_T_BACK;
                    end else if (!busy) begin
                        if (lat_acc <= RET_TOL_MM) begin
                            // ★V4.33d★ lat_fix（crush 恢复后补线）时不要再跳回 S_FOLLOW：
                            //   那时候没有新的 arm、side/phi_k/yaw_base 都是上一轮的旧值，
                            //   跳回去等于用旧状态重新绕障。直接收工交回任务。
                            if (lat_fix) begin
                                lat_fix <= 1'b0;
                                dbg_why <= 8'h56;
                                st      <= S_IDLE;
                            // ★V4.0d★ 正前没清就不算完成：回 S_FOLLOW 继续绕，
                            //   避免 S_RET2 回到线后 S_RESUME 看到箱子判 w=27 停在侧面。
                            end else if (front_v && (front_mm < (D_ARM_MM + 16'd50))) begin
                                dbg_why <= 8'h54; st <= S_FOLLOW; wd <= 32'd0;
                            end else begin
                                dbg_why <= 8'h43; st <= S_T_END; wd <= 32'd0;
                            end
                        end else if (!ret_st) begin
                            // 回线基准：side=0 车在左 ⇒ 往右回；side=1 反之
                            // ★V4.2e★ 回线角放平：lat/8(45°) → lat/24(25°)。
                            //   闭环模型 40 组几何：陡回线 14 个撞箱，放平后回线撞箱 0 个。
                            //   陡切会在"还没过箱子远端"时压回中线，正好横穿箱子后半段。
                            // ★V4.33a★ 回线角 = atan(lat_rem / L_look)，用比值分三档近似
                            //   lat/L ≤ 0.5 ⇒ 14°；≤0.9 ⇒ 20°；>0.9 ⇒ 26°（下面有 25° 钳位，
                            //   所以 26 档实际是 25°，这里保留 26 只为可读性）
                            //   旧版是 lat/24 的经验常数（lat=395 时只有 16°，回线腿要 1.4m）
                            // ★V4.34d★ 用户实测"回线角太小、几乎没往回走"⇒ 三档整体加大
                            //   14/20/26 → 20/26/32（对应 lat/L ≤0.5 / ≤0.9 / >0.9）
                            // ★V4.35i★ 三档再整体加 10°：20/26/32 → 30/34/38，钳位 34→40。
                            //   15:50 现场 lat=327 落最低档 20°，横向速度只有 0.34v，
                            //   8.4 秒也没回到线。30° 把横向速度提到 0.50v（约 1.5 倍）。
                            base_mag = 16'd30;
                            if (lat_acc[15:0] > L_look16)                    base_mag = 16'd38;
                            else if (lat_m10 > look_m9)                      base_mag = 16'd34;
                            // ★V4.35i★ 这里原来还有一级 `if (base_mag > 40) base_mag = 40;`，
                            //   但三档最大只到 38 ⇒ 该钳位是死代码，删掉省一级 16 位比较器
                            //   （首版 PnR 最差余量只剩 +0.021ns，瓶颈就在这条路上）。
                            // ★V4.34e★ 删掉 V4.33a 那条"没过远端就钳 16°"的检查：
                            //   它要求 fwd_acc ≥ y_far+260 ≈ 1010，而回线判据只要 fwd_acc ≥ Dm-100 ≈ 330
                            //   ⇒ 永远达不到 ⇒ 回线角被永久钳在 16°，三档加大等于没做。
                            //   16:10 现场实证：lat 411→77 花了 11.4 秒（29mm/s），就是这个钳制。
                            //   "该不该回线"已经由 fwd_need 判据管，这里不该再卡一次。
                            // 左右两侧安全项：左侧太近 ⇒ 往右；右侧太近 ⇒ 往左
                            safe_l = (hist_side_l_mm < HALF_W_M)
                                     ? (((HALF_W_M - hist_side_l_mm) * 16'd102) >> 10) : 16'd0;
                            safe_r = (hist_side_r_mm < HALF_W_M)
                                     ? (((HALF_W_M - hist_side_r_mm) * 16'd102) >> 10) : 16'd0;
                            mag_l = (side ? base_mag : 16'd0) + safe_r;
                            mag_r = (side ? 16'd0 : base_mag) + safe_l;
                            if (mag_r >= mag_l) begin
                                psi_right = 1'b1; psi_mag = mag_r - mag_l;
                            end else begin
                                psi_right = 1'b0; psi_mag = mag_l - mag_r;
                            end
                            if (psi_mag > 16'd40) psi_mag = 16'd40;   // ★V4.35i★ 34→40，别把 38 档削成 34
                            if ((psi_mag > 16'd0) && (psi_mag < 16'd6)) psi_mag = 16'd6;
                            // ★V3.0e★ 安全项不许把回线方向翻到外侧：
                            //   side=0 只允许右转/直行；side=1 只允许左转/直行。
                            if (((side == 1'b0) && (psi_right == 1'b0)) ||
                                ((side == 1'b1) && (psi_right == 1'b1)))
                                psi_mag = 16'd0;
                            // 第一拍只把结果寄存下来，第二拍再发段
                            phi_deg <= psi_mag;
                            ret_right <= psi_right;
                            ret_need  <= ((!ret_piv) && (lat_acc > 32'd150) && (psi_mag > 16'd0));
                            ret_st    <= 1'b1;
                        end else begin
                            // 第二拍：phi_deg 已是寄存器，×182 只剩一级常数乘法
                            off_s   = phi_deg * 16'd182;
                            off_neg = (~off_s) + 16'd1;
                            ret_st  <= 1'b0;
                            // ★V3.0e★ 回线入口 lat>150 先原地转到回线角（止一次），
                            //   再 A_FWD_HOLD 走；这就是用户允许的“停一下再转”。
                            // ★V4.35★ 边走边测：发这一步之前先看正前。
                            //   正前 <400 说明回线路径被堵，硬按原角走就是拿车头角去撞
                            //   （17:31 连续三次撞角）。此时交回 S_FOLLOW 继续往外让，
                            //   等 front 清出来再回线；最多 8 次，防止来回死循环。
                            if (((front_v && (front_mm < 16'd400)) || ret_corner_near) &&
                                (ret_block_cnt < 4'd8)) begin
                                ret_block_cnt <= ret_block_cnt + 4'd1;
                                ret_hold      <= 1'b1;         // ★V4.35b★ 滞回：清空前不再回线
                                fol_st        <= 3'd0;      // S_FOLLOW 的相位要归零
                                dbg_why       <= 8'h57;
                                st            <= S_FOLLOW;
                                busy          <= 1'b0;
                            end else if (ret_need) begin
                                pend_act <= A_YAW_ABS;
                                // ★V4.2d★ 原先这里写死 90° 兜底，而目标是 yaw_base±psi（实际 2φ）。
                                //   φ 不是 45° 时，IMU 一掉线就会转过头。改成按"当前航向到目标的
                                //   实测残差"折算编码器弧长，另加 5° 余量。
                                // ★V4.2g★ 时序：不要从"sad_side → psi_mag → off_s → yaw_diff"这条长链折算
                                //   （PnR 实测 slack -6.7ns）。IMU 掉线兜底只需要"大概转多少"：
                                //   回线本来就是从 +φ 转到 −φ，即 2φ；用 4 位 phi_k 乘常数，路径极短。
                                pend_n   <= ({28'd0, phi_k} * 32'd11290)   // 2φ (度) × 1129
                                            + 32'd5645;                    // +5° 余量
                                pend_yaw <= yaw_base + (ret_right ? off_neg : off_s);
                                ret_tgt  <= yaw_base + (ret_right ? off_neg : off_s);
                                // ★V4.1d★ 回线入口这一下是 2φ 大角度甩头，改 75% 速
                                pend_to_q<= 1'b0; pend_slow <= 1'b1;
                                load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                            end else if (ret_piv && yaw_ok &&
                                         (yaw_diff(yaw_now, ret_tgt) > RET_AIM_TOL) &&
                                         (ret_fix_cnt < 4'd8)) begin
                                // ★V4.34a★ 回线每步补航向：偏出 8° 就原地转回目标角，
                                //   再继续走。A_FWD_HOLD 自己追不上（只能修 1/6）。
                                ret_fix_cnt <= ret_fix_cnt + 4'd1;
                                ret_corr    <= 1'b1;          // 这一段是纯转，不计平移
                                pend_act <= A_YAW_ABS;
                                pend_n   <= 32'd338700;       // 30° 兜底（30×11290），实际由 IMU 收角
                                pend_yaw <= ret_tgt;
                                pend_to_q<= 1'b0; pend_slow <= 1'b1;
                                load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                            end else begin
                                ret_piv  <= 1'b1;
                                pend_act <= A_FWD_HOLD; pend_n <= STEP_CNT;
                                pend_yaw <= yaw_base + (ret_right ? off_neg : off_s);
                                ret_tgt  <= yaw_base + (ret_right ? off_neg : off_s);
                                pend_to_q<= 1'b0; pend_slow <= 1'b1;
                                load_pend<= 1'b1; busy <= 1'b1; wd <= 32'd0;
                            end
                        end
                    end else if (ext_done) begin
                        busy <= 1'b0; wd <= 32'd0;
                        if (!ret_piv) begin
                            ret_piv <= 1'b1;        // 这是入口的原地转，不计平移
                        end else if (ret_corr) begin
                            ret_corr <= 1'b0;       // ★V4.34a★ 补航向的原地转，同样不计平移
                        end else begin
                            trav_acc <= trav_acc + arc_mm;
                            fwd_acc  <= fwd_acc + arc_cos_w[31:0];
                            lat_rem  <= (lat_rem > arc_sin_w[31:0]) ?
                                        (lat_rem - arc_sin_w[31:0]) : 32'd0;
                            lat_acc  <= (lat_acc > arc_sin_w[31:0]) ?
                                        (lat_acc - arc_sin_w[31:0]) : 32'd0;
                        end
                    end else if ((trav_acc - fwd_acc) > EXTRA_MAX_MM) begin
                        dbg_why <= 8'h34; fail_want <= 1'b1; st <= S_T_BACK;
                    end else if (to_cnt > TO_CYC) begin
                        dbg_why <= 8'h35; fail_want <= 1'b1; st <= S_T_BACK;
                    end else if (wd >= PIVOT_TO_CYC) begin
                        dbg_why <= 8'h44; fail_want <= 1'b1; st <= S_T_BACK;
                    end else begin
                        wd <= wd + 32'd1;
                    end
                end

                default: st <= S_IDLE;
                endcase
            end
        end
    end

endmodule
