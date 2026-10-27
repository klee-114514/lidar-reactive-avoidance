//=============================================================================
// tb_lidar_parse.v —— 验证雷达感知层（5 字节节点 + 五扇区 + 控制口径）【final_leida v1.4】
//-----------------------------------------------------------------------------
// 要证明这些事：
//   ① 角度/距离换算正确（角度 = ((b2<<8|b1)>>1)/64，距离 = (b4<<8|b3)/4）
//   ② 五个扇区（★v1.9 按实测：正前 90°±20°、车左前 45°±10°、车右前 135°±10°、
//      0° 窗口 = 车正左、180° 窗口 = 车正右★）取的都是【最小值】，扇区外的点不进来
//      —— 本 tb 同时是"侧前扇区左右不能反"的回归用例（v1.8 及以前 45°/135° 是镜像的，
//         会让 avoid_fsm 朝障碍那一侧选边）
//   ③ S 位触发"新一圈"：上一圈结果锁存、累加器清零；第一圈只当"从此开始"不锁存
//   ④ 错位字节（b0 的 bit1 ≠ ~bit0）与 C 位错（b1 的 bit0 = 0）都会被丢掉
//   ⑤ 距离过滤：<150mm、>12000mm、以及规范说的"无效点距离字段 = 0"都不计入
//   ⑥ 控制口径：正前【减 SENS_FWD_MM】折算到车头前缘；扇区无点 ⇒ 折算 SENS_FAR_MM
//   ⑦ 断流：STALE_CYC 没成圈 ⇒ scan_fresh=0、dist_f_v=0、dist_f_mm=0、echo_to=1；
//      数据回来以后自动恢复
//   ⑧ 盯视：watch_far_cnt 只在 watch_en=1 时按圈累加、看见近的就清零、饱和 PASS_K
//   ⑨ 脉冲：每锁存一圈给一个 scan_done_p；整圈零有效点给一个 rev_no_pt
//
// 造数据用"反推"的办法：想发 (角度, 距离) 就按协议把字节拼出来，tb 不依赖被测算法。
//   ★注意 C 校验位★：u16 = (ang_q6<<1)|1，它的 bit0 就是规范要求的恒 1 校验位。
//
// 运行（final_leida/sim/ms 下）：
//   vlib work
//   vlog -work work <rtl 绝对路径>\lidar_parse.v tb_lidar_parse.v
//   vsim -c -do "run -all; quit -f" work.tb_lidar_parse
//=============================================================================
`timescale 1ns/1ps

module tb_lidar_parse;

    localparam integer STALE_CYC = 2000;    // 40µs（仿真是把断流判据缩短来测）

    reg clk = 1'b0;
    always #10 clk = ~clk;      // 50MHz

    reg         rst_n    = 1'b0;
    reg  [7:0]  b_data   = 8'd0;
    reg         b_valid  = 1'b0;
    reg         hdr_done = 1'b0;
    reg  [6:0]  hist_rd_addr = 7'd0;
    wire [15:0] hist_rd_data;
    wire [15:0] hist_side_l, hist_side_r;
    reg         watch_en = 1'b0;
    reg  [7:0]  watch_ang= 8'd0;

    wire [15:0] front_mm, left_mm, right_mm, side_l_mm, side_r_mm;
    wire        front_ok, left_ok, right_ok, side_l_ok, side_r_ok;
    wire [15:0] scan_pts, last_ang, last_dist;
    wire [15:0] node_cnt, scan_cnt, bad_cnt;
    wire        w_scan_done, w_rev_no_pt, w_scan_fresh;
    wire [15:0] dist_f_mm, prof_l_mm, prof_r_mm, watch_mm;
    wire        dist_f_v, w_echo_to, prof_l_v, prof_r_v, watch_v;
    wire [7:0]  watch_far_cnt;
    // ---- ★2a：障碍角向连续段 + 正前左右半区★ ----
    wire [15:0] obs_a0, obs_d0, obs_a1, obs_d1, obs_dmin, obs_amin;
    wire        obs_valid;
    wire [15:0] front_l_mm, front_r_mm;
    wire        front_l_ok, front_r_ok;

    lidar_parse #(.STALE_CYC(STALE_CYC)) dut (
        .clk(clk), .rst_n(rst_n),
        .b_data(b_data), .b_valid(b_valid), .hdr_done(hdr_done),
        .watch_en(watch_en), .watch_ang(watch_ang),
        .front_mm(front_mm), .left_mm(left_mm), .right_mm(right_mm),
        .side_l_mm(side_l_mm), .side_r_mm(side_r_mm),
        .front_ok(front_ok), .left_ok(left_ok), .right_ok(right_ok),
        .side_l_ok(side_l_ok), .side_r_ok(side_r_ok),
        .scan_pts(scan_pts), .last_ang_deg(last_ang), .last_dist_mm(last_dist),
        .node_cnt(node_cnt), .scan_cnt(scan_cnt), .bad_cnt(bad_cnt),
        .scan_done_p(w_scan_done), .rev_no_pt(w_rev_no_pt), .scan_fresh(w_scan_fresh),
        .dist_f_mm(dist_f_mm), .dist_f_v(dist_f_v), .echo_to(w_echo_to),
        .prof_l_mm(prof_l_mm), .prof_r_mm(prof_r_mm),
        .prof_l_v(prof_l_v), .prof_r_v(prof_r_v),
        .watch_mm(watch_mm), .watch_v(watch_v), .watch_far_cnt(watch_far_cnt),
        .obs_a0_deg(obs_a0), .obs_d0_mm(obs_d0),
        .obs_a1_deg(obs_a1), .obs_d1_mm(obs_d1),
        .obs_dmin_mm(obs_dmin), .obs_amin_deg(obs_amin), .obs_valid(obs_valid),
        .front_l_mm(front_l_mm), .front_r_mm(front_r_mm),
        .front_l_ok(front_l_ok), .front_r_ok(front_r_ok),
        .hist_rd_addr(hist_rd_addr), .hist_rd_data(hist_rd_data)
        ,.hist_side_l_mm(hist_side_l), .hist_side_r_mm(hist_side_r)
    );

    // ---------------- 脉冲计数（扫描若干拍里的 1 拍脉冲）----------------
    integer done_pulses = 0, nopt_pulses = 0;
    always @(posedge clk) begin
        if (w_scan_done) done_pulses = done_pulses + 1;
        if (w_rev_no_pt) nopt_pulses = nopt_pulses + 1;
    end

    // ---------------- 按协议拼一个节点并送出去 ----------------
    //   ang_x4：角度，单位 1/4 度（90° 就给 360）；dist_mm：距离，单位 mm
    task send_node(input [15:0] ang_x4, input [15:0] dist_mm, input is_start);
        integer    i;
        reg [15:0] ang_q6, dist_q2, u16;
        reg [7:0]  bb [0:4];
        begin
            ang_q6  = ang_x4 * 16;                        // 1/4 度 → 1/64 度
            dist_q2 = dist_mm * 4;                        // mm → mm×4
            u16     = (ang_q6 << 1) | 16'h0001;           // ★bit0 = C 校验位，恒 1★
            bb[0] = {6'b111111, ~is_start, is_start};     // b0：质量 + ~S + S
            bb[1] = u16[7:0];                             // b1：角度低 7 位 + C
            bb[2] = u16[15:8];                            // b2：角度高 8 位
            bb[3] = dist_q2[7:0];                         // b3/b4：距离（mm×4）
            bb[4] = dist_q2[15:8];
            for (i = 0; i < 5; i = i + 1) begin
                @(posedge clk);
                b_data  <= bb[i];
                b_valid <= 1'b1;
            end
            @(posedge clk);
            b_valid <= 1'b0;
            @(posedge clk);
        end
    endtask

    // 人工送一个字节（用来插错位字节）
    task send_raw(input [7:0] b);
        begin
            @(posedge clk);
            b_data  <= b;
            b_valid <= 1'b1;
            @(posedge clk);
            b_valid <= 1'b0;
            @(posedge clk);
        end
    endtask

    integer errors = 0;
    task chk16(input [15:0] got, input [15:0] exp, input [8*52:1] msg);
        begin
            if (got !== exp) begin
                $display("  FAIL: %0s  got=%0d(0x%04h)  期望=%0d(0x%04h)", msg, got, got, exp, exp);
                errors = errors + 1;
            end
        end
    endtask
    task chk1(input got, input exp, input [8*52:1] msg);
        begin
            if (got !== exp) begin
                $display("  FAIL: %0s  got=%b  期望=%b", msg, got, exp);
                errors = errors + 1;
            end
        end
    endtask
    task chk_hist(input [6:0] a, input [15:0] exp, input [8*52:1] msg);
        begin
            hist_rd_addr = a;
            @(posedge clk); @(posedge clk);
            chk16(hist_rd_data, exp, msg);
        end
    endtask

    initial begin
        #200; rst_n = 1'b1; #200;

        // =========================================================================
        // 第 1 圈：五扇区最小值 + 控制口径折算 + 扇区外的点不进来
        // =========================================================================
        $display("\n---- 第 1 圈：正前 2500 / 左前 1800 / 右前 900 / 正右 800 / 正左 650 ----");
        send_node(16'd360, 16'd9999, 1'b1);   // 第 1 个 S：只当"从此开始"，不锁存
        send_node(16'd360, 16'd2500, 1'b0);   // 90°  正前（最小值）
        send_node(16'd180, 16'd1800, 1'b0);   // ★45°  车左前（实测方位）★
        send_node(16'd540, 16'd900,  1'b0);   // ★135° 车右前（实测方位）★
        send_node(16'd360, 16'd7000, 1'b0);   // 90°  更远，不该覆盖最小值
        send_node(16'd0,   16'd800,  1'b0);   // 0°   窗口（实测=车正左；变量名 side_r 是历史遗留）
        send_node(16'd720, 16'd650,  1'b0);   // 180° 窗口（实测=车正右；变量名 side_l 是历史遗留）
        // ★V2.17★：侧窗从 ±10° 放宽到 ±20° 后，这个 20° 的点**落在 0° 窗口里了**
        //   （不再是"扇区外"）⇒ 下面 side_r_mm/watch_mm 的期望值随之改成 300。
        send_node(16'd80,  16'd300,  1'b0);   // 20°  现在算进 0° 窗口(±20°)
        watch_en  = 1'b1; watch_ang = 8'd0;   // 盯 0° 窗口（=实测车正左；"向左绕"用的正是它 ✓）
        send_node(16'd360, 16'd3000, 1'b1);   // 第 2 个 S ⇒ 锁存第 1 圈
        #2000;
        chk16(front_mm,   16'd2500, "T1 front（取 2500 不是 7000）");
        chk16(left_mm,    16'd1800, "T1 left（车左前 45°）");
        chk16(right_mm,   16'd900,  "T1 right（车右前 135°）");
        chk16(side_r_mm,  16'd300,  "T1 side_r（0° 窗口 ±20° ⇒ 取到 20°/300mm）");
        chk16(side_l_mm,  16'd650,  "T1 side_l（180° 窗口 = 实测车正右）");
        chk16(scan_pts,   16'd8,    "T1 scan_pts（8 个有效点）");
        chk16(scan_cnt,   16'd1,    "T1 scan_cnt（锁存 1 圈）");
        chk1 (front_ok,   1'b1,     "T1 front_ok");
        chk1 (side_l_ok,  1'b1,     "T1 side_l_ok");
        // 控制口径
        chk16(dist_f_mm,  16'd2350, "T1 dist_f（2500 − SENS_FWD 150 = 2350）");
        chk1 (dist_f_v,   1'b1,     "T1 dist_f_v");
        chk16(prof_l_mm,  16'd1800, "T1 prof_l");
        chk16(prof_r_mm,  16'd900,  "T1 prof_r");
        chk16(watch_mm,   16'd300,  "T1 watch_mm（watch_ang=0；±20° 窗口 ⇒ 300）");
        chk1 (watch_v,    1'b1,     "T1 watch_v（watch_en=1）");
        chk16({8'd0, watch_far_cnt}, 16'd0, "T1 watch_far_cnt（800 < D_PASS 900 ⇒ 0）");
        // ★V4.0a★ 72 扇区 × 5° 直方图：第 1 圈各扇区最小值
        chk_hist(7'd18, 16'd2500, "hist[18] 90deg 最小 2500");
        chk_hist(7'd9,  16'd1800, "hist[9] 45deg = 1800");
        chk_hist(7'd27, 16'd900,  "hist[27] 135deg = 900");
        chk_hist(7'd0,  16'd800,  "hist[0] 0deg = 800");
        chk_hist(7'd36, 16'd650,  "hist[36] 180deg = 650");
        chk_hist(7'd4,  16'd300,  "hist[4] 20deg = 300");
        chk16(hist_side_l, 16'd300, "hist_side_l 0deg±20(含20deg边界) = 300");
        chk16(hist_side_r, 16'd650, "hist_side_r 180deg±20 = 650");

        // =========================================================================
        // 第 2 圈：错位字节 + 盲区点 + 无效点（距离字段 0）
        // =========================================================================
        $display("\n---- 第 2 圈：错位字节 / 100mm 盲区 / 距离 0 无效点 / 正前 1200 ----");
        send_node(16'd360, 16'd1200, 1'b0);   // 正前 1200（本圈最小值）
        send_raw(8'hFF);                      // 错位：bit1 不反相 ⇒ 丢掉
        send_node(16'd360, 16'd100,  1'b0);   // <150mm 盲区 ⇒ 不计入
        send_node(16'd360, 16'd4000, 1'b0);   // 正前 4000
        send_node(16'd360, 16'd0,    1'b0);   // ★规范：无效点距离字段为 0 ⇒ 不计入★
        send_node(16'd360, 16'd3000, 1'b1);   // 第 3 个 S ⇒ 锁存第 2 圈
        #2000;
        chk16(front_mm, 16'd1200, "T2 front（错位/盲区/无效点都不该进来）");
        chk16(bad_cnt,  16'd1,    "T2 bad_cnt（记录 1 个错位字节）");
        chk16(scan_pts, 16'd3,    "T2 scan_pts（1200/4000/3000 三点）");
        chk16(dist_f_mm,16'd1050, "T2 dist_f（1200 − 150）");

        // =========================================================================
        // 第 3 圈：C 校验位为 0 的节点必须被丢掉
        // =========================================================================
        $display("\n---- 第 3 圈：C 位错的节点（b1 bit0=0）⇒ 丢字节、不算点 ----");
        begin : c_bit_bad
            integer n0, b0cnt;
            n0    = node_cnt;
            b0cnt = bad_cnt;
            // 手工拼一个"角度低字节 bit0 = 0"的节点：01 02 2D 00 00
            send_raw(8'h01);   // b0：S=1、~S=0、质量 0
            send_raw(8'h02);   // b1：★bit0=0 ⇒ C 位错，应被丢掉★
            send_raw(8'h2D);   // 后面的字节会被当成新节点的开头，但都过不了校验
            send_raw(8'h00);
            send_raw(8'h00);
            #500;
            chk16(node_cnt, n0[15:0],          "T3 C 位错的节点不该被算作点");
            if (bad_cnt <= b0cnt) begin
                $display("  FAIL: T3 bad_cnt 没有增加（%0d → %0d）", b0cnt, bad_cnt);
                errors = errors + 1;
            end
        end
        hdr_done = 1'b1; @(posedge clk); hdr_done = 1'b0; @(posedge clk);  // 真头重新对齐

        // =========================================================================
        // 第 4 圈：整圈只有无效点 ⇒ 扇区无点=FFFF、rev_no_pt 脉冲
        // =========================================================================
        $display("\n---- 第 4 圈：整圈都是无效点 ⇒ front=FFFF + rev_no_pt ----");
        send_node(16'd360, 16'd0, 1'b1);      // S 点自己就是无效点（距离 0）
        send_node(16'd360, 16'd0, 1'b0);      // 再一个无效点
        begin : no_point
            integer n0;
            n0 = nopt_pulses;
            send_node(16'd360, 16'd3000, 1'b1);   // S ⇒ 锁存"空圈"
            #2000;
            chk16(front_mm, 16'd65535, "T4 front（本圈无有效点 ⇒ FFFF）");
            chk1 (front_ok, 1'b0,      "T4 front_ok（无点）");
            chk16(scan_pts, 16'd0,     "T4 scan_pts（0 个有效点）");
            chk16(dist_f_mm,16'd4000,  "T4 dist_f（无点 ⇒ 折算 SENS_FAR 4000）");
            if (nopt_pulses !== n0 + 1) begin
                $display("  FAIL: T4 rev_no_pt 脉冲数 %0d，期望 %0d", nopt_pulses, n0 + 1);
                errors = errors + 1;
            end
        end

        // =========================================================================
        // 第 5 圈：盯视计数（连续 3 圈很远 ⇒ 饱和 3；看见近的 ⇒ 清零；watch_en=0 ⇒ 0）
        // =========================================================================
        $display("\n---- 第 5 圈起：watch_far_cnt 门控与清零 ----");
        watch_en = 1'b1; watch_ang = 8'd0;    // 盯正右
        // 连续 3 圈正右都是 3000mm（>= D_PASS 900）
        begin : far3
            integer i2;
            for (i2 = 0; i2 < 3; i2 = i2 + 1) begin
                send_node(16'd360, 16'd3000, 1'b1);   // S：锁存上一圈 + 更新计数
                send_node(16'd0,   16'd3000, 1'b0);   // 正右 3000（远）
            end
            send_node(16'd360, 16'd3000, 1'b1);       // 再锁存一次
            #2000;
            chk16({8'd0, watch_far_cnt}, 16'd3, "T5 watch_far_cnt 应饱和在 3");
        end
        // 正右突然近（500 < 900）⇒ 清零
        send_node(16'd360, 16'd3000, 1'b1);
        send_node(16'd0,   16'd500,  1'b0);
        send_node(16'd360, 16'd3000, 1'b1);
        #2000;
        chk16({8'd0, watch_far_cnt}, 16'd0, "T5 看见近的 ⇒ watch_far_cnt 清零");
        chk16(watch_mm, 16'd500, "T5 watch_mm 应跟着变近");
        // watch_en=0 ⇒ 不数、watch_v=0
        watch_en = 1'b0;
        send_node(16'd360, 16'd3000, 1'b1);
        send_node(16'd0,   16'd3000, 1'b0);
        send_node(16'd360, 16'd3000, 1'b1);
        #2000;
        chk16({8'd0, watch_far_cnt}, 16'd0, "T5 watch_en=0 ⇒ far_cnt 保持 0");
        chk1 (watch_v, 1'b0, "T5 watch_en=0 ⇒ watch_v=0");

        // =========================================================================
        // 第 6 圈：断流（STALE_CYC 没成圈）
        // =========================================================================
        $display("\n---- 断流测试：超过 STALE_CYC 没有成圈 ----");
        #100_000;                                   // 100µs >> STALE_CYC(40µs)
        chk1 (w_scan_fresh, 1'b0, "T6 断流 ⇒ scan_fresh=0");
        chk1 (dist_f_v,     1'b0, "T6 断流 ⇒ dist_f_v=0");
        chk16(dist_f_mm,    16'd0, "T6 断流 ⇒ dist_f 强制 0（当被堵）");
        chk1 (w_echo_to,    1'b1, "T6 断流 ⇒ echo_to=1（顶层据此判 sensor_fault）");
        // 数据回来 ⇒ 自动恢复
        send_node(16'd360, 16'd2000, 1'b1);
        send_node(16'd360, 16'd2000, 1'b0);
        send_node(16'd360, 16'd2500, 1'b1);         // 锁存一圈
        #2000;
        chk1 (w_scan_fresh, 1'b1, "T7 数据回来 ⇒ scan_fresh=1");
        chk1 (dist_f_v,     1'b1, "T7 数据回来 ⇒ dist_f_v=1");
        chk1 (w_echo_to,    1'b0, "T7 数据回来 ⇒ echo_to=0");
        chk16(dist_f_mm,    16'd1850, "T7 dist_f = 2000 − 150");

        // =========================================================================
        // 第 8 圈：★2a 障碍角向连续段（前半球最近的那一段）+ 正前左右半区★
        //   造一个"箱子"：车正前方 600mm、宽 ±150mm ⇒ 方位 76°~104°、距离 600~618mm
        //   再在左侧 20°~40° 造一面 2000mm 的"墙"，验证"最近的那一段"才会被选中
        //   （方位约定：0°=车正左、90°=正前、180°=车正右，与实车一致）
        // =========================================================================
        //   ★点的方位间隔必须 ≤ 6°（RUN_GAP_Q6），否则会被判成"两段"——真雷达到处是点，
        //     这里按 4°/点 喂，模拟"平整的面"★
        $display("\n---- 2a 障碍段测试：箱 600mm（±150mm 宽）+ 左侧 2000mm 墙 ----");
        send_node(16'd0,   16'd2000, 1'b1);    // 新一圈的 S（0°，先当"从此开始"）
        send_node(16'd80,  16'd2000, 1'b0);    // 20°  墙
        send_node(16'd100, 16'd2000, 1'b0);    // 25°
        send_node(16'd120, 16'd2000, 1'b0);    // 30°
        send_node(16'd140, 16'd2000, 1'b0);    // 35°
        send_node(16'd160, 16'd2000, 1'b0);    // 40°
        send_node(16'd280, 16'd3000, 1'b0);    // 70°  背景（与箱子断开）
        send_node(16'd304, 16'd618,  1'b0);    // 76°  ★箱子左端（平面前缘 600mm）
        send_node(16'd320, 16'd609,  1'b0);    // 80°
        send_node(16'd336, 16'd603,  1'b0);    // 84°
        send_node(16'd352, 16'd600,  1'b0);    // 88°
        send_node(16'd368, 16'd600,  1'b0);    // 92°
        send_node(16'd384, 16'd603,  1'b0);    // 96°
        send_node(16'd400, 16'd609,  1'b0);    // 100°
        send_node(16'd416, 16'd618,  1'b0);    // 104° ★箱子右端
        send_node(16'd440, 16'd3000, 1'b0);    // 110° 背景（与箱子断开）
        send_node(16'd0,   16'd3000, 1'b1);    // 下一个 S ⇒ 锁存这一圈
        #2000;
        chk1 (obs_valid,   1'b1,    "2a 本圈应有障碍段");
        chk16(obs_a0,      16'd76,  "2a 段起始方位角 = 76°（箱子左端）");
        chk16(obs_d0,      16'd618, "2a 段起始端距离 = 618mm");
        chk16(obs_a1,      16'd104, "2a 段结束方位角 = 104°（箱子右端）");
        chk16(obs_d1,      16'd618, "2a 段结束端距离 = 618mm");
        chk16(obs_dmin,    16'd600, "2a 段内最近距离 = 600mm（墙 2000 不该被选）");
        chk16(obs_amin,    16'd88,  "2a 最近点方位角 = 88°（第一个到 600 的点）");
        chk16(front_l_mm,  16'd600, "2a 正前偏车左半区最小 = 600mm");
        chk16(front_r_mm,  16'd600, "2a 正前偏车右半区最小 = 600mm");
        chk1 (front_l_ok,  1'b1,    "2a 正前偏车左半区有效");
        chk1 (front_r_ok,  1'b1,    "2a 正前偏车右半区有效");

        // =========================================================================
        // 第 8b 圈：★V2.7（清单5/H1）★ "点稀疏/会抖的箱子" 也必须算成【一整段】
        //   段内故意造两处旧阈值挡不住的瑕疵：
        //     · 84° 处距离跳变 +300mm（旧 RUN_JUMP=250 ⇒ 会断开；新 400 ⇒ 连着）
        //     · 88°→96° 方位缝 8°（旧 RUN_GAP=6° ⇒ 会断开；新 10° ⇒ 连着）
        //   旧阈值下这段会被切成 3 截，"最近的那一段"只剩一小截 ⇒ dx 被算小
        //   ⇒ 就是现场 P=0F0014（φ=15°、需求横移 ≤100mm）那一类问题的来源。
        // =========================================================================
        $display("\n---- 2a/H1 测试：点稀疏的箱子（跳变 300mm + 8° 缝）应算成一段 ----");
        send_node(16'd0,   16'd2000, 1'b1);   // S（0°，不参与分段）
        send_node(16'd320, 16'd600,  1'b0);   // 80° 箱子左端
        send_node(16'd336, 16'd900,  1'b0);   // 84° ← 距离跳变 +300mm
        send_node(16'd352, 16'd900,  1'b0);   // 88°
        send_node(16'd384, 16'd900,  1'b0);   // 96° ← 方位缝 8°
        send_node(16'd400, 16'd600,  1'b0);   // 100° ← 距离跳变 −300mm
        send_node(16'd416, 16'd600,  1'b0);   // 104° 箱子右端
        send_node(16'd0,   16'd2000, 1'b1);   // 下一圈 S ⇒ 锁存
        #2000;
        chk1 (obs_valid, 1'b1,    "2a/H1 点稀疏的箱子也应有段");
        chk16(obs_a0,    16'd80,  "2a/H1 段起点应回到 80°（放宽后不再被切碎）");
        chk16(obs_d0,    16'd600, "2a/H1 段起点距离 600mm");
        chk16(obs_a1,    16'd104, "2a/H1 段终点应为 104°");
        chk16(obs_dmin,  16'd600, "2a/H1 段内最近仍应是 600mm");
        chk16(obs_amin,  16'd80,  "2a/H1 最近点方位应为 80°（第一次到 600）");

        // 第 9 圈：本圈前半球完全没有点 ⇒ obs_valid=0、dmin=FAR
        send_node(16'd720, 16'd3000, 1'b1);    // S 就在后半（180°+）
        send_node(16'd800, 16'd1000, 1'b0);    // 200°（后半，不参与分段）
        send_node(16'd720, 16'd3000, 1'b1);    // 锁存
        #2000;
        chk1 (obs_valid, 1'b0,    "2a 前半球无点 ⇒ obs_valid=0");
        chk16(obs_dmin,  16'hFFFF,"2a 前半球无点 ⇒ dmin=FFFF");

        // =========================================================================
        // 收尾：脉冲数与圈数一致
        // =========================================================================
        if (done_pulses !== scan_cnt) begin
            $display("  FAIL: scan_done 脉冲数 %0d != scan_cnt %0d", done_pulses, scan_cnt);
            errors = errors + 1;
        end
        $display("\n==== tb_lidar_parse 结果 ====");
        $display("  node_cnt=%0d scan_cnt=%0d bad_cnt=%0d 脉冲(scan_done=%0d rev_no_pt=%0d)",
                 node_cnt, scan_cnt, bad_cnt, done_pulses, nopt_pulses);
        $display("  front=%0d left=%0d right=%0d side_l=%0d side_r=%0d dist_f=%0d",
                 front_mm, left_mm, right_mm, side_l_mm, side_r_mm, dist_f_mm);
        if (errors == 0) $display("==== tb_lidar_parse PASS ====");
        else             $display("==== tb_lidar_parse FAIL: errors=%0d ====", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("==== tb_lidar_parse TIMEOUT (errors=%0d) ====", errors);
        $finish;
    end

endmodule
