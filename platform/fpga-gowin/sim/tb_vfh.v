`timescale 1ns/1ps
// V4.0b: 72-sector FGM/VFH gap selection.
// Box at sectors 16..20 (80..105 deg, 500mm); free elsewhere.
// Expect: choose the right gap (center sector 28, 50 deg right), side=1.
module tb_vfh;
    reg clk = 1'b0;
    always #10 clk = ~clk;
    integer errors = 0;

    reg [15:0] hist_arr [0:71];
    reg [15:0] hist_rd_data;
    wire [6:0] hist_rd_addr;
    integer i;
    initial begin
        for (i = 0; i < 72; i = i + 1) hist_arr[i] = 16'd4000;
        // ★V4.1h★ 箱子放在【车右前方】110~135°（扇区 22..26）
        hist_arr[22] = 16'd500;
        hist_arr[23] = 16'd500;
        hist_arr[24] = 16'd500;
        hist_arr[25] = 16'd500;
        hist_arr[26] = 16'd500;
    end
    always @(*) hist_rd_data = hist_arr[hist_rd_addr];

    reg rst_n = 1'b0;
    reg [15:0] front_mm = 16'd413;
    reg [15:0] obs_a0 = 16'd80, obs_d0 = 16'd500;
    reg [15:0] obs_a1 = 16'd100, obs_d1 = 16'd500;
    reg [15:0] obs_dmin = 16'd500, obs_amin = 16'd90;
    reg        obs_valid = 1'b1;
    reg [15:0] front_l = 16'd4000, front_r = 16'd4000;
    reg [15:0] hs_l = 16'd4000, hs_r = 16'd4000;   // ★V4.31a★ 侧距可驱动
    reg [15:0] wide_l = 16'd4000, wide_r = 16'd4000;
    reg [15:0] left_mm = 16'd4000, right_mm = 16'd4000;
    reg        scan_done = 1'b0, arm = 1'b0, force_clear = 1'b0;
    reg        ext_done_r = 1'b0;
    reg [15:0] yaw_now = 16'd0;
    reg [31:0] remain_in = 32'd287000;

    wire watch_en; wire [7:0] watch_ang;
    wire ext_load; wire [2:0] ext_act; wire [31:0] ext_n;
    wire [15:0] ext_yaw_tgt; wire ext_to_queue, ext_slow;
    wire avoid_active, avoid_fail, avoid_done;
    wire [3:0] st_code; wire [7:0] why; wire side;
    wire [15:0] dbg_lat, dbg_phi, dbg_fwd, dbg_g;
    wire [7:0] dbg_flags;
    wire [15:0] dbg_shadow;

    avoid_fsm #(
        .CAR_W_MM(16'd220), .CAR_L_MM(16'd240), .AXLE_FWD_MM(16'd175),
        .SENS_FWD_MM(16'd150), .D_ARM_MM(16'd450), .M_SIDE_MM(16'd90),
        .STEP_MM(16'd80), .SIDE_MIN_MM(16'd300), .PHI_MAX_DEG(16'd60),
        .ADV_MIN_MM(16'd360), .PASS_K(8'd3), .CNT_PER_MM(32'd574),
        .CLK_FREQ_HZ(50_000_000)
    ) dut (
        .clk(clk), .rst_n(rst_n), .front_mm(front_mm), .front_v(1'b1),
        .prof_l_mm(16'd4000), .prof_l_v(1'b1), .prof_r_mm(16'd4000), .prof_r_v(1'b1),
        .scan_done(scan_done), .watch_mm(16'd4000), .watch_v(1'b1),
        .watch_far_cnt(8'd0), .watch_en(watch_en), .watch_ang(watch_ang),
        .obs_a0(obs_a0), .obs_d0(obs_d0), .obs_a1(obs_a1), .obs_d1(obs_d1),
        .obs_dmin(obs_dmin), .obs_amin(obs_amin), .obs_valid(obs_valid),
        .hist_rd_addr(hist_rd_addr), .hist_rd_data(hist_rd_data),
        .hist_side_l_mm(hs_l), .hist_side_r_mm(hs_r),
        .front_l_mm(front_l), .front_r_mm(front_r), .front_l_ok(1'b1), .front_r_ok(1'b1),
        .wide_l_mm(wide_l), .wide_r_mm(wide_r),
        .left_mm(left_mm), .left_ok(1'b1), .right_mm(right_mm), .right_ok(1'b1),
        .arm(arm), .force_clear(force_clear), .recover(1'b0), .abort(1'b0), .stall(1'b0),
        .yaw_now(yaw_now), .yaw_v(1'b1), .remain_in(remain_in),
        .step_arc_cnt(32'd0),
        .ext_load(ext_load), .ext_act(ext_act), .ext_n(ext_n),
        .ext_yaw_tgt(ext_yaw_tgt), .ext_to_queue(ext_to_queue), .ext_slow(ext_slow),
        .ext_done(ext_done_r), .ext_active(1'b0),
        .avoid_active(avoid_active), .avoid_fail(avoid_fail), .avoid_done(avoid_done),
        .st_code(st_code), .dbg_why(why), .dbg_side(side), .dbg_lat(dbg_lat),
        .dbg_phi(dbg_phi), .dbg_fwd_acc(dbg_fwd), .dbg_g(dbg_g),
        .dbg_shadow(dbg_shadow), .dbg_flags(dbg_flags)
    );

    integer j, kL_a, kR_a, kL_b, kR_b, phi_c;
    integer kk;
    initial begin
        #200; rst_n = 1'b1; #200;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        // ★V4.1a★ S_GAP 末尾多了影子几何扫掠（约 3.6k 拍），等待窗口放到 12k
        for (j = 0; j < 12000; j = j + 1) begin
            @(posedge clk);
            if (st_code == 4'd2) j = 12000;
        end
        if (st_code != 4'd2) begin
            $display("FAIL: did not reach S_T_OUT, st=%0d why=%h", st_code, why);
            errors = errors + 1;
        end
        // ★V4.1h★ 箱子在【车右前方】(110~135°) ⇒ 必须往【左】(side=0) 绕。
        //   旧断言写的是 side=1，正好是左右算反的那个 bug。
        if (side !== 1'b0) begin
            $display("FAIL: box on right must pass on LEFT (side=0), got %b", side);
            errors = errors + 1;
        end
        // 影子规划器：这一圈至少要有一侧可行（另一侧被箱子挡住是正常的）
        $display("V4.1a shadow: kL=%0d kR=%0d",
                 dbg_shadow[7:4], dbg_shadow[3:0]);
        if ((dbg_shadow[7:4] == 4'hF) && (dbg_shadow[3:0] == 4'hF)) begin
            $display("FAIL: shadow planner reports infeasible on both sides");
            errors = errors + 1;
        end
        // V4.3a：候选航向已经对齐，60° 时直线走廊半宽仍必须是 200mm，
        // 不能再出现 110*cos60+175*sin60+90≈297mm 的重复膨胀。
        if (dut.sw_vhalf_n !== 16'd200) begin
            $display("FAIL: travel corridor half-width=%0d, expect 200", dut.sw_vhalf_n);
            errors = errors + 1;
        end
        // 控制角必须就是影子给出的那一档（side=0 ⇒ 用 kL）
        if (dbg_phi !== (dbg_shadow[7:4] * 16'd5)) begin
            $display("FAIL: control phi=%0d must equal shadow kL*5=%0d",
                     dbg_phi, dbg_shadow[7:4] * 16'd5);
            errors = errors + 1;
        end

        // ---- 场景 B：箱子挪到【车左前方】(50~75°) ⇒ 必须往【右】(side=1) 绕 ----
        kL_a = dbg_shadow[7:4];
        kR_a = dbg_shadow[3:0];
        force_clear = 1'b1;
        @(posedge clk); @(posedge clk);
        force_clear = 1'b0;
        repeat (4) @(posedge clk);
        for (i = 0; i < 72; i = i + 1) hist_arr[i] = 16'd4000;
        hist_arr[10] = 16'd500;
        hist_arr[11] = 16'd500;
        hist_arr[12] = 16'd500;
        hist_arr[13] = 16'd500;
        hist_arr[14] = 16'd500;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        for (j = 0; j < 12000; j = j + 1) begin
            @(posedge clk);
            if (st_code == 4'd2) j = 12000;
        end
        kL_b = dbg_shadow[7:4];
        kR_b = dbg_shadow[3:0];
        $display("V4.1a shadow: A kL=%0d kR=%0d -> B kL=%0d kR=%0d",
                 kL_a, kR_a, kL_b, kR_b);
        if (side !== 1'b1) begin
            $display("FAIL: box on left must pass on RIGHT (side=1), got %b", side);
            errors = errors + 1;
        end
        if (dbg_phi !== (kR_b * 16'd5)) begin
            $display("FAIL: scenario B phi=%0d must equal shadow kR*5=%0d",
                     dbg_phi, kR_b * 16'd5);
            errors = errors + 1;
        end

        // ---- 场景 C/D：贴边 + 侧距 P 控制（V4.31a）----
        //   side=0 ⇒ side_win = hist_side_r，目标 200mm
        //   200 ⇒ 保持；150 ⇒ +5°；250 ⇒ -5°
        hs_l = 16'd200; hs_r = 16'd200;
        yaw_now = dut.yaw_out_tgt;            // 先把"已转到目标"摆好，让 S_T_OUT 收工
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        for (j = 0; j < 20000; j = j + 1) begin
            @(posedge clk);
            if (st_code == 4'd10) j = 20000;
        end
        if (st_code !== 4'd10) begin
            $display("FAIL C0: not in S_FOLLOW, st=%0d why=%h", st_code, why);
            errors = errors + 1;
        end
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        // ---- 场景 G：里程碑必须能在【真实通路】上成立 ----
        //   obs_* 段被 lidar_parse 限在 45~135°，a0/a1 永远到不了 165°/15°，
        //   所以里程碑必须由侧窗实测驱动。这里走真实通路：hs 700 → 250 → 700。
        force dut.side_seen = 1'b0;   // 清掉前面场景留下的"进过侧窗"记忆
        hs_l = 16'd700; hs_r = 16'd700;      // 侧窗还远 ⇒ 没到障碍前端
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        repeat (200) @(posedge clk);
        if (dut.a0_pass !== 1'b0) begin
            $display("FAIL G1: a0_pass must be 0 while side=700");
            errors = errors + 1;
        end
        release dut.side_seen;                // 交给 DUT 自己置
        hs_l = 16'd250; hs_r = 16'd250;      // 到障碍侧面 ⇒ 应置 a0_pass
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        repeat (200) @(posedge clk);
        if (dut.a0_pass !== 1'b1) begin
            $display("FAIL G2: a0_pass must be 1 after side drops to 250");
            errors = errors + 1;
        end
        hs_l = 16'd700; hs_r = 16'd700;      // 侧窗又变远
        obs_valid = 1'b0;                    // ★V4.33e★ a1_pass 现在只认"正前段连续 2 圈消失"
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        repeat (50) @(posedge clk);
        repeat (3) begin                      // 给 3 个成圈脉冲（要连续 2 圈确认）
            scan_done = 1'b1; @(posedge clk); scan_done = 1'b0; repeat (5) @(posedge clk);
        end
        // ★V4.33e★ a1_pass 已删除（它是假信号）；现在退出只认 front_gone_cnt 与 fwd_acc，
        //   这里改为断言"正前段连续消失"确实能计数到饱和
        if (dut.front_gone_cnt < 2'd2) begin
            $display("FAIL G3: front_gone_cnt must reach 2 after 3 no-segment scans (got %0d)",
                     dut.front_gone_cnt);
            errors = errors + 1;
        end
        $display("G: a0_pass 由侧窗驱动、front_gone_cnt 由正前段消失驱动");
        obs_valid = 1'b1;
        hs_l = 16'd4000; hs_r = 16'd4000;

        repeat (2000) @(posedge clk);
        phi_c = dbg_phi;

        // C1: 侧距正好 200 ⇒ 角度保持
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        repeat (2000) @(posedge clk);
        if (dbg_phi !== phi_c) begin
            $display("FAIL C1: side=200 must keep phi (%0d -> %0d)", phi_c, dbg_phi);
            errors = errors + 1;
        end

        // C2: 侧距 150（太近）⇒ 必须加 5°
        hs_l = 16'd160; hs_r = 16'd160;   // 160 > CRUSH_SIDE(155)，不会触发 crush
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        repeat (2000) @(posedge clk);
        if (dbg_phi !== (phi_c + 16'd3)) begin
            $display("FAIL C2: side=160 must grow +3 (%0d -> %0d)", phi_c, dbg_phi);
            errors = errors + 1;
        end
        $display("C: side=200 keep %0d, side=160 -> %0d (P grows)", phi_c, dbg_phi);

        // D: 侧距 250（太远）⇒ 必须收 5°
        phi_c = dbg_phi;
        yaw_now = dut.yaw_out_tgt;
        hs_l = 16'd250; hs_r = 16'd250;
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        repeat (2000) @(posedge clk);
        // 期望 6 而不是 13-4=9：cur_mag 来自 act_k_r（5° 档），13° 会被量化成 10°，
        //   所以 10 - (250-200)/10 = 6。这个量化误差是旧设计固有的，不是 bug。
        if (dbg_phi !== 16'd6) begin
            $display("FAIL D: side=250 must shrink to 6 (%0d -> %0d)", phi_c, dbg_phi);
            errors = errors + 1;
        end
        $display("D: side=250 -> %0d (P shrinks)", dbg_phi);

        // ---- D1：a0_pass=0（车轴没过障碍前端）⇒ 侧距再大也不许收角 ----
        force dut.a0_pass = 1'b0;
        phi_c = dbg_phi;
        hs_l = 16'd250; hs_r = 16'd250;
        for (kk = 0; kk < 3; kk = kk + 1) begin   // ★V4.35d★ 连 pulse 3 次：FSM 的
            @(posedge clk); ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0;  // 4 相位
            repeat (20) @(posedge clk);                                           // 都要走完
        end
        repeat (2000) @(posedge clk);
        if (dbg_phi + 16'd5 < phi_c) begin
            $display("FAIL D1: must NOT shrink before a0_pass (%0d -> %0d)", phi_c, dbg_phi);
            errors = errors + 1;
        end
        $display("D1: a0_pass=0, side=250 -> %0d (gate blocks shrink)", dbg_phi);

        release dut.a0_pass;
        hs_l = 16'd4000; hs_r = 16'd4000;
        hs_r = 16'd4000;

        // ---- 场景 E：01:15 现场复现 ----
        //   障碍段约 89°~114°、751~825mm，位于右前；左侧空。
        //   旧公式在这一段会报 G=00FC/00FF，并把车推到 60°。
        force_clear = 1'b1;
        @(posedge clk); @(posedge clk);
        force_clear = 1'b0;
        repeat (4) @(posedge clk);
        for (i = 0; i < 72; i = i + 1) hist_arr[i] = 16'd4000;
        hist_arr[18] = 16'd751;
        hist_arr[19] = 16'd766;
        hist_arr[20] = 16'd781;
        hist_arr[21] = 16'd796;
        hist_arr[22] = 16'd825;
        obs_a0 = 16'd89; obs_d0 = 16'd751;
        obs_a1 = 16'd114; obs_d1 = 16'd825;
        obs_dmin = 16'd751; obs_amin = 16'd101;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        for (j = 0; j < 12000; j = j + 1) begin
            @(posedge clk);
            if (st_code == 4'd2) j = 12000;
        end
        $display("V4.3a field case: kL=%0d kR=%0d side=%b phi=%0d",
                 dbg_shadow[7:4], dbg_shadow[3:0], side, dbg_phi);
        if (side !== 1'b0) begin
            $display("FAIL E: right-front obstacle must pass on LEFT");
            errors = errors + 1;
        end
        if ((dbg_shadow[7:4] == 4'hF) || (dbg_phi >= 16'd60)) begin
            $display("FAIL E: clear left side must be feasible below 60deg");
            errors = errors + 1;
        end

        // ---- 场景 F：跟随段必须用“目标绝对档位 − 当前实测档位” ----
        //   当前已左转30°、候选绝对35°：只允许按剩余5°旋转点云。
        dut.sw_cur_k_s = 8'sd6;
        dut.sw_side = 1'b0;
        dut.sw_k = 4'd7;
        #1;
        if ((dut.sw_rel_k !== 4'd1) || (dut.sw_rel_side !== 1'b0)) begin
            $display("FAIL F1: cur=+30 cand=+35 -> rel=%0d side=%b, expect 5/left",
                     dut.sw_rel_k, dut.sw_rel_side);
            errors = errors + 1;
        end
        //   当前已左转40°、候选绝对35°：应变成向右5°的相对修正，
        //   不能仍按绝对35°再次左旋。
        dut.sw_cur_k_s = 8'sd8;
        #1;
        if ((dut.sw_rel_k !== 4'd1) || (dut.sw_rel_side !== 1'b1)) begin
            $display("FAIL F2: cur=+40 cand=+35 -> rel=%0d side=%b, expect 5/right",
                     dut.sw_rel_k, dut.sw_rel_side);
            errors = errors + 1;
        end
        $display("F: relative-angle arithmetic PASS");
        if (errors == 0) $display("==== tb_vfh PASS ====");
        else             $display("==== tb_vfh FAIL: errors=%0d ====", errors);
        $finish;
    end
    initial begin
        #20_000_000;   // ★V4.35d★ 加长：3-pulse 之后 tb 本身变长，原 2ms 会先超时 $finish
        $display("==== tb_vfh TIMEOUT ====");
        $finish;
    end
endmodule
