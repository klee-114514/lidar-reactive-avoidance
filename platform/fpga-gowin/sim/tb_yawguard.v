// ★V4.1c★ 转向卡死看门狗回归
//   A: 航向完全不动 ⇒ 1.5s 窗口后必须报 w=53（真卡死仍要抓）
//   B: 航向一直在动、但 4.5s 内到不了目标 ⇒ 不许报 w=53（过冲/回收不准误杀）
`timescale 1ns/1ps
module tb_yawguard;
    reg clk = 1'b0;
    always #10 clk = ~clk;
    integer errors = 0;

    reg rst_n = 1'b0;
    reg [15:0] front_mm = 16'd413;
    reg [15:0] obs_a0 = 16'd80, obs_d0 = 16'd500;
    reg [15:0] obs_a1 = 16'd100, obs_d1 = 16'd500;
    reg [15:0] obs_dmin = 16'd500, obs_amin = 16'd90;
    reg        obs_valid = 1'b1;
    reg [15:0] front_l = 16'd4000, front_r = 16'd4000;
    reg [15:0] wide_l = 16'd4000, wide_r = 16'd4000;
    reg [15:0] left_mm = 16'd4000, right_mm = 16'd4000;
    reg        scan_done = 1'b0, arm = 1'b0;
    reg        ext_done_r = 1'b0, recover_r = 1'b0;
    reg [15:0] yaw_now = 16'd0;
    reg [31:0] remain_in = 32'd287000;

    wire [6:0] hist_addr;
    wire watch_en; wire [7:0] watch_ang;
    wire ext_load; wire [2:0] ext_act; wire [31:0] ext_n;
    wire [15:0] ext_yaw_tgt; wire ext_to_queue, ext_slow;
    wire avoid_active, avoid_fail, avoid_done;
    wire [3:0] st_code; wire [7:0] why; wire side;
    wire [15:0] dbg_lat, dbg_phi, dbg_fwd, dbg_g;

    avoid_fsm #(
        .CAR_W_MM(16'd220), .CAR_L_MM(16'd240), .AXLE_FWD_MM(16'd175),
        .SENS_FWD_MM(16'd150), .D_ARM_MM(16'd450), .M_SIDE_MM(16'd90),
        .STEP_MM(16'd80), .SIDE_MIN_MM(16'd300), .PHI_MAX_DEG(16'd60),
        .ADV_MIN_MM(16'd360), .PASS_K(8'd3), .CNT_PER_MM(32'd574),
        // ★注意★ 这是逻辑单测：把 CLK_FREQ_HZ 调小，
        //   让 YAW_STUCK_CYC=(F/1000)*1500 从 7500 万拍缩到 4500 拍，否则仿真跑不动。
        .CLK_FREQ_HZ(3_333)
    ) dut (
        .clk(clk), .rst_n(rst_n), .front_mm(front_mm), .front_v(1'b1),
        .prof_l_mm(16'd4000), .prof_l_v(1'b1), .prof_r_mm(16'd4000), .prof_r_v(1'b1),
        .scan_done(scan_done), .watch_mm(16'd4000), .watch_v(1'b1),
        .watch_far_cnt(8'd0), .watch_en(watch_en), .watch_ang(watch_ang),
        .obs_a0(obs_a0), .obs_d0(obs_d0), .obs_a1(obs_a1), .obs_d1(obs_d1),
        .obs_dmin(obs_dmin), .obs_amin(obs_amin), .obs_valid(obs_valid),
        .hist_rd_addr(hist_addr), .hist_rd_data(16'hFFFF),
        .hist_side_l_mm(16'd4000), .hist_side_r_mm(16'd4000),
        .front_l_mm(front_l), .front_r_mm(front_r), .front_l_ok(1'b1), .front_r_ok(1'b1),
        .wide_l_mm(wide_l), .wide_r_mm(wide_r),
        .left_mm(left_mm), .left_ok(1'b1), .right_mm(right_mm), .right_ok(1'b1),
        .arm(arm), .force_clear(1'b0), .recover(recover_r), .abort(1'b0), .stall(1'b0),
        .yaw_now(yaw_now), .yaw_v(1'b1), .remain_in(remain_in),
        .step_arc_cnt(32'd0),
        .ext_load(ext_load), .ext_act(ext_act), .ext_n(ext_n),
        .ext_yaw_tgt(ext_yaw_tgt), .ext_to_queue(ext_to_queue), .ext_slow(ext_slow),
        .ext_done(ext_done_r), .ext_active(1'b0),
        .avoid_active(avoid_active), .avoid_fail(avoid_fail), .avoid_done(avoid_done),
        .st_code(st_code), .dbg_why(why), .dbg_side(side), .dbg_lat(dbg_lat),
        .dbg_phi(dbg_phi), .dbg_fwd_acc(dbg_fwd), .dbg_g(dbg_g)
    );

    reg seen53 = 1'b0;
    reg seen3c = 1'b0;
    always @(posedge clk) if (why == 8'h53) seen53 <= 1'b1;

    integer k, bdiv;
    initial begin
        // ---------------- A: 航向完全不动 ----------------
        #200; rst_n = 1'b1; #200;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        for (k = 0; k < 7000; k = k + 1) @(posedge clk);     // > 4500 拍窗口
        if (!seen53) begin
            $display("FAIL A: yaw frozen 2s but no w=53 (st=%0d why=%h)", st_code, why);
            errors = errors + 1;
        end
        $display("A: seen53=%0d st=%0d why=%h", seen53, st_code, why);

        // ---------------- B: 航向一直在动（每 100 拍 +1 计数） ----------------
        rst_n = 1'b0; #100; rst_n = 1'b1; #100;
        seen53 = 1'b0; yaw_now = 16'd0; bdiv = 0;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        for (k = 0; k < 15000; k = k + 1) begin   // 3 个窗口，仍 < PIVOT 16500
            @(posedge clk);
            if (bdiv == 4) begin bdiv = 0; yaw_now = yaw_now + 16'd1; end
            else bdiv = bdiv + 1;
        end
        if (seen53) begin
            $display("FAIL B: yaw still moving but w=53 fired (st=%0d)", st_code);
            errors = errors + 1;
        end
        if (st_code != 4'd2) begin
            $display("FAIL B: expected still in S_T_OUT, st=%0d why=%h", st_code, why);
            errors = errors + 1;
        end
        $display("B: seen53=%0d st=%0d why=%h yaw=%0d", seen53, st_code, why, yaw_now);

        // ---------------- C: recover 必须把 avoid_cnt 清零（21:18 第三次原地不动） ----------------
        rst_n = 1'b0; #100; rst_n = 1'b1; #100;
        dut.avoid_cnt = 4'd2;         // 模拟已经失败过两次
        recover_r = 1'b1;
        @(posedge clk); @(posedge clk);
        recover_r = 1'b0;
        @(posedge clk);
        if (dut.avoid_cnt !== 4'd0) begin
            $display("FAIL C: recover must clear avoid_cnt, got %0d", dut.avoid_cnt);
            errors = errors + 1;
        end
        $display("C: avoid_cnt after recover = %0d", dut.avoid_cnt);

        // ---------------- D: 出航向重试用尽仍没到位 ⇒ 先回正认输，不许进 FOLLOW ----------------
        rst_n = 1'b0; #100; rst_n = 1'b1; #100;
        yaw_now = 16'd0; ext_done_r = 1'b0;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        for (k = 0; k < 6000; k = k + 1) begin
            @(posedge clk);
            if (st_code == 4'd2) k = 6000;   // 等到 S_T_OUT 并已发段
        end
        if (st_code != 4'd2) begin
            $display("FAIL D: never reached S_T_OUT, st=%0d why=%h", st_code, why);
            errors = errors + 1;
        end
        // 连续 4 次 ext_done，航向一直不动 ⇒ 3 次重试后用尽
        for (k = 0; k < 4; k = k + 1) begin
            @(posedge clk); ext_done_r = 1'b1;
            @(posedge clk); ext_done_r = 1'b0;
            repeat (4) @(posedge clk);
        end
        repeat (4) @(posedge clk);
        // 允许已经走到 S_T_BACK(4) 之后的 S_FAIL(9)，但绝不能进 S_FOLLOW(10)
        if ((why != 8'h39) || (st_code == 4'd10)) begin
            $display("FAIL D: retry exhausted must fail (w=39) not follow, got st=%0d why=%h",
                     st_code, why);
            errors = errors + 1;
        end
        $display("D: st=%0d why=%h", st_code, why);

        // ---------------- E: S_FOLLOW 里航向偏出 8° 必须原地补转（w=3C） ----------------
        rst_n = 1'b0; #100; rst_n = 1'b1; #100;
        yaw_now = 16'd0; ext_done_r = 1'b0; seen3c = 1'b0;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        for (k = 0; k < 6000; k = k + 1) begin
            @(posedge clk);
            if (st_code == 4'd2) k = 6000;
        end
        // 让航向先到目标 ⇒ ext_done 后进 S_FOLLOW
        yaw_now = dut.yaw_out_tgt;
        @(posedge clk); ext_done_r = 1'b1;
        @(posedge clk); ext_done_r = 1'b0;
        for (k = 0; k < 6000; k = k + 1) begin
            @(posedge clk);
            if (st_code == 4'd10) k = 6000;
        end
        if (st_code != 4'd10) begin
            $display("FAIL E: did not reach S_FOLLOW, st=%0d why=%h", st_code, why);
            errors = errors + 1;
        end
        // 偏出 11°，看 FSM 会不会自己补转（需要让每一步的 ext_done 回来，否则一直 busy）
        yaw_now = dut.yaw_out_tgt + 16'd4000;   // 22°，超过 AIM_TOL_CNT(15°)
        for (k = 0; k < 20000; k = k + 1) begin
            @(posedge clk);
            if ((k % 50) == 49) begin ext_done_r = 1'b1; @(posedge clk); ext_done_r = 1'b0; end
            if ((k % 2000) == 0)
                $display("   E dbg k=%0d busy=%b hdgfix=%0d yaw=%0d tgt=%0d why=%h",
                         k, dut.busy, dut.hdg_fix_cnt, yaw_now, dut.yaw_out_tgt, why);
            if (why == 8'h3C) k = 20000;
        end
        if (why != 8'h3C) begin
            $display("FAIL E: heading off 11deg but no w=3C, st=%0d why=%h", st_code, why);
            errors = errors + 1;
        end
        $display("E: st=%0d why=%h", st_code, why);

        if (errors == 0) $display("==== tb_yawguard PASS ====");
        else             $display("==== tb_yawguard FAIL: errors=%0d ====", errors);
        $finish;
    end
    initial begin
        #30_000_000;
        $display("==== tb_yawguard TIMEOUT ====");
        $finish;
    end
endmodule
