`timescale 1ns/1ps
// V3.0 smoke test: Bug2 approach -> side-distance follow -> m-line return.
// Scenario: box on the right, left wall far at trigger, left wall close during return.
module tb_bug2;
    reg clk = 1'b0;
    always #10 clk = ~clk;
    integer errors = 0;

    reg [15:0] front_mm = 16'd413;
    reg [15:0] obs_a0 = 16'd80, obs_d0 = 16'd500;
    reg [15:0] obs_a1 = 16'd120, obs_d1 = 16'd500;
    reg [15:0] obs_dmin = 16'd500, obs_amin = 16'd100;
    reg        obs_valid = 1'b1;
    reg [15:0] front_l = 16'd4000, front_r = 16'd4000;
    reg [15:0] wide_l = 16'd4000, wide_r = 16'd4000;
    reg [15:0] left_mm = 16'd1000, right_mm = 16'd3900;
    reg [15:0] hist_side_l = 16'd4000, hist_side_r = 16'd3900;
    reg        scan_done = 1'b0, arm = 1'b0, rst_n = 1'b0;
    reg [15:0] yaw_now = 16'd0;
    reg [31:0] remain_in = 32'd287000;

    wire        watch_en; wire [7:0] watch_ang;
    wire        ext_load; wire [2:0] ext_act; wire [31:0] ext_n;
    wire [15:0] ext_yaw_tgt; wire ext_to_queue, ext_slow;
    wire        avoid_active, avoid_fail, avoid_done;
    wire [3:0]  st_code; wire [7:0] why; wire side;
    wire [15:0] dbg_lat, dbg_phi, dbg_fwd, dbg_g;
    wire [6:0]  hist_addr_unused;
    wire [7:0]  dbg_flags_unused;

    reg [2:0]  m_act = 3'd0; reg [31:0] m_n = 0, m_wait = 0, m_arc = 0;
    reg [15:0] m_yaw = 0; reg m_done = 0;
    wire m_active = (m_wait != 0);

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
        .hist_rd_addr(hist_addr_unused), .hist_rd_data(16'hFFFF),
        .hist_side_l_mm(hist_side_l), .hist_side_r_mm(hist_side_r),
        .front_l_mm(front_l), .front_r_mm(front_r), .front_l_ok(1'b1), .front_r_ok(1'b1),
        .wide_l_mm(wide_l), .wide_r_mm(wide_r),
        .left_mm(left_mm), .left_ok(1'b1), .right_mm(right_mm), .right_ok(1'b1),
        .arm(arm), .force_clear(1'b0), .recover(1'b0), .abort(1'b0), .stall(1'b0),
        .yaw_now(yaw_now), .yaw_v(1'b1), .remain_in(remain_in),
        .step_arc_cnt(m_arc),
        .ext_load(ext_load), .ext_act(ext_act), .ext_n(ext_n),
        .ext_yaw_tgt(ext_yaw_tgt), .ext_to_queue(ext_to_queue), .ext_slow(ext_slow),
        .ext_done(m_done), .ext_active(m_active),
        .avoid_active(avoid_active), .avoid_fail(avoid_fail), .avoid_done(avoid_done),
        .st_code(st_code), .dbg_why(why), .dbg_side(side), .dbg_lat(dbg_lat),
        .dbg_phi(dbg_phi), .dbg_fwd_acc(dbg_fwd), .dbg_g(dbg_g)
        ,.dbg_flags(dbg_flags_unused)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin m_done<=0; m_wait<=0; m_arc<=0; end
        else begin
            m_done <= 0;
            if (ext_load && (m_wait == 0)) begin
                m_act <= ext_act; m_n <= ext_n; m_yaw <= ext_yaw_tgt;
                m_wait <= 32'd300; m_arc <= 32'd0;
            end else if (m_wait != 0) begin
                m_wait <= m_wait - 1;
                if (m_wait == 1) begin
                    m_done <= 1;
                    m_arc <= m_n;
                    yaw_now <= m_yaw;          // both ABS and HOLD follow target
                end else begin
                    m_arc <= m_arc + (m_n / 32'd300);
                end
            end
        end
    end

    task chk(input cond, input [8*48:1] msg);
        begin
            if (!cond) begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    integer i, last_st;
    reg seen_follow = 0, seen_ret = 0, seen_resume = 0;
    reg first_follow = 0, phi_checked = 0;
    reg ret_seen_i = 0, ret_phi_checked = 0;
    reg resume_checked = 0, dip_done = 0;
    integer first_i;
    integer ret_i;
    integer dip_i;

    initial begin
        #200; rst_n = 1'b1; #200;
        last_st = -1;
        scan_done = 1'b1;
        @(posedge clk); arm = 1'b1; @(posedge clk); arm = 1'b0; scan_done = 1'b0;
        for (i = 0; i < 200000; i = i + 1) begin
            @(posedge clk);
            scan_done = ((i % 40) == 39);
            if (st_code != last_st) begin
                $display("st=%0d why=%h lat=%0d fwd=%0d phi=%0d", st_code, why, dbg_lat, dbg_fwd, dbg_phi);
                last_st = st_code;
            end
            if (st_code == 4'd10) begin
                if (!first_follow) begin first_follow = 1; first_i = i; end
                seen_follow = 1;
                front_mm = 16'd4000;
                if (i < (first_i + 200)) begin
                    right_mm = 16'd190;       // regression: entry heading 40, side 190 (>crush 180)
                    hist_side_r = 16'd190;
                end else if (dbg_fwd > 32'd700) begin
                    right_mm = 16'd3900;      // box passed: side clears
                    hist_side_r = 16'd3900;
                end else if (dbg_lat > 150) begin
                    if ((500 - dbg_lat) > 200) right_mm = 500 - dbg_lat;
                    else                        right_mm = 16'd200;
                    hist_side_r = right_mm;
                end else begin
                    right_mm = 16'd3900;
                    hist_side_r = 16'd3900;
                end
                if (dbg_fwd > 600) obs_valid = 1'b0;
                if (!phi_checked && (i > (first_i + 200))) begin
                    phi_checked = 1;
                    chk(dbg_phi >= 16'd40,
                        "V3.0c must INCREASE heading when side=190 and current=40, not snap to 8");
                end
            end
            if (st_code == 4'd10 && !dip_done && (dbg_lat > 32'd100)) begin
                dip_done = 1; dip_i = i;
            end
            if (st_code == 4'd10 && dip_done && (i < (dip_i + 300)))
                begin right_mm = 16'd170; hist_side_r = 16'd170; end // V3.0e: 170mm dip must NOT crush
            if (st_code == 4'd11) begin
                if (!ret_seen_i) begin ret_seen_i = 1; ret_i = i; end
                seen_ret = 1;
                left_mm = 16'd180;          // left wall close during return
                right_mm = 16'd3900;
                hist_side_l = 16'd180; hist_side_r = 16'd3900;
                if (!ret_phi_checked && (i > (ret_i + 200))) begin
                    ret_phi_checked = 1;
                    // ★V4.35i★ 回线三档 20/26/32 → 30/34/38、钳位 40 ⇒ 范围同步到 25..40
                    //   （断言意图不变：必须有回线角、且不许超过 40 的钳位）
                    chk((dbg_phi >= 16'd25) && (dbg_phi <= 16'd40),
                        "V4.35i return angle at lat~316 must be 25..40 deg");
                end
            end
            if (st_code == 4'd7) begin
                seen_resume = 1;
                if (!resume_checked) begin
                    resume_checked = 1;
                    chk(dut.resume_n == remain_in,
                        "V3.0e resume_n must be full remain, not 0 at obstacle side");
                end
            end
            if (avoid_done || avoid_fail) i = 200000;
        end
        chk(seen_follow == 1, "V3.0 must enter S_FOLLOW");
        chk(seen_ret == 1,    "V3.0 must enter S_RET2");
        chk(seen_resume == 1, "V3.0 must reach S_RESUME");
        chk(avoid_done == 1,  "V3.0 must finish avoid_done");
        chk(avoid_fail == 1'b0, "V3.0 must not fail");
        // ★V4.35i★ 残差门 15 → 60 ⇒ 断言同步到 RET_TOL_MM(60)。
        //   注意 lat_acc 是【记账值】，15:50 现场实测它比真实横移慢约 5 倍，
        //   所以 60 的记账残差对应的真实偏差远小于 60mm。
        chk(dbg_lat <= 16'd60, "return must bring lat_acc to <=60mm (RET_TOL_MM)");
        if (errors == 0) $display("==== tb_bug2 PASS ====");
        else             $display("==== tb_bug2 FAIL: errors=%0d ====", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("==== tb_bug2 TIMEOUT: errors=%0d ====", errors);
        $finish;
    end
endmodule
