`timescale 1ns/1ps
// V5.0a unit test: 72-bin snapshot, corridor decisions, right-wall lock,
// box/avoid handoff, dead-end rejection and exit confirmation.
module tb_maze_nav;
    reg clk = 1'b0;
    always #10 clk = ~clk;

    integer errors = 0;
    integer scenario = 0;
    integer i;

    reg rst_n = 1'b0;
    reg start = 1'b0;
    reg abort = 1'b0;
    reg scan_done = 1'b0;
    reg scan_fresh = 1'b1;
    reg [15:0] front_mm = 16'd4000;
    reg [15:0] yaw_now = 16'd0;
    reg yaw_ok = 1'b1;
    reg avoid_active = 1'b0;
    reg avoid_done = 1'b0;
    reg avoid_fail = 1'b0;
    reg [15:0] hist_rd_data;
    wire [6:0] hist_rd_addr;

    wire ext_load, ext_to_queue, ext_slow;
    wire [2:0] ext_act;
    wire [31:0] ext_n;
    wire [15:0] ext_yaw_tgt;
    wire active, done, fail, allow_avoid, heading_changed;
    wire [3:0] st_code;
    wire [7:0] dbg_why;
    wire dbg_inside;
    wire [15:0] dbg_right_mm, dbg_left_mm;
    wire [15:0] dbg_right_depth, dbg_straight_depth, dbg_left_depth, dbg_turn_min;
    wire [7:0] dbg_turn_cnt, dbg_scan_cnt;

    reg m_busy = 1'b0;
    reg m_done = 1'b0;
    reg [2:0] m_act = 3'd0;
    reg [15:0] m_tgt = 16'd0;
    reg [3:0] m_cnt = 4'd0;
    wire ext_active = m_busy;

    integer seen_r_turn = 0;
    integer seen_l_turn = 0;
    integer seen_u_turn = 0;
    integer seen_box = 0;
    integer seen_done = 0;

    // 场景图：
    // 0=全开阔；1=右侧墙 250；2=右开口；3=左开口；
    // 4=死胡同；5=转不开；6=小箱；7=宽墙；8=歪10°的右墙。
    always @* begin
        hist_rd_data = 16'd2000;
        case (scenario)
            0: hist_rd_data = 16'd4000;
            1: begin
                if ((hist_rd_addr >= 7'd31) && (hist_rd_addr <= 7'd41))
                    hist_rd_data = 16'd250;
                else
                    hist_rd_data = 16'd4000;
            end
            2: hist_rd_data = 16'd4000;
            3: begin
                if ((hist_rd_addr >= 7'd31) && (hist_rd_addr <= 7'd41))
                    hist_rd_data = 16'd250;
                else if ((hist_rd_addr >= 7'd12) && (hist_rd_addr <= 7'd24))
                    hist_rd_data = 16'd500;
                else
                    hist_rd_data = 16'd4000;
            end
            4: begin
                if ((hist_rd_addr >= 7'd31) && (hist_rd_addr <= 7'd41))
                    hist_rd_data = 16'd250;
                else if ((hist_rd_addr >= 7'd63) || (hist_rd_addr <= 7'd9))
                    hist_rd_data = 16'd250;
                else if ((hist_rd_addr >= 7'd12) && (hist_rd_addr <= 7'd24))
                    hist_rd_data = 16'd500;
                else if ((hist_rd_addr >= 7'd50) && (hist_rd_addr <= 7'd58))
                    hist_rd_data = 16'd500;
                else
                    hist_rd_data = 16'd4000;
            end
            5: begin
                if (((hist_rd_addr >= 7'd31) && (hist_rd_addr <= 7'd41)) ||
                    (hist_rd_addr >= 7'd63) || (hist_rd_addr <= 7'd9))
                    hist_rd_data = 16'd200;
                else if ((hist_rd_addr >= 7'd12) && (hist_rd_addr <= 7'd24))
                    hist_rd_data = 16'd400;
                else
                    hist_rd_data = 16'd4000;
            end
            6: begin
                if ((hist_rd_addr >= 7'd16) && (hist_rd_addr <= 7'd20))
                    hist_rd_data = 16'd500;
                else
                    hist_rd_data = 16'd2000;
            end
            8: begin
                if (hist_rd_addr == 7'd34)
                    hist_rd_data = 16'd254;
                else
                    hist_rd_data = 16'd4000;
            end
            default: begin
                if ((hist_rd_addr >= 7'd9) && (hist_rd_addr <= 7'd27))
                    hist_rd_data = 16'd500;
                else
                    hist_rd_data = 16'd2000;
            end
        endcase
    end

    // 假电机：8 拍完成一个 ext 段；绝对航向段立即更新 yaw。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_busy <= 1'b0; m_done <= 1'b0; m_act <= 3'd0;
            m_tgt <= 16'd0; m_cnt <= 4'd0;
        end else begin
            m_done <= 1'b0;
            if (ext_load && !m_busy) begin
                m_busy <= 1'b1;
                m_act  <= ext_act;
                m_tgt  <= ext_yaw_tgt;
                m_cnt  <= 4'd8;
                if (ext_act == 3'd5) begin
                    if ((ext_yaw_tgt == (yaw_now - 16'd16384)) && ext_n == 32'd101600)
                        seen_r_turn = 1;
                    if ((ext_yaw_tgt == (yaw_now + 16'd16384)) && ext_n == 32'd101600)
                        seen_l_turn = 1;
                    if ((ext_yaw_tgt == (yaw_now + 16'd32768)) && ext_n == 32'd203200)
                        seen_u_turn = 1;
                end
            end else if (m_busy) begin
                if (m_cnt == 4'd1) begin
                    m_busy <= 1'b0;
                    m_done <= 1'b1;
                    if (m_act == 3'd5) yaw_now <= m_tgt;
                end else begin
                    m_cnt <= m_cnt - 4'd1;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) seen_done <= 0;
        else if (done) seen_done <= 1;
    end

    maze_nav_fsm #(
        .MOVE_TO_CYC(32'd1000),
        .STALE_TO_CYC(32'd200)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .abort(abort),
        .scan_done(scan_done), .scan_fresh(scan_fresh),
        .front_mm(front_mm), .yaw_now(yaw_now), .yaw_ok(yaw_ok),
        .avoid_active(avoid_active), .avoid_done(avoid_done), .avoid_fail(avoid_fail),
        .hist_rd_addr(hist_rd_addr), .hist_rd_data(hist_rd_data),
        .ext_load(ext_load), .ext_act(ext_act), .ext_n(ext_n),
        .ext_yaw_tgt(ext_yaw_tgt), .ext_to_queue(ext_to_queue), .ext_slow(ext_slow),
        .ext_done(m_done), .ext_active(ext_active),
        .active(active), .done(done), .fail(fail),
        .allow_avoid(allow_avoid), .heading_changed(heading_changed),
        .st_code(st_code), .dbg_why(dbg_why), .dbg_inside(dbg_inside),
        .dbg_right_mm(dbg_right_mm), .dbg_left_mm(dbg_left_mm),
        .dbg_right_depth(dbg_right_depth),
        .dbg_straight_depth(dbg_straight_depth),
        .dbg_left_depth(dbg_left_depth), .dbg_turn_min(dbg_turn_min),
        .dbg_turn_cnt(dbg_turn_cnt), .dbg_scan_cnt(dbg_scan_cnt)
    );

    task chk;
        input cond;
        input [8*64:1] msg;
        begin
            if (!cond) begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    task pulse_start;
        begin
            @(posedge clk); start <= 1'b1;
            @(posedge clk); start <= 1'b0;
        end
    endtask

    task wait_ready;
        integer n;
        begin
            n = 0;
            while ((st_code != 4'd1) && (st_code != 4'd9) && (st_code != 4'd10) && (n < 5000)) begin
                @(posedge clk); n = n + 1;
            end
            if (n >= 5000) begin
                $display("FAIL: wait_ready timeout st=%0d why=%h", st_code, dbg_why);
                errors = errors + 1;
            end
        end
    endtask

    task do_scan;
        begin
            wait_ready;
            @(posedge clk); scan_done <= 1'b1;
            @(posedge clk); scan_done <= 1'b0;
            repeat (420) @(posedge clk);
            wait_ready;
        end
    endtask

    initial begin
        #100; rst_n = 1'b1; #100;

        // 1) 起点全开阔：可以前进，但不得误判完赛。
        scenario = 0; front_mm = 16'd4000;
        pulse_start();
        repeat (5) do_scan();
        chk(active == 1'b1, "open start must stay active");
        chk(done == 1'b0, "open start must not finish");
        chk(dbg_inside == 1'b0, "open start must not lock maze");

        // 2) 右侧墙连续 3 圈后锁存已进迷宫。
        scenario = 1; front_mm = 16'd4000;
        repeat (4) do_scan();
        chk(dbg_inside == 1'b1, "right wall must latch maze entry");

        // 2b) 车身相对右墙歪约 10°：垂直距离仍应接近 250mm，
        //     不能把斜距 1440mm 当成真实墙距。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 8; front_mm = 16'd4000;
        pulse_start();
        repeat (4) do_scan();
        chk(dbg_inside == 1'b1, "tilted right wall must still latch maze entry");
        chk((dbg_right_mm >= 16'd240) && (dbg_right_mm <= 16'd260),
            "right wall must use perpendicular distance");

        // 3) 右侧开口：右转优先。
        scenario = 2; front_mm = 16'd4000;
        do_scan();
        repeat (30) @(posedge clk);
        chk(seen_r_turn == 1, "right opening must request right 90 turn");

        // 4) 重新开始，左开口优先于死胡同。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 1; front_mm = 16'd4000;
        pulse_start();
        repeat (4) do_scan();
        scenario = 3; front_mm = 16'd500;
        do_scan();
        repeat (30) @(posedge clk);
        chk(seen_l_turn == 1, "left-only branch must request left 90 turn");

        // 5) 死胡同：扫掠半径足够时必须请求 180 掉头。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 4; front_mm = 16'd500;
        pulse_start();
        repeat (4) do_scan();
        repeat (30) @(posedge clk);
        chk(seen_u_turn == 1, "clear dead end must request 180 turn");

        // 6) 转不开：不能盲转，连续无安全出口后停车失败。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 5; front_mm = 16'd400;
        pulse_start();
        repeat (12) do_scan();
        repeat (30) @(posedge clk);
        chk(fail == 1'b1, "unsafe dead end must fail-stop");

        // 7) 开阔区小箱：允许 avoid_fsm 接管，避障期间迷宫不混发动作。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 6; front_mm = 16'd500;
        pulse_start();
        do_scan();
        repeat (30) @(posedge clk);
        $display("box diag inside=%b allow=%b width=%0d sd=%0d rd=%0d ld=%0d why=%h st=%0d",
                 dbg_inside, allow_avoid, dut.r_front_occ_cnt, dbg_straight_depth,
                 dbg_right_depth, dbg_left_depth, dbg_why, st_code);
        chk(allow_avoid == 1'b1, "small box must allow existing avoid");
        seen_box = 1;
        avoid_active <= 1'b1;
        repeat (20) @(posedge clk);
        chk((st_code == 4'd8), "maze must wait while avoid owns bus");
        chk(ext_load == 1'b0, "maze must not issue ext while avoid owns bus");
        avoid_active <= 1'b0;
        avoid_done <= 1'b1;
        @(posedge clk); avoid_done <= 1'b0;
        repeat (20) @(posedge clk);
        chk(active == 1'b1, "maze must resume after avoid done");
        scenario = 0; front_mm = 16'd4000;
        do_scan();
        repeat (20) @(posedge clk);
        chk(allow_avoid == 1'b0, "maze must revoke avoid permission after box clears");

        // 8) 宽墙不能当箱子交给避障。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 7; front_mm = 16'd500;
        pulse_start();
        do_scan();
        repeat (30) @(posedge clk);
        chk(allow_avoid == 1'b0, "wide wall must not be treated as box");

        // 9) 已进迷宫后全开阔连续 3 圈：确认出口并停车。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 1; front_mm = 16'd4000;
        pulse_start();
        repeat (4) do_scan();
        scenario = 0; front_mm = 16'd4000;
        repeat (4) do_scan();
        repeat (20) @(posedge clk);
        chk((done == 1'b1) || (seen_done == 1), "open exit after maze entry must finish");

        // 10) 断流：新鲜度保持低电平，必须停车失败。
        abort = 1'b1; @(posedge clk); abort = 1'b0;
        scenario = 1; front_mm = 16'd4000;
        pulse_start();
        scan_fresh = 1'b0;
        repeat (260) @(posedge clk);
        chk(fail == 1'b1, "stale lidar must fail-stop");

        if (errors == 0) $display("==== tb_maze_nav PASS ====");
        else             $display("==== tb_maze_nav FAIL errors=%0d ====", errors);
        $finish;
    end

    initial begin
        #10_000_000;
        $display("==== tb_maze_nav TIMEOUT errors=%0d ====", errors);
        $finish;
    end
endmodule
