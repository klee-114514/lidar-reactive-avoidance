//=============================================================================
// tb_motor_seg.v -- key -> queue -> segment execution regression
//   (S2 enqueue forward 10cm, S3 enqueue in-place left turn, S1 execute)
//-----------------------------------------------------------------------------
// Why this testbench exists (2026-10-08):
//   To close 50MHz timing I pipelined the "segment finished" condition
//   (rot_done / yaw_done).  A pipelined flag still shows the PREVIOUS segment's
//   arc during the first cycle of a new segment, so seg_done went high instantly
//   and the state machine thought the new segment was already done.
//   Symptom on the car: pressing S3/S4 (in-place turn) did almost nothing,
//   and queued actions got eaten one by one.
//   tb_dbg_uart (top level) and tb_avoid_fsm do NOT go through motor_driver_top,
//   which is why that regression slipped through -- this tb covers that path.
//
// PASS criteria:
//   1) forward segment really runs    : arc reaches ~N_FWD_10, fwd_ms  > 5
//   2) in-place turn really runs      : turn_left_ms > 5
//   3) no "finished with ~zero arc"   : instant_complete_count == 0
//   4) both segments drained          : 2 pops, back to st == IDLE(0)
//
// Run:  vlib work
//       vlog -work work <all rtl from gprj> tb_motor_seg.v
//       vsim -c -voptargs=+acc -do "run -all; quit -f" work.tb_motor_seg
//=============================================================================
`timescale 1ns/1ps

module tb_motor_seg;

    // ---------------- 50MHz ----------------
    reg clk = 1'b0;
    always #10 clk = ~clk;

    // ---------------- keys / inputs (active low, 1 = not pressed) ----------------
    reg key_stop  = 1'b1;
    reg key_start = 1'b1;
    reg key_fwd   = 1'b1;
    reg key_left  = 1'b1;
    reg key_r     = 1'b1;

    reg  [7:0]  sw      = 8'hFF;   // nothing switched on = normal logic
    reg         yaw_val = 1'b0;    // no IMU in this tb (imu_ok = 0 -> encoder fallback)
    reg  [15:0] yaw_raw = 16'd0;
    reg         slow_req = 1'b0;   // v3.2：限速档（在 DUT 之前声明）

    // ---------------- motor outputs ----------------
    wire ain1_l, ain2_l, pwma;
    wire ain1_r, ain2_r, pwmb;
    wire stby;
    wire [7:0]  led;
    wire [31:0] dbg_cnt_l, dbg_cnt_r;
    wire [15:0] dbg_spd_l, dbg_spd_r, dbg_yaw;
    wire        clr_out, ext_done, ext_active, remain_cap;
    wire [31:0] remain_out;

    // ---------------- wheel + encoder model ----------------
    //   Convention (matches the car's measured polarity): ain1=1/ain2=0 with PWM
    //   high means "motor forward", and the RAW encoder counts DOWN in that
    //   direction (that is exactly what INV_L_DEF=1 / INV_R_DEF=0 compensate).
    //   One "position step" = one quadrature cycle = 4 counts after 4x decoding.
    //   Step rate is proportional to PWM duty: ~300 permille duty -> ~600 counts
    //   per 5ms, same order as the real calibration, so the PID settles.
    reg [31:0] pos_l = 32'd0;
    reg [31:0] pos_r = 32'd0;
    reg [15:0] acc_l = 16'd0;
    reg [15:0] acc_r = 16'd0;

    always @(posedge clk) begin
        if (pwma && ain1_l && ~ain2_l) begin               // forward
            if (acc_l >= 16'd499) begin acc_l <= 16'd0; pos_l <= pos_l - 32'd1; end
            else acc_l <= acc_l + 16'd1;
        end else if (pwma && ~ain1_l && ain2_l) begin       // reverse
            if (acc_l >= 16'd499) begin acc_l <= 16'd0; pos_l <= pos_l + 32'd1; end
            else acc_l <= acc_l + 16'd1;
        end
        if (pwmb && ain1_r && ~ain2_r) begin
            if (acc_r >= 16'd499) begin acc_r <= 16'd0; pos_r <= pos_r - 32'd1; end
            else acc_r <= acc_r + 16'd1;
        end else if (pwmb && ~ain1_r && ain2_r) begin
            if (acc_r >= 16'd499) begin acc_r <= 16'd0; pos_r <= pos_r + 32'd1; end
            else acc_r <= acc_r + 16'd1;
        end
    end

    // quadrature A/B (one full cycle per 4 position steps)
    wire ea_l = pos_l[1] ^ pos_l[0];
    wire eb_l = pos_l[1];
    wire ea_r = pos_r[1] ^ pos_r[0];
    wire eb_r = pos_r[1];

    // ---------------- DUT ----------------
    motor_driver_top #(
        .CLK_FREQ (50_000_000), .PWM_FREQ (20_000), .DUTY_MAX (1000), .SPEED_MS (5),
                .YAW_STOP_M (16'd2800), .YAW_STOP_S (16'd1024), .YAW_LEFT_POS (1'b1),
        .HEAD_SH (3), .HEAD_LIM (16'sd120),
        .YAW_90_RAW (16'd16384), .YAW_45_RAW (16'd8192),
        .KP (16'd64), .KI (16'd4), .KD (16'd0),
        // short segment lengths: this tb checks sequencing, not travel calibration
        .N_FWD_5 (32'd1500), .N_FWD_10 (32'd3000), .N_FWD_20 (32'd6000),
        .N_TURN_45 (32'd1500), .N_TURN_90 (32'd3000), .N_ARC (32'd3000),
        .SAFE_MS (20000), .PIVOT_MS (5000),
        // small key windows to keep the simulation short
        //   single click = hold 40ms (>20ms debounce) -> release -> wait 200ms
        .DOUBLE_MS (60), .LONG_MS (200)
    ) dut (
        .clk (clk), .force_stop (1'b0),
        .slow_req (slow_req), .ext_slow (1'b0),   // v3.2：slow_req 由 tb 驱动，用来断言限速
        .yaw_raw (yaw_raw), .yaw_valid (yaw_val),
        .key_stop (key_stop), .key_start (key_start), .key_fwd (key_fwd),
        .key_left (key_left), .key_r (key_r),
        .ir_fwd (1'b0), .ir_dist (2'd0), .ir_left (1'b0), .ir_right (1'b0),
        .ir_exec (1'b0), .ir_stop (1'b0), .ir_clear (1'b0), .ir_cancel (1'b0),
        .ir_stop_hold (1'b0),
        .gp_fwd (1'b0), .gp_back (1'b0), .gp_left (1'b0), .gp_right (1'b0),
        .sw (sw),
        .ain1_l (ain1_l), .ain2_l (ain2_l), .pwma (pwma),
        .ain1_r (ain1_r), .ain2_r (ain2_r), .pwmb (pwmb), .stby (stby),
        .enc_al (ea_l), .enc_bl (eb_l), .enc_ar (ea_r), .enc_br (eb_r),
        .led (led),
        .dbg_cnt_l (dbg_cnt_l), .dbg_cnt_r (dbg_cnt_r),
        .dbg_spd_l (dbg_spd_l), .dbg_spd_r (dbg_spd_r), .dbg_yaw (dbg_yaw),
        .clr_out (clr_out),
        .ext_load (1'b0), .ext_act (3'd0), .ext_n (32'd0), .ext_yaw_tgt (16'd0),
        .ext_to_queue (1'b0), .ext_done (ext_done), .ext_active (ext_active),
        .remain_out (remain_out), .remain_cap (remain_cap)
    );

    // ---------------- single key click ----------------
    task press_single;
        input integer which;      // 1=S1 run  2=S2 forward  3=S3 left  4=S4 right
        begin
            case (which)
                1: key_start = 1'b0;
                2: key_fwd   = 1'b0;
                3: key_left  = 1'b0;
                default: key_r = 1'b0;
            endcase
            #40_000_000;          // hold 40ms
            case (which)
                1: key_start = 1'b1;
                2: key_fwd   = 1'b1;
                3: key_left  = 1'b1;
                default: key_r = 1'b1;
            endcase
            #200_000_000;         // wait past the double-click window
        end
    endtask

    // ---------------- monitors ----------------
    integer    seg_pops  = 0;     // queue pops = segments actually executed
    integer    fwd_ms    = 0;
    integer    turn_l_ms = 0;
    integer    zero_seg  = 0;     // "finished with ~zero arc" count (the bug signature)
    reg [1:0]  st_d      = 2'd0;
    reg [2:0]  act_d     = 3'd0;
    reg        segd_d    = 1'b0;
    integer    errors    = 0;
    reg signed [15:0] tgt_l_now, slw_l_now;
    reg [2:0]  q_cnt_d   = 3'd0;
    reg [31:0] cnt_tick  = 32'd0;

    always @(posedge clk) begin
        // 1ms buckets for per-action time accounting
        if (cnt_tick >= 32'd49_999) begin
            cnt_tick <= 32'd0;
            if (dut.st == 2'd1) begin
                if (dut.cur_act == 3'd0) fwd_ms    = fwd_ms    + 1;
                if (dut.cur_act == 3'd1) turn_l_ms = turn_l_ms + 1;
            end
        end else
            cnt_tick <= cnt_tick + 32'd1;

        // state / action change -> one line
        if (dut.st != st_d || dut.cur_act != act_d) begin
            $display("[%0t ns] st %0d->%0d  act %0d->%0d  cur_n=%0d  dist_l=%0d dist_r=%0d  spd_l=%0d spd_r=%0d",
                     $time, st_d, dut.st, act_d, dut.cur_act,
                     dut.cur_n, dut.dist_l, dut.dist_r, dut.spd_l, dut.spd_r);
            st_d  <= dut.st;
            act_d <= dut.cur_act;
        end

        // queue pops
        if (dut.q_cnt != q_cnt_d) begin
            if (dut.q_cnt < q_cnt_d) seg_pops = seg_pops + 1;
            q_cnt_d <= dut.q_cnt;
        end

        // seg_done rising edge: was this segment finished with (almost) no travel?
        if (dut.seg_done && !segd_d) begin
            $display("[%0t ns] SEG_DONE act=%0d dist_l=%0d dist_r=%0d cur_n=%0d avg_arc=%0d",
                     $time, dut.cur_act, dut.dist_l, dut.dist_r, dut.cur_n,
                     (dut.dist_l + dut.dist_r) >> 1);
            if (((dut.dist_l + dut.dist_r) >> 1) < (dut.cur_n >> 3)) zero_seg = zero_seg + 1;
        end
        segd_d <= dut.seg_done;
    end

    // ---------------- main sequence ----------------
    initial begin
        #50_000_000;                       // 50ms power-on settle (internal POR is short)

        $display("== enqueue S2 (forward 10cm) ==");
        press_single(2);
        $display("   q_cnt=%0d", dut.q_cnt);
        $display("== enqueue S3 (in-place left 90) ==");
        press_single(3);
        $display("   q_cnt=%0d", dut.q_cnt);
        $display("== S1 execute ==");
        press_single(1);

        // ---------------- ★出厂速度断言（v3.2 新增）----------------
        //   上次的教训：这个 tb 以前把 TARGET_SPD 覆盖成 600/750/525，
        //   等于"出厂速度从来没被测过"。现在不覆盖参数，直接查
        //   电机模块内部的档位选择：巡航档 = TARGET_SPD、减速档 = SLOW_SPD。
        slow_req = 1'b0; #1000;
        // ★V4.35j★ 断言改成引用 DUT 的 TARGET_SPD，不再写死数值
        //   （原来写 560，巡航提到 840 后必然误报；断言的意图是"巡航档=TARGET_SPD"）
        $display("  巡航档 sel_spd=%0d（应=%0d=TARGET_SPD）", dut.sel_spd, dut.TARGET_SPD);
        if (dut.sel_spd !== dut.TARGET_SPD) begin
            $display("  FAIL: 巡航档速度不对 sel_spd=%0d", dut.sel_spd); errors=errors+1;
        end
        slow_req = 1'b1; #1000;
        $display("  减速档 sel_spd=%0d（应=320=SLOW_SPD）", dut.sel_spd);
        if (dut.sel_spd !== 16'sd320) begin
            $display("  FAIL: 减速档速度不对 sel_spd=%0d（限速路径没接上）", dut.sel_spd); errors=errors+1;
        end
        slow_req = 1'b0; #1000;
        #300_000_000;                      // let both segments finish

        $display("\n==== tb_motor_seg RESULT ====");
        $display("  seg_pops (segments actually executed) = %0d   (expect 2)", seg_pops);
        $display("  fwd_ms = %0d    turn_left_ms = %0d", fwd_ms, turn_l_ms);
        $display("  instant_complete_count (expect 0)     = %0d", zero_seg);
        $display("  cnt_l=%0d  cnt_r=%0d  st=%0d", dut.cnt_l, dut.cnt_r, dut.st);

        if (seg_pops >= 2 && turn_l_ms >= 5 && fwd_ms >= 5 && zero_seg == 0 &&
            dut.st == 2'd0 && errors == 0)
            $display("==== tb_motor_seg PASS ====");
        else
            $display("==== tb_motor_seg FAIL ==== (errors=%0d)", errors);
        $finish;
    end

endmodule
