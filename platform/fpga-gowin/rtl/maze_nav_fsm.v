//=============================================================================
// maze_nav_fsm.v —— V5.0 树状迷宫右墙导航
//-----------------------------------------------------------------------------
// 设计边界：
//   · 只使用 lidar_parse 的 72×5° 直方图和 IMU 航向，不建立全局地图；
//   · 不接管 avoid_fsm 的绕障算法，只在“开阔区小箱子可安全绕过”时让出总线；
//   · 对未知障碍一律保守处理：不能证明可绕，就按墙处理；
//   · 动作仍走 motor_driver_top 既有的 ext_load/ext_done 接口。
//
// 方位口径与 lidar_parse 一致：
//   0°=车正左，90°=车正前，180°=车正右，270°=车后；
//   直方图扇区 k=0..71，对应 5°×k。
//
// 局部坐标：
//   x 向右为正，y 向前为正，原点为雷达中心。
//   x = -d*cos(a)，y = d*sin(a)。
//
// 走廊判定：
//   把点云投到候选方向的“400mm 宽、700mm 深”管道里；
//   管道半宽由 110mm 车体半宽 + 90mm 余量得到。
//
// 动作：
//   A_FWD_HOLD(6) 每步 80mm，航向按右墙误差做 ±5°/±10° 小修正；
//   A_YAW_ABS(5) 用 IMU 绝对航向完成 90°/180° 原地转；
//   ext_to_queue 恒 0，迷宫每步完成后都重新扫描。
//=============================================================================

module maze_nav_fsm #(
    parameter [15:0] CAR_HALF_W_MM  = 16'd110,
    parameter [15:0] SIDE_MARGIN_MM = 16'd90,
    parameter [15:0] SIDE_TARGET_MM = 16'd200,
    parameter [15:0] CORRIDOR_HALF_MM = 16'd200,
    parameter [15:0] LOOKAHEAD_MM   = 16'd700,
    parameter [15:0] WALL_LOCK_MM   = 16'd600,
    parameter [15:0] WALL_STABLE_MM = 16'd80,
    parameter [15:0] FRONT_ARM_MM   = 16'd650,
    parameter [15:0] BLOCK_MM       = 16'd800,
    parameter [15:0] BOX_MAX_ANG_DEG= 16'd60,
    parameter [15:0] EXIT_FRONT_MM  = 16'd1500,
    parameter [15:0] EXIT_SIDE_MM   = 16'd800,
    parameter [15:0] TURN_RADIUS_MM = 16'd235,
    parameter [15:0] STEP_MM        = 16'd80,
    parameter [31:0] CNT_PER_MM     = 32'd574,
    parameter [31:0] N_TURN_90      = 32'd101600,
    parameter [7:0]  PASS_K         = 8'd3,
    parameter [7:0]  FAIL_K         = 8'd8,
    parameter [31:0] MOVE_TO_CYC    = 32'd275_000_000, // 5.5s @50MHz, TB 可覆盖
    parameter [31:0] STALE_TO_CYC   = 32'd25_000_000,  // 0.5s @50MHz, TB 可覆盖
    parameter [11:0] FAR_MM         = 12'd1500
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        abort,
    input  wire        scan_done,
    input  wire        scan_fresh,
    input  wire [15:0] front_mm,
    input  wire [15:0] yaw_now,
    input  wire        yaw_ok,
    input  wire        avoid_active,
    input  wire        avoid_done,
    input  wire        avoid_fail,

    output reg  [6:0]  hist_rd_addr,
    input  wire [15:0] hist_rd_data,

    output reg         ext_load,
    output reg  [2:0]  ext_act,
    output reg  [31:0] ext_n,
    output reg  [15:0] ext_yaw_tgt,
    output wire        ext_to_queue,
    output wire        ext_slow,
    input  wire        ext_done,
    input  wire        ext_active,

    output reg         active,
    output reg         done,
    output reg         fail,
    output reg         allow_avoid,
    output reg         heading_changed,

    output reg  [3:0]  st_code,
    output reg  [7:0]  dbg_why,
    output wire        dbg_inside,
    output wire [15:0] dbg_right_mm,
    output wire [15:0] dbg_left_mm,
    output wire [15:0] dbg_right_depth,
    output wire [15:0] dbg_straight_depth,
    output wire [15:0] dbg_left_depth,
    output wire [15:0] dbg_turn_min,
    output wire [7:0]  dbg_turn_cnt,
    output wire [7:0]  dbg_scan_cnt
);

    localparam A_YAW_ABS  = 3'd5;
    localparam A_FWD_HOLD = 3'd6;

    localparam [3:0] S_IDLE        = 4'd0,
                     S_SCAN_REQ    = 4'd1,
                     S_SCAN_ADDR   = 4'd2,
                     S_SCAN_DATA   = 4'd3,
                     S_EVAL_SET    = 4'd4,
                     S_EVAL_ACC    = 4'd5,
                     S_DECIDE      = 4'd6,
                     S_MOVE_WAIT   = 4'd7,
                     S_AVOID_WAIT  = 4'd8,
                     S_DONE        = 4'd9,
                     S_FAIL        = 4'd10;

    localparam [15:0] FAR16 = 16'hFFFF;

    // 0..90° 的正弦 Q10 查表；其它角度由 sinA/cosA 镜像。
    function [15:0] sin90_q10;
        input [15:0] k;
        begin
            case (k)
                16'd0:  sin90_q10 = 16'd0;    16'd1:  sin90_q10 = 16'd89;
                16'd2:  sin90_q10 = 16'd178;  16'd3:  sin90_q10 = 16'd265;
                16'd4:  sin90_q10 = 16'd350;  16'd5:  sin90_q10 = 16'd433;
                16'd6:  sin90_q10 = 16'd512;  16'd7:  sin90_q10 = 16'd587;
                16'd8:  sin90_q10 = 16'd658;  16'd9:  sin90_q10 = 16'd724;
                16'd10: sin90_q10 = 16'd784;  16'd11: sin90_q10 = 16'd839;
                16'd12: sin90_q10 = 16'd887;  16'd13: sin90_q10 = 16'd928;
                16'd14: sin90_q10 = 16'd962;  16'd15: sin90_q10 = 16'd989;
                16'd16: sin90_q10 = 16'd1008; 16'd17: sin90_q10 = 16'd1020;
                default: sin90_q10 = 16'd1024;
            endcase
        end
    endfunction

    function signed [15:0] sinA_q10;
        input [15:0] k;                         // a/5，0..36
        begin
            sinA_q10 = (k <= 16'd18) ? $signed({1'b0, sin90_q10(k)})
                                     : $signed({1'b0, sin90_q10(16'd36 - k)});
        end
    endfunction

    function signed [15:0] cosA_q10;
        input [15:0] k;                         // a/5，0..36
        begin
            cosA_q10 = (k <= 16'd18) ? $signed({1'b0, sin90_q10(16'd18 - k)})
                                     : (-$signed({1'b0, sin90_q10(k - 16'd18)}));
        end
    endfunction

    // 点到像素级的保守判断：fabs(x) <= bound，避免再引入有符号分支块。
    function in_half_width;
        input signed [31:0] x;
        input [15:0] bound;
        begin
            in_half_width = (x >= -$signed({16'd0, bound})) &&
                            (x <=  $signed({16'd0, bound}));
        end
    endfunction

    function [15:0] min16;
        input [15:0] a;
        input [15:0] b;
        begin
            min16 = (a < b) ? a : b;
        end
    endfunction

    function absdiff16;
        input [15:0] a;
        input [15:0] b;
        reg   [15:0] d;
        begin
            d = (a >= b) ? (a - b) : (b - a);
            absdiff16 = d;
        end
    endfunction

    // 用于按实际方位角执行“接近 0°”的环绕窗口判断。
    function in_angle_win;
        input integer idx;
        input integer center;
        input integer half;
        integer a;
        integer d;
        begin
            a = idx * 5;
            d = (a >= center) ? (a - center) : (center - a);
            if (d > 180) d = 360 - d;
            in_angle_win = (d <= half);
        end
    endfunction

    reg [15:0] scan_buf [0:71];
    reg [6:0]  scan_idx;
    reg [6:0]  eval_idx;
    reg        scan_pending;
    reg        scan_wait1;

    reg [15:0] r_front_min, r_left_min, r_right_min, r_all_min;
    reg [15:0] r_right_depth, r_straight_depth, r_left_depth;
    reg [15:0] r_front_occ_cnt;
    reg [15:0] r_prev_right;

    reg signed [31:0] qx_r, qy_r;
    reg [31:0] move_timer;
    reg [31:0] stale_timer;
    reg [7:0]  right_lock_cnt;
    reg [7:0]  exit_cnt;
    reg [7:0]  bad_streak;
    reg [7:0]  turn_cnt;
    reg [7:0]  scan_cnt;
    reg        inside;
    reg        pending_turn;
    reg        pending_dead;
    reg [15:0] pending_yaw;
    reg        avoid_seen;

    wire signed [15:0] s_q10 = sinA_q10({9'd0, eval_idx});
    wire signed [15:0] c_q10 = cosA_q10({9'd0, eval_idx});
    wire signed [47:0] qy_w  = $signed({1'b0, scan_buf[eval_idx][14:0]}) * s_q10;
    wire signed [47:0] qx_w  = -($signed({1'b0, scan_buf[eval_idx][14:0]}) * c_q10);

    // 当前点投到三个候选方向后的坐标：
    //   straight: (x,y) = (qx,qy)
    //   right:    (x,y) = (-qy,qx)
    //   left:     (x,y) = ( qy,-qx)
    wire signed [31:0] st_x = qx_r;
    wire signed [31:0] st_y = qy_r;
    wire signed [31:0] rt_x = -qy_r;
    wire signed [31:0] rt_y = qx_r;
    wire signed [31:0] lt_x = qy_r;
    wire signed [31:0] lt_y = -qx_r;
    wire signed [31:0] left_x_pos = -qx_r;

    wire [15:0] st_y16 = (st_y > 0) ? st_y[15:0] : 16'd0;
    wire [15:0] rt_y16 = (rt_y > 0) ? rt_y[15:0] : 16'd0;
    wire [15:0] lt_y16 = (lt_y > 0) ? lt_y[15:0] : 16'd0;

    wire st_hit = in_half_width(st_x, CORRIDOR_HALF_MM) &&
                  (st_y >= 32'sd0) && (st_y16 <= FAR_MM);
    wire rt_hit = in_half_width(rt_x, CORRIDOR_HALF_MM) &&
                  (rt_y >= 32'sd0) && (rt_y16 <= FAR_MM);
    wire lt_hit = in_half_width(lt_x, CORRIDOR_HALF_MM) &&
                  (lt_y >= 32'sd0) && (lt_y16 <= FAR_MM);

    wire front_sector = in_angle_win(eval_idx, 90, 20);
    wire left_sector  = in_angle_win(eval_idx, 0, 25);
    wire right_sector = in_angle_win(eval_idx, 180, 25);
    wire side_band    = (qy_r >= -32'sd400) && (qy_r <= 32'sd400);
    wire front_occ    = in_angle_win(eval_idx, 90, 30) &&
                        (scan_buf[eval_idx] != FAR16) &&
                        (scan_buf[eval_idx] <= BLOCK_MM);

    wire right_lock_now = inside ||
                          ((r_right_min != FAR16) &&
                           (r_right_min >= 16'd150) &&
                           (r_right_min <= WALL_LOCK_MM) &&
                           (right_lock_cnt != 8'd0 ? absdiff16(r_right_min, r_prev_right) <= WALL_STABLE_MM : 1'b1));

    wire [15:0] front_exit_ok = (front_mm >= EXIT_FRONT_MM) &&
                                ((r_left_min == FAR16) || (r_left_min >= EXIT_SIDE_MM)) &&
                                ((r_right_min == FAR16) || (r_right_min >= EXIT_SIDE_MM));
    wire [15:0] box_width_deg = r_front_occ_cnt * 16'd5;
    wire box_ok = (~inside) &&
                  (front_mm <= FRONT_ARM_MM) &&
                  (front_mm > 16'd200) &&
                  (box_width_deg <= BOX_MAX_ANG_DEG) &&
                  ((r_right_depth >= LOOKAHEAD_MM) ||
                   (r_left_depth  >= LOOKAHEAD_MM));

    wire right_ok = (r_right_depth >= LOOKAHEAD_MM) &&
                    (r_all_min >= TURN_RADIUS_MM || r_all_min == FAR16);
    wire straight_ok = (r_straight_depth >= LOOKAHEAD_MM);
    wire left_ok = (r_left_depth >= LOOKAHEAD_MM) &&
                   (r_all_min >= TURN_RADIUS_MM || r_all_min == FAR16);
    wire turn_ok = (r_all_min >= TURN_RADIUS_MM) || (r_all_min == FAR16);

    assign ext_to_queue = 1'b0;
    assign ext_slow     = 1'b1;
    assign dbg_inside = inside;
    assign dbg_right_mm = r_right_min;
    assign dbg_left_mm = r_left_min;
    assign dbg_right_depth = r_right_depth;
    assign dbg_straight_depth = r_straight_depth;
    assign dbg_left_depth = r_left_depth;
    assign dbg_turn_min = r_all_min;
    assign dbg_turn_cnt = turn_cnt;
    assign dbg_scan_cnt = scan_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st_code <= S_IDLE; active <= 1'b0; done <= 1'b0; fail <= 1'b0;
            allow_avoid <= 1'b0; heading_changed <= 1'b0;
            ext_load <= 1'b0; ext_act <= A_FWD_HOLD; ext_n <= 32'd0; ext_yaw_tgt <= 16'd0;
            hist_rd_addr <= 7'd0; scan_idx <= 7'd0; eval_idx <= 7'd0;
            scan_pending <= 1'b0; scan_wait1 <= 1'b0;
            r_front_min <= FAR16; r_left_min <= FAR16; r_right_min <= FAR16; r_all_min <= FAR16;
            r_right_depth <= FAR_MM; r_straight_depth <= FAR_MM; r_left_depth <= FAR_MM;
            r_front_occ_cnt <= 16'd0; r_prev_right <= FAR16;
            qx_r <= 32'sd0; qy_r <= 32'sd0;
            move_timer <= 32'd0; stale_timer <= 32'd0;
            right_lock_cnt <= 8'd0; exit_cnt <= 8'd0; bad_streak <= 8'd0;
            turn_cnt <= 8'd0; scan_cnt <= 8'd0;
            inside <= 1'b0; pending_turn <= 1'b0; pending_dead <= 1'b0;
            pending_yaw <= 16'd0; avoid_seen <= 1'b0;
            dbg_why <= 8'h00;
        end else begin
            done <= 1'b0;
            heading_changed <= 1'b0;
            ext_load <= 1'b0;

            if (avoid_active) avoid_seen <= 1'b1;
            if (scan_done) scan_pending <= 1'b1;

            if (active && !scan_fresh) begin
                if (stale_timer < STALE_TO_CYC) stale_timer <= stale_timer + 32'd1;
                else begin
                    active <= 1'b0; fail <= 1'b1; allow_avoid <= 1'b0;
                    ext_load <= 1'b0; dbg_why <= 8'hE1; st_code <= S_FAIL;
                end
            end else begin
                stale_timer <= 32'd0;
            end

            if (abort) begin
                active <= 1'b0; done <= 1'b0; fail <= 1'b0; allow_avoid <= 1'b0;
                ext_load <= 1'b0; st_code <= S_IDLE; dbg_why <= 8'h02;
            end else if (start) begin
                active <= 1'b1; done <= 1'b0; fail <= 1'b0; allow_avoid <= 1'b0;
                ext_load <= 1'b0; scan_pending <= 1'b0; stale_timer <= 32'd0;
                scan_cnt <= 8'd0; turn_cnt <= 8'd0; bad_streak <= 8'd0;
                right_lock_cnt <= 8'd0; exit_cnt <= 8'd0;
                inside <= 1'b0; avoid_seen <= 1'b0;
                right_lock_cnt <= 8'd0;
                st_code <= S_SCAN_REQ; dbg_why <= 8'h10;
            end else if (active) begin
                case (st_code)
                    S_IDLE: begin
                        st_code <= S_SCAN_REQ;
                    end

                    S_SCAN_REQ: begin
                        allow_avoid <= box_ok;
                        if (avoid_fail) begin
                            active <= 1'b0; fail <= 1'b1; dbg_why <= 8'hE2; st_code <= S_FAIL;
                        end else if (avoid_active) begin
                            avoid_seen <= 1'b1;
                            allow_avoid <= 1'b1;
                            st_code <= S_AVOID_WAIT;
                            dbg_why <= 8'hA1;
                        end else if (avoid_seen && avoid_done) begin
                            avoid_seen <= 1'b0;
                            allow_avoid <= 1'b0;
                            st_code <= S_SCAN_REQ;
                            dbg_why <= 8'hA2;
                        end else if (scan_pending) begin
                            scan_pending <= 1'b0;
                            scan_idx <= 7'd0;
                            hist_rd_addr <= 7'd0;
                            scan_wait1 <= 1'b1;
                            st_code <= S_SCAN_ADDR;
                        end
                    end

                    S_SCAN_ADDR: begin
                        if (scan_wait1) begin
                            scan_wait1 <= 1'b0;
                            st_code <= S_SCAN_DATA;
                        end
                    end

                    S_SCAN_DATA: begin
                        scan_buf[scan_idx] <= hist_rd_data;
                        if (scan_idx == 7'd71) begin
                            eval_idx <= 7'd0;
                            r_front_min <= FAR16; r_left_min <= FAR16;
                            r_right_min <= FAR16; r_all_min <= FAR16;
                            r_right_depth <= FAR_MM; r_straight_depth <= FAR_MM;
                            r_left_depth <= FAR_MM; r_front_occ_cnt <= 16'd0;
                            st_code <= S_EVAL_SET;
                            dbg_why <= 8'h20;
                        end else begin
                            scan_idx <= scan_idx + 7'd1;
                            hist_rd_addr <= scan_idx + 7'd1;
                            scan_wait1 <= 1'b1;
                            st_code <= S_SCAN_ADDR;
                        end
                    end

                    S_EVAL_SET: begin
                        qx_r <= qx_w >>> 10;
                        qy_r <= qy_w >>> 10;
                        st_code <= S_EVAL_ACC;
                    end

                    S_EVAL_ACC: begin
                        if (scan_buf[eval_idx] != FAR16) begin
                            r_all_min <= min16(r_all_min, scan_buf[eval_idx]);
                            if (front_sector)
                                r_front_min <= min16(r_front_min, scan_buf[eval_idx]);
                            // 侧墙距离取“近横向带内的垂直距离 qx”，不取斜距；
                            // 这样车身只偏 5° 时，250mm 侧墙仍保持约 250mm。
                            if (left_sector && side_band && (qx_r < 0))
                                r_left_min <= min16(r_left_min, left_x_pos[15:0]);
                            if (right_sector && side_band && (qx_r > 0))
                                r_right_min <= min16(r_right_min, qx_r[15:0]);
                            if (front_occ)
                                r_front_occ_cnt <= r_front_occ_cnt + 16'd1;
                        end
                        if (st_hit && (st_y16 < r_straight_depth))
                            r_straight_depth <= st_y16;
                        if (rt_hit && (rt_y16 < r_right_depth))
                            r_right_depth <= rt_y16;
                        if (lt_hit && (lt_y16 < r_left_depth))
                            r_left_depth <= lt_y16;

                        if (eval_idx == 7'd71) begin
                            scan_cnt <= scan_cnt + 8'd1;
                            st_code <= S_DECIDE;
                            dbg_why <= 8'h21;
                        end else begin
                            eval_idx <= eval_idx + 7'd1;
                            st_code <= S_EVAL_SET;
                        end
                    end

                    S_DECIDE: begin
                        // 默认撤销“可让避障”许可；只有本圈仍确认是小箱子时才重新置位。
                        allow_avoid <= box_ok;
                        // 右墙锁存与出口计数都使用刚刚完成的整圈快照。
                        if (right_lock_now) begin
                            if (right_lock_cnt != 8'hFF) right_lock_cnt <= right_lock_cnt + 8'd1;
                        end else begin
                            right_lock_cnt <= 8'd0;
                        end
                        r_prev_right <= r_right_min;

                        if (!inside && right_lock_now &&
                            (right_lock_cnt >= (PASS_K - 8'd1))) begin
                            inside <= 1'b1;
                            dbg_why <= 8'h30;
                        end

                        if (inside && front_exit_ok) begin
                            if (exit_cnt != 8'hFF) exit_cnt <= exit_cnt + 8'd1;
                        end else begin
                            exit_cnt <= 8'd0;
                        end

                        if (inside && front_exit_ok &&
                            (exit_cnt >= (PASS_K - 8'd1))) begin
                            active <= 1'b0; done <= 1'b1; allow_avoid <= 1'b0;
                            dbg_why <= 8'h50; st_code <= S_DONE;
                        end else if (avoid_fail) begin
                            active <= 1'b0; fail <= 1'b1; allow_avoid <= 1'b0;
                            dbg_why <= 8'hE2; st_code <= S_FAIL;
                        end else if (box_ok) begin
                            // 开阔区小箱子：继续低速接近，让现有 avoid_fsm 在 450mm 接管。
                            allow_avoid <= 1'b1;
                            ext_act <= A_FWD_HOLD;
                            ext_n <= STEP_MM * CNT_PER_MM;
                            ext_yaw_tgt <= yaw_now;
                            ext_load <= 1'b1;
                            move_timer <= 32'd0;
                            pending_turn <= 1'b0;
                            st_code <= S_MOVE_WAIT;
                            dbg_why <= 8'hA0;
                        end else if (front_mm <= FRONT_ARM_MM || inside) begin
                            // 严格右墙优先；每个候选必须已通过 400×700 管道检查。
                            if (right_ok) begin
                                if (!yaw_ok || !turn_ok) begin
                                    active <= 1'b0; fail <= 1'b1;
                                    dbg_why <= 8'hE3; st_code <= S_FAIL;
                                end else begin
                                    pending_yaw <= yaw_now - 16'd16384;
                                    ext_act <= A_YAW_ABS;
                                    ext_n <= N_TURN_90;
                                    ext_yaw_tgt <= yaw_now - 16'd16384;
                                    ext_load <= 1'b1;
                                    pending_turn <= 1'b1;
                                    pending_dead <= 1'b0;
                                    move_timer <= 32'd0;
                                    dbg_why <= 8'h40;
                                    st_code <= S_MOVE_WAIT;
                                end
                            end else if (straight_ok) begin
                                ext_act <= A_FWD_HOLD;
                                ext_n <= STEP_MM * CNT_PER_MM;
                                if (r_right_min != FAR16 && r_right_min > (SIDE_TARGET_MM + 16'd80))
                                    ext_yaw_tgt <= yaw_now - 16'd910;   // 向右 5°
                                else if (r_right_min != FAR16 && r_right_min + 16'd50 < SIDE_TARGET_MM)
                                    ext_yaw_tgt <= yaw_now + 16'd910;   // 向左 5°
                                else
                                    ext_yaw_tgt <= yaw_now;
                                ext_load <= 1'b1;
                                pending_turn <= 1'b0;
                                move_timer <= 32'd0;
                                bad_streak <= 8'd0;
                                dbg_why <= 8'h41;
                                st_code <= S_MOVE_WAIT;
                            end else if (left_ok) begin
                                if (!yaw_ok || !turn_ok) begin
                                    active <= 1'b0; fail <= 1'b1;
                                    dbg_why <= 8'hE3; st_code <= S_FAIL;
                                end else begin
                                    pending_yaw <= yaw_now + 16'd16384;
                                    ext_act <= A_YAW_ABS;
                                    ext_n <= N_TURN_90;
                                    ext_yaw_tgt <= yaw_now + 16'd16384;
                                    ext_load <= 1'b1;
                                    pending_turn <= 1'b1;
                                    pending_dead <= 1'b0;
                                    move_timer <= 32'd0;
                                    dbg_why <= 8'h42;
                                    st_code <= S_MOVE_WAIT;
                                end
                            end else if (turn_ok && yaw_ok) begin
                                // 三面皆堵：死胡同掉头；转轴圆圈由 235mm 半径保守覆盖。
                                pending_yaw <= yaw_now + 16'd32768;
                                ext_act <= A_YAW_ABS;
                                ext_n <= N_TURN_90 << 1;
                                ext_yaw_tgt <= yaw_now + 16'd32768;
                                ext_load <= 1'b1;
                                pending_turn <= 1'b1;
                                pending_dead <= 1'b1;
                                move_timer <= 32'd0;
                                dbg_why <= 8'h43;
                                st_code <= S_MOVE_WAIT;
                            end else begin
                                if (bad_streak >= FAIL_K) begin
                                    active <= 1'b0; fail <= 1'b1;
                                    dbg_why <= 8'hE4; st_code <= S_FAIL;
                                end else begin
                                    bad_streak <= bad_streak + 8'd1;
                                    st_code <= S_SCAN_REQ;
                                    dbg_why <= 8'hE5;
                                end
                            end
                        end else begin
                            // 开阔区或长直路：慢慢向前，下一步重新测。
                            ext_act <= A_FWD_HOLD;
                            ext_n <= STEP_MM * CNT_PER_MM;
                            ext_yaw_tgt <= yaw_now;
                            ext_load <= 1'b1;
                            pending_turn <= 1'b0;
                            move_timer <= 32'd0;
                            dbg_why <= 8'h11;
                            st_code <= S_MOVE_WAIT;
                        end
                    end

                    S_MOVE_WAIT: begin
                        if (avoid_active) begin
                            allow_avoid <= 1'b1;
                            avoid_seen <= 1'b1;
                            dbg_why <= 8'hA1;
                            st_code <= S_AVOID_WAIT;
                        end else if (ext_done) begin
                            if (pending_turn) begin
                                pending_turn <= 1'b0;
                                heading_changed <= 1'b1;
                                turn_cnt <= turn_cnt + 8'd1;
                                if (pending_dead) begin
                                    pending_dead <= 1'b0;
                                    dbg_why <= 8'h44;
                                end else dbg_why <= 8'h45;
                            end
                            st_code <= S_SCAN_REQ;
                        end else if (move_timer >= MOVE_TO_CYC) begin
                            active <= 1'b0; fail <= 1'b1; allow_avoid <= 1'b0;
                            dbg_why <= 8'hE6; st_code <= S_FAIL;
                        end else move_timer <= move_timer + 32'd1;
                    end

                    S_AVOID_WAIT: begin
                        // 动作总线此时归 avoid_fsm；迷宫绝不发第二个 ext_load。
                        if (avoid_fail) begin
                            active <= 1'b0; fail <= 1'b1; allow_avoid <= 1'b0;
                            dbg_why <= 8'hE2; st_code <= S_FAIL;
                        end else if (avoid_done) begin
                            allow_avoid <= 1'b0;
                            avoid_seen <= 1'b0;
                            dbg_why <= 8'hA2;
                            st_code <= S_SCAN_REQ;
                        end else if (!avoid_active) begin
                            // avoid 尚未真正启动时允许重新看圈，防止卡在等待态。
                            if (bad_streak >= FAIL_K) begin
                                active <= 1'b0; fail <= 1'b1; allow_avoid <= 1'b0;
                                dbg_why <= 8'hE7; st_code <= S_FAIL;
                            end else begin
                                bad_streak <= bad_streak + 8'd1;
                                st_code <= S_SCAN_REQ;
                            end
                        end
                    end

                    S_DONE, S_FAIL: begin
                        active <= 1'b0;
                        allow_avoid <= 1'b0;
                    end

                    default: begin
                        active <= 1'b0; fail <= 1'b1;
                        dbg_why <= 8'hEF; st_code <= S_FAIL;
                    end
                endcase
            end
        end
    end

endmodule
