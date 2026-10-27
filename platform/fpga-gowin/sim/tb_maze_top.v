`timescale 1ns/1ps
// V5.0b integration smoke test:
// 1) SW1 selects maze mode and S1 starts only maze, not the old queue;
// 2) avoid_fsm owns the ext bus whenever avoid_active is high;
// 3) maze owns it only after avoid is inactive.
module tb_maze_top;
    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg [7:0] sw = 8'hFF;
    reg key_stop = 1'b1;
    reg key_start = 1'b1;
    reg key_fwd = 1'b1;
    reg key_left = 1'b1;
    reg key_r = 1'b1;
    reg enc_al = 1'b0, enc_bl = 1'b0, enc_ar = 1'b0, enc_br = 1'b0;
    reg imu_rx = 1'b1, lidar_rx = 1'b1, ir_rx = 1'b1;

    wire ain1_l, ain2_l, pwma, ain1_r, ain2_r, pwmb, stby;
    wire lidar_tx, uart_tx, beep, bt_tx;
    wire [7:0] led;

    integer errors = 0;
    integer start_seen = 0;
    always @(posedge clk) if (dut.maze_start_req) start_seen = start_seen + 1;

    final_leida_top #(
        .BT_BAUD(115200),
        .BEEP_DEAD_DLY_MS(50)
    ) dut (
        .clk(clk), .key_stop(key_stop), .key_start(key_start),
        .key_fwd(key_fwd), .key_left(key_left), .key_r(key_r), .sw(sw),
        .ain1_l(ain1_l), .ain2_l(ain2_l), .pwma(pwma),
        .ain1_r(ain1_r), .ain2_r(ain2_r), .pwmb(pwmb), .stby(stby),
        .enc_al(enc_al), .enc_bl(enc_bl), .enc_ar(enc_ar), .enc_br(enc_br),
        .imu_rx(imu_rx), .lidar_rx(lidar_rx), .lidar_tx(lidar_tx),
        .ir_rx(ir_rx), .uart_tx(uart_tx), .beep(beep), .bt_tx(bt_tx), .led(led)
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

    initial begin
        // 顶层真实 POR 是 40ms；本测试只验接口仲裁，直接置高 por_done 缩短仿真。
        #100;
        force dut.por_done = 1'b1;
        #100;
        wait (dut.rst_n === 1'b1);
        repeat (20) @(posedge clk);

        // 正常模式不得启动迷宫。
        sw = 8'hFF;
        @(posedge clk); key_start <= 1'b0;
        repeat (3) @(posedge clk); key_start <= 1'b1;
        repeat (10) @(posedge clk);
        chk(dut.maze_active == 1'b0, "normal mode must not start maze");

        // SW1=ON，S1 启动迷宫；启动瞬间清旧队列，但不得被清队列脉冲立即 abort。
        sw = 8'b1111_1101;
        @(posedge clk); key_start <= 1'b0;
        repeat (3) @(posedge clk); key_start <= 1'b1;
        repeat (12) @(posedge clk);
        $display("maze start diag mode=%b seen=%0d hold=%0d active=%b abort=%b motor_clr=%b st=%0d",
                 dut.maze_mode, start_seen, dut.maze_start_hold, dut.maze_active,
                 dut.maze_abort, dut.motor_clr, dut.u_maze.st_code);
        chk(dut.maze_active == 1'b1, "maze mode S1 must start maze");
        chk(dut.u_motor.q_cnt == 3'd0, "maze start must leave old queue empty");

        // 强制两端同时请求动作：避障必须优先，不得把迷宫动作送到电机。
        force dut.avoid_active = 1'b1;
        force dut.av_ext_load = 1'b1;
        force dut.av_ext_act = 3'd0;
        force dut.av_ext_n = 32'd123;
        force dut.av_hist_addr = 7'd10;
        force dut.maze_hist_addr = 7'd40;
        force dut.maze_active = 1'b1;
        force dut.maze_ext_load = 1'b1;
        force dut.maze_ext_act = 3'd6;
        force dut.maze_ext_n = 32'd456;
        repeat (3) @(posedge clk);
        chk(dut.ext_load == 1'b1, "avoid request must reach motor");
        chk(dut.ext_act == 3'd0, "avoid must win ext_act arbitration");
        chk(dut.ext_n == 32'd123, "avoid must win ext_n arbitration");
        chk(dut.u_lidar_parse.hist_rd_addr == 7'd10, "avoid must win histogram reader");

        // 撤掉避障后，迷宫才拥有动作总线。
        force dut.avoid_active = 1'b0;
        repeat (3) @(posedge clk);
        chk(dut.ext_act == 3'd6, "maze must own ext bus after avoid is inactive");
        chk(dut.ext_n == 32'd456, "maze step length must reach motor");
        chk(dut.u_lidar_parse.hist_rd_addr == 7'd40, "maze must own histogram reader");

        release dut.avoid_active;
        release dut.av_ext_load;
        release dut.av_ext_act;
        release dut.av_ext_n;
        release dut.av_hist_addr;
        release dut.maze_hist_addr;
        release dut.maze_active;
        release dut.maze_ext_load;
        release dut.maze_ext_act;
        release dut.maze_ext_n;
        release dut.por_done;

        if (errors == 0) $display("==== tb_maze_top PASS ====");
        else             $display("==== tb_maze_top FAIL errors=%0d ====", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("==== tb_maze_top TIMEOUT errors=%0d ====", errors);
        $finish;
    end
endmodule
