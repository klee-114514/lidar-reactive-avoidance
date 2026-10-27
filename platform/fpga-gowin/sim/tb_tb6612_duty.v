//=============================================================================
// tb_tb6612_duty.v —— 验证"千分比 → PWM 阈值"改写后仍然精确
//-----------------------------------------------------------------------------
// 2026-10-08 把  pwm_thr = duty_lim * PWM_PERIOD / DUTY_MAX   （变量除法）
//          改成  pwm_thr <= (duty_lim * THR_MUL) >> THR_SHIFT （常数乘+右移，打一拍）
// 本 tb 直接数"一个完整 PWM 周期里 pwm 高了多少拍"：
//   duty=1000(‰) → 2500 拍全高；500 → 1250；250 → 625；0 → 0；−500 → 1250 且 AIN 反向
// 运行：vlog -work work ../rtl/tb6612_driver.v tb_tb6612_duty.v
//       vsim -c -do "run -all; quit -f" work.tb_tb6612_duty
//=============================================================================
`timescale 1ns/1ps

module tb_tb6612_duty;

    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50MHz

    reg               rst_n      = 1'b0;
    reg               pwm_enable = 1'b1;
    reg               stop_mode  = 1'b1;
    reg signed [15:0] duty_cmd   = 16'sd0;

    wire ain1, ain2, pwm;

    tb6612_driver #(.CLK_FREQ(50_000_000), .PWM_FREQ(20_000), .DUTY_MAX(1000)) dut (
        .clk(clk), .rst_n(rst_n), .pwm_enable(pwm_enable), .stop_mode(stop_mode),
        .duty_cmd(duty_cmd), .ain1(ain1), .ain2(ain2), .pwm(pwm)
    );

    integer i;
    integer hi;
    integer fails = 0;

    task test_duty;
        input signed [15:0] d;
        input integer       exp_hi;      // 期望一个 2500 拍周期内的高电平拍数
        input [1:0]         exp_dir;     // 2'b10 = 正转(ain1=1,ain2=0), 2'b01 = 反转, 2'b11 = 刹车
        begin
            duty_cmd = d;
            repeat (5) @(posedge clk);   // 阈值打一拍 + 稳定
            hi = 0;
            for (i = 0; i < 2500; i = i + 1) begin   // 正好一个 PWM 周期
                @(posedge clk);
                if (pwm) hi = hi + 1;
            end
            if ((hi > exp_hi + 1) || (hi < exp_hi - 1)) begin
                $display("  duty=%0d  pwm高=%0d 拍 (期望 %0d)  <== MISMATCH", d, hi, exp_hi);
                fails = fails + 1;
            end else
                $display("  duty=%0d  pwm高=%0d 拍 (期望 %0d)  OK", d, hi, exp_hi);
            if ({ain1, ain2} !== exp_dir) begin
                $display("    方向错: ain1=%b ain2=%b (期望 %b)", ain1, ain2, exp_dir);
                fails = fails + 1;
            end
        end
    endtask

    initial begin
        #200 rst_n = 1'b1;
        #2000;
        $display("==== tb_tb6612_duty ====");
        test_duty( 16'sd0,      0, 2'b11);   // 停：短路刹车
        test_duty( 16'sd250,  625, 2'b10);
        test_duty( 16'sd500, 1250, 2'b10);
        test_duty( 16'sd1000,2500, 2'b10);
        test_duty(-16'sd500, 1250, 2'b01);   // 反转：同样占空比
        test_duty( 16'sd3,      7, 2'b10);   // 小占空比：3×2.5=7.5 → 7
        if (fails == 0) $display("==== tb_tb6612_duty PASS ====");
        else            $display("==== tb_tb6612_duty FAIL (%0d) ====", fails);
        $finish;
    end

endmodule
