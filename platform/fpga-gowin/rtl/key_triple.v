//=============================================================================
// key_triple.v —— 一键三义：单击 / 双击 / 长按 + 消抖电平
//-----------------------------------------------------------------------------
// 【输出】
//   click_single  ：松手且未双击、未长按 → 一个时钟脉冲
//   click_double  ：窗口内第 2 下 → 一个时钟脉冲（在第 2 下【按下】时给）
//   click_long    ：按住满 LONG_MS → 一个时钟脉冲（只给一次）
//   key_level     ：消抖后电平，1 = 正被按着（急停保底用）
//
// 【判决顺序】
//   按下 → 开始计长按
//     若一直按到 LONG_MS     → click_long，等松手回 IDLE（不再发单击）
//     若中途松手             → 开双击窗
//       窗内再按下           → click_double
//       窗超时无第二次       → click_single
//
// 【为什么长按优先】
//   长按若不优先，按住 1 秒会先被拆成一次单击再误判——参数入队会乱。
//=============================================================================
module key_triple #(
    parameter CLK_FREQ    = 50_000_000,
    parameter DEBOUNCE_MS = 20,
    parameter DOUBLE_MS   = 400,
    parameter LONG_MS     = 800,
    parameter ACTIVE_LOW  = 1
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,
    output reg  click_single,
    output reg  click_double,
    output reg  click_long,
    output wire key_level
);

    localparam DBL_CYC  = CLK_FREQ / 1000 * DOUBLE_MS;
    localparam LONG_CYC = CLK_FREQ / 1000 * LONG_MS;

    // ---- 声明集中 ----
    wire key_down;
    wire key_rise;
    wire key_fall;
    reg  key_level_d;
    reg  [1:0]  st_k;
    reg  [31:0] cnt_k;

    localparam K_IDLE  = 2'd0;
    localparam K_PRESS = 2'd1;   // 按住中，等长按或松手
    localparam K_LATCH = 2'd2;   // 已发长按/双击，等松手
    localparam K_WAIT2 = 2'd3;   // 已松手，等第二击

    key_debounce #(
        .CLK_FREQ    (CLK_FREQ),
        .DEBOUNCE_MS (DEBOUNCE_MS),
        .REPEAT_MS   (200),
        .ACTIVE_LOW  (ACTIVE_LOW)
    ) u_db (
        .clk        (clk),
        .rst_n      (rst_n),
        .key_in     (key_in),
        .key_down   (key_down),
        .key_repeat (),
        .key_level  (key_level)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            key_level_d <= 1'b0;
        else
            key_level_d <= key_level;
    end

    assign key_rise = key_level & ~key_level_d;
    assign key_fall = ~key_level & key_level_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st_k         <= K_IDLE;
            cnt_k        <= 32'd0;
            click_single <= 1'b0;
            click_double <= 1'b0;
            click_long   <= 1'b0;
        end
        else begin
            click_single <= 1'b0;
            click_double <= 1'b0;
            click_long   <= 1'b0;

            case (st_k)
                K_IDLE: begin
                    cnt_k <= 32'd0;
                    if (key_rise) begin
                        st_k  <= K_PRESS;
                        cnt_k <= 32'd0;
                    end
                end

                K_PRESS: begin
                    if ((cnt_k + 32'd1) >= LONG_CYC) begin
                        click_long <= 1'b1;
                        st_k       <= K_LATCH;
                        cnt_k      <= 32'd0;
                    end
                    else begin
                        cnt_k <= cnt_k + 32'd1;
                        if (key_fall) begin
                            st_k  <= K_WAIT2;
                            cnt_k <= 32'd0;
                        end
                    end
                end

                K_LATCH: begin
                    cnt_k <= 32'd0;
                    if (key_fall)
                        st_k <= K_IDLE;
                end

                K_WAIT2: begin
                    if (key_rise) begin
                        click_double <= 1'b1;
                        st_k         <= K_LATCH;
                        cnt_k        <= 32'd0;
                    end
                    else if ((cnt_k + 32'd1) >= DBL_CYC) begin
                        click_single <= 1'b1;
                        st_k         <= K_IDLE;
                        cnt_k        <= 32'd0;
                    end
                    else
                        cnt_k <= cnt_k + 32'd1;
                end

                default: st_k <= K_IDLE;
            endcase
        end
    end

endmodule
