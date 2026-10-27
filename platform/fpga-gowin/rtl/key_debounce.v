//=============================================================================
// key_debounce.v —— 按键消抖 + 单击/连发输出
//-----------------------------------------------------------------------------
// 【为什么按键必须消抖】这个必须搞懂，不然你调参时会疯
//
//   按键内部的金属弹片是按下去、弹起来的过程，不是干净的"通/断"。
//   一次按压在电气上实际长这样（T = 你按下那一刻）：
//
//     ─────┐ ┌─┐ ┌─┐ ┌───────────────────────────┐ ┌─┐ ┌─────
//          └─┘ └─┘ └─┘                           └─┘ └─┘
//          ↑                                    ↑
//       按下瞬间抖 10~20 次                   松开也抖
//
//   如果直接拿这个信号去"每来一次就 +1"，按一下会加十几次 —— 参数根本没法调。
//
//   **解决办法（工业标准做法）**：
//     连续 N 毫秒都读到同一个值，才认为它真的变了。
//     抖动通常几毫秒内结束，所以取 20ms 很稳。
//     20ms @ 50MHz = 1_000_000 个时钟。
//
// 【两个输出，对应两种用法】
//   · key_down  —— 按下的那一刻给一个时钟的高脉冲（"单击"）
//                  用于：切换 KP/KI/KD 这种"按一下变一次"的操作
//   · key_repeat—— 按住不放时，每 REPEAT_MS 毫秒给一个脉冲（"连发"）
//                  用于：连续加减参数，不用按到手酸
//
//   我们让按键 S1/S2/S3 同时输出这两种，上层自己选用哪种。
//=============================================================================
module key_debounce #(
    parameter CLK_FREQ   = 50_000_000,
    parameter DEBOUNCE_MS = 20,      // 消抖时长（毫秒）
    parameter REPEAT_MS   = 200,     // 按住时的连发周期（毫秒）
    parameter ACTIVE_LOW  = 1        // 1 = 按下时读到 0（本板按键是这样）
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,      // 直接来自引脚
    output reg  key_down,    // 按下瞬间：单周期脉冲
    output reg  key_repeat,  // 按住不放：每 REPEAT_MS 一个脉冲
    output reg  key_level    // 消抖后的电平：1 = 正被按着
);

    localparam DEB_CYC = CLK_FREQ / 1000 * DEBOUNCE_MS;
    localparam REP_CYC = CLK_FREQ / 1000 * REPEAT_MS;

    // 两级同步 + 消抖计数
    reg  [1:0]  sync;
    reg  [19:0] deb_cnt;
    reg         key_state;      // 消抖后的稳定状态（1 = 按下）
    reg         key_state_d;    // 上一拍的稳定状态（用来抓"按下"这个边沿）
    reg  [19:0] rep_cnt;        // 连发计时器
    reg         rep_armed;      // 连发计时器是否已开始跑

    wire        key_raw;        // 归一化成"1 = 按下"

    assign key_raw = ACTIVE_LOW ? ~sync[1] : sync[1];

    // =========================================================================
    // ⚠️ 踩过的坑（2026-09-20）：不要把"消抖判定"和"边沿检测"混在同一段逻辑里
    //
    //   最初的写法是：在 deb_cnt 数够 DEB_CYC 的那一拍，直接 if(key_raw) 就输出
    //   key_down 脉冲。这有个隐患：那一拍 key_raw 恰好又抖回去的话，
    //   就会漏掉或重复产生脉冲。
    //
    //   正确做法是分两级：
    //     第一级：消抖计数 → 只负责把 key_state 这个【稳定电平】维护对
    //     第二级：拿 key_state 的上升沿 → 产生 key_down 脉冲
    //   两级之间用 key_state_d 做"上一拍快照"，边沿检测就是 std 的
    //   `key_state & ~key_state_d`。
    //   这样 key_down 的宽度精确是 1 个时钟，且与抖动过程完全解耦。
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync        <= 2'b00;
            deb_cnt     <= 20'd0;
            key_state   <= 1'b0;
            key_state_d <= 1'b0;
            key_down    <= 1'b0;
            key_repeat  <= 1'b0;
            key_level   <= 1'b0;
            rep_cnt     <= 20'd0;
            rep_armed   <= 1'b0;
        end
        else begin
            // ---- 0) 板外信号先打两拍（跨时钟域同步）----
            sync <= {sync[0], key_in};

            // ---- 1) 第一级：消抖，只维护 key_state 这个稳定电平 ----
            if (key_raw == key_state) begin
                // 读到的和当前认定的一致 → 计时器清零
                deb_cnt <= 20'd0;
            end
            else if (deb_cnt >= DEB_CYC - 1) begin
                // 连续 DEB_CYC 拍都读到相反值 → 正式改判
                deb_cnt   <= 20'd0;
                key_state <= key_raw;
            end
            else begin
                deb_cnt <= deb_cnt + 20'd1;
            end

            // ---- 2) 第二级：用稳定电平的上升沿产生"按下"脉冲 ----
            key_state_d <= key_state;
            key_down    <= key_state & ~key_state_d;   // 只在这一拍为高
            key_level   <= key_state;

            // ---- 3) 连发：按下期间每 REP_CYC 给一个脉冲 ----
            //    第一次"按下"已经由 key_down 给了，所以连发只负责之后的重复。
            if (key_state) begin
                if (rep_armed) begin
                    if (rep_cnt >= REP_CYC - 1) begin
                        rep_cnt    <= 20'd0;
                        key_repeat <= 1'b1;
                    end
                    else begin
                        rep_cnt <= rep_cnt + 20'd1;
                    end
                end
                else begin
                    // 刚检测到按下，先把计时器架起来，这次不发脉冲
                    //（避免和 key_down 撞在同一拍）
                    rep_armed <= 1'b1;
                    rep_cnt   <= 20'd0;
                end
            end
            else begin
                rep_armed  <= 1'b0;
                rep_cnt    <= 20'd0;
                key_repeat <= 1'b0;
            end
        end
    end

endmodule
