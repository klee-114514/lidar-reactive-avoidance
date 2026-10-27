//=============================================================================
// imu_parse.v —— 维特 0x55 姿态帧解析（Yaw）
//-----------------------------------------------------------------------------
// 帧：0x55 | type | d0..d7 | sum
//   type=0x53 角度：Roll/Pitch/Yaw 各 int16 小端
//   Yaw = raw/32768*180° → 90° 对应 raw≈16384
// 本模块输出 yaw_raw（有符号 16 位）与 yaw_valid 脉冲。
//=============================================================================
module imu_parse (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output reg  [15:0] yaw_raw,
    output reg         yaw_valid,
    output reg         frame_ok
);
    localparam S_HUNT = 3'd0, S_TYPE = 3'd1, S_DATA = 3'd2, S_SUM = 3'd3;
    reg [2:0]  st;
    reg [7:0]  type;
    reg [7:0]  d [0:7];
    reg [2:0]  idx;
    reg [7:0]  sum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_HUNT; type <= 0; idx <= 0; sum <= 0;
            yaw_raw <= 0; yaw_valid <= 0; frame_ok <= 0;
        end else begin
            yaw_valid <= 1'b0;
            frame_ok  <= 1'b0;
            if (rx_valid) begin
                case (st)
                    S_HUNT:
                        if (rx_data == 8'h55) begin
                            sum <= 8'h55;
                            st  <= S_TYPE;
                        end
                    S_TYPE: begin
                        type <= rx_data;
                        sum  <= sum + rx_data;
                        idx  <= 0;
                        st   <= S_DATA;
                    end
                    S_DATA: begin
                        d[idx] <= rx_data;
                        sum    <= sum + rx_data;
                        if (idx >= 3'd7) st <= S_SUM;
                        else idx <= idx + 3'd1;
                    end
                    S_SUM: begin
                        if (rx_data == sum) begin
                            frame_ok <= 1'b1;
                            // 0x53：d[4]=yaw_lo, d[5]=yaw_hi
                            if (type == 8'h53) begin
                                yaw_raw   <= {d[5], d[4]};
                                yaw_valid <= 1'b1;
                            end
                        end
                        st <= S_HUNT;
                    end
                    default: st <= S_HUNT;
                endcase
            end
        end
    end
endmodule
