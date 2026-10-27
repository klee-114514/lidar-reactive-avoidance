//=============================================================================
// uart_tx.v —— 串口发送模块（只发不收）
//-----------------------------------------------------------------------------
// 与 motor_driver 工程一致：9600-8-N-1，接 FPGA_UART_TX（M15）。
// 空闲线为高；起始位低 1 位 → 8 数据位低位在前 → 停止位高 1 位。
//=============================================================================
module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 9600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx,
    output reg        tx_busy
);

    localparam BAUD_DIV = CLK_FREQ / BAUD;

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [12:0] cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_lat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            cnt      <= 13'd0;
            bit_idx  <= 3'd0;
            data_lat <= 8'd0;
            tx       <= 1'b1;
            tx_busy  <= 1'b0;
        end
        else begin
            case (state)
                S_IDLE: begin
                    tx      <= 1'b1;
                    tx_busy <= 1'b0;
                    cnt     <= 13'd0;
                    bit_idx <= 3'd0;
                    if (tx_start) begin
                        data_lat <= tx_data;
                        tx_busy  <= 1'b1;
                        state    <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0;
                    if (cnt >= BAUD_DIV - 1) begin
                        cnt   <= 13'd0;
                        state <= S_DATA;
                    end
                    else
                        cnt <= cnt + 13'd1;
                end

                S_DATA: begin
                    tx <= data_lat[bit_idx];
                    if (cnt >= BAUD_DIV - 1) begin
                        cnt   <= 13'd0;
                        if (bit_idx >= 3'd7)
                            state <= S_STOP;
                        else
                            bit_idx <= bit_idx + 3'd1;
                    end
                    else
                        cnt <= cnt + 13'd1;
                end

                S_STOP: begin
                    tx <= 1'b1;
                    if (cnt >= BAUD_DIV - 1) begin
                        cnt   <= 13'd0;
                        state <= S_IDLE;
                    end
                    else
                        cnt <= cnt + 13'd1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
