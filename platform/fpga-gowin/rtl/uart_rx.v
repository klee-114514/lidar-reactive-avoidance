//=============================================================================
// uart_rx.v —— 简单串口接收 9600-8-N-1
//=============================================================================
module uart_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 9600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);
    localparam BAUD_DIV = CLK_FREQ / BAUD;
    localparam S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;
    reg [1:0]  st;
    reg [12:0] cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;
    reg        rx_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; cnt <= 0; bit_idx <= 0; shreg <= 0;
            data <= 0; valid <= 0; rx_d <= 1'b1;
        end else begin
            rx_d  <= rx;
            valid <= 1'b0;
            case (st)
                S_IDLE: begin
                    cnt <= 0; bit_idx <= 0;
                    if (rx_d && !rx) // 下降沿起始
                        st <= S_START;
                end
                S_START: begin
                    if (cnt >= (BAUD_DIV/2)) begin
                        cnt <= 0;
                        if (!rx) st <= S_DATA;
                        else     st <= S_IDLE; // 假起始
                    end else cnt <= cnt + 13'd1;
                end
                S_DATA: begin
                    if (cnt >= BAUD_DIV-1) begin
                        cnt <= 0;
                        shreg <= {rx, shreg[7:1]}; // 先收低位
                        if (bit_idx >= 3'd7) st <= S_STOP;
                        else bit_idx <= bit_idx + 3'd1;
                    end else cnt <= cnt + 13'd1;
                end
                S_STOP: begin
                    if (cnt >= BAUD_DIV-1) begin
                        cnt <= 0;
                        data  <= shreg;
                        valid <= 1'b1;
                        st    <= S_IDLE;
                    end else cnt <= cnt + 13'd1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
