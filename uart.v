module uart
    #(
        parameter DBIT = 8,    // Number of data bits
        parameter SB_TICK = 16, // Stop bit duration in ticks
        parameter DVSR = 326    // Baud rate divisor
    )
    (
        input wire clk,
        input wire rst_n,
        input wire rx,                // UART receive pin
        output wire tx,               // UART transmit pin
        input wire rd_uart,           // UART read enable
        input wire wr_uart,           // UART write enable
        input wire [7:0] wr_data,     // Data to write
        output reg [7:0] rd_data,     // Data received
        output reg rx_empty,          // Receive buffer empty flag
        output reg tx_full            // Transmit buffer full flag
    );

    // Internal signals
    wire tick;                        
    wire rx_done_tick, tx_done_tick;  
    wire [7:0] rx_buffer;              
    wire tx_wire;                     

    // Baud rate generator instance
    baud_gen baud_gen_inst (
        .clk(clk),
        .reset(rst_n),
        .baud_clk(tick)
    );

    // UART receiver instance
    uart_rx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_rx_inst (
        .clk(clk),
        .reset(rst_n),
        .data_in(rx),
        .s_tick(tick),
        .rx_done_tick(rx_done_tick),
        .data_out(rx_buffer)
    );

    // UART transmitter instance
    uart_tx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_tx_inst (
        .clk(clk),
        .reset(rst_n),
        .data_in(wr_data),
        .s_tick(tick),
        .tx_start(wr_uart),
        .tx_done_tick(tx_done_tick),
        .data_out(tx_wire)
    );

    assign tx = tx_wire;

    // RX and TX control logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_data <= 8'b0;
            rx_empty <= 1'b1;
            tx_full <= 1'b0;
        end else begin
            if (rx_done_tick) begin
                rd_data <= rx_buffer;
                rx_empty <= 1'b0;
            end
            if (rd_uart) begin
                rx_empty <= 1'b1;
            end

            if (wr_uart && !tx_full) begin
                tx_full <= 1'b1;
            end
            if (tx_done_tick) begin
                tx_full <= 1'b0;
            end
        end
    end
endmodule
