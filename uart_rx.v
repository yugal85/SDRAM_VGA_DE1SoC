`timescale 1ns / 1ps

module uart_rx
  #(
    parameter DBIT = 8,          // Number of data bits
              SB_TICK = 16       // Number of ticks for the stop bit
  )
  ( 
    input wire clk,
    input wire reset,
    input wire data_in,
    input wire s_tick,
    output reg rx_done_tick,     // Signal that data reception is complete
    output wire [7:0] data_out   // Received data output
  );

  // State declaration for FSM
  localparam [1:0] 
    IDLE  = 2'b00, 
    START = 2'b01, 
    DATA  = 2'b10, 
    STOP  = 2'b11;

  // Internal signal declarations
  reg [1:0] state_reg, state_next;
  reg [3:0] cnt_15_reg, cnt_15_next;   // Sampling tick counter (4-bit)
  reg [2:0] cnt_8_reg, cnt_8_next;     // Bit counter (3-bit)
  reg [7:0] shift_out_reg, shift_out_next;  // Shift register for incoming data

  // Constants for bit-length matching
  localparam [2:0] DBIT_MAX = 3'b111;        // Set explicitly to the maximum value (7 for 3 bits)
  localparam [3:0] SB_TICK_MAX = 4'b1111;    // Set explicitly to the maximum value (15 for 4 bits)

  // FSMD state & data registers
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      state_reg <= IDLE;
      cnt_15_reg <= 4'b0000;
      cnt_8_reg <= 3'b000;
      shift_out_reg <= 8'b00000000;
      rx_done_tick <= 1'b0;
    end else begin
      state_reg <= state_next;
      cnt_15_reg <= cnt_15_next;
      cnt_8_reg <= cnt_8_next;
      shift_out_reg <= shift_out_next;
      
      // Generate rx_done_tick only in the STOP state
      rx_done_tick <= (state_reg == STOP) && (s_tick && cnt_15_reg == SB_TICK_MAX);
    end
  end

  // FSMD next-state logic
  always @* begin
    // Default assignments
    state_next = state_reg;
    cnt_15_next = cnt_15_reg;
    cnt_8_next = cnt_8_reg;
    shift_out_next = shift_out_reg;

    case (state_reg)
      IDLE: begin
        if (~data_in) begin
          state_next = START;
          cnt_15_next = 4'b0000;
        end
      end

      START: begin
        if (s_tick) begin
          if (cnt_15_reg == 4'b0111) begin  // 7 as a 4-bit value
            state_next = DATA;
            cnt_15_next = 4'b0000;
            cnt_8_next = 3'b000;
          end else begin
            cnt_15_next = cnt_15_reg + 4'b0001;
          end
        end
      end

      DATA: begin
        if (s_tick) begin
          if (cnt_15_reg == 4'b1111) begin  // 15 as a 4-bit value
            cnt_15_next = 4'b0000;
            shift_out_next = {data_in, shift_out_reg[7:1]};
            if (cnt_8_reg == DBIT_MAX)  // Use precomputed DBIT_MAX
              state_next = STOP;
            else
              cnt_8_next = cnt_8_reg + 3'b001;
          end else begin
            cnt_15_next = cnt_15_reg + 4'b0001;
          end
        end
      end

      STOP: begin
        if (s_tick) begin
          if (cnt_15_reg == SB_TICK_MAX) begin  // Use precomputed SB_TICK_MAX
            state_next = IDLE;
          end else begin
            cnt_15_next = cnt_15_reg + 4'b0001;
          end
        end
      end
    endcase
  end

  // Output data
  assign data_out = shift_out_reg;

endmodule
