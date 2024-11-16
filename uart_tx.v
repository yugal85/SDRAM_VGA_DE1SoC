`timescale 1ns / 1ps

module uart_tx
  #(
    parameter DBIT = 8,    // Number of data bits
              SB_TICK = 16 // Number of ticks for the stop bit
  )
  ( 
    input wire clk,
    input wire reset,
    input wire [7:0] data_in,
    input wire s_tick,
    input wire tx_start,
    output reg tx_done_tick,
    output wire data_out
  );

  // State declaration for FSMD
  localparam [1:0] idle = 2'b00, start = 2'b01, data = 2'b10, stop = 2'b11;

  // Signal declarations
  reg [1:0] state_reg, state_next;
  reg [3:0] cnt_15_reg, cnt_15_next;      // 4-bit counter for ticks
  reg [2:0] cnt_8_reg, cnt_8_next;        // 3-bit counter for bits
  reg [7:0] shift_out_reg, shift_out_next;
  reg tx_reg, tx_next;

  // Constants for bit-length matching
  localparam [3:0] TICK_COUNT = 4'b1111;       // 15 as a 4-bit value
  //parameter [3:0] SB_TICK = 4'd16; // Explicitly set SB_TICK as a 4-bit parameter

// Constants for bit-length matching
  localparam [3:0] STOP_TICK_COUNT = SB_TICK - 1;


  // Register and state update
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      state_reg <= idle;
      cnt_15_reg <= 4'b0000;
      cnt_8_reg <= 3'b000;
      shift_out_reg <= 8'b00000000;
      tx_reg <= 1'b1;
    end else begin
      state_reg <= state_next;
      cnt_15_reg <= cnt_15_next;
      cnt_8_reg <= cnt_8_next;
      shift_out_reg <= shift_out_next;
      tx_reg <= tx_next;
    end
  end

  // FSMD next-state logic
  always @* begin
    // Default assignments
    state_next = state_reg;
    cnt_15_next = cnt_15_reg;
    cnt_8_next = cnt_8_reg;
    shift_out_next = shift_out_reg;
    tx_next = tx_reg;
    tx_done_tick = 1'b0;

    case (state_reg)
      idle: begin
        tx_next = 1'b1;
        if (tx_start) begin
          state_next = start;
          cnt_15_next = 4'b0000;
          shift_out_next = data_in;
        end
      end

      start: begin
        tx_next = 1'b0;
        if (s_tick) begin
          if (cnt_15_reg == 4'b0111) begin  // 7 as a 4-bit value
            state_next = data;
            cnt_15_next = 4'b0000;
            cnt_8_next = 3'b000;
          end else begin
            cnt_15_next = cnt_15_reg + 4'b0001;
          end
        end
      end

      data: begin
        tx_next = shift_out_reg[0];
        if (s_tick) begin
          if (cnt_15_reg == TICK_COUNT) begin  // Use precomputed 15
            cnt_15_next = 4'b0000;
            shift_out_next = shift_out_reg >> 1;
            if (cnt_8_reg == (DBIT - 1))
              state_next = stop;
            else
              cnt_8_next = cnt_8_reg + 3'b001;
          end else begin
            cnt_15_next = cnt_15_reg + 4'b0001;
          end
        end
      end

      stop: begin
        tx_next = 1'b1;
        if (s_tick) begin
          if (cnt_15_reg == STOP_TICK_COUNT) begin  // Use precomputed SB_TICK - 1
            state_next = idle;
            tx_done_tick = 1'b1;
          end else begin
            cnt_15_next = cnt_15_reg + 4'b0001;
          end
        end
      end
    endcase
  end

  // Output
  assign data_out = tx_reg;

endmodule
