`timescale 1ns / 1ps

module fifo
    #(parameter W = 4, B = 8)  // W: address width, B: data width
    (
        input clk,
        input rst_n,
        input wr,              // Write enable
        input rd,              // Read enable
        input [B-1:0] wr_data, // Data to write
        output reg [B-1:0] rd_data,  // Data to read
        output reg full,       // Full flag
        output reg empty       // Empty flag
    );

    // Internal memory array
    reg [B-1:0] array_reg [0:(2**W)-1];
    reg [W-1:0] rd_ptr = 0;   // Read pointer
    reg [W-1:0] wr_ptr = 0;   // Write pointer
    reg fifo_wrap = 0;        // Wrap flag for differentiating full/empty

    // Synchronous logic for writing, reading, and updating flags
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= 0;
            wr_ptr <= 0;
            full <= 0;
            empty <= 1;
            fifo_wrap <= 0;
            rd_data <= 0;
        end else begin
            // Write operation
            if (wr && !full) begin
                array_reg[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + 1;
                if (wr_ptr == (2**W - 1)) fifo_wrap <= 1; // Set wrap on write pointer wrap-around
                empty <= 0;  // Not empty if we just wrote data
            end

            // Read operation
            if (rd && !empty) begin
                rd_data <= array_reg[rd_ptr];
                rd_ptr <= rd_ptr + 1;
                if (rd_ptr == (2**W - 1)) fifo_wrap <= 0; // Clear wrap on read pointer wrap-around
                full <= 0;  // Not full if we just read data
            end

            // Update full and empty flags
            full <= (wr_ptr == rd_ptr) && fifo_wrap; // Full when pointers are equal with wrap
            empty <= (wr_ptr == rd_ptr) && !fifo_wrap; // Empty when pointers are equal without wrap
        end
    end
endmodule
