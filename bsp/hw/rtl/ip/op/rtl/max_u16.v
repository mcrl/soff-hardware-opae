`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module max_u16(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [16 - 1 : 0] a,
    input wire [16 - 1 : 0] b,
    output wire [16 - 1 : 0] q
    );
reg [16 - 1 : 0] q_reg;
assign q = q_reg;
always @(posedge clk) begin
    if (enable) begin
        q_reg <= a > b ? a : b;
    end
end
endmodule