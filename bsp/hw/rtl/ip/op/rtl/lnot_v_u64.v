`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module lnot_v_u64(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    output wire [32 - 1 : 0] q
    );
reg [32 - 1 : 0] q_reg;
assign q = q_reg;
always @(posedge clk) begin
    if (enable) begin
        q_reg <= a ? 0 : -1;
    end
end
endmodule
