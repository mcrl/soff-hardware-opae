`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module rem_u32(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [32 - 1 : 0] a,
    input wire [32 - 1 : 0] b,
    output wire [32 - 1 : 0] q
    );
wire [32 - 1 : 0] quotient, remainder;
assign q = remainder;
div_u32_core div_u32_core_inst(
    .clock(clk),
    .clken(enable),
    .numer(a),
    .denom(b),
    .quotient(quotient),
    .remain(remainder)
);
endmodule
