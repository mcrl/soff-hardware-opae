`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module div_u64(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    input wire [64 - 1 : 0] b,
    output wire [64 - 1 : 0] q
    );
wire [64 - 1 : 0] quotient, remainder;
assign q = quotient;
div_u64_core div_u64_core_inst(
    .clock(clk),
    .clken(enable),
    .numer(a),
    .denom(b),
    .quotient(quotient),
    .remain(remainder)
);
endmodule
