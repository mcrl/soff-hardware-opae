`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module rem_i64(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    input wire [64 - 1 : 0] b,
    output wire [64 - 1 : 0] q
    );
wire [64 - 1 : 0] quotient, remainder;
assign q = remainder;
div_i64_core div_i64_core_inst(
    .clock(clk),
    .clken(enable),
    .numer(a),
    .denom(b),
    .quotient(quotient),
    .remain(remainder)
);
endmodule
