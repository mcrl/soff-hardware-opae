`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module cmp_ge_double(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    input wire [64 - 1 : 0] b,
    output wire [32 - 1 : 0] q
    );
wire qq;
assign q = qq ? 1 : 0;
cmp_ge_double_core cmp_ge_double_core_inst(
    .clk(clk),
    .areset(~rstn),
    .en(enable),
    .a(a),
    .b(b),
    .q(qq)
);
endmodule
