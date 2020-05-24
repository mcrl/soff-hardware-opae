`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module cmp_le_v_float(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [32 - 1 : 0] a,
    input wire [32 - 1 : 0] b,
    output wire [32 - 1 : 0] q
    );
wire qq;
assign q = qq ? -1 : 0;
cmp_le_float_core cmp_le_float_core_inst(
    .clk(clk),
    .areset(~rstn),
    .en(enable),
    .a(a),
    .b(b),
    .q(qq)
);
endmodule
