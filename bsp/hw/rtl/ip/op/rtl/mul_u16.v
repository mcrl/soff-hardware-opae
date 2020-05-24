`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module mul_u16(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [16 - 1 : 0] a,
    input wire [16 - 1 : 0] b,
    output wire [16 - 1 : 0] q
    );
mul_u16_core mul_u16_core_inst(
    .clk(clk),
    .rst(~rstn),
    .en(enable),
    .a(a),
    .b(b),
    .result(q)
);
endmodule
