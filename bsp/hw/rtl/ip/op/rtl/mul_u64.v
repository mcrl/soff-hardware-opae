`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module mul_u64(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    input wire [64 - 1 : 0] b,
    output wire [64 - 1 : 0] q
    );
mul_u64_core mul_u64_core_inst(
    .clk(clk),
    .rst(~rstn),
    .en(enable),
    .a(a),
    .b(b),
    .result(q)
);
endmodule
