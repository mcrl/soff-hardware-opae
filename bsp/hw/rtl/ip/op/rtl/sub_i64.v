`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module sub_i64(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    input wire [64 - 1 : 0] b,
    output wire [64 - 1 : 0] q
    );
add_i64_core add_i64_core_inst(
    .clk(clk),
    .rst(~rstn),
    .en(enable),
    .a0(a),
    .a1(-b),
    .result(q)
);
endmodule
