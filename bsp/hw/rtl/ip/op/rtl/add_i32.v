`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module add_i32(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [32 - 1 : 0] a,
    input wire [32 - 1 : 0] b,
    output wire [32 - 1 : 0] q
    );
add_i32_core add_i32_core_inst(
    .clk(clk),
    .rst(~rstn),
    .en(enable),
    .a0(a),
    .a1(b),
    .result(q)
);
endmodule
