`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module dtoi_i32(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    output wire [32 - 1 : 0] q
    );
localparam BIAS = 'h3ff;
localparam MANT = 52;
reg [32 - 1 : 0] q_reg;
assign q = q_reg;
always @(posedge clk) begin
    if (enable) begin
        reg signed [63 : 0] exponent;
        reg unsigned [63 : 0] ix;
        reg negative;
        reg [32 - 1 : 0] uret;
        negative = a[63];
        ix = a[62 : 0];
        exponent = (ix >> MANT) - BIAS;
        ix &= {MANT{1'b1}};
        ix[MANT] = 1;
        if (exponent >= MANT) begin
            uret = ix;
            uret <<= exponent - MANT;
        end else if (exponent >= -1) begin
            uret = ix >> (MANT - exponent);
        end else begin
            uret = 0;
        end
        uret = negative ? -uret : uret;
        q_reg <= uret;
    end
end
endmodule
