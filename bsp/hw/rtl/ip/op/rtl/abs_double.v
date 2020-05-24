`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module abs_double(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [64 - 1 : 0] a,
    output wire [64 - 1 : 0] q
    );
reg [64 - 1 : 0] q_reg;
assign q = q_reg;
always @(posedge clk) begin
    if (enable) begin
        q_reg <= {1'b0, a[64 - 2 : 0]};
    end
end
endmodule
