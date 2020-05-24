`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module trunc_double(
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
        reg signed [31 : 0] i0, j0;
        reg unsigned [31 : 0] i1, sx;
        {i0, i1} = a;
        sx = i0 & 32'h80000000;
        j0 = ((i0 >> 20) & 12'h7ff) - 12'h3ff;
        if (j0 < 20) begin
            if (j0 < 0) begin
                i0 = sx; i1 = 0;
            end else begin
                i0 = sx | (i0 & ~(32'h000fffff >> j0)); i1 = 0;
            end
        end else if (j0 > 51) begin
        end else begin
            i1 = i1 & ~(32'hffffffff >> (j0 - 20));
        end
        q_reg <= {i0, i1};
    end
end
endmodule
