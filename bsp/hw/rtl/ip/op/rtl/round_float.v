`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module round_float(
    input wire clk,
    input wire rstn,
    input wire enable,
    input wire [32 - 1 : 0] a,
    output wire [32 - 1 : 0] q
    );
reg [32 - 1 : 0] q_reg;
assign q = q_reg;
always @(posedge clk) begin
    if (enable) begin
        reg signed [31 : 0] i0, j0;
        reg unsigned [31 : 0] i;
        i0 = a;
        j0 = ((i0 >> 23) & 8'hff) - 8'h7f;
        if (j0 < 23) begin
            if (j0 < 0) begin
                i0 &= 32'h80000000;
                if (j0 == -1) i0 |= 32'h3f800000;
            end else begin
                i = 32'h007fffff >> j0;
                if ((i0 & i) == 0) begin
                end else begin
                    i0 += 32'h00400000 >> j0;
                    i0 &= ~i;
                end
            end
        end
        q_reg <= i0;
    end
end
endmodule
