`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module floor_double(
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
        reg signed [31 : 0] i0, i1, j0;
        reg unsigned [31 : 0] i, j;
        {i0, i1} = a;
        j0 = ((i0 >> 20) & 12'h7ff) - 12'h3ff;
        if (j0 < 20) begin
            if (j0 < 0) begin
                if (i0 >= 0) begin
                    i0 = 0; i1 = 0;
                end else if (((i0 & 32'h7fffffff) | i1) != 0) begin
                    i0 = 32'hbff00000; i1 = 0;
                end
            end else begin
                i = 32'h000fffff >> j0;
                if (((i0 & i) | i1) == 0) begin
                end else begin
                    if (i0 < 0) i0 += 32'h00100000 >> j0;
                    i0 &= ~i; i1 = 0;
                end
            end
        end else if (j0 > 51) begin
        end else begin
            i = 32'hffffffff >> (j0 - 20);
            if ((i1 & i) == 0) begin
            end else begin
                if (i0 < 0) begin
                    if (j0 == 20) begin
                        i0 += 1;
                    end else begin
                        j = i1 + (1 << (52 - j0));
                        if (j < i1) i0 += 1;
                        i1 = j;
                    end
                end
                i1 &= ~i;
            end
        end
        q_reg <= {i0, i1};
    end
end
endmodule
