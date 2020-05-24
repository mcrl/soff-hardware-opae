/*****************************************************************************/
/*                                                                           */
/* Copyright (c) 2020 Seoul National University.                             */
/* All rights reserved.                                                      */
/*                                                                           */
/* Redistribution and use in source and binary forms, with or without        */
/* modification, are permitted provided that the following conditions        */
/* are met:                                                                  */
/*   1. Redistributions of source code must retain the above copyright       */
/*      notice, this list of conditions and the following disclaimer.        */
/*   2. Redistributions in binary form must reproduce the above copyright    */
/*      notice, this list of conditions and the following disclaimer in the  */
/*      documentation and/or other materials provided with the distribution. */
/*   3. Neither the name of Seoul National University nor the names of its   */
/*      contributors may be used to endorse or promote products derived      */
/*      from this software without specific prior written permission.        */
/*                                                                           */
/* THIS SOFTWARE IS PROVIDED BY SEOUL NATIONAL UNIVERSITY "AS IS" AND ANY    */
/* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE    */
/* DISCLAIMED. IN NO EVENT SHALL SEOUL NATIONAL UNIVERSITY BE LIABLE FOR ANY */
/* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL        */
/* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS   */
/* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)     */
/* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       */
/* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  */
/* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/* POSSIBILITY OF SUCH DAMAGE.                                               */
/*                                                                           */
/* Contact information:                                                      */
/*   Center for Manycore Programming                                         */
/*   Department of Computer Science and Engineering                          */
/*   Seoul National University, Seoul 08826, Korea                           */
/*   http://aces.snu.ac.kr                                                   */
/*                                                                           */
/* Contributors:                                                             */
/*   Gangwon Jo, Heehoon Kim, Jeesoo Lee, and Jaejin Lee                     */
/*                                                                           */
/*****************************************************************************/

module hs_cutter #(
    parameter WIDTH
) (
    input logic clk,
    input logic rstn,
    input logic [WIDTH - 1 : 0] in,
    input logic valid_in,
    output logic wait_in,
    output logic [WIDTH - 1 : 0] out,
    output logic valid_out,
    input logic wait_out
);

    // hs unit which "cuts" combinational paths

    logic [WIDTH - 1 : 0] s0_data;
    logic s0_valid;
    logic [WIDTH - 1 : 0] buf_data;
    logic buf_valid;

    always_comb begin
        out = buf_valid ? buf_data : s0_data;
        valid_out = buf_valid ? 1 : s0_valid;
        wait_in = buf_valid & s0_valid;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            s0_valid <= 0;
            buf_valid <= 0;
        end else begin
            if (~wait_in) begin
                s0_valid <= valid_in;
                s0_data <= in;
            end
            if (wait_out & ~buf_valid) begin
                buf_valid <= s0_valid;
                buf_data <= s0_data;
            end else if (~wait_out) begin
                buf_valid <= 0;
            end
        end
    end

endmodule

module hs_blocker #(
    parameter WIDTH
) (
    input logic clk,
    input logic rstn,
    input logic [WIDTH - 1 : 0] in,
    input logic valid_in,
    output logic wait_in,
    output logic [WIDTH - 1 : 0] out,
    output logic valid_out,
    input logic wait_out,
    input logic cond
);

    // hs unit which "blocks" signals if cond is true

    always_comb begin
        out = in;
        valid_out = valid_in & ~cond;
        wait_in = wait_out | cond;
    end

endmodule

module hs_bypass #(
    parameter WIDTH
) (
    input logic clk,
    input logic rstn,
    input logic [WIDTH - 1 : 0] in,
    input logic valid_in,
    output logic wait_in,
    output logic [WIDTH - 1 : 0] out,
    output logic valid_out,
    input logic wait_out
);

    // just dummy module

    always_comb begin
        out = in;
        valid_out = valid_in;
        wait_in = wait_out;
    end

endmodule

module hs_queue #(
  parameter WIDTH,
  parameter DEPTH,
  parameter SPEED = "ON"
) (
  input logic clk,
  input logic rstn,
  input logic [WIDTH - 1 : 0] in,
  input logic valid_in,
  output logic wait_in,
  output logic [WIDTH - 1 : 0] out,
  output logic valid_out,
  input logic wait_out
);

  if (WIDTH == 0) begin
    localparam COUNT_WIDTH = $clog2(DEPTH + 1);
    logic [COUNT_WIDTH - 1 : 0] count;
    always_comb begin
      wait_in = count == DEPTH;
      valid_out = count != 0;
    end
    always_ff @(posedge clk, negedge rstn) begin
      if (~rstn) begin
        count <= 0;
      end else begin
        logic enq, deq;
        enq = valid_in & ~wait_in;
        deq = valid_out & ~wait_out;
        if (enq & ~deq) begin
          count <= count + 1'b1;
        end
        if (~enq & deq) begin
          count <= count - 1'b1;
        end
      end
    end
  end else begin
    logic empty;
    logic full;
    always_comb begin
      wait_in = full;
      valid_out = ~empty;
    end
    scfifo #(
      .lpm_width(WIDTH),
      .lpm_widthu($clog2(DEPTH)),
      .lpm_numwords(2 ** $clog2(DEPTH)),
      .lpm_showahead("ON"),
      .overflow_checking("OFF"),
      .underflow_checking("OFF"),
      .add_ram_output_register(SPEED)
    ) scfifo_inst (
      .clock(clk),
      .aclr(~rstn),
      .data(in),
      .wrreq(valid_in & ~wait_in),
      .rdreq(valid_out & ~wait_out),
      .q(out),
      .empty(empty),
      .full(full)
    );
  end

endmodule

module hs_cqc #(
    parameter WIDTH,
    parameter DEPTH
) (
    input logic clk,
    input logic rstn,
    input logic [WIDTH - 1 : 0] in,
    input logic valid_in,
    output logic wait_in,
    output logic [WIDTH - 1 : 0] out,
    output logic valid_out,
    input logic wait_out
);
    // cutter - queue - cutter
    logic [WIDTH - 1 : 0] s0_data, s1_data;
    logic s0_valid, s0_wait, s1_valid, s1_wait;
    hs_cutter #(
        .WIDTH(WIDTH)
    ) cutter0 (
        .clk(clk),
        .rstn(rstn),
        .in(in),
        .valid_in(valid_in),
        .wait_in(wait_in),
        .out(s0_data),
        .valid_out(s0_valid),
        .wait_out(s0_wait)
    );
    hs_queue #(
        .WIDTH(WIDTH),
        .DEPTH(DEPTH)
    ) queue (
        .clk(clk),
        .rstn(rstn),
        .in(s0_data),
        .valid_in(s0_valid),
        .wait_in(s0_wait),
        .out(s1_data),
        .valid_out(s1_valid),
        .wait_out(s1_wait)
    );
    hs_cutter #(
        .WIDTH(WIDTH)
    ) cutter1 (
        .clk(clk),
        .rstn(rstn),
        .in(s1_data),
        .valid_in(s1_valid),
        .wait_in(s1_wait),
        .out(out),
        .valid_out(valid_out),
        .wait_out(wait_out)
    );
endmodule

module hs_cb #(
    parameter WIDTH
) (
    input logic clk,
    input logic rstn,
    input logic [WIDTH - 1 : 0] in,
    input logic valid_in,
    output logic wait_in,
    output logic [WIDTH - 1 : 0] out,
    output logic valid_out,
    input logic wait_out,
    input logic cond
);
    // cutter - blocker
    logic [WIDTH - 1 : 0] s0_data;
    logic s0_valid, s0_wait;
    hs_cutter #(
        .WIDTH(WIDTH)
    ) cutter (
        .clk(clk),
        .rstn(rstn),
        .in(in),
        .valid_in(valid_in),
        .wait_in(wait_in),
        .out(s0_data),
        .valid_out(s0_valid),
        .wait_out(s0_wait)
    );
    hs_blocker #(
        .WIDTH(WIDTH)
    ) blocker (
        .clk(clk),
        .rstn(rstn),
        .in(s0_data),
        .valid_in(s0_valid),
        .wait_in(s0_wait),
        .out(out),
        .valid_out(valid_out),
        .wait_out(wait_out),
        .cond(cond)
    );
endmodule

module hs_bc #(
    parameter WIDTH
) (
    input logic clk,
    input logic rstn,
    input logic [WIDTH - 1 : 0] in,
    input logic valid_in,
    output logic wait_in,
    output logic [WIDTH - 1 : 0] out,
    output logic valid_out,
    input logic wait_out,
    input logic cond
);
    // blocker - cutter
    logic [WIDTH - 1 : 0] s0_data;
    logic s0_valid, s0_wait;
    hs_blocker #(
        .WIDTH(WIDTH)
    ) blocker (
        .clk(clk),
        .rstn(rstn),
        .in(in),
        .valid_in(valid_in),
        .wait_in(wait_in),
        .out(s0_data),
        .valid_out(s0_valid),
        .wait_out(s0_wait),
        .cond(cond)
    );
    hs_cutter #(
        .WIDTH(WIDTH)
    ) cutter (
        .clk(clk),
        .rstn(rstn),
        .in(s0_data),
        .valid_in(s0_valid),
        .wait_in(s0_wait),
        .out(out),
        .valid_out(valid_out),
        .wait_out(wait_out)
    );
endmodule
