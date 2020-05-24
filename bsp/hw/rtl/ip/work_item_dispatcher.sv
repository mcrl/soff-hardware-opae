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

module work_item_dispatcher #(parameter GWIDTH = 32, parameter LWIDTH = 10) (
  input logic clk,
  input logic rstn,
  input logic [LWIDTH : 0] local_size_0,
  input logic [LWIDTH : 0] local_size_1,
  input logic [LWIDTH : 0] local_size_2,
  input logic [GWIDTH - 1 : 0] global_offset_0,
  input logic [GWIDTH - 1 : 0] global_offset_1,
  input logic [GWIDTH - 1 : 0] global_offset_2,
  input logic valid_in,
  output logic wait_in,
  output logic [GWIDTH - 1 : 0] global_id_0,
  output logic [GWIDTH - 1 : 0] global_id_1,
  output logic [GWIDTH - 1 : 0] global_id_2,
  output logic [LWIDTH - 1 : 0] local_id_0,
  output logic [LWIDTH - 1 : 0] local_id_1,
  output logic [LWIDTH - 1 : 0] local_id_2,
  output logic [LWIDTH - 1 : 0] flat_local_id,
  output logic valid_out,
  input logic wait_out
);

    logic [LWIDTH : 0] my_local_size_0;
    logic [LWIDTH : 0] my_local_size_1;
    logic [LWIDTH : 0] my_local_size_2;
    logic [GWIDTH - 1 : 0] my_global_offset_0;
    logic [GWIDTH - 1 : 0] my_global_offset_1;
    logic [GWIDTH - 1 : 0] my_global_offset_2;
    logic [GWIDTH - 1 : 0] my_global_id_0;
    logic [GWIDTH - 1 : 0] my_global_id_1;
    logic [GWIDTH - 1 : 0] my_global_id_2;
    logic [LWIDTH - 1 : 0] my_local_id_0;
    logic [LWIDTH - 1 : 0] my_local_id_1;
    logic [LWIDTH - 1 : 0] my_local_id_2;
    logic [LWIDTH - 1 : 0] my_flat_local_id;

    typedef enum { IDLE, BUSY } state_t;
    state_t state;

    always_comb begin
        logic last;
        if (state == IDLE) begin
            global_id_0 = global_offset_0;
            global_id_1 = global_offset_1;
            global_id_2 = global_offset_2;
            local_id_0 = 0;
            local_id_1 = 0;
            local_id_2 = 0;
            flat_local_id = 0;
        end else begin // state == BUSY
            global_id_0 = my_global_id_0;
            global_id_1 = my_global_id_1;
            global_id_2 = my_global_id_2;
            local_id_0 = my_local_id_0;
            local_id_1 = my_local_id_1;
            local_id_2 = my_local_id_2;
            flat_local_id = my_flat_local_id;
        end
        last = local_id_0 + 1 == local_size_0 & local_id_1 + 1 == local_size_1 & local_id_2 + 1 == local_size_2;
        wait_in = ~last | wait_out;
        valid_out = valid_in;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            state <= IDLE;
        end else begin
            if (state == IDLE) begin
                if (valid_in & wait_in) begin
                    state <= BUSY;
                    my_local_size_0 <= local_size_0;
                    my_local_size_1 <= local_size_1;
                    my_local_size_2 <= local_size_2;
                    my_global_offset_0 <= global_offset_0;
                    my_global_offset_1 <= global_offset_1;
                    my_global_offset_2 <= global_offset_2;
                    if (wait_out) begin
                        my_global_id_0 <= global_offset_0;
                        my_global_id_1 <= global_offset_1;
                        my_global_id_2 <= global_offset_2;
                        my_local_id_0 <= 0;
                        my_local_id_1 <= 0;
                        my_local_id_2 <= 0;
                        my_flat_local_id <= 0;
                    end else begin
                        if (1 == local_size_0) begin
                            if (1 == local_size_1) begin
                                if (1 == local_size_2) begin
                                    // wait_in & ~wait_out => ~last; never reach here
                                end else begin
                                    my_local_id_0 <= 0;
                                    my_local_id_1 <= 0;
                                    my_local_id_2 <= 1;
                                    my_flat_local_id <= 1;
                                    my_global_id_0 <= global_offset_0;
                                    my_global_id_1 <= global_offset_1;
                                    my_global_id_2 <= global_offset_2 + 1'b1;
                                end
                            end else begin
                                my_local_id_0 <= 0;
                                my_local_id_1 <= 1;
                                my_local_id_2 <= 0;
                                my_flat_local_id <= 1;
                                my_global_id_0 <= global_offset_0;
                                my_global_id_1 <= global_offset_1 + 1'b1;
                                my_global_id_2 <= global_offset_2;
                            end
                        end else begin
                            my_local_id_0 <= 1;
                            my_local_id_1 <= 0;
                            my_local_id_2 <= 0;
                            my_flat_local_id <= 1;
                            my_global_id_0 <= global_offset_0 + 1'b1;
                            my_global_id_1 <= global_offset_1;
                            my_global_id_2 <= global_offset_2;
                        end
                    end
                end
            end else begin // state == BUSY
                if (valid_in & ~wait_in) begin
                    state <= IDLE;
                end else begin
                    if (~wait_out) begin
                        if (my_local_id_0 + 1 == my_local_size_0) begin
                            if (my_local_id_1 + 1 == my_local_size_1) begin
                                if (my_local_id_2 + 1 == my_local_size_2) begin
                                    state <= IDLE;
                                end else begin
                                    my_local_id_0 <= 0;
                                    my_local_id_1 <= 0;
                                    my_local_id_2 <= my_local_id_2 + 1'b1;
                                    my_flat_local_id <= my_flat_local_id + 1'b1;
                                    my_global_id_0 <= my_global_offset_0;
                                    my_global_id_1 <= my_global_offset_1;
                                    my_global_id_2 <= my_global_id_2 + 1'b1;
                                end
                            end else begin
                                my_local_id_0 <= 0;
                                my_local_id_1 <= my_local_id_1 + 1'b1;
                                my_flat_local_id <= my_flat_local_id + 1'b1;
                                my_global_id_0 <= my_global_offset_0;
                                my_global_id_1 <= my_global_id_1 + 1'b1;
                            end
                        end else begin
                            my_local_id_0 <= my_local_id_0 + 1'b1;
                            my_flat_local_id <= my_flat_local_id + 1'b1;
                            my_global_id_0 <= my_global_id_0 + 1'b1;
                        end
                    end
                end
            end
        end
    end

endmodule
