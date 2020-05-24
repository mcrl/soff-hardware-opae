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

module work_group_dispatcher #(
    parameter NUM_OUTPUT = 1,
    parameter GWIDTH = 32,
    parameter LWIDTH = 11,
    parameter NWIDTH = 64
) (
  input logic clk,
  input logic rstn,
  input logic trigger,
  input logic [GWIDTH - 1 : 0] num_groups_0,
  input logic [GWIDTH - 1 : 0] num_groups_1,
  input logic [GWIDTH - 1 : 0] num_groups_2,
  input logic [LWIDTH : 0] local_size_0,
  input logic [LWIDTH : 0] local_size_1,
  input logic [LWIDTH : 0] local_size_2,
  output logic [GWIDTH - 1 : 0] group_id_0 [0 : NUM_OUTPUT - 1],
  output logic [GWIDTH - 1 : 0] group_id_1 [0 : NUM_OUTPUT - 1],
  output logic [GWIDTH - 1 : 0] group_id_2 [0 : NUM_OUTPUT - 1],
  output logic [GWIDTH - 1 : 0] global_offset_0 [0 : NUM_OUTPUT - 1],
  output logic [GWIDTH - 1 : 0] global_offset_1 [0 : NUM_OUTPUT - 1],
  output logic [GWIDTH - 1 : 0] global_offset_2 [0 : NUM_OUTPUT - 1],
  output logic [NWIDTH - 1 : 0] flat_group_id [0 : NUM_OUTPUT - 1],
  output logic valid_out [0 : NUM_OUTPUT - 1],
  input logic wait_out [0 : NUM_OUTPUT - 1]
);

  logic [GWIDTH - 1 : 0] my_num_groups_0;
  logic [GWIDTH - 1 : 0] my_num_groups_1;
  logic [GWIDTH - 1 : 0] my_num_groups_2;
  logic [LWIDTH : 0] my_local_size_0;
  logic [LWIDTH : 0] my_local_size_1;
  logic [LWIDTH : 0] my_local_size_2;
  logic [GWIDTH - 1 : 0] my_group_id_0;
  logic [GWIDTH - 1 : 0] my_group_id_1;
  logic [GWIDTH - 1 : 0] my_group_id_2;
  logic [GWIDTH - 1 : 0] my_global_offset_0;
  logic [GWIDTH - 1 : 0] my_global_offset_1;
  logic [GWIDTH - 1 : 0] my_global_offset_2;
  logic [NWIDTH - 1 : 0] my_flat_group_id;

  typedef enum { IDLE, BUSY, WAIT } state_t;
  state_t state;
  localparam NUM_OUTPUT_WIDTH = $clog2(NUM_OUTPUT) > 1 ? $clog2(NUM_OUTPUT) : 1;
  logic [NUM_OUTPUT_WIDTH - 1 : 0] output_index;

  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      state <= IDLE;
      for (int i = 0; i < NUM_OUTPUT; ++i) begin
        valid_out[i] <= 0;
      end
    end else begin
      if (state == IDLE) begin
        if (trigger) begin
          my_num_groups_0 <= num_groups_0;
          my_num_groups_1 <= num_groups_1;
          my_num_groups_2 <= num_groups_2;
          my_local_size_0 <= local_size_0;
          my_local_size_1 <= local_size_1;
          my_local_size_2 <= local_size_2;
          my_group_id_0 <= 0;
          my_group_id_1 <= 0;
          my_group_id_2 <= 0;
          my_global_offset_0 <= 0;
          my_global_offset_1 <= 0;
          my_global_offset_2 <= 0;
          my_flat_group_id <= 0;
          state <= BUSY;
          output_index <= 0;
        end
      end // state == IDLE
      if (state == BUSY) begin
        for (int i = 0; i < NUM_OUTPUT; ++i) begin
          if (valid_out[i] & ~wait_out[i]) begin
            valid_out[i] <= 0;
          end
        end
        if (~valid_out[output_index] | ~wait_out[output_index]) begin
          valid_out[output_index] <= 1;
          group_id_0[output_index] <= my_group_id_0;
          group_id_1[output_index] <= my_group_id_1;
          group_id_2[output_index] <= my_group_id_2;
          global_offset_0[output_index] <= my_global_offset_0;
          global_offset_1[output_index] <= my_global_offset_1;
          global_offset_2[output_index] <= my_global_offset_2;
          flat_group_id[output_index] <= my_flat_group_id;
          if (my_group_id_0 + 1 == my_num_groups_0) begin
            if (my_group_id_1 + 1 == my_num_groups_1) begin
              if (my_group_id_2 + 1 == my_num_groups_2) begin
                state <= WAIT;
              end else begin
                my_group_id_0 <= 0;
                my_group_id_1 <= 0;
                my_group_id_2 <= my_group_id_2 + 1;
                my_global_offset_0 <= 0;
                my_global_offset_1 <= 0;
                my_global_offset_2 <= my_global_offset_2 + my_local_size_2;
              end
            end else begin
              my_group_id_0 <= 0;
              my_group_id_1 <= my_group_id_1 + 1;
              my_global_offset_0 <= 0;
              my_global_offset_1 <= my_global_offset_1 + my_local_size_1;
            end
          end else begin
            my_group_id_0 <= my_group_id_0 + 1;
            my_global_offset_0 <= my_global_offset_0 + my_local_size_0;
          end
          my_flat_group_id <= my_flat_group_id + 1;
          if (output_index == NUM_OUTPUT - 1) begin
            output_index <= 0;
          end else begin
            output_index <= output_index + 1;
          end
        end
      end // state = BUSY
      if (state == WAIT) begin
        for (int i = 0; i < NUM_OUTPUT; ++i) begin
          if (valid_out[i] & ~wait_out[i]) begin
            valid_out[i] <= 0;
          end
        end
        if (~|NUM_OUTPUT'(valid_out)) begin
          state <= IDLE;
        end
      end
    end
  end

endmodule
