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

module lock #(
    parameter NUM_MASTERS,
    parameter LOCK_WIDTH
) (
    input logic clk,
    input logic rstn,
    input logic [LOCK_WIDTH - 1 : 0] bys_lockid[NUM_MASTERS],
    input logic bys_lockrequestvalid[NUM_MASTERS],
    output logic bys_lockwaitrequest[NUM_MASTERS],
    output logic bys_lockresponsevalid[NUM_MASTERS],
    input logic bys_lockwaitresponse[NUM_MASTERS],
    input logic [LOCK_WIDTH - 1 : 0] bys_unlockid[NUM_MASTERS],
    input logic bys_unlockrequestvalid[NUM_MASTERS],
    output logic bys_unlockwaitrequest[NUM_MASTERS],
    output logic bys_unlockresponsevalid[NUM_MASTERS],
    input logic bys_unlockwaitresponse[NUM_MASTERS]
);

    // lock input => lock_s0 ===(acquire lock)===> lock_s1 ===(lock_buf)===> lock output
    // lock_buf is used to cut long combinational wait_out chain

    localparam SELECTED_WIDTH = $clog2(NUM_MASTERS + 1);
    localparam NUM_LOCK = 1 << LOCK_WIDTH;

    logic array_valid[NUM_LOCK];

    logic [LOCK_WIDTH - 1 : 0] lock_s0_in[NUM_MASTERS];
    logic lock_s0_valid[NUM_MASTERS];
    logic lock_s1_valid[NUM_MASTERS];
    logic lock_buf_valid[NUM_MASTERS];
    logic [SELECTED_WIDTH - 1 : 0] lock_selected[NUM_LOCK];

    logic [LOCK_WIDTH - 1 : 0] unlock_s0_in[NUM_MASTERS];
    logic unlock_s0_valid[NUM_MASTERS];
    logic unlock_s1_valid[NUM_MASTERS];
    logic unlock_buf_valid[NUM_MASTERS];
    logic [SELECTED_WIDTH - 1 : 0] unlock_selected[NUM_LOCK];

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                lock_s0_valid[i] <= 0;
                lock_s1_valid[i] <= 0;
                lock_buf_valid[i] <= 0;
                unlock_s0_valid[i] <= 0;
                unlock_s1_valid[i] <= 0;
                unlock_buf_valid[i] <= 0;
            end
            for (int i = 0; i < NUM_LOCK; ++i) begin
                array_valid[i] <= 0;
            end
        end else begin
            // lock_buf => lock output
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (~bys_lockwaitresponse[i]) begin
                    lock_buf_valid[i] <= 0;
                end
            end
            // lock_s1 => lock_buf
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (~lock_buf_valid[i] & bys_lockwaitresponse[i]) begin
                    lock_buf_valid[i] <= lock_s1_valid[i];
                    lock_s1_valid[i] <= 0;
                end else if (~lock_buf_valid[i]) begin
                    lock_s1_valid[i] <= 0;
                end
            end
            // lock_s0 => lock_s1
            for (int i = 0; i < NUM_LOCK; ++i) begin
                if (lock_selected[i] != NUM_MASTERS) begin
                    lock_s0_valid[lock_selected[i]] <= 0;
                    lock_s1_valid[lock_selected[i]] <= 1;
                    array_valid[i] <= 1;
                end
            end
            // lock input => lock_s0
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (bys_lockrequestvalid[i] & ~bys_lockwaitrequest[i]) begin
                    lock_s0_valid[i] <= 1;
                    lock_s0_in[i] <= bys_lockid[i];
                end
            end
            // unlock_buf => unlock output
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (~bys_unlockwaitresponse[i]) begin
                    unlock_buf_valid[i] <= 0;
                end
            end
            // unlock_s1 => unlock_buf
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (~unlock_buf_valid[i] & bys_unlockwaitresponse[i]) begin
                    unlock_buf_valid[i] <= unlock_s1_valid[i];
                    unlock_s1_valid[i] <= 0;
                end else if (~unlock_buf_valid[i]) begin
                    unlock_s1_valid[i] <= 0;
                end
            end
            // unlock_s0 => unlock_s1
            for (int i = 0; i < NUM_LOCK; ++i) begin
                if (unlock_selected[i] != NUM_MASTERS) begin
                    unlock_s0_valid[unlock_selected[i]] <= 0;
                    unlock_s1_valid[unlock_selected[i]] <= 1;
                    array_valid[i] <= 0;
                end
            end
            // unlock input => unlock_s0
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (bys_unlockrequestvalid[i] & ~bys_unlockwaitrequest[i]) begin
                    unlock_s0_valid[i] <= 1;
                    unlock_s0_in[i] <= bys_unlockid[i];
                end
            end
        end
    end

    always_comb begin
        // each lock choose master to be consumed
        for (int i = 0; i < NUM_LOCK; ++i) begin
            lock_selected[i] = NUM_MASTERS;
            if (~array_valid[i]) begin
                for (int j = 0; j < NUM_MASTERS; ++j) begin
                    if (lock_s0_valid[j] & lock_s0_in[j] == i & ~(lock_s1_valid[j] & lock_buf_valid[j])) begin
                        lock_selected[i] = j;
                    end
                end
            end
        end
        for (int i = 0; i < NUM_LOCK; ++i) begin
            unlock_selected[i] = NUM_MASTERS;
            if (array_valid[i]) begin
                for (int j = 0; j < NUM_MASTERS; ++j) begin
                    if (unlock_s0_valid[j] & unlock_s0_in[j] == i & ~(unlock_s1_valid[j] & unlock_buf_valid[j])) begin
                        unlock_selected[i] = j;
                    end
                end
            end
        end

        // handle wait_in, valid_out signal
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            bys_lockwaitrequest[i] = lock_s0_valid[i];
            for (int j = 0; j < NUM_LOCK; ++j) begin
                if (lock_selected[j] == i) begin // if master is selected
                    bys_lockwaitrequest[i] = 0; // no need to wait
                end
            end
            bys_lockresponsevalid[i] = lock_s1_valid[i] | lock_buf_valid[i];
        end
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            bys_unlockwaitrequest[i] = unlock_s0_valid[i];
            for (int j = 0; j < NUM_LOCK; ++j) begin
                if (unlock_selected[j] == i) begin // if master is selected
                    bys_unlockwaitrequest[i] = 0; // no need to wait
                end
            end
            bys_unlockresponsevalid[i] = unlock_s1_valid[i] | unlock_buf_valid[i];
        end
    end

endmodule
