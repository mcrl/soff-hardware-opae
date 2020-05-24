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

module mem_arbiter_41 #(
    parameter ADDR_WIDTH = 48,
    parameter DATA_WIDTH = 512,
    parameter BURST_WIDTH = 4
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs0_address,
    input logic avs0_read,
    output logic [DATA_WIDTH - 1 : 0] avs0_readdata,
    output logic avs0_readdatavalid,
    input logic avs0_write,
    input logic [DATA_WIDTH - 1 : 0] avs0_writedata,
    input logic [DATA_WIDTH / 8 - 1 : 0] avs0_byteenable,
    input logic [BURST_WIDTH - 1 : 0] avs0_burstcount,
    output logic avs0_waitrequest,
    input logic [ADDR_WIDTH - 1 : 0] avs1_address,
    input logic avs1_read,
    output logic [DATA_WIDTH - 1 : 0] avs1_readdata,
    output logic avs1_readdatavalid,
    input logic avs1_write,
    input logic [DATA_WIDTH - 1 : 0] avs1_writedata,
    input logic [DATA_WIDTH / 8 - 1 : 0] avs1_byteenable,
    input logic [BURST_WIDTH - 1 : 0] avs1_burstcount,
    output logic avs1_waitrequest,
    input logic [ADDR_WIDTH - 1 : 0] avs2_address,
    input logic avs2_read,
    output logic [DATA_WIDTH - 1 : 0] avs2_readdata,
    output logic avs2_readdatavalid,
    input logic avs2_write,
    input logic [DATA_WIDTH - 1 : 0] avs2_writedata,
    input logic [DATA_WIDTH / 8 - 1 : 0] avs2_byteenable,
    input logic [BURST_WIDTH - 1 : 0] avs2_burstcount,
    output logic avs2_waitrequest,
    input logic [ADDR_WIDTH - 1 : 0] avs3_address,
    input logic avs3_read,
    output logic [DATA_WIDTH - 1 : 0] avs3_readdata,
    output logic avs3_readdatavalid,
    input logic avs3_write,
    input logic [DATA_WIDTH - 1 : 0] avs3_writedata,
    input logic [DATA_WIDTH / 8 - 1 : 0] avs3_byteenable,
    input logic [BURST_WIDTH - 1 : 0] avs3_burstcount,
    output logic avs3_waitrequest,
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid,
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount,
    input logic avm_waitrequest
);

    genvar i;

    localparam NUM_MASTERS = 4;
    localparam MASTER_WIDTH = $clog2(NUM_MASTERS);
    localparam MAX_READS_PER_SLAVE = 512;

    // array version of avs
    logic [ADDR_WIDTH - 1 : 0] avs_address[NUM_MASTERS];
    logic avs_read[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_readdata[NUM_MASTERS];
    logic avs_readdatavalid[NUM_MASTERS];
    logic avs_write[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_writedata[NUM_MASTERS];
    logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable[NUM_MASTERS];
    logic [BURST_WIDTH - 1 : 0] avs_burstcount[NUM_MASTERS];
    logic avs_waitrequest[NUM_MASTERS];

    always_comb begin
        avs_address[0] = avs0_address;
        avs_read[0] = avs0_read;
        avs0_readdata = avs_readdata[0];
        avs0_readdatavalid = avs_readdatavalid[0];
        avs_write[0] = avs0_write;
        avs_writedata[0] = avs0_writedata;
        avs_byteenable[0] = avs0_byteenable;
        avs_burstcount[0] = avs0_burstcount;
        avs0_waitrequest = avs_waitrequest[0];
        avs_address[1] = avs1_address;
        avs_read[1] = avs1_read;
        avs1_readdata = avs_readdata[1];
        avs1_readdatavalid = avs_readdatavalid[1];
        avs_write[1] = avs1_write;
        avs_writedata[1] = avs1_writedata;
        avs_byteenable[1] = avs1_byteenable;
        avs_burstcount[1] = avs1_burstcount;
        avs1_waitrequest = avs_waitrequest[1];
        avs_address[2] = avs2_address;
        avs_read[2] = avs2_read;
        avs2_readdata = avs_readdata[2];
        avs2_readdatavalid = avs_readdatavalid[2];
        avs_write[2] = avs2_write;
        avs_writedata[2] = avs2_writedata;
        avs_byteenable[2] = avs2_byteenable;
        avs_burstcount[2] = avs2_burstcount;
        avs2_waitrequest = avs_waitrequest[2];
        avs_address[3] = avs3_address;
        avs_read[3] = avs3_read;
        avs3_readdata = avs_readdata[3];
        avs3_readdatavalid = avs_readdatavalid[3];
        avs_write[3] = avs3_write;
        avs_writedata[3] = avs3_writedata;
        avs_byteenable[3] = avs3_byteenable;
        avs_burstcount[3] = avs3_burstcount;
        avs3_waitrequest = avs_waitrequest[3];
    end

    // avs_req
    // rw | address | writedata | byteenable | burstcount
    localparam AVS_REQ_WIDTH = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8 + BURST_WIDTH;
    logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in[NUM_MASTERS], avs_req_out[NUM_MASTERS];
    logic avs_req_valid_in[NUM_MASTERS], avs_req_valid_out[NUM_MASTERS];
    logic avs_req_wait_in[NUM_MASTERS], avs_req_wait_out[NUM_MASTERS];
    logic avs_req_read[NUM_MASTERS], avs_req_write[NUM_MASTERS];
    logic [ADDR_WIDTH - 1 : 0] avs_req_address[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_req_writedata[NUM_MASTERS];
    logic [DATA_WIDTH / 8 - 1 : 0] avs_req_byteenable[NUM_MASTERS];
    logic [BURST_WIDTH - 1 : 0] avs_req_burstcount[NUM_MASTERS];
    logic avs_req_waitrequest[NUM_MASTERS];
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_in[i] = {avs_write[i], avs_address[i], avs_writedata[i], avs_byteenable[i], avs_burstcount[i]};
            avs_req_valid_in[i] = avs_read[i] | avs_write[i];
            avs_waitrequest[i] = avs_req_wait_in[i];
        end
    end
    for (i = 0; i < NUM_MASTERS; ++i) begin: avs_req_gates
        hs_cutter #(
            .WIDTH(AVS_REQ_WIDTH)
        ) avs_req_gate (
            .clk(clk),
            .rstn(rstn),
            .in(avs_req_in[i]),
            .valid_in(avs_req_valid_in[i]),
            .wait_in(avs_req_wait_in[i]),
            .out(avs_req_out[i]),
            .valid_out(avs_req_valid_out[i]),
            .wait_out(avs_req_wait_out[i])
        );
    end
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            logic rw;
            {rw, avs_req_address[i], avs_req_writedata[i], avs_req_byteenable[i], avs_req_burstcount[i]} = avs_req_out[i];
            avs_req_read[i] = avs_req_valid_out[i] & ~rw;
            avs_req_write[i] = avs_req_valid_out[i] & rw;
            avs_req_wait_out[i] = avs_req_waitrequest[i];
        end
    end

    // avm_req
    // rw | address | writedata | byteenable | burstcount
    localparam AVM_REQ_WIDTH = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8 + BURST_WIDTH;
    logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
    logic avm_req_valid_in, avm_req_valid_out;
    logic avm_req_wait_in, avm_req_wait_out;
    logic avm_req_read, avm_req_write;
    logic [ADDR_WIDTH - 1 : 0] avm_req_address;
    logic [DATA_WIDTH - 1 : 0] avm_req_writedata;
    logic [DATA_WIDTH / 8 - 1 : 0] avm_req_byteenable;
    logic [BURST_WIDTH - 1 : 0] avm_req_burstcount;
    logic avm_req_waitrequest;
    always_comb begin
        avm_req_in = {avm_req_write, avm_req_address, avm_req_writedata, avm_req_byteenable, avm_req_burstcount};
        avm_req_valid_in = avm_req_read | avm_req_write;
        avm_req_waitrequest = avm_req_wait_in;
    end
    hs_cutter #(
        .WIDTH(AVM_REQ_WIDTH)
    ) avm_req_gate (
        .clk(clk),
        .rstn(rstn),
        .in(avm_req_in),
        .valid_in(avm_req_valid_in),
        .wait_in(avm_req_wait_in),
        .out(avm_req_out),
        .valid_out(avm_req_valid_out),
        .wait_out(avm_req_wait_out)
    );
    always_comb begin
        logic rw;
        {rw, avm_address, avm_writedata, avm_byteenable, avm_burstcount} = avm_req_out;
        avm_read = avm_req_valid_out & ~rw;
        avm_write = avm_req_valid_out & rw;
        avm_req_wait_out = avm_waitrequest;
    end

    // avm_res
    // readdata
    localparam AVM_RES_WIDTH = DATA_WIDTH;
    logic [AVM_RES_WIDTH - 1 : 0] avm_res_in, avm_res_out;
    logic avm_res_valid_in, avm_res_valid_out;
    logic avm_res_wait_in, avm_res_wait_out;
    logic [DATA_WIDTH - 1 : 0] avm_res_readdata;
    logic avm_res_readdatavalid;
    always_comb begin
        avm_res_in = {avm_readdata};
        avm_res_valid_in = avm_readdatavalid;
        // avm_res_wait_in will never assert
    end
    hs_cutter #(
        .WIDTH(AVM_RES_WIDTH)
    ) avm_res_gate (
        .clk(clk),
        .rstn(rstn),
        .in(avm_res_in),
        .valid_in(avm_res_valid_in),
        .wait_in(avm_res_wait_in),
        .out(avm_res_out),
        .valid_out(avm_res_valid_out),
        .wait_out(avm_res_wait_out)
    );
    always_comb begin
        {avm_res_readdata} = avm_res_out;
        avm_res_readdatavalid = avm_res_valid_out;
        avm_res_wait_out = 0;
    end

    // avs_res
    // readdata
    localparam AVS_RES_WIDTH = DATA_WIDTH;
    logic [AVS_RES_WIDTH - 1 : 0] avs_res_in[NUM_MASTERS], avs_res_out[NUM_MASTERS];
    logic avs_res_valid_in[NUM_MASTERS], avs_res_valid_out[NUM_MASTERS];
    logic avs_res_wait_in[NUM_MASTERS], avs_res_wait_out[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_res_readdata[NUM_MASTERS];
    logic avs_res_readdatavalid[NUM_MASTERS];
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_in[i] = {avs_res_readdata[i]};
            avs_res_valid_in[i] = avs_res_readdatavalid[i];
            // avs_res_wait_in[i] will never assert
        end
    end
    for (i = 0; i < NUM_MASTERS; ++i) begin: avs_res_gates
        hs_cutter #(
            .WIDTH(AVS_RES_WIDTH)
        ) avs_res_gate (
            .clk(clk),
            .rstn(rstn),
            .in(avs_res_in[i]),
            .valid_in(avs_res_valid_in[i]),
            .wait_in(avs_res_wait_in[i]),
            .out(avs_res_out[i]),
            .valid_out(avs_res_valid_out[i]),
            .wait_out(avs_res_wait_out[i])
        );
    end
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            {avs_readdata[i]} = avs_res_out[i];
            avs_readdatavalid[i] = avs_res_valid_out[i];
            avs_res_wait_out[i] = 0;
        end
    end

    // avm_queue
    // master | burstcount
    localparam AVM_QUEUE_WIDTH = MASTER_WIDTH + BURST_WIDTH;
    logic [AVM_QUEUE_WIDTH - 1 : 0] avm_queue_in, avm_queue_out;
    logic avm_queue_valid_in, avm_queue_valid_out;
    logic avm_queue_wait_in, avm_queue_wait_out;
    hs_cqc #(
        .WIDTH(AVM_QUEUE_WIDTH),
        .DEPTH(MAX_READS_PER_SLAVE)
    ) avm_queue (
        .clk(clk),
        .rstn(rstn),
        .in(avm_queue_in),
        .valid_in(avm_queue_valid_in),
        .wait_in(avm_queue_wait_in),
        .out(avm_queue_out),
        .valid_out(avm_queue_valid_out),
        .wait_out(avm_queue_wait_out)
    );

    // request handler
    // multi-succ pattern
    // pred = avs_req
    // succ = avm_req, avm_queue
    logic req_master_valid;
    logic [MASTER_WIDTH - 1 : 0] req_master;
    logic [MASTER_WIDTH - 1 : 0] req_prev_master;

    logic req_avm_req_state;
    logic req_avm_queue_state;

    logic req_locked;
    logic [MASTER_WIDTH - 1 : 0] req_locked_master;
    logic [BURST_WIDTH - 1 : 0] req_locked_burstcount;
    logic [BURST_WIDTH - 1 : 0] req_locked_count;

    always_comb begin
        // choose master
        req_master_valid = 0;
        req_master = 'x;
        if (req_locked) begin
            // arbitration locked
            req_master_valid = 1;
            req_master = req_locked_master;
        end else begin
            // find a master with request
            logic [NUM_MASTERS - 1 : 0] x, y;
            x = 'x;
            for (int j = 0; j < NUM_MASTERS; ++j) begin
                x[j] = avs_req_read[j] | avs_req_write[j];
            end
            y = {x, x} >> req_prev_master;
            // why reverse? To check req_prev_master at last in order to replay prev request
            // if it is stalled due to waitrequest from slave
            for (int j = NUM_MASTERS - 1; j >= 0; --j) begin
                if (y[j]) begin
                    req_master_valid = 1;
                    req_master = j + req_prev_master < NUM_MASTERS ? j + req_prev_master : j + req_prev_master - NUM_MASTERS;
                end
            end
        end

        // init succ1
        avm_req_read = 0;
        avm_req_write = 0;
        avm_req_address = 'x;
        avm_req_writedata = 'x;
        avm_req_byteenable = 'x;
        avm_req_burstcount = 'x;
        // init succ2
        avm_queue_valid_in = 0;
        avm_queue_in = 'x;
        // init pred
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_waitrequest[i] = 1;
        end

        if (req_master_valid) begin
            // setup succ1
            avm_req_read = avs_req_read[req_master] & req_avm_req_state == 0;
            avm_req_write = avs_req_write[req_master] & req_avm_req_state == 0;
            avm_req_address = avs_req_address[req_master];
            avm_req_writedata = avs_req_writedata[req_master];
            avm_req_byteenable = avs_req_byteenable[req_master];
            avm_req_burstcount = avs_req_burstcount[req_master];

            // setup succ2
            if (avs_req_read[req_master]) begin
                avm_queue_valid_in = 1 & req_avm_queue_state == 0;
                avm_queue_in = {req_master, avs_req_burstcount[req_master]};
            end

            // setup pred
            avs_req_waitrequest[req_master] = ((avm_req_read | avm_req_write) & avm_req_waitrequest) // succ1 wait
                                            | (avm_queue_valid_in & avm_queue_wait_in); // succ2 wait
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_avm_req_state <= 0;
            req_avm_queue_state <= 0;
            req_locked <= 0;
            req_prev_master <= 0;
        end else begin
            // arbiter in idle
            if (~req_master_valid) begin
                req_avm_req_state <= 0;
                req_avm_queue_state <= 0;
            // pred consume
            end else if ((avs_req_read[req_master] | avs_req_write[req_master]) & ~avs_req_waitrequest[req_master]) begin
                req_avm_req_state <= 0;
                req_avm_queue_state <= 0;
            end else begin
                // succ1 consume
                if ((avm_req_read | avm_req_write) & ~avm_req_waitrequest) begin
                    req_avm_req_state <= req_avm_req_state + 1'b1;
                end
                // succ2 consume
                if (avm_queue_valid_in & ~avm_queue_wait_in) begin
                    req_avm_queue_state <= req_avm_queue_state + 1'b1;
                end
            end

            // arbitration lock mechanism
            if (~req_locked) begin
                if (req_master_valid) begin
                    // unlocked + new request => locked
                    req_locked <= 1;
                    req_locked_master <= req_master;
                    req_locked_burstcount <= avs_req_burstcount[req_master];
                    req_locked_count <= 1;
                    if (avs_req_read[req_master] & ~avs_req_waitrequest[req_master]) begin
                        // read immediately accepted => do not lock
                        req_locked <= 0;
                    end
                    if (avs_req_write[req_master] & ~avs_req_waitrequest[req_master]) begin
                        if (avs_req_burstcount[req_master] == 1) begin
                            // write immediately accepted & length 1 burst => do not lock
                            req_locked <= 0;
                        end else begin
                            req_locked_count <= 2;
                        end
                    end
                end
            end else begin
                // read accepted => unlock
                if (avs_req_read[req_master] & ~avs_req_waitrequest[req_master]) begin
                    req_locked <= 0;
                end
                // write accepted
                if (avs_req_write[req_master] & ~avs_req_waitrequest[req_master]) begin
                    if (req_locked_count == req_locked_burstcount) begin
                        // burst done => unlock
                        req_locked <= 0;
                    end else begin
                        // burst not done yet
                        req_locked_count <= req_locked_count + 1'b1;
                    end
                end
            end

            req_prev_master <= req_master_valid ? req_master : 0;
        end
    end

    // response handler
    // multi-pred pattern
    // pred = avm_queue, avm_res
    // succ = avs_res
    logic [MASTER_WIDTH - 1 : 0] res_master;
    logic [BURST_WIDTH - 1 : 0] res_burstcount;
    logic [BURST_WIDTH - 1 : 0] res_locked_count;

    always_comb begin
        // init
        avm_queue_wait_out = 1;
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_readdatavalid[i] = 0;
            avs_res_readdata[i] = 'x;
        end

        {res_master, res_burstcount} = avm_queue_out;
        if (avm_res_readdatavalid) begin
            avs_res_readdatavalid[res_master] = 1;
            avs_res_readdata[res_master] = avm_res_readdata;
            avm_queue_wait_out = res_locked_count != res_burstcount;
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_locked_count <= 1;
        end else begin
            if (avm_res_readdatavalid) begin
                if (avm_queue_wait_out) begin
                    res_locked_count <= res_locked_count + 1'b1;
                end else begin
                    res_locked_count <= 1;
                end
            end
        end
    end

endmodule
