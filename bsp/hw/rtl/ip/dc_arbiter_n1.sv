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

module dc_arbiter_n1 #(
    parameter NUM_MASTERS = 2,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ENABLE_PERFORMANCE_COUNTER = 0
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address[NUM_MASTERS],
    input logic avs_read[NUM_MASTERS],
    output logic [DATA_WIDTH - 1 : 0] avs_readdata[NUM_MASTERS],
    output logic avs_readdatavalid[NUM_MASTERS],
    input logic avs_write[NUM_MASTERS],
    input logic [DATA_WIDTH - 1 : 0] avs_writedata[NUM_MASTERS],
    input logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable[NUM_MASTERS],
    output logic avs_waitrequest[NUM_MASTERS],
    input logic avs_waitresponse[NUM_MASTERS],
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid, 
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest,
    output logic avm_waitresponse,
    output logic pc_request[NUM_MASTERS],
    output logic pc_request_cycle[NUM_MASTERS]
);

    genvar i;

    localparam MASTER_WIDTH = $clog2(NUM_MASTERS) > 0 ? $clog2(NUM_MASTERS) : 1;
    localparam MAX_READS_PER_MASTER = 512;
    localparam MAX_READS_PER_SLAVE = 512;

    logic [$clog2(MAX_READS_PER_MASTER + 1) - 1 : 0] read_count[NUM_MASTERS];

    // avs -> cutter -> blocker -> req handler
    // rw | address | writedata | byteenable
    localparam AVS_REQ_WIDTH = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8;
    logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in[NUM_MASTERS], avs_req_out[NUM_MASTERS];
    logic avs_req_valid_in[NUM_MASTERS], avs_req_wait_in[NUM_MASTERS];
    logic avs_req_valid_out[NUM_MASTERS], avs_req_wait_out[NUM_MASTERS];
    logic [ADDR_WIDTH - 1 : 0] avs_req_address[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_req_writedata[NUM_MASTERS];
    logic [DATA_WIDTH / 8 - 1 : 0] avs_req_byteenable[NUM_MASTERS];
    logic avs_req_read[NUM_MASTERS], avs_req_write[NUM_MASTERS], avs_req_waitrequest[NUM_MASTERS];
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_in[i] = {avs_write[i], avs_address[i], avs_writedata[i], avs_byteenable[i]};
            avs_req_valid_in[i] = avs_read[i] | avs_write[i];
            avs_waitrequest[i] = avs_req_wait_in[i];
        end
    end
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            logic rw;
            {rw, avs_req_address[i], avs_req_writedata[i], avs_req_byteenable[i]} = avs_req_out[i];
            avs_req_read[i] = avs_req_valid_out[i] & ~rw;
            avs_req_write[i] = avs_req_valid_out[i] & rw;
            avs_req_wait_out[i] = avs_req_waitrequest[i];
        end
    end
    for (i = 0; i < NUM_MASTERS; ++i) begin: avs_req
        hs_blocker #(
            .WIDTH(AVS_REQ_WIDTH)
        ) avs_req_cutter (
            .clk(clk),
            .rstn(rstn),
            .in(avs_req_in[i]),
            .valid_in(avs_req_valid_in[i]),
            .wait_in(avs_req_wait_in[i]),
            .out(avs_req_out[i]),
            .valid_out(avs_req_valid_out[i]),
            .wait_out(avs_req_wait_out[i]),
            .cond((read_count[i] & MAX_READS_PER_MASTER) != 0)
        );
    end

    // req handler -> cutter -> avm
    // rw | address | writedata | byteenable
    localparam AVM_REQ_WIDTH = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8;
    logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
    logic avm_req_valid_in, avm_req_wait_in;
    logic avm_req_valid_out, avm_req_wait_out;
    logic [ADDR_WIDTH - 1 : 0] avm_req_address;
    logic [DATA_WIDTH - 1 : 0] avm_req_writedata;
    logic [DATA_WIDTH / 8 - 1 : 0] avm_req_byteenable;
    logic avm_req_read, avm_req_write, avm_req_waitrequest;
    always_comb begin
        avm_req_in = {avm_req_write, avm_req_address, avm_req_writedata, avm_req_byteenable};
        avm_req_valid_in = avm_req_read | avm_req_write;
        avm_req_waitrequest = avm_req_wait_in;
    end
    always_comb begin
        logic rw;
        {rw, avm_address, avm_writedata, avm_byteenable} = avm_req_out;
        avm_read = avm_req_valid_out & ~rw;
        avm_write = avm_req_valid_out & rw;
        avm_req_wait_out = avm_waitrequest;
    end
    hs_cutter #(
        .WIDTH(AVM_REQ_WIDTH)
    ) avm_req_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(avm_req_in),
        .valid_in(avm_req_valid_in),
        .wait_in(avm_req_wait_in),
        .out(avm_req_out),
        .valid_out(avm_req_valid_out),
        .wait_out(avm_req_wait_out)
    );

    // avm -> cutter -> res handler
    // data
    localparam AVM_RES_WIDTH = DATA_WIDTH;
    logic [AVM_RES_WIDTH - 1 : 0] avm_res_in, avm_res_out;
    logic avm_res_valid_in, avm_res_wait_in;
    logic avm_res_valid_out, avm_res_wait_out;
    logic [DATA_WIDTH - 1 : 0] avm_res_readdata;
    logic avm_res_readdatavalid, avm_res_waitresponse;
    always_comb begin
        avm_res_in = {avm_readdata};
        avm_res_valid_in = avm_readdatavalid;
        avm_waitresponse = avm_res_wait_in;
    end
    always_comb begin
        avm_res_readdata = avm_res_out;
        avm_res_readdatavalid = avm_res_valid_out;
        avm_res_wait_out = avm_res_waitresponse;
    end
    hs_cutter #(
        .WIDTH(AVM_RES_WIDTH)
    ) avm_res_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(avm_res_in),
        .valid_in(avm_res_valid_in),
        .wait_in(avm_res_wait_in),
        .out(avm_res_out),
        .valid_out(avm_res_valid_out),
        .wait_out(avm_res_wait_out)
    );

    // res handler -> cutter -> avs
    // data
    localparam AVS_RES_WIDTH = DATA_WIDTH;
    logic [AVS_RES_WIDTH - 1 : 0] avs_res_in[NUM_MASTERS], avs_res_out[NUM_MASTERS];
    logic avs_res_valid_in[NUM_MASTERS], avs_res_wait_in[NUM_MASTERS];
    logic avs_res_valid_out[NUM_MASTERS], avs_res_wait_out[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_res_readdata[NUM_MASTERS];
    logic avs_res_readdatavalid[NUM_MASTERS], avs_res_waitresponse[NUM_MASTERS];
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_in[i] = avs_res_readdata[i];
            avs_res_valid_in[i] = avs_res_readdatavalid[i];
            avs_res_waitresponse[i] = avs_res_wait_in[i];
        end
    end
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_readdata[i] = avs_res_out[i];
            avs_readdatavalid[i] = avs_res_valid_out[i];
            avs_res_wait_out[i] = avs_waitresponse[i];
        end
    end
    for (i = 0; i < NUM_MASTERS; ++i) begin: avs_res
        hs_cutter #(
            .WIDTH(AVS_RES_WIDTH)
        ) avs_res_cutter (
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

    logic req_master_valid;
    logic [MASTER_WIDTH - 1 : 0] req_master;
    logic [MASTER_WIDTH - 1 : 0] req_prev_master;

    logic [MASTER_WIDTH - 1 : 0] read_slave_queue_in;
    logic read_slave_queue_valid_in;
    logic read_slave_queue_wait_in;
    logic [MASTER_WIDTH - 1 : 0] read_slave_queue_out;
    logic read_slave_queue_valid_out;
    logic read_slave_queue_wait_out;

    logic [DATA_WIDTH - 1 : 0] read_res_queue_in[NUM_MASTERS];
    logic read_res_queue_valid_in[NUM_MASTERS];
    logic read_res_queue_wait_in[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] read_res_queue_out[NUM_MASTERS];
    logic read_res_queue_valid_out[NUM_MASTERS];
    logic read_res_queue_wait_out[NUM_MASTERS];

    // request handling
    always_comb begin
        // choose master to accept request
        req_master_valid = 0;
        req_master = 'x;
        if (~read_slave_queue_wait_in) begin
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

        // setup slave signal
        avm_req_read = 0;
        avm_req_write = 0;
        avm_req_address = 'x;
        avm_req_writedata = 'x;
        avm_req_byteenable = 'x;
        if (req_master_valid) begin
            avm_req_read = avs_req_read[req_master];
            avm_req_write = avs_req_write[req_master];
            avm_req_address = avs_req_address[req_master];
            avm_req_writedata = avs_req_writedata[req_master];
            avm_req_byteenable = avs_req_byteenable[req_master];
        end

        // setup queue signal
        read_slave_queue_in = req_master;
        read_slave_queue_valid_in = req_master_valid & avm_req_read & ~avm_req_waitrequest;

        // setup waitrequest signal to master
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_waitrequest[i] = 1;
            if (req_master_valid & req_master == i) begin
                avs_req_waitrequest[i] = avm_req_waitrequest;
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_prev_master <= 0;
        end else begin
            req_prev_master <= req_master_valid ? req_master : 0;
        end
    end

    // response handling
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            logic valid;

            valid = read_slave_queue_valid_out & avm_res_readdatavalid & read_slave_queue_out == i;

            read_res_queue_valid_in[i] = valid;
            read_res_queue_in[i] = avm_res_readdata;
        end

        begin
            logic valid;
            logic [MASTER_WIDTH - 1 : 0] master;

            master = read_slave_queue_out;
            valid = avm_res_readdatavalid & read_slave_queue_valid_out;

            read_slave_queue_wait_out = ~valid;
            avm_res_waitresponse = ~valid;
        end

        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_readdata[i] = read_res_queue_out[i];
            avs_res_readdatavalid[i] = read_res_queue_valid_out[i];
            read_res_queue_wait_out[i] = avs_res_waitresponse[i];
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                read_count[i] <= 0;
            end
        end else begin
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                logic enq, deq;
                enq = avs_req_read[i] & ~avs_req_waitrequest[i];
                deq = avs_res_readdatavalid[i] & ~avs_res_waitresponse[i];
                if (enq & ~deq) begin
                    read_count[i] <= read_count[i] + 1'b1;
                end
                if (~enq & deq) begin
                    read_count[i] <= read_count[i] - 1'b1;
                end
            end
        end
    end

    hs_cqc #(
        .WIDTH(MASTER_WIDTH),
        .DEPTH(MAX_READS_PER_SLAVE)
    ) read_slave_queue (
        .clk(clk),
        .rstn(rstn),
        .in(read_slave_queue_in),
        .valid_in(read_slave_queue_valid_in),
        .wait_in(read_slave_queue_wait_in),
        .out(read_slave_queue_out),
        .valid_out(read_slave_queue_valid_out),
        .wait_out(read_slave_queue_wait_out)
    );

    for (i = 0; i < NUM_MASTERS; ++i) begin : read_res_queues
        hs_cqc #(
            .WIDTH(DATA_WIDTH),
            .DEPTH(MAX_READS_PER_MASTER)
        ) read_res_queue (
            .clk(clk),
            .rstn(rstn),
            .in(read_res_queue_in[i]),
            .valid_in(read_res_queue_valid_in[i]),
            .wait_in(read_res_queue_wait_in[i]),
            .out(read_res_queue_out[i]),
            .valid_out(read_res_queue_valid_out[i]),
            .wait_out(read_res_queue_wait_out[i])
        );
    end

    // performance counter
    if (~ENABLE_PERFORMANCE_COUNTER) begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end else begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end
        end
    end else begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end else begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= (avs_read[i] | avs_write[i]) & ~avs_waitrequest[i];
                    pc_request_cycle[i] <= (avs_read[i] | avs_write[i]);
                end
            end
        end
    end

endmodule

module dc_arbiter_n1_ro #(
    parameter NUM_MASTERS = 2,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ENABLE_PERFORMANCE_COUNTER = 0
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address[NUM_MASTERS],
    input logic avs_read[NUM_MASTERS],
    output logic [DATA_WIDTH - 1 : 0] avs_readdata[NUM_MASTERS],
    output logic avs_readdatavalid[NUM_MASTERS],
    output logic avs_waitrequest[NUM_MASTERS],
    input logic avs_waitresponse[NUM_MASTERS],
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid, 
    input logic avm_waitrequest,
    output logic avm_waitresponse,
    output logic pc_request[NUM_MASTERS],
    output logic pc_request_cycle[NUM_MASTERS]
);

    genvar i;

    localparam MASTER_WIDTH = $clog2(NUM_MASTERS) > 0 ? $clog2(NUM_MASTERS) : 1;
    localparam MAX_READS_PER_MASTER = 512;
    localparam MAX_READS_PER_SLAVE = 512;

    logic [$clog2(MAX_READS_PER_MASTER + 1) - 1 : 0] read_count[NUM_MASTERS];

    // avs -> cutter -> blocker -> req handler
    // address
    localparam AVS_REQ_WIDTH = ADDR_WIDTH;
    logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in[NUM_MASTERS], avs_req_out[NUM_MASTERS];
    logic avs_req_valid_in[NUM_MASTERS], avs_req_wait_in[NUM_MASTERS];
    logic avs_req_valid_out[NUM_MASTERS], avs_req_wait_out[NUM_MASTERS];
    logic [ADDR_WIDTH - 1 : 0] avs_req_address[NUM_MASTERS];
    logic avs_req_read[NUM_MASTERS], avs_req_waitrequest[NUM_MASTERS];
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_in[i] = {avs_address[i]};
            avs_req_valid_in[i] = avs_read[i];
            avs_waitrequest[i] = avs_req_wait_in[i];
        end
    end
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            {avs_req_address[i]} = avs_req_out[i];
            avs_req_read[i] = avs_req_valid_out[i];
            avs_req_wait_out[i] = avs_req_waitrequest[i];
        end
    end
    for (i = 0; i < NUM_MASTERS; ++i) begin: avs_req
        hs_cb #(
            .WIDTH(AVS_REQ_WIDTH)
        ) avs_req_cutter (
            .clk(clk),
            .rstn(rstn),
            .in(avs_req_in[i]),
            .valid_in(avs_req_valid_in[i]),
            .wait_in(avs_req_wait_in[i]),
            .out(avs_req_out[i]),
            .valid_out(avs_req_valid_out[i]),
            .wait_out(avs_req_wait_out[i]),
            .cond((read_count[i] & MAX_READS_PER_MASTER) != 0)
        );
    end

    // req handler -> cutter -> avm
    // address
    localparam AVM_REQ_WIDTH = ADDR_WIDTH;
    logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
    logic avm_req_valid_in, avm_req_wait_in;
    logic avm_req_valid_out, avm_req_wait_out;
    logic [ADDR_WIDTH - 1 : 0] avm_req_address;
    logic avm_req_read, avm_req_waitrequest;
    always_comb begin
        avm_req_in = {avm_req_address};
        avm_req_valid_in = avm_req_read;
        avm_req_waitrequest = avm_req_wait_in;
    end
    always_comb begin
        {avm_address} = avm_req_out;
        avm_read = avm_req_valid_out;
        avm_req_wait_out = avm_waitrequest;
    end
    hs_cutter #(
        .WIDTH(AVM_REQ_WIDTH)
    ) avm_req_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(avm_req_in),
        .valid_in(avm_req_valid_in),
        .wait_in(avm_req_wait_in),
        .out(avm_req_out),
        .valid_out(avm_req_valid_out),
        .wait_out(avm_req_wait_out)
    );

    // avm -> cutter -> res handler
    // data
    localparam AVM_RES_WIDTH = DATA_WIDTH;
    logic [AVM_RES_WIDTH - 1 : 0] avm_res_in, avm_res_out;
    logic avm_res_valid_in, avm_res_wait_in;
    logic avm_res_valid_out, avm_res_wait_out;
    logic [DATA_WIDTH - 1 : 0] avm_res_readdata;
    logic avm_res_readdatavalid, avm_res_waitresponse;
    always_comb begin
        avm_res_in = {avm_readdata};
        avm_res_valid_in = avm_readdatavalid;
        avm_waitresponse = avm_res_wait_in;
    end
    always_comb begin
        avm_res_readdata = avm_res_out;
        avm_res_readdatavalid = avm_res_valid_out;
        avm_res_wait_out = avm_res_waitresponse;
    end
    hs_cutter #(
        .WIDTH(AVM_RES_WIDTH)
    ) avm_res_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(avm_res_in),
        .valid_in(avm_res_valid_in),
        .wait_in(avm_res_wait_in),
        .out(avm_res_out),
        .valid_out(avm_res_valid_out),
        .wait_out(avm_res_wait_out)
    );

    // res handler -> cutter -> avs
    // data
    localparam AVS_RES_WIDTH = DATA_WIDTH;
    logic [AVS_RES_WIDTH - 1 : 0] avs_res_in[NUM_MASTERS], avs_res_out[NUM_MASTERS];
    logic avs_res_valid_in[NUM_MASTERS], avs_res_wait_in[NUM_MASTERS];
    logic avs_res_valid_out[NUM_MASTERS], avs_res_wait_out[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_res_readdata[NUM_MASTERS];
    logic avs_res_readdatavalid[NUM_MASTERS], avs_res_waitresponse[NUM_MASTERS];
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_in[i] = avs_res_readdata[i];
            avs_res_valid_in[i] = avs_res_readdatavalid[i];
            avs_res_waitresponse[i] = avs_res_wait_in[i];
        end
    end
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_readdata[i] = avs_res_out[i];
            avs_readdatavalid[i] = avs_res_valid_out[i];
            avs_res_wait_out[i] = avs_waitresponse[i];
        end
    end
    for (i = 0; i < NUM_MASTERS; ++i) begin: avs_res
        hs_cutter #(
            .WIDTH(AVS_RES_WIDTH)
        ) avs_res_cutter (
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

    logic req_master_valid;
    logic [MASTER_WIDTH - 1 : 0] req_master;
    logic [MASTER_WIDTH - 1 : 0] req_prev_master;

    logic [MASTER_WIDTH - 1 : 0] read_slave_queue_in;
    logic read_slave_queue_valid_in;
    logic read_slave_queue_wait_in;
    logic [MASTER_WIDTH - 1 : 0] read_slave_queue_out;
    logic read_slave_queue_valid_out;
    logic read_slave_queue_wait_out;

    logic [DATA_WIDTH - 1 : 0] read_res_queue_in[NUM_MASTERS];
    logic read_res_queue_valid_in[NUM_MASTERS];
    logic read_res_queue_wait_in[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] read_res_queue_out[NUM_MASTERS];
    logic read_res_queue_valid_out[NUM_MASTERS];
    logic read_res_queue_wait_out[NUM_MASTERS];

    // request handling
    always_comb begin
        // choose master to accept request
        req_master_valid = 0;
        req_master = 'x;
        if (~read_slave_queue_wait_in) begin
            logic [NUM_MASTERS - 1 : 0] x, y;
            x = 'x;
            for (int j = 0; j < NUM_MASTERS; ++j) begin
                x[j] = avs_req_read[j];
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

        // setup slave signal
        avm_req_read = 0;
        avm_req_address = 'x;
        if (req_master_valid) begin
            avm_req_read = avs_req_read[req_master];
            avm_req_address = avs_req_address[req_master];
        end

        // setup queue signal
        read_slave_queue_in = req_master;
        read_slave_queue_valid_in = req_master_valid & avm_req_read & ~avm_req_waitrequest;

        // setup waitrequest signal to master
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_waitrequest[i] = 1;
            if (req_master_valid & req_master == i) begin
                avs_req_waitrequest[i] = avm_req_waitrequest;
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_prev_master <= 0;
        end else begin
            req_prev_master <= req_master_valid ? req_master : 0;
        end
    end

    // response handling
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            logic valid;

            valid = read_slave_queue_valid_out & avm_res_readdatavalid & read_slave_queue_out == i;

            read_res_queue_valid_in[i] = valid;
            read_res_queue_in[i] = avm_res_readdata;
        end

        begin
            logic valid;
            logic [MASTER_WIDTH - 1 : 0] master;

            master = read_slave_queue_out;
            valid = avm_res_readdatavalid & read_slave_queue_valid_out;

            read_slave_queue_wait_out = ~valid;
            avm_res_waitresponse = ~valid;
        end

        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_readdata[i] = read_res_queue_out[i];
            avs_res_readdatavalid[i] = read_res_queue_valid_out[i];
            read_res_queue_wait_out[i] = avs_res_waitresponse[i];
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                read_count[i] <= 0;
            end
        end else begin
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                logic enq, deq;
                enq = avs_req_read[i] & ~avs_req_waitrequest[i];
                deq = avs_res_readdatavalid[i] & ~avs_res_waitresponse[i];
                if (enq & ~deq) begin
                    read_count[i] <= read_count[i] + 1'b1;
                end
                if (~enq & deq) begin
                    read_count[i] <= read_count[i] - 1'b1;
                end
            end
        end
    end

    hs_cqc #(
        .WIDTH(MASTER_WIDTH),
        .DEPTH(MAX_READS_PER_SLAVE)
    ) read_slave_queue (
        .clk(clk),
        .rstn(rstn),
        .in(read_slave_queue_in),
        .valid_in(read_slave_queue_valid_in),
        .wait_in(read_slave_queue_wait_in),
        .out(read_slave_queue_out),
        .valid_out(read_slave_queue_valid_out),
        .wait_out(read_slave_queue_wait_out)
    );

    for (i = 0; i < NUM_MASTERS; ++i) begin : read_res_queues
        hs_cqc #(
            .WIDTH(DATA_WIDTH),
            .DEPTH(MAX_READS_PER_MASTER)
        ) read_res_queue (
            .clk(clk),
            .rstn(rstn),
            .in(read_res_queue_in[i]),
            .valid_in(read_res_queue_valid_in[i]),
            .wait_in(read_res_queue_wait_in[i]),
            .out(read_res_queue_out[i]),
            .valid_out(read_res_queue_valid_out[i]),
            .wait_out(read_res_queue_wait_out[i])
        );
    end

    // performance counter
    if (~ENABLE_PERFORMANCE_COUNTER) begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end else begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end
        end
    end else begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end else begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= avs_read[i] & ~avs_waitrequest[i];
                    pc_request_cycle[i] <= avs_read[i];
                end
            end
        end
    end

endmodule

module dc_arbiter_n1_wo #(
    parameter NUM_MASTERS = 2,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ENABLE_PERFORMANCE_COUNTER = 0
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address[NUM_MASTERS],
    input logic avs_write[NUM_MASTERS],
    input logic [DATA_WIDTH - 1 : 0] avs_writedata[NUM_MASTERS],
    input logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable[NUM_MASTERS],
    output logic avs_waitrequest[NUM_MASTERS],
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest,
    output logic pc_request[NUM_MASTERS],
    output logic pc_request_cycle[NUM_MASTERS]
);

    genvar i;

    localparam MASTER_WIDTH = $clog2(NUM_MASTERS) > 0 ? $clog2(NUM_MASTERS) : 1;

    // avs -> cutter -> blocker -> req handler
    // address | writedata | byteenable
    localparam AVS_REQ_WIDTH = ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8;
    logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in[NUM_MASTERS], avs_req_out[NUM_MASTERS];
    logic avs_req_valid_in[NUM_MASTERS], avs_req_wait_in[NUM_MASTERS];
    logic avs_req_valid_out[NUM_MASTERS], avs_req_wait_out[NUM_MASTERS];
    logic [ADDR_WIDTH - 1 : 0] avs_req_address[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_req_writedata[NUM_MASTERS];
    logic [DATA_WIDTH / 8 - 1 : 0] avs_req_byteenable[NUM_MASTERS];
    logic avs_req_write[NUM_MASTERS], avs_req_waitrequest[NUM_MASTERS];
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_in[i] = {avs_address[i], avs_writedata[i], avs_byteenable[i]};
            avs_req_valid_in[i] = avs_write[i];
            avs_waitrequest[i] = avs_req_wait_in[i];
        end
    end
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            {avs_req_address[i], avs_req_writedata[i], avs_req_byteenable[i]} = avs_req_out[i];
            avs_req_write[i] = avs_req_valid_out[i];
            avs_req_wait_out[i] = avs_req_waitrequest[i];
        end
    end
    for (i = 0; i < NUM_MASTERS; ++i) begin: avs_req
        hs_bypass #(
            .WIDTH(AVS_REQ_WIDTH)
        ) avs_req_cutter (
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

    // req handler -> cutter -> avm
    // address | writedata | byteenable
    localparam AVM_REQ_WIDTH = ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8;
    logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
    logic avm_req_valid_in, avm_req_wait_in;
    logic avm_req_valid_out, avm_req_wait_out;
    logic [ADDR_WIDTH - 1 : 0] avm_req_address;
    logic [DATA_WIDTH - 1 : 0] avm_req_writedata;
    logic [DATA_WIDTH / 8 - 1 : 0] avm_req_byteenable;
    logic avm_req_write, avm_req_waitrequest;
    always_comb begin
        avm_req_in = {avm_req_address, avm_req_writedata, avm_req_byteenable};
        avm_req_valid_in = avm_req_write;
        avm_req_waitrequest = avm_req_wait_in;
    end
    always_comb begin
        {avm_address, avm_writedata, avm_byteenable} = avm_req_out;
        avm_write = avm_req_valid_out;
        avm_req_wait_out = avm_waitrequest;
    end
    hs_cutter #(
        .WIDTH(AVM_REQ_WIDTH)
    ) avm_req_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(avm_req_in),
        .valid_in(avm_req_valid_in),
        .wait_in(avm_req_wait_in),
        .out(avm_req_out),
        .valid_out(avm_req_valid_out),
        .wait_out(avm_req_wait_out)
    );

    logic req_master_valid;
    logic [MASTER_WIDTH - 1 : 0] req_master;
    logic [MASTER_WIDTH - 1 : 0] req_prev_master;

    // request handling
    always_comb begin
        // choose master to accept request
        req_master_valid = 0;
        req_master = 'x;
        if (1) begin
            logic [NUM_MASTERS - 1 : 0] x, y;
            x = 'x;
            for (int j = 0; j < NUM_MASTERS; ++j) begin
                x[j] = avs_req_write[j];
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

        // setup slave signal
        avm_req_write = 0;
        avm_req_address = 'x;
        avm_req_writedata = 'x;
        avm_req_byteenable = 'x;
        if (req_master_valid) begin
            avm_req_write = avs_req_write[req_master];
            avm_req_address = avs_req_address[req_master];
            avm_req_writedata = avs_req_writedata[req_master];
            avm_req_byteenable = avs_req_byteenable[req_master];
        end

        // setup waitrequest signal to master
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_waitrequest[i] = 1;
            if (req_master_valid & req_master == i) begin
                avs_req_waitrequest[i] = avm_req_waitrequest;
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_prev_master <= 0;
        end else begin
            req_prev_master <= req_master_valid ? req_master : 0;
        end
    end

    // performance counter
    if (~ENABLE_PERFORMANCE_COUNTER) begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end else begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end
        end
    end else begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= 0;
                    pc_request_cycle[i] <= 0;
                end
            end else begin
                for (int i = 0; i < NUM_MASTERS; ++i) begin
                    pc_request[i] <= avs_write[i] & ~avs_waitrequest[i];
                    pc_request_cycle[i] <= avs_write[i];
                end
            end
        end
    end

endmodule
