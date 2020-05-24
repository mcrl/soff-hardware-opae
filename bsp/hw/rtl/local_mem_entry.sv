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

`include "platform_if.vh"
`include "u_mem.vh"

module local_mem_entry #(
    parameter ADDR_WIDTH = 27,
    parameter DATA_WIDTH = 512,
    parameter BURST_WIDTH = 7,
    parameter NUM_MASTERS = 2,
    parameter NUM_SLAVES = 2
) (
    input logic clk,
    input logic rstn,
    u_mem_if.to_afu u_mem[NUM_MASTERS],
    avalon_mem_if.to_fiu local_mem[NUM_SLAVES]
);

    localparam MASTER_WIDTH = $clog2(NUM_MASTERS);
    localparam SLAVE_WIDTH = $clog2(NUM_SLAVES);
    localparam MASTER_ADDR_WIDTH = ADDR_WIDTH;
    localparam SLAVE_ADDR_WIDTH = ADDR_WIDTH - SLAVE_WIDTH;

    logic [MASTER_ADDR_WIDTH - 1 : 0] avs_address[NUM_MASTERS];
    logic avs_read[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_readdata[NUM_MASTERS];
    logic avs_readdatavalid[NUM_MASTERS];
    logic avs_write[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] avs_writedata[NUM_MASTERS];
    logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable[NUM_MASTERS];
    logic [BURST_WIDTH - 1 : 0] avs_burstcount[NUM_MASTERS];
    logic avs_waitrequest[NUM_MASTERS];
    logic [SLAVE_ADDR_WIDTH - 1 : 0] avm_address[NUM_SLAVES];
    logic avm_read[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] avm_readdata[NUM_SLAVES];
    logic avm_readdatavalid[NUM_SLAVES];
    logic avm_write[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] avm_writedata[NUM_SLAVES];
    logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable[NUM_SLAVES];
    logic [BURST_WIDTH - 1 : 0] avm_burstcount[NUM_SLAVES];
    logic avm_waitrequest[NUM_SLAVES];

    for (genvar i = 0; i < NUM_MASTERS; ++i) begin
        assign avs_address[i] = u_mem[i].address;
        assign avs_read[i] = u_mem[i].read;
        assign u_mem[i].readdata = avs_readdata[i];
        assign u_mem[i].readdatavalid = avs_readdatavalid[i];
        assign avs_write[i] = u_mem[i].write;
        assign avs_writedata[i] = u_mem[i].writedata;
        assign avs_byteenable[i] = u_mem[i].byteenable;
        assign avs_burstcount[i] = u_mem[i].burstcount;
        assign u_mem[i].waitrequest = avs_waitrequest[i];
    end

    for (genvar i = 0; i < NUM_SLAVES; ++i) begin
        assign local_mem[i].address = avm_address[i];
        assign local_mem[i].read = avm_read[i];
        assign avm_readdata[i] = local_mem[i].readdata;
        assign avm_readdatavalid[i] = local_mem[i].readdatavalid;
        assign local_mem[i].write = avm_write[i];
        assign local_mem[i].writedata = avm_writedata[i];
        assign local_mem[i].byteenable = avm_byteenable[i];
        assign local_mem[i].burstcount = avm_burstcount[i];
        assign avm_waitrequest[i] = local_mem[i].waitrequest;
    end

    if (0) begin
        // interleave using high bits
        // e.g., addr[COL_WIDTH] 0 -> slave 0, addr[COL_WIDTH] 1 -> slave 1
        // For pac_a10, effective row buffer size = 2KB * 2 (# of bank groups) * 4 (# of chips) = 16KB
        // = 14 bit byte address = 8 bit byte address
        // => COL_WIDTH = 8 for row buffer interleaving
        amm_arbiter_high_bits_interleave #(
            .NUM_MASTERS(NUM_MASTERS),
            .NUM_SLAVES(NUM_SLAVES),
            .MASTER_ADDR_WIDTH(MASTER_ADDR_WIDTH),
            .SLAVE_ADDR_WIDTH(SLAVE_ADDR_WIDTH),
            .DATA_WIDTH(DATA_WIDTH),
            .BURST_WIDTH(BURST_WIDTH),
            .COL_WIDTH(8)
        ) amm_arbiter_high_bits_interleave_inst (
            .clk(clk),
            .rstn(rstn),
            .avs_address(avs_address),
            .avs_read(avs_read),
            .avs_readdata(avs_readdata),
            .avs_readdatavalid(avs_readdatavalid),
            .avs_write(avs_write),
            .avs_writedata(avs_writedata),
            .avs_byteenable(avs_byteenable),
            .avs_burstcount(avs_burstcount),
            .avs_waitrequest(avs_waitrequest),
            .avm_address(avm_address),
            .avm_read(avm_read),
            .avm_readdata(avm_readdata),
            .avm_readdatavalid(avm_readdatavalid),
            .avm_write(avm_write),
            .avm_writedata(avm_writedata),
            .avm_byteenable(avm_byteenable),
            .avm_burstcount(avm_burstcount),
            .avm_waitrequest(avm_waitrequest)
        );
    end else begin
        // interleave using last bits
        // e.g., addr 0 -> slave 0, addr 1 -> slave 1
        amm_arbiter_last_bits_interleave #(
            .NUM_MASTERS(NUM_MASTERS),
            .NUM_SLAVES(NUM_SLAVES),
            .MASTER_ADDR_WIDTH(MASTER_ADDR_WIDTH),
            .SLAVE_ADDR_WIDTH(SLAVE_ADDR_WIDTH),
            .DATA_WIDTH(DATA_WIDTH),
            .BURST_WIDTH(BURST_WIDTH)
        ) amm_arbiter_last_bits_interleave_inst (
            .clk(clk),
            .rstn(rstn),
            .avs_address(avs_address),
            .avs_read(avs_read),
            .avs_readdata(avs_readdata),
            .avs_readdatavalid(avs_readdatavalid),
            .avs_write(avs_write),
            .avs_writedata(avs_writedata),
            .avs_byteenable(avs_byteenable),
            .avs_burstcount(avs_burstcount),
            .avs_waitrequest(avs_waitrequest),
            .avm_address(avm_address),
            .avm_read(avm_read),
            .avm_readdata(avm_readdata),
            .avm_readdatavalid(avm_readdatavalid),
            .avm_write(avm_write),
            .avm_writedata(avm_writedata),
            .avm_byteenable(avm_byteenable),
            .avm_burstcount(avm_burstcount),
            .avm_waitrequest(avm_waitrequest)
        );
    end

endmodule

module amm_arbiter_high_bits_interleave #(
    parameter NUM_MASTERS = 2,
    parameter NUM_SLAVES = 2,
    parameter MASTER_ADDR_WIDTH = 27,
    parameter SLAVE_ADDR_WIDTH = 26,
    parameter DATA_WIDTH = 512,
    parameter BURST_WIDTH = 7,
    parameter COL_WIDTH = 8
) (
    input logic clk,
    input logic rstn,
    input logic [MASTER_ADDR_WIDTH - 1 : 0] avs_address[NUM_MASTERS],
    input logic avs_read[NUM_MASTERS],
    output logic [DATA_WIDTH - 1 : 0] avs_readdata[NUM_MASTERS],
    output logic avs_readdatavalid[NUM_MASTERS],
    input logic avs_write[NUM_MASTERS],
    input logic [DATA_WIDTH - 1 : 0] avs_writedata[NUM_MASTERS],
    input logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable[NUM_MASTERS],
    input logic [BURST_WIDTH - 1 : 0] avs_burstcount[NUM_MASTERS],
    output logic avs_waitrequest[NUM_MASTERS],
    output logic [SLAVE_ADDR_WIDTH - 1 : 0] avm_address[NUM_SLAVES],
    output logic avm_read[NUM_SLAVES],
    input logic [DATA_WIDTH - 1 : 0] avm_readdata[NUM_SLAVES],
    input logic avm_readdatavalid[NUM_SLAVES],
    output logic avm_write[NUM_SLAVES],
    output logic [DATA_WIDTH - 1 : 0] avm_writedata[NUM_SLAVES],
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable[NUM_SLAVES],
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount[NUM_SLAVES],
    input logic avm_waitrequest[NUM_SLAVES]
);

    genvar i;

    localparam MASTER_WIDTH = $clog2(NUM_MASTERS);
    localparam SLAVE_WIDTH = $clog2(NUM_SLAVES);
    localparam MAX_READS_PER_SLAVE = 512;

    logic [$clog2(MAX_READS_PER_SLAVE + 1) - 1 : 0] avm_read_count[NUM_SLAVES];

    // avs_req
    // rw | address | writedata | byteenable | burstcount
    localparam AVS_REQ_WIDTH = 1 + MASTER_ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8 + BURST_WIDTH;
    logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in[NUM_MASTERS], avs_req_out[NUM_MASTERS];
    logic avs_req_valid_in[NUM_MASTERS], avs_req_valid_out[NUM_MASTERS];
    logic avs_req_wait_in[NUM_MASTERS], avs_req_wait_out[NUM_MASTERS];
    logic avs_req_read[NUM_MASTERS], avs_req_write[NUM_MASTERS];
    logic [MASTER_ADDR_WIDTH - 1 : 0] avs_req_address[NUM_MASTERS];
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
    localparam AVM_REQ_WIDTH = 1 + SLAVE_ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8 + BURST_WIDTH;
    logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in[NUM_SLAVES], avm_req_out[NUM_SLAVES];
    logic avm_req_valid_in[NUM_SLAVES], avm_req_valid_out[NUM_SLAVES];
    logic avm_req_wait_in[NUM_SLAVES], avm_req_wait_out[NUM_SLAVES];
    logic avm_req_read[NUM_SLAVES], avm_req_write[NUM_SLAVES];
    logic avm_req_cond[NUM_SLAVES];
    logic [SLAVE_ADDR_WIDTH - 1 : 0] avm_req_address[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] avm_req_writedata[NUM_SLAVES];
    logic [DATA_WIDTH / 8 - 1 : 0] avm_req_byteenable[NUM_SLAVES];
    logic [BURST_WIDTH - 1 : 0] avm_req_burstcount[NUM_SLAVES];
    logic avm_req_waitrequest[NUM_SLAVES];
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_req_in[i] = {avm_req_write[i], avm_req_address[i], avm_req_writedata[i], avm_req_byteenable[i], avm_req_burstcount[i]};
            avm_req_valid_in[i] = avm_req_read[i] | avm_req_write[i];
            avm_req_waitrequest[i] = avm_req_wait_in[i];
        end
    end
    for (i = 0; i < NUM_SLAVES; ++i) begin: avm_req_gates
        hs_cb #(
            .WIDTH(AVM_REQ_WIDTH)
        ) avm_req_gate (
            .clk(clk),
            .rstn(rstn),
            .in(avm_req_in[i]),
            .valid_in(avm_req_valid_in[i]),
            .wait_in(avm_req_wait_in[i]),
            .out(avm_req_out[i]),
            .valid_out(avm_req_valid_out[i]),
            .wait_out(avm_req_wait_out[i]),
            .cond(avm_req_cond[i])
        );
    end
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            logic rw;
            {rw, avm_address[i], avm_writedata[i], avm_byteenable[i], avm_burstcount[i]} = avm_req_out[i];
            avm_read[i] = avm_req_valid_out[i] & ~rw;
            avm_write[i] = avm_req_valid_out[i] & rw;
            avm_req_wait_out[i] = avm_waitrequest[i];
            avm_req_cond[i] = avm_read_count[i] + avm_burstcount[i] > MAX_READS_PER_SLAVE;
        end
    end

    // avm_res
    // readdata
    localparam AVM_RES_WIDTH = DATA_WIDTH;
    logic [AVM_RES_WIDTH - 1 : 0] avm_res_in[NUM_SLAVES], avm_res_out[NUM_SLAVES];
    logic avm_res_valid_in[NUM_SLAVES], avm_res_valid_out[NUM_SLAVES];
    logic avm_res_wait_in[NUM_SLAVES], avm_res_wait_out[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] avm_res_readdata[NUM_SLAVES];
    logic avm_res_readdatavalid[NUM_SLAVES];
    logic avm_res_waitresponse[NUM_SLAVES];
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_res_in[i] = {avm_readdata[i]};
            avm_res_valid_in[i] = avm_readdatavalid[i];
            // avm_res_wait_in will never assert
        end
    end
    for (i = 0; i < NUM_SLAVES; ++i) begin: avm_res_gates
        hs_cqc #(
            .WIDTH(AVM_RES_WIDTH),
            .DEPTH(MAX_READS_PER_SLAVE)
        ) avm_res_gate (
            .clk(clk),
            .rstn(rstn),
            .in(avm_res_in[i]),
            .valid_in(avm_res_valid_in[i]),
            .wait_in(avm_res_wait_in[i]),
            .out(avm_res_out[i]),
            .valid_out(avm_res_valid_out[i]),
            .wait_out(avm_res_wait_out[i])
        );
    end
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            {avm_res_readdata[i]} = avm_res_out[i];
            avm_res_readdatavalid[i] = avm_res_valid_out[i];
            avm_res_wait_out[i] = avm_res_waitresponse[i];
        end
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
    // master | burstcount | slave | burstcount
    localparam AVM_QUEUE_WIDTH = MASTER_WIDTH + BURST_WIDTH + SLAVE_WIDTH + BURST_WIDTH;
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
    // succ = avm_req, avs_queue, avm_queue
    /*
     * TODO This is optimized for the case where only one master is active at a time.
     * Since one master is kernel and another master is DMA engine, this holds true if there is no comm-comp overlap.
     */
    logic req_master_valid;
    logic [MASTER_WIDTH - 1 : 0] req_master;
    logic [BURST_WIDTH - 1 : 0] req_bc0;

    logic req_avm_req_state[NUM_SLAVES];
    logic req_avm_queue_state;

    logic req_locked;
    logic [MASTER_WIDTH - 1 : 0] req_locked_master;
    logic [BURST_WIDTH - 1 : 0] req_locked_burstcount;
    logic [BURST_WIDTH - 1 : 0] req_locked_count;
    logic [MASTER_ADDR_WIDTH - 1 : 0] req_locked_addr;
    logic [SLAVE_WIDTH - 1 : 0] req_locked_slave;
    logic [BURST_WIDTH - 1 : 0] req_locked_bc0;

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
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (avs_req_read[i] | avs_req_write[i]) begin
                    req_master_valid = 1;
                    req_master = i;
                end
            end
        end
        req_bc0 = 0;

        // init
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_req_read[i] = 0;
            avm_req_write[i] = 0;
            avm_req_address[i] = 'x;
            avm_req_writedata[i] = 'x;
            avm_req_byteenable[i] = 'x;
            avm_req_burstcount[i] = 'x;
        end
        avm_queue_valid_in = 0;
        avm_queue_in = 'x;
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_waitrequest[i] = 1;
        end

        if (req_master_valid) begin
            if (avs_req_read[req_master]) begin
                logic [MASTER_ADDR_WIDTH : 0] addr;
                logic [BURST_WIDTH - 1 : 0] bc;
                addr = avs_req_address[req_master];
                bc = avs_req_burstcount[req_master];
                if ({addr}[COL_WIDTH] == {addr + (bc - 1'b1)}[COL_WIDTH]) begin
                    // does not split
                    logic [SLAVE_WIDTH - 1 : 0] slave;
                    slave = addr[COL_WIDTH +: SLAVE_WIDTH];
                    avm_req_address[slave] = {addr[MASTER_ADDR_WIDTH - 1 : COL_WIDTH + SLAVE_WIDTH], addr[COL_WIDTH - 1 : 0]};
                    avm_req_burstcount[slave] = bc;
                    avm_req_read[slave] = 1 & req_avm_req_state[slave] == 0;
                    avm_req_byteenable[slave] = avs_req_byteenable[req_master];

                    avm_queue_valid_in = 1 & req_avm_queue_state == 0;
                    avm_queue_in = {req_master, bc, slave, {BURST_WIDTH{1'b0}}};

                    avs_req_waitrequest[req_master] = (avm_queue_valid_in & avm_queue_wait_in)
                                                    | (avm_req_read[slave] & avm_req_waitrequest[slave]);
                end else begin
                    // split into two slaves
                    logic [BURST_WIDTH - 1 : 0] bc0, bc1;
                    logic [SLAVE_WIDTH - 1 : 0] slave0, slave1;
                    slave0 = addr[COL_WIDTH +: SLAVE_WIDTH];
                    slave1 = slave0 + 1'b1;
                    bc0 = {{addr + bc}[MASTER_ADDR_WIDTH : COL_WIDTH], {COL_WIDTH{1'b0}}} - addr;
                    bc1 = bc - bc0;
                    avm_req_address[slave0] = {addr[MASTER_ADDR_WIDTH - 1 : COL_WIDTH + SLAVE_WIDTH], addr[COL_WIDTH - 1 : 0]};
                    avm_req_address[slave1] = {{addr + bc}[MASTER_ADDR_WIDTH - 1 : COL_WIDTH + SLAVE_WIDTH], {COL_WIDTH{1'b0}}};
                    avm_req_burstcount[slave0] = bc0;
                    avm_req_burstcount[slave1] = bc1;
                    avm_req_read[slave0] = 1 & req_avm_req_state[slave0] == 0;
                    avm_req_read[slave1] = 1 & req_avm_req_state[slave1] == 0;
                    avm_req_byteenable[slave0] = avs_req_byteenable[req_master];
                    avm_req_byteenable[slave1] = avs_req_byteenable[req_master];

                    avm_queue_valid_in = 1 & req_avm_queue_state == 0;
                    avm_queue_in = {req_master, bc, slave0, bc0};

                    avs_req_waitrequest[req_master] = (avm_queue_valid_in & avm_queue_wait_in)
                                                    | (avm_req_read[slave0] & avm_req_waitrequest[slave0])
                                                    | (avm_req_read[slave1] & avm_req_waitrequest[slave1]);
                end
            end

            if (avs_req_write[req_master]) begin
                logic [MASTER_ADDR_WIDTH : 0] addr;
                logic [BURST_WIDTH - 1 : 0] bc;
                logic [SLAVE_WIDTH - 1 : 0] slave_cur;
                if (~req_locked) begin
                    addr = avs_req_address[req_master];
                    bc = avs_req_burstcount[req_master];
                    slave_cur = addr[COL_WIDTH +: SLAVE_WIDTH];
                end else begin
                    addr = req_locked_addr;
                    bc = req_locked_burstcount;
                    slave_cur = req_locked_slave;
                end
                if ({addr}[COL_WIDTH] == {addr + (bc - 1'b1)}[COL_WIDTH]) begin
                    // does not split
                    logic [SLAVE_WIDTH - 1 : 0] slave;
                    slave = addr[COL_WIDTH +: SLAVE_WIDTH];
                    avm_req_address[slave] = {addr[MASTER_ADDR_WIDTH - 1 : COL_WIDTH + SLAVE_WIDTH], addr[COL_WIDTH - 1 : 0]};
                    avm_req_burstcount[slave] = bc;
                    avm_req_write[slave] = 1 & req_avm_req_state[slave] == 0;
                    avm_req_byteenable[slave] = avs_req_byteenable[req_master];
                    avm_req_writedata[slave] = avs_req_writedata[req_master];

                    avs_req_waitrequest[req_master] = (avm_req_write[slave] & avm_req_waitrequest[slave]);

                end else begin
                    // split into two slaves
                    logic [BURST_WIDTH - 1 : 0] bc0, bc1;
                    logic [SLAVE_WIDTH - 1 : 0] slave0, slave1;
                    slave0 = addr[COL_WIDTH +: SLAVE_WIDTH];
                    slave1 = slave0 + 1'b1;
                    bc0 = {{addr + bc}[MASTER_ADDR_WIDTH : COL_WIDTH], {COL_WIDTH{1'b0}}} - addr;
                    req_bc0 = bc0;
                    bc1 = bc - bc0;
                    avm_req_address[slave0] = {addr[MASTER_ADDR_WIDTH - 1 : COL_WIDTH + SLAVE_WIDTH], addr[COL_WIDTH - 1 : 0]};
                    avm_req_address[slave1] = {{addr + bc}[MASTER_ADDR_WIDTH - 1 : COL_WIDTH + SLAVE_WIDTH], {COL_WIDTH{1'b0}}};
                    avm_req_burstcount[slave0] = bc0;
                    avm_req_burstcount[slave1] = bc1;
                    avm_req_write[slave0] = slave0 == slave_cur & req_avm_req_state[slave0] == 0;
                    avm_req_write[slave1] = slave1 == slave_cur & req_avm_req_state[slave1] == 0;
                    avm_req_byteenable[slave0] = avs_req_byteenable[req_master];
                    avm_req_byteenable[slave1] = avs_req_byteenable[req_master];
                    avm_req_writedata[slave0] = avs_req_writedata[req_master];
                    avm_req_writedata[slave1] = avs_req_writedata[req_master];

                    avs_req_waitrequest[req_master] = (avm_req_write[slave0] & avm_req_waitrequest[slave0])
                                                    | (avm_req_write[slave1] & avm_req_waitrequest[slave1]);
                end
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                req_avm_req_state[i] <= 0;
            end
            req_avm_queue_state <= 0;
            req_locked <= 0;
        end else begin
            if (~req_master_valid) begin
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    req_avm_req_state[i] <= 0;
                end
                req_avm_queue_state <= 0;
            end else if ((avs_req_read[req_master] | avs_req_write[req_master]) & ~avs_req_waitrequest[req_master]) begin
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    req_avm_req_state[i] <= 0;
                end
                req_avm_queue_state <= 0;
            end else begin
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    if ((avm_req_read[i] | avm_req_write[i]) & ~avm_req_waitrequest[i]) begin
                        req_avm_req_state[i] <= req_avm_req_state[i] + 1'b1;
                    end
                end
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
                    req_locked_addr <= avs_req_address[req_master];
                    req_locked_slave <= avs_req_address[req_master][COL_WIDTH +: SLAVE_WIDTH];
                    req_locked_bc0 <= req_bc0;
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
                            if (1 == req_bc0) begin
                                req_locked_slave <= avs_req_address[req_master][COL_WIDTH +: SLAVE_WIDTH] + 1'b1;
                            end
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
                        if (req_locked_count == req_locked_bc0) begin
                            req_locked_slave <= req_locked_slave + 1'b1;
                        end
                    end
                end
            end
        end
    end

    // response handler
    // multi-pred pattern
    // pred = avm_queue, avm_res
    // succ = avs_res
    logic [MASTER_WIDTH - 1 : 0] res_master;
    logic [BURST_WIDTH - 1 : 0] res_burstcount;
    logic [SLAVE_WIDTH - 1 : 0] res_slave;
    logic [BURST_WIDTH - 1 : 0] res_bc0;

    logic res_locked;
    logic [SLAVE_WIDTH - 1 : 0] res_locked_slave;
    logic [BURST_WIDTH - 1 : 0] res_locked_count;

    always_comb begin
        // init
        avm_queue_wait_out = 1;
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_res_waitresponse[i] = 1;
        end
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_readdatavalid[i] = 0;
            avs_res_readdata[i] = 'x;
        end

        {res_master, res_burstcount, res_slave, res_bc0} = avm_queue_out;
        if (avm_queue_valid_out) begin
            logic [SLAVE_WIDTH - 1 : 0] slave;
            slave = res_locked ? res_locked_slave : res_slave;

            if (avm_res_readdatavalid[slave]) begin
                avs_res_readdatavalid[res_master] = 1;
                avs_res_readdata[res_master] = avm_res_readdata[slave];
                avm_res_waitresponse[slave] = 0;
                avm_queue_wait_out = (res_locked ? res_locked_count : 1) != res_burstcount;
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_locked <= 0;
        end else begin
            if (~res_locked) begin
                if (avm_queue_valid_out) begin
                    res_locked <= 1;
                    res_locked_slave <= res_slave;
                    res_locked_count <= 1;
                    if (avm_res_readdatavalid[res_slave]) begin
                        if (res_burstcount == 1) begin
                            res_locked <= 0;
                        end else begin
                            res_locked_count <= 2;
                            if (1 == res_bc0) begin
                                res_locked_slave <= res_slave + 1'b1;
                            end
                        end
                    end
                end
            end else begin
                if (avm_res_readdatavalid[res_locked_slave]) begin
                    if (res_locked_count == res_burstcount) begin
                        res_locked <= 0;
                    end else begin
                        res_locked_count <= res_locked_count + 1'b1;
                        if (res_locked_count == res_bc0) begin
                            res_locked_slave <= res_locked_slave + 1'b1;
                        end
                    end
                end
            end
        end
    end

    // counter
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                avm_read_count[i] <= 0;
            end
        end else begin
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                logic avm_read_enq, avm_read_deq;
                avm_read_enq = avm_read[i] & ~avm_waitrequest[i]; // + avm_burstcount[i];
                avm_read_deq = avm_res_readdatavalid[i] & ~avm_res_waitresponse[i]; // - 1
                if (avm_read_enq) avm_read_count[i] = avm_read_count[i] + avm_burstcount[i];
                if (avm_read_deq) avm_read_count[i] = avm_read_count[i] - 1'b1;
            end
        end
    end

endmodule

module amm_arbiter_last_bits_interleave #(
    parameter NUM_MASTERS = 2,
    parameter NUM_SLAVES = 2,
    parameter MASTER_ADDR_WIDTH = 27,
    parameter SLAVE_ADDR_WIDTH = 26,
    parameter DATA_WIDTH = 512,
    parameter BURST_WIDTH = 7
) (
    input logic clk,
    input logic rstn,
    input logic [MASTER_ADDR_WIDTH - 1 : 0] avs_address[NUM_MASTERS],
    input logic avs_read[NUM_MASTERS],
    output logic [DATA_WIDTH - 1 : 0] avs_readdata[NUM_MASTERS],
    output logic avs_readdatavalid[NUM_MASTERS],
    input logic avs_write[NUM_MASTERS],
    input logic [DATA_WIDTH - 1 : 0] avs_writedata[NUM_MASTERS],
    input logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable[NUM_MASTERS],
    input logic [BURST_WIDTH - 1 : 0] avs_burstcount[NUM_MASTERS],
    output logic avs_waitrequest[NUM_MASTERS],
    output logic [SLAVE_ADDR_WIDTH - 1 : 0] avm_address[NUM_SLAVES],
    output logic avm_read[NUM_SLAVES],
    input logic [DATA_WIDTH - 1 : 0] avm_readdata[NUM_SLAVES],
    input logic avm_readdatavalid[NUM_SLAVES],
    output logic avm_write[NUM_SLAVES],
    output logic [DATA_WIDTH - 1 : 0] avm_writedata[NUM_SLAVES],
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable[NUM_SLAVES],
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount[NUM_SLAVES],
    input logic avm_waitrequest[NUM_SLAVES]
);

    genvar i;

    localparam MASTER_WIDTH = $clog2(NUM_MASTERS);
    localparam SLAVE_WIDTH = $clog2(NUM_SLAVES);
    localparam MAX_READS_PER_SLAVE = 512;

    logic [$clog2(MAX_READS_PER_SLAVE + 1) - 1 : 0] avm_read_count[NUM_SLAVES];

    // avs_req
    // rw | address | writedata | byteenable | burstcount
    localparam AVS_REQ_WIDTH = 1 + MASTER_ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8 + BURST_WIDTH;
    logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in[NUM_MASTERS], avs_req_out[NUM_MASTERS];
    logic avs_req_valid_in[NUM_MASTERS], avs_req_valid_out[NUM_MASTERS];
    logic avs_req_wait_in[NUM_MASTERS], avs_req_wait_out[NUM_MASTERS];
    logic avs_req_read[NUM_MASTERS], avs_req_write[NUM_MASTERS];
    logic [MASTER_ADDR_WIDTH - 1 : 0] avs_req_address[NUM_MASTERS];
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
    localparam AVM_REQ_WIDTH = 1 + SLAVE_ADDR_WIDTH + DATA_WIDTH + DATA_WIDTH / 8 + BURST_WIDTH;
    logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in[NUM_SLAVES], avm_req_out[NUM_SLAVES];
    logic avm_req_valid_in[NUM_SLAVES], avm_req_valid_out[NUM_SLAVES];
    logic avm_req_wait_in[NUM_SLAVES], avm_req_wait_out[NUM_SLAVES];
    logic avm_req_read[NUM_SLAVES], avm_req_write[NUM_SLAVES];
    logic avm_req_cond[NUM_SLAVES];
    logic [SLAVE_ADDR_WIDTH - 1 : 0] avm_req_address[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] avm_req_writedata[NUM_SLAVES];
    logic [DATA_WIDTH / 8 - 1 : 0] avm_req_byteenable[NUM_SLAVES];
    logic [BURST_WIDTH - 1 : 0] avm_req_burstcount[NUM_SLAVES];
    logic avm_req_waitrequest[NUM_SLAVES];
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_req_in[i] = {avm_req_write[i], avm_req_address[i], avm_req_writedata[i], avm_req_byteenable[i], avm_req_burstcount[i]};
            avm_req_valid_in[i] = avm_req_read[i] | avm_req_write[i];
            avm_req_waitrequest[i] = avm_req_wait_in[i];
        end
    end
    for (i = 0; i < NUM_SLAVES; ++i) begin: avm_req_gates
        hs_cb #(
            .WIDTH(AVM_REQ_WIDTH)
        ) avm_req_gate (
            .clk(clk),
            .rstn(rstn),
            .in(avm_req_in[i]),
            .valid_in(avm_req_valid_in[i]),
            .wait_in(avm_req_wait_in[i]),
            .out(avm_req_out[i]),
            .valid_out(avm_req_valid_out[i]),
            .wait_out(avm_req_wait_out[i]),
            .cond(avm_req_cond[i])
        );
    end
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            logic rw;
            {rw, avm_address[i], avm_writedata[i], avm_byteenable[i], avm_burstcount[i]} = avm_req_out[i];
            avm_read[i] = avm_req_valid_out[i] & ~rw;
            avm_write[i] = avm_req_valid_out[i] & rw;
            avm_req_wait_out[i] = avm_waitrequest[i];
            avm_req_cond[i] = avm_read_count[i] + avm_burstcount[i] > MAX_READS_PER_SLAVE;
        end
    end

    // avm_res
    // readdata
    localparam AVM_RES_WIDTH = DATA_WIDTH;
    logic [AVM_RES_WIDTH - 1 : 0] avm_res_in[NUM_SLAVES], avm_res_out[NUM_SLAVES];
    logic avm_res_valid_in[NUM_SLAVES], avm_res_valid_out[NUM_SLAVES];
    logic avm_res_wait_in[NUM_SLAVES], avm_res_wait_out[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] avm_res_readdata[NUM_SLAVES];
    logic avm_res_readdatavalid[NUM_SLAVES];
    logic avm_res_waitresponse[NUM_SLAVES];
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_res_in[i] = {avm_readdata[i]};
            avm_res_valid_in[i] = avm_readdatavalid[i];
            // avm_res_wait_in will never assert
        end
    end
    for (i = 0; i < NUM_SLAVES; ++i) begin: avm_res_gates
        hs_cqc #(
            .WIDTH(AVM_RES_WIDTH),
            .DEPTH(MAX_READS_PER_SLAVE)
        ) avm_res_gate (
            .clk(clk),
            .rstn(rstn),
            .in(avm_res_in[i]),
            .valid_in(avm_res_valid_in[i]),
            .wait_in(avm_res_wait_in[i]),
            .out(avm_res_out[i]),
            .valid_out(avm_res_valid_out[i]),
            .wait_out(avm_res_wait_out[i])
        );
    end
    always_comb begin
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            {avm_res_readdata[i]} = avm_res_out[i];
            avm_res_readdatavalid[i] = avm_res_valid_out[i];
            avm_res_wait_out[i] = avm_res_waitresponse[i];
        end
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
    // master | burstcount | slave
    localparam AVM_QUEUE_WIDTH = MASTER_WIDTH + BURST_WIDTH + SLAVE_WIDTH;
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
    // succ = avm_req, avs_queue, avm_queue
    /*
     * TODO This is optimized for the case where only one master is active at a time.
     * Since one master is kernel and another master is DMA engine, this holds true if there is no comm-comp overlap.
     */
    logic req_master_valid;
    logic [MASTER_WIDTH - 1 : 0] req_master;

    logic req_avm_req_state[NUM_SLAVES];
    logic req_avm_queue_state;

    logic req_locked;
    logic [MASTER_WIDTH - 1 : 0] req_locked_master;
    logic [BURST_WIDTH - 1 : 0] req_locked_burstcount;
    logic [BURST_WIDTH - 1 : 0] req_locked_count;
    logic [MASTER_ADDR_WIDTH - 1 : 0] req_locked_addr;
    logic [SLAVE_WIDTH - 1 : 0] req_locked_slave;

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
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                if (avs_req_read[i] | avs_req_write[i]) begin
                    req_master_valid = 1;
                    req_master = i;
                end
            end
        end

        // init
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_req_read[i] = 0;
            avm_req_write[i] = 0;
            avm_req_address[i] = 'x;
            avm_req_writedata[i] = 'x;
            avm_req_byteenable[i] = 'x;
            avm_req_burstcount[i] = 'x;
        end
        avm_queue_valid_in = 0;
        avm_queue_in = 'x;
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_req_waitrequest[i] = 1;
        end

        if (req_master_valid) begin
            if (avs_req_read[req_master]) begin
                logic [SLAVE_ADDR_WIDTH - 1 : 0] addr;
                logic [SLAVE_WIDTH - 1 : 0] ofs;
                logic [BURST_WIDTH - 1 : 0] bc;
                addr = avs_req_address[req_master][MASTER_ADDR_WIDTH - 1 -: SLAVE_ADDR_WIDTH];
                ofs = avs_req_address[req_master][SLAVE_WIDTH - 1 : 0];
                bc = avs_req_burstcount[req_master];
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    if (i < ofs) begin
                        // ceil(((ofs + bc) - (i + NUM_SLAVES)) / NUM_SLAVES)
                        // = (ofs + bc - i - 1) / NUM_SLAVES
                        // note that ofs + bc - i - 1 = (ofs - i) + (bc - 1) > 0
                        avm_req_address[i] = addr + 1'b1;
                        avm_req_burstcount[i] = ((ofs - i) + (bc - 1'b1)) >> SLAVE_WIDTH;
                    end else begin
                        // ceil(((ofs + bc) - (i)) / NUM_SLAVES)
                        // = (ofs + bc - i + NUM_SLAVES - 1) / NUM_SLAVES
                        // note that ofs + bc - i + NUM_SLAVES - 1 = (ofs + bc) + (NUM_SLAVES - 1 - i) > 0
                        avm_req_address[i] = addr;
                        avm_req_burstcount[i] = ((ofs + bc) + (NUM_SLAVES - 1 - i)) >> SLAVE_WIDTH;
                    end
                    avm_req_read[i] = avm_req_burstcount[i] > 0 & req_avm_req_state[i] == 0;
                    avm_req_byteenable[i] = avs_req_byteenable[req_master];
                end

                avm_queue_valid_in = 1 & req_avm_queue_state == 0;
                avm_queue_in = {req_master, bc, ofs};

                avs_req_waitrequest[req_master] = avm_queue_valid_in & avm_queue_wait_in;
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    avs_req_waitrequest[req_master] |= avm_req_read[i] & avm_req_waitrequest[i];
                end
            end

            if (avs_req_write[req_master]) begin
                logic [SLAVE_ADDR_WIDTH - 1 : 0] addr;
                logic [SLAVE_WIDTH - 1 : 0] ofs;
                logic [BURST_WIDTH - 1 : 0] bc;
                logic [SLAVE_WIDTH - 1 : 0] slave;
                if (~req_locked) begin
                    addr = avs_req_address[req_master][MASTER_ADDR_WIDTH - 1 -: SLAVE_ADDR_WIDTH];
                    ofs = avs_req_address[req_master][SLAVE_WIDTH - 1 : 0];
                    bc = avs_req_burstcount[req_master];
                    slave = avs_req_address[req_master][SLAVE_WIDTH - 1 : 0];
                end else begin
                    addr = req_locked_addr[MASTER_ADDR_WIDTH - 1 -: SLAVE_ADDR_WIDTH];
                    ofs = req_locked_addr[SLAVE_WIDTH - 1 : 0];
                    bc = req_locked_burstcount;
                    slave = req_locked_slave;
                end
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    if (i < ofs) begin
                        // ceil(((ofs + bc) - (i + NUM_SLAVES)) / NUM_SLAVES)
                        // = (ofs + bc - i - 1) / NUM_SLAVES
                        // note that ofs + bc - i - 1 = (ofs - i) + (bc - 1) > 0
                        avm_req_address[i] = addr + 1'b1;
                        avm_req_burstcount[i] = ((ofs - i) + (bc - 1'b1)) >> SLAVE_WIDTH;
                    end else begin
                        // ceil(((ofs + bc) - (i)) / NUM_SLAVES)
                        // = (ofs + bc - i + NUM_SLAVES - 1) / NUM_SLAVES
                        // note that ofs + bc - i + NUM_SLAVES - 1 = (ofs + bc) + (NUM_SLAVES - 1 - i) > 0
                        avm_req_address[i] = addr;
                        avm_req_burstcount[i] = ((ofs + bc) + (NUM_SLAVES - 1 - i)) >> SLAVE_WIDTH;
                    end
                    avm_req_write[i] = i == slave & req_avm_req_state[i] == 0;
                    avm_req_byteenable[i] = avs_req_byteenable[req_master];
                    avm_req_writedata[i] = avs_req_writedata[req_master];
                end

                avs_req_waitrequest[req_master] = 0;
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    avs_req_waitrequest[req_master] |= avm_req_write[i] & avm_req_waitrequest[i];
                end
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                req_avm_req_state[i] <= 0;
            end
            req_avm_queue_state <= 0;
            req_locked <= 0;
        end else begin
            if (~req_master_valid) begin
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    req_avm_req_state[i] <= 0;
                end
                req_avm_queue_state <= 0;
            end else if ((avs_req_read[req_master] | avs_req_write[req_master]) & ~avs_req_waitrequest[req_master]) begin
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    req_avm_req_state[i] <= 0;
                end
                req_avm_queue_state <= 0;
            end else begin
                for (int i = 0; i < NUM_SLAVES; ++i) begin
                    if ((avm_req_read[i] | avm_req_write[i]) & ~avm_req_waitrequest[i]) begin
                        req_avm_req_state[i] <= req_avm_req_state[i] + 1'b1;
                    end
                end
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
                    req_locked_addr <= avs_req_address[req_master];
                    req_locked_slave <= avs_req_address[req_master][SLAVE_WIDTH - 1 : 0];
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
                            req_locked_slave <= avs_req_address[req_master][SLAVE_WIDTH - 1 : 0] + 1'b1;
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
                        req_locked_slave <= req_locked_slave + 1'b1;
                    end
                end
            end
        end
    end

    // response handler
    // multi-pred pattern
    // pred = avm_queue, avm_res
    // succ = avs_res
    logic [MASTER_WIDTH - 1 : 0] res_master;
    logic [BURST_WIDTH - 1 : 0] res_burstcount;
    logic [SLAVE_WIDTH - 1 : 0] res_slave;

    logic res_locked;
    logic [SLAVE_WIDTH - 1 : 0] res_locked_slave;
    logic [BURST_WIDTH - 1 : 0] res_locked_count;

    always_comb begin
        // init
        avm_queue_wait_out = 1;
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            avm_res_waitresponse[i] = 1;
        end
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_res_readdatavalid[i] = 0;
            avs_res_readdata[i] = 'x;
        end

        {res_master, res_burstcount, res_slave} = avm_queue_out;
        if (avm_queue_valid_out) begin
            logic [SLAVE_WIDTH - 1 : 0] slave;
            slave = res_locked ? res_locked_slave : res_slave;

            if (avm_res_readdatavalid[slave]) begin
                avs_res_readdatavalid[res_master] = 1;
                avs_res_readdata[res_master] = avm_res_readdata[slave];
                avm_res_waitresponse[slave] = 0;
                avm_queue_wait_out = (res_locked ? res_locked_count : 1) != res_burstcount;
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_locked <= 0;
        end else begin
            if (~res_locked) begin
                if (avm_queue_valid_out) begin
                    res_locked <= 1;
                    res_locked_slave <= res_slave;
                    res_locked_count <= 1;
                    if (avm_res_readdatavalid[res_slave]) begin
                        if (res_burstcount == 1) begin
                            res_locked <= 0;
                        end else begin
                            res_locked_slave <= res_slave + 1'b1;
                            res_locked_count <= 2;
                        end
                    end
                end
            end else begin
                if (avm_res_readdatavalid[res_locked_slave]) begin
                    if (res_locked_count == res_burstcount) begin
                        res_locked <= 0;
                    end else begin
                        res_locked_slave <= res_locked_slave + 1'b1;
                        res_locked_count <= res_locked_count + 1'b1;
                    end
                end
            end
        end
    end

    // counter
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                avm_read_count[i] <= 0;
            end
        end else begin
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                logic avm_read_enq, avm_read_deq;
                avm_read_enq = avm_read[i] & ~avm_waitrequest[i]; // + avm_burstcount[i];
                avm_read_deq = avm_res_readdatavalid[i] & ~avm_res_waitresponse[i]; // - 1
                if (avm_read_enq) avm_read_count[i] = avm_read_count[i] + avm_burstcount[i];
                if (avm_read_deq) avm_read_count[i] = avm_read_count[i] - 1'b1;
            end
        end
    end

endmodule
