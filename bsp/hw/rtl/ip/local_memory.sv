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

module local_memory #(
    parameter NUM_MASTERS = 1,
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 512,
    parameter BANK_WIDTH = 2
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
    input logic avs_waitresponse[NUM_MASTERS]
);

    // think bank as slave
    localparam NUM_SLAVES = 2 ** BANK_WIDTH;
    localparam MASTER_WIDTH = $clog2(NUM_MASTERS) > 1 ? $clog2(NUM_MASTERS) : 1;
    localparam SLAVE_WIDTH = $clog2(NUM_SLAVES) > 1 ? $clog2(NUM_SLAVES) : 1;

    localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
    localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
    localparam INDEX_WIDTH = ADDR_WIDTH - $clog2(NUM_SLAVES) - OFFSET_WIDTH; // be careful when NUM_SLAVES == 1
    localparam MAX_READS_PER_MASTER = 512;

    logic [$clog2(MAX_READS_PER_MASTER + 1) - 1 : 0] read_count[NUM_MASTERS];

    logic [SLAVE_WIDTH - 1 : 0] req_slave[NUM_MASTERS];
    logic req_master_valid[NUM_SLAVES];
    logic [MASTER_WIDTH - 1 : 0] req_master[NUM_SLAVES];

    logic [INDEX_WIDTH - 1 : 0] ram_address[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] ram_readdata[NUM_SLAVES];
    logic ram_readaddressstall[NUM_SLAVES];
    logic ram_write[NUM_SLAVES];
    logic [DATA_WIDTH - 1 : 0] ram_writedata[NUM_SLAVES];
    logic [DATA_WIDTH_BYTE - 1 : 0] ram_byteenable[NUM_SLAVES];

    // master outstanding read pipeline
    logic master_valid_in[NUM_MASTERS];
    logic master_wait_in[NUM_MASTERS];
    logic [SLAVE_WIDTH - 1 : 0] master_data_in[NUM_MASTERS];
    logic master_s0_valid[NUM_MASTERS];
    logic master_s0_wait[NUM_MASTERS];
    logic [SLAVE_WIDTH - 1 : 0] master_s0_data[NUM_MASTERS];
    logic master_valid_out[NUM_MASTERS];
    logic master_wait_out[NUM_MASTERS];
    logic [SLAVE_WIDTH - 1 : 0] master_data_out[NUM_MASTERS];

    // slave outstanding read pipeline
    logic slave_valid_in[NUM_SLAVES];
    logic slave_wait_in[NUM_SLAVES];
    logic [MASTER_WIDTH - 1 : 0] slave_data_in[NUM_SLAVES];
    logic slave_s0_valid[NUM_SLAVES];
    logic slave_s0_wait[NUM_SLAVES];
    logic [MASTER_WIDTH - 1 : 0] slave_s0_data[NUM_SLAVES];
    logic slave_valid_out[NUM_SLAVES];
    logic slave_wait_out[NUM_SLAVES];
    logic [MASTER_WIDTH - 1 : 0] slave_data_out[NUM_SLAVES];

    // master read response queue
    logic [DATA_WIDTH - 1 : 0] read_res_queue_in[NUM_MASTERS];
    logic read_res_queue_valid_in[NUM_MASTERS];
    logic read_res_queue_wait_in[NUM_MASTERS];
    logic [DATA_WIDTH - 1 : 0] read_res_queue_out[NUM_MASTERS];
    logic read_res_queue_valid_out[NUM_MASTERS];
    logic read_res_queue_wait_out[NUM_MASTERS];
    logic read_res_queue_empty[NUM_MASTERS];
    logic read_res_queue_full[NUM_MASTERS];

    // request handling
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            // each master picks slave
            req_slave[i] = NUM_SLAVES == 1 ? 0 : avs_address[i][OFFSET_WIDTH +: SLAVE_WIDTH]; // be careful when NUM_SLAVES == 1
        end
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            // each slave picks master to accept request
            req_master_valid[i] = 0;
            req_master[i] = 'x;
            if (~slave_wait_in[i]) begin
                for (int j = 0; j < NUM_MASTERS; ++j) begin
                    if (req_slave[j] == i & ((avs_read[j] & read_count[j] != MAX_READS_PER_MASTER) | avs_write[j]) & ~master_wait_in[j]) begin
                        req_master_valid[i] = 1;
                        req_master[i] = j;
                    end
                end
            end

            // setup slave signal
            ram_address[i] = 'x;
            ram_write[i] = 0;
            ram_writedata[i] = 'x;
            ram_byteenable[i] = 'x;
            if (req_master_valid[i]) begin
                ram_address[i] = avs_address[req_master[i]][ADDR_WIDTH - 1 -: INDEX_WIDTH];
                ram_write[i] = avs_write[req_master[i]];
                ram_writedata[i] = avs_writedata[req_master[i]];
                ram_byteenable[i] = avs_byteenable[req_master[i]];
            end

            // setup queue signal
            slave_data_in[i] = req_master[i];
            slave_valid_in[i] = req_master_valid[i] & ~ram_write[i]; // ~write == read
        end
        // setup master signal
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_waitrequest[i] = 1;
            for (int j = 0; j < NUM_SLAVES; ++j) begin
                if (req_master_valid[j] & req_master[j] == i) begin
                    avs_waitrequest[i] = 0;
                end
            end
            master_data_in[i] = req_slave[i];
            master_valid_in[i] = avs_read[i] & ~avs_waitrequest[i];
        end
    end

    // repsonse handling
    // in
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            master_wait_in[i] = master_s0_wait[i];
        end
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            slave_wait_in[i] = slave_s0_wait[i];
        end
    end
    // in -> s0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                master_s0_valid[i] <= 0;
            end
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                slave_s0_valid[i] <= 0;
            end
        end else begin
            for (int i = 0; i < NUM_MASTERS; ++i) begin
                master_s0_valid[i] <= master_s0_wait[i] | (master_valid_in[i] & ~master_wait_in[i]);
                if (~master_s0_wait[i]) begin
                    master_s0_data[i] <= master_data_in[i];
                end
            end
            for (int i = 0; i < NUM_SLAVES; ++i) begin
                slave_s0_valid[i] <= slave_s0_wait[i] | (slave_valid_in[i] & ~slave_wait_in[i]);
                if (~slave_s0_wait[i]) begin
                    slave_s0_data[i] <= slave_data_in[i];
                end
            end
        end
    end
    // s0
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            master_valid_out[i] = master_s0_valid[i];
            master_s0_wait[i] = master_valid_out[i] & master_wait_out[i];
            master_data_out[i] = master_s0_data[i];
        end
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            slave_valid_out[i] = slave_s0_valid[i];
            slave_s0_wait[i] = slave_valid_out[i] & slave_wait_out[i];
            slave_data_out[i] = slave_s0_data[i];

            ram_readaddressstall[i] = slave_s0_wait[i];
        end
    end

    always_comb begin
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            logic valid;
            logic [SLAVE_WIDTH - 1 : 0] slave;

            slave = master_data_out[i];
            valid = master_valid_out[i] & slave_valid_out[slave] & slave_data_out[slave] == i;

            read_res_queue_valid_in[i] = valid;
            read_res_queue_in[i] = ram_readdata[slave];

            master_wait_out[i] = ~valid;
        end
        for (int i = 0; i < NUM_SLAVES; ++i) begin
            logic valid;
            logic [MASTER_WIDTH - 1 : 0] master;

            master = slave_data_out[i];
            valid = slave_valid_out[i] & master_valid_out[master] & master_data_out[master] == i;

            slave_wait_out[i] = ~valid;
        end
        for (int i = 0; i < NUM_MASTERS; ++i) begin
            avs_readdata[i] = read_res_queue_out[i];
            avs_readdatavalid[i] = read_res_queue_valid_out[i];
            read_res_queue_wait_out[i] = avs_waitresponse[i];
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
                enq = avs_read[i] & ~avs_waitrequest[i];
                deq = avs_readdatavalid[i] & ~avs_waitresponse[i];
                if (enq & ~deq) begin
                    read_count[i] <= read_count[i] + 1;
                end
                if (~enq & deq) begin
                    read_count[i] <= read_count[i] - 1;
                end
            end
        end
    end

    genvar i;

    for (i = 0; i < NUM_SLAVES; ++i) begin: slave_ram_inst
        ram_sdp_be #(
            .WIDTH(DATA_WIDTH),
            .DEPTH(2 ** INDEX_WIDTH)
        ) ram_inst (
            .byteena_a(ram_byteenable[i]),
            .clock(clk),
            .data(ram_writedata[i]),
            .rd_addressstall(ram_readaddressstall[i]),
            .rdaddress(ram_address[i]),
            .wraddress(ram_address[i]),
            .wren(ram_write[i]),
            .q(ram_readdata[i])
        );
    end

    for (i = 0; i < NUM_MASTERS; ++i) begin: master_read_res_queue
        scfifo #(
            .lpm_width(DATA_WIDTH),
            .lpm_widthu($clog2(MAX_READS_PER_MASTER)),
            .lpm_numwords(2 ** $clog2(MAX_READS_PER_MASTER)),
            .lpm_showahead("ON"),
            .overflow_checking("OFF"),
            .underflow_checking("OFF"),
            .add_ram_output_register("ON")
        ) read_res_queue (
            .clock(clk),
            .aclr(~rstn),
            .data(read_res_queue_in[i]),
            .wrreq(read_res_queue_valid_in[i] & ~read_res_queue_wait_in[i]),
            .rdreq(read_res_queue_valid_out[i] & ~read_res_queue_wait_out[i]),
            .q(read_res_queue_out[i]),
            .empty(read_res_queue_empty[i]),
            .full(read_res_queue_full[i])
        );
        assign read_res_queue_wait_in[i] = read_res_queue_full[i];
        assign read_res_queue_valid_out[i] = ~read_res_queue_empty[i];
    end

endmodule
