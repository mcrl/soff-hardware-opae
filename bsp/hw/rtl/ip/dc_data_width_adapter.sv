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

module dc_data_width_adapter #(
    parameter ADDR_WIDTH = 32,
    parameter MASTER_DATA_WIDTH = 8,
    parameter SLAVE_DATA_WIDTH = 32
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_read,
    output logic [MASTER_DATA_WIDTH - 1 : 0] avs_readdata,
    output logic avs_readdatavalid,
    input logic avs_write,
    input logic [MASTER_DATA_WIDTH - 1 : 0] avs_writedata,
    input logic [MASTER_DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
    output logic avs_waitrequest,
    input logic avs_waitresponse,
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [SLAVE_DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid,
    output logic avm_write,
    output logic [SLAVE_DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [SLAVE_DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest,
    output logic avm_waitresponse
);

    localparam MASTER_DATA_WIDTH_BYTE = MASTER_DATA_WIDTH / 8;
    localparam MASTER_OFFSET_WIDTH = $clog2(MASTER_DATA_WIDTH_BYTE);
    localparam MASTER_INDEX_WIDTH = ADDR_WIDTH - MASTER_OFFSET_WIDTH;
    localparam SLAVE_DATA_WIDTH_BYTE = SLAVE_DATA_WIDTH / 8;
    localparam SLAVE_OFFSET_WIDTH = $clog2(SLAVE_DATA_WIDTH_BYTE);
    localparam SLAVE_INDEX_WIDTH = ADDR_WIDTH - SLAVE_OFFSET_WIDTH;

    // rw (0=read, 1=write) | addr | data | byteenable
    localparam DREQ_WIDTH = 1 + MASTER_INDEX_WIDTH + MASTER_DATA_WIDTH + MASTER_DATA_WIDTH_BYTE;
    logic [DREQ_WIDTH - 1 : 0] dreq_in, dreq_out;
    logic dreq_valid_in, dreq_wait_in;
    logic dreq_valid_out, dreq_wait_out;

    hs_cutter #(
        .WIDTH(DREQ_WIDTH)
    ) dreq_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(dreq_in),
        .valid_in(dreq_valid_in),
        .wait_in(dreq_wait_in),
        .out(dreq_out),
        .valid_out(dreq_valid_out),
        .wait_out(dreq_wait_out)
    );

    // rw (0=read, 1=write) | addr | data | byteenable
    localparam CREQ_WIDTH = 1 + SLAVE_INDEX_WIDTH + SLAVE_DATA_WIDTH + SLAVE_DATA_WIDTH_BYTE;
    logic [CREQ_WIDTH - 1 : 0] creq_in, creq_out;
    logic creq_valid_in, creq_wait_in;
    logic creq_valid_out, creq_wait_out;

    hs_cutter #(
        .WIDTH(CREQ_WIDTH)
    ) creq_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(creq_in),
        .valid_in(creq_valid_in),
        .wait_in(creq_wait_in),
        .out(creq_out),
        .valid_out(creq_valid_out),
        .wait_out(creq_wait_out)
    );

    // data
    localparam CRES_WIDTH = SLAVE_DATA_WIDTH;
    logic [CRES_WIDTH - 1 : 0] cres_in, cres_out;
    logic cres_valid_in, cres_wait_in;
    logic cres_valid_out, cres_wait_out;

    hs_cutter #(
        .WIDTH(CRES_WIDTH)
    ) cres_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(cres_in),
        .valid_in(cres_valid_in),
        .wait_in(cres_wait_in),
        .out(cres_out),
        .valid_out(cres_valid_out),
        .wait_out(cres_wait_out)
    );

    // data
    localparam DRES_WIDTH = MASTER_DATA_WIDTH;
    logic [DRES_WIDTH - 1 : 0] dres_in, dres_out;
    logic dres_valid_in, dres_wait_in;
    logic dres_valid_out, dres_wait_out;

    hs_cutter #(
        .WIDTH(DRES_WIDTH)
    ) dres_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(dres_in),
        .valid_in(dres_valid_in),
        .wait_in(dres_wait_in),
        .out(dres_out),
        .valid_out(dres_valid_out),
        .wait_out(dres_wait_out)
    );

    // offset
    localparam TXN_WIDTH = SLAVE_OFFSET_WIDTH - MASTER_OFFSET_WIDTH;
    localparam TXN_DEPTH = 512;
    logic [TXN_WIDTH - 1 : 0] txn_in, txn_out;
    logic txn_valid_in, txn_wait_in;
    logic txn_valid_out, txn_wait_out;

    hs_queue #(
        .WIDTH(TXN_WIDTH),
        .DEPTH(TXN_DEPTH)
    ) txn_queue (
        .clk(clk),
        .rstn(rstn),
        .in(txn_in),
        .valid_in(txn_valid_in),
        .wait_in(txn_wait_in),
        .out(txn_out),
        .valid_out(txn_valid_out),
        .wait_out(txn_wait_out)
    );

    // avs -> dreq
    always_comb begin
        dreq_valid_in = avs_read | avs_write;
        dreq_in = {avs_write, avs_address[ADDR_WIDTH - 1 -: MASTER_INDEX_WIDTH], avs_writedata, avs_byteenable};
        avs_waitrequest = dreq_wait_in;
    end

    // creq -> avm
    always_comb begin
        logic creq_out_rw;
        logic [SLAVE_INDEX_WIDTH - 1 : 0] creq_out_index;
        logic [SLAVE_DATA_WIDTH - 1 : 0] creq_out_data;
        logic [SLAVE_DATA_WIDTH_BYTE - 1 : 0] creq_out_byteenable;

        {creq_out_rw, creq_out_index, creq_out_data, creq_out_byteenable} = creq_out;

        avm_read = 0;
        avm_write = 0;
        avm_address = {creq_out_index, {SLAVE_OFFSET_WIDTH{1'b0}}};
        avm_writedata = creq_out_data;
        avm_byteenable = creq_out_byteenable;
        if (creq_valid_out) begin
            if (creq_out_rw) begin // write
                avm_write = 1;
            end else begin // read
                avm_read = 1;
            end
        end
        creq_wait_out = avm_waitrequest;
    end

    // avm -> cres
    always_comb begin
        cres_valid_in = avm_readdatavalid;
        cres_in = avm_readdata;
        avm_waitresponse = cres_wait_in;
    end

    // dres -> avs
    always_comb begin
        avs_readdatavalid = dres_valid_out;
        avs_readdata = dres_out;
        dres_wait_out = avs_waitresponse;
    end

    // dreq -> creq, txn
    logic req_creq_cnt;
    logic req_txn_cnt;

    always_comb begin
        logic dreq_out_rw;
        logic [MASTER_INDEX_WIDTH - 1 : 0] dreq_out_index;
        logic [MASTER_DATA_WIDTH - 1 : 0] dreq_out_data;
        logic [MASTER_DATA_WIDTH_BYTE - 1 : 0] dreq_out_byteenable;

        {dreq_out_rw, dreq_out_index, dreq_out_data, dreq_out_byteenable} = dreq_out;
        dreq_wait_out = 1;
        creq_valid_in = 0;
        creq_in = 'x;
        txn_valid_in = 0;
        txn_in = 'x;
        if (dreq_valid_out) begin
            logic [SLAVE_OFFSET_WIDTH - 1 : 0] offset;
            logic [SLAVE_INDEX_WIDTH - 1 : 0] creq_in_index;
            logic [SLAVE_DATA_WIDTH_BYTE - 1 : 0] creq_in_byteenable;
            logic [SLAVE_DATA_WIDTH - 1 : 0] creq_in_data;
            creq_in_index = dreq_out_index[MASTER_INDEX_WIDTH - 1 -: SLAVE_INDEX_WIDTH];
            if (dreq_out_rw) begin // write
                offset = {dreq_out_index[SLAVE_OFFSET_WIDTH - MASTER_OFFSET_WIDTH - 1 : 0], {MASTER_OFFSET_WIDTH{1'b0}}};
                creq_in_byteenable = {SLAVE_DATA_WIDTH_BYTE{1'b0}} | (dreq_out_byteenable << offset);
                creq_in_data = {SLAVE_DATA_WIDTH{1'b0}} | (dreq_out_data << offset * 8);

                creq_valid_in = 1;
                creq_in = {dreq_out_rw, creq_in_index, creq_in_data, creq_in_byteenable};
                dreq_wait_out = creq_wait_in;
            end else begin // read
                creq_in_byteenable = {SLAVE_DATA_WIDTH_BYTE{1'bx}};
                creq_in_data = {SLAVE_DATA_WIDTH{1'bx}};

                creq_valid_in = req_creq_cnt != 1;
                creq_in = {dreq_out_rw, creq_in_index, creq_in_data, creq_in_byteenable};
                txn_valid_in = req_txn_cnt != 1;
                txn_in = dreq_out_index[SLAVE_OFFSET_WIDTH - MASTER_OFFSET_WIDTH - 1 : 0];
                dreq_wait_out = (creq_valid_in & creq_wait_in) | (txn_valid_in & txn_wait_in);
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_creq_cnt <= 0;
            req_txn_cnt <= 0;
        end else begin
            if (dreq_valid_out & ~dreq_wait_out) begin
                req_creq_cnt <= 0;
                req_txn_cnt <= 0;
            end else begin
                if (creq_valid_in & ~creq_wait_in) begin
                    req_creq_cnt <= req_creq_cnt + 1'b1;
                end
                if (txn_valid_in & ~txn_wait_in) begin
                    req_txn_cnt <= req_txn_cnt + 1'b1;
                end
            end
        end
    end

    // txn, cres -> dres
    always_comb begin
        txn_wait_out = 1;
        cres_wait_out = 1;
        dres_valid_in = 0;
        dres_in = 'x;
        if (txn_valid_out & cres_valid_out) begin
            logic [SLAVE_OFFSET_WIDTH - 1 : 0] offset;
            logic [SLAVE_DATA_WIDTH - 1 : 0] data;
            offset = {txn_out, {MASTER_OFFSET_WIDTH{1'b0}}};
            data = cres_out;

            dres_valid_in = 1;
            dres_in = {data >> offset * 8}[MASTER_DATA_WIDTH - 1 : 0];
            txn_wait_out = dres_wait_in;
            cres_wait_out = dres_wait_in;
        end
    end

endmodule // dc_data_width_adapter

module dc_data_width_adapter_ro #(
    parameter ADDR_WIDTH = 32,
    parameter MASTER_DATA_WIDTH = 8,
    parameter SLAVE_DATA_WIDTH = 32
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_read,
    output logic [MASTER_DATA_WIDTH - 1 : 0] avs_readdata,
    output logic avs_readdatavalid,
    output logic avs_waitrequest,
    input logic avs_waitresponse,
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [SLAVE_DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid,
    input logic avm_waitrequest,
    output logic avm_waitresponse
);

    localparam MASTER_DATA_WIDTH_BYTE = MASTER_DATA_WIDTH / 8;
    localparam MASTER_OFFSET_WIDTH = $clog2(MASTER_DATA_WIDTH_BYTE);
    localparam MASTER_INDEX_WIDTH = ADDR_WIDTH - MASTER_OFFSET_WIDTH;
    localparam SLAVE_DATA_WIDTH_BYTE = SLAVE_DATA_WIDTH / 8;
    localparam SLAVE_OFFSET_WIDTH = $clog2(SLAVE_DATA_WIDTH_BYTE);
    localparam SLAVE_INDEX_WIDTH = ADDR_WIDTH - SLAVE_OFFSET_WIDTH;

    // addr
    localparam DREQ_WIDTH = MASTER_INDEX_WIDTH;
    logic [DREQ_WIDTH - 1 : 0] dreq_in, dreq_out;
    logic dreq_valid_in, dreq_wait_in;
    logic dreq_valid_out, dreq_wait_out;

    hs_cutter #(
        .WIDTH(DREQ_WIDTH)
    ) dreq_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(dreq_in),
        .valid_in(dreq_valid_in),
        .wait_in(dreq_wait_in),
        .out(dreq_out),
        .valid_out(dreq_valid_out),
        .wait_out(dreq_wait_out)
    );

    // addr
    localparam CREQ_WIDTH = SLAVE_INDEX_WIDTH;
    logic [CREQ_WIDTH - 1 : 0] creq_in, creq_out;
    logic creq_valid_in, creq_wait_in;
    logic creq_valid_out, creq_wait_out;

    hs_cutter #(
        .WIDTH(CREQ_WIDTH)
    ) creq_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(creq_in),
        .valid_in(creq_valid_in),
        .wait_in(creq_wait_in),
        .out(creq_out),
        .valid_out(creq_valid_out),
        .wait_out(creq_wait_out)
    );

    // data
    localparam CRES_WIDTH = SLAVE_DATA_WIDTH;
    logic [CRES_WIDTH - 1 : 0] cres_in, cres_out;
    logic cres_valid_in, cres_wait_in;
    logic cres_valid_out, cres_wait_out;

    hs_cutter #(
        .WIDTH(CRES_WIDTH)
    ) cres_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(cres_in),
        .valid_in(cres_valid_in),
        .wait_in(cres_wait_in),
        .out(cres_out),
        .valid_out(cres_valid_out),
        .wait_out(cres_wait_out)
    );

    // data
    localparam DRES_WIDTH = MASTER_DATA_WIDTH;
    logic [DRES_WIDTH - 1 : 0] dres_in, dres_out;
    logic dres_valid_in, dres_wait_in;
    logic dres_valid_out, dres_wait_out;

    hs_cutter #(
        .WIDTH(DRES_WIDTH)
    ) dres_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(dres_in),
        .valid_in(dres_valid_in),
        .wait_in(dres_wait_in),
        .out(dres_out),
        .valid_out(dres_valid_out),
        .wait_out(dres_wait_out)
    );

    // offset
    localparam TXN_WIDTH = SLAVE_OFFSET_WIDTH - MASTER_OFFSET_WIDTH;
    localparam TXN_DEPTH = 512;
    logic [TXN_WIDTH - 1 : 0] txn_in, txn_out;
    logic txn_valid_in, txn_wait_in;
    logic txn_valid_out, txn_wait_out;

    hs_queue #(
        .WIDTH(TXN_WIDTH),
        .DEPTH(TXN_DEPTH)
    ) txn_queue (
        .clk(clk),
        .rstn(rstn),
        .in(txn_in),
        .valid_in(txn_valid_in),
        .wait_in(txn_wait_in),
        .out(txn_out),
        .valid_out(txn_valid_out),
        .wait_out(txn_wait_out)
    );

    // avs -> dreq
    always_comb begin
        dreq_valid_in = avs_read;
        dreq_in = {avs_address[ADDR_WIDTH - 1 -: MASTER_INDEX_WIDTH]};
        avs_waitrequest = dreq_wait_in;
    end

    // creq -> avm
    always_comb begin
        logic [SLAVE_INDEX_WIDTH - 1 : 0] creq_out_index;

        {creq_out_index} = creq_out;

        avm_read = 0;
        avm_address = {creq_out_index, {SLAVE_OFFSET_WIDTH{1'b0}}};
        if (creq_valid_out) begin
            avm_read = 1;
        end
        creq_wait_out = avm_waitrequest;
    end

    // avm -> cres
    always_comb begin
        cres_valid_in = avm_readdatavalid;
        cres_in = avm_readdata;
        avm_waitresponse = cres_wait_in;
    end

    // dres -> avs
    always_comb begin
        avs_readdatavalid = dres_valid_out;
        avs_readdata = dres_out;
        dres_wait_out = avs_waitresponse;
    end

    // dreq -> creq, txn
    logic req_creq_cnt;
    logic req_txn_cnt;

    always_comb begin
        logic [MASTER_INDEX_WIDTH - 1 : 0] dreq_out_index;

        {dreq_out_index} = dreq_out;
        dreq_wait_out = 1;
        creq_valid_in = 0;
        creq_in = 'x;
        txn_valid_in = 0;
        txn_in = 'x;
        if (dreq_valid_out) begin
            logic [SLAVE_INDEX_WIDTH - 1 : 0] creq_in_index;
            creq_in_index = dreq_out_index[MASTER_INDEX_WIDTH - 1 -: SLAVE_INDEX_WIDTH];

            creq_valid_in = req_creq_cnt != 1;
            creq_in = {creq_in_index};
            txn_valid_in = req_txn_cnt != 1;
            txn_in = dreq_out_index[SLAVE_OFFSET_WIDTH - MASTER_OFFSET_WIDTH - 1 : 0];
            dreq_wait_out = (creq_valid_in & creq_wait_in) | (txn_valid_in & txn_wait_in);
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_creq_cnt <= 0;
            req_txn_cnt <= 0;
        end else begin
            if (dreq_valid_out & ~dreq_wait_out) begin
                req_creq_cnt <= 0;
                req_txn_cnt <= 0;
            end else begin
                if (creq_valid_in & ~creq_wait_in) begin
                    req_creq_cnt <= req_creq_cnt + 1'b1;
                end
                if (txn_valid_in & ~txn_wait_in) begin
                    req_txn_cnt <= req_txn_cnt + 1'b1;
                end
            end
        end
    end

    // txn, cres -> dres
    always_comb begin
        txn_wait_out = 1;
        cres_wait_out = 1;
        dres_valid_in = 0;
        dres_in = 'x;
        if (txn_valid_out & cres_valid_out) begin
            logic [SLAVE_OFFSET_WIDTH - 1 : 0] offset;
            logic [SLAVE_DATA_WIDTH - 1 : 0] data;
            offset = {txn_out, {MASTER_OFFSET_WIDTH{1'b0}}};
            data = cres_out;

            dres_valid_in = 1;
            dres_in = {data >> offset * 8}[MASTER_DATA_WIDTH - 1 : 0];
            txn_wait_out = dres_wait_in;
            cres_wait_out = dres_wait_in;
        end
    end

endmodule // dc_data_width_adapter_ro

module dc_data_width_adapter_wo #(
    parameter ADDR_WIDTH = 32,
    parameter MASTER_DATA_WIDTH = 8,
    parameter SLAVE_DATA_WIDTH = 32
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_write,
    input logic [MASTER_DATA_WIDTH - 1 : 0] avs_writedata,
    input logic [MASTER_DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
    output logic avs_waitrequest,
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_write,
    output logic [SLAVE_DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [SLAVE_DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest
);

    localparam MASTER_DATA_WIDTH_BYTE = MASTER_DATA_WIDTH / 8;
    localparam MASTER_OFFSET_WIDTH = $clog2(MASTER_DATA_WIDTH_BYTE);
    localparam MASTER_INDEX_WIDTH = ADDR_WIDTH - MASTER_OFFSET_WIDTH;
    localparam SLAVE_DATA_WIDTH_BYTE = SLAVE_DATA_WIDTH / 8;
    localparam SLAVE_OFFSET_WIDTH = $clog2(SLAVE_DATA_WIDTH_BYTE);
    localparam SLAVE_INDEX_WIDTH = ADDR_WIDTH - SLAVE_OFFSET_WIDTH;

    // addr | data | byteenable
    localparam DREQ_WIDTH = MASTER_INDEX_WIDTH + MASTER_DATA_WIDTH + MASTER_DATA_WIDTH_BYTE;
    logic [DREQ_WIDTH - 1 : 0] dreq_in, dreq_out;
    logic dreq_valid_in, dreq_wait_in;
    logic dreq_valid_out, dreq_wait_out;

    hs_cutter #(
        .WIDTH(DREQ_WIDTH)
    ) dreq_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(dreq_in),
        .valid_in(dreq_valid_in),
        .wait_in(dreq_wait_in),
        .out(dreq_out),
        .valid_out(dreq_valid_out),
        .wait_out(dreq_wait_out)
    );

    // addr | data | byteenable
    localparam CREQ_WIDTH = SLAVE_INDEX_WIDTH + SLAVE_DATA_WIDTH + SLAVE_DATA_WIDTH_BYTE;
    logic [CREQ_WIDTH - 1 : 0] creq_in, creq_out;
    logic creq_valid_in, creq_wait_in;
    logic creq_valid_out, creq_wait_out;

    hs_cutter #(
        .WIDTH(CREQ_WIDTH)
    ) creq_cutter (
        .clk(clk),
        .rstn(rstn),
        .in(creq_in),
        .valid_in(creq_valid_in),
        .wait_in(creq_wait_in),
        .out(creq_out),
        .valid_out(creq_valid_out),
        .wait_out(creq_wait_out)
    );

    // avs -> dreq
    always_comb begin
        dreq_valid_in = avs_write;
        dreq_in = {avs_address[ADDR_WIDTH - 1 -: MASTER_INDEX_WIDTH], avs_writedata, avs_byteenable};
        avs_waitrequest = dreq_wait_in;
    end

    // creq -> avm
    always_comb begin
        logic [SLAVE_INDEX_WIDTH - 1 : 0] creq_out_index;
        logic [SLAVE_DATA_WIDTH - 1 : 0] creq_out_data;
        logic [SLAVE_DATA_WIDTH_BYTE - 1 : 0] creq_out_byteenable;

        {creq_out_index, creq_out_data, creq_out_byteenable} = creq_out;

        avm_write = 0;
        avm_address = {creq_out_index, {SLAVE_OFFSET_WIDTH{1'b0}}};
        avm_writedata = creq_out_data;
        avm_byteenable = creq_out_byteenable;
        if (creq_valid_out) begin
            avm_write = 1;
        end
        creq_wait_out = avm_waitrequest;
    end

    // dreq -> creq
    always_comb begin
        logic [MASTER_INDEX_WIDTH - 1 : 0] dreq_out_index;
        logic [MASTER_DATA_WIDTH - 1 : 0] dreq_out_data;
        logic [MASTER_DATA_WIDTH_BYTE - 1 : 0] dreq_out_byteenable;

        {dreq_out_index, dreq_out_data, dreq_out_byteenable} = dreq_out;
        dreq_wait_out = 1;
        creq_valid_in = 0;
        creq_in = 'x;
        if (dreq_valid_out) begin
            logic [SLAVE_OFFSET_WIDTH - 1 : 0] offset;
            logic [SLAVE_INDEX_WIDTH - 1 : 0] creq_in_index;
            logic [SLAVE_DATA_WIDTH_BYTE - 1 : 0] creq_in_byteenable;
            logic [SLAVE_DATA_WIDTH - 1 : 0] creq_in_data;
            creq_in_index = dreq_out_index[MASTER_INDEX_WIDTH - 1 -: SLAVE_INDEX_WIDTH];
            offset = {dreq_out_index[SLAVE_OFFSET_WIDTH - MASTER_OFFSET_WIDTH - 1 : 0], {MASTER_OFFSET_WIDTH{1'b0}}};
            creq_in_byteenable = {SLAVE_DATA_WIDTH_BYTE{1'b0}} | (dreq_out_byteenable << offset);
            creq_in_data = {SLAVE_DATA_WIDTH{1'b0}} | (dreq_out_data << offset * 8);

            creq_valid_in = 1;
            creq_in = {creq_in_index, creq_in_data, creq_in_byteenable};
            dreq_wait_out = creq_wait_in;
        end
    end

endmodule // dc_data_width_adapter_wo
