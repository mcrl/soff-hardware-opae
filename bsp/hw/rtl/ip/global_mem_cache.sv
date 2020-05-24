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

module global_mem_cache #(
    parameter ADDR_WIDTH = 36,
    parameter DATA_WIDTH = 512,
    parameter DATAPATH_DATA_WIDTH = 32,
    parameter NUM_CACHE_ENTRY = 1024,
    parameter DISTRIBUTION_FACTOR = 1,
    parameter BURST_WIDTH = 4,
    parameter ENABLE_PERFORMANCE_COUNTER = 0
) (
    input logic clk,
    input logic rstn,
    input logic cache_clean,
    output logic cache_cleaned,

    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_read,
    output logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_readdata,
    output logic avs_readdatavalid,
    input logic avs_write,
    input logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_writedata,
    input logic [DATAPATH_DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
    output logic avs_waitrequest,
    input logic avs_waitresponse,

    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid,
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest,
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount,
    output logic pc_cache_hit,
    output logic pc_cache_miss
);

    localparam NUM_CACHE_ENTRY_ = NUM_CACHE_ENTRY / DISTRIBUTION_FACTOR;
    localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
    localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
    localparam INDEX_WIDTH = $clog2(NUM_CACHE_ENTRY_);
    localparam TAG_WIDTH = ADDR_WIDTH - INDEX_WIDTH - OFFSET_WIDTH;
    localparam DATAPATH_DATA_WIDTH_BYTE = DATAPATH_DATA_WIDTH / 8;
    localparam DATAPATH_OFFSET_WIDTH = $clog2(DATAPATH_DATA_WIDTH_BYTE);

    // when cleaning, we scan through index.
    // NUM_CACHE_ENTRY_ + 1 = start
    // 0 ~ NUM_CACHE_ENTRY_ - 1 = counting
    // NUM_CACHE_ENTRY_ = end
    // [0, NUM_CACHE_ENTRY_ + 2)
    localparam INDEX_CNT_WIDTH = $clog2(NUM_CACHE_ENTRY_ + 2);

    // STATE_CLEAN_PENDING: Clean signal is received, but there are pending requests.
    // Do not accept further requests, but should process all requests before going to STATE_CLEANING.
    localparam STATE_NORMAL = 0;
    localparam STATE_CLEAN_PENDING = 1;
    localparam STATE_CLEANING = 2;
    logic [1:0] state;

    // COUNTERS
    localparam MAX_AVS_WRITE_COUNT = 512;
    localparam MAX_AVM_READ_COUNT = 512;
    localparam MAX_CLEAN_COUNT = NUM_CACHE_ENTRY_;
    localparam MAX_WB_COUNT = NUM_CACHE_ENTRY_;
    logic [$clog2(MAX_AVS_WRITE_COUNT + 1) - 1 : 0] avs_write_count;
    logic [$clog2(MAX_AVM_READ_COUNT + 1) - 1 : 0] avm_read_count;
    logic [$clog2(MAX_CLEAN_COUNT + 1) - 1 : 0] clean_count;
    logic [$clog2(MAX_WB_COUNT + 1) - 1 : 0] wb_count;

    // datapath request
    // data format: RW (0 = read, 1 = write) | TAG | INDEX | OFFSET | DATA | BYTEENABLE
    // structure: avs req -> dp_req_blocker -> dp_req_cutter -> req handler
    localparam DP_REQ_WIDTH = 1 + TAG_WIDTH + INDEX_WIDTH + OFFSET_WIDTH + DATAPATH_DATA_WIDTH + DATAPATH_DATA_WIDTH_BYTE;
    logic [DP_REQ_WIDTH - 1 : 0] dp_req_in, dp_req_out;
    logic dp_req_valid_in, dp_req_wait_in;
    logic dp_req_valid_out, dp_req_wait_out;
    logic dp_req_cond;
    hs_cb #(
        .WIDTH(DP_REQ_WIDTH)
    ) dp_req (
        .clk(clk),
        .rstn(rstn),
        .in(dp_req_in),
        .valid_in(dp_req_valid_in),
        .wait_in(dp_req_wait_in),
        .out(dp_req_out),
        .valid_out(dp_req_valid_out),
        .wait_out(dp_req_wait_out),
        .cond(dp_req_cond)
    );

    // wait fifo: request handler -> response handler
    // RW (0 = read, 1 = write) | hit | index | offset | data | byteenable | wb | wbtag
    localparam WAIT_FIFO_WIDTH = 1 + 1 + INDEX_WIDTH + OFFSET_WIDTH + DATAPATH_DATA_WIDTH + DATAPATH_DATA_WIDTH_BYTE + 1 + TAG_WIDTH;
    localparam WAIT_FIFO_DEPTH = 512;
    logic [WAIT_FIFO_WIDTH - 1 : 0] wait_fifo_in, wait_fifo_out;
    logic wait_fifo_valid_in, wait_fifo_wait_in, wait_fifo_valid_out, wait_fifo_wait_out;
    hs_cqc #(
        .WIDTH(WAIT_FIFO_WIDTH),
        .DEPTH(WAIT_FIFO_DEPTH)
    ) wait_fifo (
        .clk(clk),
        .rstn(rstn),
        .in(wait_fifo_in),
        .valid_in(wait_fifo_valid_in),
        .wait_in(wait_fifo_wait_in),
        .out(wait_fifo_out),
        .valid_out(wait_fifo_valid_out),
        .wait_out(wait_fifo_wait_out)
    );

    // memory read request
    // data format: tag | index
    // structure: req handler -> mem_read_req_cutter -> mem_read_req_blocker -> avm req
    localparam MEM_READ_REQ_WIDTH = TAG_WIDTH + INDEX_WIDTH;
    logic [MEM_READ_REQ_WIDTH - 1 : 0] mem_read_req_in, mem_read_req_out;
    logic mem_read_req_valid_in, mem_read_req_wait_in;
    logic mem_read_req_valid_out, mem_read_req_wait_out;
    logic mem_read_req_cond;
    hs_bc #(
        .WIDTH(MEM_READ_REQ_WIDTH)
    ) mem_read_req (
        .clk(clk),
        .rstn(rstn),
        .in(mem_read_req_in),
        .valid_in(mem_read_req_valid_in),
        .wait_in(mem_read_req_wait_in),
        .out(mem_read_req_out),
        .valid_out(mem_read_req_valid_out),
        .wait_out(mem_read_req_wait_out),
        .cond(mem_read_req_cond)
    );

    // memory write request
    // data format: tag | index | data | byteenable
    // structure: res handler -> mem_write_req_cutter -> avm req
    localparam MEM_WRITE_REQ_WIDTH = TAG_WIDTH + INDEX_WIDTH + DATA_WIDTH + DATA_WIDTH_BYTE;
    logic [MEM_WRITE_REQ_WIDTH - 1 : 0] mem_write_req_in, mem_write_req_out;
    logic mem_write_req_valid_in, mem_write_req_wait_in;
    logic mem_write_req_valid_out, mem_write_req_wait_out;
    hs_cutter #(
        .WIDTH(MEM_WRITE_REQ_WIDTH)
    ) mem_write_req (
        .clk(clk),
        .rstn(rstn),
        .in(mem_write_req_in),
        .valid_in(mem_write_req_valid_in),
        .wait_in(mem_write_req_wait_in),
        .out(mem_write_req_out),
        .valid_out(mem_write_req_valid_out),
        .wait_out(mem_write_req_wait_out)
    );

    // memory response fifo: memory -> repsonse handler
    // data
    localparam MEM_RES_FIFO_WIDTH = DATA_WIDTH;
    localparam MEM_RES_FIFO_DEPTH = MAX_AVM_READ_COUNT;
    logic [MEM_RES_FIFO_WIDTH - 1 : 0] mem_res_fifo_in, mem_res_fifo_out;
    logic mem_res_fifo_valid_in, mem_res_fifo_wait_in, mem_res_fifo_valid_out, mem_res_fifo_wait_out;
    hs_cqc #(
        .WIDTH(MEM_RES_FIFO_WIDTH),
        .DEPTH(MEM_RES_FIFO_DEPTH)
    ) mem_res_fifo (
        .clk(clk),
        .rstn(rstn),
        .in(mem_res_fifo_in),
        .valid_in(mem_res_fifo_valid_in),
        .wait_in(mem_res_fifo_wait_in),
        .out(mem_res_fifo_out),
        .valid_out(mem_res_fifo_valid_out),
        .wait_out(mem_res_fifo_wait_out)
    );

    // datapath response
    // data format: data
    // structure: res handler -> dp_res_cutter -> avs res
    localparam DP_RES_WIDTH = DATAPATH_DATA_WIDTH;
    logic [DP_RES_WIDTH - 1 : 0] dp_res_in, dp_res_out;
    logic dp_res_valid_in, dp_res_wait_in;
    logic dp_res_valid_out, dp_res_wait_out;
    hs_cutter #(
        .WIDTH(DP_RES_WIDTH)
    ) dp_res (
        .clk(clk),
        .rstn(rstn),
        .in(dp_res_in),
        .valid_in(dp_res_valid_in),
        .wait_in(dp_res_wait_in),
        .out(dp_res_out),
        .valid_out(dp_res_valid_out),
        .wait_out(dp_res_wait_out)
    );

    // on-chip ram for metadata
    // dirty | tag
    localparam META_RAM_WIDTH = 1 + TAG_WIDTH;

    logic [META_RAM_WIDTH - 1 : 0] meta_ram_input;
    logic [META_RAM_WIDTH - 1 : 0] meta_ram_output_dc;
    logic [INDEX_WIDTH - 1 : 0] meta_ram_rdaddr;
    logic [INDEX_WIDTH - 1 : 0] meta_ram_wraddr;
    logic meta_ram_wren;
    logic meta_ram_rdaddrstall;

    logic [META_RAM_WIDTH - 1 : 0] meta_ram_buf;
    logic meta_ram_bufsw;
    logic [META_RAM_WIDTH - 1 : 0] meta_ram_output;

    ram_sdp #(
        .WIDTH(META_RAM_WIDTH),
        .DEPTH(NUM_CACHE_ENTRY_)
    ) meta_ram (
        .clock(clk),
        .data(meta_ram_input),
        .rd_addressstall(meta_ram_rdaddrstall),
        .rdaddress(meta_ram_rdaddr),
        .wraddress(meta_ram_wraddr),
        .wren(meta_ram_wren),
        .q(meta_ram_output_dc)
    );

    always_comb begin
        meta_ram_output = meta_ram_bufsw ? meta_ram_buf : meta_ram_output_dc;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
        end else begin
            meta_ram_buf <= meta_ram_input;
            meta_ram_bufsw <= meta_ram_wren & (meta_ram_rdaddr == meta_ram_wraddr);
        end
    end

    // on-chip ram for data
    // dirty | data
    localparam DATA_RAM_WIDTH = DATA_WIDTH_BYTE + DATA_WIDTH;

    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_input;
    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_output_dc;
    logic [INDEX_WIDTH - 1 : 0] data_ram_rdaddr;
    logic [INDEX_WIDTH - 1 : 0] data_ram_wraddr;
    logic data_ram_wren;
    logic data_ram_rdaddrstall;

    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_buf;
    logic data_ram_bufsw;
    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_output;

    ram_sdp #(
        .WIDTH(DATA_RAM_WIDTH),
        .DEPTH(NUM_CACHE_ENTRY_)
    ) data_ram (
        .clock(clk),
        .data(data_ram_input),
        .rd_addressstall(data_ram_rdaddrstall),
        .rdaddress(data_ram_rdaddr),
        .wraddress(data_ram_wraddr),
        .wren(data_ram_wren),
        .q(data_ram_output_dc)
    );

    always_comb begin
        data_ram_output = data_ram_bufsw ? data_ram_buf : data_ram_output_dc;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
        end else begin
            data_ram_buf <= data_ram_input;
            data_ram_bufsw <= data_ram_wren & (data_ram_rdaddr == data_ram_wraddr);
        end
    end

    // reg for valid bit
    // on-chip ram DOES NOT have reset functionality, so valid bit should NOT be on on-chip ram to be reset.

    // interface
    logic meta_valid_wr;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_wraddr;
    logic meta_valid_wrdata;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_rdaddr;
    logic meta_valid_rddata;
    logic meta_valid_rdaddrstall;

    // states
    logic meta_valid [0 : NUM_CACHE_ENTRY_ - 1];
    logic meta_valid_wr_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_wraddr_reg;
    logic meta_valid_wrdata_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_rdaddr_reg;

    always_comb begin
        meta_valid_rddata = (meta_valid_wr_reg & (meta_valid_wraddr_reg == meta_valid_rdaddr_reg)) ? meta_valid_wrdata_reg : meta_valid[meta_valid_rdaddr_reg];
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            meta_valid <= '{NUM_CACHE_ENTRY_{1'b0}};
            meta_valid_wr_reg <= 0;
        end else begin
            meta_valid_wr_reg <= meta_valid_wr;
            meta_valid_wraddr_reg <= meta_valid_wraddr;
            meta_valid_wrdata_reg <= meta_valid_wrdata;
            if (meta_valid_wr_reg) begin
                meta_valid[meta_valid_wraddr_reg] <= meta_valid_wrdata_reg;
            end
            if (~meta_valid_rdaddrstall) begin
                meta_valid_rdaddr_reg <= meta_valid_rdaddr;
            end
        end
    end

    // reg for writeback bit
    // writeback bit is updated by both request handler and repsonse handler.
    // since request handler is already using two port of meta_ram, response handler cannot use it.
    // so writeback bit is managed separately on registers.

    // interface
    logic meta_wb_wr_a;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_wraddr_a;
    logic meta_wb_wrdata_a;
    logic meta_wb_wr_b;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_wraddr_b;
    logic meta_wb_wrdata_b;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_rdaddr_a;
    logic meta_wb_rddata_a;
    logic meta_wb_rdaddrstall_a;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_rdaddr_b;
    logic meta_wb_rddata_b;

    // states
    logic meta_wb [0 : NUM_CACHE_ENTRY_ - 1];
    logic meta_wb_wr_a_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_wraddr_a_reg;
    logic meta_wb_wrdata_a_reg;
    logic meta_wb_wr_b_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_wraddr_b_reg;
    logic meta_wb_wrdata_b_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_rdaddr_a_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_wb_rdaddr_b_reg;

    always_comb begin
        meta_wb_rddata_a = (meta_wb_wr_a_reg & (meta_wb_wraddr_a_reg == meta_wb_rdaddr_a_reg)) ? meta_wb_wrdata_a_reg : meta_wb[meta_wb_rdaddr_a_reg];
        meta_wb_rddata_b = meta_wb[meta_wb_rdaddr_b_reg];
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            meta_wb_wr_a_reg <= 0;
            meta_wb_wr_b_reg <= 0;
        end else begin
            meta_wb_wr_a_reg <= meta_wb_wr_a;
            meta_wb_wraddr_a_reg <= meta_wb_wraddr_a;
            meta_wb_wrdata_a_reg <= meta_wb_wrdata_a;
            meta_wb_wr_b_reg <= meta_wb_wr_b;
            meta_wb_wraddr_b_reg <= meta_wb_wraddr_b;
            meta_wb_wrdata_b_reg <= meta_wb_wrdata_b;
            if (meta_wb_wr_a_reg) begin
                meta_wb[meta_wb_wraddr_a_reg] <= meta_wb_wrdata_a_reg;
            end
            if (meta_wb_wr_b_reg) begin
                meta_wb[meta_wb_wraddr_b_reg] <= meta_wb_wrdata_b_reg;
            end
            if (~meta_wb_rdaddrstall_a) begin
                meta_wb_rdaddr_a_reg <= meta_wb_rdaddr_a;
            end
            meta_wb_rdaddr_b_reg <= meta_wb_rdaddr_b;
        end
    end

    // request handler

    // wires
    logic req_s0_stall;
    logic req_s0_rw;
    logic [TAG_WIDTH - 1 : 0] req_s0_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s0_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s0_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] req_s0_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_s0_byteenable;
    logic req_s1_stall;
    logic req_s1_cache_dirty;
    logic [TAG_WIDTH - 1 : 0] req_s1_cache_tag;
    logic req_s1_cache_valid;
    logic req_s1_cache_wb;
    logic req_s1_cache_wbpoll;
    logic req_s2_stall;
    logic req_s2_hit;

    // states
    logic req_s0_valid;
    logic [INDEX_CNT_WIDTH - 1 : 0] req_s0_index_cnt;
    logic req_s1_valid;
    logic req_s1_rw;
    logic [TAG_WIDTH - 1 : 0] req_s1_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s1_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s1_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] req_s1_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_s1_byteenable;
    logic req_s2_valid;
    logic req_s2_rw;
    logic [TAG_WIDTH - 1 : 0] req_s2_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s2_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s2_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] req_s2_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_s2_byteenable;
    logic req_s2_cache_dirty;
    logic [TAG_WIDTH - 1 : 0] req_s2_cache_tag;
    logic req_s2_cache_valid;
    logic req_s2_cache_wb;

    logic req_wb_cur_state;
    logic req_wb_next_state;

    // controls
    // dp_req_fifo: dp_req_fifo_deq
    // mem_read_req_fifo: mem_read_req_fifo_enq, mem_read_req_fifo_input
    // wait_fifo: wait_fifo_enq, wait_fifo_input
    // meta_ram: meta_ram_rdaddr, meta_ram_rdaddrstall, meta_ram_wren, meta_ram_wraddr, meta_ram_input
    // meta_valid: meta_valid_wr, meta_valid_wraddr, meta_valid_wrdata
    // meta_wb (port a): meta_wb_wr_a, meta_wb_wraddr_a, meta_wb_wrdata_a

    // pipeline stage start -> 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s0_valid <= 0;
            req_s0_index_cnt <= NUM_CACHE_ENTRY_ + 1;
        end else begin
            if (state != STATE_CLEANING) begin
                req_s0_valid <= (dp_req_valid_out & ~dp_req_wait_out) | req_s0_stall;
                if (~req_s0_stall) begin
                    {req_s0_rw, req_s0_tag, req_s0_index, req_s0_offset, req_s0_data, req_s0_byteenable} <= dp_req_out;
                end
                req_s0_index_cnt <= NUM_CACHE_ENTRY_ + 1;
            end else begin
                if (~req_s0_stall) begin
                    if (req_s0_index_cnt == NUM_CACHE_ENTRY_ + 1) begin
                        req_s0_index_cnt <= 0;
                        req_s0_valid <= 1;
                    end else if (req_s0_index_cnt != NUM_CACHE_ENTRY_) begin
                        req_s0_index_cnt <= req_s0_index_cnt + 1'b1;
                        req_s0_valid <= (req_s0_index_cnt != NUM_CACHE_ENTRY_ - 1);
                    end else begin
                        req_s0_valid <= 0;
                    end
                end
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        req_s0_stall = req_s0_valid & req_s1_stall;
        if (state != STATE_CLEANING) begin
            dp_req_wait_out = req_s0_stall;
            meta_ram_rdaddr = req_s0_index;
            meta_valid_rdaddr = req_s0_index;
        end else begin
            dp_req_wait_out = 1;
            meta_ram_rdaddr = req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
            meta_valid_rdaddr = req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
        end
        meta_wb_rdaddr_a = req_s0_index;
        meta_wb_rdaddr_b = req_s2_index;
    end

    // pipeline state 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s1_valid <= 0;
        end else begin
            req_s1_valid <= (req_s0_valid & ~req_s0_stall) | req_s1_stall;
            if (~req_s1_stall) begin
                req_s1_rw <= req_s0_rw;
                req_s1_tag <= req_s0_tag;
                req_s1_index <= state != STATE_CLEANING ? req_s0_index : req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
                req_s1_offset <= req_s0_offset;
                req_s1_data <= req_s0_data;
                req_s1_byteenable <= req_s0_byteenable;
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        req_s1_stall = req_s1_valid & req_s2_stall;
        meta_ram_rdaddrstall = req_s1_stall;
        meta_valid_rdaddrstall = req_s1_stall;
        meta_wb_rdaddrstall_a = req_s1_stall;

        {req_s1_cache_dirty, req_s1_cache_tag} = meta_ram_output;
        req_s1_cache_valid = meta_valid_rddata;
        req_s1_cache_wb = meta_wb_rddata_a;
        req_s1_cache_wbpoll = meta_wb_rddata_b;
        if (req_s2_valid & (req_s1_index == req_s2_index)) begin
            req_s1_cache_dirty = req_s2_cache_valid & req_s2_hit ? req_s2_cache_dirty | req_s2_rw : req_s2_rw;
            req_s1_cache_tag = req_s2_tag;
            req_s1_cache_valid = 1;
            if (~req_s2_cache_valid) begin
                req_s1_cache_wb = 0;
            end else if (req_s2_hit) begin
                req_s1_cache_wb = req_s2_cache_wb;
            end else begin
                req_s1_cache_wb = req_s2_cache_wb | req_s2_cache_dirty;
            end
        end
    end

    // pipeline state 1 -> 2
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s2_valid <= 0;
        end else begin
            req_s2_valid <= (req_s1_valid & ~req_s1_stall) | req_s2_stall;
            if (~req_s2_stall) begin
                req_s2_rw <= req_s1_rw;
                req_s2_tag <= req_s1_tag;
                req_s2_index <= req_s1_index;
                req_s2_offset <= req_s1_offset;
                req_s2_data <= req_s1_data;
                req_s2_byteenable <= req_s1_byteenable;
                req_s2_cache_dirty <= req_s1_cache_dirty;
                req_s2_cache_tag <= req_s1_cache_tag;
                req_s2_cache_valid <= req_s1_cache_valid;
            end
            if (req_wb_next_state == 0) begin
                if (~req_s2_stall) begin
                    req_s2_cache_wb <= req_s1_cache_wb;
                end
            end else begin
                req_s2_cache_wb <= req_wb_cur_state == 0 ? 1 : req_s1_cache_wbpoll;
            end
        end
    end

    // pipeline stage 2
    always_comb begin
        mem_read_req_valid_in = 0;
        mem_read_req_in = 'x;
        wait_fifo_valid_in = 0;
        wait_fifo_in = 1'bx;
        meta_ram_wren = 0;
        meta_ram_wraddr = 1'bx;
        meta_ram_input = 1'bx;
        meta_valid_wr = 0;
        meta_valid_wraddr = 1'bx;
        meta_valid_wrdata = 1'bx;
        meta_wb_wr_a = 0;
        meta_wb_wraddr_a = 1'bx;
        meta_wb_wrdata_a = 1'bx;
        req_wb_next_state = req_wb_cur_state;

        req_s2_hit = (req_s2_tag == req_s2_cache_tag);

        if (state != STATE_CLEANING) begin
            req_s2_stall = req_s2_valid & (wait_fifo_wait_in | (mem_read_req_wait_in & (~req_s2_cache_valid | ~req_s2_hit)) | (req_s2_cache_valid & ~req_s2_hit & req_s2_cache_wb));
            if (req_wb_cur_state == 0) begin
                if (req_s2_valid & (req_s2_cache_valid & ~req_s2_hit & req_s2_cache_wb)) begin
                    req_wb_next_state = 1;
                end
            end else begin
                if (~req_s2_cache_wb) begin
                    req_wb_next_state = 0;
                end
            end
            if (~req_s2_valid | req_s2_stall) begin
            end else if (~req_s2_cache_valid) begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {req_s2_rw, 1'b0, req_s2_index, req_s2_offset, req_s2_data, req_s2_byteenable, 1'b0, {TAG_WIDTH{1'bx}}};
                mem_read_req_valid_in = 1;
                mem_read_req_in = {req_s2_tag, req_s2_index};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_rw, req_s2_tag};
                meta_valid_wr = 1;
                meta_valid_wraddr = req_s2_index;
                meta_valid_wrdata = 1;
                meta_wb_wr_a = 1;
                meta_wb_wraddr_a = req_s2_index;
                meta_wb_wrdata_a = 0;
            end else if (req_s2_hit) begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {req_s2_rw, 1'b1, req_s2_index, req_s2_offset, req_s2_data, req_s2_byteenable, 1'b0, {TAG_WIDTH{1'bx}}};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_cache_dirty | req_s2_rw, req_s2_tag};
            end else if (~req_s2_cache_wb) begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {req_s2_rw, 1'b0, req_s2_index, req_s2_offset, req_s2_data, req_s2_byteenable, req_s2_cache_dirty, req_s2_cache_tag};
                mem_read_req_valid_in = 1;
                mem_read_req_in = {req_s2_tag, req_s2_index};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_rw, req_s2_tag};
                meta_wb_wr_a = req_s2_cache_dirty;
                meta_wb_wraddr_a = req_s2_index;
                meta_wb_wrdata_a = 1;
            end else begin
            end
        end else begin
            req_s2_stall = req_s2_valid & (wait_fifo_wait_in & (req_s2_cache_valid & req_s2_cache_dirty));
            if (~req_s2_valid | req_s2_stall) begin
            end else if (req_s2_cache_valid) begin
                meta_valid_wr = 1;
                meta_valid_wraddr = req_s2_index;
                meta_valid_wrdata = 0;
                if (req_s2_cache_dirty) begin
                    wait_fifo_valid_in = 1;
                    wait_fifo_in = {1'bx, 1'bx, req_s2_index, {OFFSET_WIDTH{1'bx}}, {DATAPATH_DATA_WIDTH{1'bx}}, {DATAPATH_DATA_WIDTH_BYTE{1'bx}}, 1'b1, req_s2_cache_tag};
                end
            end else begin
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_wb_cur_state <= 0;
        end else begin
            req_wb_cur_state <= req_wb_next_state;
        end
    end

    // response handler variables

    // wires
    logic res_s0_stall;
    logic res_s0_rw;
    logic res_s0_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s0_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s0_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] res_s0_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] res_s0_byteenable;
    logic res_s0_wb;
    logic [TAG_WIDTH - 1 : 0] res_s0_wbtag;
    logic res_s1_stall;
    logic [DATA_WIDTH_BYTE - 1 : 0] res_s1_cache_dirty;
    logic [DATA_WIDTH - 1 : 0] res_s1_cache_data;
    logic [DATA_WIDTH - 1 : 0] res_s1_mem_data;
    logic [DATA_WIDTH - 1 : 0] res_s1_old_data;
    logic res_s2_stall;
    logic [DATA_WIDTH_BYTE - 1 : 0] res_s2_new_dirty;
    logic [DATA_WIDTH - 1 : 0] res_s2_new_data;

    // states
    logic res_s0_valid;
    logic res_s1_valid;
    logic res_s1_rw;
    logic res_s1_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s1_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s1_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] res_s1_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] res_s1_byteenable;
    logic res_s1_wb;
    logic [TAG_WIDTH - 1 : 0] res_s1_wbtag;
    logic res_s2_valid;
    logic res_s2_rw;
    logic res_s2_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s2_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s2_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] res_s2_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] res_s2_byteenable;
    logic res_s2_wb;
    logic [TAG_WIDTH - 1 : 0] res_s2_wbtag;
    logic [DATA_WIDTH_BYTE - 1 : 0] res_s2_cache_dirty;
    logic [DATA_WIDTH - 1 : 0] res_s2_old_data;

    // writeback handler variables

    // wires
    logic wb_s0_stall;
    logic wb_s1_stall;

    // states
    logic wb_ext_valid;
    logic wb_ext_old;
    logic wb_s0_valid;
    logic wb_s0_buf_valid;
    logic [TAG_WIDTH - 1 : 0] wb_s0_buf_wbtag;
    logic [INDEX_WIDTH - 1 : 0] wb_s0_buf_index;
    logic [DATA_WIDTH - 1 : 0] wb_s0_buf_cache_data;
    logic [DATA_WIDTH_BYTE - 1 : 0] wb_s0_buf_cache_dirty;
    logic wb_s1_valid;
    logic [TAG_WIDTH - 1 : 0] wb_s1_wbtag;
    logic [INDEX_WIDTH - 1 : 0] wb_s1_index;
    logic [DATA_WIDTH - 1 : 0] wb_s1_cache_data;
    logic [DATA_WIDTH_BYTE - 1 : 0] wb_s1_cache_dirty;

    // response handler

    // controls
    // wait_fifo: wait_fifo_deq
    // data_ram: data_ram_rdaddr, data_ram_rdaddrstall, data_ram_wren, data_ram_wraddr, data_ram_input
    // mem_write_req_fifo: mem_write_req_fifo_enq, mem_write_req_fifo_input
    // mem_res_fifo: mem_res_fifo_deq
    // dp_res_fifo: dp_res_fifo_enq, dp_res_fifo_input
    // meta_wb (port b): meta_wb_wr_b, meta_wb_wraddr_b, meta_wb_wrdata_b

    // pipeline stage external -> 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s0_valid <= 0;
        end else begin
            res_s0_valid <= (wait_fifo_valid_out & ~wait_fifo_wait_out) | res_s0_stall;
            if (~res_s0_stall) begin
                {res_s0_rw, res_s0_hit, res_s0_index, res_s0_offset, res_s0_data, res_s0_byteenable, res_s0_wb, res_s0_wbtag} <= wait_fifo_out;
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        data_ram_rdaddr = res_s0_index;
        if (state != STATE_CLEANING) begin
            res_s0_stall = res_s0_valid & (res_s1_stall | (~res_s0_hit & ~mem_res_fifo_valid_out) | (wb_s0_stall & res_s0_wb & ~wb_ext_old));
            wait_fifo_wait_out = res_s0_stall;
            if (~res_s0_valid | res_s0_stall) begin
                mem_res_fifo_wait_out = 1;
            end else if (res_s0_hit) begin
                mem_res_fifo_wait_out = 1;
            end else begin
                if (~mem_res_fifo_valid_out) begin
                    mem_res_fifo_wait_out = 1;
                end else begin
                    mem_res_fifo_wait_out = 0;
                end
            end
        end else begin
            res_s0_stall = res_s0_valid & (res_s1_stall | (wb_s0_stall & res_s0_wb & ~wb_ext_old));
            wait_fifo_wait_out = res_s0_stall;
            mem_res_fifo_wait_out = 1;
        end
    end

    // pipeline stage 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s1_valid <= 0;
        end else begin
            res_s1_valid <= (res_s0_valid & ~res_s0_stall) | res_s1_stall;
            if (~res_s1_stall) begin
                {res_s1_rw, res_s1_hit, res_s1_index, res_s1_offset, res_s1_data, res_s1_byteenable, res_s1_wb, res_s1_wbtag} <= {res_s0_rw, res_s0_hit, res_s0_index, res_s0_offset, res_s0_data, res_s0_byteenable, res_s0_wb, res_s0_wbtag};
                res_s1_mem_data = mem_res_fifo_out;
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        {res_s1_cache_dirty, res_s1_cache_data} = data_ram_output;
        res_s1_stall = res_s1_valid & res_s2_stall;
        data_ram_rdaddrstall = res_s1_stall;

        if (res_s2_valid & res_s1_index == res_s2_index) begin
            if (res_s2_rw | ~res_s2_hit) begin
                {res_s1_cache_dirty, res_s1_cache_data} = data_ram_input;
            end
        end

        res_s1_old_data = res_s1_hit ? res_s1_cache_data : res_s1_mem_data;
    end

    // pipeline stage 1 -> 2
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s2_valid <= 0;
        end else begin
            res_s2_valid <= (res_s1_valid & ~res_s1_stall) | res_s2_stall;
            if (~res_s2_stall) begin
                {res_s2_rw, res_s2_hit, res_s2_index, res_s2_offset, res_s2_data, res_s2_byteenable, res_s2_wb, res_s2_wbtag} <= {res_s1_rw, res_s1_hit, res_s1_index, res_s1_offset, res_s1_data, res_s1_byteenable, res_s1_wb, res_s1_wbtag};
                res_s2_cache_dirty <= res_s1_cache_dirty;
                res_s2_old_data <= res_s1_old_data;
            end
        end
    end

    // pipeline stage 2
    always_comb begin
        logic [DATAPATH_DATA_WIDTH - 1 : 0] data;
        logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] dirty;
        logic [OFFSET_WIDTH - 1 : 0] offset;
        offset = OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH
               ? 0
               : {res_s2_offset[OFFSET_WIDTH - 1 : DATAPATH_OFFSET_WIDTH], {DATAPATH_OFFSET_WIDTH{1'b0}}};
        for (int i = 0; i < DATAPATH_DATA_WIDTH_BYTE; ++i) begin
            data[i * 8 +: 8] = res_s2_rw & res_s2_byteenable[i] ? res_s2_data[i * 8 +: 8] : res_s2_old_data[(offset + i) * 8 +: 8];
            dirty[i] = (res_s2_hit ? res_s2_cache_dirty[offset + i] : 0) | (res_s2_rw ? res_s2_byteenable[i] : 0);
        end

        res_s2_new_data = (res_s2_old_data & ~({DATAPATH_DATA_WIDTH{1'b1}} << offset * 8))
                        | (data << offset * 8);
        res_s2_new_dirty = ((res_s2_hit ? res_s2_cache_dirty : 0) & ~({DATAPATH_DATA_WIDTH_BYTE{1'b1}} << offset))
                         | (dirty << offset);

        data_ram_wren = 0;
        data_ram_wraddr = 'x;
        data_ram_input = 'x;
        dp_res_valid_in = 0;
        dp_res_in = 'x;

        if (state != STATE_CLEANING) begin
            res_s2_stall = res_s2_valid & (dp_res_wait_in & ~res_s2_rw);
            if (~res_s2_valid | res_s2_stall) begin
            end else if (res_s2_rw) begin
                if (res_s2_hit) begin
                    data_ram_wren = 1;
                    data_ram_wraddr = res_s2_index;
                    data_ram_input = {res_s2_new_dirty, res_s2_new_data};
                end else begin
                    data_ram_wren = 1;
                    data_ram_wraddr = res_s2_index;
                    data_ram_input = {res_s2_new_dirty, res_s2_new_data};
                end
            end else begin
                if (res_s2_hit) begin
                    dp_res_valid_in = 1;
                    dp_res_in = data;
                end else begin
                    dp_res_valid_in = 1;
                    dp_res_in = data;
                    data_ram_wren = 1;
                    data_ram_wraddr = res_s2_index;
                    data_ram_input = {res_s2_new_dirty, res_s2_new_data};
                end
            end
        end else begin
            res_s2_stall = res_s2_valid & 0;
        end
    end

    // writeback handler

    // pipeline ext -> stage 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            wb_ext_old <= 0;
            wb_s0_valid <= 0;
        end else begin
            wb_s0_valid <= wb_ext_valid | wb_s0_stall;
            if (wb_ext_valid & ~wb_s0_stall) begin
                wb_ext_old <= 1;
            end
            if (~res_s0_stall) begin
                wb_ext_old <= 0;
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        wb_ext_valid = res_s0_valid & res_s0_wb & ~res_s1_stall & ~wb_ext_old;
        wb_s0_stall = wb_s0_valid & wb_s1_stall;
    end

    // pipeline stage 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            wb_s1_valid <= 0;
            wb_s0_buf_valid <= 0;
        end else begin
            wb_s1_valid <= (wb_s0_valid & ~wb_s0_stall) | wb_s1_stall;
            if (wb_s0_stall & ~wb_s0_buf_valid) begin
                wb_s0_buf_valid <= 1;
                {wb_s0_buf_wbtag, wb_s0_buf_index, wb_s0_buf_cache_data, wb_s0_buf_cache_dirty} <= {res_s1_wbtag, res_s1_index, res_s1_cache_data, res_s1_cache_dirty};
            end
            if (~wb_s1_stall) begin
                if (wb_s0_buf_valid) begin
                    wb_s0_buf_valid <= 0;
                    {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty} <= {wb_s0_buf_wbtag, wb_s0_buf_index, wb_s0_buf_cache_data, wb_s0_buf_cache_dirty};
                end else begin
                    {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty} <= {res_s1_wbtag, res_s1_index, res_s1_cache_data, res_s1_cache_dirty};
                end
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        mem_write_req_valid_in = 0;
        mem_write_req_in = 'x;
        meta_wb_wr_b = 0;
        meta_wb_wraddr_b = 'x;
        meta_wb_wrdata_b = 'x;
        if (state != STATE_CLEANING) begin
            wb_s1_stall = wb_s1_valid & (mem_write_req_wait_in);
            if (~wb_s1_valid | wb_s1_stall) begin
            end else begin
                mem_write_req_valid_in = 1;
                mem_write_req_in = {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty};
                meta_wb_wr_b = 1;
                meta_wb_wraddr_b = wb_s1_index;
                meta_wb_wrdata_b = 0;
            end
        end else begin
            wb_s1_stall = wb_s1_valid & (mem_write_req_wait_in);
            if (~wb_s1_valid | wb_s1_stall) begin
            end else begin
                mem_write_req_valid_in = 1;
                mem_write_req_in = {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty};
            end
        end
    end

    // datapath request
    always_comb begin
        dp_req_in = {avs_write, avs_address, avs_writedata, avs_byteenable};
        dp_req_valid_in = avs_read | avs_write;
        avs_waitrequest = dp_req_wait_in;
        dp_req_cond = (avs_write_count & MAX_AVS_WRITE_COUNT) != 0;
    end

    // memory read request
    always_comb begin
        mem_read_req_cond = (avm_read_count & MAX_AVM_READ_COUNT) != 0;
    end

    // mem_read_req, mem_write_req -> avm
    // priority on write
    logic [TAG_WIDTH - 1 : 0] mem_read_req_out_tag;
    logic [INDEX_WIDTH - 1 : 0] mem_read_req_out_index;
    logic [TAG_WIDTH - 1 : 0] mem_write_req_out_tag;
    logic [INDEX_WIDTH - 1 : 0] mem_write_req_out_index;
    logic [DATA_WIDTH - 1 : 0] mem_write_req_out_data;
    logic [DATA_WIDTH_BYTE - 1 : 0] mem_write_req_out_byteenable;
    always_comb begin
        {mem_read_req_out_tag, mem_read_req_out_index} = mem_read_req_out;
        {mem_write_req_out_tag, mem_write_req_out_index, mem_write_req_out_data, mem_write_req_out_byteenable} = mem_write_req_out;
    end

    always_comb begin
        avm_write = 0;
        avm_read = 0;
        avm_address = 'x;
        avm_writedata = 'x;
        avm_byteenable = 'x;
        avm_burstcount = 1;
        mem_read_req_wait_out = 1;
        mem_write_req_wait_out = 1;
        if (mem_write_req_valid_out) begin
            avm_write = 1;
            avm_address = {mem_write_req_out_tag, mem_write_req_out_index, {OFFSET_WIDTH{1'b0}}};
            avm_writedata = mem_write_req_out_data;
            avm_byteenable = mem_write_req_out_byteenable;
            mem_write_req_wait_out = avm_waitrequest;
        end else if (mem_read_req_valid_out) begin
            avm_read = 1;
            avm_address = {mem_read_req_out_tag, mem_read_req_out_index, {OFFSET_WIDTH{1'b0}}};
            avm_byteenable = {DATA_WIDTH_BYTE{1'b1}};
            mem_read_req_wait_out = avm_waitrequest;
        end
    end

    // memory <-> mem_res_fifo
    // Here, we do not check mem_res_fifo_full, since mem does not have backpressure signal.
    always_comb begin
        mem_res_fifo_valid_in = avm_readdatavalid;
        mem_res_fifo_in = avm_readdata;
    end

    // datapath response
    always_comb begin
        avs_readdata = dp_res_out;
        avs_readdatavalid = dp_res_valid_out;
        dp_res_wait_out = avs_waitresponse;
    end

    // counter
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            avs_write_count <= 0;
            avm_read_count <= 0;
            clean_count <= 0;
            wb_count <= 0;
        end else begin
            if (state != STATE_CLEANING) begin
                logic avs_write_enq, avs_write_deq;
                logic avm_read_enq, avm_read_deq;
                logic wb_enq, wb_deq;
                avs_write_enq = dp_req_valid_out & ~dp_req_wait_out & dp_req_out[DP_REQ_WIDTH - 1];
                avs_write_deq = res_s2_valid & ~res_s2_stall & res_s2_rw;
                avm_read_enq = mem_read_req_valid_in & ~mem_read_req_wait_in;
                avm_read_deq = mem_res_fifo_valid_out & ~mem_res_fifo_wait_out;
                wb_enq = wb_ext_valid & ~wb_s0_stall;
                wb_deq = avm_write & ~avm_waitrequest;
                if (avs_write_enq & ~avs_write_deq) avs_write_count <= avs_write_count + 1'b1;
                if (~avs_write_enq & avs_write_deq) avs_write_count <= avs_write_count - 1'b1;
                if (avm_read_enq & ~avm_read_deq) avm_read_count <= avm_read_count + 1'b1;
                if (~avm_read_enq & avm_read_deq) avm_read_count <= avm_read_count - 1'b1;
                if (wb_enq & ~wb_deq) wb_count <= wb_count + 1'b1;
                if (~wb_enq & wb_deq) wb_count <= wb_count - 1'b1;
            end else begin
                logic clean_enq, clean_deq;
                clean_enq = wait_fifo_valid_in & ~wait_fifo_wait_in;
                clean_deq = avm_write & ~avm_waitrequest;
                if (clean_enq & ~clean_deq) clean_count <= clean_count + 1'b1;
                if (~clean_enq & clean_deq) clean_count <= clean_count - 1'b1;
            end
        end
    end

    // cache clean
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            state <= STATE_NORMAL;
            cache_cleaned <= 0;
        end else begin
            if (state == STATE_NORMAL) begin
                if (cache_clean) begin
                    state <= STATE_CLEAN_PENDING;
                end
                cache_cleaned <= 0;
            end else if (state == STATE_CLEAN_PENDING) begin
                // read transactions have responses, so cache_clean is asserted only when all reads are done
                // for write transactions, however, we should check
                if (avs_write_count == 0 & wb_count == 0) begin
                    state <= STATE_CLEANING;
                end
            end else if (state == STATE_CLEANING) begin
                if (req_s0_index_cnt == NUM_CACHE_ENTRY_ & ~req_s0_valid & ~req_s1_valid & ~req_s2_valid & clean_count == 0) begin
                    state <= STATE_NORMAL;
                    cache_cleaned <= 1;
                end
            end
        end
    end

    // performance counter
    if (~ENABLE_PERFORMANCE_COUNTER) begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end else begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end
        end
    end else begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end else begin
                pc_cache_hit <= req_s2_hit & req_s2_valid & ~req_s2_stall;
                pc_cache_miss <= ~req_s2_hit & req_s2_valid & ~req_s2_stall;
            end
        end
    end

endmodule // global_mem_cache

module global_mem_cache_ro #(
    parameter ADDR_WIDTH = 36,
    parameter DATA_WIDTH = 512,
    parameter DATAPATH_DATA_WIDTH = 32,
    parameter NUM_CACHE_ENTRY = 1024,
    parameter DISTRIBUTION_FACTOR = 1,
    parameter BURST_WIDTH = 4,
    parameter USE_BURST = 1,
    parameter ENABLE_PERFORMANCE_COUNTER = 0
) (
    input logic clk,
    input logic rstn,
    input logic cache_clean,
    output logic cache_cleaned,

    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_read,
    output logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_readdata,
    output logic avs_readdatavalid,
    output logic avs_waitrequest,
    input logic avs_waitresponse,

    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid,
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest,
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount,
    output logic pc_cache_hit,
    output logic pc_cache_miss
);

    localparam NUM_CACHE_ENTRY_ = NUM_CACHE_ENTRY / DISTRIBUTION_FACTOR;
    localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
    localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
    localparam INDEX_WIDTH = $clog2(NUM_CACHE_ENTRY_);
    localparam TAG_WIDTH = ADDR_WIDTH - INDEX_WIDTH - OFFSET_WIDTH;
    localparam DATAPATH_DATA_WIDTH_BYTE = DATAPATH_DATA_WIDTH / 8;
    localparam DATAPATH_OFFSET_WIDTH = $clog2(DATAPATH_DATA_WIDTH_BYTE);

    // when cleaning, we scan through index.
    // NUM_CACHE_ENTRY_ + 1 = start
    // 0 ~ NUM_CACHE_ENTRY_ - 1 = counting
    // NUM_CACHE_ENTRY_ = end
    // [0, NUM_CACHE_ENTRY_ + 2)
    localparam INDEX_CNT_WIDTH = $clog2(NUM_CACHE_ENTRY_ + 2);

    // STATE_CLEAN_PENDING: Clean signal is received, but there are pending requests.
    // Do not accept further requests, but should process all requests before going to STATE_CLEANING.
    localparam STATE_NORMAL = 0;
    localparam STATE_CLEAN_PENDING = 1;
    localparam STATE_CLEANING = 2;
    logic [1:0] state;

    // COUNTERS
    localparam MAX_AVM_READ_COUNT = 512;
    localparam MAX_CLEAN_COUNT = NUM_CACHE_ENTRY_;
    logic [$clog2(MAX_AVM_READ_COUNT + 1) - 1 : 0] avm_read_count;
    logic [$clog2(MAX_CLEAN_COUNT + 1) - 1 : 0] clean_count;

    // datapath request
    // data format: TAG | INDEX | OFFSET
    // structure: avs req -> dp_req_blocker -> dp_req_cutter -> req handler
    localparam DP_REQ_WIDTH = TAG_WIDTH + INDEX_WIDTH + OFFSET_WIDTH;
    logic [DP_REQ_WIDTH - 1 : 0] dp_req_in, dp_req_out;
    logic dp_req_valid_in, dp_req_wait_in;
    logic dp_req_valid_out, dp_req_wait_out;
    logic dp_req_cond;
    hs_cb #(
        .WIDTH(DP_REQ_WIDTH)
    ) dp_req (
        .clk(clk),
        .rstn(rstn),
        .in(dp_req_in),
        .valid_in(dp_req_valid_in),
        .wait_in(dp_req_wait_in),
        .out(dp_req_out),
        .valid_out(dp_req_valid_out),
        .wait_out(dp_req_wait_out),
        .cond(dp_req_cond)
    );

    // wait fifo: request handler -> response handler
    // hit | index | offset
    localparam WAIT_FIFO_WIDTH = 1 + INDEX_WIDTH + OFFSET_WIDTH;
    localparam WAIT_FIFO_DEPTH = 512;
    logic [WAIT_FIFO_WIDTH - 1 : 0] wait_fifo_in, wait_fifo_out;
    logic wait_fifo_valid_in, wait_fifo_wait_in, wait_fifo_valid_out, wait_fifo_wait_out;
    hs_cqc #(
        .WIDTH(WAIT_FIFO_WIDTH),
        .DEPTH(WAIT_FIFO_DEPTH)
    ) wait_fifo (
        .clk(clk),
        .rstn(rstn),
        .in(wait_fifo_in),
        .valid_in(wait_fifo_valid_in),
        .wait_in(wait_fifo_wait_in),
        .out(wait_fifo_out),
        .valid_out(wait_fifo_valid_out),
        .wait_out(wait_fifo_wait_out)
    );

    // memory read request
    // data format: tag | index
    // structure: req handler -> mem_read_req_cutter -> mem_read_req_blocker -> avm req
    localparam MEM_READ_REQ_WIDTH = TAG_WIDTH + INDEX_WIDTH;
    logic [MEM_READ_REQ_WIDTH - 1 : 0] mem_read_req_in, mem_read_req_out;
    logic mem_read_req_valid_in, mem_read_req_wait_in;
    logic mem_read_req_valid_out, mem_read_req_wait_out;
    logic mem_read_req_cond;
    hs_bc #(
        .WIDTH(MEM_READ_REQ_WIDTH)
    ) mem_read_req (
        .clk(clk),
        .rstn(rstn),
        .in(mem_read_req_in),
        .valid_in(mem_read_req_valid_in),
        .wait_in(mem_read_req_wait_in),
        .out(mem_read_req_out),
        .valid_out(mem_read_req_valid_out),
        .wait_out(mem_read_req_wait_out),
        .cond(mem_read_req_cond)
    );

    // memory response fifo: memory -> repsonse handler
    // data
    localparam MEM_RES_FIFO_WIDTH = DATA_WIDTH;
    localparam MEM_RES_FIFO_DEPTH = MAX_AVM_READ_COUNT;
    logic [MEM_RES_FIFO_WIDTH - 1 : 0] mem_res_fifo_in, mem_res_fifo_out;
    logic mem_res_fifo_valid_in, mem_res_fifo_wait_in, mem_res_fifo_valid_out, mem_res_fifo_wait_out;
    hs_cqc #(
        .WIDTH(MEM_RES_FIFO_WIDTH),
        .DEPTH(MEM_RES_FIFO_DEPTH)
    ) mem_res_fifo (
        .clk(clk),
        .rstn(rstn),
        .in(mem_res_fifo_in),
        .valid_in(mem_res_fifo_valid_in),
        .wait_in(mem_res_fifo_wait_in),
        .out(mem_res_fifo_out),
        .valid_out(mem_res_fifo_valid_out),
        .wait_out(mem_res_fifo_wait_out)
    );

    // datapath response
    // data format: data
    // structure: res handler -> dp_res_cutter -> avs res
    localparam DP_RES_WIDTH = DATAPATH_DATA_WIDTH;
    logic [DP_RES_WIDTH - 1 : 0] dp_res_in, dp_res_out;
    logic dp_res_valid_in, dp_res_wait_in;
    logic dp_res_valid_out, dp_res_wait_out;
    hs_cutter #(
        .WIDTH(DP_RES_WIDTH)
    ) dp_res (
        .clk(clk),
        .rstn(rstn),
        .in(dp_res_in),
        .valid_in(dp_res_valid_in),
        .wait_in(dp_res_wait_in),
        .out(dp_res_out),
        .valid_out(dp_res_valid_out),
        .wait_out(dp_res_wait_out)
    );

    // on-chip ram for metadata
    // tag
    localparam META_RAM_WIDTH = TAG_WIDTH;

    logic [META_RAM_WIDTH - 1 : 0] meta_ram_input;
    logic [META_RAM_WIDTH - 1 : 0] meta_ram_output_dc;
    logic [INDEX_WIDTH - 1 : 0] meta_ram_rdaddr;
    logic [INDEX_WIDTH - 1 : 0] meta_ram_wraddr;
    logic meta_ram_wren;
    logic meta_ram_rdaddrstall;

    logic [META_RAM_WIDTH - 1 : 0] meta_ram_buf;
    logic meta_ram_bufsw;
    logic [META_RAM_WIDTH - 1 : 0] meta_ram_output;

    ram_sdp #(
        .WIDTH(META_RAM_WIDTH),
        .DEPTH(NUM_CACHE_ENTRY_)
    ) meta_ram (
        .clock(clk),
        .data(meta_ram_input),
        .rd_addressstall(meta_ram_rdaddrstall),
        .rdaddress(meta_ram_rdaddr),
        .wraddress(meta_ram_wraddr),
        .wren(meta_ram_wren),
        .q(meta_ram_output_dc)
    );

    always_comb begin
        meta_ram_output = meta_ram_bufsw ? meta_ram_buf : meta_ram_output_dc;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
        end else begin
            meta_ram_buf <= meta_ram_input;
            meta_ram_bufsw <= meta_ram_wren & (meta_ram_rdaddr == meta_ram_wraddr);
        end
    end

    // on-chip ram for data
    // data
    localparam DATA_RAM_WIDTH = DATA_WIDTH;

    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_input;
    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_output_dc;
    logic [INDEX_WIDTH - 1 : 0] data_ram_rdaddr;
    logic [INDEX_WIDTH - 1 : 0] data_ram_wraddr;
    logic data_ram_wren;
    logic data_ram_rdaddrstall;

    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_buf;
    logic data_ram_bufsw;
    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_output;

    ram_sdp #(
        .WIDTH(DATA_RAM_WIDTH),
        .DEPTH(NUM_CACHE_ENTRY_)
    ) data_ram (
        .clock(clk),
        .data(data_ram_input),
        .rd_addressstall(data_ram_rdaddrstall),
        .rdaddress(data_ram_rdaddr),
        .wraddress(data_ram_wraddr),
        .wren(data_ram_wren),
        .q(data_ram_output_dc)
    );

    always_comb begin
        data_ram_output = data_ram_bufsw ? data_ram_buf : data_ram_output_dc;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
        end else begin
            data_ram_buf <= data_ram_input;
            data_ram_bufsw <= data_ram_wren & (data_ram_rdaddr == data_ram_wraddr);
        end
    end

    // reg for valid bit
    // on-chip ram DOES NOT have reset functionality, so valid bit should NOT be on on-chip ram to be reset.

    // interface
    logic meta_valid_wr;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_wraddr;
    logic meta_valid_wrdata;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_rdaddr;
    logic meta_valid_rddata;
    logic meta_valid_rdaddrstall;

    // states
    logic meta_valid [0 : NUM_CACHE_ENTRY_ - 1];
    logic meta_valid_wr_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_wraddr_reg;
    logic meta_valid_wrdata_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_rdaddr_reg;

    always_comb begin
        meta_valid_rddata = (meta_valid_wr_reg & (meta_valid_wraddr_reg == meta_valid_rdaddr_reg)) ? meta_valid_wrdata_reg : meta_valid[meta_valid_rdaddr_reg];
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            meta_valid <= '{NUM_CACHE_ENTRY_{1'b0}};
            meta_valid_wr_reg <= 0;
        end else begin
            meta_valid_wr_reg <= meta_valid_wr;
            meta_valid_wraddr_reg <= meta_valid_wraddr;
            meta_valid_wrdata_reg <= meta_valid_wrdata;
            if (meta_valid_wr_reg) begin
                meta_valid[meta_valid_wraddr_reg] <= meta_valid_wrdata_reg;
            end
            if (~meta_valid_rdaddrstall) begin
                meta_valid_rdaddr_reg <= meta_valid_rdaddr;
            end
        end
    end

    // request handler

    // wires
    logic req_s0_stall;
    logic [TAG_WIDTH - 1 : 0] req_s0_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s0_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s0_offset;
    logic req_s1_stall;
    logic [TAG_WIDTH - 1 : 0] req_s1_cache_tag;
    logic req_s1_cache_valid;
    logic req_s2_stall;
    logic req_s2_hit;

    // states
    logic req_s0_valid;
    logic [INDEX_CNT_WIDTH - 1 : 0] req_s0_index_cnt;
    logic req_s1_valid;
    logic [TAG_WIDTH - 1 : 0] req_s1_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s1_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s1_offset;
    logic req_s2_valid;
    logic [TAG_WIDTH - 1 : 0] req_s2_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s2_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s2_offset;
    logic [TAG_WIDTH - 1 : 0] req_s2_cache_tag;
    logic req_s2_cache_valid;

    // controls
    // dp_req_fifo: dp_req_fifo_deq
    // mem_read_req_fifo: mem_read_req_fifo_enq, mem_read_req_fifo_input
    // wait_fifo: wait_fifo_enq, wait_fifo_input
    // meta_ram: meta_ram_rdaddr, meta_ram_rdaddrstall, meta_ram_wren, meta_ram_wraddr, meta_ram_input
    // meta_valid: meta_valid_wr, meta_valid_wraddr, meta_valid_wrdata
    // meta_wb (port a): meta_wb_wr_a, meta_wb_wraddr_a, meta_wb_wrdata_a

    // pipeline stage start -> 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s0_valid <= 0;
            req_s0_index_cnt <= NUM_CACHE_ENTRY_ + 1;
        end else begin
            if (state != STATE_CLEANING) begin
                req_s0_valid <= (dp_req_valid_out & ~dp_req_wait_out) | req_s0_stall;
                if (~req_s0_stall) begin
                    {req_s0_tag, req_s0_index, req_s0_offset} <= dp_req_out;
                end
                req_s0_index_cnt <= NUM_CACHE_ENTRY_ + 1;
            end else begin
                if (~req_s0_stall) begin
                    if (req_s0_index_cnt == NUM_CACHE_ENTRY_ + 1) begin
                        req_s0_index_cnt <= 0;
                        req_s0_valid <= 1;
                    end else if (req_s0_index_cnt != NUM_CACHE_ENTRY_) begin
                        req_s0_index_cnt <= req_s0_index_cnt + 1'b1;
                        req_s0_valid <= (req_s0_index_cnt != NUM_CACHE_ENTRY_ - 1);
                    end else begin
                        req_s0_valid <= 0;
                    end
                end
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        req_s0_stall = req_s0_valid & req_s1_stall;
        if (state != STATE_CLEANING) begin
            dp_req_wait_out = req_s0_stall;
            meta_ram_rdaddr = req_s0_index;
            meta_valid_rdaddr = req_s0_index;
        end else begin
            dp_req_wait_out = 1;
            meta_ram_rdaddr = req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
            meta_valid_rdaddr = req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
        end
    end

    // pipeline state 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s1_valid <= 0;
        end else begin
            req_s1_valid <= (req_s0_valid & ~req_s0_stall) | req_s1_stall;
            if (~req_s1_stall) begin
                req_s1_tag <= req_s0_tag;
                req_s1_index <= state != STATE_CLEANING ? req_s0_index : req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
                req_s1_offset <= req_s0_offset;
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        req_s1_stall = req_s1_valid & req_s2_stall;
        meta_ram_rdaddrstall = req_s1_stall;
        meta_valid_rdaddrstall = req_s1_stall;

        {req_s1_cache_tag} = meta_ram_output;
        req_s1_cache_valid = meta_valid_rddata;
        if (req_s2_valid & (req_s1_index == req_s2_index)) begin
            req_s1_cache_tag = req_s2_tag;
            req_s1_cache_valid = 1;
        end
    end

    // pipeline state 1 -> 2
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s2_valid <= 0;
        end else begin
            req_s2_valid <= (req_s1_valid & ~req_s1_stall) | req_s2_stall;
            if (~req_s2_stall) begin
                req_s2_tag <= req_s1_tag;
                req_s2_index <= req_s1_index;
                req_s2_offset <= req_s1_offset;
                req_s2_cache_tag <= req_s1_cache_tag;
                req_s2_cache_valid <= req_s1_cache_valid;
            end
        end
    end

    // pipeline stage 2
    always_comb begin
        mem_read_req_valid_in = 0;
        mem_read_req_in = 'x;
        wait_fifo_valid_in = 0;
        wait_fifo_in = 1'bx;
        meta_ram_wren = 0;
        meta_ram_wraddr = 1'bx;
        meta_ram_input = 1'bx;
        meta_valid_wr = 0;
        meta_valid_wraddr = 1'bx;
        meta_valid_wrdata = 1'bx;

        req_s2_hit = (req_s2_tag == req_s2_cache_tag);

        if (state != STATE_CLEANING) begin
            req_s2_stall = req_s2_valid & (wait_fifo_wait_in | (mem_read_req_wait_in & (~req_s2_cache_valid | ~req_s2_hit)));
            if (~req_s2_valid | req_s2_stall) begin
            end else if (~req_s2_cache_valid) begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {1'b0, req_s2_index, req_s2_offset};
                mem_read_req_valid_in = 1;
                mem_read_req_in = {req_s2_tag, req_s2_index};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_tag};
                meta_valid_wr = 1;
                meta_valid_wraddr = req_s2_index;
                meta_valid_wrdata = 1;
            end else if (req_s2_hit) begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {1'b1, req_s2_index, req_s2_offset};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_tag};
            end else begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {1'b0, req_s2_index, req_s2_offset};
                mem_read_req_valid_in = 1;
                mem_read_req_in = {req_s2_tag, req_s2_index};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_tag};
            end
        end else begin
            req_s2_stall = 0;
            if (~req_s2_valid | req_s2_stall) begin
            end else if (req_s2_cache_valid) begin
                meta_valid_wr = 1;
                meta_valid_wraddr = req_s2_index;
                meta_valid_wrdata = 0;
            end else begin
            end
        end
    end

    // response handler variables

    // wires
    logic res_s0_stall;
    logic res_s0_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s0_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s0_offset;
    logic res_s1_stall;
    logic [DATA_WIDTH - 1 : 0] res_s1_cache_data;
    logic [DATA_WIDTH - 1 : 0] res_s1_mem_data;
    logic [DATA_WIDTH - 1 : 0] res_s1_old_data;
    logic res_s2_stall;
    logic [DATA_WIDTH - 1 : 0] res_s2_new_data;

    // states
    logic res_s0_valid;
    logic res_s1_valid;
    logic res_s1_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s1_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s1_offset;
    logic res_s2_valid;
    logic res_s2_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s2_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s2_offset;
    logic [DATA_WIDTH - 1 : 0] res_s2_old_data;

    // response handler

    // controls
    // wait_fifo: wait_fifo_deq
    // data_ram: data_ram_rdaddr, data_ram_rdaddrstall, data_ram_wren, data_ram_wraddr, data_ram_input
    // mem_write_req_fifo: mem_write_req_fifo_enq, mem_write_req_fifo_input
    // mem_res_fifo: mem_res_fifo_deq
    // dp_res_fifo: dp_res_fifo_enq, dp_res_fifo_input
    // meta_wb (port b): meta_wb_wr_b, meta_wb_wraddr_b, meta_wb_wrdata_b

    // pipeline stage external -> 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s0_valid <= 0;
        end else begin
            res_s0_valid <= (wait_fifo_valid_out & ~wait_fifo_wait_out) | res_s0_stall;
            if (~res_s0_stall) begin
                {res_s0_hit, res_s0_index, res_s0_offset} <= wait_fifo_out;
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        data_ram_rdaddr = res_s0_index;
        if (state != STATE_CLEANING) begin
            res_s0_stall = res_s0_valid & (res_s1_stall | (~res_s0_hit & ~mem_res_fifo_valid_out));
            wait_fifo_wait_out = res_s0_stall;
            if (~res_s0_valid | res_s0_stall) begin
                mem_res_fifo_wait_out = 1;
            end else if (res_s0_hit) begin
                mem_res_fifo_wait_out = 1;
            end else begin
                if (~mem_res_fifo_valid_out) begin
                    mem_res_fifo_wait_out = 1;
                end else begin
                    mem_res_fifo_wait_out = 0;
                end
            end
        end else begin
            res_s0_stall = res_s0_valid & (res_s1_stall);
            wait_fifo_wait_out = res_s0_stall;
            mem_res_fifo_wait_out = 1;
        end
    end

    // pipeline stage 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s1_valid <= 0;
        end else begin
            res_s1_valid <= (res_s0_valid & ~res_s0_stall) | res_s1_stall;
            if (~res_s1_stall) begin
                {res_s1_hit, res_s1_index, res_s1_offset} <= {res_s0_hit, res_s0_index, res_s0_offset};
                res_s1_mem_data <= mem_res_fifo_out;
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        {res_s1_cache_data} = data_ram_output;
        res_s1_stall = res_s1_valid & res_s2_stall;
        data_ram_rdaddrstall = res_s1_stall;

        if (res_s2_valid & res_s1_index == res_s2_index) begin
            if (~res_s2_hit) begin
                {res_s1_cache_data} = data_ram_input;
            end
        end

        res_s1_old_data = res_s1_hit ? res_s1_cache_data : res_s1_mem_data;
    end

    // pipeline stage 1 -> 2
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s2_valid <= 0;
        end else begin
            res_s2_valid <= (res_s1_valid & ~res_s1_stall) | res_s2_stall;
            if (~res_s2_stall) begin
                {res_s2_hit, res_s2_index, res_s2_offset} <= {res_s1_hit, res_s1_index, res_s1_offset};
                res_s2_old_data <= res_s1_old_data;
            end
        end
    end

    // pipeline stage 2
    always_comb begin
        logic [DATAPATH_DATA_WIDTH - 1 : 0] data;
        logic [OFFSET_WIDTH - 1 : 0] offset;
        offset = OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH
               ? 0
               : {res_s2_offset[OFFSET_WIDTH - 1 : DATAPATH_OFFSET_WIDTH], {DATAPATH_OFFSET_WIDTH{1'b0}}};
        for (int i = 0; i < DATAPATH_DATA_WIDTH_BYTE; ++i) begin
            data[i * 8 +: 8] = res_s2_old_data[(offset + i) * 8 +: 8];
        end

        res_s2_new_data = (res_s2_old_data & ~({DATAPATH_DATA_WIDTH{1'b1}} << offset * 8))
                        | (data << offset * 8);

        data_ram_wren = 0;
        data_ram_wraddr = 'x;
        data_ram_input = 'x;
        dp_res_valid_in = 0;
        dp_res_in = 'x;

        if (state != STATE_CLEANING) begin
            res_s2_stall = res_s2_valid & (dp_res_wait_in);
            if (~res_s2_valid | res_s2_stall) begin
            end else begin
                if (res_s2_hit) begin
                    dp_res_valid_in = 1;
                    dp_res_in = data;
                end else begin
                    dp_res_valid_in = 1;
                    dp_res_in = data;
                    data_ram_wren = 1;
                    data_ram_wraddr = res_s2_index;
                    data_ram_input = {res_s2_new_data};
                end
            end
        end else begin
            res_s2_stall = res_s2_valid & 0;
        end
    end

    // datapath request
    always_comb begin
        dp_req_in = {avs_address};
        dp_req_valid_in = avs_read;
        avs_waitrequest = dp_req_wait_in;
        dp_req_cond = 0;
    end

    // memory read request
    always_comb begin
        mem_read_req_cond = (avm_read_count & MAX_AVM_READ_COUNT) != 0;
    end

    // mem_read_req -> avm
    logic [TAG_WIDTH - 1 : 0] mem_read_req_out_tag;
    logic [INDEX_WIDTH - 1 : 0] mem_read_req_out_index;
    always_comb begin
        {mem_read_req_out_tag, mem_read_req_out_index} = mem_read_req_out;
    end

    if (USE_BURST) begin
        logic [ADDR_WIDTH - 1 : 0] read_address;
        logic read;
        logic [BURST_WIDTH - 1 : 0] read_burstcount;
        logic read_waitrequest;

        amm_burstify_ro #(
            .ADDR_WIDTH(ADDR_WIDTH),
            .DATA_WIDTH(DATA_WIDTH),
            .BURST_WIDTH(BURST_WIDTH)
        ) amm_burstify_ro_inst (
            .clk(clk),
            .rstn(rstn),
            .avs_address({mem_read_req_out_tag, mem_read_req_out_index, {OFFSET_WIDTH{1'b0}}}),
            .avs_read(mem_read_req_valid_out),
            .avs_waitrequest(mem_read_req_wait_out),
            .avm_address(read_address),
            .avm_read(read),
            .avm_burstcount(read_burstcount),
            .avm_waitrequest(read_waitrequest)
        );

        always_comb begin
            avm_write = 0;
            avm_read = 0;
            avm_address = 'x;
            avm_writedata = 'x;
            avm_byteenable = 'x;
            avm_burstcount = 'x;
            read_waitrequest = 1;
            if (read) begin
                avm_read = 1;
                avm_address = read_address;
                avm_byteenable = {DATA_WIDTH_BYTE{1'b1}};
                avm_burstcount = read_burstcount;
                read_waitrequest = avm_waitrequest;
            end
        end
    end else begin
        always_comb begin
            avm_write = 0;
            avm_read = 0;
            avm_address = 'x;
            avm_writedata = 'x;
            avm_byteenable = 'x;
            avm_burstcount = 1;
            mem_read_req_wait_out = 1;
            if (mem_read_req_valid_out) begin
                avm_read = 1;
                avm_address = {mem_read_req_out_tag, mem_read_req_out_index, {OFFSET_WIDTH{1'b0}}};
                avm_byteenable = {DATA_WIDTH_BYTE{1'b1}};
                mem_read_req_wait_out = avm_waitrequest;
            end
        end
    end

    // memory <-> mem_res_fifo
    // Here, we do not check mem_res_fifo_full, since mem does not have backpressure signal.
    always_comb begin
        mem_res_fifo_valid_in = avm_readdatavalid;
        mem_res_fifo_in = avm_readdata;
    end

    // datapath response
    always_comb begin
        avs_readdata = dp_res_out;
        avs_readdatavalid = dp_res_valid_out;
        dp_res_wait_out = avs_waitresponse;
    end

    // counter
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            avm_read_count <= 0;
            clean_count <= 0;
        end else begin
            if (state != STATE_CLEANING) begin
                logic avm_read_enq, avm_read_deq;
                avm_read_enq = mem_read_req_valid_in & ~mem_read_req_wait_in;
                avm_read_deq = mem_res_fifo_valid_out & ~mem_res_fifo_wait_out;
                if (avm_read_enq & ~avm_read_deq) avm_read_count <= avm_read_count + 1'b1;
                if (~avm_read_enq & avm_read_deq) avm_read_count <= avm_read_count - 1'b1;
            end else begin
                logic clean_enq, clean_deq;
                clean_enq = wait_fifo_valid_in & ~wait_fifo_wait_in;
                clean_deq = avm_write & ~avm_waitrequest;
                if (clean_enq & ~clean_deq) clean_count <= clean_count + 1'b1;
                if (~clean_enq & clean_deq) clean_count <= clean_count - 1'b1;
            end
        end
    end

    // cache clean
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            state <= STATE_NORMAL;
            cache_cleaned <= 0;
        end else begin
            if (state == STATE_NORMAL) begin
                if (cache_clean) begin
                    state <= STATE_CLEAN_PENDING;
                end
                cache_cleaned <= 0;
            end else if (state == STATE_CLEAN_PENDING) begin
                state <= STATE_CLEANING;
            end else if (state == STATE_CLEANING) begin
                if (req_s0_index_cnt == NUM_CACHE_ENTRY_ & ~req_s0_valid & ~req_s1_valid & ~req_s2_valid & clean_count == 0) begin
                    state <= STATE_NORMAL;
                    cache_cleaned <= 1;
                end
            end
        end
    end

    // performance counter
    if (~ENABLE_PERFORMANCE_COUNTER) begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end else begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end
        end
    end else begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end else begin
                pc_cache_hit <= req_s2_hit & req_s2_valid & ~req_s2_stall;
                pc_cache_miss <= ~req_s2_hit & req_s2_valid & ~req_s2_stall;
            end
        end
    end

endmodule // global_mem_cache_ro

module global_mem_cache_wo #(
    parameter ADDR_WIDTH = 36,
    parameter DATA_WIDTH = 512,
    parameter DATAPATH_DATA_WIDTH = 32,
    parameter NUM_CACHE_ENTRY = 1024,
    parameter DISTRIBUTION_FACTOR = 1,
    parameter BURST_WIDTH = 4,
    parameter USE_BURST = 1,
    parameter ENABLE_PERFORMANCE_COUNTER = 0
) (
    input logic clk,
    input logic rstn,
    input logic cache_clean,
    output logic cache_cleaned,

    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_write,
    input logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_writedata,
    input logic [DATAPATH_DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
    output logic avs_waitrequest,

    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid,
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest,
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount,
    output logic pc_cache_hit,
    output logic pc_cache_miss
);

    localparam NUM_CACHE_ENTRY_ = NUM_CACHE_ENTRY / DISTRIBUTION_FACTOR;
    localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
    localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
    localparam INDEX_WIDTH = $clog2(NUM_CACHE_ENTRY_);
    localparam TAG_WIDTH = ADDR_WIDTH - INDEX_WIDTH - OFFSET_WIDTH;
    localparam DATAPATH_DATA_WIDTH_BYTE = DATAPATH_DATA_WIDTH / 8;
    localparam DATAPATH_OFFSET_WIDTH = $clog2(DATAPATH_DATA_WIDTH_BYTE);

    // when cleaning, we scan through index.
    // NUM_CACHE_ENTRY_ + 1 = start
    // 0 ~ NUM_CACHE_ENTRY_ - 1 = counting
    // NUM_CACHE_ENTRY_ = end
    // [0, NUM_CACHE_ENTRY_ + 2)
    localparam INDEX_CNT_WIDTH = $clog2(NUM_CACHE_ENTRY_ + 2);

    // STATE_CLEAN_PENDING: Clean signal is received, but there are pending requests.
    // Do not accept further requests, but should process all requests before going to STATE_CLEANING.
    localparam STATE_NORMAL = 0;
    localparam STATE_CLEAN_PENDING = 1;
    localparam STATE_CLEANING = 2;
    logic [1:0] state;

    // COUNTERS
    localparam MAX_AVS_WRITE_COUNT = 512;
    localparam MAX_CLEAN_COUNT = NUM_CACHE_ENTRY_;
    localparam MAX_WB_COUNT = NUM_CACHE_ENTRY_;
    logic [$clog2(MAX_AVS_WRITE_COUNT + 1) - 1 : 0] avs_write_count;
    logic [$clog2(MAX_CLEAN_COUNT + 1) - 1 : 0] clean_count;
    logic [$clog2(MAX_WB_COUNT + 1) - 1 : 0] wb_count;

    // datapath request
    // data format: TAG | INDEX | OFFSET | DATA | BYTEENABLE
    // structure: avs req -> dp_req_blocker -> dp_req_cutter -> req handler
    localparam DP_REQ_WIDTH = TAG_WIDTH + INDEX_WIDTH + OFFSET_WIDTH + DATAPATH_DATA_WIDTH + DATAPATH_DATA_WIDTH_BYTE;
    logic [DP_REQ_WIDTH - 1 : 0] dp_req_in, dp_req_out;
    logic dp_req_valid_in, dp_req_wait_in;
    logic dp_req_valid_out, dp_req_wait_out;
    logic dp_req_cond;
    hs_cb #(
        .WIDTH(DP_REQ_WIDTH)
    ) dp_req (
        .clk(clk),
        .rstn(rstn),
        .in(dp_req_in),
        .valid_in(dp_req_valid_in),
        .wait_in(dp_req_wait_in),
        .out(dp_req_out),
        .valid_out(dp_req_valid_out),
        .wait_out(dp_req_wait_out),
        .cond(dp_req_cond)
    );

    // wait fifo: request handler -> response handler
    // hit | index | offset | data | byteenable | wb | wbtag
    localparam WAIT_FIFO_WIDTH = 1 + INDEX_WIDTH + OFFSET_WIDTH + DATAPATH_DATA_WIDTH + DATAPATH_DATA_WIDTH_BYTE + 1 + TAG_WIDTH;
    logic [WAIT_FIFO_WIDTH - 1 : 0] wait_fifo_in, wait_fifo_out;
    logic wait_fifo_valid_in, wait_fifo_wait_in, wait_fifo_valid_out, wait_fifo_wait_out;
    hs_cutter #(
        .WIDTH(WAIT_FIFO_WIDTH)
    ) wait_fifo (
        .clk(clk),
        .rstn(rstn),
        .in(wait_fifo_in),
        .valid_in(wait_fifo_valid_in),
        .wait_in(wait_fifo_wait_in),
        .out(wait_fifo_out),
        .valid_out(wait_fifo_valid_out),
        .wait_out(wait_fifo_wait_out)
    );

    // memory write request
    // data format: tag | index | data | byteenable
    // structure: res handler -> mem_write_req_cutter -> avm req
    localparam MEM_WRITE_REQ_WIDTH = TAG_WIDTH + INDEX_WIDTH + DATA_WIDTH + DATA_WIDTH_BYTE;
    logic [MEM_WRITE_REQ_WIDTH - 1 : 0] mem_write_req_in, mem_write_req_out;
    logic mem_write_req_valid_in, mem_write_req_wait_in;
    logic mem_write_req_valid_out, mem_write_req_wait_out;
    hs_cutter #(
        .WIDTH(MEM_WRITE_REQ_WIDTH)
    ) mem_write_req (
        .clk(clk),
        .rstn(rstn),
        .in(mem_write_req_in),
        .valid_in(mem_write_req_valid_in),
        .wait_in(mem_write_req_wait_in),
        .out(mem_write_req_out),
        .valid_out(mem_write_req_valid_out),
        .wait_out(mem_write_req_wait_out)
    );

    // on-chip ram for metadata
    // tag
    localparam META_RAM_WIDTH = TAG_WIDTH;

    logic [META_RAM_WIDTH - 1 : 0] meta_ram_input;
    logic [META_RAM_WIDTH - 1 : 0] meta_ram_output_dc;
    logic [INDEX_WIDTH - 1 : 0] meta_ram_rdaddr;
    logic [INDEX_WIDTH - 1 : 0] meta_ram_wraddr;
    logic meta_ram_wren;
    logic meta_ram_rdaddrstall;

    logic [META_RAM_WIDTH - 1 : 0] meta_ram_buf;
    logic meta_ram_bufsw;
    logic [META_RAM_WIDTH - 1 : 0] meta_ram_output;

    ram_sdp #(
        .WIDTH(META_RAM_WIDTH),
        .DEPTH(NUM_CACHE_ENTRY_)
    ) meta_ram (
        .clock(clk),
        .data(meta_ram_input),
        .rd_addressstall(meta_ram_rdaddrstall),
        .rdaddress(meta_ram_rdaddr),
        .wraddress(meta_ram_wraddr),
        .wren(meta_ram_wren),
        .q(meta_ram_output_dc)
    );

    always_comb begin
        meta_ram_output = meta_ram_bufsw ? meta_ram_buf : meta_ram_output_dc;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
        end else begin
            meta_ram_buf <= meta_ram_input;
            meta_ram_bufsw <= meta_ram_wren & (meta_ram_rdaddr == meta_ram_wraddr);
        end
    end

    // on-chip ram for data
    // dirty | data
    localparam DATA_RAM_WIDTH = DATA_WIDTH_BYTE + DATA_WIDTH;

    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_input;
    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_output_dc;
    logic [INDEX_WIDTH - 1 : 0] data_ram_rdaddr;
    logic [INDEX_WIDTH - 1 : 0] data_ram_wraddr;
    logic data_ram_wren;
    logic data_ram_rdaddrstall;

    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_buf;
    logic data_ram_bufsw;
    logic [DATA_RAM_WIDTH - 1 : 0] data_ram_output;

    ram_sdp #(
        .WIDTH(DATA_RAM_WIDTH),
        .DEPTH(NUM_CACHE_ENTRY_)
    ) data_ram (
        .clock(clk),
        .data(data_ram_input),
        .rd_addressstall(data_ram_rdaddrstall),
        .rdaddress(data_ram_rdaddr),
        .wraddress(data_ram_wraddr),
        .wren(data_ram_wren),
        .q(data_ram_output_dc)
    );

    always_comb begin
        data_ram_output = data_ram_bufsw ? data_ram_buf : data_ram_output_dc;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
        end else begin
            data_ram_buf <= data_ram_input;
            data_ram_bufsw <= data_ram_wren & (data_ram_rdaddr == data_ram_wraddr);
        end
    end

    // reg for valid bit
    // on-chip ram DOES NOT have reset functionality, so valid bit should NOT be on on-chip ram to be reset.

    // interface
    logic meta_valid_wr;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_wraddr;
    logic meta_valid_wrdata;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_rdaddr;
    logic meta_valid_rddata;
    logic meta_valid_rdaddrstall;

    // states
    logic meta_valid [0 : NUM_CACHE_ENTRY_ - 1];
    logic meta_valid_wr_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_wraddr_reg;
    logic meta_valid_wrdata_reg;
    logic [INDEX_WIDTH - 1 : 0] meta_valid_rdaddr_reg;

    always_comb begin
        meta_valid_rddata = (meta_valid_wr_reg & (meta_valid_wraddr_reg == meta_valid_rdaddr_reg)) ? meta_valid_wrdata_reg : meta_valid[meta_valid_rdaddr_reg];
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            meta_valid <= '{NUM_CACHE_ENTRY_{1'b0}};
            meta_valid_wr_reg <= 0;
        end else begin
            meta_valid_wr_reg <= meta_valid_wr;
            meta_valid_wraddr_reg <= meta_valid_wraddr;
            meta_valid_wrdata_reg <= meta_valid_wrdata;
            if (meta_valid_wr_reg) begin
                meta_valid[meta_valid_wraddr_reg] <= meta_valid_wrdata_reg;
            end
            if (~meta_valid_rdaddrstall) begin
                meta_valid_rdaddr_reg <= meta_valid_rdaddr;
            end
        end
    end

    // request handler

    // wires
    logic req_s0_stall;
    logic [TAG_WIDTH - 1 : 0] req_s0_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s0_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s0_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] req_s0_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_s0_byteenable;
    logic req_s1_stall;
    logic [TAG_WIDTH - 1 : 0] req_s1_cache_tag;
    logic req_s1_cache_valid;
    logic req_s2_stall;
    logic req_s2_hit;

    // states
    logic req_s0_valid;
    logic [INDEX_CNT_WIDTH - 1 : 0] req_s0_index_cnt;
    logic req_s1_valid;
    logic [TAG_WIDTH - 1 : 0] req_s1_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s1_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s1_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] req_s1_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_s1_byteenable;
    logic req_s2_valid;
    logic [TAG_WIDTH - 1 : 0] req_s2_tag;
    logic [INDEX_WIDTH - 1 : 0] req_s2_index;
    logic [OFFSET_WIDTH - 1 : 0] req_s2_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] req_s2_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_s2_byteenable;
    logic [TAG_WIDTH - 1 : 0] req_s2_cache_tag;
    logic req_s2_cache_valid;

    // controls
    // dp_req_fifo: dp_req_fifo_deq
    // mem_read_req_fifo: mem_read_req_fifo_enq, mem_read_req_fifo_input
    // wait_fifo: wait_fifo_enq, wait_fifo_input
    // meta_ram: meta_ram_rdaddr, meta_ram_rdaddrstall, meta_ram_wren, meta_ram_wraddr, meta_ram_input
    // meta_valid: meta_valid_wr, meta_valid_wraddr, meta_valid_wrdata
    // meta_wb (port a): meta_wb_wr_a, meta_wb_wraddr_a, meta_wb_wrdata_a

    // pipeline stage start -> 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s0_valid <= 0;
            req_s0_index_cnt <= NUM_CACHE_ENTRY_ + 1;
        end else begin
            if (state != STATE_CLEANING) begin
                req_s0_valid <= (dp_req_valid_out & ~dp_req_wait_out) | req_s0_stall;
                if (~req_s0_stall) begin
                    {req_s0_tag, req_s0_index, req_s0_offset, req_s0_data, req_s0_byteenable} <= dp_req_out;
                end
                req_s0_index_cnt <= NUM_CACHE_ENTRY_ + 1;
            end else begin
                if (~req_s0_stall) begin
                    if (req_s0_index_cnt == NUM_CACHE_ENTRY_ + 1) begin
                        req_s0_index_cnt <= 0;
                        req_s0_valid <= 1;
                    end else if (req_s0_index_cnt != NUM_CACHE_ENTRY_) begin
                        req_s0_index_cnt <= req_s0_index_cnt + 1'b1;
                        req_s0_valid <= (req_s0_index_cnt != NUM_CACHE_ENTRY_ - 1);
                    end else begin
                        req_s0_valid <= 0;
                    end
                end
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        req_s0_stall = req_s0_valid & req_s1_stall;
        if (state != STATE_CLEANING) begin
            dp_req_wait_out = req_s0_stall;
            meta_ram_rdaddr = req_s0_index;
            meta_valid_rdaddr = req_s0_index;
        end else begin
            dp_req_wait_out = 1;
            meta_ram_rdaddr = req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
            meta_valid_rdaddr = req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
        end
    end

    // pipeline state 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s1_valid <= 0;
        end else begin
            req_s1_valid <= (req_s0_valid & ~req_s0_stall) | req_s1_stall;
            if (~req_s1_stall) begin
                req_s1_tag <= req_s0_tag;
                req_s1_index <= state != STATE_CLEANING ? req_s0_index : req_s0_index_cnt[INDEX_WIDTH - 1 : 0];
                req_s1_offset <= req_s0_offset;
                req_s1_data <= req_s0_data;
                req_s1_byteenable <= req_s0_byteenable;
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        req_s1_stall = req_s1_valid & req_s2_stall;
        meta_ram_rdaddrstall = req_s1_stall;
        meta_valid_rdaddrstall = req_s1_stall;

        {req_s1_cache_tag} = meta_ram_output;
        req_s1_cache_valid = meta_valid_rddata;
        if (req_s2_valid & (req_s1_index == req_s2_index)) begin
            req_s1_cache_tag = req_s2_tag;
            req_s1_cache_valid = 1;
        end
    end

    // pipeline state 1 -> 2
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_s2_valid <= 0;
        end else begin
            req_s2_valid <= (req_s1_valid & ~req_s1_stall) | req_s2_stall;
            if (~req_s2_stall) begin
                req_s2_tag <= req_s1_tag;
                req_s2_index <= req_s1_index;
                req_s2_offset <= req_s1_offset;
                req_s2_data <= req_s1_data;
                req_s2_byteenable <= req_s1_byteenable;
                req_s2_cache_tag <= req_s1_cache_tag;
                req_s2_cache_valid <= req_s1_cache_valid;
            end
        end
    end

    // pipeline stage 2
    always_comb begin
        wait_fifo_valid_in = 0;
        wait_fifo_in = 1'bx;
        meta_ram_wren = 0;
        meta_ram_wraddr = 1'bx;
        meta_ram_input = 1'bx;
        meta_valid_wr = 0;
        meta_valid_wraddr = 1'bx;
        meta_valid_wrdata = 1'bx;

        req_s2_hit = (req_s2_tag == req_s2_cache_tag);

        if (state != STATE_CLEANING) begin
            req_s2_stall = req_s2_valid & (wait_fifo_wait_in);
            if (~req_s2_valid | req_s2_stall) begin
            end else if (~req_s2_cache_valid) begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {1'b0, req_s2_index, req_s2_offset, req_s2_data, req_s2_byteenable, 1'b0, {TAG_WIDTH{1'bx}}};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_tag};
                meta_valid_wr = 1;
                meta_valid_wraddr = req_s2_index;
                meta_valid_wrdata = 1;
            end else if (req_s2_hit) begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {1'b1, req_s2_index, req_s2_offset, req_s2_data, req_s2_byteenable, 1'b0, {TAG_WIDTH{1'bx}}};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_tag};
            end else begin
                wait_fifo_valid_in = 1;
                wait_fifo_in = {1'b0, req_s2_index, req_s2_offset, req_s2_data, req_s2_byteenable, 1'b1, req_s2_cache_tag};
                meta_ram_wren = 1;
                meta_ram_wraddr = req_s2_index;
                meta_ram_input = {req_s2_tag};
            end
        end else begin
            req_s2_stall = req_s2_valid & (wait_fifo_wait_in & (req_s2_cache_valid));
            if (~req_s2_valid | req_s2_stall) begin
            end else if (req_s2_cache_valid) begin
                meta_valid_wr = 1;
                meta_valid_wraddr = req_s2_index;
                meta_valid_wrdata = 0;
                if (1) begin
                    wait_fifo_valid_in = 1;
                    wait_fifo_in = {1'bx, req_s2_index, {OFFSET_WIDTH{1'bx}}, {DATAPATH_DATA_WIDTH{1'bx}}, {DATAPATH_DATA_WIDTH_BYTE{1'bx}}, 1'b1, req_s2_cache_tag};
                end
            end else begin
            end
        end
    end

    // response handler variables

    // wires
    logic res_s0_stall;
    logic res_s0_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s0_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s0_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] res_s0_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] res_s0_byteenable;
    logic res_s0_wb;
    logic [TAG_WIDTH - 1 : 0] res_s0_wbtag;
    logic res_s1_stall;
    logic [DATA_WIDTH_BYTE - 1 : 0] res_s1_cache_dirty;
    logic [DATA_WIDTH - 1 : 0] res_s1_cache_data;
    logic [DATA_WIDTH - 1 : 0] res_s1_old_data;
    logic res_s2_stall;
    logic [DATA_WIDTH_BYTE - 1 : 0] res_s2_new_dirty;
    logic [DATA_WIDTH - 1 : 0] res_s2_new_data;

    // states
    logic res_s0_valid;
    logic res_s1_valid;
    logic res_s1_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s1_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s1_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] res_s1_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] res_s1_byteenable;
    logic res_s1_wb;
    logic [TAG_WIDTH - 1 : 0] res_s1_wbtag;
    logic res_s2_valid;
    logic res_s2_hit;
    logic [INDEX_WIDTH - 1 : 0] res_s2_index;
    logic [OFFSET_WIDTH - 1 : 0] res_s2_offset;
    logic [DATAPATH_DATA_WIDTH - 1 : 0] res_s2_data;
    logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] res_s2_byteenable;
    logic res_s2_wb;
    logic [TAG_WIDTH - 1 : 0] res_s2_wbtag;
    logic [DATA_WIDTH_BYTE - 1 : 0] res_s2_cache_dirty;
    logic [DATA_WIDTH - 1 : 0] res_s2_old_data;

    // writeback handler variables

    // wires
    logic wb_s0_stall;
    logic wb_s1_stall;

    // states
    logic wb_ext_valid;
    logic wb_ext_old;
    logic wb_s0_valid;
    logic wb_s0_buf_valid;
    logic [TAG_WIDTH - 1 : 0] wb_s0_buf_wbtag;
    logic [INDEX_WIDTH - 1 : 0] wb_s0_buf_index;
    logic [DATA_WIDTH - 1 : 0] wb_s0_buf_cache_data;
    logic [DATA_WIDTH_BYTE - 1 : 0] wb_s0_buf_cache_dirty;
    logic wb_s1_valid;
    logic [TAG_WIDTH - 1 : 0] wb_s1_wbtag;
    logic [INDEX_WIDTH - 1 : 0] wb_s1_index;
    logic [DATA_WIDTH - 1 : 0] wb_s1_cache_data;
    logic [DATA_WIDTH_BYTE - 1 : 0] wb_s1_cache_dirty;

    // response handler

    // controls
    // wait_fifo: wait_fifo_deq
    // data_ram: data_ram_rdaddr, data_ram_rdaddrstall, data_ram_wren, data_ram_wraddr, data_ram_input
    // mem_write_req_fifo: mem_write_req_fifo_enq, mem_write_req_fifo_input
    // mem_res_fifo: mem_res_fifo_deq
    // dp_res_fifo: dp_res_fifo_enq, dp_res_fifo_input
    // meta_wb (port b): meta_wb_wr_b, meta_wb_wraddr_b, meta_wb_wrdata_b

    // pipeline stage external -> 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s0_valid <= 0;
        end else begin
            res_s0_valid <= (wait_fifo_valid_out & ~wait_fifo_wait_out) | res_s0_stall;
            if (~res_s0_stall) begin
                {res_s0_hit, res_s0_index, res_s0_offset, res_s0_data, res_s0_byteenable, res_s0_wb, res_s0_wbtag} <= wait_fifo_out;
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        data_ram_rdaddr = res_s0_index;
        if (state != STATE_CLEANING) begin
            res_s0_stall = res_s0_valid & (res_s1_stall | (wb_s0_stall & res_s0_wb & ~wb_ext_old));
            wait_fifo_wait_out = res_s0_stall;
            if (~res_s0_valid | res_s0_stall) begin
            end else if (res_s0_hit) begin
            end else begin
            end
        end else begin
            res_s0_stall = res_s0_valid & (res_s1_stall | (wb_s0_stall & res_s0_wb & ~wb_ext_old));
            wait_fifo_wait_out = res_s0_stall;
        end
    end

    // pipeline stage 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s1_valid <= 0;
        end else begin
            res_s1_valid <= (res_s0_valid & ~res_s0_stall) | res_s1_stall;
            if (~res_s1_stall) begin
                {res_s1_hit, res_s1_index, res_s1_offset, res_s1_data, res_s1_byteenable, res_s1_wb, res_s1_wbtag} <= {res_s0_hit, res_s0_index, res_s0_offset, res_s0_data, res_s0_byteenable, res_s0_wb, res_s0_wbtag};
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        {res_s1_cache_dirty, res_s1_cache_data} = data_ram_output;
        res_s1_stall = res_s1_valid & res_s2_stall;
        data_ram_rdaddrstall = res_s1_stall;

        if (res_s2_valid & res_s1_index == res_s2_index) begin
            if (1) begin
                {res_s1_cache_dirty, res_s1_cache_data} = data_ram_input;
            end
        end

        res_s1_old_data = res_s1_cache_data;
    end

    // pipeline stage 1 -> 2
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            res_s2_valid <= 0;
        end else begin
            res_s2_valid <= (res_s1_valid & ~res_s1_stall) | res_s2_stall;
            if (~res_s2_stall) begin
                {res_s2_hit, res_s2_index, res_s2_offset, res_s2_data, res_s2_byteenable, res_s2_wb, res_s2_wbtag} <= {res_s1_hit, res_s1_index, res_s1_offset, res_s1_data, res_s1_byteenable, res_s1_wb, res_s1_wbtag};
                res_s2_cache_dirty <= res_s1_cache_dirty;
                res_s2_old_data <= res_s1_old_data;
            end
        end
    end

    // pipeline stage 2
    always_comb begin
        logic [DATAPATH_DATA_WIDTH - 1 : 0] data;
        logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] dirty;
        logic [OFFSET_WIDTH - 1 : 0] offset;
        offset = OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH
               ? 0
               : {res_s2_offset[OFFSET_WIDTH - 1 : DATAPATH_OFFSET_WIDTH], {DATAPATH_OFFSET_WIDTH{1'b0}}};
        for (int i = 0; i < DATAPATH_DATA_WIDTH_BYTE; ++i) begin
            data[i * 8 +: 8] = res_s2_byteenable[i] ? res_s2_data[i * 8 +: 8] : res_s2_old_data[(offset + i) * 8 +: 8];
            dirty[i] = (res_s2_hit ? res_s2_cache_dirty[offset + i] : 0) | (res_s2_byteenable[i]);
        end

        res_s2_new_data = (res_s2_old_data & ~({DATAPATH_DATA_WIDTH{1'b1}} << offset * 8))
                        | (data << offset * 8);
        res_s2_new_dirty = ((res_s2_hit ? res_s2_cache_dirty : 0) & ~({DATAPATH_DATA_WIDTH_BYTE{1'b1}} << offset))
                         | (dirty << offset);

        data_ram_wren = 0;
        data_ram_wraddr = 'x;
        data_ram_input = 'x;

        if (state != STATE_CLEANING) begin
            res_s2_stall = 0;
            if (~res_s2_valid | res_s2_stall) begin
            end else begin
                if (res_s2_hit) begin
                    data_ram_wren = 1;
                    data_ram_wraddr = res_s2_index;
                    data_ram_input = {res_s2_new_dirty, res_s2_new_data};
                end else begin
                    data_ram_wren = 1;
                    data_ram_wraddr = res_s2_index;
                    data_ram_input = {res_s2_new_dirty, res_s2_new_data};
                end
            end
        end else begin
            res_s2_stall = res_s2_valid & 0;
        end
    end

    // writeback handler

    // pipeline ext -> stage 0
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            wb_ext_old <= 0;
            wb_s0_valid <= 0;
        end else begin
            wb_s0_valid <= wb_ext_valid | wb_s0_stall;
            if (wb_ext_valid & ~wb_s0_stall) begin
                wb_ext_old <= 1;
            end
            if (~res_s0_stall) begin
                wb_ext_old <= 0;
            end
        end
    end

    // pipeline stage 0
    always_comb begin
        wb_ext_valid = res_s0_valid & res_s0_wb & ~res_s1_stall & ~wb_ext_old;
        wb_s0_stall = wb_s0_valid & wb_s1_stall;
    end

    // pipeline stage 0 -> 1
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            wb_s1_valid <= 0;
            wb_s0_buf_valid <= 0;
        end else begin
            wb_s1_valid <= (wb_s0_valid & ~wb_s0_stall) | wb_s1_stall;
            if (wb_s0_stall & ~wb_s0_buf_valid) begin
                wb_s0_buf_valid <= 1;
                {wb_s0_buf_wbtag, wb_s0_buf_index, wb_s0_buf_cache_data, wb_s0_buf_cache_dirty} <= {res_s1_wbtag, res_s1_index, res_s1_cache_data, res_s1_cache_dirty};
            end
            if (~wb_s1_stall) begin
                if (wb_s0_buf_valid) begin
                    wb_s0_buf_valid <= 0;
                    {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty} <= {wb_s0_buf_wbtag, wb_s0_buf_index, wb_s0_buf_cache_data, wb_s0_buf_cache_dirty};
                end else begin
                    {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty} <= {res_s1_wbtag, res_s1_index, res_s1_cache_data, res_s1_cache_dirty};
                end
            end
        end
    end

    // pipeline stage 1
    always_comb begin
        mem_write_req_valid_in = 0;
        mem_write_req_in = 'x;
        if (state != STATE_CLEANING) begin
            wb_s1_stall = wb_s1_valid & (mem_write_req_wait_in);
            if (~wb_s1_valid | wb_s1_stall) begin
            end else begin
                mem_write_req_valid_in = 1;
                mem_write_req_in = {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty};
            end
        end else begin
            wb_s1_stall = wb_s1_valid & (mem_write_req_wait_in);
            if (~wb_s1_valid | wb_s1_stall) begin
            end else begin
                mem_write_req_valid_in = 1;
                mem_write_req_in = {wb_s1_wbtag, wb_s1_index, wb_s1_cache_data, wb_s1_cache_dirty};
            end
        end
    end

    // datapath request
    always_comb begin
        dp_req_in = {avs_address, avs_writedata, avs_byteenable};
        dp_req_valid_in = avs_write;
        avs_waitrequest = dp_req_wait_in;
        dp_req_cond = (avs_write_count & MAX_AVS_WRITE_COUNT) != 0;
    end

    // mem_read_req, mem_write_req -> avm
    // priority on write
    logic [TAG_WIDTH - 1 : 0] mem_write_req_out_tag;
    logic [INDEX_WIDTH - 1 : 0] mem_write_req_out_index;
    logic [DATA_WIDTH - 1 : 0] mem_write_req_out_data;
    logic [DATA_WIDTH_BYTE - 1 : 0] mem_write_req_out_byteenable;
    always_comb begin
        {mem_write_req_out_tag, mem_write_req_out_index, mem_write_req_out_data, mem_write_req_out_byteenable} = mem_write_req_out;
    end

    if (USE_BURST) begin
        logic [ADDR_WIDTH - 1 : 0] write_address;
        logic write;
        logic [BURST_WIDTH - 1 : 0] write_burstcount;
        logic write_waitrequest;
        logic [DATA_WIDTH - 1 : 0] write_data;
        logic [DATA_WIDTH_BYTE - 1 : 0] write_byteenable;

        amm_burstify_wo #(
            .ADDR_WIDTH(ADDR_WIDTH),
            .DATA_WIDTH(DATA_WIDTH),
            .BURST_WIDTH(BURST_WIDTH)
        ) amm_burstify_wo_inst (
            .clk(clk),
            .rstn(rstn),
            .avs_address({mem_write_req_out_tag, mem_write_req_out_index, {OFFSET_WIDTH{1'b0}}}),
            .avs_write(mem_write_req_valid_out),
            .avs_writedata(mem_write_req_out_data),
            .avs_byteenable(mem_write_req_out_byteenable),
            .avs_waitrequest(mem_write_req_wait_out),
            .avm_address(write_address),
            .avm_write(write),
            .avm_writedata(write_data),
            .avm_byteenable(write_byteenable),
            .avm_burstcount(write_burstcount),
            .avm_waitrequest(write_waitrequest)
        );

        always_comb begin
            avm_write = 0;
            avm_read = 0;
            avm_address = 'x;
            avm_writedata = 'x;
            avm_byteenable = 'x;
            avm_burstcount = 'x;
            write_waitrequest = 1;
            if (write) begin
                avm_write = 1;
                avm_address = write_address;
                avm_writedata = write_data;
                avm_byteenable = write_byteenable;
                avm_burstcount = write_burstcount;
                write_waitrequest = avm_waitrequest;
            end
        end
    end else begin
        always_comb begin
            avm_write = 0;
            avm_read = 0;
            avm_address = 'x;
            avm_writedata = 'x;
            avm_byteenable = 'x;
            avm_burstcount = 1;
            mem_write_req_wait_out = 1;
            if (mem_write_req_valid_out) begin
                avm_write = 1;
                avm_address = {mem_write_req_out_tag, mem_write_req_out_index, {OFFSET_WIDTH{1'b0}}};
                avm_writedata = mem_write_req_out_data;
                avm_byteenable = mem_write_req_out_byteenable;
                mem_write_req_wait_out = avm_waitrequest;
            end
        end
    end

    // counter
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            avs_write_count <= 0;
            clean_count <= 0;
            wb_count <= 0;
        end else begin
            if (state != STATE_CLEANING) begin
                logic avs_write_enq, avs_write_deq;
                logic wb_enq, wb_deq;
                avs_write_enq = dp_req_valid_out & ~dp_req_wait_out;
                avs_write_deq = res_s2_valid & ~res_s2_stall;
                wb_enq = wb_ext_valid & ~wb_s0_stall;
                wb_deq = avm_write & ~avm_waitrequest;
                if (avs_write_enq & ~avs_write_deq) avs_write_count <= avs_write_count + 1'b1;
                if (~avs_write_enq & avs_write_deq) avs_write_count <= avs_write_count - 1'b1;
                if (wb_enq & ~wb_deq) wb_count <= wb_count + 1'b1;
                if (~wb_enq & wb_deq) wb_count <= wb_count - 1'b1;
            end else begin
                logic clean_enq, clean_deq;
                clean_enq = wait_fifo_valid_in & ~wait_fifo_wait_in;
                clean_deq = avm_write & ~avm_waitrequest;
                if (clean_enq & ~clean_deq) clean_count <= clean_count + 1'b1;
                if (~clean_enq & clean_deq) clean_count <= clean_count - 1'b1;
            end
        end
    end

    // cache clean
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            state <= STATE_NORMAL;
            cache_cleaned <= 0;
        end else begin
            if (state == STATE_NORMAL) begin
                if (cache_clean) begin
                    state <= STATE_CLEAN_PENDING;
                end
                cache_cleaned <= 0;
            end else if (state == STATE_CLEAN_PENDING) begin
                // read transactions have responses, so cache_clean is asserted only when all reads are done
                // for write transactions, however, we should check
                if (avs_write_count == 0 & wb_count == 0) begin
                    state <= STATE_CLEANING;
                end
            end else if (state == STATE_CLEANING) begin
                if (req_s0_index_cnt == NUM_CACHE_ENTRY_ & ~req_s0_valid & ~req_s1_valid & ~req_s2_valid & clean_count == 0) begin
                    state <= STATE_NORMAL;
                    cache_cleaned <= 1;
                end
            end
        end
    end

    // performance counter
    if (~ENABLE_PERFORMANCE_COUNTER) begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end else begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end
        end
    end else begin
        always_ff @(posedge clk, negedge rstn) begin
            if (~rstn) begin
                pc_cache_hit <= 0;
                pc_cache_miss <= 0;
            end else begin
                pc_cache_hit <= req_s2_hit & req_s2_valid & ~req_s2_stall;
                pc_cache_miss <= ~req_s2_hit & req_s2_valid & ~req_s2_stall;
            end
        end
    end

endmodule // global_mem_cache_wo
