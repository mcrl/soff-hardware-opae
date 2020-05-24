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

module global_mem_bypass #(
  parameter ADDR_WIDTH = 33,
  parameter DATA_WIDTH = 512,
  parameter DATAPATH_DATA_WIDTH = 32,
  parameter BURST_WIDTH = 4,
  parameter USE_BURST = 1
) (
  input logic clk,
  input logic rstn,

  // aligned to DATAPATH_DATA_WIDTH
  input logic [ADDR_WIDTH - 1 : 0] avs_address,
  input logic avs_read,
  output logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_readdata,
  output logic avs_readdatavalid,
  input logic avs_write,
  input logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_writedata,
  input logic [DATAPATH_DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
  output logic avs_waitrequest,
  input logic avs_waitresponse,

  // aligned to DATA_WIDTH
  output logic [ADDR_WIDTH - 1 : 0] avm_address,
  output logic avm_read,
  input logic [DATA_WIDTH - 1 : 0] avm_readdata,
  input logic avm_readdatavalid,
  output logic avm_write,
  output logic [DATA_WIDTH - 1 : 0] avm_writedata,
  output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
  input logic avm_waitrequest,
  output logic [BURST_WIDTH - 1 : 0] avm_burstcount
);

  localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
  localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
  localparam TAG_WIDTH = ADDR_WIDTH - OFFSET_WIDTH;
  localparam DATAPATH_DATA_WIDTH_BYTE = DATAPATH_DATA_WIDTH / 8;
  localparam DATAPATH_OFFSET_WIDTH = $clog2(DATAPATH_DATA_WIDTH_BYTE);

  localparam MAX_AVM_READ_COUNT = 512;
  logic [$clog2(MAX_AVM_READ_COUNT + 1) - 1 : 0] avm_read_count;

  // avs req : rw | addr | data | byteenable
  localparam AVS_REQ_WIDTH = 1 + (ADDR_WIDTH - DATAPATH_OFFSET_WIDTH) + DATAPATH_DATA_WIDTH + DATAPATH_DATA_WIDTH_BYTE;
  logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in, avs_req_out;
  logic avs_req_valid_in, avs_req_wait_in;
  logic avs_req_valid_out, avs_req_wait_out;
  hs_cutter #(
    .WIDTH(AVS_REQ_WIDTH)
  ) avs_req_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(avs_req_in),
    .valid_in(avs_req_valid_in),
    .wait_in(avs_req_wait_in),
    .out(avs_req_out),
    .valid_out(avs_req_valid_out),
    .wait_out(avs_req_wait_out)
  );

  // avm req : rw | addr | data | byteenable
  localparam AVM_REQ_WIDTH = 1 + (ADDR_WIDTH - OFFSET_WIDTH) + DATA_WIDTH + DATA_WIDTH_BYTE;
  logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
  logic avm_req_valid_in, avm_req_wait_in;
  logic avm_req_valid_out, avm_req_wait_out;
  logic avm_req_cond;
  hs_bc #(
    .WIDTH(AVM_REQ_WIDTH)
  ) avm_req_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(avm_req_in),
    .valid_in(avm_req_valid_in),
    .wait_in(avm_req_wait_in),
    .out(avm_req_out),
    .valid_out(avm_req_valid_out),
    .wait_out(avm_req_wait_out),
    .cond(avm_req_cond)
  );

  // avm res : data
  localparam AVM_RES_WIDTH = DATA_WIDTH;
  logic [AVM_RES_WIDTH - 1 : 0] avm_res_in, avm_res_out;
  logic avm_res_valid_in, avm_res_wait_in;
  logic avm_res_valid_out, avm_res_wait_out;
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

  // avs res : data
  localparam AVS_RES_WIDTH = DATAPATH_DATA_WIDTH;
  localparam AVS_RES_DEPTH = MAX_AVM_READ_COUNT;
  logic [AVS_RES_WIDTH - 1 : 0] avs_res_in, avs_res_out;
  logic avs_res_valid_in, avs_res_wait_in;
  logic avs_res_valid_out, avs_res_wait_out;
  hs_cqc #(
    .WIDTH(AVS_RES_WIDTH),
    .DEPTH(AVS_RES_DEPTH)
  ) avs_res_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(avs_res_in),
    .valid_in(avs_res_valid_in),
    .wait_in(avs_res_wait_in),
    .out(avs_res_out),
    .valid_out(avs_res_valid_out),
    .wait_out(avs_res_wait_out)
  );

  // txn : offset
  localparam TXN_WIDTH = (OFFSET_WIDTH - DATAPATH_OFFSET_WIDTH);
  localparam TXN_DEPTH = MAX_AVM_READ_COUNT;
  logic [TXN_WIDTH - 1 : 0] txn_in, txn_out;
  logic txn_valid_in, txn_wait_in;
  logic txn_valid_out, txn_wait_out;
  hs_cqc #(
    .WIDTH(TXN_WIDTH),
    .DEPTH(TXN_DEPTH)
  ) txn_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(txn_in),
    .valid_in(txn_valid_in),
    .wait_in(txn_wait_in),
    .out(txn_out),
    .valid_out(txn_valid_out),
    .wait_out(txn_wait_out)
  );

  // avs -> avs req
  always_comb begin
    avs_req_in = {avs_write, avs_address[ADDR_WIDTH - 1 : DATAPATH_OFFSET_WIDTH], avs_writedata, avs_byteenable};
    avs_req_valid_in = avs_read | avs_write;
    avs_waitrequest = avs_req_wait_in;
  end

  // avs req -> avm req, txn
  logic req_avm_state;
  logic req_txn_state;
  logic req_rw;
  logic [TAG_WIDTH - 1 : 0] req_tag;
  logic [OFFSET_WIDTH - DATAPATH_OFFSET_WIDTH - 1 : 0] req_offset;
  logic [DATAPATH_DATA_WIDTH - 1 : 0] req_data;
  logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_byteenable;
  always_comb begin
    if (OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH) begin
      {req_rw, req_tag, req_data, req_byteenable} = avs_req_out;
    end else begin
      {req_rw, req_tag, req_offset, req_data, req_byteenable} = avs_req_out;
    end

    txn_valid_in = 0;
    txn_in = 'x;
    avm_req_valid_in = 0;
    avm_req_in = 'x;
    if (avs_req_valid_out) begin
      if (~req_rw) begin // read
        txn_valid_in = req_txn_state == 0;
        txn_in = req_offset;
        avm_req_valid_in = req_avm_state == 0;
        avm_req_in = {req_rw, req_tag, {DATA_WIDTH{1'bx}}, {DATA_WIDTH_BYTE{1'bx}}};
      end else begin // write
        logic [OFFSET_WIDTH - 1 : 0] offset;
        logic [DATA_WIDTH - 1 : 0] data;
        logic [DATA_WIDTH_BYTE - 1 : 0] byteenable;
        offset = OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH
               ? 0
               : {req_offset, {DATAPATH_OFFSET_WIDTH{1'b0}}};
        data = req_data << (offset * 8);
        byteenable = req_byteenable << offset;
        avm_req_valid_in = req_avm_state == 0;
        avm_req_in = {req_rw, req_tag, data, byteenable};
      end
    end
    avs_req_wait_out = (txn_valid_in & txn_wait_in) | (avm_req_valid_in & avm_req_wait_in);
  end

  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      req_txn_state <= 0;
      req_avm_state <= 0;
    end else begin
      if (avs_req_valid_out & ~avs_req_wait_out) begin
        req_txn_state <= 0;
        req_avm_state <= 0;
      end else begin
        if (txn_valid_in & ~txn_wait_in) begin
          req_txn_state <= req_txn_state + 1'b1;
        end
        if (avm_req_valid_in & ~avm_req_wait_in) begin
          req_avm_state <= req_avm_state + 1'b1;
        end
      end
    end
  end

  // avm req -> avm
  logic avm_req_out_rw;
  logic [TAG_WIDTH - 1 : 0] avm_req_out_tag;
  logic [DATA_WIDTH - 1 : 0] avm_req_out_data;
  logic [DATA_WIDTH_BYTE - 1 : 0] avm_req_out_byteenable;
  always_comb begin
    avm_req_cond = (avm_read_count & MAX_AVM_READ_COUNT) != 0;
    {avm_req_out_rw, avm_req_out_tag, avm_req_out_data, avm_req_out_byteenable} = avm_req_out;
  end

  if (USE_BURST) begin
    amm_burstify #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH),
      .BURST_WIDTH(BURST_WIDTH)
    ) amm_burstify_inst (
      .clk(clk),
      .rstn(rstn),
      .avs_address({avm_req_out_tag, {OFFSET_WIDTH{1'b0}}}),
      .avs_read(avm_req_valid_out & ~avm_req_out_rw),
      .avs_write(avm_req_valid_out & avm_req_out_rw),
      .avs_writedata(avm_req_out_data),
      .avs_byteenable(avm_req_out_byteenable),
      .avs_waitrequest(avm_req_wait_out),
      .avm_address(avm_address),
      .avm_read(avm_read),
      .avm_write(avm_write),
      .avm_writedata(avm_writedata),
      .avm_byteenable(avm_byteenable),
      .avm_burstcount(avm_burstcount),
      .avm_waitrequest(avm_waitrequest)
    );
  end else begin
    always_comb begin
      avm_address = {avm_req_out_tag, {OFFSET_WIDTH{1'b0}}};
      avm_read = avm_req_valid_out & ~avm_req_out_rw;
      avm_write = avm_req_valid_out & avm_req_out_rw;
      avm_writedata = avm_req_out_data;
      avm_byteenable = avm_req_out_byteenable;
      avm_req_wait_out = avm_waitrequest;
    end
  end

  // avm -> avm_res
  always_comb begin
    avm_res_valid_in = avm_readdatavalid;
    avm_res_in = avm_readdata;
  end

  // avm_res, txn -> avs_res
  always_comb begin
    logic [OFFSET_WIDTH - DATAPATH_OFFSET_WIDTH - 1 : 0] offset;
    {offset} = txn_out;

    avs_res_valid_in = 0;
    avs_res_in = 'x;
    txn_wait_out = 1;
    avm_res_wait_out = 1;
    if (txn_valid_out & avm_res_valid_out) begin
      avs_res_valid_in = 1;
      avs_res_in = OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH
                 ? avm_res_out
                 : avm_res_out[{offset, {DATAPATH_OFFSET_WIDTH{1'b0}}, 3'b000} +: DATAPATH_DATA_WIDTH];
      txn_wait_out = avs_res_wait_in;
      avm_res_wait_out = avs_res_wait_in;
    end
  end

  // avs_res -> avs
  always_comb begin
    avs_readdata = avs_res_out;
    avs_readdatavalid = avs_res_valid_out;
    avs_res_wait_out = avs_waitresponse;
  end

  // read count
  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      avm_read_count <= 0;
    end else begin
      logic enq, deq;
      enq = avm_req_valid_in & ~avm_req_wait_in;
      deq = avs_res_valid_out & ~avs_res_wait_out;
      if (enq & ~deq) begin
        avm_read_count <= avm_read_count + 1'b1;
      end
      if (~enq & deq) begin
        avm_read_count <= avm_read_count - 1'b1;
      end
    end
  end

endmodule

module global_mem_bypass_ro #(
  parameter ADDR_WIDTH = 33,
  parameter DATA_WIDTH = 512,
  parameter DATAPATH_DATA_WIDTH = 32,
  parameter BURST_WIDTH = 4,
  parameter USE_BURST = 1
) (
  input logic clk,
  input logic rstn,

  // aligned to DATAPATH_DATA_WIDTH
  input logic [ADDR_WIDTH - 1 : 0] avs_address,
  input logic avs_read,
  output logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_readdata,
  output logic avs_readdatavalid,
  output logic avs_waitrequest,
  input logic avs_waitresponse,

  // aligned to DATA_WIDTH
  output logic [ADDR_WIDTH - 1 : 0] avm_address,
  output logic avm_read,
  input logic [DATA_WIDTH - 1 : 0] avm_readdata,
  input logic avm_readdatavalid,
  input logic avm_waitrequest,
  output logic [BURST_WIDTH - 1 : 0] avm_burstcount
);

  localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
  localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
  localparam TAG_WIDTH = ADDR_WIDTH - OFFSET_WIDTH;
  localparam DATAPATH_DATA_WIDTH_BYTE = DATAPATH_DATA_WIDTH / 8;
  localparam DATAPATH_OFFSET_WIDTH = $clog2(DATAPATH_DATA_WIDTH_BYTE);

  localparam MAX_AVM_READ_COUNT = 512;
  logic [$clog2(MAX_AVM_READ_COUNT + 1) - 1 : 0] avm_read_count;

  // avs req : addr
  localparam AVS_REQ_WIDTH = (ADDR_WIDTH - DATAPATH_OFFSET_WIDTH);
  logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in, avs_req_out;
  logic avs_req_valid_in, avs_req_wait_in;
  logic avs_req_valid_out, avs_req_wait_out;
  hs_cutter #(
    .WIDTH(AVS_REQ_WIDTH)
  ) avs_req_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(avs_req_in),
    .valid_in(avs_req_valid_in),
    .wait_in(avs_req_wait_in),
    .out(avs_req_out),
    .valid_out(avs_req_valid_out),
    .wait_out(avs_req_wait_out)
  );

  // avm req : addr
  localparam AVM_REQ_WIDTH = (ADDR_WIDTH - OFFSET_WIDTH);
  logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
  logic avm_req_valid_in, avm_req_wait_in;
  logic avm_req_valid_out, avm_req_wait_out;
  logic avm_req_cond;
  hs_bc #(
    .WIDTH(AVM_REQ_WIDTH)
  ) avm_req_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(avm_req_in),
    .valid_in(avm_req_valid_in),
    .wait_in(avm_req_wait_in),
    .out(avm_req_out),
    .valid_out(avm_req_valid_out),
    .wait_out(avm_req_wait_out),
    .cond(avm_req_cond)
  );

  // avm res : data
  localparam AVM_RES_WIDTH = DATA_WIDTH;
  logic [AVM_RES_WIDTH - 1 : 0] avm_res_in, avm_res_out;
  logic avm_res_valid_in, avm_res_wait_in;
  logic avm_res_valid_out, avm_res_wait_out;
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

  // avs res : data
  localparam AVS_RES_WIDTH = DATAPATH_DATA_WIDTH;
  localparam AVS_RES_DEPTH = MAX_AVM_READ_COUNT;
  logic [AVS_RES_WIDTH - 1 : 0] avs_res_in, avs_res_out;
  logic avs_res_valid_in, avs_res_wait_in;
  logic avs_res_valid_out, avs_res_wait_out;
  hs_cqc #(
    .WIDTH(AVS_RES_WIDTH),
    .DEPTH(AVS_RES_DEPTH)
  ) avs_res_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(avs_res_in),
    .valid_in(avs_res_valid_in),
    .wait_in(avs_res_wait_in),
    .out(avs_res_out),
    .valid_out(avs_res_valid_out),
    .wait_out(avs_res_wait_out)
  );

  // txn : offset
  localparam TXN_WIDTH = (OFFSET_WIDTH - DATAPATH_OFFSET_WIDTH);
  localparam TXN_DEPTH = MAX_AVM_READ_COUNT;
  logic [TXN_WIDTH - 1 : 0] txn_in, txn_out;
  logic txn_valid_in, txn_wait_in;
  logic txn_valid_out, txn_wait_out;
  hs_cqc #(
    .WIDTH(TXN_WIDTH),
    .DEPTH(TXN_DEPTH)
  ) txn_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(txn_in),
    .valid_in(txn_valid_in),
    .wait_in(txn_wait_in),
    .out(txn_out),
    .valid_out(txn_valid_out),
    .wait_out(txn_wait_out)
  );

  // avs -> avs req
  always_comb begin
    avs_req_in = {avs_address[ADDR_WIDTH - 1 : DATAPATH_OFFSET_WIDTH]};
    avs_req_valid_in = avs_read;
    avs_waitrequest = avs_req_wait_in;
  end

  // avs req -> avm req, txn
  logic req_avm_state;
  logic req_txn_state;
  logic [TAG_WIDTH - 1 : 0] req_tag;
  logic [OFFSET_WIDTH - DATAPATH_OFFSET_WIDTH - 1 : 0] req_offset;
  always_comb begin
    if (OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH) begin
      {req_tag} = avs_req_out;
    end else begin
      {req_tag, req_offset} = avs_req_out;
    end

    txn_valid_in = 0;
    txn_in = 'x;
    avm_req_valid_in = 0;
    avm_req_in = 'x;
    if (avs_req_valid_out) begin
      txn_valid_in = req_txn_state == 0;
      txn_in = req_offset;
      avm_req_valid_in = req_avm_state == 0;
      avm_req_in = {req_tag};
    end
    avs_req_wait_out = (txn_valid_in & txn_wait_in) | (avm_req_valid_in & avm_req_wait_in);
  end

  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      req_txn_state <= 0;
      req_avm_state <= 0;
    end else begin
      if (avs_req_valid_out & ~avs_req_wait_out) begin
        req_txn_state <= 0;
        req_avm_state <= 0;
      end else begin
        if (txn_valid_in & ~txn_wait_in) begin
          req_txn_state <= req_txn_state + 1'b1;
        end
        if (avm_req_valid_in & ~avm_req_wait_in) begin
          req_avm_state <= req_avm_state + 1'b1;
        end
      end
    end
  end

  // avm req -> avm
  logic [TAG_WIDTH - 1 : 0] avm_req_out_tag;
  always_comb begin
    avm_req_cond = (avm_read_count & MAX_AVM_READ_COUNT) != 0;
    {avm_req_out_tag} = avm_req_out;
  end

  if (USE_BURST) begin
    amm_burstify_ro #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH),
      .BURST_WIDTH(BURST_WIDTH)
    ) amm_burstify_inst (
      .clk(clk),
      .rstn(rstn),
      .avs_address({avm_req_out_tag, {OFFSET_WIDTH{1'b0}}}),
      .avs_read(avm_req_valid_out),
      .avs_waitrequest(avm_req_wait_out),
      .avm_address(avm_address),
      .avm_read(avm_read),
      .avm_burstcount(avm_burstcount),
      .avm_waitrequest(avm_waitrequest)
    );
  end else begin
    always_comb begin
      avm_address = {avm_req_out_tag, {OFFSET_WIDTH{1'b0}}};
      avm_read = avm_req_valid_out;
      avm_req_wait_out = avm_waitrequest;
    end
  end

  // avm -> avm_res
  always_comb begin
    avm_res_valid_in = avm_readdatavalid;
    avm_res_in = avm_readdata;
  end

  // avm_res, txn -> avs_res
  always_comb begin
    logic [OFFSET_WIDTH - DATAPATH_OFFSET_WIDTH - 1 : 0] offset;
    {offset} = txn_out;

    avs_res_valid_in = 0;
    avs_res_in = 'x;
    txn_wait_out = 1;
    avm_res_wait_out = 1;
    if (txn_valid_out & avm_res_valid_out) begin
      avs_res_valid_in = 1;
      avs_res_in = OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH
                 ? avm_res_out
                 : avm_res_out[{offset, {DATAPATH_OFFSET_WIDTH{1'b0}}, 3'b000} +: DATAPATH_DATA_WIDTH];
      txn_wait_out = avs_res_wait_in;
      avm_res_wait_out = avs_res_wait_in;
    end
  end

  // avs_res -> avs
  always_comb begin
    avs_readdata = avs_res_out;
    avs_readdatavalid = avs_res_valid_out;
    avs_res_wait_out = avs_waitresponse;
  end

  // read count
  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      avm_read_count <= 0;
    end else begin
      logic enq, deq;
      enq = avm_req_valid_in & ~avm_req_wait_in;
      deq = avs_res_valid_out & ~avs_res_wait_out;
      if (enq & ~deq) begin
        avm_read_count <= avm_read_count + 1'b1;
      end
      if (~enq & deq) begin
        avm_read_count <= avm_read_count - 1'b1;
      end
    end
  end

endmodule

module global_mem_bypass_wo #(
  parameter ADDR_WIDTH = 33,
  parameter DATA_WIDTH = 512,
  parameter DATAPATH_DATA_WIDTH = 32,
  parameter BURST_WIDTH = 4,
  parameter USE_BURST = 1
) (
  input logic clk,
  input logic rstn,

  // aligned to DATAPATH_DATA_WIDTH
  input logic [ADDR_WIDTH - 1 : 0] avs_address,
  input logic avs_write,
  input logic [DATAPATH_DATA_WIDTH - 1 : 0] avs_writedata,
  input logic [DATAPATH_DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
  output logic avs_waitrequest,
  input logic avs_waitresponse,

  // aligned to DATA_WIDTH
  output logic [ADDR_WIDTH - 1 : 0] avm_address,
  output logic avm_write,
  output logic [DATA_WIDTH - 1 : 0] avm_writedata,
  output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
  input logic avm_waitrequest,
  output logic [BURST_WIDTH - 1 : 0] avm_burstcount
);

  localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
  localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
  localparam TAG_WIDTH = ADDR_WIDTH - OFFSET_WIDTH;
  localparam DATAPATH_DATA_WIDTH_BYTE = DATAPATH_DATA_WIDTH / 8;
  localparam DATAPATH_OFFSET_WIDTH = $clog2(DATAPATH_DATA_WIDTH_BYTE);

  // avs req : addr | data | byteenable
  localparam AVS_REQ_WIDTH = (ADDR_WIDTH - DATAPATH_OFFSET_WIDTH) + DATAPATH_DATA_WIDTH + DATAPATH_DATA_WIDTH_BYTE;
  logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in, avs_req_out;
  logic avs_req_valid_in, avs_req_wait_in;
  logic avs_req_valid_out, avs_req_wait_out;
  hs_cutter #(
    .WIDTH(AVS_REQ_WIDTH)
  ) avs_req_cutter (
    .clk(clk),
    .rstn(rstn),
    .in(avs_req_in),
    .valid_in(avs_req_valid_in),
    .wait_in(avs_req_wait_in),
    .out(avs_req_out),
    .valid_out(avs_req_valid_out),
    .wait_out(avs_req_wait_out)
  );

  // avm req : addr | data | byteenable
  localparam AVM_REQ_WIDTH = (ADDR_WIDTH - OFFSET_WIDTH) + DATA_WIDTH + DATA_WIDTH_BYTE;
  logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
  logic avm_req_valid_in, avm_req_wait_in;
  logic avm_req_valid_out, avm_req_wait_out;
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

  // avs -> avs req
  always_comb begin
    avs_req_in = {avs_address[ADDR_WIDTH - 1 : DATAPATH_OFFSET_WIDTH], avs_writedata, avs_byteenable};
    avs_req_valid_in = avs_write;
    avs_waitrequest = avs_req_wait_in;
  end

  // avs req -> avm req
  logic req_avm_state;
  logic [TAG_WIDTH - 1 : 0] req_tag;
  logic [OFFSET_WIDTH - DATAPATH_OFFSET_WIDTH - 1 : 0] req_offset;
  logic [DATAPATH_DATA_WIDTH - 1 : 0] req_data;
  logic [DATAPATH_DATA_WIDTH_BYTE - 1 : 0] req_byteenable;
  always_comb begin
    if (OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH) begin
      {req_tag, req_data, req_byteenable} = avs_req_out;
    end else begin
      {req_tag, req_offset, req_data, req_byteenable} = avs_req_out;
    end

    avm_req_valid_in = 0;
    avm_req_in = 'x;
    if (avs_req_valid_out) begin
      logic [OFFSET_WIDTH - 1 : 0] offset;
      logic [DATA_WIDTH - 1 : 0] data;
      logic [DATA_WIDTH_BYTE - 1 : 0] byteenable;
      offset = OFFSET_WIDTH == DATAPATH_OFFSET_WIDTH
               ? 0
               : {req_offset, {DATAPATH_OFFSET_WIDTH{1'b0}}};
      data = req_data << (offset * 8);
      byteenable = req_byteenable << offset;
      avm_req_valid_in = req_avm_state == 0;
      avm_req_in = {req_tag, data, byteenable};
    end
    avs_req_wait_out = (avm_req_valid_in & avm_req_wait_in);
  end

  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      req_avm_state <= 0;
    end else begin
      if (avs_req_valid_out & ~avs_req_wait_out) begin
        req_avm_state <= 0;
      end else begin
        if (avm_req_valid_in & ~avm_req_wait_in) begin
          req_avm_state <= req_avm_state + 1'b1;
        end
      end
    end
  end

  // avm req -> avm
  logic [TAG_WIDTH - 1 : 0] avm_req_out_tag;
  logic [DATA_WIDTH - 1 : 0] avm_req_out_data;
  logic [DATA_WIDTH_BYTE - 1 : 0] avm_req_out_byteenable;
  always_comb begin
    {avm_req_out_tag, avm_req_out_data, avm_req_out_byteenable} = avm_req_out;
  end

  if (USE_BURST) begin
    amm_burstify_wo #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH),
      .BURST_WIDTH(BURST_WIDTH)
    ) amm_burstify_inst (
      .clk(clk),
      .rstn(rstn),
      .avs_address({avm_req_out_tag, {OFFSET_WIDTH{1'b0}}}),
      .avs_write(avm_req_valid_out),
      .avs_writedata(avm_req_out_data),
      .avs_byteenable(avm_req_out_byteenable),
      .avs_waitrequest(avm_req_wait_out),
      .avm_address(avm_address),
      .avm_write(avm_write),
      .avm_writedata(avm_writedata),
      .avm_byteenable(avm_byteenable),
      .avm_burstcount(avm_burstcount),
      .avm_waitrequest(avm_waitrequest)
    );
  end else begin
    always_comb begin
      avm_address = {avm_req_out_tag, {OFFSET_WIDTH{1'b0}}};
      avm_write = avm_req_valid_out;
      avm_writedata = avm_req_out_data;
      avm_byteenable = avm_req_out_byteenable;
      avm_req_wait_out = avm_waitrequest;
    end
  end

endmodule
