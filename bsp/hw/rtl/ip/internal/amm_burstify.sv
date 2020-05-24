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

module amm_burstify #(
  parameter ADDR_WIDTH = 33,
  parameter DATA_WIDTH = 512,
  parameter BURST_WIDTH = 4
) (
  input logic clk,
  input logic rstn,
  input logic [ADDR_WIDTH - 1 : 0] avs_address,
  input logic avs_read,
  input logic avs_write,
  input logic [DATA_WIDTH - 1 : 0] avs_writedata,
  input logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
  output logic avs_waitrequest,
  output logic [ADDR_WIDTH - 1 : 0] avm_address,
  output logic avm_read,
  output logic avm_write,
  output logic [DATA_WIDTH - 1 : 0] avm_writedata,
  output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
  output logic [BURST_WIDTH - 1 : 0] avm_burstcount,
  input logic avm_waitrequest
);

  localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
  localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
  localparam TAG_WIDTH = ADDR_WIDTH - OFFSET_WIDTH;
  localparam TIMEOUT = 64;
  localparam TIME_WIDTH = $clog2(TIMEOUT + 1);
  localparam MAX_BURSTCOUNT = 2 ** (BURST_WIDTH - 1);

  // avs req : rw | tag | data | byteenable
  localparam AVS_REQ_WIDTH = 1 + TAG_WIDTH + DATA_WIDTH + DATA_WIDTH_BYTE;
  logic [AVS_REQ_WIDTH - 1 : 0] avs_req_in, avs_req_out;
  logic avs_req_valid_in, avs_req_wait_in, avs_req_valid_out, avs_req_wait_out;
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

  // avm req : rw | tag | data | byteenable | burstcount
  localparam AVM_REQ_WIDTH = 1 + TAG_WIDTH + DATA_WIDTH + DATA_WIDTH_BYTE + BURST_WIDTH;
  logic [AVM_REQ_WIDTH - 1 : 0] avm_req_in, avm_req_out;
  logic avm_req_valid_in, avm_req_wait_in, avm_req_valid_out, avm_req_wait_out;
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

  // data queue : data | byteenable
  localparam DATA_QUEUE_WIDTH = DATA_WIDTH + DATA_WIDTH_BYTE;
  localparam DATA_QUEUE_DEPTH = 2 * MAX_BURSTCOUNT; // MAX_BURSTCOUNT is too tight, so multiply by 2
  logic [DATA_QUEUE_WIDTH - 1 : 0] data_queue_in, data_queue_out;
  logic data_queue_valid_in, data_queue_wait_in;
  logic data_queue_valid_out, data_queue_wait_out;
  hs_queue #(
    .WIDTH(DATA_QUEUE_WIDTH),
    .DEPTH(DATA_QUEUE_DEPTH)
  ) data_queue (
    .clk(clk),
    .rstn(rstn),
    .in(data_queue_in),
    .valid_in(data_queue_valid_in),
    .wait_in(data_queue_wait_in),
    .out(data_queue_out),
    .valid_out(data_queue_valid_out),
    .wait_out(data_queue_wait_out)
  );

  // addr queue : rw | addr | burstcount
  localparam ADDR_QUEUE_WIDTH = 1 + TAG_WIDTH + BURST_WIDTH;
  // same as data queue, considering worst case (i.e., every burstcount == 1)
  localparam ADDR_QUEUE_DEPTH = DATA_QUEUE_DEPTH;
  logic [ADDR_QUEUE_WIDTH - 1 : 0] addr_queue_in, addr_queue_out;
  logic addr_queue_valid_in, addr_queue_wait_in;
  logic addr_queue_valid_out, addr_queue_wait_out;
  hs_queue #(
    .WIDTH(ADDR_QUEUE_WIDTH),
    .DEPTH(ADDR_QUEUE_DEPTH)
  ) addr_queue (
    .clk(clk),
    .rstn(rstn),
    .in(addr_queue_in),
    .valid_in(addr_queue_valid_in),
    .wait_in(addr_queue_wait_in),
    .out(addr_queue_out),
    .valid_out(addr_queue_valid_out),
    .wait_out(addr_queue_wait_out)
  );

  // avs -> avs req
  always_comb begin
    avs_req_in = {avs_write, avs_address[ADDR_WIDTH - 1 -: TAG_WIDTH], avs_writedata, avs_byteenable};
    avs_req_valid_in = avs_read | avs_write;
    avs_waitrequest = avs_req_wait_in;
  end

  // avs req -> data queue, addr queue
  logic cache_valid;
  logic cache_rw;
  logic [TAG_WIDTH - 1 : 0] cache_tag_start;
  logic [TAG_WIDTH : 0] cache_tag_current;
  logic [BURST_WIDTH - 1 : 0] cache_burstcount;
  logic [TIME_WIDTH - 1 : 0] cache_time;
  logic avs_req_out_rw;
  logic [TAG_WIDTH - 1 : 0] avs_req_out_tag;
  logic [DATA_WIDTH - 1 : 0] avs_req_out_data;
  logic [DATA_WIDTH_BYTE - 1 : 0] avs_req_out_byteenable;
  logic req_addr_state;
  logic req_data_state;
  always_comb begin
    {avs_req_out_rw, avs_req_out_tag, avs_req_out_data, avs_req_out_byteenable} = avs_req_out;
    data_queue_valid_in = 0;
    data_queue_in = 'x;
    addr_queue_valid_in = 0;
    addr_queue_in = 'x;
    if (cache_valid & (cache_time & TIMEOUT) != 0) begin
      // timeout : evict & accept if req exists
      addr_queue_valid_in = req_addr_state == 0;
      addr_queue_in = {cache_rw, cache_tag_start, cache_burstcount};
      if (avs_req_valid_out & avs_req_out_rw) begin
        data_queue_valid_in = req_data_state == 0;
        data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
      end
    end else if (cache_valid & (cache_burstcount & MAX_BURSTCOUNT) != 0) begin
      // burstcount full : evict & accept if req exists
      addr_queue_valid_in = req_addr_state == 0;
      addr_queue_in = {cache_rw, cache_tag_start, cache_burstcount};
      if (avs_req_valid_out & avs_req_out_rw) begin
        data_queue_valid_in = req_data_state == 0;
        data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
      end
    end else if (avs_req_valid_out & cache_valid & cache_tag_current == avs_req_out_tag & cache_rw == avs_req_out_rw) begin
      // cache valid & hit : accept
      if (avs_req_out_rw) begin
        data_queue_valid_in = req_data_state == 0;
        data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
      end
    end else if (avs_req_valid_out & cache_valid) begin
      // cache valid & miss : evict & accept
      addr_queue_valid_in = req_addr_state == 0;
      addr_queue_in = {cache_rw, cache_tag_start, cache_burstcount};
      if (avs_req_out_rw) begin
        data_queue_valid_in = req_data_state == 0;
        data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
      end
    end else if (avs_req_valid_out) begin
      // cache invalid : accept
      if (avs_req_out_rw) begin
        data_queue_valid_in = req_data_state == 0;
        data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
      end
    end
    avs_req_wait_out = (addr_queue_valid_in & addr_queue_wait_in) | (data_queue_valid_in & data_queue_wait_in);
  end

  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      req_addr_state <= 0;
      req_data_state <= 0;
    end else begin
      if (avs_req_valid_out & ~avs_req_wait_out) begin
        req_addr_state <= 0;
        req_data_state <= 0;
      end else begin
        if (addr_queue_valid_in & ~addr_queue_wait_in) begin
          req_addr_state <= req_addr_state + 1'b1;
        end
        if (data_queue_valid_in & ~data_queue_wait_in) begin
          req_data_state <= req_data_state + 1'b1;
        end
      end
    end
  end

  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      cache_valid <= 0;
    end else begin
      // increment time as default
      if (cache_valid) begin
        if ((cache_time & TIMEOUT) == 0) begin
          cache_time <= cache_time + 1'b1;
        end
      end
      if (cache_valid & (cache_time & TIMEOUT) != 0) begin
        // timeout : evict & accept if req exists
        if (~avs_req_wait_out) begin
          if (avs_req_valid_out) begin
            cache_rw <= avs_req_out_rw;
            cache_tag_start <= avs_req_out_tag;
            cache_tag_current <= avs_req_out_tag + 1'b1;
            cache_burstcount <= 1;
            cache_time <= 1;
          end else begin
            cache_valid <= 0;
          end
        end
      end else if (cache_valid & (cache_burstcount & MAX_BURSTCOUNT) != 0) begin
        // burstcount full : evict & accept if req exists
        if (~avs_req_wait_out) begin
          if (avs_req_valid_out) begin
            cache_rw <= avs_req_out_rw;
            cache_tag_start <= avs_req_out_tag;
            cache_tag_current <= avs_req_out_tag + 1'b1;
            cache_burstcount <= 1;
            cache_time <= 1;
          end else begin
            cache_valid <= 0;
          end
        end
      end else if (avs_req_valid_out & cache_valid & cache_tag_current == avs_req_out_tag & cache_rw == avs_req_out_rw) begin
        // cache valid & hit : accept
        if (~avs_req_wait_out) begin
          cache_tag_current <= cache_tag_current + 1'b1;
          cache_burstcount <= cache_burstcount + 1'b1;
        end
      end else if (avs_req_valid_out & cache_valid) begin
        // cache valid & miss : evict & accept
        if (~avs_req_wait_out) begin
          cache_rw <= avs_req_out_rw;
          cache_tag_start <= avs_req_out_tag;
          cache_tag_current <= avs_req_out_tag + 1'b1;
          cache_burstcount <= 1;
          cache_time <= 1;
        end
      end else if (avs_req_valid_out) begin
        // cache invalid : accept
        if (~avs_req_wait_out) begin
          cache_valid <= 1;
          cache_rw <= avs_req_out_rw;
          cache_tag_start <= avs_req_out_tag;
          cache_tag_current <= avs_req_out_tag + 1'b1;
          cache_burstcount <= 1;
          cache_time <= 1;
        end
      end
    end
  end

  // data queue, addr queue -> avm req
  // at first write of burst : req_valid = 0, check addr_queue_out_burstcount for burst length
  // at remaining writes : req_valid = 1, check req_burstcount for remaining burst length
  // read : req_valid = 0
  logic req_valid;
  logic [BURST_WIDTH - 1 : 0] req_burstcount;
  logic addr_queue_out_rw;
  logic [BURST_WIDTH - 1 : 0] addr_queue_out_burstcount;
  always_comb begin
    logic rw;
    logic [TAG_WIDTH - 1 : 0] tag;
    logic [DATA_WIDTH - 1 : 0] data;
    logic [DATA_WIDTH_BYTE - 1 : 0] byteenable;
    logic [BURST_WIDTH - 1 : 0] burstcount;
    {rw, tag, burstcount} = addr_queue_out;
    {data, byteenable} = data_queue_out;
    addr_queue_out_rw = rw;
    addr_queue_out_burstcount = burstcount;

    avm_req_valid_in = 0;
    avm_req_in = 'x;
    addr_queue_wait_out = 1;
    data_queue_wait_out = 1;
    if (addr_queue_valid_out) begin
      avm_req_valid_in = 1;
      avm_req_in = {rw, tag, data, byteenable, burstcount};
      addr_queue_wait_out = avm_req_wait_in
                            | (~req_valid & rw & burstcount != 1)
                            | (req_valid & req_burstcount != 1);
      data_queue_wait_out = avm_req_wait_in | ~rw;
    end
  end

  always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
      req_valid <= 0;
    end else begin
      if (avm_req_valid_in & ~avm_req_wait_in) begin
        if (~req_valid & addr_queue_out_rw & addr_queue_out_burstcount != 1) begin
          req_valid <= 1;
          req_burstcount <= addr_queue_out_burstcount - 1'b1;
        end else if (req_valid) begin
          if (req_burstcount == 1) begin
            req_valid <= 0;
          end else begin
            req_burstcount <= req_burstcount - 1'b1;
          end
        end
      end
    end
  end

  // avm req -> avm
  always_comb begin
    logic rw;
    logic [TAG_WIDTH - 1 : 0] tag;
    logic [DATA_WIDTH - 1 : 0] data;
    logic [DATA_WIDTH_BYTE - 1 : 0] byteenable;
    logic [BURST_WIDTH - 1 : 0] burstcount;
    {rw, tag, data, byteenable, burstcount} = avm_req_out;

    avm_read = avm_req_valid_out & ~rw;
    avm_write = avm_req_valid_out & rw;
    avm_address = {tag, {OFFSET_WIDTH{1'b0}}};
    avm_writedata = data;
    avm_byteenable = rw ? byteenable : {DATA_WIDTH_BYTE{1'b1}};
    avm_burstcount = burstcount;
    avm_req_wait_out = avm_waitrequest;
  end

endmodule

module amm_burstify_ro #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 512,
    parameter BURST_WIDTH = 4
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_read,
    output logic avs_waitrequest,
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount,
    input logic avm_waitrequest
);

    localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
    localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
    localparam TAG_WIDTH = ADDR_WIDTH - OFFSET_WIDTH;
    localparam TIMEOUT = 64;
    localparam TIME_WIDTH = $clog2(TIMEOUT + 1);
    localparam MAX_BURSTCOUNT = 2 ** (BURST_WIDTH - 1);

    // avs req : addr
    localparam AVS_REQ_WIDTH = TAG_WIDTH;
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

    // avm req : addr | burst
    localparam AVM_REQ_WIDTH = TAG_WIDTH + BURST_WIDTH;
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
        avs_req_in = avs_address[ADDR_WIDTH - 1 -: TAG_WIDTH];
        avs_req_valid_in = avs_read;
        avs_waitrequest = avs_req_wait_in;
    end

    // avs req -> avm req
    logic cache_valid;
    logic [TAG_WIDTH - 1 : 0] cache_tag_start;
    logic [TAG_WIDTH : 0] cache_tag_current;
    logic [BURST_WIDTH - 1 : 0] cache_burstcount;
    logic [TIME_WIDTH - 1 : 0] cache_time;
    always_comb begin
        logic [TAG_WIDTH - 1 : 0] tag;
        tag = avs_req_out;
        avs_req_wait_out = 1;
        avm_req_valid_in = 0;
        avm_req_in = 'x;

        if (cache_valid & (cache_time & TIMEOUT) != 0) begin
            // timeout : evict
            avm_req_valid_in = 1;
            avm_req_in = {cache_tag_start, cache_burstcount};
            avs_req_wait_out = avm_req_wait_in;
        end else if (cache_valid & (cache_burstcount & MAX_BURSTCOUNT) != 0) begin
            // burstcount full : evict
            avm_req_valid_in = 1;
            avm_req_in = {cache_tag_start, cache_burstcount};
            avs_req_wait_out = avm_req_wait_in;
        end else if (avs_req_valid_out & cache_valid & cache_tag_current == tag) begin
            // valid & hit : wait
            avs_req_wait_out = 0;
        end else if (avs_req_valid_out & cache_valid) begin
            // valid & miss : evict
            avm_req_valid_in = 1;
            avm_req_in = {cache_tag_start, cache_burstcount};
            avs_req_wait_out = avm_req_wait_in;
        end else if (avs_req_valid_out) begin
            // invalid : wait
            avs_req_wait_out = 0;
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            cache_valid <= 0;
        end else begin
            logic [TAG_WIDTH - 1 : 0] tag;
            tag = avs_req_out;
            // increment time as default
            if (cache_valid) begin
                if ((cache_time & TIMEOUT) == 0) begin
                    cache_time <= cache_time + 1'b1;
                end
            end
            if (cache_valid & (cache_time & TIMEOUT) != 0) begin
                // timeout : evict & accept if there is request
                if (~avs_req_wait_out) begin
                    if (avs_req_valid_out) begin
                        cache_tag_start <= tag;
                        cache_tag_current <= tag + 1'b1;
                        cache_burstcount <= 1;
                        cache_time <= 1;
                    end else begin
                        cache_valid <= 0;
                    end
                end
            end else if (cache_valid & (cache_burstcount & MAX_BURSTCOUNT) != 0) begin
                // burstcount full : evict & accept if there is request
                if (~avs_req_wait_out) begin
                    if (avs_req_valid_out) begin
                        cache_tag_start <= tag;
                        cache_tag_current <= tag + 1'b1;
                        cache_burstcount <= 1;
                        cache_time <= 1;
                    end else begin
                        cache_valid <= 0;
                    end
                end
            end else if (avs_req_valid_out & cache_valid & cache_tag_current == tag) begin
                // valid & hit : increment count
                cache_tag_current <= cache_tag_current + 1'b1;
                cache_burstcount <= cache_burstcount + 1'b1;
            end else if (avs_req_valid_out & cache_valid) begin
                // valid & miss : evict & accept
                if (~avs_req_wait_out) begin
                    cache_tag_start <= tag;
                    cache_tag_current <= tag + 1'b1;
                    cache_burstcount <= 1;
                    cache_time <= 1;
                end
            end else if (avs_req_valid_out) begin
                // invalid : accept
                cache_valid <= 1;
                cache_tag_start <= tag;
                cache_tag_current <= tag + 1'b1;
                cache_burstcount <= 1;
                cache_time <= 1;
            end
        end
    end

    // avm req -> avm
    always_comb begin
        logic [TAG_WIDTH - 1 : 0] tag;
        logic [BURST_WIDTH - 1 : 0] burstcount;
        avm_read = avm_req_valid_out;
        {tag, burstcount} = avm_req_out;
        avm_address = {tag, {OFFSET_WIDTH{1'b0}}};
        avm_burstcount = burstcount;
        avm_req_wait_out = avm_waitrequest;
    end

endmodule

module amm_burstify_wo #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 512,
    parameter BURST_WIDTH = 4
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] avs_address,
    input logic avs_write,
    input logic [DATA_WIDTH - 1 : 0] avs_writedata,
    input logic [DATA_WIDTH / 8 - 1 : 0] avs_byteenable,
    output logic avs_waitrequest,
    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    output logic [BURST_WIDTH - 1 : 0] avm_burstcount,
    input logic avm_waitrequest
);

    localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
    localparam OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
    localparam TAG_WIDTH = ADDR_WIDTH - OFFSET_WIDTH;
    localparam TIMEOUT = 64;
    localparam TIME_WIDTH = $clog2(TIMEOUT + 1);
    localparam MAX_BURSTCOUNT = 2 ** (BURST_WIDTH - 1);

    // avs req : addr | data | byteenable
    localparam AVS_REQ_WIDTH = TAG_WIDTH + DATA_WIDTH + DATA_WIDTH_BYTE;
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

    // avm req : addr | data | byteenable | burstcount
    localparam AVM_REQ_WIDTH = TAG_WIDTH + DATA_WIDTH + DATA_WIDTH_BYTE + BURST_WIDTH;
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

    // data queue : data | byteenable
    localparam DATA_QUEUE_WIDTH = DATA_WIDTH + DATA_WIDTH_BYTE;
    localparam DATA_QUEUE_DEPTH = 2 * MAX_BURSTCOUNT; // MAX_BURSTCOUNT is too tight, so multiply by 2
    logic [DATA_QUEUE_WIDTH - 1 : 0] data_queue_in, data_queue_out;
    logic data_queue_valid_in, data_queue_wait_in;
    logic data_queue_valid_out, data_queue_wait_out;
    hs_queue #(
        .WIDTH(DATA_QUEUE_WIDTH),
        .DEPTH(DATA_QUEUE_DEPTH)
    ) data_queue (
        .clk(clk),
        .rstn(rstn),
        .in(data_queue_in),
        .valid_in(data_queue_valid_in),
        .wait_in(data_queue_wait_in),
        .out(data_queue_out),
        .valid_out(data_queue_valid_out),
        .wait_out(data_queue_wait_out)
    );

    // addr queue : addr | burstcount
    localparam ADDR_QUEUE_WIDTH = TAG_WIDTH + BURST_WIDTH;
    // same as data queue, considering worst case (i.e., every burstcount == 1)
    localparam ADDR_QUEUE_DEPTH = DATA_QUEUE_DEPTH;
    logic [ADDR_QUEUE_WIDTH - 1 : 0] addr_queue_in, addr_queue_out;
    logic addr_queue_valid_in, addr_queue_wait_in;
    logic addr_queue_valid_out, addr_queue_wait_out;
    hs_queue #(
        .WIDTH(ADDR_QUEUE_WIDTH),
        .DEPTH(ADDR_QUEUE_DEPTH)
    ) addr_queue (
        .clk(clk),
        .rstn(rstn),
        .in(addr_queue_in),
        .valid_in(addr_queue_valid_in),
        .wait_in(addr_queue_wait_in),
        .out(addr_queue_out),
        .valid_out(addr_queue_valid_out),
        .wait_out(addr_queue_wait_out)
    );

    // avs -> avs req
    always_comb begin
        avs_req_in = {avs_address[ADDR_WIDTH - 1 -: TAG_WIDTH], avs_writedata, avs_byteenable};
        avs_req_valid_in = avs_write;
        avs_waitrequest = avs_req_wait_in;
    end

    // avs req -> data queue, addr queue
    logic cache_valid;
    logic [TAG_WIDTH - 1 : 0] cache_tag_start;
    logic [TAG_WIDTH : 0] cache_tag_current;
    logic [BURST_WIDTH - 1 : 0] cache_burstcount;
    logic [TIME_WIDTH - 1 : 0] cache_time;
    logic [TAG_WIDTH - 1 : 0] avs_req_out_tag;
    logic [DATA_WIDTH - 1 : 0] avs_req_out_data;
    logic [DATA_WIDTH_BYTE - 1 : 0] avs_req_out_byteenable;
    logic req_addr_state;
    logic req_data_state;
    always_comb begin
        {avs_req_out_tag, avs_req_out_data, avs_req_out_byteenable} = avs_req_out;
        data_queue_valid_in = 0;
        data_queue_in = 'x;
        addr_queue_valid_in = 0;
        addr_queue_in = 'x;
        if (cache_valid & (cache_time & TIMEOUT) != 0) begin
            // timeout : evict & accept if req exists
            addr_queue_valid_in = req_addr_state == 0;
            addr_queue_in = {cache_tag_start, cache_burstcount};
            if (avs_req_valid_out) begin
                data_queue_valid_in = req_data_state == 0;
                data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
            end
        end else if (cache_valid & (cache_burstcount & MAX_BURSTCOUNT) != 0) begin
            // burstcount full : evict & accept if req exists
            addr_queue_valid_in = req_addr_state == 0;
            addr_queue_in = {cache_tag_start, cache_burstcount};
            if (avs_req_valid_out) begin
                data_queue_valid_in = req_data_state == 0;
                data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
            end
        end else if (avs_req_valid_out & cache_valid & cache_tag_current == avs_req_out_tag) begin
            // cache valid & hit : accept
            data_queue_valid_in = req_data_state == 0;
            data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
        end else if (avs_req_valid_out & cache_valid) begin
            // cache valid & miss : evict & accept
            addr_queue_valid_in = req_addr_state == 0;
            addr_queue_in = {cache_tag_start, cache_burstcount};
            data_queue_valid_in = req_data_state == 0;
            data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
        end else if (avs_req_valid_out) begin
            // cache invalid : accept
            data_queue_valid_in = req_data_state == 0;
            data_queue_in = {avs_req_out_data, avs_req_out_byteenable};
        end
        avs_req_wait_out = (addr_queue_valid_in & addr_queue_wait_in) | (data_queue_valid_in & data_queue_wait_in);
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_addr_state <= 0;
            req_data_state <= 0;
        end else begin
            if (avs_req_valid_out & ~avs_req_wait_out) begin
                req_addr_state <= 0;
                req_data_state <= 0;
            end else begin
                if (addr_queue_valid_in & ~addr_queue_wait_in) begin
                    req_addr_state <= req_addr_state + 1'b1;
                end
                if (data_queue_valid_in & ~data_queue_wait_in) begin
                    req_data_state <= req_data_state + 1'b1;
                end
            end
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            cache_valid <= 0;
        end else begin
            // increment time as default
            if (cache_valid) begin
                if ((cache_time & TIMEOUT) == 0) begin
                    cache_time <= cache_time + 1'b1;
                end
            end
            if (cache_valid & (cache_time & TIMEOUT) != 0) begin
                // timeout : evict & accept if req exists
                if (~avs_req_wait_out) begin
                    if (avs_req_valid_out) begin
                        cache_tag_start <= avs_req_out_tag;
                        cache_tag_current <= avs_req_out_tag + 1'b1;
                        cache_burstcount <= 1;
                        cache_time <= 1;
                    end else begin
                        cache_valid <= 0;
                    end
                end
            end else if (cache_valid & (cache_burstcount & MAX_BURSTCOUNT) != 0) begin
                // burstcount full : evict & accept if req exists
                if (~avs_req_wait_out) begin
                    if (avs_req_valid_out) begin
                        cache_tag_start <= avs_req_out_tag;
                        cache_tag_current <= avs_req_out_tag + 1'b1;
                        cache_burstcount <= 1;
                        cache_time <= 1;
                    end else begin
                        cache_valid <= 0;
                    end
                end
            end else if (avs_req_valid_out & cache_valid & cache_tag_current == avs_req_out_tag) begin
                // cache valid & hit : accept
                if (~avs_req_wait_out) begin
                    cache_tag_current <= cache_tag_current + 1'b1;
                    cache_burstcount <= cache_burstcount + 1'b1;
                end
            end else if (avs_req_valid_out & cache_valid) begin
                // cache valid & miss : evict & accept
                if (~avs_req_wait_out) begin
                    cache_tag_start <= avs_req_out_tag;
                    cache_tag_current <= avs_req_out_tag + 1'b1;
                    cache_burstcount <= 1;
                    cache_time <= 1;
                end
            end else if (avs_req_valid_out) begin
                // cache invalid : accept
                if (~avs_req_wait_out) begin
                    cache_valid <= 1;
                    cache_tag_start <= avs_req_out_tag;
                    cache_tag_current <= avs_req_out_tag + 1'b1;
                    cache_burstcount <= 1;
                    cache_time <= 1;
                end
            end
        end
    end

    // data queue, addr queue -> avm req
    // at first write of burst : req_valid = 0, check addr_queue_out_burstcount for burst length
    // at remaining writes : req_valid = 1, check req_burstcount for remaining burst length
    logic req_valid;
    logic [BURST_WIDTH - 1 : 0] req_burstcount;
    logic [BURST_WIDTH - 1 : 0] addr_queue_out_burstcount;
    always_comb begin
        logic [TAG_WIDTH - 1 : 0] tag;
        logic [DATA_WIDTH - 1 : 0] data;
        logic [DATA_WIDTH_BYTE - 1 : 0] byteenable;
        logic [BURST_WIDTH - 1 : 0] burstcount;
        {tag, burstcount} = addr_queue_out;
        {data, byteenable} = data_queue_out;
        addr_queue_out_burstcount = burstcount;

        avm_req_valid_in = 0;
        avm_req_in = 'x;
        addr_queue_wait_out = 1;
        data_queue_wait_out = 1;
        if (addr_queue_valid_out) begin
            avm_req_valid_in = 1;
            avm_req_in = {tag, data, byteenable, burstcount};
            addr_queue_wait_out = avm_req_wait_in
                                | (~req_valid & burstcount != 1)
                                | (req_valid & req_burstcount != 1);
            data_queue_wait_out = avm_req_wait_in;
        end
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            req_valid <= 0;
        end else begin
            if (avm_req_valid_in & ~avm_req_wait_in) begin
                if (~req_valid & addr_queue_out_burstcount != 1) begin
                    req_valid <= 1;
                    req_burstcount <= addr_queue_out_burstcount - 1'b1;
                end else if (req_valid) begin
                    if (req_burstcount == 1) begin
                        req_valid <= 0;
                    end else begin
                        req_burstcount <= req_burstcount - 1'b1;
                    end
                end
            end
        end
    end

    // avm req -> avm
    always_comb begin
        logic [TAG_WIDTH - 1 : 0] tag;
        logic [DATA_WIDTH - 1 : 0] data;
        logic [DATA_WIDTH_BYTE - 1 : 0] byteenable;
        logic [BURST_WIDTH - 1 : 0] burstcount;
        {tag, data, byteenable, burstcount} = avm_req_out;

        avm_write = avm_req_valid_out;
        avm_address = {tag, {OFFSET_WIDTH{1'b0}}};
        avm_writedata = data;
        avm_byteenable = byteenable;
        avm_burstcount = burstcount;
        avm_req_wait_out = avm_waitrequest;
    end

endmodule
