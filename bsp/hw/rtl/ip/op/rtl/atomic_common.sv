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

module atomic_common #(
    parameter ADDR_WIDTH,
    parameter DATA_WIDTH,
    parameter LOCK_WIDTH,
    parameter NUM_OPERANDS,
    parameter OPERAND_WIDTH,
    parameter OP_LATENCY
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] addr,
    input logic [OPERAND_WIDTH - 1 : 0] a,
    input logic [OPERAND_WIDTH - 1 : 0] b,
    input logic valid_in,
    output logic wait_in,
    output logic [OPERAND_WIDTH - 1 : 0] q,
    output logic valid_out,
    input logic wait_out,

    output logic [LOCK_WIDTH - 1 : 0] bym_lockid,
    output logic bym_lockrequestvalid,
    input logic bym_lockwaitrequest,
    input logic bym_lockresponsevalid,
    output logic bym_lockwaitresponse,

    output logic [LOCK_WIDTH - 1 : 0] bym_unlockid,
    output logic bym_unlockrequestvalid,
    input logic bym_unlockwaitrequest,
    input logic bym_unlockresponsevalid,
    output logic bym_unlockwaitresponse,

    output logic [ADDR_WIDTH - 1 : 0] avm_address,
    output logic avm_read,
    input logic [DATA_WIDTH - 1 : 0] avm_readdata,
    input logic avm_readdatavalid,
    output logic avm_write,
    output logic [DATA_WIDTH - 1 : 0] avm_writedata,
    output logic [DATA_WIDTH / 8 - 1 : 0] avm_byteenable,
    input logic avm_waitrequest,
    output logic avm_waitresponse,

    output logic op_enable,
    output logic [OPERAND_WIDTH - 1 : 0] op_a,
    output logic [OPERAND_WIDTH - 1 : 0] op_b,
    output logic [OPERAND_WIDTH - 1 : 0] op_c,
    input logic [OPERAND_WIDTH - 1 : 0] op_q
);

    localparam CAPACITY = 2 ** LOCK_WIDTH < 64 ? 2 ** LOCK_WIDTH : 64;
    localparam CAPACITY_WIDTH = $clog2(CAPACITY);
    localparam OPERAND_WIDTH_BYTE = OPERAND_WIDTH / 8;
    localparam OFFSET_WIDTH = $clog2(OPERAND_WIDTH_BYTE);
    localparam INDEX_WIDTH = ADDR_WIDTH - OFFSET_WIDTH;
    localparam DATA_WIDTH_BYTE = DATA_WIDTH / 8;
    localparam MEM_OFFSET_WIDTH = $clog2(DATA_WIDTH_BYTE);
    localparam MEM_INDEX_WIDTH = ADDR_WIDTH - MEM_OFFSET_WIDTH;

    // input fifo: operands | address
    localparam INPUT_FIFO_WIDTH = NUM_OPERANDS * OPERAND_WIDTH + INDEX_WIDTH;

    logic [INPUT_FIFO_WIDTH - 1 : 0] input_fifo_in;
    logic input_fifo_valid_in;
    logic input_fifo_wait_in;
    logic [INPUT_FIFO_WIDTH - 1 : 0] input_fifo_out;
    logic input_fifo_valid_out;
    logic input_fifo_wait_out;
    logic input_fifo_empty;
    logic input_fifo_full;

    scfifo #(
        .lpm_width(INPUT_FIFO_WIDTH),
        .lpm_widthu(2),
        .lpm_numwords(4),
        .lpm_showahead("ON"),
        .overflow_checking("OFF"),
        .underflow_checking("OFF"),
        .add_ram_output_register("ON")
    ) input_fifo (
        .clock(clk),
        .aclr(~rstn),
        .data(input_fifo_in),
        .wrreq(input_fifo_valid_in & ~input_fifo_wait_in),
        .q(input_fifo_out),
        .rdreq(input_fifo_valid_out & ~input_fifo_wait_out),
        .empty(input_fifo_empty),
        .full(input_fifo_full)
    );

    always_comb begin
        input_fifo_wait_in = input_fifo_full;
        input_fifo_valid_out = ~input_fifo_empty;
    end

    // lock fifo: operands | address
    localparam LOCK_FIFO_WIDTH = NUM_OPERANDS * OPERAND_WIDTH + INDEX_WIDTH;

    logic [LOCK_FIFO_WIDTH - 1 : 0] lock_fifo_in;
    logic lock_fifo_valid_in;
    logic lock_fifo_wait_in;
    logic [LOCK_FIFO_WIDTH - 1 : 0] lock_fifo_out;
    logic lock_fifo_valid_out;
    logic lock_fifo_wait_out;
    logic lock_fifo_empty;
    logic lock_fifo_full;

    scfifo #(
        .lpm_width(LOCK_FIFO_WIDTH),
        .lpm_widthu(2),
        .lpm_numwords(4),
        .lpm_showahead("ON"),
        .overflow_checking("OFF"),
        .underflow_checking("OFF"),
        .add_ram_output_register("OFF")
    ) lock_fifo (
        .clock(clk),
        .aclr(~rstn),
        .data(lock_fifo_in),
        .wrreq(lock_fifo_valid_in & ~lock_fifo_wait_in),
        .q(lock_fifo_out),
        .rdreq(lock_fifo_valid_out & ~lock_fifo_wait_out),
        .empty(lock_fifo_empty),
        .full(lock_fifo_full)
    );

    always_comb begin
        lock_fifo_wait_in = lock_fifo_full;
        lock_fifo_valid_out = ~lock_fifo_empty;
    end

    // read request fifo: address
    localparam READ_REQ_FIFO_WIDTH = MEM_INDEX_WIDTH;

    logic [READ_REQ_FIFO_WIDTH - 1 : 0] read_req_fifo_in;
    logic read_req_fifo_valid_in;
    logic read_req_fifo_wait_in;
    logic [READ_REQ_FIFO_WIDTH - 1 : 0] read_req_fifo_out;
    logic read_req_fifo_valid_out;
    logic read_req_fifo_wait_out;
    logic read_req_fifo_empty;
    logic read_req_fifo_full;

    scfifo #(
        .lpm_width(READ_REQ_FIFO_WIDTH),
        .lpm_widthu(2),
        .lpm_numwords(4),
        .lpm_showahead("ON"),
        .overflow_checking("OFF"),
        .underflow_checking("OFF"),
        .add_ram_output_register("ON")
    ) read_req_fifo (
        .clock(clk),
        .aclr(~rstn),
        .data(read_req_fifo_in),
        .wrreq(read_req_fifo_valid_in & ~read_req_fifo_wait_in),
        .q(read_req_fifo_out),
        .rdreq(read_req_fifo_valid_out & ~read_req_fifo_wait_out),
        .empty(read_req_fifo_empty),
        .full(read_req_fifo_full)
    );

    always_comb begin
        read_req_fifo_wait_in = read_req_fifo_full;
        read_req_fifo_valid_out = ~read_req_fifo_empty;
    end

    // write request fifo: address | new result
    localparam WRITE_REQ_FIFO_WIDTH = INDEX_WIDTH + OPERAND_WIDTH;

    logic [WRITE_REQ_FIFO_WIDTH - 1 : 0] write_req_fifo_in;
    logic write_req_fifo_valid_in;
    logic write_req_fifo_wait_in;
    logic [WRITE_REQ_FIFO_WIDTH - 1 : 0] write_req_fifo_out;
    logic write_req_fifo_valid_out;
    logic write_req_fifo_wait_out;
    logic write_req_fifo_empty;
    logic write_req_fifo_full;

    scfifo #(
        .lpm_width(WRITE_REQ_FIFO_WIDTH),
        .lpm_widthu(2),
        .lpm_numwords(4),
        .lpm_showahead("ON"),
        .overflow_checking("OFF"),
        .underflow_checking("OFF"),
        .add_ram_output_register("ON")
    ) write_req_fifo (
        .clock(clk),
        .aclr(~rstn),
        .data(write_req_fifo_in),
        .wrreq(write_req_fifo_valid_in & ~write_req_fifo_wait_in),
        .q(write_req_fifo_out),
        .rdreq(write_req_fifo_valid_out & ~write_req_fifo_wait_out),
        .empty(write_req_fifo_empty),
        .full(write_req_fifo_full)
    );

    always_comb begin
        write_req_fifo_wait_in = write_req_fifo_full;
        write_req_fifo_valid_out = ~write_req_fifo_empty;
    end

    // load fifo: operands | address
    localparam LOAD_FIFO_WIDTH = NUM_OPERANDS * OPERAND_WIDTH + INDEX_WIDTH;

    logic [LOAD_FIFO_WIDTH - 1 : 0] load_fifo_in;
    logic load_fifo_valid_in;
    logic load_fifo_wait_in;
    logic [LOAD_FIFO_WIDTH - 1 : 0] load_fifo_out;
    logic load_fifo_valid_out;
    logic load_fifo_wait_out;
    logic load_fifo_empty;
    logic load_fifo_full;

    scfifo #(
        .lpm_width(LOAD_FIFO_WIDTH),
        .lpm_widthu(CAPACITY_WIDTH),
        .lpm_numwords(2 ** CAPACITY_WIDTH),
        .lpm_showahead("ON"),
        .overflow_checking("OFF"),
        .underflow_checking("OFF"),
        .add_ram_output_register("ON")
    ) load_fifo (
        .clock(clk),
        .aclr(~rstn),
        .data(load_fifo_in),
        .wrreq(load_fifo_valid_in & ~load_fifo_wait_in),
        .q(load_fifo_out),
        .rdreq(load_fifo_valid_out & ~load_fifo_wait_out),
        .empty(load_fifo_empty),
        .full(load_fifo_full)
    );

    always_comb begin
        load_fifo_wait_in = load_fifo_full;
        load_fifo_valid_out = ~load_fifo_empty;
    end

    // unlock fifo: old result
    localparam UNLOCK_FIFO_WIDTH = OPERAND_WIDTH;

    logic [UNLOCK_FIFO_WIDTH - 1 : 0] unlock_fifo_in;
    logic unlock_fifo_valid_in;
    logic unlock_fifo_wait_in;
    logic [UNLOCK_FIFO_WIDTH - 1 : 0] unlock_fifo_out;
    logic unlock_fifo_valid_out;
    logic unlock_fifo_wait_out;
    logic unlock_fifo_empty;
    logic unlock_fifo_full;

    scfifo #(
        .lpm_width(UNLOCK_FIFO_WIDTH),
        .lpm_widthu(2),
        .lpm_numwords(4),
        .lpm_showahead("ON"),
        .overflow_checking("OFF"),
        .underflow_checking("OFF"),
        .add_ram_output_register("OFF")
    ) unlock_fifo (
        .clock(clk),
        .aclr(~rstn),
        .data(unlock_fifo_in),
        .wrreq(unlock_fifo_valid_in & ~unlock_fifo_wait_in),
        .q(unlock_fifo_out),
        .rdreq(unlock_fifo_valid_out & ~unlock_fifo_wait_out),
        .empty(unlock_fifo_empty),
        .full(unlock_fifo_full)
    );

    always_comb begin
        unlock_fifo_wait_in = unlock_fifo_full;
        unlock_fifo_valid_out = ~unlock_fifo_empty;
    end

    // input => input fifo
    always_comb begin
        logic [INDEX_WIDTH - 1 : 0] index;

        index = addr[ADDR_WIDTH - 1 : OFFSET_WIDTH];
        input_fifo_in = {b, a, index};
        input_fifo_valid_in = valid_in;
        wait_in = input_fifo_wait_in;
    end

    // multiple successor pattern
    // input fifo => lock, lock_fifo
    logic block_lock;
    logic block_lock_fifo;

    always_comb begin
        logic [INDEX_WIDTH - 1 : 0] index;
        logic [OPERAND_WIDTH - 1 : 0] a;
        logic [OPERAND_WIDTH - 1 : 0] b;

        {b, a, index} = input_fifo_out;

        bym_lockid = index[LOCK_WIDTH - 1 : 0];
        bym_lockrequestvalid = input_fifo_valid_out & ~block_lock;

        lock_fifo_in = {b, a, index};
        lock_fifo_valid_in = input_fifo_valid_out & ~block_lock_fifo;

        input_fifo_wait_out = (bym_lockrequestvalid & bym_lockwaitrequest) | (lock_fifo_valid_in & lock_fifo_wait_in);
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            block_lock <= 0;
            block_lock_fifo <= 0;
        end else begin
            block_lock <= input_fifo_valid_out & input_fifo_wait_out & (~bym_lockwaitrequest | block_lock);
            block_lock_fifo <= input_fifo_valid_out & input_fifo_wait_out & (~lock_fifo_wait_in | block_lock_fifo);
        end
    end

    // lock, lock_fifo => load_fifo, read_req_fifo
    logic load_valid;
    logic load_wait;
    logic block_load_fifo;
    logic block_read_req_fifo;

    always_comb begin
        logic [INDEX_WIDTH - 1 : 0] index;
        logic [NUM_OPERANDS * OPERAND_WIDTH - 1 : 0] operands;

        {operands, index} = lock_fifo_out;
        load_valid = bym_lockresponsevalid & lock_fifo_valid_out;


        load_fifo_in = {operands, index};
        load_fifo_valid_in = load_valid & ~block_load_fifo;

        read_req_fifo_in = index[INDEX_WIDTH - 1 : MEM_OFFSET_WIDTH - OFFSET_WIDTH];
        read_req_fifo_valid_in = load_valid & ~block_read_req_fifo;

        load_wait = (load_fifo_valid_in & load_fifo_wait_in) | (read_req_fifo_valid_in & read_req_fifo_wait_in);
        bym_lockwaitresponse = load_wait | (~load_valid & bym_lockresponsevalid);
        lock_fifo_wait_out = load_wait | (~load_valid & lock_fifo_valid_out);
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            block_load_fifo <= 0;
            block_read_req_fifo <= 0;
        end else begin
            block_load_fifo <= load_valid & load_wait & (~load_fifo_wait_in | block_load_fifo);
            block_read_req_fifo <= load_valid & load_wait & (~read_req_fifo_wait_in | block_read_req_fifo);
        end
    end

    // load_fifo, avm read response => op
    logic op_s0_valid;
    logic op_s0_stall;
    logic [INDEX_WIDTH - 1 : 0] op_s0_index;
    logic [OPERAND_WIDTH - 1 : 0] op_s0_a;
    logic [OPERAND_WIDTH - 1 : 0] op_s0_b;
    logic [OPERAND_WIDTH - 1 : 0] op_s0_old_result;
    logic op_s1_valid;
    logic op_s1_stall;
    logic [INDEX_WIDTH - 1 : 0] op_s1_index;
    logic [OPERAND_WIDTH - 1 : 0] op_s1_a;
    logic [OPERAND_WIDTH - 1 : 0] op_s1_b;
    logic [OPERAND_WIDTH - 1 : 0] op_s1_old_result;
    logic op_s2_valid[OP_LATENCY];
    logic op_s2_stall;
    logic [INDEX_WIDTH - 1 : 0] op_s2_index[OP_LATENCY];
    logic [OPERAND_WIDTH - 1 : 0] op_s2_old_result[OP_LATENCY];
    logic op_s3_valid;
    logic op_s3_stall;
    logic [INDEX_WIDTH - 1 : 0] op_s3_index;
    logic [OPERAND_WIDTH - 1 : 0] op_s3_old_result;
    logic [OPERAND_WIDTH - 1 : 0] op_s3_new_result;

    always_comb begin
        op_s0_valid = load_fifo_valid_out & avm_readdatavalid;
        op_s0_stall = op_s0_valid & op_s1_stall;
        load_fifo_wait_out = op_s0_stall | (~op_s0_valid & load_fifo_valid_out);
        avm_waitresponse = op_s0_stall | (~op_s0_valid & avm_readdatavalid);

        {op_s0_b, op_s0_a, op_s0_index} = load_fifo_out;
        op_s0_old_result = MEM_OFFSET_WIDTH - OFFSET_WIDTH == 0
                         ? avm_readdata
                         : avm_readdata[{op_s0_index[MEM_OFFSET_WIDTH - OFFSET_WIDTH - 1 + (MEM_OFFSET_WIDTH - OFFSET_WIDTH == 0) : 0], {OFFSET_WIDTH{1'b0}}, 3'b000} +: OPERAND_WIDTH];
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            op_s1_valid <= 0;
        end else begin
            op_s1_valid <= op_s1_stall | (op_s0_valid & ~op_s0_stall);
            if (~op_s1_stall) begin
                op_s1_index <= op_s0_index;
                op_s1_a <= op_s0_a;
                op_s1_b <= op_s0_b;
                op_s1_old_result <= op_s0_old_result;
            end
        end
    end

    always_comb begin
        op_s1_stall = op_s1_valid & op_s2_stall;
        op_a = op_s1_old_result;
        op_b = op_s1_a;
        op_c = op_s1_b;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            for (int i = 0; i < OP_LATENCY; ++i) begin
                op_s2_valid[i] <= 0;
            end
        end else begin
            if (~op_s2_stall) begin
                op_s2_valid[0] <= op_s1_valid & ~op_s1_stall;
                op_s2_index[0] <= op_s1_index;
                op_s2_old_result[0] <= op_s1_old_result;
                for (int i = 1; i < OP_LATENCY; ++i) begin
                    op_s2_valid[i] <= op_s2_valid[i - 1];
                    op_s2_index[i] <= op_s2_index[i - 1];
                    op_s2_old_result[i] <= op_s2_old_result[i - 1];
                end
            end
        end
    end

    always_comb begin
        op_s2_stall = op_s3_stall;
        op_enable = ~op_s2_stall;
    end

    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            op_s3_valid <= 0;
        end else begin
            op_s3_valid <= op_s3_stall | (op_s2_valid[OP_LATENCY - 1] & ~op_s2_stall);
            if (~op_s3_stall) begin
                op_s3_index <= op_s2_index[OP_LATENCY - 1];
                op_s3_old_result <= op_s2_old_result[OP_LATENCY - 1];
                op_s3_new_result <= op_q;
            end
        end
    end

    // op => write_req_fifo, unlock_fifo, unlock
    // write_req_fifo receive data ASAP, unlock_fifo and unlock receive data only after write_req_fifo receive data
    logic block_write_req_fifo;
    logic block_unlock_fifo;
    logic block_unlock;
    always_comb begin
        write_req_fifo_in = {op_s3_index, op_s3_new_result};
        write_req_fifo_valid_in = op_s3_valid & ~block_write_req_fifo;

        unlock_fifo_in = op_s3_old_result;
        unlock_fifo_valid_in = op_s3_valid & ~block_unlock_fifo & ~(write_req_fifo_valid_in & write_req_fifo_wait_in);

        bym_unlockid = op_s3_index[LOCK_WIDTH - 1 : 0];
        bym_unlockrequestvalid = op_s3_valid & ~block_unlock & ~(write_req_fifo_valid_in & write_req_fifo_wait_in);

        op_s3_stall = op_s3_valid & ((write_req_fifo_valid_in & write_req_fifo_wait_in) | (unlock_fifo_valid_in & unlock_fifo_wait_in) | (bym_unlockrequestvalid & bym_unlockwaitrequest));
    end
    always_ff @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            block_write_req_fifo <= 0;
            block_unlock_fifo <= 0;
            block_unlock <= 0;
        end else begin
            block_write_req_fifo <= op_s3_stall & (~write_req_fifo_wait_in | block_write_req_fifo);
            block_unlock_fifo <= op_s3_stall & ((~unlock_fifo_wait_in & ~(write_req_fifo_valid_in & write_req_fifo_wait_in)) | block_unlock_fifo);
            block_unlock <= op_s3_stall & ((~bym_unlockwaitrequest & ~(write_req_fifo_valid_in & write_req_fifo_wait_in)) | block_unlock);
        end
    end

    // unlock_fifo, unlock => output
    always_comb begin
        valid_out = unlock_fifo_valid_out & bym_unlockresponsevalid;
        unlock_fifo_wait_out = wait_out | (~valid_out & unlock_fifo_valid_out);
        bym_unlockwaitresponse = wait_out | (~valid_out & bym_unlockresponsevalid);

        q = unlock_fifo_out;
    end

    // read_req_fifo, write_req_fifo => avm
    // priority on write
    always_comb begin
        write_req_fifo_wait_out = avm_waitrequest;
        read_req_fifo_wait_out = avm_waitrequest | write_req_fifo_valid_out;

        avm_read = 0;
        avm_write = 0;
        avm_address = 'x;
        avm_writedata = 'x;
        avm_byteenable = 'x;
        if (write_req_fifo_valid_out) begin
            logic [INDEX_WIDTH - 1 : 0] index;
            logic [OPERAND_WIDTH - 1 : 0] new_result;
            logic [MEM_OFFSET_WIDTH - 1 : 0] mem_offset;
            {index, new_result} = write_req_fifo_out;
            avm_write = 1;
            avm_address = {index[INDEX_WIDTH - 1 : MEM_OFFSET_WIDTH - OFFSET_WIDTH], {MEM_OFFSET_WIDTH{1'b0}}};
            mem_offset = MEM_OFFSET_WIDTH - OFFSET_WIDTH == 0
                       ? 0
                       : {index[MEM_OFFSET_WIDTH - OFFSET_WIDTH - 1 + (MEM_OFFSET_WIDTH - OFFSET_WIDTH == 0) : 0], {OFFSET_WIDTH{1'b0}}};
            avm_writedata[mem_offset * 8 +: OPERAND_WIDTH] = new_result;
            avm_byteenable = 0;
            avm_byteenable[mem_offset +: OPERAND_WIDTH_BYTE] = {OPERAND_WIDTH_BYTE{1'b1}};
        end else if (read_req_fifo_valid_out) begin
            logic [MEM_INDEX_WIDTH - 1 : 0] mem_index;
            mem_index = read_req_fifo_out;
            avm_read = 1;
            avm_address = {mem_index, {MEM_OFFSET_WIDTH{1'b0}}};
            avm_byteenable = {DATA_WIDTH_BYTE{1'b1}};
        end
    end

endmodule
