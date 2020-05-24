`timescale 1ns / 1ps
// Automatically generated file; DO NOT EDIT
module atomic_xor_global_u64 #(
    parameter ADDR_WIDTH = 48,
    parameter DATA_WIDTH = 512,
    parameter LOCK_WIDTH = 4
) (
    input logic clk,
    input logic rstn,
    input logic [ADDR_WIDTH - 1 : 0] addr,
    input logic [64 - 1 : 0] a,
    input logic valid_in,
    output logic wait_in,
    output logic [64 - 1 : 0] q,
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
    output logic avm_waitresponse
);
localparam NUM_OPERANDS = 1;
localparam OPERAND_WIDTH = 64;
localparam OP_LATENCY = 1;

logic [OPERAND_WIDTH - 1 : 0] b;
assign b = 0;

logic op_enable;
logic [OPERAND_WIDTH - 1 : 0] op_a;
logic [OPERAND_WIDTH - 1 : 0] op_b;
logic [OPERAND_WIDTH - 1 : 0] op_c;
logic [OPERAND_WIDTH - 1 : 0] op_q;

always_ff @(posedge clk, negedge rstn) begin
    if (~rstn) begin
    end else if (op_enable) begin
        op_q <= op_a ^ op_b;
    end
end

atomic_common #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .LOCK_WIDTH(LOCK_WIDTH),
    .NUM_OPERANDS(NUM_OPERANDS),
    .OPERAND_WIDTH(OPERAND_WIDTH),
    .OP_LATENCY(OP_LATENCY)
) atomic_common_inst (
    .*
);
endmodule
