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

`ifndef SOFF_UMEM_VH
`define SOFF_UMEM_VH

`include "platform_if.vh"
import local_mem_cfg_pkg::*;

interface u_mem_if #(
  parameter U_MEM_ADDR_WIDTH = 32
) (
  input wire clk,
  input wire reset
);

  localparam ADDR_WIDTH = U_MEM_ADDR_WIDTH;
  localparam DATA_WIDTH = `PLATFORM_PARAM_LOCAL_MEMORY_DATA_WIDTH;
  localparam BURST_CNT_WIDTH = `PLATFORM_PARAM_LOCAL_MEMORY_BURST_CNT_WIDTH;
  localparam DATA_N_BYTES = (DATA_WIDTH + 7) / 8;

  logic waitrequest;
  logic [DATA_WIDTH-1:0] readdata;
  logic readdatavalid;
  logic [BURST_CNT_WIDTH-1:0] burstcount;
  logic [DATA_WIDTH-1:0] writedata;
  logic [ADDR_WIDTH-1:0] address;
  logic write;
  logic read;
  logic [DATA_N_BYTES-1:0] byteenable;

  modport to_fiu (
    input clk,
    input reset,
    input waitrequest,
    input readdata,
    input readdatavalid,
    output burstcount,
    output writedata,
    output address,
    output write,
    output read,
    output byteenable
  );
  modport to_afu (
    input clk,
    input reset,
    output waitrequest,
    output readdata,
    output readdatavalid,
    input burstcount,
    input writedata,
    input address,
    input write,
    input read,
    input byteenable
  );

endinterface // u_mem_if

`endif // SOFF_UMEM_VH
