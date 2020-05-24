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
`include "afu_json_info.vh"
`include "cci_mpf_if.vh"

`include "csr.vh"
`include "u_mem.vh"

module ccip_entry #(
  parameter MPF_DFH_MMIO_ADDR
) (
  input logic clk,
  input logic reset,
  input logic pClk,
  input logic pClkDiv2,
  input logic pClkDiv4,
  input logic uClk_usr,
  input logic uClk_usrDiv2,
  input logic pck_cp2af_softReset,
  input logic [1:0] pck_cp2af_pwrState,
  input logic pck_cp2af_error,
  input t_if_ccip_Rx pck_cp2af_sRx,
  output t_if_ccip_Tx pck_af2cp_sTx,
  output t_if_ccip_Rx sRx,
  input t_if_ccip_Tx sTx
);

  cci_mpf_if fiu(.clk(clk));
  cci_mpf_if afu(.clk(clk));

  // CCI-P to MPF interface
  ccip_wires_to_mpf #(
    .REGISTER_INPUTS(1),
    .REGISTER_OUTPUTS(1)
  ) map_ifc (
    .pClk(clk),
    .pck_cp2af_softReset(reset),
    .fiu(fiu),
    .*
  );

  // Factory
  cci_mpf #(
    .DFH_MMIO_BASE_ADDR(MPF_DFH_MMIO_ADDR),
    .ENABLE_VTP(0),
    .ENABLE_VC_MAP(0),
    .ENABLE_DYNAMIC_VC_MAPPING(1),
    .ENFORCE_WR_ORDER(0),
    .SORT_READ_RESPONSES(1),
    .PRESERVE_WRITE_MDATA(0),
    .ENABLE_PARTIAL_WRITES(0)
  ) mpf (
    .clk(clk),
    .fiu,
    .afu,
    .c0NotEmpty(),
    .c1NotEmpty()
  );

  // MPF interface to CCIP
  always_comb begin
    sRx.c0 = afu.c0Rx;
    sRx.c1 = afu.c1Rx;
    sRx.c0TxAlmFull = afu.c0TxAlmFull;
    sRx.c1TxAlmFull = afu.c1TxAlmFull;
    afu.c0Tx = cci_mpf_cvtC0TxFromBase(sTx.c0);
    if (cci_mpf_c0TxIsReadReq(afu.c0Tx)) begin
      afu.c0Tx.hdr.ext.addrIsVirtual = 1'b0;
      afu.c0Tx.hdr.ext.mapVAtoPhysChannel = 1'b1;
      afu.c0Tx.hdr.ext.checkLoadStoreOrder = 1'b1;
    end
    afu.c1Tx = cci_mpf_cvtC1TxFromBase(sTx.c1);
    if (cci_mpf_c1TxIsWriteReq(afu.c1Tx)) begin
      afu.c1Tx.hdr.ext.addrIsVirtual = 1'b0;
      afu.c1Tx.hdr.ext.mapVAtoPhysChannel = 1'b1;
      afu.c1Tx.hdr.ext.checkLoadStoreOrder = 1'b1;
    end
    afu.c2Tx = sTx.c2;
  end

endmodule // ccip_entry


module ccip_std_afu #(
  parameter NUM_LOCAL_MEM_BANKS=2
) (
  input logic pClk,
  input logic pClkDiv2,
  input logic pClkDiv4,
  input logic uClk_usr,
  input logic uClk_usrDiv2,
  input logic pck_cp2af_softReset,
  input logic [1:0] pck_cp2af_pwrState,
  input logic pck_cp2af_error,
  input t_if_ccip_Rx pck_cp2af_sRx,
  output t_if_ccip_Tx pck_af2cp_sTx,
  avalon_mem_if.to_fiu local_mem[NUM_LOCAL_MEM_BANKS]
);

  // PLATFORM_PARAM_LOCAL_MEMORY_ADDR_WIDTH is word address.
  // e.g., 8GB memory stick -> 33b byte address -> 27b word address
  //       PLATFORM_PARAM_LOCAL_MEMORY_ADDR_WIDTH = 27
  // U_MEM_ADDR_WIDTH is word address of total memory
  // e.g., 4 x 8GB memory sticks
  //       U_MEM_ADDR_WIDTH = 27 + 2 = 29
  localparam U_MEM_ADDR_WIDTH = `PLATFORM_PARAM_LOCAL_MEMORY_ADDR_WIDTH + $clog2(NUM_LOCAL_MEM_BANKS);

  logic clk;
  assign clk = `PLATFORM_PARAM_CCI_P_CLOCK; // uClk_usr
  logic reset;
  assign reset = `PLATFORM_PARAM_CCI_P_RESET; // pck_cp2af_softReset

  localparam MPF_DFH_MMIO_ADDR = 24'h1000;

  t_if_ccip_Rx sRx;
  t_if_ccip_Tx sTx;
  ccip_entry #(
    .MPF_DFH_MMIO_ADDR(MPF_DFH_MMIO_ADDR)
  ) ccip_entry_inst (
    .clk(clk),
    .reset(reset),
    .sRx(sRx),
    .sTx(sTx),
    .*
  );

  // mem_0 for kernel, mem_1 for dma engine
  u_mem_if #(.U_MEM_ADDR_WIDTH(U_MEM_ADDR_WIDTH)) mem_0 (.clk(clk), .reset(reset));
  u_mem_if #(.U_MEM_ADDR_WIDTH(U_MEM_ADDR_WIDTH)) mem_1 (.clk(clk), .reset(reset));
  local_mem_entry #(
    .ADDR_WIDTH(U_MEM_ADDR_WIDTH),
    .DATA_WIDTH(`PLATFORM_PARAM_LOCAL_MEMORY_DATA_WIDTH),
    .BURST_WIDTH(`PLATFORM_PARAM_LOCAL_MEMORY_BURST_CNT_WIDTH),
    .NUM_MASTERS(2), // 0 = kernel, 1 = dma engine
    .NUM_SLAVES(NUM_LOCAL_MEM_BANKS)
  ) local_mem_entry_inst (
    .clk(clk),
    .rstn(~reset),
    .u_mem('{mem_0, mem_1}),
    .local_mem(local_mem)
  );

  t_ccip_c0_ReqMmioHdr mmio_req_hdr;
  assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(sRx.c0.hdr);

  sub_csr_if sub_port[3]();
  logic sub_active[3];
  assign sub_active[0] = (mmio_req_hdr.address[15:4] == 12'h000);
  assign sub_active[1] = (mmio_req_hdr.address[15:4] == 12'h001);
  assign sub_active[2] = (mmio_req_hdr.address[15:12] == 4'h1);

  csr_arbiter #(
    .NUM_SUB_CSRS(3)
  ) csr_arbiter_inst (
    .clk(clk),
    .reset(reset),
    .sRx0(sRx.c0),
    .sTx2(sTx.c2),
    .sub_port(sub_port),
    .sub_active(sub_active)
  );

  common_csr #(
    .NEXT_DFH_OFFSET(MPF_DFH_MMIO_ADDR)
  ) common_csr_inst (
    .clk(clk),
    .reset(reset),
    .port(sub_port[0])
  );

  dma_engine #(
    .U_MEM_ADDR_WIDTH(U_MEM_ADDR_WIDTH)
  ) dma_engine_inst (
    .clk(clk),
    .reset(reset),
    .csr(sub_port[1]),
    .sTx0(sTx.c0),
    .sTx1(sTx.c1),
    .sTx0AlmFull(sRx.c0TxAlmFull),
    .sTx1AlmFull(sRx.c1TxAlmFull),
    .sRx0(sRx.c0),
    .sRx1(sRx.c1),
    .local_mem(mem_1)
  );

  logic opencl_rstn;
  logic [7:0] opencl_select;
  logic opencl_on;
  logic opencl_complete;
  logic opencl_clean;
  logic opencl_cleaned;
  logic [31:0] opencl_global_size_0;
  logic [31:0] opencl_global_size_1;
  logic [31:0] opencl_global_size_2;
  logic [10:0] opencl_local_size_0;
  logic [10:0] opencl_local_size_1;
  logic [10:0] opencl_local_size_2;
  logic [31:0] opencl_num_groups_0;
  logic [31:0] opencl_num_groups_1;
  logic [31:0] opencl_num_groups_2;
  logic [63:0] opencl_num_work_items;
  logic [63:0] opencl_num_work_groups;
  logic [10:0] opencl_work_group_size;
  logic [4095:0] opencl_arg;
  logic [4095:0] opencl_pc;

  kernel_controller kernel_controller_inst (
    .clk(clk),
    .reset(reset),
    .csr(sub_port[2]),
    .opencl_rstn(opencl_rstn),
    .opencl_select(opencl_select),
    .opencl_on(opencl_on),
    .opencl_complete(opencl_complete),
    .opencl_clean(opencl_clean),
    .opencl_cleaned(opencl_cleaned),
    .opencl_global_size_0(opencl_global_size_0),
    .opencl_global_size_1(opencl_global_size_1),
    .opencl_global_size_2(opencl_global_size_2),
    .opencl_local_size_0(opencl_local_size_0),
    .opencl_local_size_1(opencl_local_size_1),
    .opencl_local_size_2(opencl_local_size_2),
    .opencl_num_groups_0(opencl_num_groups_0),
    .opencl_num_groups_1(opencl_num_groups_1),
    .opencl_num_groups_2(opencl_num_groups_2),
    .opencl_num_work_items(opencl_num_work_items),
    .opencl_num_work_groups(opencl_num_work_groups),
    .opencl_work_group_size(opencl_work_group_size),
    .opencl_arg(opencl_arg),
    .opencl_pc(opencl_pc)
  );

  logic [U_MEM_ADDR_WIDTH+5:0] mem_0_symbol_address;
  assign mem_0.address = mem_0_symbol_address[U_MEM_ADDR_WIDTH+5:6];
  assign mem_0.burstcount[6:4] = 0;

  kernel kernel_inst (
    .clk(clk),
    .rstn(~reset),
    .clk2x(),
    .opencl_rstn(opencl_rstn),
    .opencl_select(opencl_select),
    .opencl_on(opencl_on),
    .opencl_complete(opencl_complete),
    .opencl_clean(opencl_clean),
    .opencl_cleaned(opencl_cleaned),
    .opencl_global_size_0(opencl_global_size_0),
    .opencl_global_size_1(opencl_global_size_1),
    .opencl_global_size_2(opencl_global_size_2),
    .opencl_local_size_0(opencl_local_size_0),
    .opencl_local_size_1(opencl_local_size_1),
    .opencl_local_size_2(opencl_local_size_2),
    .opencl_num_groups_0(opencl_num_groups_0),
    .opencl_num_groups_1(opencl_num_groups_1),
    .opencl_num_groups_2(opencl_num_groups_2),
    .opencl_num_work_items(opencl_num_work_items),
    .opencl_num_work_groups(opencl_num_work_groups),
    .opencl_work_group_size(opencl_work_group_size),
    .opencl_arg(opencl_arg),
    .opencl_pc(opencl_pc),
    .mem_0_address(mem_0_symbol_address),
    .mem_0_read(mem_0.read),
    .mem_0_readdata(mem_0.readdata),
    .mem_0_readdatavalid(mem_0.readdatavalid),
    .mem_0_write(mem_0.write),
    .mem_0_writedata(mem_0.writedata),
    .mem_0_byteenable(mem_0.byteenable),
    .mem_0_burstcount(mem_0.burstcount[3:0]),
    .mem_0_waitrequest(mem_0.waitrequest)
  );

endmodule // ccip_std_afu
