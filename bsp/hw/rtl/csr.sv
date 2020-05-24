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
`include "csr.vh"


module csr_arbiter #(
  parameter NUM_SUB_CSRS
) (
  input logic clk,
  input logic reset,
  input t_if_ccip_c0_Rx sRx0,
  output t_if_ccip_c2_Tx sTx2,
  sub_csr_if.to_sub sub_port[NUM_SUB_CSRS],
  input logic sub_active[NUM_SUB_CSRS]
);

  t_ccip_c0_ReqMmioHdr mmio_req_hdr;
  assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(sRx0.hdr);

  genvar ir;
  generate
    for (ir = 0; ir < NUM_SUB_CSRS; ir = ir + 1) begin: sub_rx
      always_comb begin
        sub_port[ir].rx.hdr = mmio_req_hdr;
        sub_port[ir].rx.data = t_ccip_mmioData'(sRx0.data);
        sub_port[ir].rx.rdValid = sRx0.mmioRdValid & sub_active[ir];
        sub_port[ir].rx.wrValid = sRx0.mmioWrValid & sub_active[ir];
      end
    end
  endgenerate

  logic sub_mmioRdValid[NUM_SUB_CSRS];
  t_ccip_c2_RspMmioHdr sub_hdr[NUM_SUB_CSRS];
  t_ccip_mmioData sub_data[NUM_SUB_CSRS];

  for (genvar i = 0; i < NUM_SUB_CSRS; ++i) begin
    assign sub_mmioRdValid[i] = sub_port[i].tx.mmioRdValid;
    assign sub_hdr[i] = sub_port[i].tx.hdr;
    assign sub_data[i] = sub_port[i].tx.data;
  end

  logic rsp_valid;
  t_ccip_c2_RspMmioHdr rsp_hdr;
  t_ccip_mmioData rsp_data;

  always_comb begin
    rsp_valid = 0;
    rsp_hdr ='x;
    rsp_data = 'x;
    for (int i = 0; i < NUM_SUB_CSRS; ++i) begin
      if (sub_mmioRdValid[i]) begin
        rsp_valid = 1;
        rsp_hdr = sub_hdr[i];
        rsp_data = sub_data[i];
      end
    end
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      sTx2.mmioRdValid <= 0;
    end
    else begin
      sTx2.mmioRdValid <= rsp_valid;
      sTx2.hdr <= rsp_hdr;
      sTx2.data <= rsp_data;
    end
  end

endmodule // csr_arbiter


module common_csr #(
  parameter NEXT_DFH_OFFSET = 24'h0000
) (
  input logic clk,
  input logic reset,
  sub_csr_if.to_afu port
);

  logic [127:0] afu_id = `AFU_ACCEL_UUID;

  always_ff @(posedge clk) begin
    if (reset) begin
      port.tx.mmioRdValid <= 0;
    end
    else begin
      port.tx.mmioRdValid <= port.rx.rdValid;
      port.tx.hdr.tid <= port.rx.hdr.tid;
      if (port.rx.rdValid) begin
        case (port.rx.hdr.address)
          16'h0000: port.tx.data <= {
            4'b0001, // feature type
            8'b0, // reserved
            4'b0, // afu minor revision
            7'b0, // reserved
            1'b1, // end of DFH list
            NEXT_DFH_OFFSET, // next DFH offset
            4'b0, // afu major revision
            12'b0 // feature ID
          };
          16'h0002: port.tx.data <= afu_id[63:0]; // AFU_ID_L
          16'h0004: port.tx.data <= afu_id[127:64]; // AFU_ID_H
          16'h0006: port.tx.data <= t_ccip_mmioData'(0); // reserved
          16'h0008: port.tx.data <= t_ccip_mmioData'(0); // reserved
          default: port.tx.data <= t_ccip_mmioData'(0);
        endcase
      end
    end
  end

endmodule // common_csr
