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
`include "csr.vh"
`include "u_mem.vh"

typedef enum logic [1:0] {
  DMA_STATE_IDLE,
  DMA_STATE_READ,
  DMA_STATE_WRITE,
  DMA_STATE_RESPONSE
} dma_state_t;

module dma_engine #(
  parameter U_MEM_ADDR_WIDTH = 32
) (
  input logic clk,
  input logic reset,
  sub_csr_if.to_afu csr,
  output t_if_ccip_c0_Tx sTx0,
  output t_if_ccip_c1_Tx sTx1,
  input logic sTx0AlmFull,
  input logic sTx1AlmFull,
  input t_if_ccip_c0_Rx sRx0,
  input t_if_ccip_c1_Rx sRx1,
  u_mem_if.to_fiu local_mem
);

  typedef logic [U_MEM_ADDR_WIDTH - 1 : 0] dma_fpga_addr_t;
  typedef t_ccip_clAddr dma_cpu_addr_t;
  typedef logic [U_MEM_ADDR_WIDTH - 1 : 0] dma_length_t;

  logic dma_read_trigger; // CPU -> FPGA
  logic dma_write_trigger; // FPGA -> CPU
  logic dma_idle_trigger;
  dma_fpga_addr_t dma_fpga_base;
  dma_cpu_addr_t dma_cpu_base;
  dma_length_t dma_length;
  dma_state_t state;

  always_ff @(posedge clk) begin
    if (reset) begin
      csr.tx.mmioRdValid <= 0;
      dma_read_trigger <= 1'b0;
      dma_write_trigger <= 1'b0;
      dma_idle_trigger <= 1'b0;
      dma_fpga_base <= '0;
      dma_cpu_base <= '0;
      dma_length <= '0;
    end
    else begin
      dma_read_trigger <= 1'b0;
      dma_write_trigger <= 1'b0;
      dma_idle_trigger <= 1'b0;

      csr.tx.mmioRdValid <= csr.rx.rdValid;
      csr.tx.hdr.tid <= csr.rx.hdr.tid;
      if (csr.rx.rdValid) begin
        case (csr.rx.hdr.address)
          16'h0010: begin
            csr.tx.data <= {62'b0, state};
            if (state == DMA_STATE_RESPONSE) begin
              dma_idle_trigger <= 1'b1;
            end
          end
          default: csr.tx.data <= t_ccip_mmioData'(0);
        endcase
      end

      if (csr.rx.wrValid) begin
        case (csr.rx.hdr.address)
          16'h0012: dma_read_trigger <= 1'b1;
          16'h0014: dma_write_trigger <= 1'b1;
          16'h0016: dma_fpga_base <= dma_fpga_addr_t'(csr.rx.data[63:6]);
          16'h0018: dma_cpu_base <= dma_cpu_addr_t'(csr.rx.data[63:6]);
          16'h001a: dma_length <= dma_length_t'(csr.rx.data);
        endcase
      end
    end
  end

  dma_fpga_addr_t fpga_addr;
  dma_length_t fpga_remained;
  dma_cpu_addr_t cpu_addr;
  dma_length_t cpu_remained;

  localparam QUEUE_SIZE = 1024;
  localparam QUEUE_SIZE_WIDTH = $clog2(QUEUE_SIZE);

  logic [511:0] qInput;
  logic [511:0] qOutput;
  logic qPush, qPop;
  logic qEmpty, qAlmFull;

  scfifo #(
    .lpm_width(512),
    .lpm_widthu(QUEUE_SIZE_WIDTH),
    .lpm_numwords(QUEUE_SIZE),
    .lpm_showahead("ON"),
    .lpm_type("SCFIFO"),
    .almost_full_value(QUEUE_SIZE / 2)
  ) queue (
    .clock(clk),
    .sclr(reset),
    .data(qInput),
    .q(qOutput),
    .wrreq(qPush),
    .rdreq(qPop),
    .empty(qEmpty),
    .almost_full(qAlmFull)
  );

  always_comb begin
    sTx0.hdr = 0;
    sTx0.hdr.address = cpu_addr;
    sTx1.hdr = 0;
    sTx1.hdr.sop = 1'b1;
    sTx1.hdr.address = cpu_addr;
    sTx1.data = qOutput;
    local_mem.address = fpga_addr;
    local_mem.burstcount = 1;
    local_mem.writedata = qOutput;
    local_mem.byteenable = 64'hFFFF_FFFF_FFFF_FFFF;

    case (state)
      DMA_STATE_READ: begin
        // CPU -> Queue
        sTx0.valid = (cpu_remained != 0 && ~sTx0AlmFull && ~qAlmFull);
        sTx1.valid = 1'b0;
        qInput = sRx0.data;
        qPush = sRx0.rspValid;
        // Queue -> FPGA
        local_mem.read = 1'b0;
        local_mem.write = ~qEmpty;
        qPop = (local_mem.write && ~local_mem.waitrequest);
      end
      DMA_STATE_WRITE: begin
        // FPGA -> Queue
        local_mem.read = (fpga_remained != 0 && ~qAlmFull);
        local_mem.write = 1'b0;
        qInput = local_mem.readdata;
        qPush = local_mem.readdatavalid;
        // Queue -> CPU
        sTx0.valid = 1'b0;
        sTx1.valid = (~qEmpty && ~sTx1AlmFull);
        qPop = sTx1.valid;
      end
      default: begin
        sTx0.valid = 1'b0;
        sTx1.valid = 1'b0;
        local_mem.read = 1'b0;
        local_mem.write = 1'b0;
        qInput = '0;
        qPush = 1'b0;
        qPop = 1'b0;
      end
    endcase
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      state <= DMA_STATE_IDLE;
      fpga_addr <= '0;
      fpga_remained <= '0;
      cpu_addr <= '0;
      cpu_remained <= '0;
    end
    else begin
      case (state)
        DMA_STATE_IDLE: begin
          if (dma_read_trigger) begin
            state <= DMA_STATE_READ;
            fpga_addr <= dma_fpga_base;
            fpga_remained <= dma_length;
            cpu_addr <= dma_cpu_base;
            cpu_remained <= dma_length;
          end
          else if (dma_write_trigger) begin
            state <= DMA_STATE_WRITE;
            fpga_addr <= dma_fpga_base;
            fpga_remained <= dma_length;
            cpu_addr <= dma_cpu_base;
            cpu_remained <= dma_length;
          end
        end
        DMA_STATE_READ: begin
          if (sTx0.valid) begin
            cpu_addr <= cpu_addr + 1;
            cpu_remained <= cpu_remained - 1;
          end
          if (local_mem.write && ~local_mem.waitrequest) begin
            fpga_addr <= fpga_addr + 1;
            fpga_remained <= fpga_remained - 1;
          end
          if (fpga_remained == 0) begin
            state <= DMA_STATE_RESPONSE;
          end
        end
        DMA_STATE_WRITE: begin
          if (local_mem.read && ~local_mem.waitrequest) begin
            fpga_addr <= fpga_addr + 1;
            fpga_remained <= fpga_remained - 1;
          end
          if (sTx1.valid) begin
            cpu_addr <= cpu_addr + 1;
          end
          if (sRx1.rspValid) begin
            cpu_remained <= cpu_remained - 1;
          end
          if (cpu_remained == 0) begin
            state <= DMA_STATE_RESPONSE;
          end
        end
        DMA_STATE_RESPONSE: begin
          if (dma_idle_trigger) begin
            state <= DMA_STATE_IDLE;
          end
        end
      endcase
    end
  end

endmodule // dma_engine
