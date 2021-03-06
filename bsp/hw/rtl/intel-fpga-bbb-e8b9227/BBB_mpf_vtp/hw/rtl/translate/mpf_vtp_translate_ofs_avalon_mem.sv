//
// Copyright (c) 2020, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// Translate Avalon requests from virtual addresses using VTP.
//

`include "ofs_plat_if.vh"
`include "cci_mpf_if.vh"

module mpf_vtp_translate_ofs_avalon_mem
  #(
    // In normal mode, when FAIL_ON_ERROR is non-zero, all translations must
    // be successful. To aid in debugging, failures will block the pipeline
    // and no further traffic will be allowed on the channel.
    //
    // When FAIL_ON_ERROR is 0, the logic controlling host_mem_if must handle
    // the translation failure. When no translation is found, the channel's
    // error bit is set, the request still flows to host_mem_if, and the
    // address in host_mem_if is unchanged from the value in host_mem_va_if.
    parameter FAIL_ON_ERROR = 1
    )
   (
    // FIU interface -- IOVA or physical addresses
    ofs_plat_avalon_mem_if.to_slave host_mem_if,
    // AFU interface -- virtual addresses
    ofs_plat_avalon_mem_if.to_master host_mem_va_if,

    // Translation error signal. When FAIL_ON_ERROR is 0, the consumer of
    // host_mem_if is expected to note the error bit and handle the failure.
    // Errors are raised for the duration of failing bursts in order to
    // simplify routing logic.
    output  logic error,

    // One port for reads, the other for writes. We split the ports because
    // read and write address streams are typically disjoint.
    mpf_vtp_port_if.to_slave vtp_ports[2]
    );

    import mpf_vtp_pkg::*;

    logic clk;
    assign clk = host_mem_if.clk;
    logic reset_n;
    assign reset_n = host_mem_if.reset_n;

    localparam ADDR_WIDTH = host_mem_va_if.ADDR_WIDTH_;
    localparam DATA_WIDTH = host_mem_va_if.DATA_WIDTH_;
    localparam DATA_N_BYTES = host_mem_va_if.DATA_N_BYTES;
    localparam BURST_CNT_WIDTH = host_mem_va_if.BURST_CNT_WIDTH_;

    // Connect responses (no translation needed)
    assign host_mem_va_if.readdata = host_mem_if.readdata;
    assign host_mem_va_if.readdatavalid = host_mem_if.readdatavalid;
    assign host_mem_va_if.response = host_mem_if.response;
    assign host_mem_va_if.writeresponsevalid = host_mem_if.writeresponsevalid;
    assign host_mem_va_if.writeresponse = host_mem_if.writeresponse;

    // Meta-data for translations (tells VTP what to do and returns result)
    t_mpf_vtp_port_wrapper_req vtp_rd_req, vtp_wr_req;
    t_mpf_vtp_port_wrapper_rsp vtp_rd_rsp, vtp_wr_rsp;

    // Flow control
    logic vtp_rd_full, vtp_wr_full, vtp_order_notFull;
    assign host_mem_va_if.waitrequest = vtp_rd_full || vtp_wr_full || !vtp_order_notFull;


    // ====================================================================
    //
    //  Sequencer
    //
    // ====================================================================

    // Keep translated reads and writes in the same relative order on
    // the outbound Avalon channel. This FIFO tracks whether the next
    // operation is a read or a write.

    logic rsp_do_read, rsp_do_write;
    assign rsp_do_write = !rsp_do_read;

    logic rd_deq_en;
    logic wr_deq_en;

    cci_mpf_prim_fifo_lutram
      #(
        .N_ENTRIES(MPF_VTP_MAX_SVC_REQS * 2),
        .N_DATA_BITS(1),
        .REGISTER_OUTPUT(1)
        )
      rd_wr_order
       (
        .clk,
        .reset(!reset_n),

        .enq_data(host_mem_va_if.read),
        .enq_en((host_mem_va_if.read || host_mem_va_if.write) && !host_mem_va_if.waitrequest),
        .notFull(vtp_order_notFull),
        .almostFull(),

        .first(rsp_do_read),
        .deq_en(rd_deq_en || wr_deq_en),
        .notEmpty()
        );


    // ====================================================================
    //
    //  Reads
    //
    // ====================================================================

    always_comb
    begin
        vtp_rd_req = '0;
        vtp_rd_req.addr = host_mem_va_if.address;
        // All reads have virtual addresses in this AFU
        vtp_rd_req.addrIsVirtual = 1'b1;
        vtp_rd_req.isSpeculative = (FAIL_ON_ERROR == 0);
    end

    localparam RD_OPAQUE_BITS = BURST_CNT_WIDTH +  // burstcount
                                DATA_N_BYTES;      // byteenable

    logic rd_rsp_valid;
    logic rd_rsp_used_vtp;

    logic [ADDR_WIDTH-1:0] rd_rsp_address;
    logic [BURST_CNT_WIDTH-1:0] rd_rsp_burstcount;
    logic [DATA_N_BYTES-1:0] rd_rsp_byteenable;

    mpf_vtp_translate_chan
      #(
        .N_OPAQUE_BITS(RD_OPAQUE_BITS)
        )
      rd
       (
        .clk,
        .reset(!reset_n),

        .rsp_valid(rd_rsp_valid),
        .opaque_rsp({ rd_rsp_burstcount,
                      rd_rsp_byteenable }),
        .deq_en(rd_deq_en),

        .req_valid(host_mem_va_if.read && !host_mem_va_if.waitrequest),
        .opaque_req({ host_mem_va_if.burstcount,
                      host_mem_va_if.byteenable }),
        .full(vtp_rd_full),

        .vtp_req(vtp_rd_req),
        .vtp_rsp(vtp_rd_rsp),
        .rsp_used_vtp(rd_rsp_used_vtp),

        .vtp_port(vtp_ports[0])
        );

    assign rd_deq_en = rsp_do_read && rd_rsp_valid && ! host_mem_if.waitrequest;
    assign rd_rsp_address = (rd_rsp_used_vtp ? vtp_rd_rsp.addr : '0);
    // Translation error?
    assign rd_error = rd_rsp_valid && (rd_rsp_used_vtp ? vtp_rd_rsp.error : 1'b0);

    generate
        if (FAIL_ON_ERROR != 0)
        begin : rne
            // Errors block pipeline. Discard reads that fail translation.
            assign host_mem_if.read = rsp_do_read && rd_rsp_valid && ! rd_error;
        end
        else
        begin : re
            // Pass even failed translations to host_mem_if (rd_error raised
            // when needed).
            assign host_mem_if.read = rsp_do_read && rd_rsp_valid;
        end
    endgenerate


    // ====================================================================
    //
    //  Writes
    //
    // ====================================================================

    // Track SOP -- only translate the address at SOP
    logic wr_sop;
    logic wr_eop;

    ofs_plat_prim_burstcount1_sop_tracker
      #(
        .BURST_CNT_WIDTH(BURST_CNT_WIDTH)
        )
      sop
       (
        .clk,
        .reset_n,
        .flit_valid(host_mem_va_if.write && ! host_mem_va_if.waitrequest),
        .burstcount(host_mem_va_if.burstcount),
        .sop(wr_sop),
        .eop(wr_eop)
        );

    always_comb
    begin
        vtp_wr_req = '0;
        vtp_wr_req.addr = host_mem_va_if.address;
        // All reads have virtual addresses in this AFU
        vtp_wr_req.addrIsVirtual = wr_sop;
        vtp_wr_req.isSpeculative = (FAIL_ON_ERROR == 0);
    end

    localparam WR_OPAQUE_BITS = BURST_CNT_WIDTH +  // wr_burstcount
                                DATA_WIDTH +       // wr_writedata
                                DATA_N_BYTES +     // wr_byteenable
                                1;                 // wr_eop

    logic wr_rsp_valid;
    logic wr_rsp_used_vtp;
    logic wr_rsp_eop;
    logic wr_rsp_reg_error;

    logic [ADDR_WIDTH-1:0] wr_rsp_address;
    logic [BURST_CNT_WIDTH-1:0] wr_rsp_burstcount;
    logic [DATA_N_BYTES-1:0] wr_rsp_byteenable;

    mpf_vtp_translate_chan
      #(
        .N_OPAQUE_BITS(WR_OPAQUE_BITS),
        .USE_LARGE_FIFO(1)
        )
      wr
       (
        .clk,
        .reset(!reset_n),

        .rsp_valid(wr_rsp_valid),
        .opaque_rsp({ wr_rsp_burstcount,
                      host_mem_if.writedata,
                      wr_rsp_byteenable,
                      wr_rsp_eop }),
        .deq_en(wr_deq_en),

        .req_valid(host_mem_va_if.write && !host_mem_va_if.waitrequest),
        .opaque_req({ host_mem_va_if.burstcount,
                      host_mem_va_if.writedata,
                      host_mem_va_if.byteenable,
                      wr_eop }),
        .full(vtp_wr_full),

        .vtp_req(vtp_wr_req),
        .vtp_rsp(vtp_wr_rsp),
        .rsp_used_vtp(wr_rsp_used_vtp),

        .vtp_port(vtp_ports[1])
        );

    assign wr_deq_en = rsp_do_write && wr_rsp_valid && ! host_mem_if.waitrequest;
    assign wr_rsp_address = (wr_rsp_used_vtp ? vtp_wr_rsp.addr : '0);
    // Translation error? Raise only on SOP beats (that is when VTP is used).
    assign wr_error = wr_rsp_valid && (wr_rsp_used_vtp ? vtp_wr_rsp.error : wr_rsp_reg_error);

    // Hold the VTP error response for an entire burst
    always_ff @(posedge clk)
    begin
        // A new response from the VTP service (as opposed to non-SOP bursts)?
        if (wr_rsp_used_vtp)
        begin
            wr_rsp_reg_error <= vtp_wr_rsp.error;
        end

        // Turn off error at EOP
        if (wr_rsp_eop)
        begin
            wr_rsp_reg_error <= 1'b0;
        end

        if (!reset_n)
        begin
            wr_rsp_reg_error <= 1'b0;
        end
    end

    generate
        if (FAIL_ON_ERROR != 0)
        begin : wne
            // Errors block pipeline. Discard reads that fail translation.
            assign host_mem_if.write = rsp_do_write && wr_rsp_valid && ! wr_error;
        end
        else
        begin : we
            // Pass even failed translations to host_mem_if (wr_error raised
            // when needed).
            assign host_mem_if.write = rsp_do_write && wr_rsp_valid;
        end
    endgenerate


    // ====================================================================
    //
    //  Merge translated requests into the shared Avalon port
    //
    // ====================================================================

    always_comb
    begin
        if (rsp_do_read)
        begin
            host_mem_if.address = rd_rsp_address;
            host_mem_if.burstcount = rd_rsp_burstcount;
            host_mem_if.byteenable = rd_rsp_byteenable;
            error = rd_error;
        end
        else
        begin
            host_mem_if.address = wr_rsp_address;
            host_mem_if.burstcount = wr_rsp_burstcount;
            host_mem_if.byteenable = wr_rsp_byteenable;
            error = wr_error;
        end
    end


    // ====================================================================
    //
    // Debugging
    //
    // ====================================================================

    // synthesis translate_off
    always_ff @(negedge clk)
    begin
        if (reset_n && (FAIL_ON_ERROR != 0))
        begin
            if (rd_deq_en && rd_rsp_used_vtp && vtp_rd_rsp.error)
            begin
                $fatal(2, "%m VTP: %0t Translation error on RD from VA 0x%x",
                       $time,
                       {vtp_rd_rsp.addr, 6'b0});
            end

            if (wr_deq_en && wr_rsp_used_vtp && vtp_wr_rsp.error)
            begin
                $fatal(2, "%m VTP: %0t Translation error on WR from VA 0x%x",
                       $time,
                       {vtp_wr_rsp.addr, 6'b0});
            end
        end
    end
    // synthesis translate_on

endmodule // mpf_vtp_translate_ofs_avalon_mem
