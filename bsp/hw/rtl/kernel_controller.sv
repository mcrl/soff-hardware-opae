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

typedef enum logic [3:0] {
  KERNEL_STATE_IDLE = 4'b0000,
  KERNEL_STATE_RUNNING = 4'b0100,
  KERNEL_STATE_CLEANING = 4'b0101,
  KERNEL_STATE_FINISHED = 4'b1000
} kernel_state_t;


module kernel_controller (
  input logic clk,
  input logic reset,
  sub_csr_if.to_afu csr,
  output logic opencl_rstn,
  output logic [7:0] opencl_select,
  output logic opencl_on,
  input logic opencl_complete,
  output logic opencl_clean,
  input logic opencl_cleaned,
  output logic [31:0] opencl_global_size_0,
  output logic [31:0] opencl_global_size_1,
  output logic [31:0] opencl_global_size_2,
  output logic [10:0] opencl_local_size_0,
  output logic [10:0] opencl_local_size_1,
  output logic [10:0] opencl_local_size_2,
  output logic [31:0] opencl_num_groups_0,
  output logic [31:0] opencl_num_groups_1,
  output logic [31:0] opencl_num_groups_2,
  output logic [63:0] opencl_num_work_items,
  output logic [63:0] opencl_num_work_groups,
  output logic [10:0] opencl_work_group_size,
  output logic [4095:0] opencl_arg,
  input logic [4095:0] opencl_pc
);

  kernel_state_t state;
  logic kernel_reset_trigger;
  logic kernel_start_trigger;
  logic kernel_idle_trigger;

  always_ff @(posedge clk) begin
    if (reset) begin
      opencl_rstn <= 1'b0;
      opencl_on <= 1'b0;
      opencl_clean <= 1'b0;
      state <= KERNEL_STATE_IDLE;
    end
    else if (kernel_reset_trigger) begin
      opencl_rstn <= 1'b0;
      opencl_on <= 1'b0;
      opencl_clean <= 1'b0;
      state <= KERNEL_STATE_IDLE;
    end
    else begin
      opencl_rstn <= 1'b1;
      case (state)
        KERNEL_STATE_IDLE: begin
          if (kernel_start_trigger) begin
            opencl_on <= 1'b1;
            state <= KERNEL_STATE_RUNNING;
          end
        end
        KERNEL_STATE_RUNNING: begin
          opencl_on <= 1'b0;
          if (opencl_complete) begin
            opencl_clean <= 1'b1;
            state <= KERNEL_STATE_CLEANING;
          end
        end
        KERNEL_STATE_CLEANING: begin
          opencl_clean <= 1'b0;
          if (opencl_cleaned) begin
            state <= KERNEL_STATE_FINISHED;
          end
        end
        KERNEL_STATE_FINISHED: begin
          if (kernel_idle_trigger) begin
            state <= KERNEL_STATE_IDLE;
          end
        end
      endcase
    end
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      csr.tx.mmioRdValid <= 0;
      kernel_reset_trigger <= 1'b0;
      kernel_start_trigger <= 1'b0;
      kernel_idle_trigger <= 1'b0;
      opencl_select <= '0;
      opencl_global_size_0 <= '0;
      opencl_global_size_1 <= '0;
      opencl_global_size_2 <= '0;
      opencl_local_size_0 <= '0;
      opencl_local_size_1 <= '0;
      opencl_local_size_2 <= '0;
      opencl_num_groups_0 <= '0;
      opencl_num_groups_1 <= '0;
      opencl_num_groups_2 <= '0;
      opencl_num_work_items <= '0;
      opencl_num_work_groups <= '0;
      opencl_work_group_size <= '0;
      opencl_arg <= '0;
    end
    else begin
      kernel_start_trigger <= 1'b0;
      kernel_idle_trigger <= 1'b0;

      csr.tx.mmioRdValid <= csr.rx.rdValid;
      csr.tx.hdr.tid <= csr.rx.hdr.tid;
      if (csr.rx.rdValid) begin
        case (csr.rx.hdr.address)
          16'h1004: begin
            csr.tx.data <= {60'b0, state};
            if (state == KERNEL_STATE_FINISHED) begin
              kernel_idle_trigger <= 1'b1;
            end
          end
          16'h1200: csr.tx.data <= opencl_pc[63:0];
          16'h1202: csr.tx.data <= opencl_pc[127:64];
          16'h1204: csr.tx.data <= opencl_pc[191:128];
          16'h1206: csr.tx.data <= opencl_pc[255:192];
          16'h1208: csr.tx.data <= opencl_pc[319:256];
          16'h120a: csr.tx.data <= opencl_pc[383:320];
          16'h120c: csr.tx.data <= opencl_pc[447:384];
          16'h120e: csr.tx.data <= opencl_pc[511:448];
          16'h1210: csr.tx.data <= opencl_pc[575:512];
          16'h1212: csr.tx.data <= opencl_pc[639:576];
          16'h1214: csr.tx.data <= opencl_pc[703:640];
          16'h1216: csr.tx.data <= opencl_pc[767:704];
          16'h1218: csr.tx.data <= opencl_pc[831:768];
          16'h121a: csr.tx.data <= opencl_pc[895:832];
          16'h121c: csr.tx.data <= opencl_pc[959:896];
          16'h121e: csr.tx.data <= opencl_pc[1023:960];
          16'h1220: csr.tx.data <= opencl_pc[1087:1024];
          16'h1222: csr.tx.data <= opencl_pc[1151:1088];
          16'h1224: csr.tx.data <= opencl_pc[1215:1152];
          16'h1226: csr.tx.data <= opencl_pc[1279:1216];
          16'h1228: csr.tx.data <= opencl_pc[1343:1280];
          16'h122a: csr.tx.data <= opencl_pc[1407:1344];
          16'h122c: csr.tx.data <= opencl_pc[1471:1408];
          16'h122e: csr.tx.data <= opencl_pc[1535:1472];
          16'h1230: csr.tx.data <= opencl_pc[1599:1536];
          16'h1232: csr.tx.data <= opencl_pc[1663:1600];
          16'h1234: csr.tx.data <= opencl_pc[1727:1664];
          16'h1236: csr.tx.data <= opencl_pc[1791:1728];
          16'h1238: csr.tx.data <= opencl_pc[1855:1792];
          16'h123a: csr.tx.data <= opencl_pc[1919:1856];
          16'h123c: csr.tx.data <= opencl_pc[1983:1920];
          16'h123e: csr.tx.data <= opencl_pc[2047:1984];
          16'h1240: csr.tx.data <= opencl_pc[2111:2048];
          16'h1242: csr.tx.data <= opencl_pc[2175:2112];
          16'h1244: csr.tx.data <= opencl_pc[2239:2176];
          16'h1246: csr.tx.data <= opencl_pc[2303:2240];
          16'h1248: csr.tx.data <= opencl_pc[2367:2304];
          16'h124a: csr.tx.data <= opencl_pc[2431:2368];
          16'h124c: csr.tx.data <= opencl_pc[2495:2432];
          16'h124e: csr.tx.data <= opencl_pc[2559:2496];
          16'h1250: csr.tx.data <= opencl_pc[2623:2560];
          16'h1252: csr.tx.data <= opencl_pc[2687:2624];
          16'h1254: csr.tx.data <= opencl_pc[2751:2688];
          16'h1256: csr.tx.data <= opencl_pc[2815:2752];
          16'h1258: csr.tx.data <= opencl_pc[2879:2816];
          16'h125a: csr.tx.data <= opencl_pc[2943:2880];
          16'h125c: csr.tx.data <= opencl_pc[3007:2944];
          16'h125e: csr.tx.data <= opencl_pc[3071:3008];
          16'h1260: csr.tx.data <= opencl_pc[3135:3072];
          16'h1262: csr.tx.data <= opencl_pc[3199:3136];
          16'h1264: csr.tx.data <= opencl_pc[3263:3200];
          16'h1266: csr.tx.data <= opencl_pc[3327:3264];
          16'h1268: csr.tx.data <= opencl_pc[3391:3328];
          16'h126a: csr.tx.data <= opencl_pc[3455:3392];
          16'h126c: csr.tx.data <= opencl_pc[3519:3456];
          16'h126e: csr.tx.data <= opencl_pc[3583:3520];
          16'h1270: csr.tx.data <= opencl_pc[3647:3584];
          16'h1272: csr.tx.data <= opencl_pc[3711:3648];
          16'h1274: csr.tx.data <= opencl_pc[3775:3712];
          16'h1276: csr.tx.data <= opencl_pc[3839:3776];
          16'h1278: csr.tx.data <= opencl_pc[3903:3840];
          16'h127a: csr.tx.data <= opencl_pc[3967:3904];
          16'h127c: csr.tx.data <= opencl_pc[4031:3968];
          16'h127e: csr.tx.data <= opencl_pc[4095:4032];
          default: csr.tx.data <= t_ccip_mmioData'(0);
        endcase
      end

      if (csr.rx.wrValid) begin
        case (csr.rx.hdr.address)
          16'h1000: kernel_reset_trigger <= 1'b1;
          16'h1002: begin
            opencl_select <= csr.rx.data[7:0];
            kernel_start_trigger <= 1'b1;
          end
          16'h1010: opencl_global_size_0 <= csr.rx.data[31:0];
          16'h1012: opencl_global_size_1 <= csr.rx.data[31:0];
          16'h1014: opencl_global_size_2 <= csr.rx.data[31:0];
          16'h1020: opencl_local_size_0 <= csr.rx.data[10:0];
          16'h1022: opencl_local_size_1 <= csr.rx.data[10:0];
          16'h1024: opencl_local_size_2 <= csr.rx.data[10:0];
          16'h1030: opencl_num_groups_0 <= csr.rx.data[31:0];
          16'h1032: opencl_num_groups_1 <= csr.rx.data[31:0];
          16'h1034: opencl_num_groups_2 <= csr.rx.data[31:0];
          16'h1040: opencl_num_work_items <= csr.rx.data;
          16'h1042: opencl_num_work_groups <= csr.rx.data;
          16'h1044: opencl_work_group_size <= csr.rx.data[10:0];
          16'h1100: opencl_arg[63:0] <= csr.rx.data;
          16'h1102: opencl_arg[127:64] <= csr.rx.data;
          16'h1104: opencl_arg[191:128] <= csr.rx.data;
          16'h1106: opencl_arg[255:192] <= csr.rx.data;
          16'h1108: opencl_arg[319:256] <= csr.rx.data;
          16'h110a: opencl_arg[383:320] <= csr.rx.data;
          16'h110c: opencl_arg[447:384] <= csr.rx.data;
          16'h110e: opencl_arg[511:448] <= csr.rx.data;
          16'h1110: opencl_arg[575:512] <= csr.rx.data;
          16'h1112: opencl_arg[639:576] <= csr.rx.data;
          16'h1114: opencl_arg[703:640] <= csr.rx.data;
          16'h1116: opencl_arg[767:704] <= csr.rx.data;
          16'h1118: opencl_arg[831:768] <= csr.rx.data;
          16'h111a: opencl_arg[895:832] <= csr.rx.data;
          16'h111c: opencl_arg[959:896] <= csr.rx.data;
          16'h111e: opencl_arg[1023:960] <= csr.rx.data;
          16'h1120: opencl_arg[1087:1024] <= csr.rx.data;
          16'h1122: opencl_arg[1151:1088] <= csr.rx.data;
          16'h1124: opencl_arg[1215:1152] <= csr.rx.data;
          16'h1126: opencl_arg[1279:1216] <= csr.rx.data;
          16'h1128: opencl_arg[1343:1280] <= csr.rx.data;
          16'h112a: opencl_arg[1407:1344] <= csr.rx.data;
          16'h112c: opencl_arg[1471:1408] <= csr.rx.data;
          16'h112e: opencl_arg[1535:1472] <= csr.rx.data;
          16'h1130: opencl_arg[1599:1536] <= csr.rx.data;
          16'h1132: opencl_arg[1663:1600] <= csr.rx.data;
          16'h1134: opencl_arg[1727:1664] <= csr.rx.data;
          16'h1136: opencl_arg[1791:1728] <= csr.rx.data;
          16'h1138: opencl_arg[1855:1792] <= csr.rx.data;
          16'h113a: opencl_arg[1919:1856] <= csr.rx.data;
          16'h113c: opencl_arg[1983:1920] <= csr.rx.data;
          16'h113e: opencl_arg[2047:1984] <= csr.rx.data;
          16'h1140: opencl_arg[2111:2048] <= csr.rx.data;
          16'h1142: opencl_arg[2175:2112] <= csr.rx.data;
          16'h1144: opencl_arg[2239:2176] <= csr.rx.data;
          16'h1146: opencl_arg[2303:2240] <= csr.rx.data;
          16'h1148: opencl_arg[2367:2304] <= csr.rx.data;
          16'h114a: opencl_arg[2431:2368] <= csr.rx.data;
          16'h114c: opencl_arg[2495:2432] <= csr.rx.data;
          16'h114e: opencl_arg[2559:2496] <= csr.rx.data;
          16'h1150: opencl_arg[2623:2560] <= csr.rx.data;
          16'h1152: opencl_arg[2687:2624] <= csr.rx.data;
          16'h1154: opencl_arg[2751:2688] <= csr.rx.data;
          16'h1156: opencl_arg[2815:2752] <= csr.rx.data;
          16'h1158: opencl_arg[2879:2816] <= csr.rx.data;
          16'h115a: opencl_arg[2943:2880] <= csr.rx.data;
          16'h115c: opencl_arg[3007:2944] <= csr.rx.data;
          16'h115e: opencl_arg[3071:3008] <= csr.rx.data;
          16'h1160: opencl_arg[3135:3072] <= csr.rx.data;
          16'h1162: opencl_arg[3199:3136] <= csr.rx.data;
          16'h1164: opencl_arg[3263:3200] <= csr.rx.data;
          16'h1166: opencl_arg[3327:3264] <= csr.rx.data;
          16'h1168: opencl_arg[3391:3328] <= csr.rx.data;
          16'h116a: opencl_arg[3455:3392] <= csr.rx.data;
          16'h116c: opencl_arg[3519:3456] <= csr.rx.data;
          16'h116e: opencl_arg[3583:3520] <= csr.rx.data;
          16'h1170: opencl_arg[3647:3584] <= csr.rx.data;
          16'h1172: opencl_arg[3711:3648] <= csr.rx.data;
          16'h1174: opencl_arg[3775:3712] <= csr.rx.data;
          16'h1176: opencl_arg[3839:3776] <= csr.rx.data;
          16'h1178: opencl_arg[3903:3840] <= csr.rx.data;
          16'h117a: opencl_arg[3967:3904] <= csr.rx.data;
          16'h117c: opencl_arg[4031:3968] <= csr.rx.data;
          16'h117e: opencl_arg[4095:4032] <= csr.rx.data;
        endcase
      end
    end
  end

endmodule
