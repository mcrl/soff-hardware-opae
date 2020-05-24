module kernel (
  input wire clk,
  input wire rstn,
  input wire [7:0] opencl_select,
  input wire opencl_on,
  input wire opencl_clean,
  input wire mem_0_readdatavalid,
  input wire clk2x,
  input wire opencl_rstn,
  output wire opencl_complete,
  output wire opencl_cleaned,
  input wire [31:0] opencl_global_size_0,
  input wire [31:0] opencl_global_size_1,
  input wire [31:0] opencl_global_size_2,
  input wire [8:0] opencl_local_size_0,
  input wire [8:0] opencl_local_size_1,
  input wire [8:0] opencl_local_size_2,
  input wire [31:0] opencl_num_groups_0,
  input wire [31:0] opencl_num_groups_1,
  input wire [31:0] opencl_num_groups_2,
  input wire [63:0] opencl_num_work_items,
  input wire [63:0] opencl_num_work_groups,
  input wire [8:0] opencl_work_group_size,
  input wire [4095:0] opencl_arg,
  output wire [4095:0] opencl_pc,
  output wire [32:0] mem_0_address,
  output wire mem_0_read,
  input wire [511:0] mem_0_readdata,
  output wire mem_0_write,
  output wire [511:0] mem_0_writedata,
  output wire [63:0] mem_0_byteenable,
  output wire [3:0] mem_0_burstcount,
  input wire mem_0_waitrequest
);

  assign opencl_complete = 0;
  assign opencl_cleaned = 0;
  assign mem_0_read = 0;
  assign mem_0_write = 0;

endmodule
