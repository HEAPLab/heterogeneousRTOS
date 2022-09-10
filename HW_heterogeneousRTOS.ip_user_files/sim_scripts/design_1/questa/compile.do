vlib questa_lib/work
vlib questa_lib/msim

vlib questa_lib/msim/xilinx_vip
vlib questa_lib/msim/xpm
vlib questa_lib/msim/axi_infrastructure_v1_1_0
vlib questa_lib/msim/axi_vip_v1_1_12
vlib questa_lib/msim/processing_system7_vip_v1_0_14
vlib questa_lib/msim/xil_defaultlib
vlib questa_lib/msim/lib_cdc_v1_0_2
vlib questa_lib/msim/proc_sys_reset_v5_0_13
vlib questa_lib/msim/generic_baseblocks_v2_1_0
vlib questa_lib/msim/fifo_generator_v13_2_7
vlib questa_lib/msim/axi_data_fifo_v2_1_25
vlib questa_lib/msim/axi_register_slice_v2_1_26
vlib questa_lib/msim/axi_protocol_converter_v2_1_26
vlib questa_lib/msim/axi_clock_converter_v2_1_25
vlib questa_lib/msim/blk_mem_gen_v8_4_5
vlib questa_lib/msim/axi_dwidth_converter_v2_1_26

vmap xilinx_vip questa_lib/msim/xilinx_vip
vmap xpm questa_lib/msim/xpm
vmap axi_infrastructure_v1_1_0 questa_lib/msim/axi_infrastructure_v1_1_0
vmap axi_vip_v1_1_12 questa_lib/msim/axi_vip_v1_1_12
vmap processing_system7_vip_v1_0_14 questa_lib/msim/processing_system7_vip_v1_0_14
vmap xil_defaultlib questa_lib/msim/xil_defaultlib
vmap lib_cdc_v1_0_2 questa_lib/msim/lib_cdc_v1_0_2
vmap proc_sys_reset_v5_0_13 questa_lib/msim/proc_sys_reset_v5_0_13
vmap generic_baseblocks_v2_1_0 questa_lib/msim/generic_baseblocks_v2_1_0
vmap fifo_generator_v13_2_7 questa_lib/msim/fifo_generator_v13_2_7
vmap axi_data_fifo_v2_1_25 questa_lib/msim/axi_data_fifo_v2_1_25
vmap axi_register_slice_v2_1_26 questa_lib/msim/axi_register_slice_v2_1_26
vmap axi_protocol_converter_v2_1_26 questa_lib/msim/axi_protocol_converter_v2_1_26
vmap axi_clock_converter_v2_1_25 questa_lib/msim/axi_clock_converter_v2_1_25
vmap blk_mem_gen_v8_4_5 questa_lib/msim/blk_mem_gen_v8_4_5
vmap axi_dwidth_converter_v2_1_26 questa_lib/msim/axi_dwidth_converter_v2_1_26

vlog -work xilinx_vip -64 -incr -mfcu -sv -L axi_vip_v1_1_12 -L processing_system7_vip_v1_0_14 -L xilinx_vip "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/axi4stream_vip_axi4streampc.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/axi_vip_axi4pc.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/xil_common_vip_pkg.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/axi4stream_vip_pkg.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/axi_vip_pkg.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/axi4stream_vip_if.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/axi_vip_if.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/clk_vip_if.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/hdl/rst_vip_if.sv" \

vlog -work xpm -64 -incr -mfcu -sv -L axi_vip_v1_1_12 -L processing_system7_vip_v1_0_14 -L xilinx_vip "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm -64 -93 \
"/home/francesco/tools/Xilinx/Vivado/2022.1/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work axi_infrastructure_v1_1_0 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl/axi_infrastructure_v1_1_vl_rfs.v" \

vlog -work axi_vip_v1_1_12 -64 -incr -mfcu -sv -L axi_vip_v1_1_12 -L processing_system7_vip_v1_0_14 -L xilinx_vip "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/1033/hdl/axi_vip_v1_1_vl_rfs.sv" \

vlog -work processing_system7_vip_v1_0_14 -64 -incr -mfcu -sv -L axi_vip_v1_1_12 -L processing_system7_vip_v1_0_14 -L xilinx_vip "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl/processing_system7_vip_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../bd/design_1/ip/design_1_processing_system7_0_0/sim/design_1_processing_system7_0_0.v" \

vcom -work lib_cdc_v1_0_2 -64 -93 \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ef1e/hdl/lib_cdc_v1_0_rfs.vhd" \

vcom -work proc_sys_reset_v5_0_13 -64 -93 \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/8842/hdl/proc_sys_reset_v5_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -64 -93 \
"../../../bd/design_1/ip/design_1_rst_ps7_0_100M_0/sim/design_1_rst_ps7_0_100M_0.vhd" \

vlog -work xil_defaultlib -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../bd/design_1/ipshared/495a/hdl/scheduler_v1_0_M_AXI.v" \
"../../../bd/design_1/ipshared/495a/hdl/scheduler_v1_0_S_AXI.v" \
"../../../bd/design_1/ipshared/495a/hdl/scheduler_v1_0.v" \
"../../../bd/design_1/ip/design_1_scheduler_0_1/sim/design_1_scheduler_0_1.v" \
"../../../bd/design_1/ip/design_1_scheduler_0_0/sim/design_1_scheduler_0_0.v" \
"../../../bd/design_1/ip/design_1_scheduler_0_2/sim/design_1_scheduler_0_2.v" \
"../../../bd/design_1/ipshared/2165/src/MajorityVoter.v" \
"../../../bd/design_1/ipshared/2165/hdl/aximvoter_v1_0.v" \
"../../../bd/design_1/ip/design_1_aximvoter_0_1/sim/design_1_aximvoter_0_1.v" \
"../../../bd/design_1/ipshared/9494/hdl/axisvoter_v1_0.v" \
"../../../bd/design_1/ip/design_1_axisvoter_0_0/sim/design_1_axisvoter_0_0.v" \

vlog -work generic_baseblocks_v2_1_0 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/b752/hdl/generic_baseblocks_v2_1_vl_rfs.v" \

vlog -work fifo_generator_v13_2_7 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/83df/simulation/fifo_generator_vlog_beh.v" \

vcom -work fifo_generator_v13_2_7 -64 -93 \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/83df/hdl/fifo_generator_v13_2_rfs.vhd" \

vlog -work fifo_generator_v13_2_7 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/83df/hdl/fifo_generator_v13_2_rfs.v" \

vlog -work axi_data_fifo_v2_1_25 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5390/hdl/axi_data_fifo_v2_1_vl_rfs.v" \

vlog -work axi_register_slice_v2_1_26 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/0a3f/hdl/axi_register_slice_v2_1_vl_rfs.v" \

vlog -work axi_protocol_converter_v2_1_26 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/90c8/hdl/axi_protocol_converter_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../bd/design_1/ip/design_1_auto_pc_0/sim/design_1_auto_pc_0.v" \
"../../../bd/design_1/ip/design_1_auto_pc_1/sim/design_1_auto_pc_1.v" \

vlog -work axi_clock_converter_v2_1_25 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/e893/hdl/axi_clock_converter_v2_1_vl_rfs.v" \

vlog -work blk_mem_gen_v8_4_5 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/25a8/simulation/blk_mem_gen_v8_4.v" \

vlog -work axi_dwidth_converter_v2_1_26 -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/b3c7/hdl/axi_dwidth_converter_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib -64 -incr -mfcu "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/ec67/hdl" "+incdir+../../../../HW_heterogeneousRTOS.gen/sources_1/bd/design_1/ipshared/5765/hdl" "+incdir+/home/francesco/tools/Xilinx/Vivado/2022.1/data/xilinx_vip/include" \
"../../../bd/design_1/ip/design_1_auto_us_0/sim/design_1_auto_us_0.v" \
"../../../bd/design_1/sim/design_1.v" \

vlog -work xil_defaultlib \
"glbl.v"

