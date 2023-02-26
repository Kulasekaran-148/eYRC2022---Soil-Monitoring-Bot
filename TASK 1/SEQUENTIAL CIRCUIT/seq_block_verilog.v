// Copyright (C) 2019  Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions 
// and other software and tools, and any partner logic 
// functions, and any output files from any of the foregoing 
// (including device programming or simulation files), and any 
// associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License 
// Subscription Agreement, the Intel Quartus Prime License Agreement,
// the Intel FPGA IP License Agreement, or other applicable license
// agreement, including, without limitation, that your use is for
// the sole purpose of programming logic devices manufactured by
// Intel and sold by Intel or its authorized distributors.  Please
// refer to the applicable agreement for further details, at
// https://fpgasoftware.intel.com/eula.

// PROGRAM		"Quartus Prime"
// VERSION		"Version 19.1.0 Build 670 09/22/2019 SJ Lite Edition"
// CREATED		"Sun Dec 19 09:29:54 2021"

module seq_block_verilog(
	CLK,
	Y
);


input wire	CLK;
output wire	[2:0] Y;

wire	[2:0] Q;
wire	SYNTHESIZED_WIRE_0;
wire	SYNTHESIZED_WIRE_16;
wire	SYNTHESIZED_WIRE_2;
wire	SYNTHESIZED_WIRE_17;
wire	SYNTHESIZED_WIRE_18;
wire	SYNTHESIZED_WIRE_6;
wire	SYNTHESIZED_WIRE_7;
wire	SYNTHESIZED_WIRE_8;
wire	SYNTHESIZED_WIRE_9;
wire	SYNTHESIZED_WIRE_13;
wire	SYNTHESIZED_WIRE_14;
wire	SYNTHESIZED_WIRE_15;





T_ff	b2v_inst(
	.T(SYNTHESIZED_WIRE_0),
	.CLK(SYNTHESIZED_WIRE_16),
	.q(Q[2]),
	.q_bar(SYNTHESIZED_WIRE_17));


T_ff	b2v_inst1(
	.T(SYNTHESIZED_WIRE_2),
	.CLK(SYNTHESIZED_WIRE_16),
	.q(Q[1]),
	.q_bar(SYNTHESIZED_WIRE_18));

assign	SYNTHESIZED_WIRE_6 = SYNTHESIZED_WIRE_17 & SYNTHESIZED_WIRE_18 & Q[0];

assign	SYNTHESIZED_WIRE_9 = SYNTHESIZED_WIRE_6 | SYNTHESIZED_WIRE_7 | SYNTHESIZED_WIRE_8;


T_ff	b2v_inst2(
	.T(SYNTHESIZED_WIRE_9),
	.CLK(SYNTHESIZED_WIRE_16),
	.q(Q[0]),
	.q_bar(SYNTHESIZED_WIRE_15));


pll	b2v_inst3(
	.inclk0(CLK),
	.c0(SYNTHESIZED_WIRE_16));

assign	SYNTHESIZED_WIRE_2 = SYNTHESIZED_WIRE_18 | SYNTHESIZED_WIRE_17;

assign	SYNTHESIZED_WIRE_0 = SYNTHESIZED_WIRE_13 | SYNTHESIZED_WIRE_14;

assign	SYNTHESIZED_WIRE_13 = Q[2] ~^ Q[1];

assign	SYNTHESIZED_WIRE_14 = Q[1] & Q[0];

assign	SYNTHESIZED_WIRE_7 = Q[1] & SYNTHESIZED_WIRE_15;

assign	SYNTHESIZED_WIRE_8 = Q[1] & Q[2];

assign	Y = Q;

endmodule
