//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.11.01 (64-bit)
//Part Number: GW2A-LV55PG484C8/I7
//Device: GW2A-55
//Device Version: C
//Created Time: Sat Jun  7 01:23:54 2025

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

	ad1278_fifo your_instance_name(
		.Data(Data), //input [191:0] Data
		.WrClk(WrClk), //input WrClk
		.RdClk(RdClk), //input RdClk
		.WrEn(WrEn), //input WrEn
		.RdEn(RdEn), //input RdEn
		.Q(Q), //output [191:0] Q
		.Empty(Empty), //output Empty
		.Full(Full) //output Full
	);

//--------Copy end-------------------
