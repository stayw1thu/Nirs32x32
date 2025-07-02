//Copyright (C)2014-2024 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.9.03 (64-bit)
//Part Number: GW2A-LV55PG484C8/I7
//Device: GW2A-55
//Device Version: C
//Created Time: Thu Jun  6 11:01:13 2024

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

	ft232hq_send_fifo your_instance_name(
		.Data(Data), //input [7:0] Data
		.WrClk(WrClk), //input WrClk
		.RdClk(RdClk), //input RdClk
		.WrEn(WrEn), //input WrEn
		.RdEn(RdEn), //input RdEn
		.Wnum(Wnum), //output [12:0] Wnum
		.Rnum(Rnum), //output [12:0] Rnum
		.Q(Q), //output [7:0] Q
		.Empty(Empty), //output Empty
		.Full(Full) //output Full
	);

//--------Copy end-------------------
