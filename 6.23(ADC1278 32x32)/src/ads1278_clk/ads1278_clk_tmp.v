//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.11.01 (64-bit)
//Part Number: GW2A-LV55PG484C8/I7
//Device: GW2A-55
//Device Version: C
//Created Time: Mon Jun  9 15:56:52 2025

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    ads1278_clk your_instance_name(
        .clkout(clkout), //output clkout
        .lock(lock), //output lock
        .reset(reset), //input reset
        .clkin(clkin) //input clkin
    );

//--------Copy end-------------------
