//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.11.01 (64-bit) 
//Created Time: 2025-06-17 16:32:03
create_clock -name sys_clk -period 20 -waveform {0 10} [get_ports {sys_clk}]
create_clock -name ft232_clk -period 16.667 -waveform {0 8.334} [get_ports {ft232_clk}]
create_clock -name o_ads1278_sclk_1 -period 160 -waveform {0 74.074} [get_ports {o_ads1278_sclk_1}]
create_clock -name o_ads1278_clk_1 -period 40 -waveform {0 18.518} [get_ports {o_ads1278_clk_1}]
create_clock -name o_ads1278_sclk_2 -period 160 -waveform {0 74.074} [get_ports {o_ads1278_sclk_2}]
create_clock -name o_ads1278_clk_2 -period 40 -waveform {0 18.518} [get_ports {o_ads1278_clk_2}]
create_clock -name o_ads1278_sclk_3 -period 160 -waveform {0 74.074} [get_ports {o_ads1278_sclk_3}]
create_clock -name o_ads1278_clk_3 -period 40 -waveform {0 18.518} [get_ports {o_ads1278_clk_3}]
create_clock -name o_ads1278_sclk_4 -period 160 -waveform {0 74.074} [get_ports {o_ads1278_sclk_4}]
create_clock -name o_ads1278_clk_4 -period 40 -waveform {0 18.518} [get_ports {o_ads1278_clk_4}]
set_clock_groups -asynchronous -group [get_clocks {sys_clk}] -group [get_clocks {ft232_clk}]
set_clock_groups -exclusive -group [get_clocks {o_ads1278_sclk_1 o_ads1278_clk_1}]
set_clock_groups -exclusive -group [get_clocks {o_ads1278_sclk_2 o_ads1278_clk_2}]
set_clock_groups -exclusive -group [get_clocks {o_ads1278_sclk_3 o_ads1278_clk_3}]
set_clock_groups -exclusive -group [get_clocks {o_ads1278_sclk_4 o_ads1278_clk_4}]
set_false_path -from [get_clocks {o_ads1278_sclk_1}] -to [get_clocks {o_ads1278_clk_1}]  -hold
set_false_path -from [get_clocks {o_ads1278_sclk_2}] -to [get_clocks {o_ads1278_clk_2}]  -hold
set_false_path -from [get_clocks {o_ads1278_sclk_3}] -to [get_clocks {o_ads1278_clk_3}]  -hold
set_false_path -from [get_clocks {o_ads1278_sclk_4}] -to [get_clocks {o_ads1278_clk_4}]  -hold

