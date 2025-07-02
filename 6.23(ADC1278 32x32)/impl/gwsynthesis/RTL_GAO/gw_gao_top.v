module gw_gao(
    sys_clk,
    ft232_clk,
    clk_rst_n,
    clk_locked,
    \u_pll_clk/clkout ,
    \u_pll_clk/lock ,
    \u_pll_clk/clkoutd ,
    \u_pll_clk/reset ,
    \u_send_data_analys/channel_num[7] ,
    \u_send_data_analys/channel_num[6] ,
    \u_send_data_analys/channel_num[5] ,
    \u_send_data_analys/channel_num[4] ,
    \u_send_data_analys/channel_num[3] ,
    \u_send_data_analys/channel_num[2] ,
    \u_send_data_analys/channel_num[1] ,
    \u_send_data_analys/channel_num[0] ,
    \u_send_data_analys/start_cmd_en ,
    \u_send_data_analys/cali_cmd_en ,
    \u_recv_data_analys/recv_cmd[15] ,
    \u_recv_data_analys/recv_cmd[14] ,
    \u_recv_data_analys/recv_cmd[13] ,
    \u_recv_data_analys/recv_cmd[12] ,
    \u_recv_data_analys/recv_cmd[11] ,
    \u_recv_data_analys/recv_cmd[10] ,
    \u_recv_data_analys/recv_cmd[9] ,
    \u_recv_data_analys/recv_cmd[8] ,
    \u_recv_data_analys/recv_cmd[7] ,
    \u_recv_data_analys/recv_cmd[6] ,
    \u_recv_data_analys/recv_cmd[5] ,
    \u_recv_data_analys/recv_cmd[4] ,
    \u_recv_data_analys/recv_cmd[3] ,
    \u_recv_data_analys/recv_cmd[2] ,
    \u_recv_data_analys/recv_cmd[1] ,
    \u_recv_data_analys/recv_cmd[0] ,
    \u_recv_data_analys/analysis_finish ,
    tms_pad_i,
    tck_pad_i,
    tdi_pad_i,
    tdo_pad_o
);

input sys_clk;
input ft232_clk;
input clk_rst_n;
input clk_locked;
input \u_pll_clk/clkout ;
input \u_pll_clk/lock ;
input \u_pll_clk/clkoutd ;
input \u_pll_clk/reset ;
input \u_send_data_analys/channel_num[7] ;
input \u_send_data_analys/channel_num[6] ;
input \u_send_data_analys/channel_num[5] ;
input \u_send_data_analys/channel_num[4] ;
input \u_send_data_analys/channel_num[3] ;
input \u_send_data_analys/channel_num[2] ;
input \u_send_data_analys/channel_num[1] ;
input \u_send_data_analys/channel_num[0] ;
input \u_send_data_analys/start_cmd_en ;
input \u_send_data_analys/cali_cmd_en ;
input \u_recv_data_analys/recv_cmd[15] ;
input \u_recv_data_analys/recv_cmd[14] ;
input \u_recv_data_analys/recv_cmd[13] ;
input \u_recv_data_analys/recv_cmd[12] ;
input \u_recv_data_analys/recv_cmd[11] ;
input \u_recv_data_analys/recv_cmd[10] ;
input \u_recv_data_analys/recv_cmd[9] ;
input \u_recv_data_analys/recv_cmd[8] ;
input \u_recv_data_analys/recv_cmd[7] ;
input \u_recv_data_analys/recv_cmd[6] ;
input \u_recv_data_analys/recv_cmd[5] ;
input \u_recv_data_analys/recv_cmd[4] ;
input \u_recv_data_analys/recv_cmd[3] ;
input \u_recv_data_analys/recv_cmd[2] ;
input \u_recv_data_analys/recv_cmd[1] ;
input \u_recv_data_analys/recv_cmd[0] ;
input \u_recv_data_analys/analysis_finish ;
input tms_pad_i;
input tck_pad_i;
input tdi_pad_i;
output tdo_pad_o;

wire sys_clk;
wire ft232_clk;
wire clk_rst_n;
wire clk_locked;
wire \u_pll_clk/clkout ;
wire \u_pll_clk/lock ;
wire \u_pll_clk/clkoutd ;
wire \u_pll_clk/reset ;
wire \u_send_data_analys/channel_num[7] ;
wire \u_send_data_analys/channel_num[6] ;
wire \u_send_data_analys/channel_num[5] ;
wire \u_send_data_analys/channel_num[4] ;
wire \u_send_data_analys/channel_num[3] ;
wire \u_send_data_analys/channel_num[2] ;
wire \u_send_data_analys/channel_num[1] ;
wire \u_send_data_analys/channel_num[0] ;
wire \u_send_data_analys/start_cmd_en ;
wire \u_send_data_analys/cali_cmd_en ;
wire \u_recv_data_analys/recv_cmd[15] ;
wire \u_recv_data_analys/recv_cmd[14] ;
wire \u_recv_data_analys/recv_cmd[13] ;
wire \u_recv_data_analys/recv_cmd[12] ;
wire \u_recv_data_analys/recv_cmd[11] ;
wire \u_recv_data_analys/recv_cmd[10] ;
wire \u_recv_data_analys/recv_cmd[9] ;
wire \u_recv_data_analys/recv_cmd[8] ;
wire \u_recv_data_analys/recv_cmd[7] ;
wire \u_recv_data_analys/recv_cmd[6] ;
wire \u_recv_data_analys/recv_cmd[5] ;
wire \u_recv_data_analys/recv_cmd[4] ;
wire \u_recv_data_analys/recv_cmd[3] ;
wire \u_recv_data_analys/recv_cmd[2] ;
wire \u_recv_data_analys/recv_cmd[1] ;
wire \u_recv_data_analys/recv_cmd[0] ;
wire \u_recv_data_analys/analysis_finish ;
wire tms_pad_i;
wire tck_pad_i;
wire tdi_pad_i;
wire tdo_pad_o;
wire tms_i_c;
wire tck_i_c;
wire tdi_i_c;
wire tdo_o_c;
wire [9:0] control0;
wire gao_jtag_tck;
wire gao_jtag_reset;
wire run_test_idle_er1;
wire run_test_idle_er2;
wire shift_dr_capture_dr;
wire update_dr;
wire pause_dr;
wire enable_er1;
wire enable_er2;
wire gao_jtag_tdi;
wire tdo_er1;

IBUF tms_ibuf (
    .I(tms_pad_i),
    .O(tms_i_c)
);

IBUF tck_ibuf (
    .I(tck_pad_i),
    .O(tck_i_c)
);

IBUF tdi_ibuf (
    .I(tdi_pad_i),
    .O(tdi_i_c)
);

OBUF tdo_obuf (
    .I(tdo_o_c),
    .O(tdo_pad_o)
);

GW_JTAG  u_gw_jtag(
    .tms_pad_i(tms_i_c),
    .tck_pad_i(tck_i_c),
    .tdi_pad_i(tdi_i_c),
    .tdo_pad_o(tdo_o_c),
    .tck_o(gao_jtag_tck),
    .test_logic_reset_o(gao_jtag_reset),
    .run_test_idle_er1_o(run_test_idle_er1),
    .run_test_idle_er2_o(run_test_idle_er2),
    .shift_dr_capture_dr_o(shift_dr_capture_dr),
    .update_dr_o(update_dr),
    .pause_dr_o(pause_dr),
    .enable_er1_o(enable_er1),
    .enable_er2_o(enable_er2),
    .tdi_o(gao_jtag_tdi),
    .tdo_er1_i(tdo_er1),
    .tdo_er2_i(1'b0)
);

gw_con_top  u_icon_top(
    .tck_i(gao_jtag_tck),
    .tdi_i(gao_jtag_tdi),
    .tdo_o(tdo_er1),
    .rst_i(gao_jtag_reset),
    .control0(control0[9:0]),
    .enable_i(enable_er1),
    .shift_dr_capture_dr_i(shift_dr_capture_dr),
    .update_dr_i(update_dr)
);

ao_top_0  u_la0_top(
    .control(control0[9:0]),
    .trig0_i({\u_send_data_analys/channel_num[7] ,\u_send_data_analys/channel_num[6] ,\u_send_data_analys/channel_num[5] ,\u_send_data_analys/channel_num[4] ,\u_send_data_analys/channel_num[3] ,\u_send_data_analys/channel_num[2] ,\u_send_data_analys/channel_num[1] ,\u_send_data_analys/channel_num[0] }),
    .trig1_i(\u_send_data_analys/start_cmd_en ),
    .trig2_i(\u_send_data_analys/cali_cmd_en ),
    .trig3_i({\u_recv_data_analys/recv_cmd[15] ,\u_recv_data_analys/recv_cmd[14] ,\u_recv_data_analys/recv_cmd[13] ,\u_recv_data_analys/recv_cmd[12] ,\u_recv_data_analys/recv_cmd[11] ,\u_recv_data_analys/recv_cmd[10] ,\u_recv_data_analys/recv_cmd[9] ,\u_recv_data_analys/recv_cmd[8] ,\u_recv_data_analys/recv_cmd[7] ,\u_recv_data_analys/recv_cmd[6] ,\u_recv_data_analys/recv_cmd[5] ,\u_recv_data_analys/recv_cmd[4] ,\u_recv_data_analys/recv_cmd[3] ,\u_recv_data_analys/recv_cmd[2] ,\u_recv_data_analys/recv_cmd[1] ,\u_recv_data_analys/recv_cmd[0] }),
    .trig4_i(\u_recv_data_analys/analysis_finish ),
    .data_i({sys_clk,ft232_clk,clk_rst_n,clk_locked,\u_pll_clk/clkout ,\u_pll_clk/lock ,\u_pll_clk/clkoutd ,\u_pll_clk/reset }),
    .clk_i(\u_pll_clk/clkout )
);

endmodule
