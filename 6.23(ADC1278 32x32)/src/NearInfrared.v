module NearInfrared(
	input		          		sys_clk,          	//外部50M时钟
	input       	    		sys_rst_n,        	//外部复位信号，低有效
	
	//FT232HQ接口
	input   	wire			ft232_clk,
	inout	  	wire 	[7:0]	ft232_data,
	input		wire			ft232_txe_n,
	input		wire			ft232_rxf_n,
	
	output		wire			ft232_oe_n,
	output  	wire			ft232_wr_n,
	output		wire			ft232_siwu_n,
	output  	wire			ft232_rd_n,
	
	//ads1278
    output                  	o_ads1278_clk_1,         // 芯片时钟
    output                  	o_ads1278_clk_div_1,     // 芯片时钟分频设置
    output                  	o_ads1278_fsync_1,       // 芯片帧同步（ADC转化设置）
    output                  	o_ads1278_sclk_1,        // SPI时钟
    input       		[7:0]   i_ads1278_data_1,        // 8通道数据输入
    output      		[2:0]   o_ads1278_format_1,      // 数据格式设置
    output      		[1:0]   o_ads1278_mode_1,        // 工作模式设置
    output      		[7:0]   o_ads1278_pwdn_1,
    output                  	o_ads1278_sync_1,        // 多片同步信号
    output      		[1:0]   o_ads1278_test_1,        // 测试模式设置		
	output						o_ads1278_din_1,
	
    output                  	o_ads1278_clk_2,         // 芯片时钟
    output                  	o_ads1278_clk_div_2,     // 芯片时钟分频设置
    output                  	o_ads1278_fsync_2,       // 芯片帧同步（ADC转化设置）
    output                  	o_ads1278_sclk_2,        // SPI时钟
    input       		[7:0]   i_ads1278_data_2,        // 8通道数据输入
    output      		[2:0]   o_ads1278_format_2,      // 数据格式设置
    output      		[1:0]   o_ads1278_mode_2,        // 工作模式设置
    output      		[7:0]   o_ads1278_pwdn_2,
    output                  	o_ads1278_sync_2,        // 多片同步信号
    output      		[1:0]   o_ads1278_test_2,        // 测试模式设置		
	output						o_ads1278_din_2,

    output                  	o_ads1278_clk_3,         // 芯片时钟
    output                  	o_ads1278_clk_div_3,     // 芯片时钟分频设置
    output                  	o_ads1278_fsync_3,       // 芯片帧同步（ADC转化设置）
    output                  	o_ads1278_sclk_3,        // SPI时钟
    input       		[7:0]   i_ads1278_data_3,        // 8通道数据输入
    output      		[2:0]   o_ads1278_format_3,      // 数据格式设置
    output      		[1:0]   o_ads1278_mode_3,        // 工作模式设置
    output      		[7:0]   o_ads1278_pwdn_3,
    output                  	o_ads1278_sync_3,        // 多片同步信号
    output      		[1:0]   o_ads1278_test_3,        // 测试模式设置		
	output						o_ads1278_din_3,

    output                  	o_ads1278_clk_4,         // 芯片时钟
    output                  	o_ads1278_clk_div_4,     // 芯片时钟分频设置
    output                  	o_ads1278_fsync_4,       // 芯片帧同步（ADC转化设置）
    output                  	o_ads1278_sclk_4,        // SPI时钟
    input       		[7:0]   i_ads1278_data_4,        // 8通道数据输入
    output      		[2:0]   o_ads1278_format_4,      // 数据格式设置
    output      		[1:0]   o_ads1278_mode_4,        // 工作模式设置
    output      		[7:0]   o_ads1278_pwdn_4,
    output                  	o_ads1278_sync_4,        // 多片同步信号
    output      		[1:0]   o_ads1278_test_4,        // 测试模式设置		
	output						o_ads1278_din_4,

	//gain 接口
	output 		wire			gain_rck,
	output 		wire			gain_scl,
	output 		wire			gain_sck,
	output 		wire			gain_ser,
	
	//laser接口
	output		wire			laser_rck,
	output		wire			laser_scl,
	output		wire			laser_sck,
	output		wire			laser_ser,

	//电源
	output		reg				usb_act,
	input		wire			usb_rdy,
	
	//触发输入
	input		wire			tri_qh,
	output		wire			tri_clk,
	output		wire			tri_ld,

	//触发输出
	output		wire			o_tri_out_rck,
	output		wire			o_tri_out_scl,
	output		wire			o_tri_out_sck,
	output		wire			o_tri_out_ser,

	//WL1接口
	output		wire			wl1_fsync,
	output		wire			wl1_sclk,
	output		wire			wl1_sdata,
	
	//WL2接口
	output		wire			wl2_fsync,
	output		wire			wl2_sclk,
	output		wire			wl2_sdata
);

//parameter define
parameter	LASER_VERSION		=	0			;		// 0: (785 830)	1: (690 830) 2: (760 850)	
parameter  	CLK_FREQ 			= 	50000000;       	//定义系统时钟频率
parameter  	FREE_STATE 			= 	6'b000000	;     	//空闲状态
parameter  	CHECK_STATE 		= 	6'b000001	;     	//查询状态
parameter  	CONFIG_STATE 		= 	6'b000010	;    	//配置状态
parameter  	START_STATE 		= 	6'b000100	;     	//开始状态
parameter  	STOP_STATE 			= 	6'b001000	;      	//停止状态
parameter	CALIBRATION_STATE 	= 	6'b010000	;		//校准状态
parameter	STOP_CALI_STATE 	= 	6'b100000	;		//停止校准
parameter	TRIGGER_OUT_FLAG	=	6'b100001	;		//触发输出标志

//recv_data_analys 和 send_data_analys 数据交互接口
wire state_flag;
wire [5:0] state;

//采样通道数据
wire  [7:0]   sampling_channel_data;
wire  [7:0]   laser_channel_data;
wire  [7:0]   sampling_frequency_data;
wire  [15:0]   channel_samp_frequency;

//电源控制
reg		[31:0]	delay_time;
reg		[31:0]	restart_time;

//触发输入
wire	[7:0]	tri_data;
wire			tri_rd_finish;
wire			tri_rd_flag;
reg				tri_rd_d0;
reg				tri_rd_d1;
wire			tri_en;

//触发输出
wire	[7:0]	w_trigger_out_data;
wire			w_trigger_out_en;
wire			w_trigger_out_finish;

assign	o_ads1278_din_1 = 0;
assign	o_ads1278_din_2 = 0;
assign	o_ads1278_din_3 = 0;
assign	o_ads1278_din_4 = 0;

assign	w_trigger_out_en	=	(state == TRIGGER_OUT_FLAG)	?
								1'b1 : 1'b0;
assign tri_rd_flag = (~tri_rd_d1) & tri_rd_d0;
/**************************************
			main code
**************************************/
//检测PC状态延时
always	@(posedge sys_clk or negedge sys_rst_n)begin
	if(!sys_rst_n)
		delay_time	<= 0;
	else
		if(delay_time < 'd1500000000)//计时30s
			delay_time <= delay_time + 'd1;
end

always	@(posedge sys_clk or negedge sys_rst_n)begin
	if(!sys_rst_n)
		restart_time	<= 0;
	else
		if(usb_rdy == 0 && delay_time == 'd1500000000)
			if(restart_time < 'd150000000)//计时3s
				restart_time <= restart_time + 'd1;
end

always @(posedge sys_clk or negedge sys_rst_n)begin
	if(!sys_rst_n)begin
		usb_act	<= 0;
	end
	else begin
		if(delay_time == 'd150000_0000)begin
			if(usb_rdy == 1)begin
				usb_act	<= 0;
			end
			else if(restart_time=='d150000000 && usb_rdy == 0)begin
				usb_act <= 1;
			end
		end
	end
end

//
always @(posedge sys_clk or negedge sys_rst_n) begin         
    if (!sys_rst_n) begin		
		tri_rd_d0 <= 1'b0;
		tri_rd_d1 <= 1'b0;
    end                                                      
    else begin                                               
      	tri_rd_d0 <= tri_rd_finish;
		tri_rd_d1 <= tri_rd_d0;
    end
end

hc165_drive u_tri_in(
	.clk							(sys_clk),
	.rst_n							(sys_rst_n),
	.i_start						(tri_en),
	
	.o_shift_clk					(tri_clk),
	.i_shift_qh4					(tri_qh),
	.o_shift_shld_n					(tri_ld),
	.o_code_valid					(tri_rd_finish),
	.o_code							(tri_data)
);

trigger_out_module	u_tri_out(
	.i_clk							(sys_clk)						,
	.i_rst_n						(sys_rst_n)						,
	.i_trigger_out_en				(w_trigger_out_en)				,
	.i_trigger_out_data				(w_trigger_out_data)			,
	
	.o_tri_out_rck					(o_tri_out_rck)					,
	.o_tri_out_sck					(o_tri_out_sck)					,
	.o_tri_out_scl					(o_tri_out_scl)					,
	.o_tri_out_ser					(o_tri_out_ser)					
);

recv_data_analys #(
	.CLK_FREQ						(CLK_FREQ))
u_recv_data_analys(
	.clk							(sys_clk),
	.rst_n							(sys_rst_n),

	.ft232_clk						(ft232_clk),
	.ft232_oe_n						(ft232_oe_n),
	.ft232_rxf_n					(ft232_rxf_n),
	.ft232_wr_n						(ft232_wr_n),
	.ft232_data_recv				(ft232_data),
	.ft232_rd_n						(ft232_rd_n),
	
	.sampling_channel_data 			(sampling_channel_data),
	.laser_channel_data	   			(laser_channel_data),
	.sampling_frequency_data		(sampling_frequency_data),
	.channel_samp_frequency    		(channel_samp_frequency),
	
	.o_trigger_out_data				(w_trigger_out_data),


	.state_flag						(state_flag),
	.state							(state)
);

send_data_analys #(
	.LASER_VERSION					(LASER_VERSION))
u_send_data_analys(
	.clk							(sys_clk),
	.rst_n							(sys_rst_n),
	
	.ft232_clk						(ft232_clk),
	.ft232_rd_n						(ft232_rd_n),
	.ft232_txe_n					(ft232_txe_n),
	.ft232_wr_n						(ft232_wr_n),
	.ft232_data_send				(ft232_data),
	.ft232_siwu_n					(ft232_siwu_n),
	
	.gain_rck						(gain_rck),
	.gain_scl						(gain_scl),
	.gain_sck						(gain_sck),
	.gain_ser						(gain_ser),
	
	.tri_data						(tri_data),
	.tri_en							(tri_en),
	.tri_rd_flag					(tri_rd_flag),

	.laser_rck						(laser_rck),
	.laser_scl						(laser_scl),
	.laser_sck						(laser_sck),
	.laser_ser						(laser_ser),
	
	.wl1_fsync						(wl1_fsync),
	.wl1_sclk						(wl1_sclk),
	.wl1_sdata						(wl1_sdata),
	
	.wl2_fsync						(wl2_fsync),
	.wl2_sclk						(wl2_sclk),
	.wl2_sdata						(wl2_sdata),

    .o_ads1278_clk_1          		(o_ads1278_clk_1),     // 芯片时钟
    .o_ads1278_clk_div_1      		(o_ads1278_clk_div_1), // 芯片时钟分频设置
    .o_ads1278_fsync_1        		(o_ads1278_fsync_1),   // 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk_1         		(o_ads1278_sclk_1),    // SPI时钟
    .i_ads1278_data_1         		(i_ads1278_data_1),    // 8通道数据输入
    .o_ads1278_format_1       		(o_ads1278_format_1),  // 数据格式设置
    .o_ads1278_mode_1         		(o_ads1278_mode_1),    // 工作模式设置
    .o_ads1278_pwdn_1         		(o_ads1278_pwdn_1),    // 通道开关设置
    .o_ads1278_sync_1         		(o_ads1278_sync_1),    // 多片同步信号
    .o_ads1278_test_1         		(o_ads1278_test_1),    // 测试模式设置
	
    .o_ads1278_clk_2          		(o_ads1278_clk_2),     // 芯片时钟
    .o_ads1278_clk_div_2      		(o_ads1278_clk_div_2), // 芯片时钟分频设置
    .o_ads1278_fsync_2        		(o_ads1278_fsync_2),   // 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk_2         		(o_ads1278_sclk_2),    // SPI时钟
    .i_ads1278_data_2         		(i_ads1278_data_2),    // 8通道数据输入
    .o_ads1278_format_2       		(o_ads1278_format_2),  // 数据格式设置
    .o_ads1278_mode_2         		(o_ads1278_mode_2),    // 工作模式设置
    .o_ads1278_pwdn_2         		(o_ads1278_pwdn_2),    // 通道开关设置
    .o_ads1278_sync_2         		(o_ads1278_sync_2),    // 多片同步信号
    .o_ads1278_test_2         		(o_ads1278_test_2),    // 测试模式设置

    .o_ads1278_clk_3          		(o_ads1278_clk_3),     // 芯片时钟
    .o_ads1278_clk_div_3      		(o_ads1278_clk_div_3), // 芯片时钟分频设置
    .o_ads1278_fsync_3        		(o_ads1278_fsync_3),   // 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk_3         		(o_ads1278_sclk_3),    // SPI时钟
    .i_ads1278_data_3         		(i_ads1278_data_3),    // 8通道数据输入
    .o_ads1278_format_3       		(o_ads1278_format_3),  // 数据格式设置
    .o_ads1278_mode_3         		(o_ads1278_mode_3),    // 工作模式设置
    .o_ads1278_pwdn_3         		(o_ads1278_pwdn_3),    // 通道开关设置
    .o_ads1278_sync_3         		(o_ads1278_sync_3),    // 多片同步信号
    .o_ads1278_test_3         		(o_ads1278_test_3),    // 测试模式设置

    .o_ads1278_clk_4          		(o_ads1278_clk_4),     // 芯片时钟
    .o_ads1278_clk_div_4      		(o_ads1278_clk_div_4), // 芯片时钟分频设置
    .o_ads1278_fsync_4        		(o_ads1278_fsync_4),   // 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk_4         		(o_ads1278_sclk_4),    // SPI时钟
    .i_ads1278_data_4         		(i_ads1278_data_4),    // 8通道数据输入
    .o_ads1278_format_4       		(o_ads1278_format_4),  // 数据格式设置
    .o_ads1278_mode_4         		(o_ads1278_mode_4),    // 工作模式设置
    .o_ads1278_pwdn_4         		(o_ads1278_pwdn_4),    // 通道开关设置
    .o_ads1278_sync_4         		(o_ads1278_sync_4),    // 多片同步信号
    .o_ads1278_test_4         		(o_ads1278_test_4),    // 测试模式设置

	.sampling_channel_data			(sampling_channel_data),
	.laser_channel_data				(laser_channel_data),
	.sampling_frequency_data 		(sampling_frequency_data),
	.channel_samp_frequency  		(channel_samp_frequency),

	.state							(state),
	.state_flag						(state_flag)
);

endmodule