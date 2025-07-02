module send_data_analys#(
	parameter						LASER_VERSION = 0		// 0: (785 830)	1: (690 830) 2: (760 850)	
)(
	input 							clk,
	input							rst_n,
	
	//ft232接口
	input  		wire				ft232_clk,
	input  		wire				ft232_rd_n,
	input  		wire				ft232_txe_n,
	output		wire				ft232_wr_n,
	output		wire	[7:0]		ft232_data_send,
	output		wire				ft232_siwu_n,
	
	//ads1278接口
    output                  		o_ads1278_clk_1,         // 芯片时钟
    output                  		o_ads1278_clk_div_1,     // 芯片时钟分频设置
    output                  		o_ads1278_fsync_1,       // 芯片帧同步（ADC转化设置）
    output                  		o_ads1278_sclk_1,        // SPI时钟
    input       		[7:0]       i_ads1278_data_1,        // 8通道数据输入
    output      		[2:0]       o_ads1278_format_1,      // 数据格式设置
    output      		[1:0]       o_ads1278_mode_1,        // 工作模式设置
    output      		[7:0]       o_ads1278_pwdn_1,
    output                  		o_ads1278_sync_1,        // 多片同步信号
    output      		[1:0]       o_ads1278_test_1,        // 测试模式设置
	
    output                  		o_ads1278_clk_2,         // 芯片时钟
    output                  		o_ads1278_clk_div_2,     // 芯片时钟分频设置
    output                  		o_ads1278_fsync_2,       // 芯片帧同步（ADC转化设置）
    output                  		o_ads1278_sclk_2,        // SPI时钟
    input       		[7:0]       i_ads1278_data_2,        // 8通道数据输入
    output      		[2:0]       o_ads1278_format_2,      // 数据格式设置
    output      		[1:0]       o_ads1278_mode_2,        // 工作模式设置
    output      		[7:0]       o_ads1278_pwdn_2,
    output                  		o_ads1278_sync_2,        // 多片同步信号
    output      		[1:0]       o_ads1278_test_2,        // 测试模式设置

    output                  		o_ads1278_clk_3,         // 芯片时钟
    output                  		o_ads1278_clk_div_3,     // 芯片时钟分频设置
    output                  		o_ads1278_fsync_3,       // 芯片帧同步（ADC转化设置）
    output                  		o_ads1278_sclk_3,        // SPI时钟
    input       		[7:0]       i_ads1278_data_3,        // 8通道数据输入
    output      		[2:0]       o_ads1278_format_3,      // 数据格式设置
    output      		[1:0]       o_ads1278_mode_3,        // 工作模式设置
    output      		[7:0]       o_ads1278_pwdn_3,
    output                  		o_ads1278_sync_3,        // 多片同步信号
    output      		[1:0]       o_ads1278_test_3,        // 测试模式设置

    output                  		o_ads1278_clk_4,         // 芯片时钟
    output                  		o_ads1278_clk_div_4,     // 芯片时钟分频设置
    output                  		o_ads1278_fsync_4,       // 芯片帧同步（ADC转化设置）
    output                  		o_ads1278_sclk_4,        // SPI时钟
    input       		[7:0]       i_ads1278_data_4,        // 8通道数据输入
    output      		[2:0]       o_ads1278_format_4,      // 数据格式设置
    output      		[1:0]       o_ads1278_mode_4,        // 工作模式设置
    output      		[7:0]       o_ads1278_pwdn_4,
    output                  		o_ads1278_sync_4,        // 多片同步信号
    output      		[1:0]       o_ads1278_test_4,        // 测试模式设置

	//pga控制接口
	output		wire 				gain_rck,
	output		wire				gain_scl,
	output		wire				gain_sck,
	output		wire 				gain_ser,
	
	//laser切换接口
	output		wire 				laser_rck,
	output		wire				laser_scl,
	output		wire				laser_sck,
	output		wire 				laser_ser,
	
	//触发输入接口
	input		wire	[7:0]		tri_data,
	input		wire				tri_rd_flag,
	output		reg					tri_en,

	//9833接口1
	output		wire				wl1_fsync,
	output		wire				wl1_sclk,
	output		wire				wl1_sdata,
	
	//9833接口2
	output		wire				wl2_fsync,
	output		wire				wl2_sclk,
	output		wire				wl2_sdata,
	
	//参数接口
	input 		wire	[7:0]   	sampling_channel_data,
	input 		wire	[7:0]   	laser_channel_data,
	input 		wire	[7:0]   	sampling_frequency_data,
	input 		wire	[15:0]		channel_samp_frequency,
	
	//状态
	input  		wire	[5:0]   	state,	
	output		reg          		state_flag
);
localparam  	PKG_CNT  			= 	10'd70		,		//发送数据包长度
				P_GAIN_LENGTH		=	8'd128		,		//PGA数据长度
				CLK_FREQ 			= 	50000000	,       //定义系统时钟频率
				FREE_STATE 			= 	6'b000000	,     	//空闲状态
				CHECK_STATE 		= 	6'b000001	,     	//查询状态
				CONFIG_STATE 		= 	6'b000010	,    	//配置状态
				START_STATE 		= 	6'b000100	,     	//开始状态
				STOP_STATE 			= 	6'b001000	,      	//停止状态
				CALIBRATION_STATE 	= 	6'b010000	,		//校准状态
				STOP_CALI_STATE 	= 	6'b100000	,		//停止校准
				TRIGGER_OUT_FLAG	=	6'b100001	,		//触发输出标志
				HIGH_LIMIT			= 	16'd12000	,		//幅值上限
				LOW_LIMIT			= 	16'd1350	;		//幅值下限

/********************ft232_fifo接口********************/
reg 				ft232_wrreq;
reg 	[7:0] 		ft232_data_in;
reg 	[7:0]   	fifo_cnt;
reg         		fifo_rd_en;
wire				fifo_empty_n;
wire  				ft232_rdreq;
wire	[7:0] 		ft232_data_out;
wire	[12:0]		rdusedw_cnt;
wire	[12:0]		wrusedw_cnt;

/********************触发输入数据读取********************/
reg		[7:0]		tri_data_temp;
reg					tri_en_d0;
reg					tri_en_d1;
reg					send_tri_start;
reg		[15:0]		tri_timing;
wire				tri_en_flag;

/********************ADC接口********************/
reg					adc_en;
reg					adc1_en;
reg					adc2_en;
reg					adc3_en;
reg					adc4_en;
reg					data_valid;
wire	[191:0]		w_ads1278_rdata_1;
wire	[191:0]		w_ads1278_rdata_2;
wire	[191:0]		w_ads1278_rdata_3;
wire	[191:0]		w_ads1278_rdata_4;
wire				w_user_rdata_ready_1;
wire				w_user_rdata_ready_2;
wire				w_user_rdata_ready_3;
wire				w_user_rdata_ready_4;
wire				w_user_rdata_valid_1;
wire				w_user_rdata_valid_2;
wire				w_user_rdata_valid_3;
wire				w_user_rdata_valid_4;

/********************增益写入********************/
reg 	[P_GAIN_LENGTH-1:0]		gain_data;
reg 	[7:0]		gain_data_len;
reg 				gain_wr_en;
reg		[31:0]		gain_data_temp1[31:0];
reg		[31:0]		gain_data_temp2[31:0];
reg		[31:0]		gain_data_temp3[31:0];
reg		[31:0]		gain_data_temp4[31:0];
wire				gain_wr_finish;

/********************光源切换控制********************/
reg		[24:0]		irq_count;
reg 	[3:0]		laser_data[3:0];
reg		[15:0]		laser_wr_data;
reg		[7:0]		laser_data_len;
reg		[3:0]		data_cnt;
reg					laser_wr_en;
reg		[7:0]      	laser_channel_cnt;
reg		[3:0]		laser_state;
wire				laser_finish_flag;					

/********************PC命令接口********************/
reg 				start_cmd_en;
reg 				stop_cmd_en;
reg 				check_cmd_en;
reg   				check_cmd_refresh;
reg 				cali_cmd_en;
reg 				stop_cali_cmd_en;
reg					state_flag_temp;

/********************AD9833波形控制********************/
reg		[3:0]		wl1_state;
reg		[15:0]		wl1_data;
reg		[7:0]		wl1_data_len;
reg					wl1_wr_en;
wire				wl1_wr_finish;
reg		[3:0]		wl2_state;
reg		[15:0]		wl2_data;
reg		[7:0]		wl2_data_len;
reg					wl2_wr_en;
wire				wl2_wr_finish;

wire				wl1_en_flag;
reg					wl1_en_d0;
reg					wl1_en_d1;
reg					wl1_finish;
wire				wl2_en_flag;
reg					wl2_en_d0;
reg					wl2_en_d1;
reg					wl2_finish;

/********************计时********************/
reg 	[27:0]		channel_count;
reg 	[27:0]		samp_count_num;	
reg		[33:0] 		cali_timecnt;
reg		[33:0]		CALI_TIME;
wire	[7:0]		samp_channel_data;

/********************数据发送********************/
reg		[15:0]		send_cnt;
reg		[7:0]		daq_pkg_data_cnt;
reg 	[7:0]		send_pkg[PKG_CNT-1:0];//70byte
reg 				send_pack_finish	;//打包完成
reg 				check_send_pack_finish	;//check打包完成
reg 	[7:0]		send_pkg_data[PKG_CNT-1:0];//70byte
reg					cali_send_pack_finish;//
reg         		stop_cali_delay_flag;
reg         		send_pkg_start_f1;
reg         		send_pkg_start_f2;
reg         		send_data_start_f1;
reg         		send_data_start_f2;
wire        		send_data_start_f;
wire        		send_pkg_start_f;
wire 	[15:0]		datapkg_length;
wire 	[15:0]		channel_count_pkg;
wire 	[15:0]		calipkg_length;	

/********************校准功能********************/
reg		[15:0]		COUNT_TIMES;
reg		[15:0] 		high_temp[31:0][7:0];	
reg		[15:0]		low_temp[31:0][7:0];
reg		[15:0] 		adjust_cnt[31:0];	
wire			 	cali_state;

/********************组合逻辑********************/
assign	w_user_rdata_ready_1 	= w_user_rdata_valid_1;
assign	w_user_rdata_ready_2 	= w_user_rdata_valid_2;
assign	w_user_rdata_ready_3 	= w_user_rdata_valid_3;
assign	w_user_rdata_ready_4 	= w_user_rdata_valid_4;
assign	tri_en_flag 			= (~tri_en_d1) & tri_en_d0;
assign	send_pkg_start_f 		= (~send_pkg_start_f2) & send_pkg_start_f1;
assign	send_data_start_f 		= (~send_data_start_f2) & send_data_start_f1;
assign	wl1_en_flag 			= (~wl1_en_d1) & wl1_en_d0;
assign	wl2_en_flag 			= (~wl2_en_d1) & wl2_en_d0;
assign	cali_state				= (cali_timecnt >= (CALI_TIME - 'd49999999)) ?
								  1'd1 : 0;

integer a,b,i,j,o,p,z,x,y,u,v;

//*****************************************************
//**                    main code
//*****************************************************

//对使能信号wl_en延后两个时钟周期
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin		
		wl1_en_d0 <= 1'b0;
		wl1_en_d1 <= 1'b0;
		wl2_en_d0 <= 1'b0;
		wl2_en_d1 <= 1'b0;
    end                                                      
    else begin                                               
      	wl1_en_d0 <= wl1_wr_finish;
		wl1_en_d1 <= wl1_en_d0;
		wl2_en_d0 <= wl2_wr_finish;
		wl2_en_d1 <= wl2_en_d0;
    end
end

//使能检测触发输入
always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		tri_en	<= 0;
	end
	else begin
		if(start_cmd_en)begin
			tri_en	<= 1;
		end
		else begin
			tri_en	<= 0;
		end
	end
end

//读取触发输入数据
always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		tri_data_temp	<= 0;
	end
	else begin	
		if(start_cmd_en)begin
			if(tri_rd_flag)begin
				tri_data_temp	<= tri_data;
			end
		end
		else begin
			tri_data_temp	<= 0;
		end
	end
end

//判断是否为对应触发信号
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin		
		tri_en_d0 <= 1'b0;
		tri_en_d1 <= 1'b0;
    end                                                      
    else begin                                               
      	tri_en_d0 <= (tri_data_temp != 8'b0000_0000) ? 1'b1:1'b0 ;
		tri_en_d1 <= tri_en_d0;
    end
end

//记录触发时刻
always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		tri_timing	<= 0;
	end
	else begin
		if(start_cmd_en)begin
			if(tri_en_flag)begin
				tri_timing	<= send_cnt;
			end
		end
		else begin
			tri_timing	<= 0;
		end
	end
end

//上位机指令接口状态机
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
		state_flag_temp <= 0;
	   	start_cmd_en <= 0;
	   	stop_cmd_en <= 0;
		cali_cmd_en <= 0;
		check_cmd_refresh <= 0;
		stop_cali_delay_flag <= 0;
    end                                                      
    else begin 
		case(state)
			FREE_STATE:begin
				state_flag_temp <= 0;
			end
			CHECK_STATE:begin
				check_cmd_refresh <= 1;
				if(check_cmd_en)begin
					check_cmd_refresh <= 0;
					state_flag_temp <= 1;
				end
			end
			CONFIG_STATE:begin
				state_flag_temp <= 1;
			end
			START_STATE:begin
				cali_cmd_en <= 0;
				stop_cmd_en <= 0;
				state_flag_temp <= 1;
				if(stop_cmd_en==0)begin
					start_cmd_en <= 1;
				end
			end
			STOP_STATE:begin 
				stop_cmd_en <= 1;
				start_cmd_en <= 0;
				state_flag_temp <= 1;
			end
			CALIBRATION_STATE:begin
				if(stop_cmd_en==0)begin
					cali_cmd_en <= 1;
				end
				stop_cmd_en <= 0;
				state_flag_temp <= 1;
			end
			STOP_CALI_STATE:begin
				stop_cali_delay_flag <= 1;
				cali_cmd_en <= 0;
				if(stop_cali_cmd_en==1)begin
					stop_cali_delay_flag <= 0;
				    state_flag_temp <= 1;
				end
			end
			default:begin
				state_flag_temp <= 1;
			end
		endcase
		if(cali_timecnt == CALI_TIME)begin
			cali_cmd_en <= 0;
		end
    end
end

assign	channel_count_pkg = 'd625;					
assign	datapkg_length = 'd1250*sampling_channel_data+'d5;
assign 	calipkg_length = laser_channel_data*sampling_channel_data+'d3;
assign	samp_channel_data = (sampling_channel_data<8)?sampling_channel_data:8;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		state_flag <= 0;
	end
	else begin
		state_flag <= state_flag_temp;
	end
end

always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		COUNT_TIMES <= 0;
	end
	else begin
		case(laser_channel_data)
		'd4:begin
			COUNT_TIMES <= 'd12500;
		end
		'd8:begin
			COUNT_TIMES <= 'd6250;
		end
		'd16,'d32:begin
			COUNT_TIMES <= 'd3125;
		end
		endcase
	end
end

always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		samp_count_num <= 0;
	end
	else begin
		case(sampling_frequency_data*laser_channel_data)
		'd4:begin
			samp_count_num <= 'd12500000;
		end
		'd8:begin
			samp_count_num <= 'd6250000;
		end
		'd16:begin
			samp_count_num <= 'd3125000;
		end
		'd32:begin
			samp_count_num <= 'd1562500;
		end
		'd64,'d80:begin
			samp_count_num <= 'd625000;
		end
		endcase
	end
end

always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		CALI_TIME <= 0;
	end
	else begin
		case(sampling_channel_data)
		'd4:begin
			CALI_TIME <= 'd800000000;//16s
		end
		'd8:begin
			CALI_TIME <= 'd1550000000;//31s
		end
		'd16:begin
			CALI_TIME <= 'd3050000000;//61s
		end
		'd32:begin
			CALI_TIME <= 33'd6050000000;//121s
		end
		endcase
	end
end

//计时操作
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
		channel_count <= 0;
    end                                                      
    else begin                                               
		if((start_cmd_en == 1 && adc_en == 1)||(cali_cmd_en == 1 && adc_en == 1))begin
			if(channel_count == samp_count_num-1)begin
				channel_count <= 0;
			end
			else begin
				channel_count <= channel_count + 1'd1;
			end
		end
		else begin
			channel_count <= 0;
		end
    end
end

//校准计时
always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		cali_timecnt <= 0;
	end
	else begin
		if(cali_cmd_en && adc_en)begin
			if(cali_timecnt == CALI_TIME)begin
				cali_timecnt <= cali_timecnt;
			end
			else begin
				cali_timecnt <= cali_timecnt + 'd1;
			end
		end
		else begin
			cali_timecnt <= 0;
		end
	end
end

//ADC使能操作
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
		adc1_en <= 0;
		adc2_en <= 0;
		adc3_en <= 0;
		adc4_en <= 0;
		adc_en  <= 0;
    end                                                      
    else begin                                               
        if(start_cmd_en == 1 || cali_cmd_en == 1)begin
			adc_en <= 1;
			adc1_en <= 1;
			case(laser_channel_data)
			'd4,'d8	:	adc2_en <= 0;
			'd16	:	adc2_en <= 1;
			'd32	:	begin
							adc2_en <= 1;
							adc3_en <= 1;
							adc4_en <= 1;
						end
			endcase	
		end
		else if(stop_cali_cmd_en == 1 || stop_cmd_en == 1)begin
			adc1_en <= 0;
			adc2_en <= 0;
			adc3_en <= 0;
			adc4_en <= 0;
			adc_en  <= 0;
		end
    end
end

//WL1
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		wl1_data		<= 16'd0;
		wl1_data_len	<= 7'd16;
		wl1_wr_en		<= 0;
		wl1_finish		<= 0;
		wl1_state		<= 0;
	end
	else begin
		if(start_cmd_en == 1||cali_cmd_en == 1)begin
			 case(wl1_state)
			 4'd0:begin									//复位
				 wl1_data  <= 16'b0000_0001_0000_0000;
				 wl1_wr_en <= 1;
				 wl1_state <= 4'd1;
			 end
			 4'd1:begin
				 if(wl1_en_flag)begin
					 wl1_wr_en <= 0;
					 wl1_state <= 4'd2;
				 end
			 end
			 4'd2:begin									//选择一次性写入
				 wl1_data  <= 16'b0010_0001_0000_0000;
				 wl1_wr_en <= 1;
				 wl1_state <= 4'd3;
			 end
			 4'd3:begin
				 if(wl1_en_flag)begin
					 wl1_wr_en <= 0;
					 wl1_state <= 4'd4;
				 end
			 end
			 4'd4:begin									//写入LSB
				 wl1_data  <= 16'b0101_1000_1001_0100;//1khz
				 //(1.5k)wl1_data  <= 16'b0110_0100_1101_1110;
				 wl1_wr_en <= 1;
				 wl1_state <= 4'd5;
			 end
			 4'd5:begin
				 if(wl1_en_flag)begin
					 wl1_wr_en <= 0;
					 wl1_state <= 4'd6;
				 end
			 end
			 4'd6:begin									//写入MSB
				 wl1_data  <= 16'b0100_0000_0001_0000;//1khz
				 //(1.5k)wl1_data  <= 16'b0100_0000_0001_1000;
				 wl1_wr_en <= 1;
				 wl1_state <= 4'd7;
			 end
			 4'd7:begin
				 if(wl1_en_flag)begin
					 wl1_wr_en <= 0;
					 wl1_state <= 4'd8;
				 end
			 end
			 4'd8:begin									//输出正弦波
				 wl1_data  <= 16'b0000_0000_0000_0000;
				 wl1_wr_en <= 1;
				 wl1_state <= 4'd9;
			 end
			 4'd9:begin
				 if(wl1_en_flag)begin
					 wl1_wr_en <= 0;
					 wl1_state <= 4'd9;
				 end
			 end
			 default:begin
				wl1_wr_en <= 0;
				wl1_data  <= 16'd0;
			 end
			 endcase	
		end
		else begin
			wl1_wr_en <= 0;
			wl1_data  <= 16'd0;
			wl1_state <= 0;
		end
	end
end

//WL2
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		wl2_data		<= 16'd0;
		wl2_data_len	<= 7'd16;
		wl2_wr_en		<= 0;
		wl2_finish		<= 0;
		wl2_state		<= 0;
	end
	else begin
		if (start_cmd_en == 1||cali_cmd_en == 1) begin
			 case(wl2_state)
			 4'd0:begin									//复位
				 wl2_data  <= 16'b0000_0001_0000_0000;
				 wl2_wr_en <= 1;
				 wl2_state <= 4'd1;
			 end
			 4'd1:begin
				 if(wl2_en_flag)begin
					 wl2_wr_en <= 0;
					 wl2_state <= 4'd2;
				 end
			 end
			 4'd2:begin									//一次性写入
				 wl2_data  <= 16'b0010_0001_0000_0000;
				 wl2_wr_en <= 1;
				 wl2_state <= 4'd3;
			 end
			 4'd3:begin
				 if(wl2_en_flag)begin
					 wl2_wr_en <= 0;
					 wl2_state <= 4'd4;
				 end
			 end
			 4'd4:begin									//LSB
				 wl2_data  <= 16'b0110_0100_1101_1110; //(1.5k)
				 wl2_wr_en <= 1;
				 wl2_state <= 4'd5;
			 end
			 4'd5:begin
				 if(wl2_en_flag)begin
					 wl2_wr_en <= 0;
					 wl2_state <= 4'd6;
				 end
			 end
			 4'd6:begin									//MSB
				 wl2_data  <= 16'b0100_0000_0001_1000;//(1.5k)
				 wl2_wr_en <= 1;
				 wl2_state <= 4'd7;
			 end
			 4'd7:begin
				 if(wl2_en_flag)begin
					 wl2_wr_en <= 0;
					 wl2_state <= 4'd8;
				 end
			 end
			 4'd8:begin									//输出正弦波
				 wl2_data  <= 16'b0000_0000_0000_0000;
				 wl2_wr_en <= 1;
				 wl2_state <= 4'd9;
			 end
			 4'd9:begin
				 if(wl2_en_flag)begin
					 wl2_wr_en <= 0;
					 wl2_state <= 4'd9;
				 end
			 end
			 default:begin
				wl2_wr_en <= 0;
				wl2_data  <= 16'd0;
			 end
			 endcase	
		end
		else begin
			wl2_wr_en <= 0;
			wl2_data  <= 16'd0;
			wl2_state <= 0;
		end
	end
end

//光源切换
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		laser_channel_cnt 	<= 1;
		data_cnt 			<= 3;
		laser_state 		<= 4'h0;
		irq_count 			<= 25'h0;
		laser_data[0]  		<= 4'h8;
		laser_data[1]		<= 4'h8;
		laser_data[2]  		<= 4'h8;
		laser_data[3]		<= 4'h8;
		laser_wr_data 		<= 16'h0;
		laser_data_len 		<= 16;
		laser_wr_en			<= 0;
	end
	else begin
		if(start_cmd_en == 1)begin
			case(laser_state)
			4'h0:begin
				if(irq_count == samp_count_num -4)begin
					irq_count <= 25'h0;
					laser_state <= 4'h1;
					if (laser_data[data_cnt] == 8'h88) begin//初始化
						 laser_data[data_cnt] <= 8'h80;
					end else begin
						 laser_data[data_cnt] <= laser_data[data_cnt];
					end
				end
				else begin
					laser_wr_en <= 0;
					irq_count <= irq_count + 1'd1;
					laser_state <= 4'h0;
				end
			end
			4'h1:begin
				laser_wr_data <= {laser_data[0],laser_data[1],laser_data[2],laser_data[3]};//写入
				laser_wr_en <= 1;
				laser_state <= 4'h2;
				laser_channel_cnt <= laser_data[data_cnt];
			end
			4'h2:begin
				case(data_cnt)
				'd0: laser_channel_cnt <= 24 + laser_channel_cnt;
				'd1: laser_channel_cnt <= 16 + laser_channel_cnt;
				'd2: laser_channel_cnt <= 8  + laser_channel_cnt;
				default:;
				endcase
				laser_state <= 4'h3;
			end
			4'h3:begin
				if(laser_channel_cnt != laser_channel_data)begin
					laser_data[data_cnt] <= (laser_data[data_cnt] == 4'h7) ? 4'h8 : laser_data[data_cnt] + 4'h1;
					data_cnt <= (laser_data[data_cnt] == 4'h7) ? data_cnt - 1'd1 : data_cnt;
				end
				else begin//写完复位
					laser_data[0]  		<= 4'h8;
					laser_data[1]		<= 4'h8;
					laser_data[2]  		<= 4'h8;
					laser_data[3]		<= 4'h8;
					data_cnt <= 3;
				end
				if(data_cnt==0)begin
					data_cnt <= 3;
				end
				laser_state <= 4'h0;
			end
			default:begin
				laser_state <= 4'h0;
			end
			endcase
		end
		else if(cali_cmd_en == 1)begin
			if(cali_state != 1)begin
				case(laser_state)
				4'h0:begin
					if(irq_count == samp_count_num -4)begin
						irq_count <= 25'h0;
						laser_state <= 4'h1;
						if (laser_data[data_cnt] == 4'h8) begin//初始化
							 laser_data[data_cnt] <= 4'h0;
						end else begin
							 laser_data[data_cnt] <= laser_data[data_cnt];
						end
					end
					else begin
						laser_wr_en <= 0;
						irq_count <= irq_count + 1'd1;
						laser_state <= 4'h0;
					end
				end
				4'h1:begin
					laser_wr_data <= {laser_data[0],laser_data[1],laser_data[2],laser_data[3]};//写入
					laser_wr_en <= 1;
					laser_state <= 4'h2;
					laser_channel_cnt <= laser_data[data_cnt];
				end
				4'h2:begin
					case(data_cnt)
					'd0: laser_channel_cnt <= 24 + laser_channel_cnt;
					'd1: laser_channel_cnt <= 16 + laser_channel_cnt;
					'd2: laser_channel_cnt <= 8  + laser_channel_cnt;
					default:;
					endcase
					laser_state <= 4'h3;
				end
				4'h3:begin
					if(laser_channel_cnt != laser_channel_data)begin
						laser_data[data_cnt] <= (laser_data[data_cnt] == 4'h7) ? 4'h8 : laser_data[data_cnt] + 4'h1;
						data_cnt <= (laser_data[data_cnt] == 4'h7) ? data_cnt - 1'd1 : data_cnt;
					end
					else begin//写完复位
						laser_data[0]  		<= 4'h8;
						laser_data[1]		<= 4'h8;
						laser_data[2]  		<= 4'h8;
						laser_data[3]		<= 4'h8;
						data_cnt <= 3;
					end
					if(data_cnt==0)begin
						data_cnt <= 3;
					end
					laser_state <= 4'h0;
				end
				default:begin
					laser_state <= 4'h0;
				end
				endcase
			end
			else begin
				laser_wr_data 		<= 16'h8888;
				laser_wr_en 		<= 1;
				laser_data[0]  		<= 4'h8;
				laser_data[1]		<= 4'h8;
				laser_data[2]  		<= 4'h8;
				laser_data[3]		<= 4'h8;
				laser_state			<= 4'h0;
				irq_count 			<= 25'h0;
				data_cnt 			<= 3;
				laser_channel_cnt	<= 1;
			end
		end
		else if(stop_cmd_en == 1 || stop_cali_cmd_en == 1)begin
			case(laser_state)
			4'h5:begin
				laser_data[0]  		<= 4'h8;
				laser_data[1]		<= 4'h8;
				laser_data[2]  		<= 4'h8;
				laser_data[3]		<= 4'h8;
				laser_wr_en 		<= 0;
				laser_state			<= 4'h6;
				irq_count 			<= 25'h0;
				data_cnt 			<= 3;
				laser_channel_cnt 	<= 1;
			end
			4'h6:begin
				laser_wr_data 		<= 16'h8888;
				laser_wr_en 		<= 1;
				laser_state 		<= 4'h7;
			end
			4'h7:begin
				if(laser_finish_flag)begin
					laser_wr_en 	<= 0;
				end
			end
			default:begin
				laser_state 		<= 4'h5;
			end
			endcase
		end
	end
end

/********************PGA增益位数********************/
always @(posedge clk)begin
	case(sampling_channel_data)
	'd4:	gain_data_len <= 'd16;
	'd8:	gain_data_len <= 'd32;
	'd16:	gain_data_len <= 'd64;
	'd32:	gain_data_len <= 'd128;
	endcase
end

/********************PGA增益写入********************/
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
        gain_data <= 128'h1111_1111_1111_1111_1111_1111_1111_1111;
		gain_wr_en <= 0;
    end                                                      
    else begin
		if(cali_cmd_en || start_cmd_en)begin
			if(data_valid)begin
				case(laser_channel_data)
				'd4,'d8 :	gain_data <= gain_data_temp1[laser_channel_cnt-1];
				'd16	:	gain_data <= {gain_data_temp2[laser_channel_cnt-1],gain_data_temp1[laser_channel_cnt-1]};
				'd32	:	gain_data <= {gain_data_temp4[laser_channel_cnt-1],gain_data_temp3[laser_channel_cnt-1],
										gain_data_temp2[laser_channel_cnt-1],gain_data_temp1[laser_channel_cnt-1]};
				endcase
				gain_wr_en	<= 1;
			 end				
			 else begin
				gain_wr_en <= 0;
			 end
		end
		else if(stop_cmd_en || stop_cali_cmd_en)begin
       		gain_data <= 128'h1111_1111_1111_1111_1111_1111_1111_1111;
			gain_wr_en <= 1;
		end 
    end
end

/********************读取输入信号幅值********************/
always @(posedge clk) begin
	if(cali_cmd_en && !cali_state)begin
		case(laser_channel_data)
		'd4:begin
			if(cali_timecnt < 'd1000)begin
				for(x=0;x<4;x=x+1)begin
					for(y=0;y<4;y=y+1)begin
						high_temp[x][y] <= 10000;
					end
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<4;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<4;a=a+1)begin
							if(w_ads1278_rdata_1[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_1[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_1[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
		end
		'd8:begin
			if(cali_timecnt < 'd1000)begin
				for(x=0;x<8;x=x+1)begin
					for(y=0;y<8;y=y+1)begin
						high_temp[x][y] <= 10000;
					end
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<8;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<8;a=a+1)begin
							if(w_ads1278_rdata_1[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_1[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_1[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
		end
		'd16:begin
			if((cali_timecnt < 'd1000) || (cali_timecnt < 'd1500001000 && cali_timecnt >= 'd1500000000))begin
				for(x=0;x<16;x=x+1)begin
					for(y=0;y<8;y=y+1)begin
						high_temp[x][y] <= 10000;
					end
				end
			end
			else if(cali_timecnt < 'd1500000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<8;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<8;a=a+1)begin
							if(w_ads1278_rdata_1[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_1[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_1[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1500001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<8;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<8;a=a+1)begin
							if(w_ads1278_rdata_2[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_2[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_2[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
		end
		'd32:begin
			if((cali_timecnt < 'd1000) || 
				(cali_timecnt < 'd1500001000 && cali_timecnt >= 'd1500000000) ||
				(cali_timecnt < 'd3000001000 && cali_timecnt >= 'd3000000000) ||
				(cali_timecnt < 33'd4500001000 && cali_timecnt >= 33'd4500000000))begin
				for(x=0;x<32;x=x+1)begin
					for(y=0;y<8;y=y+1)begin
						high_temp[x][y] <= 10000;
					end
				end
			end
			else if(cali_timecnt < 'd1500000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<8;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<8;a=a+1)begin
							if(w_ads1278_rdata_1[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_1[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_1[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < 'd3000000000 && cali_timecnt >= 'd1500001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<8;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<8;a=a+1)begin
							if(w_ads1278_rdata_2[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_2[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_2[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < 33'd4500000000 && cali_timecnt >= 'd3000001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<8;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<8;a=a+1)begin
							if(w_ads1278_rdata_3[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_3[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_3[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 33'd4500001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(a=0;a<8;a=a+1)begin
							if((high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][a] - low_temp[laser_channel_cnt-1][a]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][a] > low_temp[laser_channel_cnt-1][a])begin
							end else begin
								high_temp[laser_channel_cnt-1][a] <= 10000;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(a=0;a<8;a=a+1)begin
							if(w_ads1278_rdata_4[23 + 24*a -: 16] > high_temp[laser_channel_cnt-1][a] &&
								w_ads1278_rdata_4[23 + 24*a -: 16] < 16'd26000)begin
								high_temp[laser_channel_cnt-1][a] <= w_ads1278_rdata_4[23 + 24*a -: 16];
							end
						end
					end 
				end
			end
		end
		endcase
	end
end

always @(posedge clk) begin
	if(cali_cmd_en && !cali_state)begin
		case(laser_channel_data)
		'd4:begin
			if(cali_timecnt < 'd1000)begin
				for(u=0;u<8;u=u+1)begin
					for(v=0;v<8;v=v+1)begin
						low_temp[u][v] <= 32767;
					end
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<4;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<4;b=b+1)begin
							if(w_ads1278_rdata_1[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_1[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_1[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
		end
		'd8:begin
			if(cali_timecnt < 'd1000)begin
				for(u=0;u<8;u=u+1)begin
					for(v=0;v<8;v=v+1)begin
						low_temp[u][v] <= 32767;
					end
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<8;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<8;b=b+1)begin
							if(w_ads1278_rdata_1[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_1[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_1[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
		end
		'd16:begin
			if((cali_timecnt < 'd1000) || (cali_timecnt < 'd1500001000 && cali_timecnt >= 'd1500000000))begin
				for(u=0;u<16;u=u+1)begin
					for(v=0;v<8;v=v+1)begin
						low_temp[u][v] <= 32767;
					end
				end
			end
			else if(cali_timecnt < 'd1500000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<8;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<8;b=b+1)begin
							if(w_ads1278_rdata_1[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_1[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_1[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1500001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<8;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<8;b=b+1)begin
							if(w_ads1278_rdata_2[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_2[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_2[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
		end
		'd32:begin
			if((cali_timecnt < 'd1000) || 
				(cali_timecnt < 'd1500001000 && cali_timecnt >= 'd1500000000) ||
				(cali_timecnt < 'd3000001000 && cali_timecnt >= 'd3000000000) ||
				(cali_timecnt < 33'd4500001000 && cali_timecnt >= 33'd4500000000))begin
				for(u=0;u<32;u=u+1)begin
					for(v=0;v<8;v=v+1)begin
						low_temp[u][v] <= 32767;
					end
				end
			end
			else if(cali_timecnt < 'd1500000000 && cali_timecnt >= 'd1000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<8;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<8;b=b+1)begin
							if(w_ads1278_rdata_1[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_1[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_1[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < 'd3000000000 && cali_timecnt >= 'd1500001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<8;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<8;b=b+1)begin
							if(w_ads1278_rdata_2[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_2[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_2[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < 33'd4500000000 && cali_timecnt >= 'd3000001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<8;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<8;b=b+1)begin
							if(w_ads1278_rdata_3[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_3[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_3[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 33'd4500001000)begin
				if(data_valid == 1)begin
					if(adjust_cnt[laser_channel_cnt-1] <= 'd1000)begin
						for(b=0;b<8;b=b+1)begin
							if((high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) < HIGH_LIMIT &&
								(high_temp[laser_channel_cnt-1][b] - low_temp[laser_channel_cnt-1][b]) > LOW_LIMIT &&
								high_temp[laser_channel_cnt-1][b] > low_temp[laser_channel_cnt-1][b])begin
							end else begin
								low_temp[laser_channel_cnt-1][b] <= 32767;
							end
						end
					end
					else if(adjust_cnt[laser_channel_cnt-1] > 'd1000 && adjust_cnt[laser_channel_cnt-1] <= 'd2250)begin
						for(b=0;b<8;b=b+1)begin
							if(w_ads1278_rdata_4[23 + 24*b -: 16] < low_temp[laser_channel_cnt-1][b] &&
								w_ads1278_rdata_4[23 + 24*b -: 16] > 16'd1000)begin
								low_temp[laser_channel_cnt-1][b] <= w_ads1278_rdata_4[23 + 24*b -: 16];
							end
						end
					end 
				end
			end
		end
		endcase
	end
end

always @(posedge clk)begin
	if(cali_cmd_en && !cali_state)begin
		case(laser_channel_data)
		'd4,'d8:begin
			if(cali_timecnt < 'd1000)begin
				for(z=0;z<8;z=z+1)begin
					adjust_cnt[z] <= 0;
				end
			end
			else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1000)begin
				if(data_valid)begin
					if(adjust_cnt[laser_channel_cnt-1] < COUNT_TIMES)begin
						adjust_cnt[laser_channel_cnt-1] <= adjust_cnt[laser_channel_cnt-1] + 1'd1;
					end
					else begin
						adjust_cnt[laser_channel_cnt-1] <= 0;
					end
				end
			end
		end
		'd16:begin
			if((cali_timecnt < 'd1000) || (cali_timecnt < 'd1500001000 && cali_timecnt >= 'd1500000000))begin
				for(z=0;z<16;z=z+1)begin
					adjust_cnt[z] <= 0;
				end
			end
			else begin
				if(data_valid && (cali_timecnt < CALI_TIME - 'd50000000))begin
					if(adjust_cnt[laser_channel_cnt-1] < COUNT_TIMES)begin
						adjust_cnt[laser_channel_cnt-1] <= adjust_cnt[laser_channel_cnt-1] + 1'd1;
					end
					else begin
						adjust_cnt[laser_channel_cnt-1] <= 0;
					end
				end
			end
		end
		'd32:begin
			if((cali_timecnt < 'd1000) || 
				(cali_timecnt < 'd1500001000 && cali_timecnt >= 'd1500000000) ||
				(cali_timecnt < 'd3000001000 && cali_timecnt >= 'd3000000000) ||
				(cali_timecnt < 33'd4500001000 && cali_timecnt >= 33'd4500000000))begin
				for(z=0;z<32;z=z+1)begin
					adjust_cnt[z] <= 0;
				end
			end
			else begin
				if(data_valid && (cali_timecnt < CALI_TIME - 'd50000000))begin
					if(adjust_cnt[laser_channel_cnt-1] < COUNT_TIMES)begin
						adjust_cnt[laser_channel_cnt-1] <= adjust_cnt[laser_channel_cnt-1] + 1'd1;
					end
					else begin
						adjust_cnt[laser_channel_cnt-1] <= 0;
					end
				end
			end
		end
		endcase
	end
end

//增益控制
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for(i=0;i<32;i=i+1)begin
			gain_data_temp1[i] <= 'h1111_1111;
		end	
	end
	else begin
		if(cali_cmd_en && !cali_state)begin
			case(laser_channel_data)
			'd4:begin
				if(cali_timecnt < 'd1000)begin
					for(i=0;i<4;i=i+1)begin
						gain_data_temp1[i] <= 'h1111_1111;
					end	
				end
				else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1000)begin
					if(data_valid)begin
						if(adjust_cnt[laser_channel_cnt-1] == COUNT_TIMES)begin
							for(i=0;i<4;i=i+1)begin
								if(high_temp[laser_channel_cnt-1][i] > low_temp[laser_channel_cnt-1][i])begin
									// 信号过强：HIGH_LIMIT 限幅处理
									if((high_temp[laser_channel_cnt-1][i] - low_temp[laser_channel_cnt-1][i]) > HIGH_LIMIT)begin
										case(gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3])
											3'b000: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3] <= 3'b111;
											3'b001: ; // 保持
											default: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] <=
													 gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] - 1'd1;
										endcase
									end
									// 信号过弱：LOW_LIMIT 增益提升
									else if ((high_temp[laser_channel_cnt-1][i] - low_temp[laser_channel_cnt-1][i]) <= LOW_LIMIT) begin
										case (gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3])
											3'b111: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3] <= 3'b000;
											3'b110: ; // 保持
											default: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] <=
													 gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] + 1'd1;
										endcase
									end
								end
							end
						end
					end
				end
			end
			'd8:begin
				if(cali_timecnt < 'd1000)begin
					for(i=0;i<8;i=i+1)begin
						gain_data_temp1[i] <= 'h1111_1111;
					end	
				end
				else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 'd1000)begin
					if(data_valid)begin
						if(adjust_cnt[laser_channel_cnt-1] == COUNT_TIMES)begin
							for(i=0;i<8;i=i+1)begin
								if(high_temp[laser_channel_cnt-1][i] > low_temp[laser_channel_cnt-1][i])begin
									// 信号过强：HIGH_LIMIT 限幅处理
									if((high_temp[laser_channel_cnt-1][i] - low_temp[laser_channel_cnt-1][i]) > HIGH_LIMIT)begin
										case(gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3])
											3'b000: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3] <= 3'b111;
											3'b001: ; // 保持
											default: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] <=
													 gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] - 1'd1;
										endcase
									end
									// 信号过弱：LOW_LIMIT 增益提升
									else if ((high_temp[laser_channel_cnt-1][i] - low_temp[laser_channel_cnt-1][i]) <= LOW_LIMIT) begin
										case (gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3])
											3'b111: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3] <= 3'b000;
											3'b110: ; // 保持
											default: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] <=
													 gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] + 1'd1;
										endcase
									end
								end
							end
						end
					end
				end
			end
			'd16:begin
				if(cali_timecnt < 'd1000)begin
					for(i=0;i<16;i=i+1)begin
						gain_data_temp1[i] <= 'h1111_1111;
					end	
				end
				else if(cali_timecnt < 'd1500000000 && cali_timecnt >= 'd1000)begin
					if(data_valid)begin
						if(adjust_cnt[laser_channel_cnt-1] == COUNT_TIMES)begin
							for(i=0;i<8;i=i+1)begin
								if(high_temp[laser_channel_cnt-1][i] > low_temp[laser_channel_cnt-1][i])begin
									// 信号过强：HIGH_LIMIT 限幅处理
									if((high_temp[laser_channel_cnt-1][i] - low_temp[laser_channel_cnt-1][i]) > HIGH_LIMIT)begin
										case(gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3])
											3'b000: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3] <= 3'b111;
											3'b001: ; // 保持
											default: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] <=
													 gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] - 1'd1;
										endcase
									end
									// 信号过弱：LOW_LIMIT 增益提升
									else if ((high_temp[laser_channel_cnt-1][i] - low_temp[laser_channel_cnt-1][i]) <= LOW_LIMIT) begin
										case (gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3])
											3'b111: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 3] <= 3'b000;
											3'b110: ; // 保持
											default: gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] <=
													 gain_data_temp1[laser_channel_cnt-1][(2 + 4*i) -: 2] + 1'd1;
										endcase
									end
								end
							end
						end
					end
				end
			end
			endcase
		end
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for(j=0;j<32;j=j+1)begin
			gain_data_temp2[j] <= 'h1111_1111;
		end	
	end
	else begin
		if(cali_cmd_en && !cali_state)begin
			case(laser_channel_data)
			'd16,'d32:begin
				if(cali_timecnt < 'd1500001000)begin
					for(j=0;j<32;j=j+1)begin
						gain_data_temp2[j] <= 'h1111_1111;
					end	
				end
				else if(cali_timecnt < 'd3000000000 && cali_timecnt >= 'd1500001000)begin
					if(data_valid)begin
						if(adjust_cnt[laser_channel_cnt-1] == COUNT_TIMES)begin
							for(j=0;j<8;j=j+1)begin
								if(high_temp[laser_channel_cnt-1][j] > low_temp[laser_channel_cnt-1][j])begin
									// 信号过强：HIGH_LIMIT 限幅处理
									if((high_temp[laser_channel_cnt-1][j] - low_temp[laser_channel_cnt-1][j]) > HIGH_LIMIT)begin
										case(gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 3])
											3'b000: gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 3] <= 3'b111;
											3'b001: ; // 保持
											default: gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 2] <=
													 gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 2] - 1'd1;
										endcase
									end
									// 信号过弱：LOW_LIMIT 增益提升
									else if ((high_temp[laser_channel_cnt-1][j] - low_temp[laser_channel_cnt-1][j]) <= LOW_LIMIT) begin
										case (gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 3])
											3'b111: gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 3] <= 3'b000;
											3'b110: ; // 保持
											default: gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 2] <=
													 gain_data_temp2[laser_channel_cnt-1][(2 + 4*j) -: 2] + 1'd1;
										endcase
									end
								end
							end
						end
					end
				end
			end
			endcase
		end
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for(o=0;o<32;o=o+1)begin
			gain_data_temp3[o] <= 'h1111_1111;
		end	
	end
	else begin
		if(cali_cmd_en && !cali_state)begin
			case(laser_channel_data)
			'd32:begin
				if(cali_timecnt < 'd3000001000)begin
					for(o=0;o<32;o=o+1)begin
						gain_data_temp3[o] <= 'h1111_1111;
					end	
				end
				else if(cali_timecnt < 33'd4500000000 && cali_timecnt >= 'd3000001000)begin
					if(data_valid)begin
						if(adjust_cnt[laser_channel_cnt-1] == COUNT_TIMES)begin
							for(o=0;o<8;o=o+1)begin
								if(high_temp[laser_channel_cnt-1][o] > low_temp[laser_channel_cnt-1][o])begin
									// 信号过强：HIGH_LIMIT 限幅处理
									if((high_temp[laser_channel_cnt-1][o] - low_temp[laser_channel_cnt-1][o]) > HIGH_LIMIT)begin
										case(gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 3])
											3'b000: gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 3] <= 3'b111;
											3'b001: ; // 保持
											default: gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 2] <=
													 gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 2] - 1'd1;
										endcase
									end
									// 信号过弱：LOW_LIMIT 增益提升
									else if ((high_temp[laser_channel_cnt-1][o] - low_temp[laser_channel_cnt-1][o]) <= LOW_LIMIT) begin
										case (gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 3])
											3'b111: gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 3] <= 3'b000;
											3'b110: ; // 保持
											default: gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 2] <=
													 gain_data_temp3[laser_channel_cnt-1][(2 + 4*o) -: 2] + 1'd1;
										endcase
									end
								end
							end
						end
					end
				end
			end
			endcase
		end
	end
end
	
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for(p=0;p<32;p=p+1)begin
			gain_data_temp4[p] <= 'h1111_1111;
		end	
	end
	else begin
		if(cali_cmd_en && !cali_state)begin
			case(laser_channel_data)
			'd32:begin
				if(cali_timecnt < 33'd4500001000)begin
					for(p=0;p<32;p=p+1)begin
						gain_data_temp4[p] <= 'h1111_1111;
					end	
				end
				else if(cali_timecnt < CALI_TIME - 'd50000000 && cali_timecnt >= 33'd4500001000)begin
					if(data_valid)begin
						if(adjust_cnt[laser_channel_cnt-1] == COUNT_TIMES)begin
							for(p=0;p<8;p=p+1)begin
								if(high_temp[laser_channel_cnt-1][p] > low_temp[laser_channel_cnt-1][p])begin
									// 信号过强：HIGH_LIMIT 限幅处理
									if((high_temp[laser_channel_cnt-1][p] - low_temp[laser_channel_cnt-1][p]) > HIGH_LIMIT)begin
										case(gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 3])
											3'b000: gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 3] <= 3'b111;
											3'b001: ; // 保持
											default: gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 2] <=
													 gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 2] - 1'd1;
										endcase
									end
									// 信号过弱：LOW_LIMIT 增益提升
									else if ((high_temp[laser_channel_cnt-1][p] - low_temp[laser_channel_cnt-1][p]) <= LOW_LIMIT) begin
										case (gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 3])
											3'b111: gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 3] <= 3'b000;
											3'b110: ; // 保持
											default: gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 2] <=
													 gain_data_temp4[laser_channel_cnt-1][(2 + 4*p) -: 2] + 1'd1;
										endcase
									end
								end
							end
						end
					end
				end
			end
			endcase
		end
	end
end
	
always@(posedge clk)
	if(w_user_rdata_ready_1)
		data_valid <= 1;
	else
		data_valid <= 0;

always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		send_pkg_start_f1 <= 1;
		send_pkg_start_f2 <= 1;
		send_data_start_f1 <= 1;
		send_data_start_f2 <= 1;
	end
	else begin
		if(channel_count==0 && adc_en==1)begin
			send_pkg_start_f1 <= 1;
		end
		else begin
			send_pkg_start_f1 <= 0;
		end
		if(adc_en==1 && data_valid)begin
			send_data_start_f1 <= 1;
		end
		else begin
			send_data_start_f1 <= 0;
		end
		send_data_start_f2 <= send_data_start_f1; 
		send_pkg_start_f2 <= send_pkg_start_f1;         
	end
end			

always @(posedge clk or negedge rst_n) begin  
	 if (!rst_n) begin
		daq_pkg_data_cnt <= 0;
		send_cnt <= 0;
		send_pack_finish <= 0;
		cali_send_pack_finish <= 0;
		stop_cali_cmd_en <= 0;
		send_tri_start	<= 0;
	 end
	 else begin	
		if(stop_cmd_en || stop_cali_cmd_en)begin
			send_cnt <= 0;
			send_pack_finish <= 0;
			cali_send_pack_finish <= 0;
			send_tri_start	<= 0;
		end

		if(start_cmd_en)begin
			if(channel_count==0 && send_pack_finish == 0)begin
				send_pkg_data[0] <= 8'h3a;
				send_pkg_data[1] <= datapkg_length[7:0];
				send_pkg_data[2] <= datapkg_length[15:8];
				send_pkg_data[3] <= 8'h00;
				send_pkg_data[4] <= 8'h00;
				send_pkg_data[5] <= 8'h02;//功能码
				send_pkg_data[6] <= 8'h00;//
				send_pkg_data[7] <= laser_channel_cnt - 1'b1;//
				send_pkg_data[8] <= channel_count_pkg[7:0];
				send_pkg_data[9] <= channel_count_pkg[15:8];
				daq_pkg_data_cnt <= 10;   	      
				send_pack_finish <= 1;			
				send_cnt		 <= 0;			
			end
			if(send_pack_finish == 0 && data_valid)begin	
				send_cnt <= send_cnt + 1'd1;
				if(send_cnt >=0)begin
					send_pkg_data[0] 	<= w_ads1278_rdata_1[15:8 ];
					send_pkg_data[1] 	<= w_ads1278_rdata_1[23:16];
					send_pkg_data[2] 	<= w_ads1278_rdata_1[39:32];
					send_pkg_data[3] 	<= w_ads1278_rdata_1[47:40];
					send_pkg_data[4] 	<= w_ads1278_rdata_1[63:56];
					send_pkg_data[5] 	<= w_ads1278_rdata_1[71:64];
					send_pkg_data[6] 	<= w_ads1278_rdata_1[87:80];
					send_pkg_data[7] 	<= w_ads1278_rdata_1[95:88];
					send_pkg_data[8] 	<= w_ads1278_rdata_1[111:104];
					send_pkg_data[9]	<= w_ads1278_rdata_1[119:112];
					send_pkg_data[10]	<= w_ads1278_rdata_1[135:128];
					send_pkg_data[11] 	<= w_ads1278_rdata_1[143:136];
					send_pkg_data[12] 	<= w_ads1278_rdata_1[159:152];
					send_pkg_data[13] 	<= w_ads1278_rdata_1[167:160];
					send_pkg_data[14] 	<= w_ads1278_rdata_1[183:176];
					send_pkg_data[15] 	<= w_ads1278_rdata_1[191:184];
					send_pkg_data[16] 	<= w_ads1278_rdata_2[15:8 ];
					send_pkg_data[17] 	<= w_ads1278_rdata_2[23:16];
					send_pkg_data[18] 	<= w_ads1278_rdata_2[39:32];
					send_pkg_data[19] 	<= w_ads1278_rdata_2[47:40];
					send_pkg_data[20] 	<= w_ads1278_rdata_2[63:56];
					send_pkg_data[21] 	<= w_ads1278_rdata_2[71:64];
					send_pkg_data[22] 	<= w_ads1278_rdata_2[87:80];
					send_pkg_data[23] 	<= w_ads1278_rdata_2[95:88];
					send_pkg_data[24] 	<= w_ads1278_rdata_2[111:104];
					send_pkg_data[25]	<= w_ads1278_rdata_2[119:112];
					send_pkg_data[26]	<= w_ads1278_rdata_2[135:128];
					send_pkg_data[27] 	<= w_ads1278_rdata_2[143:136];
					send_pkg_data[28] 	<= w_ads1278_rdata_2[159:152];
					send_pkg_data[29] 	<= w_ads1278_rdata_2[167:160];
					send_pkg_data[30] 	<= w_ads1278_rdata_2[183:176];
					send_pkg_data[31] 	<= w_ads1278_rdata_2[191:184];
					send_pkg_data[32] 	<= w_ads1278_rdata_3[15:8 ];
					send_pkg_data[33] 	<= w_ads1278_rdata_3[23:16];
					send_pkg_data[34] 	<= w_ads1278_rdata_3[39:32];
					send_pkg_data[35] 	<= w_ads1278_rdata_3[47:40];
					send_pkg_data[36] 	<= w_ads1278_rdata_3[63:56];
					send_pkg_data[37] 	<= w_ads1278_rdata_3[71:64];
					send_pkg_data[38] 	<= w_ads1278_rdata_3[87:80];
					send_pkg_data[39] 	<= w_ads1278_rdata_3[95:88];
					send_pkg_data[40] 	<= w_ads1278_rdata_3[111:104];
					send_pkg_data[41]	<= w_ads1278_rdata_3[119:112];
					send_pkg_data[42]	<= w_ads1278_rdata_3[135:128];
					send_pkg_data[43] 	<= w_ads1278_rdata_3[143:136];
					send_pkg_data[44] 	<= w_ads1278_rdata_3[159:152];
					send_pkg_data[45] 	<= w_ads1278_rdata_3[167:160];
					send_pkg_data[46] 	<= w_ads1278_rdata_3[183:176];
					send_pkg_data[47] 	<= w_ads1278_rdata_3[191:184];
					send_pkg_data[48] 	<= w_ads1278_rdata_4[15:8 ];
					send_pkg_data[49] 	<= w_ads1278_rdata_4[23:16];
					send_pkg_data[50] 	<= w_ads1278_rdata_4[39:32];
					send_pkg_data[51] 	<= w_ads1278_rdata_4[47:40];
					send_pkg_data[52] 	<= w_ads1278_rdata_4[63:56];
					send_pkg_data[53] 	<= w_ads1278_rdata_4[71:64];
					send_pkg_data[54] 	<= w_ads1278_rdata_4[87:80];
					send_pkg_data[55] 	<= w_ads1278_rdata_4[95:88];
					send_pkg_data[56] 	<= w_ads1278_rdata_4[111:104];
					send_pkg_data[57]	<= w_ads1278_rdata_4[119:112];
					send_pkg_data[58]	<= w_ads1278_rdata_4[135:128];
					send_pkg_data[59] 	<= w_ads1278_rdata_4[143:136];
					send_pkg_data[60] 	<= w_ads1278_rdata_4[159:152];
					send_pkg_data[61] 	<= w_ads1278_rdata_4[167:160];
					send_pkg_data[62] 	<= w_ads1278_rdata_4[183:176];
					send_pkg_data[63] 	<= w_ads1278_rdata_4[191:184];		
					daq_pkg_data_cnt <= sampling_channel_data*2'd2;
					send_pack_finish <= 1;
				end
				if(send_cnt == 624)begin
					send_pkg_data[sampling_channel_data*2] <= 8'h1a;
					send_pkg_data[sampling_channel_data*2+1] <= 8'h0a;
					daq_pkg_data_cnt <= sampling_channel_data*2'd2+2'd2;
				end
			end
			//同步触发信号回上位机
			if(tri_en_flag)begin
				send_tri_start	<= 1;
			end
			if(send_cnt == 625 && send_tri_start && send_pack_finish == 0)begin
				send_pkg_data[0]	<= 8'h3a;
				send_pkg_data[1]	<= 8'h05;//长度
				send_pkg_data[2]	<= 8'h00;
				send_pkg_data[3]	<= 8'h00;
				send_pkg_data[4]	<= 8'h00;
				send_pkg_data[5]	<= 8'h04;//功能码
				send_pkg_data[6]	<= 8'h00;//
				send_pkg_data[7]	<= tri_data_temp;//触发
				send_pkg_data[8]	<= tri_timing[7:0];//触发时刻
				send_pkg_data[9]	<= tri_timing[15:8];
				send_pkg_data[10]	<= 8'h00;
				send_pkg_data[11]	<= 8'h0a;				
				daq_pkg_data_cnt	<= 12;  
				send_pack_finish	<= 1;
				send_tri_start		<= 0;
			end
			if(fifo_cnt == daq_pkg_data_cnt) begin
				send_pack_finish <= 0;
				cali_send_pack_finish <= 0;
			end
		end
		if(check_cmd_en || check_cmd_refresh)begin
			if(check_cmd_refresh && !send_pack_finish)begin
				send_pkg_data[0 ] <= 8'h3a;							//包头
				send_pkg_data[1 ] <= 8'h11;							//长度17
				send_pkg_data[2 ] <= 8'h00;							//
				send_pkg_data[3 ] <= 8'h00;							//
				send_pkg_data[4 ] <= 8'h00;							//
				send_pkg_data[5 ] <= 8'h00;							//功能码
				send_pkg_data[6 ] <= 8'h00;							//子功能码
				send_pkg_data[7 ] <= 8'h01;                			//
				send_pkg_data[8 ] <= 8'h00;                			//
				send_pkg_data[9 ] <= 8'h00;                			//
				send_pkg_data[10] <= 8'h00;                			//
				send_pkg_data[11] <= laser_channel_data;        	//光源数量
				send_pkg_data[12] <= sampling_channel_data;     	//探测器数量
				send_pkg_data[13] <= sampling_frequency_data;   	//光源切换频率            
				send_pkg_data[14] <= 8'h00;               			//  
				send_pkg_data[15] <= 8'h00;             			//  
				send_pkg_data[16] <= 8'h00;             			//  
				send_pkg_data[17] <= channel_samp_frequency[7:0];	//采样率
				send_pkg_data[18] <= channel_samp_frequency[15:8];	//
				send_pkg_data[19] <= 8'h00;							//
				send_pkg_data[20] <= 8'h00;							//
				send_pkg_data[21] <= LASER_VERSION;					//光源波长类型	
				send_pkg_data[22] <= 8'h00;               			
				send_pkg_data[23] <= 8'h0a;               			//包尾
				daq_pkg_data_cnt <= 8'h18;                			//长度24		
				check_cmd_en <= 1;
			end
			else if(check_cmd_en && !send_pack_finish && !check_send_pack_finish)begin
				check_send_pack_finish <= 1;
			end
			else if(fifo_cnt == daq_pkg_data_cnt)begin
				check_send_pack_finish <= 0;
				check_cmd_en <= 0;
			end
		end
			
		if(cali_cmd_en)begin
			if(send_pkg_start_f && cali_send_pack_finish==0)begin
				send_pkg_data[0] <= 8'h3a;
				send_pkg_data[1] <= calipkg_length & 16'h00_ff;
				send_pkg_data[2] <= calipkg_length >>8 & 16'h00_ff;
				send_pkg_data[3] <= 8'h00;
				send_pkg_data[4] <= 8'h00;
				send_pkg_data[5] <= 8'h01;
				send_pkg_data[6] <= 8'h00;
				send_pkg_data[7] <= cali_state;//校准状态
				daq_pkg_data_cnt <= 8;
				cali_send_pack_finish <= 1;
				send_cnt		 <= 0;
			end
			if(data_valid && cali_send_pack_finish==0 && (send_cnt < sampling_channel_data+1))begin		
				send_cnt <= send_cnt + 1'd1;
				if(send_cnt >0)begin
					send_pkg_data[0 ] <= gain_data_temp1[send_cnt-1][ 2 -:3];
					send_pkg_data[1 ] <= gain_data_temp1[send_cnt-1][ 6 -:3];
					send_pkg_data[2 ] <= gain_data_temp1[send_cnt-1][10 -:3];
					send_pkg_data[3 ] <= gain_data_temp1[send_cnt-1][14 -:3];
					send_pkg_data[4 ] <= gain_data_temp1[send_cnt-1][18 -:3];
					send_pkg_data[5 ] <= gain_data_temp1[send_cnt-1][22 -:3];
					send_pkg_data[6 ] <= gain_data_temp1[send_cnt-1][26 -:3];
					send_pkg_data[7 ] <= gain_data_temp1[send_cnt-1][30 -:3];
					send_pkg_data[8 ] <= gain_data_temp2[send_cnt-1][ 2 -:3];
					send_pkg_data[9 ] <= gain_data_temp2[send_cnt-1][ 6 -:3];
					send_pkg_data[10] <= gain_data_temp2[send_cnt-1][10 -:3];
					send_pkg_data[11] <= gain_data_temp2[send_cnt-1][14 -:3];
					send_pkg_data[12] <= gain_data_temp2[send_cnt-1][18 -:3];
					send_pkg_data[13] <= gain_data_temp2[send_cnt-1][22 -:3];
					send_pkg_data[14] <= gain_data_temp2[send_cnt-1][26 -:3];
					send_pkg_data[15] <= gain_data_temp2[send_cnt-1][30 -:3];
					send_pkg_data[16] <= gain_data_temp3[send_cnt-1][ 2 -:3];
					send_pkg_data[17] <= gain_data_temp3[send_cnt-1][ 6 -:3];
					send_pkg_data[18] <= gain_data_temp3[send_cnt-1][10 -:3];
					send_pkg_data[19] <= gain_data_temp3[send_cnt-1][14 -:3];
					send_pkg_data[20] <= gain_data_temp3[send_cnt-1][18 -:3];
					send_pkg_data[21] <= gain_data_temp3[send_cnt-1][22 -:3];
					send_pkg_data[22] <= gain_data_temp3[send_cnt-1][26 -:3];
					send_pkg_data[23] <= gain_data_temp3[send_cnt-1][30 -:3];
					send_pkg_data[24] <= gain_data_temp4[send_cnt-1][ 2 -:3];
					send_pkg_data[25] <= gain_data_temp4[send_cnt-1][ 6 -:3];
					send_pkg_data[26] <= gain_data_temp4[send_cnt-1][10 -:3];
					send_pkg_data[27] <= gain_data_temp4[send_cnt-1][14 -:3];
					send_pkg_data[28] <= gain_data_temp4[send_cnt-1][18 -:3];
					send_pkg_data[29] <= gain_data_temp4[send_cnt-1][22 -:3];
					send_pkg_data[30] <= gain_data_temp4[send_cnt-1][26 -:3];
					send_pkg_data[31] <= gain_data_temp4[send_cnt-1][30 -:3];
					cali_send_pack_finish <= 1;
					daq_pkg_data_cnt <= sampling_channel_data;
				end
				if(send_cnt==sampling_channel_data)begin
					send_pkg_data[sampling_channel_data] <= 8'h00;
					send_pkg_data[sampling_channel_data+1] <= 8'h0a;
					daq_pkg_data_cnt <= sampling_channel_data+2'd2;
				end
			end
		  	if(fifo_cnt == daq_pkg_data_cnt) begin
				send_pack_finish <= 0;
				cali_send_pack_finish <= 0;
			end
		end
		if(stop_cali_delay_flag||stop_cali_cmd_en)begin
			if(stop_cali_delay_flag && !send_pack_finish)begin
				send_pkg_data[0] <= 8'h3a;
				send_pkg_data[1] <= 8'h03;
				send_pkg_data[2] <= 8'h00;
				send_pkg_data[3] <= 8'h00;
				send_pkg_data[4] <= 8'h00;
				send_pkg_data[5] <= 8'h01;
				send_pkg_data[6] <= 8'h00;
				send_pkg_data[7] <= 8'h02;
				send_pkg_data[8] <= 8'h00;
				send_pkg_data[9] <= 8'h0a;
				daq_pkg_data_cnt <= 10;
				stop_cali_cmd_en <= 1;						
		  	end
			else if(stop_cali_cmd_en && !send_pack_finish && !check_send_pack_finish)begin
				check_send_pack_finish <= 1;
			end
			else if(fifo_cnt == daq_pkg_data_cnt)begin
				check_send_pack_finish <= 0;
				stop_cali_cmd_en <= 0;
			end
		end
	end
end

always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
       	fifo_cnt <= 0;
		ft232_wrreq <= 0;
    end                                                      
    else begin                                               
        if(send_pack_finish||check_send_pack_finish||cali_send_pack_finish) begin
			 if(fifo_cnt < daq_pkg_data_cnt)begin  
				 ft232_wrreq <= 1;
				 ft232_data_in <= send_pkg_data[fifo_cnt];
				 fifo_cnt <= fifo_cnt + 1'd1;
			 end
			 else begin
				ft232_wrreq <= 0;
				fifo_cnt <= 0;
			 end
		end
		else begin
			 fifo_cnt <= 0;
			 ft232_wrreq <= 0;
		end
    end
end

ft232hq_send u_ft232hq_send(
	.clock				(ft232_clk),
	.rst_n				(rst_n),
	
	.rd_n				(ft232_rd_n),
	.txe_n				(ft232_txe_n),
	.wr_n				(ft232_wr_n),
	.data_send			(ft232_data_send),
	.ft232_siwu_n		(ft232_siwu_n),
	.fifo_data_out		(ft232_data_out),
	.fifo_empty_n		(fifo_empty_n),
	.fifo_rdusedw		(rdusedw_cnt),
	.fifo_rd_en			(ft232_rdreq)
);

//例?��fifo
ft232hq_send_fifo u_ft232_send_fifo(
	.Data				(ft232_data_in), //input [7:0] Data
	.WrClk				(clk), //input WrClk
	.RdClk				(ft232_clk), //input RdClk
	.WrEn				(ft232_wrreq), //input WrEn
	.RdEn				(ft232_rdreq), //input RdEn
	.Wnum				(wrusedw_cnt), //output [12:0] Wnum
	.Rnum				(rdusedw_cnt), //output [12:0] Rnum
	.Empty				(fifo_empty_n),
	.Q					(ft232_data_out) //output [7:0] Q
);

ads1278_drive_1#(
    .P_ADS1278_MODE         (2'b01),             	// 0-快速 1-精准 2-低功耗 3-低速
    .P_ADS1278_PWDN         (8'b1111_1111)       	// 通道开关 0-关 1-开
)
ads1278_drive_u1(
    .i_sysclk               (clk),          		// 时钟
    .i_rst_n                (rst_n),          		// 复位信号
    .i_start                (adc_en),          		// 启动信号

    .o_ads1278_clk          (o_ads1278_clk_1),     	// 芯片时钟
    .o_ads1278_clk_div      (o_ads1278_clk_div_1), 	// 芯片时钟分频设置
    .o_ads1278_fsync        (o_ads1278_fsync_1),   	// 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk         (o_ads1278_sclk_1),    	// SPI时钟
    .i_ads1278_data         (i_ads1278_data_1),    	// 8通道数据输入
    .o_ads1278_format       (o_ads1278_format_1),  	// 数据格式设置
    .o_ads1278_mode         (o_ads1278_mode_1),    	// 工作模式设置
    .o_ads1278_pwdn         (o_ads1278_pwdn_1),    	// 通道开关设置
    .o_ads1278_sync         (o_ads1278_sync_1),    	// 多片同步信号
    .o_ads1278_test         (o_ads1278_test_1),    	// 测试模式设置

	.i_user_clk				(clk),
	.o_user_rdata			(w_ads1278_rdata_1),
	.o_user_rdata_valid		(w_user_rdata_valid_1),
	.i_user_rdata_ready		(w_user_rdata_ready_1)
);

ads1278_drive_2#(
    .P_ADS1278_MODE         (2'b01),             	// 0-快速 1-精准 2-低功耗 3-低速
    .P_ADS1278_PWDN         (8'b1111_1111)       	// 通道开关 0-关 1-开
)
ads1278_drive_u2(
    .i_sysclk               (clk),          		// 时钟
    .i_rst_n                (rst_n),          		// 复位信号
    .i_start                (adc2_en),          	// 启动信号

    .o_ads1278_clk          (o_ads1278_clk_2),     	// 芯片时钟
    .o_ads1278_clk_div      (o_ads1278_clk_div_2), 	// 芯片时钟分频设置
    .o_ads1278_fsync        (o_ads1278_fsync_2),   	// 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk         (o_ads1278_sclk_2),    	// SPI时钟
    .i_ads1278_data         (i_ads1278_data_2),    	// 8通道数据输入
    .o_ads1278_format       (o_ads1278_format_2),  	// 数据格式设置
    .o_ads1278_mode         (o_ads1278_mode_2),    	// 工作模式设置
    .o_ads1278_pwdn         (o_ads1278_pwdn_2),    	// 通道开关设置
    .o_ads1278_sync         (o_ads1278_sync_2),    	// 多片同步信号
    .o_ads1278_test         (o_ads1278_test_2),    	// 测试模式设置

	.i_user_clk				(clk),
	.o_user_rdata			(w_ads1278_rdata_2),
	.o_user_rdata_valid		(w_user_rdata_valid_2),
	.i_user_rdata_ready		(w_user_rdata_ready_2)
);

ads1278_drive_3#(
    .P_ADS1278_MODE         (2'b01),             	// 0-快速 1-精准 2-低功耗 3-低速
    .P_ADS1278_PWDN         (8'b1111_1111)       	// 通道开关 0-关 1-开
)
ads1278_drive_u3(
    .i_sysclk               (clk),          		// 时钟
    .i_rst_n                (rst_n),          		// 复位信号
    .i_start                (adc3_en),          	// 启动信号

    .o_ads1278_clk          (o_ads1278_clk_3),     	// 芯片时钟
    .o_ads1278_clk_div      (o_ads1278_clk_div_3), 	// 芯片时钟分频设置
    .o_ads1278_fsync        (o_ads1278_fsync_3),   	// 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk         (o_ads1278_sclk_3),    	// SPI时钟
    .i_ads1278_data         (i_ads1278_data_3),    	// 8通道数据输入
    .o_ads1278_format       (o_ads1278_format_3),  	// 数据格式设置
    .o_ads1278_mode         (o_ads1278_mode_3),    	// 工作模式设置
    .o_ads1278_pwdn         (o_ads1278_pwdn_3),    	// 通道开关设置
    .o_ads1278_sync         (o_ads1278_sync_3),    	// 多片同步信号
    .o_ads1278_test         (o_ads1278_test_3),    	// 测试模式设置

	.i_user_clk				(clk),
	.o_user_rdata			(w_ads1278_rdata_3),
	.o_user_rdata_valid		(w_user_rdata_valid_3),
	.i_user_rdata_ready		(w_user_rdata_ready_3)
);

ads1278_drive_4#(
    .P_ADS1278_MODE         (2'b01),             	// 0-快速 1-精准 2-低功耗 3-低速
    .P_ADS1278_PWDN         (8'b1111_1111)       	// 通道开关 0-关 1-开
)
ads1278_drive_u4(
    .i_sysclk               (clk),          		// 时钟
    .i_rst_n                (rst_n),          		// 复位信号
    .i_start                (adc4_en),          	// 启动信号

    .o_ads1278_clk          (o_ads1278_clk_4),     	// 芯片时钟
    .o_ads1278_clk_div      (o_ads1278_clk_div_4), 	// 芯片时钟分频设置
    .o_ads1278_fsync        (o_ads1278_fsync_4),   	// 芯片帧同步（ADC转化设置）
    .o_ads1278_sclk         (o_ads1278_sclk_4),    	// SPI时钟
    .i_ads1278_data         (i_ads1278_data_4),    	// 8通道数据输入
    .o_ads1278_format       (o_ads1278_format_4),  	// 数据格式设置
    .o_ads1278_mode         (o_ads1278_mode_4),    	// 工作模式设置
    .o_ads1278_pwdn         (o_ads1278_pwdn_4),    	// 通道开关设置
    .o_ads1278_sync         (o_ads1278_sync_4),    	// 多片同步信号
    .o_ads1278_test         (o_ads1278_test_4),    	// 测试模式设置

	.i_user_clk				(clk),
	.o_user_rdata			(w_ads1278_rdata_4),
	.o_user_rdata_valid		(w_user_rdata_valid_4),
	.i_user_rdata_ready		(w_user_rdata_ready_4)
);


//例?��gain
hc595d_drive gain_hc595d_drive(
	.clk					(clk),
	.rst_n					(rst_n),
	
	.hc595d_data			(gain_data),
	.hc595d_data_len		(gain_data_len),
	.hc595d_wr_en			(gain_wr_en),
	
	.hc595d_rck				(gain_rck),
	.hc595d_scl				(gain_scl),
	.hc595d_sck				(gain_sck),
	.hc595d_ser				(gain_ser),
	.hc595d_wr_finish		(gain_wr_finish)
);

hc595pw_drive laser_hc595pw_drive(
	.clk					(clk),
	.rst_n					(rst_n),
	
	.hc595d_data			(laser_wr_data),
	.hc595d_data_len		(laser_data_len),
	.hc595d_wr_en			(laser_wr_en),
	
	.hc595d_rck				(laser_rck),
	.hc595d_scl				(laser_scl),
	.hc595d_sck				(laser_sck),
	.hc595d_ser				(laser_ser),
	.hc595d_wr_finish		(laser_finish_flag)
);

//例?��WL1
ad9833 ad9833_wl1(
	.clk					(clk),
	.rst_n					(rst_n),
	
	.ad9833_data			(wl1_data),
	.ad9833_wr_en			(wl1_wr_en),
	.ad9833_fsync			(wl1_fsync),
	.ad9833_sclk			(wl1_sclk),
	.ad9833_sdata			(wl1_sdata),
	.ad9833_wr_finish		(wl1_wr_finish)
);

//例?��WL2
ad9833 ad9833_wl2(
	.clk					(clk),
	.rst_n					(rst_n),
	
	.ad9833_data			(wl2_data),
	.ad9833_wr_en			(wl2_wr_en),
	.ad9833_fsync			(wl2_fsync),
	.ad9833_sclk			(wl2_sclk),
	.ad9833_sdata			(wl2_sdata),
	.ad9833_wr_finish		(wl2_wr_finish)
);

endmodule