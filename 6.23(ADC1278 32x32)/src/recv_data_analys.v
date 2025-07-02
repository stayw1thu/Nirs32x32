module recv_data_analys#(
	parameter  				CLK_FREQ			= 	50000000	       //定义系统时钟频率
)(
	input 					clk,
	input					rst_n,
	
	//ft232接口
	input  wire				ft232_clk,
	input  wire				ft232_rxf_n,
	input  wire				ft232_wr_n,
	input  wire 	[7:0]	ft232_data_recv,
	
	output wire				ft232_oe_n,
	output wire				ft232_rd_n,
	
	//状态
	input  wire         	state_flag,//完成标志
	output reg		[5:0]   state,		
	
	//采样通道数据
	output reg  	[7:0]  	sampling_channel_data,  
	output reg  	[7:0]  	laser_channel_data,
	output reg  	[7:0]  	sampling_frequency_data,
	output reg  	[15:0]  channel_samp_frequency,

	//触发输出数据
	output	reg		[7:0]	o_trigger_out_data
);

//包含文件
`include "recv_cmd_para.v"

localparam  	FREE_STATE 			= 	6'b000000	,     	//空闲状态
				CHECK_STATE 		= 	6'b000001	,     	//查询状态
				CONFIG_STATE 		= 	6'b000010	,    	//配置状态
				START_STATE 		= 	6'b000100	,     	//开始状态
				STOP_STATE 			= 	6'b001000	,      	//停止状态
				CALIBRATION_STATE 	= 	6'b010000	,		//校准状态
				STOP_CALI_STATE 	= 	6'b100000	,		//停止校准
				TRIGGER_OUT_FLAG	=	6'b100001	,		//触发输出标志
				PKG_CNT  			= 	8'd40		;		//数据包缓存数组长度

reg  [7:0]   	sampling_channel_temp;  
reg  [7:0]   	laser_channel_temp;
reg  [7:0]   	sampling_frequency_temp;
reg  [15:0]   	channel_samp_temp;

//ft232 fifo写接口
wire 			ft232_wrreq;
wire [7:0] 		ft232_data_in;
wire 			ft232_wrfull;

reg  			ft232_rdreq;
wire [7:0] 		fifo_data_out;
wire 			ft232_rdempty;

//接收上位机发送数据包接口
reg [7:0]		recv_pkg[PKG_CNT-1:0];//数据包长度为40byte
reg 			recv_flag;
reg				recv_flag_temp;
reg	[7:0]		recv_cnt;
reg				recv_finish;
reg [15:0]		recv_cmd;//数据包的命令
reg	[7:0]		recv_check_data;//数据的校验
reg 			recv_check_flag;//数据校验标志
reg	[7:0]		recv_check_cnt;//数据校验计数
reg	[31:0]		recv_pkg_defcnt;//数据包变长度定义

//解析数据包接口
reg 			analysis_en;//解析使能
reg 			analysis_flag;//解析标志
reg 			analysis_finish;//解析完成标志

//*****************************************************
//**                    main code
//*****************************************************
integer i;
//ft232 fifo读
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
		ft232_rdreq <= 1'b0;
		recv_flag <= 0;
		recv_flag_temp <= 0;
    end                                                      
    else begin                                               
		if(!ft232_rdempty && ft232_rxf_n == 1) begin  //fifo不为空且ft232没接收到数据时
			ft232_rdreq <= 1'b1;
			recv_flag_temp <= 1'b1; 
			recv_flag <= recv_flag_temp;//更新数据包标志为1
		end
		else begin
			ft232_rdreq <= 1'b0;
			recv_flag <= 0;
			recv_flag_temp <= 0;
		end
    end
end

//更新数据包
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
		recv_cnt <= 0;
		recv_finish <= 0;
    end                                                      
    else begin                                               
        if(recv_flag)begin
			recv_pkg[0] <= recv_pkg[1];
			recv_pkg[1] <= recv_pkg[2];
			recv_pkg[2] <= recv_pkg[3];
			recv_pkg[3] <= recv_pkg[4];
			recv_pkg[4] <= recv_pkg[5];
			recv_pkg[5] <= recv_pkg[6];
			recv_pkg[6] <= recv_pkg[7];
			recv_pkg[7] <= recv_pkg[8];
			recv_pkg[8] <= recv_pkg[9];
			recv_pkg[9] <= recv_pkg[10];
			recv_pkg[10] <= recv_pkg[11];
			recv_pkg[11] <= recv_pkg[12];
			recv_pkg[12] <= recv_pkg[13];
			recv_pkg[13] <= recv_pkg[14];
			recv_pkg[14] <= recv_pkg[15];
			recv_pkg[15] <= recv_pkg[16];
			recv_pkg[16] <= recv_pkg[17];
			recv_pkg[17] <= recv_pkg[18];
			recv_pkg[18] <= recv_pkg[19];
			recv_pkg[19] <= recv_pkg[20];
			recv_pkg[20] <= recv_pkg[21];
			recv_pkg[21] <= recv_pkg[22];
			recv_pkg[22] <= recv_pkg[23];
			recv_pkg[23] <= recv_pkg[24];
			recv_pkg[24] <= recv_pkg[25];
			recv_pkg[25] <= recv_pkg[26];
			recv_pkg[26] <= recv_pkg[27];
			recv_pkg[27] <= recv_pkg[28];
			recv_pkg[28] <= recv_pkg[29];
			recv_pkg[29] <= recv_pkg[30];
			recv_pkg[30] <= recv_pkg[31];
			recv_pkg[31] <= recv_pkg[32];
			recv_pkg[32] <= recv_pkg[33];
			recv_pkg[33] <= recv_pkg[34];
			recv_pkg[34] <= recv_pkg[35];
			recv_pkg[35] <= recv_pkg[36];
			recv_pkg[36] <= recv_pkg[37];
			recv_pkg[37] <= recv_pkg[38];
			recv_pkg[38] <= recv_pkg[39];
			recv_pkg[39] <= fifo_data_out;
			recv_cnt <= recv_cnt + 1'd1;
			recv_finish <= 0;
		end
		else begin
			recv_finish <= 1'd1;
		end
		if(analysis_finish==1)begin
			for(i=0;i<40;i=i+1)begin
				recv_pkg[i] <= 0;
			end
			recv_cnt <= 0;
			recv_finish <= 0;
		end
    end
end

//校验数据，拿到校验数据
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
		recv_check_flag <= 1'd0;
		recv_check_cnt <= 8'd0;
		recv_check_data <= 8'd0;
		recv_pkg_defcnt <= 8'd60;
    end                                                      
    else begin                     
		if(recv_pkg[PKG_CNT-recv_cnt] == 8'h3a && recv_check_flag == 0 && recv_finish == 1) begin //校验包头，读取FIFO数据的同时对数据进行校验
			recv_check_flag <= 1'd1;
			recv_check_data <= 8'd0;
			recv_pkg_defcnt <= (recv_pkg[PKG_CNT-recv_cnt+4]<<24&32'hff_00_00_00)|(recv_pkg[PKG_CNT-recv_cnt+3]<<16&32'h00_ff_00_00)|(recv_pkg[PKG_CNT-recv_cnt+2]<<8&32'h00_00_ff_00)|(recv_pkg[PKG_CNT-recv_cnt+1]&32'h00_00_00_ff);
		end

		if(recv_check_flag == 1 && recv_check_cnt < recv_pkg_defcnt)begin
			recv_check_data <= 8'd0;//累加
			recv_check_cnt <= recv_check_cnt + 1'd1; 
		end
		else if(recv_check_cnt == recv_pkg_defcnt)begin
			recv_check_data <= 8'd1;//取反加1
			recv_check_flag <= 1'd0;
			recv_check_cnt <= 8'd0;
		end
		else begin
			recv_check_data <= recv_check_data;
		end
		if(analysis_finish)begin
			recv_check_flag <= 1'd0;
			recv_check_cnt <= 8'd0;
			recv_check_data <= 8'd0;
			recv_pkg_defcnt <= 8'd60;
		end
    end
end

//解析数据包
always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
		analysis_flag <= 0;
		analysis_finish <= 0;
		sampling_channel_temp <= 8;
		laser_channel_temp <= 8;
		recv_cmd <= 16'h1111;
    end                                                      
    else begin
		if(recv_pkg[PKG_CNT-recv_pkg_defcnt-1-6] == 8'h3a && recv_pkg[PKG_CNT-1] == 8'h0a && recv_check_data == 8'd1 && !analysis_flag)begin
			recv_cmd <= (recv_pkg[PKG_CNT-recv_pkg_defcnt-2]&16'h00ff)|(recv_pkg[PKG_CNT-recv_pkg_defcnt-1]<<8&16'hff00);//接收数据命令位
			analysis_finish <= 1;
			analysis_flag <= 1;
			if((recv_pkg[PKG_CNT-recv_pkg_defcnt-2]&16'h00ff)|(recv_pkg[PKG_CNT-recv_pkg_defcnt-1]<<8&16'hff00) === 16'h2 && recv_pkg_defcnt == 8)begin
				laser_channel_temp <= recv_pkg[PKG_CNT-recv_pkg_defcnt];
				sampling_channel_temp <= recv_pkg[PKG_CNT-recv_pkg_defcnt+1];
			end else begin
				laser_channel_temp <= laser_channel_temp;
				sampling_channel_temp <= sampling_channel_temp;
			end 
			if((recv_pkg[PKG_CNT-recv_pkg_defcnt-2]&16'h00ff)|(recv_pkg[PKG_CNT-recv_pkg_defcnt-1]<<8&16'hff00) === 16'h0105 && recv_pkg_defcnt == 3)begin
				o_trigger_out_data <= recv_pkg[PKG_CNT-recv_pkg_defcnt];
			end 
		end
		if(recv_check_flag)begin
			analysis_flag <= 0;    //重新开始计数就重置分析
			recv_cmd <= 16'h1111;
		end
		if(analysis_finish == 1)begin
			analysis_finish <= 0;
		end
    end
end

//配置设备参数
always @(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		state <= FREE_STATE;
		sampling_channel_data <= 8;//探测器通道数
		laser_channel_data <= 8;//光源通道数
		sampling_frequency_data <= 10;//光源切换频率
		channel_samp_frequency <= 50000;//ad采样率
	end
	else begin
		if(analysis_finish == 1)begin
			case(sampling_channel_temp)
			'd4 :	sampling_frequency_data <= 20;
			'd8 :	sampling_frequency_data	<= 10;
			'd16:	sampling_frequency_data <= 5 ;
			'd32:	sampling_frequency_data <= 2 ;
			endcase
			case(recv_cmd)
				`CMD_CHECK:begin   //查询命令
					state <= CHECK_STATE;
				end
				`CMD_CONFIG:begin //配置命令
					state <= CONFIG_STATE;
					case(laser_channel_temp)
					'd4	:	laser_channel_data	<=	4 ;
					'd8	:	laser_channel_data	<=	8 ;
					'd16:	laser_channel_data	<=	16;
					'd32:	laser_channel_data	<=	32;
					endcase
					case(sampling_channel_temp)
					'd4 :	sampling_channel_data	<=	4 ;
					'd8 :	sampling_channel_data	<=	8 ;
					'd16:	sampling_channel_data	<=	16;
					'd32:	sampling_channel_data	<=	32;
					endcase
				end
				`CMD_START:begin  //开始命令
					state <= START_STATE;
				end
				`CMD_STOP:begin   //停止命令
					state <= STOP_STATE;
				end
				`CMD_CALIBRATION:begin	//校准命令
					state <= CALIBRATION_STATE;
					sampling_frequency_data <= 1;
				end
				`CMD_STOP_CALI:begin  //停止校准命令
					state <= STOP_CALI_STATE;
				end
				`CMD_TRIGGER_OUT:begin
					state	<=	TRIGGER_OUT_FLAG;
				end
				default:begin
					state <= FREE_STATE;
				end
			endcase
		end
		if(state_flag) begin //返回标志为1后将状态置为空闲
			state <= FREE_STATE;
		end
	end
end

ft232hq_recv u_ft232hq_recv(
	.clock				(ft232_clk),
	.rst_n				(rst_n),
	
	.rxf_n				(ft232_rxf_n),
	.wr_n				(ft232_wr_n),
	.data_recv			(ft232_data_recv),
	.oe_n				(ft232_oe_n),
	.rd_n				(ft232_rd_n),
	
	.fifo_wrfull_n		(ft232_wrfull),
	.fifo_wr_en			(ft232_wrreq),
	.fifo_data_in		(ft232_data_in)
);

//例化ft232 fifo
ft232hq_recv_fifo u_ft232_fifo(
	.Data				(ft232_data_in), //input [7:0] Data
	.WrClk				(ft232_clk), //input WrClk
	.RdClk				(clk), //input RdClk
	.WrEn				(ft232_wrreq), //input WrEn
	.RdEn				(ft232_rdreq), //input RdEn
	.Q					(fifo_data_out), //output [7:0] Q
	.Empty				(ft232_rdempty), //output Empty
	.Full				(ft232_wrfull) //output Full
);

endmodule