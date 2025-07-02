module ads1278_drive_2#(
    parameter               		P_ADS1278_MODE = 2'b01,             // 0-快速 1-精准 2-低功耗 3-低速
    parameter               		P_ADS1278_PWDN = 8'b1111_1111       // 通道开关 0-关 1-开
)(
    input                   		i_sysclk,                           // 时钟
    input                   		i_rst_n,                            // 复位信号
    input                   		i_start,                            // 启动信号

    output                  		o_ads1278_clk,                      // 芯片时钟
    output                  		o_ads1278_clk_div,                  // 芯片时钟分频设置
    output 	reg              		o_ads1278_fsync,                    // 芯片帧同步（ADC转化设置）
    output                  		o_ads1278_sclk,                     // SPI时钟
    input       		[7:0]       i_ads1278_data,                     // 8通道数据输入
    output      		[2:0]       o_ads1278_format,                   // 数据格式设置
    output      		[1:0]       o_ads1278_mode,                     // 工作模式设置
    output      		[7:0]       o_ads1278_pwdn,                     // 通道开关设置
    output                  		o_ads1278_sync,                     // 多片同步信号（低有效）
    output      		[1:0]       o_ads1278_test,                     // 测试模式设置

	input							i_user_clk,
	output				[191:0]		o_user_rdata,
	output							o_user_rdata_valid,
	input							i_user_rdata_ready
);

/********************参数*********************/
localparam      		P_ST_IDLE           = 'd0,
						P_ST_CNV            = 'd1,
						P_ST_READ_DATA      = 'd2,
						P_ST_END            = 'd3;

/********************状态机*********************/
reg         [7:0]       r_st_current;
reg         [7:0]       r_st_next;

/********************寄存器*********************/
reg         [9:0]       r_st_start_cnt;
reg         [4:0]       r_st_read_cnt;
reg         [1:0]       r_sclk_cnt;
reg         [23:0]      r_ads1278_data1;					
reg         [23:0]      r_ads1278_data2;
reg         [23:0]      r_ads1278_data3;
reg         [23:0]      r_ads1278_data4;
reg         [23:0]      r_ads1278_data5;
reg         [23:0]      r_ads1278_data6;
reg         [23:0]      r_ads1278_data7;
reg         [23:0]      r_ads1278_data8;

/********************网线型*********************/
wire                    w_ads1278_clk;
wire                    w_ads1278_rst_n;
wire                    w_ads1278_rst;
wire                    w_read_cnt_rst;
wire		[191:0]		w_fifo_rdata;
wire					w_fifo_rdata_wren;
wire					w_fifo_rdata_rden;
wire					w_fifo_rdata_rdempty;

/********************组合逻辑*********************/
assign o_ads1278_test    = 		2'b00;                   			// 正常模式
assign o_ads1278_pwdn    = 		P_ADS1278_PWDN;          			// 通道开关
assign o_ads1278_mode    = 		P_ADS1278_MODE;          			// 工作模式
assign o_ads1278_format  = 		3'b101;                  			// 帧同步独立输出
assign o_ads1278_sync    = 		1'b1;                    			// 多片同步信号（低有效）
assign o_ads1278_clk     = 		w_ads1278_clk;           			// 芯片时钟
assign o_ads1278_clk_div = 		1'b1;                    			// 芯片时钟分频设置
assign w_ads1278_rst     = 		~w_ads1278_rst_n;        			// 复位信号
assign o_ads1278_sclk    = 		(r_st_current == P_ST_READ_DATA) ?
								r_sclk_cnt[1] : 1'b1;   			// SPI时钟
assign w_read_cnt_rst    = 		(r_st_current == P_ST_IDLE) ? 
								1'b1 : 1'b0;            			// 读计数器复位信号
assign w_fifo_rdata		 =		{r_ads1278_data8,r_ads1278_data7,r_ads1278_data6,r_ads1278_data5,r_ads1278_data4,r_ads1278_data3,r_ads1278_data2,r_ads1278_data1};
assign w_fifo_rdata_wren =		(r_st_current == P_ST_END) ?
								1'b1 : 1'b0;
assign o_user_rdata_valid	=	~w_fifo_rdata_rdempty;
assign w_fifo_rdata_rden =		o_user_rdata_valid & i_user_rdata_ready;

/********************例化*********************/
ads1278_clk ads1278_clk_u1(                         // PLL
    .clkin    (i_sysclk),                         // 50MHZ时钟
    .reset    (~i_rst_n),                          // 复位信号
    .clkout   (w_ads1278_clk),                    // ADS1278时钟25MHZ
    .lock     (w_ads1278_rst_n)                   // ADS1278复位信号
);

ad1278_fifo ad1278_fifo_u1(

	.Data		(w_fifo_rdata), //input [191:0] Data
	.WrClk		(o_ads1278_clk), //input WrClk
	.WrEn		(w_fifo_rdata_wren), //input WrEn
	.Full		(), //output Full

	.RdClk		(i_user_clk), //input RdClk
	.RdEn		(w_fifo_rdata_rden), //input RdEn
	.Q			(o_user_rdata), //output [191:0] Q
	.Empty		(w_fifo_rdata_rdempty) //output Empty

);

/********************进程*********************/

//第一段状态机
always @(posedge w_ads1278_clk) begin
    if(w_ads1278_rst) begin
        r_st_current <= P_ST_IDLE;
    end else begin
		if(i_start)begin
			r_st_current <= r_st_next;
		end else begin
			r_st_current <= P_ST_IDLE;
		end
	end
end

//第二段状态机
always @(*)
	case(r_st_current)
		P_ST_IDLE       : r_st_next = (r_st_start_cnt   == 'd499) ? P_ST_CNV        : P_ST_IDLE;        // 精准模式下512个时钟周期读取一个数据（SPS = 52734)
		P_ST_CNV        : r_st_next = (r_sclk_cnt       == 'd3	) ? P_ST_READ_DATA  : P_ST_CNV;         // 4分频
		P_ST_READ_DATA  : r_st_next = (r_st_read_cnt    == 'd24 ) ? P_ST_END        : P_ST_READ_DATA;
		P_ST_END        : r_st_next = P_ST_IDLE;
		default         : r_st_next = P_ST_IDLE;
	endcase

//每次读取数据间隔计数器
always @(posedge w_ads1278_clk) begin
    if(w_ads1278_rst) begin
        r_st_start_cnt <= 'd0;
    end else begin
		if(i_start)begin
			if(r_st_start_cnt < 'd499) begin
				r_st_start_cnt <= r_st_start_cnt + 1'b1;
			end else begin
				r_st_start_cnt <= 'd0;
			end
		end else begin
			r_st_start_cnt <= 0;
		end
    end
end

//转化信号控制
always @(posedge w_ads1278_clk) begin
    if(r_st_current == P_ST_CNV) begin
        o_ads1278_fsync <= 'd1;												// 开始转换
    end else begin
        o_ads1278_fsync <= 'd0;
    end
end    

//SCLK时钟计数器
always @(posedge w_ads1278_clk) begin
    if(r_st_current == P_ST_CNV || r_st_current == P_ST_READ_DATA) begin
        r_sclk_cnt <= r_sclk_cnt + 1'b1;
    end else begin
        r_sclk_cnt <= 'd0;
    end
end

//读数据计数器
always @(posedge o_ads1278_sclk or posedge w_read_cnt_rst) begin
    if(w_read_cnt_rst) begin
        r_st_read_cnt <= 'd0;
    end else begin
        r_st_read_cnt <= r_st_read_cnt + 1'b1;
    end
end

//读数据
always @(posedge o_ads1278_sclk) begin
    if(w_read_cnt_rst) begin
        r_ads1278_data1 <= 'd0;
        r_ads1278_data2 <= 'd0;
        r_ads1278_data3 <= 'd0;
        r_ads1278_data4 <= 'd0;
        r_ads1278_data5 <= 'd0;
        r_ads1278_data6 <= 'd0;
        r_ads1278_data7 <= 'd0;
        r_ads1278_data8 <= 'd0;
    end else if(r_st_read_cnt >= 'd0 && r_st_read_cnt <= 'd23)begin
        r_ads1278_data1 <= {r_ads1278_data1[22:0], i_ads1278_data[0]};
        r_ads1278_data2 <= {r_ads1278_data2[22:0], i_ads1278_data[1]};
        r_ads1278_data3 <= {r_ads1278_data3[22:0], i_ads1278_data[2]};
        r_ads1278_data4 <= {r_ads1278_data4[22:0], i_ads1278_data[3]};
        r_ads1278_data5 <= {r_ads1278_data5[22:0], i_ads1278_data[4]};
        r_ads1278_data6 <= {r_ads1278_data6[22:0], i_ads1278_data[5]};
        r_ads1278_data7 <= {r_ads1278_data7[22:0], i_ads1278_data[6]};
        r_ads1278_data8 <= {r_ads1278_data8[22:0], i_ads1278_data[7]};
    end else begin
        r_ads1278_data1 <= 'd0;
        r_ads1278_data2 <= 'd0;
        r_ads1278_data3 <= 'd0;
        r_ads1278_data4 <= 'd0;
        r_ads1278_data5 <= 'd0;
        r_ads1278_data6 <= 'd0;
        r_ads1278_data7 <= 'd0;
        r_ads1278_data8 <= 'd0;
    end
end

endmodule