//**************************************************//
//ying-chi
//author: liu-liu
//file name: FT232Hq_recv.v
//last modified date: 2022/03/2
//last version: V1.0
//description: FT232 usb 接收底层驱动
//--------------------------------------------------//
module ft232hq_recv(
	input				clock,
	input				rst_n,
	
	//ft接口
	input  wire			rxf_n,
	input  wire			wr_n,
	input  wire [7:0]	data_recv,
	
	output reg			oe_n,
	output wire			rd_n,
	
	//写入fifo接口
	input  wire			fifo_wrfull_n,
	output wire			fifo_wr_en,
	output wire	[7:0]	fifo_data_in
    );

	
reg			oe_n_d0;//oe_n下一拍	
reg			rd_n_d0;//oe_n下一拍		
reg			rxf_n_d0;//oe_n下一拍	

//*****************************************************
//**                    main code
//*****************************************************
//在oe_n为低且在oe_n下一拍也为低时拉低rd_n，其他时候为高
assign rd_n = ((oe_n_d0 == 0) && (oe_n == 0 ) && (wr_n == 1))?1'b0 : 1'b1;
//FT232H读状态，将USB数据总线上的值赋给FIFO数据输入，其他时候赋值为高阻态
assign fifo_data_in = ((rxf_n_d0 == 0) && (wr_n == 1))? data_recv:8'hzz;
//在usb_oe_n下一拍也为低，且usb_rxf_n也为低时使能FIFO写
assign fifo_wr_en = ((oe_n_d0 == 0) && (rxf_n == 0 ) && (fifo_wrfull_n == 0) && (wr_n == 1))?1'b1 : 1'b0;
	
 //产生FT232H数据输出使能oe_n
 always@(posedge clock or negedge rst_n)begin
     if(!rst_n)
         oe_n<=1;
     else if (!rxf_n)
         oe_n<=0;
     else if(rxf_n)
         oe_n<=1;
 end
 
 //FT232H数据输出使能oe_n打一拍
 always@(posedge clock or negedge rst_n)begin
     if(!rst_n)
         oe_n_d0 <= 1'b1;
     else 
         oe_n_d0 <=oe_n;
 end

//FT232H数据输出使能oe_n打一拍
 always@(posedge clock or negedge rst_n)begin
     if(!rst_n)
         rd_n_d0 <= 1'b1;
     else 
         rd_n_d0 <=rd_n;
 end 
 //FT232H数据输出使能oe_n打一拍
 always@(posedge clock or negedge rst_n)begin
     if(!rst_n)
         rxf_n_d0 <= 1'b1;
     else 
         rxf_n_d0 <=rxf_n;
 end 

endmodule