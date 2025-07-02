module ft232hq_send(
	input				clock,
	input				rst_n,
	
	//ft接口
	input  wire			rd_n,
	input  wire			txe_n,
	
	output reg			wr_n,
	output reg [7:0]	data_send,
	output wire			ft232_siwu_n,
	
	//读fifo接口
	input  wire	[7:0]	fifo_data_out,
	input  wire	[12:0] fifo_rdusedw,
	input  wire			fifo_empty_n,
	output reg			fifo_rd_en
	
    );
	
reg last_txe_n;
reg fifo_rd_en_d0;
//*****************************************************
//**                    main code
//*****************************************************

assign ft232_siwu_n = 1'b1;

always@(posedge clock or negedge rst_n)begin
	if(!rst_n)begin
		wr_n <= 1'b1;
	end
	else begin
		wr_n <= (fifo_rd_en == 1 && fifo_rd_en_d0 == 1)?1'b0 : 1'b1;
	end
end

always@(posedge clock or negedge rst_n)begin	
	if(!rst_n)begin
		data_send <= 8'hzz;	
	end	
	else begin
		data_send <= (fifo_rd_en_d0 == 1 && fifo_rd_en == 1)? fifo_data_out: 8'hzz;
	end	
end	

always@(posedge clock or negedge rst_n)begin	
	if(!rst_n)begin	
		fifo_rd_en <= 0;
	end	
	else begin	
		fifo_rd_en <= ((last_txe_n == 0) && (txe_n == 0) && (fifo_empty_n == 0) && rd_n)?1'b1 : 1'b0;
	end	
end	

always@(posedge clock or negedge rst_n)begin
    if(!rst_n)begin
        last_txe_n <= 1'b1;
		fifo_rd_en_d0 <= 1'b1;
	end	  
    else begin
		last_txe_n <= txe_n;
		fifo_rd_en_d0 <= fifo_rd_en;
	end	  
end

endmodule

