module ad9833(
	input 						clk,
	input						rst_n,
	
	input 				[15:0]	ad9833_data,
	input	wire				ad9833_wr_en,
	
	output 	reg					ad9833_sclk,
	output 	reg					ad9833_fsync,
	output 	reg 				ad9833_sdata,
	output  reg					ad9833_wr_finish
);

reg [15:0]	wr_data;			//待写入的数据
reg			wr_data_flag;		//开始写入数据标志
reg [5:0]	wr_state;			//写入数据的状态
reg [3:0]	cnt;
/**************************************
			main code
**************************************/ 

//写入数据
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		wr_state <= 6'd0;
		ad9833_fsync <= 1'd1;
		ad9833_sclk <= 1'd1;
		ad9833_sdata <= 1'd0;
		ad9833_wr_finish <= 1'b0;
		wr_data <= 16'd0;
	end
	else begin
		if(ad9833_wr_en)begin
			wr_data <= ad9833_data;
			ad9833_fsync <= 1'd0;
			case(wr_state)
				6'd1, 6'd3, 6'd5, 6'd7, 6'd9, 6'd11, 6'd13, 6'd15, 6'd17, 6'd19,
				6'd21, 6'd23, 6'd25, 6'd27, 6'd29:begin
					ad9833_sclk <= 1'd0;
					wr_state		<= wr_state + 1'd1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd0:begin
					ad9833_sdata <= wr_data[15];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd2:begin
					ad9833_sdata <= wr_data[14];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd4:begin
					ad9833_sdata <= wr_data[13];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd6:begin
					ad9833_sdata <= wr_data[12];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd8:begin
					ad9833_sdata <= wr_data[11];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd10:begin
					ad9833_sdata <= wr_data[10];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd12:begin
					ad9833_sdata <= wr_data[9];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd14:begin
					ad9833_sdata <= wr_data[8];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd16:begin
					ad9833_sdata <= wr_data[7];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd18:begin
					ad9833_sdata <= wr_data[6];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd20:begin
					ad9833_sdata <= wr_data[5];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd22:begin
					ad9833_sdata <= wr_data[4];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd24:begin
					ad9833_sdata <= wr_data[3];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd26:begin
					ad9833_sdata <= wr_data[2];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd28:begin
					ad9833_sdata <= wr_data[1];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b0;
				end
				6'd30:begin
					ad9833_sdata <= wr_data[0];
					ad9833_sclk <= 1'd1;
					wr_state <= wr_state + 1'b1;
					ad9833_wr_finish <= 1'b1;
				end
				6'd31:begin
					ad9833_sclk <= 1'd0;
					ad9833_wr_finish <= 1'b0;
					wr_state <= wr_state + 1'b1;
				end
				6'd32:begin
					ad9833_sclk <= 1'd1;
					ad9833_fsync <= 1'd1;
				end
				default:begin
					ad9833_sclk <= 1'd1;
					ad9833_fsync <= 1'd1;
				end
			endcase
		end
		else begin
			wr_state <= 6'd0;
			ad9833_fsync <= 1'd1;
			ad9833_sclk <= 1'd1;
			ad9833_sdata <= 1'd0;
			ad9833_wr_finish <= 1'b0;
		end
	end			
end


endmodule