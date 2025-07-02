module hc595pw_drive(
	input 						clk,
	input						rst_n,
	
	input 				[23:0]	hc595d_data,
	input				[7:0]	hc595d_data_len,
	input	wire				hc595d_wr_en,
	
	output 	reg 				hc595d_rck,
	output 	wire				hc595d_scl,
	output 	reg					hc595d_sck,
	output 	reg 				hc595d_ser,
	output  reg					hc595d_wr_finish
);

//
reg [23:0]	wr_data;				
reg [7:0]	wr_data_len;			
reg			wr_data_flag;			
reg [2:0]	wr_bitdata_cnt;			
reg [7:0]	wr_bytedata_cnt;		
reg [3:0]	wr_state;				

wire       en_flag;
reg        en_d0; 
reg        en_d1; 

/**************************************
			main code
**************************************/ 
assign hc595d_scl = 1;
assign en_flag = (~en_d1) & en_d0;

always @(posedge clk or negedge rst_n) begin         
    if (!rst_n) begin
        en_d0 <= 1'b0;                                  
        en_d1 <= 1'b0;
    end                                                      
    else begin                                               
        en_d0 <= hc595d_wr_en;                               
        en_d1 <= en_d0;                            
    end
end

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		wr_data <= 23'd0;
		wr_data_len <= 7'd0;
		wr_data_flag <= 0;
		hc595d_wr_finish <= 1'd0;
	end
	else begin
		if(en_flag) begin
			wr_data <= hc595d_data;
			wr_data_len <= hc595d_data_len ;
			wr_data_flag <= 1;
			hc595d_wr_finish <= 1'd0;
		end
		else if(wr_bytedata_cnt == wr_data_len) begin
			wr_data_flag <= 0;
			hc595d_wr_finish <= 1'd1;
		end
		else begin
			wr_data_flag <= wr_data_flag;
		end
	end			
end

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		wr_bytedata_cnt <= 7'd0;
		wr_state <= 0;
		hc595d_rck <= 0;
		hc595d_sck <= 1'd0;
		hc595d_ser <= 0;
	end
	else begin
		if(wr_data_flag)begin
			case(wr_state)
				4'd0:begin
					hc595d_ser <= wr_data[wr_data_len - 1 - wr_bytedata_cnt];
					wr_state <= 4'd1;
				end
				4'd1:begin
					hc595d_sck <= 1'd0;
					wr_state <= 4'd2;
				end
				4'd2:begin
					hc595d_sck <= 1'd1;
					wr_bytedata_cnt <= wr_bytedata_cnt + 1'd1;
					wr_state <= 4'd3;
				end
				4'd3:begin
					wr_state <= 4'd0;
					if(wr_bytedata_cnt == wr_data_len)begin
						wr_bytedata_cnt = 7'd0;
						hc595d_rck <= 1'd0;
					end
					else begin
						
					end
				end
				default:begin
					
				end
			endcase
		end
		else begin
			wr_bytedata_cnt <= 4'd0;
			wr_state <= 0;
			hc595d_rck <= 1;
			hc595d_sck <= 1'd1;
			hc595d_ser <= hc595d_ser;
		end
	end			
end



endmodule