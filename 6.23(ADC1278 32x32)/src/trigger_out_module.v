module trigger_out_module(
	input						i_clk				,
	input						i_rst_n				,
	input						i_trigger_out_en	,
	input			[7:0]		i_trigger_out_data	,

	output						o_tri_out_rck		,
	output						o_tri_out_scl		,
	output						o_tri_out_sck		,
	output						o_tri_out_ser		
);
/********************参数*********************/
localparam      		P_ST_IDLE           = 'd0,
						P_ST_START          = 'd1,
						P_ST_END		    = 'd2;

/********************状态机*********************/
reg         [3:0]       r_st_current;
reg         [3:0]       r_st_next;

/********************网线型*********************/
wire		[3:0]		w_tri_out_data_len;
wire					w_tri_out_en;

/*******************组合逻辑********************/
assign		w_tri_out_data_len 	= 8;
assign		w_tri_out_en		= (r_st_current == P_ST_START) ? 1'b1 : 1'b0;

/*********************例化*********************/
hc595pw_drive	u_tri_out(
	.clk							(i_clk),
	.rst_n							(i_rst_n),
	
	.hc595d_data					(i_trigger_out_data),
	.hc595d_data_len				(w_tri_out_data_len),
	.hc595d_wr_en					(w_tri_out_en),
	
	.hc595d_rck						(o_tri_out_rck),
	.hc595d_scl						(o_tri_out_scl),
	.hc595d_sck						(o_tri_out_sck),
	.hc595d_ser						(o_tri_out_ser),
	.hc595d_wr_finish				(o_tri_out_finish)
);

/********************进程*********************/

//第一段状态机
always @(posedge i_clk) begin
    if(!i_rst_n) begin
        r_st_current <= P_ST_IDLE;
    end else begin
        r_st_current <= r_st_next;
    end
end

//第二段状态机
always @(*)
	case(r_st_current)
		P_ST_IDLE       : r_st_next = (i_trigger_out_en == 1'd1	) ? P_ST_START      : P_ST_IDLE	;        // 等待触发
		P_ST_START      : r_st_next = (o_tri_out_finish == 1'd1	) ? P_ST_END	 	: P_ST_START;        // 触发输出
		P_ST_END        : r_st_next = P_ST_IDLE;
		default         : r_st_next = P_ST_IDLE;
	endcase

endmodule