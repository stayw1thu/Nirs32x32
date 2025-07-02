module hc165_drive (
    input wire clk,           // 主时钟信号
    input wire rst_n,          // 异步复位信号（低有效）
    input wire i_start,         // 启动操作信号
    input wire i_shift_qh4,     // SN74HC165 串行数据输出
    output reg o_shift_clk,     // 串行移位时钟信号
    output reg o_shift_shld_n,  // 加载信号（低有效）
    output reg o_code_valid,    // 数据有效标志
    output reg [7:0] o_code     // 最终输出的 8 位数据
);

// 状态定义
reg [2:0] state;
localparam IDLE        = 3'b000;
localparam LOAD        = 3'b001;
localparam LOAD_PULSE  = 3'b010;
localparam LOAD_DELAY  = 3'b011;
localparam SHIFT_START = 3'b100;
localparam SHIFT       = 3'b101;
localparam SHIFT_DONE  = 3'b110;

// 移位逻辑寄存器
reg [7:0] code_temp;       // 用于接收移位输出的临时寄存器
reg [2:0] counter_shift;   // 移位计数器（计数到 8）

// 时钟分频器寄存器
reg [2:0] counter_divider;
reg [1:0] clk_temp;

// 时钟分频：生成 o_shift_clk
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		o_shift_clk <= 1'b0;
		counter_divider <= 3'b0;
	end else if (counter_divider >= 3'd4) begin
		o_shift_clk <= ~o_shift_clk;
		counter_divider <= 3'b0;
	end else begin
		counter_divider <= counter_divider + 3'b1;
	end
end

// 时钟边沿检测
assign rising_edgeof_shiftclk = (clk_temp == 2'b01);
assign falling_edgeof_shiftclk = (clk_temp == 2'b10);

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		clk_temp <= 2'b0;
	end else begin
		clk_temp <= {clk_temp[0], o_shift_clk};
	end
end

// 主状态机
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		// 复位状态
		o_shift_shld_n <= 1'b1;
		o_code_valid <= 1'b0;
		o_code <= 8'hff;

		counter_shift <= 3'b0;
		state <= IDLE;
	end else begin
		case (state)
			IDLE: begin
				o_shift_shld_n <= 1'b1;
				o_code_valid <= 1'b0;
//                    o_code <= 8'hff;

				counter_shift <= 3'b0;

				if (i_start) begin
					state <= LOAD;
				end else begin
					state <= IDLE;
				end
			end
			LOAD: begin
				if (rising_edgeof_shiftclk) begin
					o_code <= 8'hff;
					o_shift_shld_n <= 1'b0;
					state <= SHIFT;
				end else begin
					state <= LOAD;
				end
			end
//                LOAD_PULSE: begin
//					o_code <= 8'hff;
//                    if (rising_edgeof_shiftclk) begin
//                        o_shift_shld_n <= 1'b1;
//                        state <= SHIFT;
//                    end else begin
//                        state <= LOAD_PULSE;
//                    end
//                end
			SHIFT: begin
				if (rising_edgeof_shiftclk) begin
					o_shift_shld_n <= 1'b1;
					counter_shift <= counter_shift + 1'b1;
					if (counter_shift >= 7) begin
						o_shift_shld_n <= 1'b1;
						state <= SHIFT_DONE;
					end
				end else begin
					state <= SHIFT;
				end
			end
			SHIFT_DONE: begin
				o_shift_shld_n <= 1'b1;
				counter_shift <= 3'b0;

				o_code_valid <= 1'b1;
				o_code <= code_temp[7:0]; // 输出最终 8 位数据
				state <= IDLE;
			end
			default: begin
				o_shift_shld_n <= 1'b1;
				o_code_valid <= 1'b0;
				o_code <= 8'hff;

				counter_shift <= 3'b0;
				state <= IDLE;
			end
		endcase
	end
end

// 串行移位数据接收
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		code_temp <= 8'hff;
	end else begin
		if(state==SHIFT)begin
			if (rising_edgeof_shiftclk && counter_shift <= 7) begin
				code_temp <= {code_temp[6:0], i_shift_qh4};
			end
		end
		else if(state==IDLE)begin
			code_temp <= 8'hff;
		end
	end
end

endmodule