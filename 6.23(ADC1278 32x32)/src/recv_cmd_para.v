//**************************************************//
//ying-chi
//author: liu-liu
//file name: recv_cmd_para.v
//last modified date: 2022/03/2
//last version: V1.0
//description: 上下位机的命令参数
//--------------------------------------------------//

`define		PKG_CNT_ADC 	 8'd133	                       //数据包

// 上位机命令
`define		CMD_PKGHEAD	    	8 'h3a		               //命令包头
`define		CMD_CHECK 	    	16'h0000		           //查询命令
`define		CMD_CONFIG 	    	16'h0002	               //配置命令
`define		CMD_START 	    	16'h0001	               //开始命令
`define		CMD_STOP 	    	16'h0101	               //停止命令
`define		CMD_CALIBRATION 	16'h0201				   //校准命令
`define		CMD_STOP_CALI	 	16'h0301				   //停止校准命令
`define		CMD_SUPER 	    	16'h0401	               //超采样
`define		CMD_TRIGGER_OUT		16'h0501	               //触发输出

