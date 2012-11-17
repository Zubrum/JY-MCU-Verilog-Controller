`include "jymcu_lib.vh"

module jy_mcu
    (
    input           wire    clk,
    input           wire    BT_in,
    output          wire    BT_out
    );
	
wire	baud_9600;
wire	baud_9600x8;
wire	baud_115200;
wire	baud_115200x8;
wire 	speed_select;
wire    baud_clk = speed_select ? baud_115200 : baud_9600;
wire    baud_clkx8 = speed_select ? baud_115200x8 : baud_9600x8;

zrb_baud_generator #(50000000,9600)   u00(clk, !speed_select, baud_9600, baud_9600x8);
zrb_baud_generator #(50000000,115200) u01(clk,  speed_select, baud_115200, baud_115200x8);

wire [7:0] rx_data;
wire rx_write;
wire rx_busy;
wire rx_full;
wire rx_empty;
zrb_uart_rx #(8,"NO",1) u1(
    clk, //HIGH FREQ		//input clock
    baud_clkx8,				//clock_enable
    1'b0,					//reset
    BT_in,					//data_in
    rx_data,			//data_out[7:0]
	rx_write,					//write_enable_out
    rx_busy 				//rx_busy_out
    );
zrb_sync_fifo #(2,8) u10(
    1'b0,			//reset
    clk,			//write_clock
    rx_write,			//write_enable
    rx_data,		//data_in
    crd,		//read_enable
    cindata,		//data_out
    rx_full,			//fifo_full_out
    rx_empty			//fifo_empty_out
    );

wire [7:0]cindata;
wire [7:0]cdata;
wire cwr;
wire crd;
zrb_bt_controller instance_name(
	clk,
	baud_clk,
	cindata,
	~rx_empty,
	tx_full,
	speed_select,
	cwr,
	crd,
	cdata
	);

wire	[  7 :  0 ] tx_data;
wire				tx_write;
wire				tx_read;
wire				tx_busy;
wire				tx_full;
wire				tx_empty;

zrb_sync_fifo #(2,8) u20(
    1'b0,			//reset
    clk,			//clock
    cwr,//cntrl_wr,		//write_enable
    cdata,//cntrl_data_out,	//data_in
    tx_read,		//read_enable
    tx_data,		//data_out
    tx_full,			//fifo_full_out
    tx_empty			//fifo_empty_out
    );

zrb_uart_tx #(8,"NO",1) u2(
	clk,					//input clock
    baud_clk, //BAUD RATE	//clock_enable_8x
    1'b0,					//reset
    ~tx_empty,					//write_in
    tx_data,				//input_data[7:0]
    BT_out,					//data_out
    tx_busy,					//tx_busy_out
	tx_read
    );

endmodule
