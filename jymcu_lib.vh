module zrb_sync_fifo
/*
zrb_sync_fifo #(3,8) instance_name (
    RESET,
    WR_CLK,
    WR_EN,
    WR_DATA[7:0],
    RD_EN,
    RD_DATA[7:0],
    FIFO_FULL,
    FIFO_EMPTY
    );
*/
    #(parameter ADDR_WIDTH = 3, DATA_WIDTH = 8)
    (
    input   wire                            reset,

    input   wire                            clk,
    input   wire                            wr_en,
    input   wire    [ (DATA_WIDTH-1) :  0 ] data_in,

    input   wire                            rd_en,
    output  wire    [ (DATA_WIDTH-1) :  0 ] data_out,

    output  wire                            fifo_full,
    output  wire                            fifo_empty
    );
localparam DEPTH = 1 << ADDR_WIDTH;
reg     [ (ADDR_WIDTH) :  0 ] wr_ptr = {(ADDR_WIDTH+1){1'b0}};
reg     [ (ADDR_WIDTH) :  0 ] rd_ptr = {(ADDR_WIDTH+1){1'b0}};
wire    [ (ADDR_WIDTH-1) :  0 ] wr_loc = wr_ptr[ (ADDR_WIDTH-1) :  0 ];
wire    [ (ADDR_WIDTH-1) :  0 ] rd_loc = rd_ptr[ (ADDR_WIDTH-1) :  0 ];

reg     [ (DATA_WIDTH-1) :  0 ] mem [ (DEPTH-1) :  0 ];

reg full = 1'b0;
reg empty = 1'b0;
assign data_out = mem[rd_loc];
assign fifo_empty = empty;
assign fifo_full = full;

always@(wr_ptr or rd_ptr)
begin
    full <= 1'b0;
    empty <= 1'b0;
    if(wr_ptr[ (ADDR_WIDTH-1) :  0 ] == rd_ptr[ (ADDR_WIDTH-1) :  0 ])
        if(rd_ptr[ADDR_WIDTH] == wr_ptr[ADDR_WIDTH])
            empty <= 1'b1;
        else
            full <= 1'b1;
end

always@(posedge clk or posedge reset)
if(reset)
begin
    wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
    rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
end
else
begin
    if(wr_en & !fifo_full)
    begin
        mem[wr_loc] <= data_in;
        wr_ptr <= wr_ptr + 1'b1;
    end
    
    if(rd_en & !fifo_empty)
        rd_ptr <= rd_ptr + 1'b1;
end
endmodule


module zrb_baud_generator
/*
zrb_baud_generator #(50000000,9600) instance_name(input_clk, enable, baud_clk, baud_clk_8);
*/
    #(parameter INPUT_CLK = 50000000, parameter BAUD = 9600)
    (
    input   wire                clk,
    input   wire                en,
    output  wire                baud_clk_tx_en,
    output  wire                baud_clk_rx_en
    );
/*http://www.excamera.com/sphinx/fpga-uart.html#uart*/
localparam BAUD_TX = BAUD;
localparam BAUD_RX = 8*BAUD;
reg     [ 28 :  0 ] r_tx = 29'b0;
reg     [ 28 :  0 ] r_rx = 29'b0;
wire    [ 28 :  0 ] inc_tx = r_tx[28] ? (BAUD_TX) : (BAUD_TX-INPUT_CLK);
wire    [ 28 :  0 ] inc_rx = r_rx[28] ? (BAUD_RX) : (BAUD_RX-INPUT_CLK);
wire    [ 28 :  0 ] tx_tic = r_tx + inc_tx;
wire    [ 28 :  0 ] rx_tic = r_rx + inc_rx;
always@(posedge clk)
if(en)
begin
    r_tx <= tx_tic;
    r_rx <= rx_tic;
end
assign baud_clk_tx_en = ~r_tx[28];
assign baud_clk_rx_en = ~r_rx[28];
endmodule


module zrb_uart_tx
/*
zrb_uart_tx #(8,"NO",1) instance_name(
    CLK,
    CLK_EN, //BAUD RATE
    RESET,
    NEW_DATA,
    INPUT_DATA[7:0],
    OUTPUT_TX,
    OUTPUT_BUSY,
	READ
    );
*/
    #(parameter NUM_BITS = 4'd8, parameter PARITY = "NO", parameter STOP_BIT = 4'd1)
    (
    input   wire                clk,
    input   wire                clk_en,
    input   wire                reset,
    input   wire                new_data,
    input   wire    [  7 :  0 ] data,

    output  wire                tx,
    output  wire                busy,
	output	wire				read
    );
localparam START_BIT = 4'd1;
localparam WIDTH =  PARITY == "NO"   ?  NUM_BITS + START_BIT + STOP_BIT;
                    PARITY == "EVEN" ?  NUM_BITS + START_BIT + STOP_BIT + 1 :
                    PARITY == "ODD"  ?  NUM_BITS + START_BIT + STOP_BIT + 1 : 1;

reg     [  8 :  0 ] r_data = 9'b0;
reg     [  3 :  0 ] r_cnt = 4'b0;
reg                 r_tx = 1'b1;
reg					r_rd = 1'b0;

wire                sending = |r_cnt;
assign              busy = sending;
assign              tx = r_tx;
assign				read = r_rd;

always@(posedge clk)
if(reset)
begin
    r_tx <= 1'b1;
    r_cnt <= 4'b0;
    r_data <= 9'b0;
	r_rd <= 1'b0;
end
else
begin
	r_rd <= 1'b0;
    if(new_data & ~busy)
    begin
		r_rd <= 1'b1;
        r_data <= {data, 1'b0};
        r_cnt <= WIDTH;
    end
    
    if(sending & clk_en)
    begin
        {r_data, r_tx} <= {1'b1, r_data};
        r_cnt <= r_cnt - 1'b1;
    end
end
endmodule


module zrb_uart_rx
/*
zrb_uart_rx #(8,"NO",1) instance_name(
    INPUT_CLK, //HIGH FREQ
    CLK_EN,
    RESET
    INPUT_RX,
    OUTPUT_DATA[7:0],
    WRITE_EN,
    OUTPUT_BUSY,
    );
*/
    #(parameter [3:0] NUM_BITS = 8, parameter PARITY = "NO", parameter [3:0]STOP_BIT = 1)
    (
    input   wire                clk,
    input   wire                clk_en,
    input   wire                reset,
    
    input   wire                rx,

    output  wire    [  7 :  0 ] data_out,
    output  wire                write_en,
    output  wire                busy
    );

localparam [3:0] START_BIT = 1;
localparam [3:0] WIDTH = PARITY == "NO"   ?     NUM_BITS + START_BIT + STOP_BIT : 
                         PARITY == "EVEN" ?     NUM_BITS + START_BIT + STOP_BIT + 1 :
                         PARITY == "ODD"  ?     NUM_BITS + START_BIT + STOP_BIT + 1 : 1;
reg                 start_sync = 1'b0;
reg                 start_en = 1'b0;
wire                start = ~start_sync & start_en;
reg     [  9 :  0 ] r_data = 10'b0;
reg     [  3 :  0 ] r_cnt = 4'b0;
reg     [  2 :  0 ] clk_en_cnt = 3'b0;
wire                receiving = |r_cnt;
assign              busy = receiving;
assign              write_en = clk_en & r_cnt == 1 & clk_en_cnt == 3'd3;
assign              data_out = r_data[(WIDTH-2)-:8];

always@(posedge clk)
if(reset)
begin
    start_sync <= 1'b0;
    start_en <= 1'b0;
    r_data <= 10'b0;
    r_cnt <= 4'b0;
    clk_en_cnt <= 3'b0;
end
else
begin
    start_sync <= rx;
    start_en <= start_sync;
    
    if(start & ~receiving)
    begin
        r_cnt <= WIDTH;
        clk_en_cnt <= 3'b0;
    end
    if(receiving & clk_en)
    begin   
        clk_en_cnt <= clk_en_cnt + 1'b1;
        if(clk_en_cnt == 3'd3)
        begin
            //r_data <= {r_data[(WIDTH-2):0], rx};
            r_data <= {rx, r_data[(WIDTH-2):1]};
            r_cnt <= r_cnt - 1'b1;
        end
    end
end
endmodule

module zrb_bt_controller
/*
zrb_bt_controller instance_name(
	CLK_IN,
	CLK_EN
	DATA_IN[7:0],
	NEW_DATA,
	TX_FULL,
	SPEED_SELECT,
	WR,
	RD,
	DATA_OUT[7:0]
	);
*/
	(
	input	wire				clk,
	input	wire				clk_en,
	input	wire	[  7 :  0 ] data_in,
	input	wire				new_data,
	input	wire				tx_full,
	output	wire				speed_select,
	output	wire				wr,
	output	wire				rd,
	output	wire	[  7 :  0 ] data_out
	);



reg				flg_9600 = 1'b0;
reg				flg_115200 = 1'b1;
reg				r_rd = 1'b0;
reg				r_wr = 1'b0;


reg	[  2 :  0 ] at_cnt = 3'b0;
reg [  7 :  0 ] AT [  0 :  7 ];
initial begin
	AT[0] = 8'h41; //A
	AT[1] = 8'h54; //T
	AT[2] = 8'h2B; //+
	AT[3] = 8'h42; //B
	AT[4] = 8'h41; //A
	AT[5] = 8'h55; //U
	AT[6] = 8'h44; //D
	AT[7] = 8'h38; //8 - speed 115200
end

reg [  2 :  0 ] resp_cnt = 3'b0;
reg [  7 :  0 ] RESP [  0 :  7 ];
initial begin
	RESP[0] = 8'h3F; //?
	RESP[1] = 8'h3F; //?
	RESP[2] = 8'h3F; //?
	RESP[3] = 8'h3F; //?
	RESP[4] = 8'h3F; //?
	RESP[5] = 8'h3F; //?
	RESP[6] = 8'h3F; //?
	RESP[7] = 8'h3F; //?
end


localparam [  2 :  0 ]  TEST_AT = 		3'b000,
						RESPONSE = 		3'b001,
						READ =			3'b101,
						
						CONFIG115200 = 	3'b100,
						IDLE =			3'b011,
						
						SHOW = 			3'b010,
						ERROR =			3'b110;
reg	[  2 :  0 ] r_state = TEST_AT;

reg	[  7 :  0 ] r_di = 8'b0;
reg [  7 :  0 ] r_do = 8'b0;
reg [ 18 :  0 ] time_out = 19'b0;

assign speed_select = 1;//flg_115200;	
assign rd = r_rd;
assign wr = r_wr;
assign data_out = r_do;

always@(posedge clk)
begin
	r_rd <= 1'b0;
	r_wr <= 1'b0;
	case(r_state)
		TEST_AT:
			if(at_cnt == 3'd1)
				r_state <= RESPONSE;
		RESPONSE:
			if(time_out[18])
				r_state <= ERROR;
		READ:
		r_state <= ERROR;
		/*begin
			if(flg_115200 & ~flg_9600)
				r_state <= IDLE;
			if(flg_9600 & ~flg_115200)
				r_state <= CONFIG115200;
		end
*/
		IDLE:
		begin
			if(new_data & data_in == 8'hDD)
				r_state <= TEST_AT;
			if(new_data & data_in == 8'hFF)
				r_state <= SHOW;
		end

		CONFIG115200:
			if(at_cnt == 3'd7)
				r_state <= RESPONSE;

		ERROR:
			r_state <= ERROR;
		default:
			r_state <= ERROR;
	endcase

	case(r_state)
		TEST_AT:
		begin
			resp_cnt <= 3'b0;
			flg_9600 <= 1'b0;
			flg_115200 <= 1'b1;
			time_out <= 24'b0;
			if(~tx_full)
			begin
				r_wr <= 1'b1;
				at_cnt <= at_cnt + 1'b1;
				r_do <= AT[at_cnt];
			end
		end

		RESPONSE:
		begin
			if(clk_en)
				time_out  <= time_out + 1'b1;
			if(new_data & ~r_rd)
			begin
				r_rd <= 1'b1;
				r_di <= data_in;
			end	

			if(r_rd)
			begin
				resp_cnt <= resp_cnt + 1'b1;
				RESP[resp_cnt] <= r_di;
			end
		end

		READ:
		begin
			time_out <= 16'b0;
			at_cnt <= 3'd0;
			if(resp_cnt != 3'd0)
			begin
				flg_9600 <= 1'b0;
				flg_115200 <= 1'b1;
			end
			else
			begin
				flg_9600 <= 1'b1;
				flg_115200 <= 1'b0;
			end
		end

		CONFIG115200:
		begin
			flg_9600 <= 1'b1;
			flg_115200 <= 1'b0;		
			time_out <= 19'b0;
			resp_cnt <= 3'b0;
			if(~tx_full)
			begin
				r_wr <= 1'b1;
				at_cnt <= at_cnt + 1'b1;
				r_do <= AT[at_cnt];
			end
		end
		
		SHOW:
		begin
			if(new_data & ~r_rd)
				r_rd <= 1'b1;

			if(r_rd)
			begin
				resp_cnt <= resp_cnt + 1'b1;
				r_wr <= 1'b1;
				r_do <= RESP[resp_cnt];
			end
		end
		
		ERROR:
		begin
			r_wr <= 1'b1;
			r_do <= {1'b0, at_cnt, 1'b0, resp_cnt};
		end
		
		IDLE:
		begin
			at_cnt <= 3'b0;
			resp_cnt <= 3'b0;
			if(new_data & ~r_rd & ~tx_full)
			begin
				r_rd <= 1'b1;
				r_di <= data_in;
			end
			if(r_rd)
			begin
				r_wr <= 1'b1;
				case(data_in)
					8'h11:
						r_do <= 8'hFF;
					default:
						r_do <= r_di + 1'b1;
				endcase
			end
		end
	endcase
end

endmodule
