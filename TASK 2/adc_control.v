// SM : Task 2 A : ADC
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.
This file is used to design ADC Controller.

Recommended Quartus Version : 19.1
The submitted project file must be 19.1 compatible as the evaluation will be done on Quartus Prime Lite 19.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

//ADC Controller design
//Inputs  : clk_50 : 50 MHz clock, dout : digital output from ADC128S022 (serial 12-bit)
//Output  : adc_cs_n : Chip Select, din : Ch. address input to ADC128S022, adc_sck : 2.5 MHz ADC clock,
//				d_out_ch5, d_out_ch6, d_out_ch7 : 12-bit output of ch. 5,6 & 7,
//				data_frame : To represent 16-cycle frame (optional)

//////////////////DO NOT MAKE ANY CHANGES IN MODULE//////////////////
module adc_control(
	input  clk_50,				//50 MHz clock
	input  dout,				//digital output from ADC128S022 (serial 12-bit)
	output adc_cs_n,			//ADC128S022 Chip Select
	output din,					//Ch. address input to ADC128S022 (serial)
	output adc_sck,			//2.5 MHz ADC clock
	output [11:0]d_out_ch5,	//12-bit output of ch. 5 (parallel)
	output [11:0]d_out_ch6,	//12-bit output of ch. 6 (parallel)
	output [11:0]d_out_ch7,	//12-bit output of ch. 7 (parallel)
	output [1:0]data_frame	//To represent 16-cycle frame (optional)
);
	
////////////////////////WRITE YOUR CODE FROM HERE////////////////////

// decalring registers for neccessary operations
reg[11:0] dout_chx;
reg[11:0] dout_ch5 = 0;
reg[11:0] dout_ch6 = 0;
reg[11:0] dout_ch7 = 0;
reg[4:0] data_counter = 0;
reg[3:0] s_clk_counter = 0;
reg[3:0] din_counter = 0;
reg[3:0] sp_counter = 0;
reg[2:0] channel_select = 5;
reg[2:0] channel = 0;
reg[1:0] data_viz = 1;

reg sclk_out = 0;
reg din_temp = 0;

// Frequency Scaling Block
// 50Mhz to 2.5Mhz 
always @(negedge clk_50) begin  
    if (s_clk_counter == 10) begin
        if(sclk_out == 0) sclk_out = 1;
        else sclk_out = 0;
        s_clk_counter = 0;
    end
    s_clk_counter = s_clk_counter + 1;
end

// Selecting data Channels Block
// Channels need to be selected 5, 6, 7
always @(negedge adc_sck) begin
    if (channel_select > 7) channel_select = 5;
    else if((din_counter >= 2) && (din_counter <= 4)) channel = channel_select;
    else if(din_counter == 15) begin
            channel_select = channel_select + 1;
        end
    case(din_counter)
        2: din_temp = channel[2]; 
        3: din_temp = channel[1];
        4: din_temp = channel[0];
        default: din_temp = 0;
    endcase
    din_counter = din_counter + 1;
end

// Serial Read Parallel Out Block
always @(posedge adc_sck) begin
    if(sp_counter >= 4 && sp_counter <=15) begin
        dout_chx = {dout_chx[10:0], dout};
    end
    sp_counter = sp_counter + 1;
end

// Data Visualization and d_out_chx mapping Block
always @(negedge adc_sck) begin

    if(data_counter == 16) begin
        data_counter = 0;
        data_viz = data_viz + 1;
        if(data_viz == 0) data_viz = 1;
        case(channel)
            5: dout_ch5 = dout_chx;
            6: dout_ch6 = dout_chx;
            7: dout_ch7 = dout_chx;
        endcase
    end
    data_counter = data_counter + 1;
end

// Assigning Outputs with corresponding variables
assign adc_sck = sclk_out;
assign adc_cs_n = 0;
assign din = din_temp;
assign d_out_ch5 = dout_ch6;
assign d_out_ch6 = dout_ch7;
assign d_out_ch7 = dout_ch5;
assign data_frame = data_viz;

////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule
///////////////////////////////MODULE ENDS///////////////////////////