/*
Team Id    : 1118
Author List: Hari Vikinesh M, Kulasekaran S, Jerish Abijith Singh  A S,
             Aryan Vinunath
File Name  : SM_1118_Color_Detection.v
Theme      : Soil Monitoring Bot

Module SM_1118_Color_Detection

This Module is interfaced with Color Sensor to detect various color
This module has been optimized for detecting red, green and blue colors.

To get better results, color sensors values are read every 0.06s,
from each color filters during the traversal, As it improves accuracy.

The Detected colors from the sensor are appropriately indicated using
the RGB leds which are present on the bot.

Output Frequency from color sensor is Set to 20% 
i.e is S0 = 1, s1 = 0

PhotoDiode Filters are used for more accurate color detection
S2      S3    PhotoDiode Type
0       0      Red Filter
0       1      Blue Filter
1       0      No Filter
1       1      Green Filter

Also, this module will make the RGB Leds connected to the bot
indicates the appropriate color based on the color sensed

Color Detection Design
Input  - clk       - 8KHz
         cs_out    - 20% Frequency Scaling
         endofrun  - To blink Led at taskend
         colorflag - Blocks Color Detection at Nodes
         oindex    - Turns Off particular Led Using Index
         blockflag - Blocks Color Detection during pick & place
         si        - Enables Color Detection in appropriate Nodes

Output - S0, S1 - Frequency Selection
         S2, S3 - Color Filter Selection
         OE     - ~ Output Enable 
         color  - RGB Led Indication
         led1, led2, led3 - rgb leds */

// Module Declaration
module SM_1118_Color_Detection(
    input  cs_out, clk,
	input  endofrun, colorflag,
	input  [1:0] oindex,
	input  blockflag,
	input  [1:0] si,
    output reg S0, S1, S2, S3, OE,
    output reg [2:0] led1, led2, led3,
    output reg [1:0] color
);

// Parameters For Defining state Machine
parameter   idle = 2'b00, filter_swap = 2'b01,
            start_read = 2'b10, detect_color = 2'b11,
			sred   = 2'b01, sblue = 2'b10, sgreen = 2'b11;

// Variable Declaration
reg [14:0] delay_counter = 0, end_counter = 0;
reg [9:0] red_counter = 0, green_counter = 0, blue_counter = 0;
reg [7:0] counter = 0;
reg [1:0] filter [0:3];
reg [1:0] present=idle, filter_change = 0, sout = 0;
reg [1:0] red = 0, green = 0, blue = 0, temp_si = 0;
reg init_flag = 1, flag1 = 0, flag2 = 0, flag3 = 0, color_flag = 0, white_surface = 0;

// Counter Runs at  8khz of FPGA Oscillators
always @(posedge clk) begin
	if(init_flag == 1) begin
		// Assigning Filter Values to a Two Bit Array
	    filter[0] = 2'b10; filter[2] = 2'b01;
	    filter[1] = 2'b00; filter[3] = 2'b11;
	    S0 = 1; S1 = 0; OE = 0; S2 = 1; S3 = 0; init_flag = 0;
	end
	// If there is No SI present with the graph of the Arena or pick place operation
	// is performing, then Color Detection is temporarily disabled.
	else if(color_flag == 1) color = 0;
	else begin
	    counter = counter + 1'b1;
		case(present)
			idle:
				begin
					// For every 0.03s color filter will get changed
					filter_change = filter_change + 1'b1; present = filter_swap;
				end
			filter_swap:
				begin
					sout = filter[filter_change];
					S2 = sout[1]; S3 = sout[0]; present = start_read;
				end
			start_read:
				begin
					if(filter_change == 3 && counter == 255) present = detect_color;
					else if(counter == 0) present = idle;
				end
			detect_color:
				begin
					if(si != 0 && colorflag == 0 && blockflag == 0 && si != temp_si) begin
						// Led Detection
						// Tolerance are set based on the real-time conditions
						// if(red_counter > 300 && green_counter > 300 && blue_counter > 300) begin
						if(red_counter > green_counter && red_counter > blue_counter && (red_counter - green_counter) < 15 ) white_surface = 1;
						else if (blue_counter > green_counter && blue_counter > red_counter && (blue_counter - green_counter) < 15) white_surface = 1;
						else if (green_counter > red_counter && green_counter > blue_counter && (green_counter - blue_counter) < 15) white_surface = 1;
						if(white_surface) begin
						    color = 2'b00; red = 0; green = 0; blue = 0;
						end
						// A particular color is identified only if it detects the same color for
						// two consecutive times.
						if(blue_counter > red_counter && blue_counter > green_counter) begin
							blue = blue + 1'b1; red = 0; green = 0;
							if(blue == 2) color = 2'b10;
						end
						else if(red_counter > green_counter+20 && red_counter > blue_counter+20) begin
							red = red + 1'b1; green = 0; blue = 0;
							if(red == 2) color = 2'b01;
						end
						else if(green_counter > red_counter+20 && green_counter > blue_counter+20) begin
						    green = green + 1'b1; blue = 0; red = 0;
							if(green == 2) color = 2'b11;
						end
						if(red == 2 || green == 2 || blue == 2) temp_si = si;
						counter = 0; white_surface = 0;
					end
					else if(si == 0) temp_si = 0;
					else color = 0;
					present = idle; filter_change = 0;
				end
		endcase
	end
end

// Clock Output of Color Sensor
always @(posedge cs_out) begin
    if(filter_change == 0) begin
        red_counter = 0; blue_counter = 0; green_counter = 0;
    end
    // Increasing the counters for particular filter change
    else if(filter_change == 1) red_counter = red_counter + 1'b1;
    else if(filter_change == 2) blue_counter = blue_counter + 1'b1;
    else if(filter_change == 3) green_counter = green_counter + 1'b1;
end

always @(posedge clk) begin
	// Initializing Leds Output to Zero
	if(init_flag == 1) begin
	 	led1[2:0] = 3'b000; led2[2:0] = 3'b000; flag1 = 0;
		flag2 = 0; flag3 = 0; led3[2:0] = 3'b000;
	end
	// Blink the Leds at task end
	else if(endofrun == 1) begin
		if(end_counter < 2000) begin
			led1[2:0] = 3'b000; led2[2:0] = 3'b000; led3[2:0] = 3'b000;
		end
		else if(end_counter < 4000) begin
			led1[2:0] = 3'b100; led2[2:0] = 3'b100; led3[2:0] = 3'b100;
		end
		else end_counter = 0;
		end_counter = end_counter + 1'b1;
	end
    else begin
    	// Flags are used to make sure that the color identified is
    	// indicated in different leds
	    case(color)
	        sred:
	            begin
	            	// Indicates Red Color
					if (color_flag == 0) begin
						if(flag1 == 0) begin
						led1[2:0] = 3'b001; flag1 = 1; color_flag = 1;
						end
						else if(flag2 == 0) begin
						led2[2:0] = 3'b001; flag2 = 1; color_flag = 1; 
						end
						else if(flag3 == 0) begin
						led3[2:0] = 3'b001; flag3 = 1; color_flag = 1;
						end
					end
	            	end
	        sblue:
	            begin
	            	// Indicates Blue Color
					if (color_flag == 0) begin
						if(flag1 == 0) begin
							led1[2:0] = 3'b100; flag1 = 1; color_flag = 1;
						end
						else if(flag2 == 0) begin
							led2[2:0] = 3'b100; flag2 = 1; color_flag = 1;
						end
						else if(flag3 == 0) begin
							led3[2:0] = 3'b100; flag3 = 1; color_flag = 1;
						end
					end
				end
	        sgreen:
	            begin
	            	// Indicates Green Color
					if (color_flag == 0) begin
						if(flag1 == 0) begin
							led1[2:0] = 3'b010; flag1 = 1; color_flag = 1;
						end
						else if(flag2 == 0) begin
							led2[2:0] = 3'b010; flag2 = 1; color_flag = 1;
						end
						else if(flag3 == 0) begin
							led3[2:0] = 3'b010; flag3 = 1; color_flag = 1;
						end
					end
	            end
	    endcase
	    // Delay Added, So that the detected color
	    // will not get indicated in multiple leds
		if(color_flag == 1) begin
			delay_counter = delay_counter + 1'b1;
			if(delay_counter > 8000) begin
				color_flag = 0; delay_counter = 0;
			end
		end
		// Turns off a led using the input index
		case(oindex)
		0:
		begin
			led1[2:0] = 0; flag1 = 0;
		end
		1:
		begin
			led2[2:0] = 0; flag2 = 0;
		end
		2:
		begin
			led3[2:0] = 0; flag3 = 0;
		end
		endcase
	end
end

endmodule
