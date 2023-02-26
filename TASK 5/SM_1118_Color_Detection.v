/* Module SM_1118_Color_Detection

This Module is interfaced with Color Sensor, to detect various color
This module has been optimized for detecting red, green and blue.

To get better results, color sensors values are read every 0.06s,
for each color filters.
As it improves accuracy of color detection.

The Detected colors from the sensor appropriately indicated using
the RGB leds which are present on the bot.

Output Frequency from color sensor is Set to 20% 
i.e is S0 = 1, s1 = 0

PhotoDiode Filters are used for more accurate color detection
S2      S3    PhotoDiode Type
0       0      Red Filter
0       1      Blue Filter
1       0      No Filter
1       1      Green Filter

Color Detection Design
Input  - clk    - 8KHz
         cs_out - 20% Frequency Scaling
Output - S0, S1 - Frequency Selection
         S2, S3 - Color Filter Selection
         OE     - ~ Output Enable 
         color  - RGB Led Indication */

// Module Declaration
module SM_1118_Color_Detection(
    input  cs_out, clk,
	 input [1:0] si,
    output reg S0, S1, S2, S3, OE,
    output [1:0] color
);

// Parameters For Defining state Machine
parameter   idle = 2'b00, filter_swap = 2'b01,
            start_read = 2'b10, detect_color = 2'b11;

// Variable Declaration
reg [9:0]   red_counter = 0, 
            green_counter = 0, blue_counter = 0;
reg [7:0]   counter = 0;
reg [1:0]   filter [0:3];
reg [1:0]   present=idle, filter_change = 0, sout = 0;
reg [1:0]   indicator = 0, red = 0, green = 0, blue = 0;

// Assigning Filter Values to a Two Bit Array
initial begin
    // Assigning Color Filter Values
    filter[0] = 2'b10; filter[2] = 2'b01;
    filter[1] = 2'b00; filter[3] = 2'b11;
    S0 = 1; S1 = 0; OE = 0; S2 = 1; S3 = 0;
end

// Counter Runs at  50 Mhz of FPGA Oscillators
always @(posedge clk) begin
	// If there is No SI present with the graph of the Arena
	// then Color Detection is temporarily disabled
	if(si != 0) begin
	    counter = counter + 1'b1;
		case(present)
			idle:
				begin
					// For every 0.03s color filter will get changed
					filter_change = filter_change + 1'b1;
					present = filter_swap;
				end
			filter_swap:
				begin
					sout = filter[filter_change];
					S2 = sout[1]; S3 = sout[0];
					present = start_read;
				end
			start_read:
				begin
					if(filter_change == 3 && counter == 255) present = detect_color;
					else if(counter == 0) present = idle;
				end
			detect_color:
				begin
					// Led Detection
					// Tolerance are set based on the real-time conditions
					if(red_counter > 400 && green_counter > 400 && blue_counter > 400) begin
					    indicator = 2'b00; red = 0; green = 0; blue = 0;
					end
					// The colors are identified only if it detects the same color for
					// two consecutive times.
					else if(red_counter > green_counter && red_counter > blue_counter && green_counter < 200 && blue_counter < 200) begin
						red = red + 1'b1; green = 0; blue = 0;
						if(red == 2) indicator = 2'b01;
					end
					else if(blue_counter > red_counter && blue_counter > green_counter && red_counter < 300 && green_counter < 300) begin
						blue = blue + 1'b1; red = 0; green = 0;
						if(blue == 2) indicator = 2'b10;
					end
					else if(green_counter > red_counter && green_counter > blue_counter && red_counter < 300 && blue_counter < 300) begin
					    green = green + 1'b1; blue = 0; red = 0;
						if(green == 2) indicator = 2'b11;
					end
					present = idle;
					counter = 0; filter_change = 0;
				end
		endcase
	end
	else begin
		filter_change = 0; indicator = 2'b00;
	end 
end

// Clock Output of Color Sensor
always @(posedge cs_out) begin
    if(filter_change == 0) begin
        red_counter = 0; blue_counter = 0; green_counter = 0;
    end
    // Increansing the counters for particular filter change
    else if(filter_change == 1) red_counter = red_counter + 1'b1;
    else if(filter_change == 2) blue_counter = blue_counter + 1'b1;
    else if(filter_change == 3) green_counter = green_counter + 1'b1;
end

// Assigning Led for Appropriate Color Output
assign color = indicator;

endmodule
