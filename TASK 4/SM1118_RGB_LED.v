/* Module SM1118_RGB_LED

This module will make the RGB Leds connected to the bot
indicates the appropriate color based on the input from the 
color detection

RGB Led Design
Input  : clk   - 800Khz Clock 
		 color - 2-bit color value
Output : indicator - 2-bit color ouput
		 led1, led2, led3 - rgb leds
*/
module SM1118_RGB_LED(
    input [1:0] color,
    input clk, 
    output reg [2:0] led1, led2, led3,
    output reg [1:0] indicator
);

// Constant Declaration
parameter   init = 2'b00,
            red = 2'b01,
            blue = 2'b10,
            green = 2'b11;

// Variable Declaration
reg flag1 = 0, flag2 = 0, flag3 = 0;
reg color_flag = 0;
reg [14:0] delay_counter = 0;
reg init_flag = 0;

always @(posedge clk) begin
    case(color)
        init:
            begin
				if (init_flag == 0) begin
					led1[2:0] = 3'b000; led2[2:0] = 3'b000;
					led3[2:0] = 3'b000; 
					flag1 = 0;
					flag2 = 0; flag3 = 0;
				end
            end
        red:
            begin
				if (color_flag == 0) begin
				indicator = 2'b01;
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
				init_flag = 1;
            	end
        blue:
            begin 
				if (color_flag == 0) begin
				indicator = 2'b10;
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
				init_flag = 1;
			end
        green:
            begin
				if (color_flag == 0) begin
				indicator = 2'b11;
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
				init_flag = 1;
            end
    endcase
    // Delay Added 
    // So that the same color will not get indicated in multiple leds
	if(color_flag == 1) begin
		delay_counter = delay_counter + 1;
		if(delay_counter > 12000) begin
			color_flag = 0;
			delay_counter = 0;
		end
	end
	// If all the leds are indicating color then make them to zero
	// Just for Task4
	if (flag1 == 1 && flag2 == 1 && flag3 == 1) init_flag = 0;
end

endmodule
