/* Module SM1118_Black_Line_Following

This module will get the input of line sensor array values
from the A/D Converter of the FPGA and uses the value to
make the bot follows the black line.

Black Line Following Design
Input  : clk          - 3.125 Mhz ADC Clock,
         nodx         - input node to stop the bot,
         turn         - input turn for the bot,
         left_value   - 12-bit digital output of left sensor,
         center_value - 12-bit digital output of center sensor,
         right_value  - 12-bit digital output of right sensor,
Output : direction - 3-bit Output indicating possible direction to move.
         nodes     - Count No of Nodes */ 

// Module Declaration 
module SM1118_Black_Line_Following(
    input clk, 
    input  [11:0] left_value, center_value, right_value,
    input [5:0] nodex,
    input [2:0] turn,
    output reg [2:0] movement,
    output reg [5:0] node,
    output reg [3:0] node_detected,
	 output reg color_flag
	
);

// Variable Declaration
reg [21:0]  delay_counter_stop = 0, node_delay_counter = 0,
            turn_delay_counter = 0, push_delay_counter = 0;
reg [23:0] 	thresh_delay_counter = 0;
reg [17:0]  white_thresh = 150;
reg [8:0] 	thresh = 150;
reg [6:0]   count = 0;
reg [3:0]   node_detection = 0;
reg [1:0]   lap_count = 0;
reg node_flag = 0, before_turn_flag = 0, turn_flag = 0, 
	 after_turn_flag = 0, push_flag = 0, wait_flag = 0;

/* Movements Used             Delays Used
movement   bot_response       time(s)     Counter Value
    0         Stop            0.25        781250
    1         Forward         0.4         1250000
    2         Right           0.50        1562500
    3         Left            0.75        2343750
    4         Reverse         1.00        3125000
    5         turn Right
    6         turn Left     	 */

initial begin
	color_flag = 0;
end

always@(posedge clk) begin
//    if (lap_count < 2) begin
			if(thresh_delay_counter < 2343750) thresh_delay_counter = thresh_delay_counter + 1'b1;
			else if(left_value < thresh && center_value > thresh && right_value < thresh) begin 
				if(count == 0) white_thresh = 150;
				count = count + 1;
				white_thresh = white_thresh + ((left_value + right_value)/2);
				if(count == 100) begin
				thresh = (white_thresh/100) + 30;
				thresh_delay_counter = 0; count = 0;
				end
			end	
		  
        // If the Line follower sensors white light intensity
        // then first push the bot forward for 0.75s else stop the bot
        if (turn_flag == 0 && after_turn_flag == 0 && wait_flag == 0) begin
            if (left_value < thresh && center_value < thresh && right_value < thresh) begin
                if (delay_counter_stop < 781250) begin
                    movement = 1;
                    delay_counter_stop = delay_counter_stop + 1;
                end
                else movement = 0; 
				end
            // If the Line follower sensor senses white, black and white light intensity
            // then follow the black line
            else if (left_value < thresh && center_value > thresh && right_value < thresh) begin
                movement = 1; delay_counter_stop = 0;
            end
            // If the Line Follower sensor senses black light intensity on the right sensor
            // then rotate the bot right
            else if (left_value < thresh  && right_value > thresh) begin
                movement = 2; delay_counter_stop = 0;
            end
            // If the Line Follower sensor senses black light intensity on the left sensor
            // then rotate the bot left
            else if (left_value > thresh && center_value < thresh && right_value < thresh) begin
                movement = 3; delay_counter_stop = 0;
            end
        end
        // Check whether the bot is in the node or not
        // The bot will take the required turn if it senses the input node
        if (left_value > thresh-15 && center_value > thresh-15 && right_value > thresh-15 && turn_flag == 0) begin
            if (node_flag == 0 && turn_flag == 0 && after_turn_flag == 0) begin
                node_flag = 1; //push_flag = 1;
                node = node + 1; node_detected = node; color_flag = 1;
					 if(node == 2 || node == 5 || node == 7) before_turn_flag = 1;
            end
            // If node is 8, then make node to 0
            // and increment the lap count
            if (node == 8) begin
                node = 0; lap_count = lap_count + 1;
            end
        end
        // A 1 second delay added to make sure not to add
        // the same node multiple times
        if (node_flag == 1) begin
            node_delay_counter = node_delay_counter + 1;
            if (node_delay_counter > 1254000) begin
                node_delay_counter = 0;
                node_flag = 0; color_flag = 0;
            end
        end
        // A small Delay to make bot move forward for 0.25s
        // after node detection
        if (push_flag == 1) begin
            movement = 1;
            push_delay_counter = push_delay_counter + 1;
            if (push_delay_counter > 781250) begin
                push_flag = 0;
                push_delay_counter = 0;
            end
        end
        // Align the bot before taking the turn
        if (before_turn_flag == 1) begin
            movement = 1;
            turn_delay_counter = turn_delay_counter + 1;
            if (turn_delay_counter > 300250) begin
                turn_delay_counter = 0; before_turn_flag = 0;
                turn_flag = 1;
            end
        end
        // Turn the bot
        else if (turn_flag == 1) begin
            movement = 5;
            turn_delay_counter = turn_delay_counter + 1;
            if (turn_delay_counter > 681250) begin
                turn_delay_counter = 0; turn_flag = 0;
                after_turn_flag = 1;
            end
        end
        // Turn the bot until the bot detects the line
        else if (after_turn_flag == 1) begin
            movement = 5;
            if (left_value > thresh || right_value > thresh) begin 
					after_turn_flag = 0; color_flag = 0; wait_flag = 1;
				end
		  end
		  else if(wait_flag == 1) begin
			movement = 0; turn_delay_counter = turn_delay_counter + 1'b1;
			if (turn_delay_counter > 350250) begin
				wait_flag = 0; turn_delay_counter = 0;
			end
		  end
//    end

//    else begin
//        // If Two Laps Completed, then push the bot for 0.25s
//        // then stop the bot
//        if (push_delay_counter < 781250) begin
//            movement = 1; push_delay_counter = push_delay_counter + 1;
//        end
//        else movement = 0;
//    end
end

// Assigning Output direction to the Movement Register
endmodule
