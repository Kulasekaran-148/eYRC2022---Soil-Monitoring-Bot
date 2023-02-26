/* 
Team Id    : 1118
Author List: Hari Vikinesh M, Kulasekaran S, Jerish Abijith Singh  A S,
             Aryan Vinunath
File Name  : SM_1118_Black_Line_Following.v
Theme      : Soil Monitoring Bot

Module SM_1118_Black_Line_Following

This module will get the input of line sensor values from 
the A/D Converter of the FPGA and uses those values to
achieve black line following. 

Black Line Following Design
Input  : clk          - 3.125 Mhz ADC Clock
         nodex        - input node to stop the bot
         turn         - input turn for the bot
         left_value   - 12-bit digital output of left sensor
         center_value - 12-bit digital output of center sensor
         right_value  - 12-bit digital output of right sensor

Output : direction    - 3-bit register to control movement of motors
         nodes        - number of nodes traversed                     */

// Module Declaration 
module SM_1118_Black_Line_Following(
    input  taskend,
    input  clk,
    input  [11:0] left_value, center_value, right_value,
    input  [2:0] turn,
    input  rxdone,
    output reg colorflag,
    output reg [4:0] node, nodesdetected,
    output reg [3:0] direction
);

// Variable Declaration
reg [21:0] delay_counter_stop = 0, node_delay_counter = 0, push_delay_counter = 0,
           thresh_delay_counter = 0, turn_delay_counter = 0;
reg [17:0] white_thresh = 150, thresh = 150, node_thresh = 150;
reg [6:0]  count = 0;
reg [3:0]  movement = 0, stable_counter = 0;
reg [2:0]  temp_turn = 0;
reg [1:0]  sync_counter = 0;
reg node_flag = 0, before_turn_flag = 0, turn_flag = 0, push_flag = 0,
    sync_flag = 1, after_turn_flag = 0,  wait_flag = 0;

/* Movements Used             Delays Used
movement   bot_response       time(s)     Counter Value
    0         Stop            0.11        350250
    1         Forward         0.16        500250
    2         Right           0.21        681250
    3         Left            0.25        781250
    4         Reverse         0.32        1000500
    5         90Deg Right     0.40        1250000
    6         90Deg Left      0.50        1562500
    7         180 turn        0.69        2162500
                              0.75        2343750
    
The value assignment convention for the 3-bit
output register "direction" used for controlling
the motors is explained below.

+ = Clockwise
- = Anti-Clockwise
/ = doesn't rotate

Input   Binary Values   Motor Rotation  Direction
                           M1     M2
  0         0000           /      /       Stop
  1         1010           +      +       Forward
  2         1000           +      /       Right
  3         0010           /      +       Left
  4         0101           -      -       Reverse
  5         1001           +      -       turn Right
  6         0110           -      +       turn left
  7         0110           -      +       180 turn   */

always@(posedge clk) begin
    if(rxdone == 1 && taskend == 0) begin
        // Thresh Delay Counter is used to get threshold value of
        // white surface for every 0.75s
		if(thresh_delay_counter < 2343750) thresh_delay_counter = thresh_delay_counter + 1'b1;
		else if(left_value < thresh && center_value > thresh && right_value < thresh) begin
			if(count == 0) white_thresh = 150;
			count = count + 1'b1;
			white_thresh = white_thresh + ((left_value + right_value)/2'b10);
			// Calculating average thresh after 100 cycles
            if(count == 100) begin
    			thresh = (white_thresh/8'b01100100) + 9'b000011110;
                node_thresh = (white_thresh/8'b01100100) + 9'b000001010;
    			thresh_delay_counter = 0; count = 0;
			end
		end
        // If the Line follower sensors white light intensity
        // then first push the bot forward for 0.25s else stop the bot
        if(turn_flag == 0 && wait_flag == 0) begin
            //If bot is in white surface, pushing the bot forward for 0.16s
            if(left_value < thresh && center_value < thresh && right_value < thresh) begin
                if(delay_counter_stop < 500250) begin
                    movement = 1; delay_counter_stop = delay_counter_stop + 1'b1;
                end
                // if bot is still in white surface, moving the bot
                // in reverse for 0.25s
                else if(delay_counter_stop < 2343750) begin
                    movement = 4; delay_counter_stop = delay_counter_stop + 1'b1;
                end
                //if bot is still in white surface, stopping the bot
                else movement = 0;
			end
            // if the line follower sensor senses white - black - white combination,
            // the bot is made to move forward
            else if(left_value < thresh && center_value > thresh && right_value < thresh) begin
                // Stable Counter is used to make the bot move in a stable manner
                // by counter acting the difference between the speed of two wheels
                if(stable_counter == 15) movement = 1;
                else movement = 3;
                delay_counter_stop = 0; stable_counter = stable_counter + 1'b1;
            end
            // If the Line Follower sensor senses white - white - black combination,
            // then adjust the bot by rotating right
            else if(left_value < thresh && center_value < thresh && right_value > thresh) begin
                movement = 2; delay_counter_stop = 0;
            end
            // If the Line Follower sensor senses black - white - white combination,
            // then adjust the bot by rotating left
            else if(left_value > thresh && center_value < thresh && right_value < thresh) begin
                movement = 3; delay_counter_stop = 0;
            end
        end

        // if the line follower sensor senses black - black - black combination,
        // then the bot is in a node and node count is added
        if(left_value > node_thresh && center_value > node_thresh && right_value > node_thresh) begin
            if(node_flag == 0 && turn_flag == 0 && after_turn_flag == 0 && before_turn_flag == 0) begin
                node_flag = 1; node = node + 1'b1; colorflag = 1; sync_flag = 1; sync_counter = 0;
                nodesdetected = nodesdetected + 1'b1; push_flag = 1;
            end
        end
        // Sync Flag and Sync Counter is used to synchronize multiple Clock of different codes
        if(sync_flag == 1 && sync_counter == 3) begin
            if(turn == 4 || turn == 5 || turn == 6 || turn == 7) begin
                temp_turn = turn; before_turn_flag = 1; sync_flag = 0; sync_counter = 0;
                if(turn == 4) temp_turn = 7;
            end
        end
        else sync_counter = sync_counter + 1'b1;
        if(turn == 0) node = 0;
        // A 0.4s delay added to make sure not to add
        // the same node multiple times
        if (node_flag == 1) begin
            node_delay_counter = node_delay_counter + 1'b1;
            if (node_delay_counter > 1250000) begin
                node_flag = 0; node_delay_counter = 0; colorflag = 0;
                if(temp_turn == 5 || temp_turn == 6) colorflag = 1;
            end
        end
        // Make the Bot Move Forward for 0.25s after detecting the Node
        if(push_flag == 1 && before_turn_flag != 0) begin
            push_delay_counter = push_delay_counter + 1'b1;
            if(push_delay_counter > 781250) begin
                push_flag = 0; push_delay_counter = 0;
            end
        end
        // Align the bot before taking the turn
        if (before_turn_flag == 1) begin
            movement = 1; turn_delay_counter = turn_delay_counter + 1'b1;
            if (turn_delay_counter > 300250) begin
                before_turn_flag = 0; turn_delay_counter = 0; turn_flag = 1;
            end
        end
        // Turn the bot
        else if (turn_flag == 1) begin
            movement = temp_turn; turn_delay_counter = turn_delay_counter + 1'b1;
            if ((temp_turn == 7 && turn_delay_counter > 2162500) || (temp_turn != 7 && turn_delay_counter > 681250)) begin
                turn_delay_counter = 0; turn_flag = 0; after_turn_flag = 1;
            end
        end
        // Turn the bot until the bot detects the line
        else if (after_turn_flag == 1) begin
            movement = temp_turn;
            if((temp_turn == 6 || temp_turn == 7) && left_value > thresh) begin
                after_turn_flag = 0; wait_flag = 1;
            end
            if(temp_turn == 5 && right_value > thresh) begin
                after_turn_flag = 0; wait_flag = 1;
            end
        end
        // Wait for 0.11s after turn
        else if(wait_flag == 1 && turn_delay_counter < 350750) begin
            movement = 0; turn_delay_counter = turn_delay_counter + 1'b1;
//        end
//        else if(wait_flag == 1) begin
            turn_delay_counter = turn_delay_counter + 1'b1;
            if(turn_delay_counter > 350250 && turn_delay_counter < 350500) movement = 1;
            else if(temp_turn == 5 && turn_delay_counter < 350750) movement = 2;
            else if((temp_turn == 6 || temp_turn == 7) && turn_delay_counter > 350750) movement = 3;
            else if(turn_delay_counter >= 350750) begin
                turn_delay_counter = 0; wait_flag = 0; temp_turn = 0; colorflag = 0;
            end
        end

        // Mapping Movement Values to Level Shifter for Movement Control
        case(movement)
            0: direction[3:0] = 4'b0000;
            1: direction[3:0] = 4'b1010;
            2: direction[3:0] = 4'b1000;
            3: direction[3:0] = 4'b0010;
            4: direction[3:0] = 4'b0101;
            5: direction[3:0] = 4'b1001;
            6: direction[3:0] = 4'b0110;
            7: direction[3:0] = 4'b0110;
        endcase
    end
    else direction = 0;
end

endmodule
