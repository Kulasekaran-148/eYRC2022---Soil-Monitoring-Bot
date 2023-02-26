/*
Team Id    : 1118
Author List: Hari Vikinesh M, Kulasekaran S, Jerish Abijith Singh  A S,
             Aryan Vinunath
File Name  : SM_Xbee_Receiver.v
Theme      : Soil Monitoring Bot

Module SM_1118_Xbee_Receiver

This Module is responsible for receving the Supply Position Message
from the Digi Xbee which is connected to PC. Each character in the
SPM message is sent using serail terminal H-term. The recieved data 
is converted into parallel 8-bit data and decoded into respective
color values. These color values are then stored in an array. 

Xbee Receiver design
Input   : clk_50M  - 50 MHz clock
          node_si  - 1, 2, 3
          color    - R, G, B
          msg_type - SI, SPiM, SDM
          field    - MT, PP, VG, NG
Output  : tx          - UART transmit output
          tx_complete - Status of Transmit */

// Module Declaration
module SM_1118_Xbee_Receiver(
    input rx, clk_50m,
    output reg [3:0] rx_index,
    output reg [1:0] rx_color
);

// Parameters for State Machine Declared
parameter   idle = 2'b00, start = 2'b01, rxspm = 2'b10, stop = 2'b11,
            char_dash = 8'b00101101, char_hash = 8'b00100011,
            char_B = 8'b01000010, char_G = 8'b01000111,
            char_R = 8'b01010010, char_N = 8'b01001110, cpb = 434;

// Variable Declarations
reg [8:0] counter = 0;
reg [7:0] msg = 0;
reg [3:0] index = 0, color_ind = 0;
reg [1:0] present = idle, color_array[0:7], sync_counter = 0;
reg rx_done = 0, init_flag = 1;

always @(posedge clk_50m) begin
    if(init_flag == 1) begin
        if(color_ind < 8) begin
            color_array[color_ind] = 0; color_ind = color_ind + 1'b1;
        end
        else begin
            color_ind = 1; init_flag = 0;
        end
    end
    else if(rx_done == 0) begin
        case(present)
            idle:
                // Idle State
                begin
                    counter = 0; 
                    if (rx == 1'b0) present = start;
                end

            start:
                // Reading Start Bit
                begin
                    if(counter == 434) begin
                        present = rxspm; counter = 0;
                    end
                    else counter = counter + 1'b1;
                end

            rxspm:
                // Reading Data Bits
                begin
                    counter = counter + 1'b1;
                    // Serial to Parallel Conversion
                    // Using Shift Register
                    if(counter == 15) msg <= {rx, msg[7:1]};
                    if(counter == 434) begin
                        index = index + 1'b1; counter = 0;
                    end
                    // Mapping the Input Color to an Array
                    if(index == 8) begin
                        index = 0; present = stop;
                        // If Data rx is # or 7 colors recieved, then
                        // the message has been recieved successfully. 
                        if(msg == char_hash) rx_done = 1;
                        // Mapping Input Color to Value
                        else if(msg == char_R) begin
                            color_array[color_ind] = 1; color_ind = color_ind + 1'b1;
                        end
                        else if(msg == char_B) begin
                            color_array[color_ind] = 2; color_ind = color_ind + 1'b1;
                        end
                        else if(msg == char_G) begin
                            color_array[color_ind] = 3; color_ind = color_ind + 1'b1;
                        end
                        else if(msg == char_N) begin
                            color_array[color_ind] = 0; color_ind = color_ind + 1'b1;
                        end
                        if(color_ind == 8) begin
                            rx_done = 1; color_ind = 1;
                        end
                        msg <= 0;
                    end
                end

            stop:
                // Reading Stop Bit
                begin
                    if(counter == 434) present = idle;
                    counter = counter + 1'b1;
                end
        endcase
    end
    else if(rx_done == 1 && color_ind < 8) begin
        if(sync_counter == 0) begin
            rx_color = color_array[color_ind]; rx_index = color_ind;
        end
        else if(sync_counter == 3) color_ind = color_ind + 1'b1;
        sync_counter = sync_counter + 1'b1;
    end
end

endmodule