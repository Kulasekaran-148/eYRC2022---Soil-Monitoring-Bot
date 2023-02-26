/*
Team Id    : 1118
Author List: Hari Vikinesh M, Kulasekaran S, Jerish Abijith Singh  A S,
             Aryan Vinunath
File Name  : SM_1118_Xbee_Transmitter.v
Theme      : Soil Monitoring Bot

Module SM_1118_Xbee_Transmitter

This Module is responsible for all the messages that is being transmitted
from the FPGA board to a Digi Xbee connected to a PC, using UART Communication.

Xbee Transmitter design
Input   : clk_50M  - 50 MHz clock
          node_si  - 1, 2, 3
          color    - R, G, B
          msg_type - SI, SPiM, SDM
          field    - MT, PP, VG, NG

Output  : tx          - UART transmit output
          tx_complete - Status of Transmit */

// Module Declaration
module SM_1118_Xbee_Transmitter(
    input  [1:0] node_si, color,
    input  tx_start,
    input  [1:0] field, msg_type,
    input  clk_50M,
    output tx_complete, tx
);

/*  Parameters for UART Transmission
    baud rate = 115200
    clocks per bit (cpb) = 115200/(50*10^6) = 434
    start_bit = 0
    stop_bit = 1
    1 start & stop bit & no parity bit

Ascii Values for Required  Characters
- - 00101101    V  - 01010110
# - 00100011    W  - 01010111
D - 01000100    Z  - 01011010
M - 01001101    1  - 00110001
N - 01001110    2  - 00110010
I - 01001001    3  - 00110011    
P - 01010000    8  - 00111000
S - 01010011    \n - 00001010

Different Type of messages that are need to be sent are
1. SI Identification Message (SI)
   Ex:
    If Blue SI is identified at SIV1 in Vegetable Garden (VG), the message is “SI-SIV1-W-#”.
    If Green SI is identified at SIM3 in Maize Terrains (MT), the message is “SI-SIM3-N-#”.
2. Supply Pick Message (SPiM)
   Ex:
    If Blue Supply is picked for DZV1, the message is “S-P-DZV1-W-#”
    If Green Supply is picked for DZM3, the message is “S-P-DZM3-N-#”
3. Supply Deposition Message (SDM)
   Ex:
    If Blue Supply is deposited at DZV1, the message is “S-D-DZV1-W-#”
    If Green Supply is deposited at DZM3, the message is “S-D-DZM3-N-#”

Here SI message has total of 11 characters to transmit whereas 
SPM and SDM has total of 12 charamters to transmit.
This message are sent based on the input color and the node Identified. */

// Required Parameters for the state machine is defined below
parameter   idle = 3'b001, start = 3'b010, stop = 3'b011, cpb = 434,
            tx_ft_1 = 3'b100, tx_ft_2 = 3'b101, tx_ft_3 = 3'b110,
            char_dash = 8'b00101101, char_hash = 8'b00100011,
            char_D = 8'b01000100, char_M = 8'b01001101,
            char_N = 8'b01001110, char_I = 8'b01001001,
            char_P = 8'b01010000, char_S = 8'b01010011,
            char_V = 8'b01010110, char_W = 8'b01010111,
            char_Z = 8'b01011010, char_1 = 8'b00110001, 
            char_2 = 8'b00110010, char_3 = 8'b00110011, 
            char_8 = 8'b00111000, char__ = 8'b00001010;

// Initializing Registers with appropriate Values
reg [11:0] counter = 0;
reg [7:0] msg;
reg [3:0] data_index = 0;
reg [2:0] present=idle, index = 0;
reg tx_done = 0, tx_out = 1;

always @(posedge clk_50M) begin
    if(tx_start == 1) begin
        tx_done = 0; counter = counter + 1'b1;
        case(present)
            idle: 
                begin
                if(tx_done == 1) tx_out = 1'b1;
                else if(counter < cpb) tx_out = 1'b1;
                else begin
                    counter = 0; present = start;
                    end
                end

            start: 
                begin
                // Sending Start Bit
                if(counter < cpb)   tx_out = start[index];
                else if(counter > cpb-1) begin
                    counter = 0;
                    if(msg_type == 0) tx_out = 1'b1;
                    else if(msg_type == 1) present = tx_ft_1;
                    else if(msg_type == 2) present = tx_ft_2;
                    else if(msg_type == 3) present = tx_ft_3;
                    end
                end

            stop:
                begin
                // Sending Stop Bit
                if(counter < cpb) tx_out = stop[index];
                else begin
                    counter = 0; present = idle;
                    end
                end

            tx_ft_1:
                // Msg Format : SI-SIN1-N-# 
                begin
                if(data_index == 0) msg = char_S;
                else if(data_index == 1) msg = char_I;
                else if(data_index == 2) msg = char_dash;
                else if(data_index == 3) msg = char_S;
                else if(data_index == 4) msg = char_I;
                else if(data_index == 5) begin
                    if (field == 0) msg = char_M;
                    else if (field == 1) msg = char_P;
                    else if (field == 2) msg = char_N;
                    else if (field == 3) msg = char_V;
                    end
                else if(data_index == 6) begin
                    if(node_si == 1) msg = char_1;
                    else if(node_si == 2) msg = char_2;
                    else if(node_si == 3) msg = char_3;
                    end
                else if(data_index == 7) msg = char_dash;
                else if(data_index == 8) begin
                    if(color == 1) msg = char_P;
                    else if(color == 2) msg = char_W;
                    else if(color == 3) msg = char_N;
                    end
                else if(data_index == 9) msg = char_dash;
                else if(data_index == 10) msg = char_hash;
                else if(data_index == 11) msg = char__;
                if(counter < cpb) tx_out = msg[index];
                else if(index < 8) begin
                    counter = 0;
                    if (index!= 7) index = index + 1'b1;
                    else begin
                        counter = 0; index = 0;
                        data_index = data_index + 1'b1;
                        present = stop;
                        if(data_index == 12) begin
                            data_index = 0; tx_done = 1;
                        end
                    end
                end       
                end

            tx_ft_2:
                // Msg Format : S-P-DZN1-N-#
                begin
                if(data_index == 0) msg = char_S;
                else if(data_index == 1) msg = char_dash;
                else if(data_index == 2) msg = char_P;
                else if(data_index == 3) msg = char_dash;
                else if(data_index == 4) msg = char_D;
                else if(data_index == 5) msg = char_Z;
                else if(data_index == 6) begin
                    if (field == 0) msg = char_M;
                    else if (field == 1) msg = char_P;
                    else if (field == 2) msg = char_N;
                    else if (field == 3) msg = char_V;
                    end
                else if(data_index == 7) begin
                    if(node_si == 1) msg = char_1;
                    else if(node_si == 2) msg = char_2;
                    else if(node_si == 3) msg = char_3;
                    end
                else if(data_index == 8) msg = char_dash;
                else if(data_index == 9) begin
                    if(color == 1) msg = char_P;
                    else if(color == 2) msg = char_W;
                    else if(color == 3) msg = char_N;
                    end
                else if(data_index == 10) msg = char_dash;
                else if(data_index == 11) msg = char_hash;
                else if(data_index == 12) msg = char__;
                if(counter < cpb)   tx_out = msg[index];
                else if(index < 8) begin
                    counter = 0;
                    if (index!= 7) index = index + 1'b1;
                    else begin
                        counter = 0; index = 0;
                        data_index = data_index + 1'b1;
                        present = stop;
                        if(data_index == 13) begin
                            data_index = 0; tx_done = 1;
                        end
                    end
                end   
                end

            tx_ft_3:
                // Msg Format : S-D-DZN1-N-#
                begin
                if(data_index == 0) msg = char_S;
                else if(data_index == 1) msg = char_dash;
                else if(data_index == 2) msg = char_D;
                else if(data_index == 3) msg = char_dash;
                else if(data_index == 4) msg = char_D;
                else if(data_index == 5) msg = char_Z;
                else if(data_index == 6) begin
                    if (field == 0) msg = char_M;
                    else if (field == 1) msg = char_P;
                    else if (field == 2) msg = char_N;
                    else if (field == 3) msg = char_V;
                    end
                else if(data_index == 7) begin
                    if(node_si == 1) msg = char_1;
                    else if(node_si == 2) msg = char_2;
                    else if(node_si == 3) msg = char_3;
                    end
                else if(data_index == 8) msg = char_dash;
                else if(data_index == 9) begin
                    if(color == 1) msg = char_P;
                    else if(color == 2) msg = char_W;
                    else if(color == 3) msg = char_N;
                    end
                else if(data_index == 10) msg = char_dash;
                else if(data_index == 11) msg = char_hash;
                else if(data_index == 12) msg = char__;
                if(counter < cpb)   tx_out = msg[index];
                else if(index < 8) begin
                    counter = 0;
                    if (index!= 7) index = index + 1'b1;
                    else begin
                        counter = 0; index = 0;
                        data_index = data_index + 1'b1;
                        present = stop;
                        if(data_index == 13) begin
                            data_index = 0; tx_done = 1;
                        end
                    end
                end
                end

            default:
                tx_out = 1;
        endcase
    end
end

// Assigning Outputs to Appropriate Registers
assign tx = tx_out;
assign tx_complete = tx_done;

endmodule
