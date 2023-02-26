// SM : Task 2 B : UART
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.
This file is used to design UART Transmitter.

Recommended Quartus Version : 19.1
The submitted project file must be 19.1 compatible as the evaluation will be done on Quartus Prime Lite 19.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

//UART Transmitter design
//Input   : clk_50M : 50 MHz clock
//Output  : tx : UART transmit output

//////////////////DO NOT MAKE ANY CHANGES IN MODULE//////////////////
module uart(
	input clk_50M,	//50 MHz clock
	output tx		//UART transmit output
);
////////////////////////WRITE YOUR CODE FROM HERE////////////////////

/*
Parameters for UART Transmission
baud rate = 115200
clocks per bit (cpb) = 115200/(50*10^6) = 434
start_bit = 0
2 Stop Bits
stop_bit = 1
*/
parameter cpb = 434,
			 tx_idle = 1,
          tx_start_bit = 0,
          tx_stop_bit = 1;

/*
Data Need to be sent is SM18
8-bit Ascii Values for the indvidual data is
S = 01010011
M = 01001101
1 = 00110001
8 = 00111000
Here the parameter values of all the individual bits
are sent in reverse order
*/
parameter idle = 1'b1,
          start = 1'b0,
          stop = 2'b11,
          S0   = 8'b01010011,
          S1   = 8'b01001101,
          S2   = 8'b00110001,
          S3   = 8'b00111000;

// Initializing Registers with appropriate Values
reg[11:0] counter = 0;
reg[7:0] present=idle;
reg[2:0] index = 0;
reg tx_out = 1;
reg [2:0]data_index = 0;

assign tx = tx_out;

always @(posedge clk_50M) begin
    case(present)

        idle: 
            begin
            counter = counter + 1;
            if (counter < cpb) tx_out = tx_idle;
            else begin
                counter = 0;
                present = start;
            end
            end

        start: 
            begin
            counter = counter + 1;
				if(data_index == 4) tx_out = tx_idle;
				else if(counter < cpb)   tx_out = start[index];
				else if(counter>cpb-1) begin
					counter = 0;
				   if(data_index == 0) present = S0;
				   if(data_index == 1) present = S1;
				   if(data_index == 2) present = S2;
					if(data_index == 3) present = S3;
				 end
				end

        stop:
             begin
             counter = counter + 1;
  				 if(counter < cpb) tx_out = stop[index];
				 else begin
						counter = 0;
						present = idle;
					 end
				 end
        S0  : 
             begin
             counter = counter + 1;
             if(counter < cpb)   tx_out = S0[index];
             else if(index < 8) begin
                counter = 0;
                if (index!= 7) index = index + 1;
					 else begin
                     counter = 0;
                     index = 0;
                     present = stop;
                     data_index = 1;
                     end
					 end
             end

        S1  : 
             begin
             counter = counter + 1;
             if(counter < cpb)   tx_out = S1[index];
             else if(index < 8) begin
                counter = 0;
                if (index!= 7) index = index + 1;
					 else begin
                     counter = 0;
                     index = 0;
                     present = stop;
                     data_index = 2;
                     end
					 end
             end

        S2  : 
             begin
             counter = counter + 1;
             if(counter < cpb)   tx_out = S2[index];
             else if(index < 8) begin
                counter = 0;
                if (index!= 7) index = index + 1;
					 else begin
                     counter = 0;
                     index = 0;
                     present = stop;
                     data_index = 3;
                     end
					 end
             end

        S3  : 
             begin
             counter = counter + 1;
             if(counter < cpb)   tx_out = S3[index];
             else if(index < 8) begin
                counter = 0;
                if (index!= 7) index = index + 1;
					 else begin
                     counter = 0;
                     index = 0;
                     present = stop;
                     data_index = 4;
                     end
					 end
             end

        default:
            tx_out = 1;
    endcase
end

////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule
///////////////////////////////MODULE ENDS///////////////////////////