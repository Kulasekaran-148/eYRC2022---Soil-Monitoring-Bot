// SM : Task 1 C : Finite State Machine
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.
This file is used to design a Finite State Machine.

Recommended Quartus Version : 19.1
The submitted project file must be 19.1 compatible as the evaluation will be done on Quartus Prime Lite 19.1.

Warning: The error due to compatibility will not be entertained.
			Do not make any changes to Test_Bench_Vector.txt file. Violating will result into Disqualification.
-------------------
*/

//Finite State Machine design
//Inputs  : I (4 bit) and CLK (clock)
//Output  : Y (Y = 1 when 1094 sequence(decimal number sequence) is detected)

//////////////////DO NOT MAKE ANY CHANGES IN MODULE//////////////////
module fsm(
	input CLK,			  //Clock
	input [3:0]I,       //INPUT I
	output	  Y		  //OUTPUT Y
);

////////////////////////WRITE YOUR CODE FROM HERE//////////////////// 
	

// Tip : Write your code such that Quartus Generates a State Machine 
//			(Tools > Netlist Viewers > State Machine Viewer).
// 		For doing so, you will have to properly declare State Variables of the
//       State Machine and also perform State Assignments correctly.
//			Use Verilog case statement to design.

parameter init = 4'b0000,
          S0   = 4'b0001,
          S1   = 4'b0000,
          S2   = 4'b1001,
          S3   = 4'b0100;

reg [3:0] present, next=init;
reg temp = 0;
assign Y = temp;

always@(posedge CLK)
	begin
		 temp = (next == S0) ? 1:0;
		 present <= next;
	end

always@(present or I)
	case(present)
		 S0: if(I==1) next = S1;
			  else next = init;
		 S1: if(I==0) next = S2;
			  else next = init;
		 S2: if(I==9) next = S3;
			  else next = init;
		 S3: if(I==4) next = S0;
			  else next = init;

			default: next = init;
	endcase

////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule
///////////////////////////////MODULE ENDS///////////////////////////