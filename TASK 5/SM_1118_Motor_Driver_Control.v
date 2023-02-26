/* Module SM_1118_MotorDriver_Control

This module controls the Motor Driver to rotate the bot accordingly.

Motor Driver Control Design
Input  : clk       : Input Clk 3.125 Mhz
         direction : 2-bit value representing Movement
Output : em : 2-bit level shifter values for ElectroMagnet
         mc : 3-bit level shifter values to rotate Motor
*/

// Module Decalration
module SM_1118_Motor_Driver_Control(
    input  pickup,
    input  [3:0] direction,
    input clk,
    output reg [1:0] em,
    output reg [3:0] mc
);
/* The 4-bit binary Values for rotating Motor in desired direction
is represented here.

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

/* pickup is a input value which is used to power ON/OFF ElectroMagnet
using Level Shifter 1 - 10, 0 - 00 */
always @(posedge clk) begin
    case(direction)
        0: mc[3:0] = 4'b0000;
        1: mc[3:0] = 4'b1010;
        2: mc[3:0] = 4'b1000;
        3: mc[3:0] = 4'b0010;
        4: mc[3:0] = 4'b0101;
        5: mc[3:0] = 4'b1001;
        6: mc[3:0] = 4'b0110;
		7: mc[3:0] = 4'b0110;
    endcase
    case(pickup)
        0: em[1:0] = 2'b00;
        1: em[1:0] = 2'b10;
    endcase
end

endmodule
