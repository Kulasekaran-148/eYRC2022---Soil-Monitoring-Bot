/* Module SM1118_Status_Update

This Module is responsible to update the Status Identification
for each successful detection by the bot.
This Node will also control the UART, So that it is will transmit
the messages only once.

Status Update Design
Input  : clk     - 50Mhz Clock
         tx_done - Status of message Transmission
         si_no   - 2-bit Status Number
         color   - 2-bit Color Value
Output : tx_start - Enable message Transmission
*/

// Module Declaration
module SM1118_Status_Update(
    input tx_done,
    input clk,
    input [1:0] color, si_no,
    output reg tx_start
);

// Variable Declaration
reg [1:0] local_si = 0, local_color = 0;

always @(posedge clk) begin
    // If message Transmission is completed, then make tx_start to 0
    if (tx_done == 1) begin
        tx_start = 0;
    end
    else begin
        // Make sure same message doesn't not transmitted multiple times
        if(si_no != local_si && color != local_color && tx_done == 0) begin
            tx_start = 1;
            local_si = si_no; local_color = color;
        end
    end
end

endmodule