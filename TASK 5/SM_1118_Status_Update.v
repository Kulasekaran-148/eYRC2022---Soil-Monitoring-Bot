/* Module SM_1118_Status_Update

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
module SM_1118_Status_Update(
    input clk,
    input [1:0] color, si_no, farm, msgtype,
    input tx_done,
    output reg tx_start,
    output reg [1:0] su_color, su_sino, su_farm, su_msgtype
);

// Variable Declaration
reg [1:0] local_si = 0, local_color = 0;
reg [1:0] local_farm = 0, local_msgtype = 1;

always @(posedge clk) begin
    // If message Transmission is completed, then make tx_start to 0
    if (tx_done == 1) begin
        tx_start = 0;
    end
    else begin
        // If the Msg Type is changed from previous Messages
		if(msgtype!=local_msgtype) begin
            su_color = local_color; su_sino = local_si; su_farm = local_farm;
			su_msgtype = msgtype; local_msgtype = msgtype; tx_start = 1;
		end
        // Make sure same message doesn't not transmitted multiple times
        if(si_no != local_si && color != local_color && tx_done == 0 && si_no != 0) begin
            tx_start = 1; local_farm = farm; local_msgtype = msgtype;
            local_si = si_no; local_color = color; su_msgtype = msgtype;
            su_color = color; su_sino = si_no; su_farm = farm;
        end
    end
end

endmodule