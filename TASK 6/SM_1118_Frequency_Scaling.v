/*
Team Id    : 1118
Author List: Hari Vikinesh M, Kulasekaran S, Jerish Abijith Singh  A S,
             Aryan Vinunath
File Name  : SM_1118_Frequency_Scaling.v
Theme      : Soil Monitoring Bot

Module - SM_1118_Frequency_Scaling

This Module will handle all the frequency scaling,
which is required for task implementation

Frequency Scaling Design
Input  - clk_50M  - 50 Mhz FPGA Oscillator

Output - cs_clk_out  - 8khz Output Frequency to Color sensor
         adc_clk_out - 3.125Mhz Output Frequency to ADC Module   */

// Module Declaration
module SM_1118_Frequency_Scaling(
    input clk_50M,
    output cs_clk_out, adc_clk_out
);

// Declaring Variables
reg [14:0] cs_counter = 1;
reg [3:0] s_clk_counter = 1;
reg cs_out = 1, adc_out = 0;

// For Color Sensor 50 Mhz to 8 Khz
always @(posedge clk_50M) begin
    if (cs_counter == 3125) begin
        cs_out = ~cs_out; cs_counter = 0;
    end
    else cs_counter = cs_counter + 1'b1;
end

// For ADC Module 50Mhz to 3.125Mhz
always @(negedge clk_50M) begin  
    if (s_clk_counter == 8) begin
        adc_out = ~adc_out; s_clk_counter = 0;
    end
    s_clk_counter = s_clk_counter + 1'b1;
end

// Assigning Output Clock
assign cs_clk_out = cs_out;
assign adc_clk_out = adc_out;

endmodule
