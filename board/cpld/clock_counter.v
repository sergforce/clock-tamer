// Clocktamer HDL gps counter

module clock_counter (
       input clk,      // High clock input
       input one_pps,  // One PPS signal
       input nreset,   // Reset
       output reg one_pps_cont, // Continues 1PPS signal 
       output clk_div,

       input spi_clk,     // serial clock 
       input spi_sen,     // crystal enable for serial clk
       output spi_out,    // serial data out
       input  spi_in,     // serial data in
       output spi_out_oen // serial output enable
);

parameter COUNTER_BITS     = 16;

parameter COUNTER_MAX      = 28;
parameter COMPARE_PPS_BITS = 28;

reg [COUNTER_MAX-1:0] high_counter; // Counting value
reg [COUNTER_BITS:0] cload;         // Serializable data over SPI (one extra bit as register present flag)
reg        one_pps_latch;           // For rising-edge detection

reg [1:0]  spi_clke;                // For SPI clk rising edge detection
reg        spi_trans_update;        // Set to one when processing divider update SPI transaction
reg        spi_trans_started;       // Set to one for on the first bit of transaction

reg [COMPARE_PPS_BITS-1:0]  pps_compare;

assign clk_div = high_counter[COUNTER_BITS];

always @(posedge clk)
if (~nreset) begin
	high_counter <= 0;
	cload        <= 0;
	one_pps_latch <= 1'b0;

	one_pps_cont <= 1'b0;
	pps_compare <= 1'b0;
	spi_clke <= 2'b0;
	spi_trans_update <= 1'b0;
	spi_trans_started <= 1'b0;
end else begin
	one_pps_latch <= one_pps;

	if (~one_pps_latch && one_pps) begin
		// shifting out lower parts
		cload[COUNTER_BITS-1:0] <= high_counter[COUNTER_BITS-1:0];
		cload[COUNTER_BITS] <= 1'b1; // Set flag that we have new valid data
		high_counter <= 0;
	end else begin
		high_counter <= high_counter + 1;
		
		if (spi_clke == 2'b01) begin
			if (spi_sen == 0'b0) begin
				if (spi_trans_started == 1'b0) begin
					spi_trans_update <= ~spi_in;
					spi_trans_started <= 1'b1;
				end else begin
					if (spi_trans_update == 1'b1) begin
						pps_compare[COMPARE_PPS_BITS-1:1] <= pps_compare[COMPARE_PPS_BITS-2:0];
						pps_compare[0] <= spi_in;
					end
				end
				cload <= cload << 1;
			end else begin
				spi_trans_update <= 1'b0;
				spi_trans_started <= 1'b0;
			end
		end
		
		// THIS DOESN'T WORK YET
		if (pps_compare[COMPARE_PPS_BITS-1:0] == high_counter[COUNTER_MAX-1:COUNTER_MAX-COMPARE_PPS_BITS]) begin
			one_pps_cont <= ~one_pps_cont;
		end
		
		spi_clke[1] <= spi_clke[0];
		spi_clke[0] <= spi_clk;
	end
end

assign spi_out = cload[COUNTER_BITS];
assign spi_out_oen = ~spi_sen;

endmodule

