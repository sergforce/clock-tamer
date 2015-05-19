// Clocktamer HDL gps counter

// Fsum_n = (1-a)*Fsum + F*a   // where a small value in 0..1, say a=1/N where N is integer
// exponential filtering in integer
// Fgr*N = Fgr*N - Fgr + F
// Fext = Fext - Fext/N + F

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
       output reg spi_out_oen // serial output enable
);

parameter COUNTER_BITS = 16 ;
parameter COUNTER_MAX  = 28 ;
parameter COMPARE_PPS_BITS = 28;

reg [COUNTER_MAX-1:0] high_counter; // Counting value
reg [COUNTER_BITS-1:0] cload;       // Serializable data over SPI
reg        one_pps_latch;           // For rising-edge detection
reg        sh_load;

reg [1:0]  spi_clke;                // For SPI clk rising edge detection

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
end else begin
	one_pps_latch <= one_pps;

	if (~one_pps_latch && one_pps) begin
		cload[COUNTER_BITS-1:0] <= high_counter[COUNTER_BITS-1:0];
		high_counter <= 0;
		sh_load <= 1;
	end else begin
		high_counter <= high_counter + 1;
		sh_load <= 0;
		
		if (spi_clke == 2'b01) begin
			cload <= cload << 1;
			pps_compare[COMPARE_PPS_BITS-1:1] <= pps_compare[COMPARE_PPS_BITS-2:0];
			pps_compare[0] <= spi_in;
		end
		
		if (pps_compare[COMPARE_PPS_BITS-1:0] == high_counter[COUNTER_MAX-1:COUNTER_MAX-COMPARE_PPS_BITS]) begin
			one_pps_cont <= ~one_pps_cont;
		end
		
		spi_clke[1] <= spi_clke[0];
		spi_clke[0] <= spi_clk;
		
	end
end

assign spi_out = cload[15];


endmodule

