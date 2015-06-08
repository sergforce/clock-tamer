// Clocktamer HDL gps counter

module clock_counter_fixed_1 #(
		parameter COUNTER_BITS     = 27,
		parameter COMPARE_PPS_BITS = 25,
		parameter FIXED_CLOCK = 19200000
)(
       input clk,      // High clock input
       input one_pps,  // One PPS signal
       input nreset,   // Reset
       input pps_sync_mode,     // use cont 1pps instead gps
       output reg one_pps_cont, // Continues 1PPS signal 
       output clk_div,

       input fixed_clk,

       input spi_clk,      // serial clock 
       input spi_sen,      // crystal enable for serial clk
       output spi_out,     // serial data out
       input  spi_in,      // serial data in
       output spi_out_oen // serial output enable
);

reg [COUNTER_BITS:0] high_counter;  // Counting value
reg [COUNTER_BITS:0] cload;         // Serializable data over SPI (one extra bit as register present flag)

reg [COUNTER_BITS:0] buf_cload;
reg [COMPARE_PPS_BITS-1:0]  pps_div;

assign clk_div = high_counter[COUNTER_BITS];

always @(posedge clk or negedge nreset)
if (~nreset) begin
	high_counter <= 0;
end else begin
	high_counter <= high_counter + 1;
end


always @(posedge one_pps or negedge nreset)
if (~nreset) begin
	cload <= 0;
end else begin
	cload[COUNTER_BITS-1:0] <= high_counter[COUNTER_BITS-1:0];
	cload[COUNTER_BITS]     <= 1'b1; // Set flag that we have new valid data
end


always @(posedge spi_clk or negedge nreset)
if (~nreset) begin
	buf_cload <= 0;
end else begin
	if (!spi_sen) begin
		buf_cload <= cload;
	end else begin
		buf_cload <= buf_cload << 1;
	end
end


always @(posedge fixed_clk or negedge nreset)
if (~nreset) begin
	one_pps_cont <= 1'b0;
   pps_div      <= 0;
end else begin
	if (pps_sync_mode) begin
		if (FIXED_CLOCK == pps_div[COMPARE_PPS_BITS-1:0]) begin
			one_pps_cont <= ~one_pps_cont;
			pps_div <= 0;
		end else begin
			pps_div <= pps_div + 1;
		end
	end else begin
	   one_pps_cont <= one_pps;
	end
end

assign spi_out = buf_cload[COUNTER_BITS];
assign spi_out_oen = ~spi_sen;

endmodule

