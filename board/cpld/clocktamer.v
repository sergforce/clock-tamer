// Copyright (C) 1991-2014 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions 
// and other software and tools, and its AMPP partner logic 
// functions, and any output files from any of the foregoing 
// (including device programming or simulation files), and any 
// associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License 
// Subscription Agreement, the Altera Quartus II License Agreement,
// the Altera MegaCore Function License Agreement, or other 
// applicable license agreement, including, without limitation, 
// that your use is for the sole purpose of programming logic 
// devices manufactured by Altera and sold by Altera or its 
// authorized distributors.  Please refer to the applicable 
// agreement for further details.

module clocktamer
(
// {ALTERA_ARGS_BEGIN} DO NOT REMOVE THIS LINE!

// {ALTERA_ARGS_END} DO NOT REMOVE THIS LINE!

       input clk,      // High clock input
       input one_pps,  // One PPS signal
       input nreset,   // Reset
       output  one_pps_cont, // Continues 1PPS signal 
		 output clk_div,
		 
       input spi_clk,     // serial clock 
       input spi_sen,     // crystal enable for serial clk
       output spi_out,    // serial data out
		 input  spi_in,     // serial data in
       output  spi_out_oen // serial output enable);

);

clock_counter c(clk, one_pps, nreset, 
	one_pps_cont, clk_div, spi_clk, spi_sen, 
	spi_out, spi_in, spi_out_oen);

// {ALTERA_IO_BEGIN} DO NOT REMOVE THIS LINE!

// {ALTERA_IO_END} DO NOT REMOVE THIS LINE!
// {ALTERA_MODULE_BEGIN} DO NOT REMOVE THIS LINE!
// {ALTERA_MODULE_END} DO NOT REMOVE THIS LINE!
endmodule
