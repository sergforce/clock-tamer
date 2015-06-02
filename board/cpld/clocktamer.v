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

	FO_CPLD,
	F_TCXO,
	FIN_CPLD,
	SPI_CE,
	SPI_SCLK,
	SPI_MOSI,
	SPI_MISO,
	ONEPPS_GPS,
	ONEPPS_CNT,
	NRESET,
	MODE_SELECT,
	ONEPPS_1,
	ONEPPS_2,
	ONEPPS_3,
	ONEPPS_4,
    ONEPPS_MODE
// {ALTERA_ARGS_END} DO NOT REMOVE THIS LINE!

);

// {ALTERA_IO_BEGIN} DO NOT REMOVE THIS LINE!
output			FO_CPLD;
input			F_TCXO;
input			FIN_CPLD;
input			SPI_CE;
input			SPI_SCLK;
input			SPI_MOSI;
output		SPI_MISO;
input			ONEPPS_GPS;
output		ONEPPS_CNT;
input			NRESET;
input			MODE_SELECT;
input			ONEPPS_1;
input			ONEPPS_2;
input			ONEPPS_3;
input			ONEPPS_4;
input           ONEPPS_MODE;

// {ALTERA_IO_END} DO NOT REMOVE THIS LINE!
// {ALTERA_MODULE_BEGIN} DO NOT REMOVE THIS LINE!

wire spi_oe;
wire spi_out;

ALT_OUTBUF_TRI spi_outbuf_tri (
   .i(spi_out),
	.oe(spi_oe),
   .o(SPI_MISO)); //out must be declared as an output pin
	
clock_counter c(
	.clk(FIN_CPLD),
	.one_pps(ONEPPS_GPS),
	.nreset(NRESET), 
	.pps_sync_mode(ONEPPS_MODE),
	.one_pps_cont(ONEPPS_CNT),
	.clk_div(FO_CPLD),
	.spi_clk(SPI_SCLK),
	.spi_sen(SPI_CE), 
	.spi_out(spi_out),
	.spi_in(SPI_MOSI),
	.spi_out_oen(spi_oe));

	
// {ALTERA_MODULE_END} DO NOT REMOVE THIS LINE!
endmodule

