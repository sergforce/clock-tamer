// Test bench for clock_counter


module clock_counter_tb();
   parameter COUNTER_MAX      = 28;
    
   reg clk = 0;
   reg rst = 0;
   
   reg one_pps = 0;
   wire one_pps_cont;
   wire clk_div;
   
   // SPI LOGIC
   reg spi_clk = 0;
   reg spi_sen = 1;
   reg spi_in = 0;
   
   wire spi_out;
   wire spi_out_oen;

   // Initializing
   initial #10 rst = 1;
   always #5 clk = ~clk;
   
   
   clock_counter clock_counter (
       .clk(clk),
       .one_pps(one_pps),
       .nreset(rst),
       .one_pps_cont(one_pps_cont),
       .clk_div(clk_div),

       .spi_clk(spi_clk),
       .spi_sen(spi_sen),        // crystal enable for serial clk
       .spi_out(spi_out),        // serial data out
       .spi_in(spi_in),          // serial data in
       .spi_out_oen(spi_out_oen) // serial output enable
  );
  
  
  initial $dumpfile("clock_counter_tb.vcd");
  initial $dumpvars(0,clock_counter_tb);
  
  // Write SPI command
  task SPIWrite;
  input[COUNTER_MAX-1:0] spi_command;
  
  reg[COUNTER_MAX-1:0]   spi_outvalue;
  //reg   spi_invalue[27:0];
  begin
       spi_outvalue <= 0;
       
       #1;
       spi_sen <= 0;
       #1;
     
       repeat (28)
       begin
          spi_in <= 0;
          
          #22;
          spi_clk <= 1;
          
          #22;
          spi_clk <= 0;
          //spi_outvalue <= {spi_out, spi_outvalue};
          spi_outvalue[27:1] <= spi_outvalue[26:0];
          spi_outvalue[0] <= spi_out;
       end

       #1;
       spi_sen <= 1;
       #1;

  end
  endtask // InsertRead
  
   
  initial
  begin
    @(posedge rst);
    //#10;
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    
    SPIWrite(28'h0);
    
    #30;
    @(posedge clk);
    one_pps <= 1;
    @(posedge clk);
    one_pps <= 0;
    
    #300;
    
    SPIWrite(28'h0);
    
    
    #200 $finish;
  end

  

   
endmodule // clock_counter_tb