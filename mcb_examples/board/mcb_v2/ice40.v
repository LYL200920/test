// yosys -p 'synth_ice40' -b json -o ice40.json ice40.v
// nextpnr-ice40 --up5k --package sg48 --pcf ice40.pcf --pre-pack ice40_pnr.py --json ice40.json --asc ice40.pnr
// icepack ice40.pnr ice40.bin

module top (
  inout wire[15:0] ad16_d,
  input wire[4:1] ad16_a,
  inout wire exp_int,
  input wire ad16_cs,
  input wire ad16_wr,
  input wire ad16_bclk,

  inout wire dspi1_cs,
  inout wire dspi1_d1,
  inout wire dspi1_d0,
  inout wire dspi1_clk,

  inout wire dspi0_cs,
  inout wire dspi0_d1,
  inout wire dspi0_d0,
  inout wire dspi0_clk,

  output reg le_do_1,
  output reg le_do_0,
  output reg le_trg_out,
  output reg oe_di_1,
  output reg oe_di_0,
  output reg le_mcx_zu,
  output reg le_mcx_xy
);


assign exp_int = 1'bz;

assign ad16_d = 'bz;

assign dspi1_cs = 1'bz;
assign dspi1_d1 = 1'bz;
assign dspi1_d0 = 1'bz;
assign dspi1_clk = 1'bz;

assign dspi0_cs = 1'bz;
assign dspi0_d1 = 1'bz;
assign dspi0_d0 = 1'bz;
assign dspi0_clk = 1'bz;


// internal reset signal, released after 63 clocks
reg[5:0] reset_counter = 6'b000000;
reg reset = 1'b1;

always @(negedge ad16_bclk) begin
  if (reset_counter == 6'b111111)
    reset <= 1'b0;
  else begin
    reset <= 1'b1;
    reset_counter <= reset_counter + 1;
  end
end


wire le_trg_out_s = { ad16_a, 1'b0 } == 5'h00;
wire le_do_0_s    = { ad16_a, 1'b0 } == 5'h02;
wire le_do_1_s    = { ad16_a, 1'b0 } == 5'h04;
wire le_mcx_xy_s  = { ad16_a, 1'b0 } == 5'h06;
wire le_mcx_zu_s  = { ad16_a, 1'b0 } == 5'h08;
wire oe_di_0_s    = ~( { ad16_a, 1'b0 } == 5'h0A );
wire oe_di_1_s    = ~( { ad16_a, 1'b0 } == 5'h0C );

reg[3:0] state;

always @(negedge ad16_bclk, posedge reset) begin
  if (reset) begin
    le_trg_out <= 1'b0;
    le_do_0 <= 1'b0;
    le_do_1 <= 1'b0;
    le_mcx_xy <= 1'b0;
    le_mcx_zu <= 1'b0;
    oe_di_0 <= 1'b1;
    oe_di_1 <= 1'b1;
    state <= 4'b0000;
  end
  else begin
    case (state)
      4'b0000: begin
        if (ad16_cs == 0) begin
          le_trg_out <= le_trg_out_s;
          le_do_0    <= le_do_0_s;
          le_do_1    <= le_do_1_s;
          le_mcx_xy  <= le_mcx_xy_s;
          le_mcx_zu  <= le_mcx_zu_s;
          oe_di_0    <= oe_di_0_s;
          oe_di_1    <= oe_di_1_s;
          state <= 4'b0001;
        end
      end

      4'b0001: begin
        // hold outputs
        state <= 4'b0100;
      end

/*
      4'b0001: begin
        // hold outputs
        state <= 4'b0010;
      end

      4'b0010: begin
        // hold outputs
        state <= 4'b0100;
      end
*/
      4'b0100: begin
        // reset outputs
        le_trg_out <= 1'b0;
        le_do_0 <= 1'b0;
        le_do_1 <= 1'b0;
        le_mcx_xy <= 1'b0;
        le_mcx_zu <= 1'b0;
        oe_di_0 <= 1'b1;
        oe_di_1 <= 1'b1;
        state <= 4'b0000;
      end
    endcase
  end

end

endmodule
