
/*

GP output ports

                  | 144 pin | 176 pin
------------------+---------------------
LD4               |      P07
------------------+---------------------
LD3               |      P05
------------------+---------------------
LD2               |      P03
------------------+---------------------
LD1               |      P02
------------------+---------------------
EXP_COM_DE1       |      PA7
RS485 SCI6        |
------------------+---------------------
ETH2 orange LED   |      PC5
------------------+---------------------
trigger latch     |      P73
output enable     |
------------------+---------------------
EXP_COM_DE2       |      PJ5
RS485 SCI7/SCI10  |
------------------+---------------------
ETH2 green LED    |      P9
------------------+---------------------


GP input ports

                  | 144 pin | 176 pin
------------------+---------------------
DIPSW2            |      P35
------------------+---------------------
DIPSW1            |      P93
------------------+---------------------
DIPSW3            |      PC7
------------------+---------------------


interrupt inputs

                  | 144 pin | 176 pin
------------------+---------------------
IRQ0              |    --   |    P10
INT1N_MCX514      |         |
------------------+---------------------
IRQ1              |    not used
------------------+---------------------
IRQ2              |      P32
PCA9698           |
------------------+---------------------
IRQ3              |      P33
INT0N_MCX514      |
------------------+---------------------
IRQ4              |      PF5
LAN9250           |
------------------+---------------------
IRQ5              |   P15   |    --
INT1N_MCX514      |         |
------------------+---------------------
IRQ6              |    not used
------------------+---------------------
IRQ7              |      P17
INT_PCD4641       |
------------------+---------------------
IRQ[8-15]         |    trigger inputs


external bus CS areas

                  | 144 pin | 176 pin
------------------+---------------------
reserved          |     CS0
------------------+---------------------
MCX514 area B     |     CS1
------------------+---------------------
MCX514 area A     |     CS2
------------------+---------------------
PCD4641 area B    |     CS3
------------------+---------------------
PCD4641 area A    |     CS4
------------------+---------------------
LAN9250           |     CS5
------------------+---------------------
reserved          |     CS6
------------------+---------------------
trigger output    |     CS7, P73



------------------------------------------------------------------

before PERI_RST release, before changing OE_TRG_OUT IO direction on RX
  clear trigger output latch by writing 0 to trigger CS area
  set /OE_TRG_OUT IO value to 1 to enable the latch output.
  at reset, the latch output is disabled and the outputs are pulled low
  but latch contents are not reset to zero.

before MCX reset release:
  - setup timings for CS areas
  - setup clock output on MCX

after MCX reset release:
  - configure GPIO pins for each axis according to wiring of the board.

before PCD reset release:
  - setup timings for CS areas
  - setup clock output on PCD

after PCD reset release:
  - configure GPIO pins for each axis according to wiring of the board.

before using trigger input for LAN9250, set TRG_IN3_EN accordingly.

if using ethernet, enable the PHYs.  after power-on/reset they will be in
power-down mode.

if using CAN, select CAN_RS line accordingly.

make sure not to enable NMI, as the pin is used as an input pin.
NMI can only be enabled once and can't be disabled in software afterwards.
only reset can disable NMI again.  so take care.




*/


#include <cstdio>
#include <chrono>
#include <thread>

#include <board/board.hpp>
#include <board/board_info.hpp>

#include <utils/rodata.hpp>
#include <utils/text.hpp>

#include <dev/cpu.hpp>
#include <dev/rx_bsc.hpp>
#include <dev/rx_clk.hpp>
#include <dev/rx_sys.hpp>

// make sure that the board constructor is ran as the very first static
// initializer by specifying the init_priority attribute.
[[gnu::init_priority (101), gnu::used]] mcb_v13_board mcb_v13_board::g_inst;

[[gnu::cold]]
mcb_v13_board::reset_hw_init_t::reset_hw_init_t (mcb_v13_board& brd)
{
#ifndef MCB_NO_RESET_HW_INIT


  dev::rx_sys::disable_register_protection ();

  // the reset pin is pulled high externally.  before enabling the output
  // set the output state to high, too.
  set_peripheral_reset (true);

  // the trigger output latch "output enable" signal (active low) is also
  // pulled up high externally during reset.  initial value is off = set.
  // after trigger outputs have been initialized, it will be released.
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p7), 3> OE_TRG_OUT;
  OE_TRG_OUT.set ();

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

#error not implemented

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)

#error not implemented

#elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)


  // PODR
  // The B5 bit in PORT3.PDR is reserved, because the P35 pin is input only.
  // Data is not output from the corresponding pins even if these bits are set

  // PMR
  // However, bits that correspond to port m on the 176-pin product but do not
  // exist on a product with fewer pins (except for ports 54 to 56) are reserved.
  // When writing, write 0 (general I/O port) to these bits. For ports 54 to 56,
  // write 0 (general I/O port) to the bits corresponding to ports 54 to 56 on
  // the 176-pin product; write 0 (general I/O port) to the bit
  // corresponding to port 56 on the 100-pin product.
  // The bit corresponding to a pin that does not exist is reserved. A reserved
  // bit is always read as 0. The write value should always be 0.

  // ODR0
  // However, the output type of the port PE1 pin is specified by the combination
  // of b3 and b2.  Bits that correspond to port m on the 176-pin product but do
  // not exist on a product with fewer pins are reserved. When writing, write 0
  // (CMOS output) to these bits.

  // ODR1
  // Bits that correspond to port m on the 176-pin product but do not exist on
  // a product with fewer pins are reserved. When writing, write 0 (CMOS output)
  // to these bits.


  // clock source setup
  // on 176pin RX64 and RX71 always use the on-chip HOCO as the main MCU
  // clock source.  the external xtal is used only as a clock source for USBA.

  // dev::rx_gpio::pdr::p3 = 0b0000'0000;  default after reset
  dev::rx_gpio::pmr::p3 = 0b1100'0000;  // P36 = EXTAL, P37 = XTAL

  // MOFCR = 0b00000001; // 0x00 is default after reset.
  // MOSCWTCR = 0x0E; // 0x53 is default after reset. not sure which value is good enough.
  dev::rx_clk::mosccr = 0; // start main clock oscillator

  // wait for the main oscillator to become stable.
  while ((dev::rx_clk::oscovfsr.read () & 0b00001) == 0) { }

  // HOCO clock is not very precise.
  // because the MCU timers are used to generate the clocks for various
  // other devices, it's better to use an external crystal.

//  #define USE_HOCO_CLOCK
  #define USE_MAIN_OSC_CLOCK


  #ifdef USE_HOCO_CLOCK

    // set HOCO frequecy.
    static_assert (hoco_hz == 16'000'000 || hoco_hz == 18'000'000 || hoco_hz == 20'000'000, "");
    if (hoco_hz == 16'000'000)
      dev::rx_clk::hococr2 = 0b00;
    if (hoco_hz == 18'000'000)
      dev::rx_clk::hococr2 = 0b01;
    if (hoco_hz == 20'000'000)
      dev::rx_clk::hococr2 = 0b10;

    // start the HOCO clock
    dev::rx_clk::hococr = 0;

    // wait for HOCO clock to become stable.
    while ((dev::rx_clk::oscovfsr.read () & 0b01000) == 0) { }

    // the PLL multiplication factor is a 5.1 decimal fixed point number, where
    // x10.0 = 010011 = 19 (min)
    // x10.5 = 010110 = 20
    // ...
    // x16.0 = 011111 = 31
    // ...
    // x30.0 = 111011 = 59 (max)
    //

    // PLL division ratio for HOCO is x1

    constexpr unsigned int pll_mult = ((pll_hz * 2) / hoco_hz) - 1;
    static_assert (pll_mult >= 0b010011 && pll_mult <= 0b111011, "");
    static_assert (pll_hz >= 120'000'000 && pll_hz <= 240'000'000, "");
    static_assert ((hoco_hz * (pll_mult + 1)) / 2 == pll_hz, "");

    dev::rx_clk::pllcr = (pll_mult << 8) | 0b0001'0000;  // select HOCO as PLL clock source
  #endif


  #ifdef USE_MAIN_OSC_CLOCK

    // PLL division ratio for main oscillator is x1/2

    constexpr unsigned int pll_in_clk = xtal_hz / 2;

    constexpr unsigned int pll_mult = ((pll_hz * 2) / pll_in_clk) - 1;
    static_assert (pll_mult >= 0b010011 && pll_mult <= 0b111011, "");
    static_assert (pll_hz >= 120'000'000 && pll_hz <= 240'000'000, "");
    static_assert ((pll_in_clk * (pll_mult + 1)) / 2 == pll_hz, "");

    dev::rx_clk::pllcr = (pll_mult << 8) | 0b0000'0001;  // select main osc as PLL clock source
  #endif

  dev::rx_clk::pllcr2 = 0; // start PLL

  // wait for PLL to become stable.
  while ((dev::rx_clk::oscovfsr.read () & 0b00100) == 0) { }

  // on RX71 we need to insert one wait cycle if cpu speed is higher than 120 mhz.
  if (iclk_hz > 120'000'000)
    dev::rx_clk::memwait = 1;

  // select clock divisors
  constexpr auto pclkd_div = dev::rx_clk::sckcr_div (pll_hz / pclkd_hz);
  static_assert (pclkd_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto pclkc_div = dev::rx_clk::sckcr_div (pll_hz / pclkc_hz);
  static_assert (pclkc_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto pclkb_div = dev::rx_clk::sckcr_div (pll_hz / pclkb_hz);
  static_assert (pclkb_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto pclka_div = dev::rx_clk::sckcr_div (pll_hz / pclka_hz);
  static_assert (pclka_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto bclk_div = dev::rx_clk::sckcr_div (pll_hz / bclk_hz);
  static_assert (bclk_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto iclk_div = dev::rx_clk::sckcr_div (pll_hz / iclk_hz);
  static_assert (iclk_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto fclk_div = dev::rx_clk::sckcr_div (pll_hz / fclk_hz);
  static_assert (fclk_div != dev::rx_clk::sckcr_div_invalid (), "");

  dev::rx_clk::sckcr =
	    (pclkd_div << 0)
	  | (pclkc_div << 4)
	  | (pclkb_div << 8)
	  | (pclka_div << 12)
	  | (bclk_div << 16)
	  | (1 << 22) // disable SDCLK pin output
	  | (0 << 23) // enable BCLK pin output
	  | (iclk_div << 24)
	  | (fclk_div << 28);

  static_assert (iclk_hz >= bclk_hz, "");

  // UCLK select
  constexpr auto uck_div = dev::rx_clk::sckcr_uck_div_rx64 (pll_hz / uck_hz);
  static_assert (uck_div != dev::rx_clk::sckcr_uck_div_invalid (), "");

  dev::rx_clk::sckcr2 = 1 | (uck_div << 4);

  // BCLK pin output = 1/2 bclk
  dev::rx_clk::bckcr = 1;

  // use the PLL as the main system clock
  dev::rx_clk::sckcr3 = 0b100 << 8;

  // ETH2 LEDs are inverted, so make sure that they are in the off state
  // before switching the port to output mode.
  dev::rx_gpio::podr::p9 = 0xFF;
  dev::rx_gpio::podr::pc = 0xFF;

  // set output port directions
  // notice that after reset all ports are set to inputs (PDR = 0x00)
  dev::rx_gpio::pdr::p0 = 0b1010'1100;	// P02 = LD1, P03 = LD2, P05 = LD3, P07 = LD4
  dev::rx_gpio::pdr::p5 = 0b0101'0000;  // Write 1 (output) to bits that correspond to ports 54 to 56 on the 176-pin product.
  dev::rx_gpio::pdr::p7 = 0b0101'1001;  // P76 = TRG_IN3_EN, P74 = CAN_RS, P73 = /OE_TRG_OUT, P70 = /PERI_RST
  dev::rx_gpio::pdr::p9 = 0b1000'0000;  // P97 = ETH2 LED green

  // EXP_COM_DE1 does not work on PA because
  // PA is fixed as address A0-A7 outputs.
  // dev::rx_gpio::pdr::pa = 0b1000'0000;  // EXP_COM_DE1 (exclusive output port)

  dev::rx_gpio::pdr::pb = 0b1101'0000;  // PB7 = DAC_CS1N, PB6 = DAC_CS2N, PB4 = DAC_CS3N
  dev::rx_gpio::pdr::pc = 0b0011'0000;  // PC5 = ETH2 LED orange (exclusive output port)
					// PC4 = MTIOC3D (MCX clock B), but don't use clock
					//       doubler and always output 0
  dev::rx_gpio::pdr::pj = 0b0010'0000;  // PJ5 = EXP_COM_DE2 (exclusive output port)

  // use high-drive for LEDs
  dev::rx_gpio::dscr::p0 = 0b1010'1100;
  dev::rx_gpio::dscr::p9 = 0b1000'0000;
  dev::rx_gpio::dscr::pc = 0b0010'0000;

  // high-drive for bus WR and RD signals.
  dev::rx_gpio::dscr::p5 = 0b0000'0101;

  // select open drain output
  #if defined (MCB_USE_I2C)
  dev::rx_gpio::odr0::p1 = 0b01'01'00'00;  // P12 = SSCL2, P13 = SSDA2
  #endif

  // set pull-up resistors for serial ports which don't have external resistors.
  dev::rx_gpio::pcr::p0 = 0b0000'0011;  // TXD6, RXD6
  dev::rx_gpio::pcr::p8 = 0b1100'0000;  // TXD10, RXD10

  // select pinmux peripherals
  dev::rx_gpio::pfs::p00 = 0b00'001010;	// TXD6
  dev::rx_gpio::pfs::p01 = 0b00'001010;	// RXD6
  dev::rx_gpio::pmr::p0 = 0b0000'0011;

  dev::rx_gpio::pfs::p10 = 0b01'000000; // IRQ0 (MCX INT1)
  dev::rx_gpio::pfs::p11 = 0b00'010101; // USBA_VBUSEN
  dev::rx_gpio::pfs::p12 = 0b00'001010; // SSCL2
  dev::rx_gpio::pfs::p13 = 0b00'001010; // SSDA2
  dev::rx_gpio::pfs::p14 = 0b00'010000; // CTX1
  dev::rx_gpio::pfs::p15 = 0b00'010000; // CRX1
  dev::rx_gpio::pfs::p16 = 0b00'010001; // USB0_VBUS
  dev::rx_gpio::pfs::p17 = 0b01'000000; // IRQ7 (PCD)
  dev::rx_gpio::pmr::p1 = 0b0111'1110;

  dev::rx_gpio::pfs::p20 = 0b00'001010; // TXD0
  dev::rx_gpio::pfs::p21 = 0b00'001010; // RXD0
  dev::rx_gpio::pfs::p22 = 0b00'001010; // SCK0
  dev::rx_gpio::pfs::p23 = 0b00'001010; // TXD3
  dev::rx_gpio::pfs::p24 = 0b00'001010; // SCK3
  dev::rx_gpio::pfs::p25 = 0b00'001010; // RXD3
  dev::rx_gpio::pmr::p2 = 0b0011'1111;

  dev::rx_gpio::pfs::p32 = 0b01'000000; // IRQ2 (PCA)
  dev::rx_gpio::pfs::p33 = 0b01'000000; // IRQ3 (MCX INT0)

  dev::rx_gpio::pfs::p40 = 0b01'000000; // IRQ8, TRG_IN1
  dev::rx_gpio::pfs::p41 = 0b01'000000; // IRQ9, TRG_IN2
  dev::rx_gpio::pfs::p42 = 0b01'000000; // IRQ10, TRG_IN3
  dev::rx_gpio::pfs::p43 = 0b01'000000; // IRQ11, TRG_IN4
  dev::rx_gpio::pfs::p44 = 0b01'000000; // IRQ12, TRG_IN5
  dev::rx_gpio::pfs::p45 = 0b01'000000; // IRQ13, TRG_IN6
  dev::rx_gpio::pfs::p46 = 0b01'000000; // IRQ14, TRG_IN7
  dev::rx_gpio::pfs::p47 = 0b01'000000; // IRQ15, TRG_IN8

  dev::rx_gpio::pfs::p60 = 0b00'010101; // RMII1_TXD_EN
  dev::rx_gpio::pfs::p66 = 0b00'001000; // MTIOC7D (USB hub clock)
  dev::rx_gpio::pmr::p6 = 0b0100'0001;

// it's not needed to set MDIO to open-drain but it also doesn't harm.
// both works OK.
//  dev::rx_gpio::odr0::p7 = 0b00'00'01'00;  // P71 = ET0_MDIO
  dev::rx_gpio::pfs::p71 = 0b00'010001; // ET0_MDIO
  dev::rx_gpio::pfs::p72 = 0b00'010001; // ET0_MDC
  dev::rx_gpio::pfs::p75 = 0b00'010010; // RMII0_RXD0
  dev::rx_gpio::pfs::p77 = 0b00'010010; // RMII0_RX_ER
  dev::rx_gpio::pmr::p7 = 0b1010'0110;

  dev::rx_gpio::pfs::p80 = 0b00'010010; // RMII0_TXD_EN
  dev::rx_gpio::pfs::p81 = 0b00'010010; // RMII0_TXD0
  dev::rx_gpio::pfs::p82 = 0b00'010010; // RMII0_TXD1
  dev::rx_gpio::pfs::p83 = 0b00'010010; // RMII0_CRS_DV

  // RX71 manual is inconsistent about TXD10 on P87 and RXD10 on P86.
  // they are not listed in the PFS register description, but in the other
  // overview table.  the RX64 manual is more consistent about it.  just try ..
  dev::rx_gpio::pfs::p86 = 0b00'001010; // RXD10
  dev::rx_gpio::pfs::p87 = 0b00'001010; // TXD10
  dev::rx_gpio::pmr::p8 = 0b1100'1111;

  dev::rx_gpio::pfs::p92 = 0b00'010101; // RMII1_CRS_DV
  dev::rx_gpio::pfs::p94 = 0b00'010101; // RMII1_RXD0
  dev::rx_gpio::pfs::p95 = 0b00'010101; // RMII1_RXD1
  dev::rx_gpio::pmr::p9 = 0b0011'0100;

// can't use RTS5 on PA6.  in external bus mode, all PA pins are fixed
// address A0..A7 outputs.
//  dev::rx_gpio::pfs::pa6 = 0b00'001011; // RTS5
//  dev::rx_gpio::pmr::pa = 0b0100'0000;

  dev::rx_gpio::pfs::pb0 = 0b00'010010; // RMII0_RXD1
  dev::rx_gpio::pfs::pb1 = 0b00'001010; // TXD4/SMOSI4
  dev::rx_gpio::pfs::pb2 = 0b00'010010; // REF50CK0
  dev::rx_gpio::pfs::pb3 = 0b00'001010; // SCK4
  dev::rx_gpio::pfs::pb5 = 0b00'000001; // MTIOC2A (PCD clock)
  dev::rx_gpio::pmr::pb = 0b0010'1111;

  dev::rx_gpio::pfs::pc0 = 0b00'001011; // CTS5
  dev::rx_gpio::pfs::pc1 = 0b00'001010; // SCK5
  dev::rx_gpio::pfs::pc2 = 0b00'001010; // RXD5
  dev::rx_gpio::pfs::pc3 = 0b00'001010; // TXD5
//  dev::rx_gpio::pfs::pc4 = 0b00'000001; // MTIOC3D (MCX clock B, not used, fixed low output)
  dev::rx_gpio::pfs::pc6 = 0b00'000010; // MTCLKA (from LAN9250)
//  dev::rx_gpio::pmr::pc = 0b0101'1111;
  dev::rx_gpio::pmr::pc = 0b0100'1111;

  dev::rx_gpio::pfs::pf0 = 0b00'001010; // TXD1
  dev::rx_gpio::pfs::pf1 = 0b00'001010; // SCK1
  dev::rx_gpio::pfs::pf2 = 0b00'001010; // RXD1
  dev::rx_gpio::pfs::pf5 = 0b01'000000;	// IRQ4 (LAN9250)
  dev::rx_gpio::pmr::pf = 0b0000'0111;

  dev::rx_gpio::pfs::pg0 = 0b00'010101; // REF50CK1
  dev::rx_gpio::pfs::pg1 = 0b00'010101; // RMII1_RX_ER
  dev::rx_gpio::pfs::pg3 = 0b00'010101; // RMII1_TXD0
  dev::rx_gpio::pfs::pg4 = 0b00'010101; // RMII1_TXD1
  dev::rx_gpio::pmr::pg = 0b0001'1011;

//  #if defined (MCB_USE_MCX514)
  dev::rx_gpio::pfs::pj3 = 0b00'000001;	// MTIOC3C (MCX clock A)
  dev::rx_gpio::pmr::pj = 0b0000'1000;
//  #endif


#endif // MCU type


  dev::rx_sys::enable_register_protection ();

  // enable MTU module to allow outputting clock signals before PERI_RESET
  // is deasserted.
  dev::rx_mstpcra_bit<9> () (true);


  #if defined (MCB_USE_PCD4641) || defined (MCB_USE_MCX514) \
      || defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW) \
      || defined (MCB_USE_TRIGGER_IO)

    // enable external bus, if needed.

    // dev::rx_gpio::pfcss0 = 0b00'00'00'00; // CS1 = P61, CS2 = P62, CS3 = P63
    // dev::rx_gpio::pfcss1 = 0b00'00'00'00; // CS4 = P64, CS5 = P65, CS7 = P67

    // enable CS outputs
    dev::rx_gpio::pfcse = 0b10111110;

    // enable A17 output (P91)
    dev::rx_gpio::pfaoe1 = 0b00000010;

    // enable A0-A7 outputs on PA0-PA7
    // enable A16-A23 outputs on P90-P97
    // enable D8-D15 outputs on PE0-PE7
    dev::rx_gpio::pfbcr0 = 0b0001'0'01'1;

    // P51 = WAIT
    dev::rx_gpio::pfbcr1 = 0b000000'11;
  #endif


  // PCD4641 clock external bus setup
  // MTIOC2A = clock output
  // use CS4 area 0x04000000 - 0x04FFFFFF for register access
  // use CS3 area 0x05000000 - 0x05FFFFFF for command write
  #if defined (MCB_USE_PCD4641)
  {
    // byte strobe mode, external wait enable, page read/write off,
    // normal access mode
    // the PCD will insert additional bus wait cycles as needed.
    dev::rx_bsc::cs4mod = dev::rx_bsc::cs3mod = 0b0'00000'0'0'0000'1'00'0;

    // PCD4641A needs to see a CS-deassert for at least 10 ns between
    // subsequent accesses.
    // because of some inconsistencies with read and write cycles and cson
    // timings, use recovery cycles.
    // theoretically, it should be possible to get exactly the same timings
    // by setting cson to 10 ns and additing it to the cswwait and csrwait
    // times.  but then in the read cycle the cson time is longer than specified..
    constexpr unsigned int cson = 0;
    constexpr unsigned int rd_recovery = ns_to_bclk (10);
    constexpr unsigned int wr_recovery = ns_to_bclk (10);

    // write cycle
    //    16 ns of WR & CS assert
    //    10 ns between accesses
    // the datasheet seems wrong about the TWT (WRQ ON delay time).
    // the PCD asserts the WRQ line 26 ns after WR, not 16.
    // 96 mhz bclk: 24 mhz access speed, 120 mhz blck: 20 mhz access speed.
    constexpr unsigned int wron = 0;
    constexpr unsigned int wdon = 0;
    constexpr unsigned int wdoff = 0;
    constexpr unsigned int cswoff = 0;
    constexpr unsigned int cswwait = ns_to_bclk (26) - 1;

    // read cycle
    //    32 ns of RD & CS assert (but it seems 31.25 ns is also OK).
    //    10 ns between accesses
    // 96 mhz bclk: 24 mhz access speed, 120 mhz bclk: 20 mhz access speed.
    constexpr unsigned int rdon = 0;
    constexpr unsigned int csrwait = ns_to_bclk (31) - 1;
    constexpr unsigned int csroff = 0;

    static_assert (csrwait <= 31, "");
    static_assert (cswwait <= 31, "");
    static_assert (csroff <= 7, "");
    static_assert (cswoff <= 7, "");
    static_assert (cson <= 7, "");
    static_assert (rdon <= 7, "");
    static_assert (wron <= 7, "");
    static_assert (wdon <= 7, "");

    dev::rx_bsc::cs4wcr1 =
	(csrwait << 24) | (cswwait << 16);

    dev::rx_bsc::cs4wcr2 =
	csroff | (cswoff << 4) | (cson << 28) | (rdon << 16) | (wron << 20) | (wdon << 24) | (wdoff << 8);

    // recovery cycles for normal register access
    static_assert (rd_recovery <= 15, "");
    static_assert (wr_recovery <= 15, "");

    dev::rx_bsc::cs4rec = (wr_recovery << 8) | (rd_recovery << 0);


    // the PCD needs additional 2 wait cycles after drive start command
    // writes.  otherwise drive start command might be missed and not
    // executed.
    // because the PCD is much slower than the bus interface, the max. 15
    // recovery cycles are not sufficient.  instead the write cycles
    // are stretched by inserting delays in other places (cswoff delay).
    // still, that is not enough.  because of that the PCD bus interface
    // will actually do two consecutive 8 bit writes into the command write
    // CS area.

    static_assert (bclk_hz > pcd4641_clock_hz, "");

    // 1.3125 PCD cycles
    // when measuring with the scope, remember that the PCD will insert
    // wait cycles.
    constexpr unsigned int cmd_wr_write_wait_bclk =
	(((131LL * bclk_hz) / pcd4641_clock_hz) + 99) / 100;

    constexpr unsigned int cmd_cswoff = std::min (cmd_wr_write_wait_bclk, 7u);
    constexpr unsigned int cmd_wr_recovery = cmd_wr_write_wait_bclk - cmd_cswoff;

    static_assert (cmd_cswoff == 7, "");
    static_assert (cmd_wr_recovery == 14, "");

    constexpr unsigned int cmd_cson = 0;

    constexpr unsigned int cmd_wron = cmd_cson;
    constexpr unsigned int cmd_wdon = 0;
    constexpr unsigned int cmd_wdoff = 0;
    constexpr unsigned int cmd_cswwait = ns_to_bclk (26) - 1 + cmd_cson;

    constexpr unsigned int cmd_rdon = cmd_cson;
    constexpr unsigned int cmd_csrwait = ns_to_bclk (31) - 1 + cmd_cson;
    constexpr unsigned int cmd_csroff = 0;

    static_assert (cmd_csrwait <= 31, "");
    static_assert (cmd_cswwait <= 31, "");
    static_assert (cmd_csroff <= 7, "");
    static_assert (cmd_cswoff <= 7, "");
    static_assert (cmd_cson <= 7, "");
    static_assert (cmd_rdon <= 7, "");
    static_assert (cmd_wron <= 7, "");
    static_assert (cmd_wdon <= 7, "");

    dev::rx_bsc::cs3wcr1 = dev::rx_bsc::cs3wcr1 =
	(cmd_csrwait << 24) | (cmd_cswwait << 16);

    dev::rx_bsc::cs3wcr2 =
	csroff | (cmd_cswoff << 4) | (cmd_cson << 28) | (cmd_rdon << 16) | (cmd_wron << 20) | (cmd_wdon << 24) | (cmd_wdoff << 8);

    // recovery cycles for command write access
    static_assert (cmd_wr_recovery <= 15, "");

    dev::rx_bsc::cs3rec = (cmd_wr_recovery << 8) | (cmd_wr_recovery << 0);

    // enable operation, 8 bit bus, explicit big endian, MPX disable.
    dev::rx_bsc::cs4cr = dev::rx_bsc::cs3cr =
	0b000'0'000'0'00'10'000'1
	| (utils::native_byte_order () != utils::big_endian ? (1 << 8) : 0);

    static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1400>> mtu2_tcr;
    static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1402>> mtu2_tior;
    static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0xC1408>> mtu2_tgra;
    static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1280>> mtu_tstra;

    // count at rising edge, TCNT clear on TGRA match, internal clock PCLK/1
    mtu2_tcr = 0b001'00'000;

    // MTIOC2B: no output, MTIOC2A: initial output is 0, toggle output at compare match.
    mtu2_tior = 0b0000'0011;

    static_assert (mtu_tick_count_to_hz (hz_to_mtu_tick_count (pcd4641_clock_hz))
		   == pcd4641_clock_hz, "");

    mtu2_tgra = hz_to_mtu_tick_count (pcd4641_clock_hz);

    // start MTU2 counter
    mtu_tstra |= 0b00000100;
  }
  #endif


  // MCX514 clock and external bus setup
  // MTIOC3C = clock output
  // use CS2 area 0x06000000 - 0x06FFFFFF for register access
  // use CS1 area 0x07000000 - 0x07FFFFFF for command write
  #if defined (MCB_USE_MCX514)
  {
    // single write strobe mode, no external wait, page read/write off,
    // normal access mode
    dev::rx_bsc::cs2mod = dev::rx_bsc::cs1mod = 0b0'00000'0'0'0000'0'00'1;

    // the MCX needs to see a CS de-assertion between the accesses.
    // for some combinations of the wait values, CS is never de-asserted, which
    // is a problem.  so better to check with a scope.
    // notice that the timing granularity is in 1000/96M = 10.42 ns
    // (internal bus clock).  values are rounded up to multiples of that.
    // so a 4ns delay will actually be 10.42 ns.

    // writes to the command area (CS1) will insert a 2 MCX clock wait cycle.
    // 20 MHz MCX clock: 100 ns wait
    // 16 MHz MCX clock: 125 ns wait
    // 12 MHz MCX clock: 166 ns wait

    // max. 15 recovery cycles @  96 MHz BCLK = 156 ns
    // max. 15 recovery cycles @ 120 MHz BCLK = 125 ns

    // 2x 20 MHz clocks = 10x 96 MHz clocks
    // 2x 12 MHz clocks = 16x 96 MHz clocks

    // use recovery cycles as much as possible.   however, in case of 12 MHz MCX
    // clock 15 recovery cycles might be not enough.  in this case, stretch the
    // write access by one cycle by adjusting the 'cswoff' time.

    static_assert (bclk_hz > mcx514_clock_hz, "");

    constexpr unsigned int cmd_wr_write_wait_bclk =
	(((2LL * 100 * bclk_hz) / mcx514_clock_hz) + 99) / 100 - 1;

    constexpr unsigned int cmd_wr_wait_recovery_bclk =
	std::min (cmd_wr_write_wait_bclk, 15u);

    constexpr unsigned int cmd_cswoff =
	cmd_wr_write_wait_bclk - cmd_wr_wait_recovery_bclk;

    // read cycle
    // continuous register read = 24 MHz (96 MHz blck), 20 MHz (120 MHz bclk)
    constexpr unsigned int rdon = ns_to_bclk (4);
    constexpr unsigned int csrwait = ns_to_bclk (20);
    constexpr unsigned int csroff = ns_to_bclk (5);

    // write cycle
    // continuous register write = 24 MHz.
    constexpr unsigned int wron = ns_to_bclk (4);
    constexpr unsigned int cswwait = ns_to_bclk (30);
    constexpr unsigned int cswoff = ns_to_bclk (0);

    static_assert (csrwait <= 31, "");
    static_assert (cswwait <= 31, "");

    dev::rx_bsc::cs2wcr1 = dev::rx_bsc::cs1wcr1 =
	(csrwait << 24) | (cswwait << 16);

    constexpr unsigned int cson = ns_to_bclk (4);

    static_assert (csroff <= 7, "");
    static_assert (cswoff <= 7, "");
    static_assert (cson <= 7, "");
    static_assert (cmd_cswoff <= 7, "");

    dev::rx_bsc::cs2wcr2 =
	csroff | (cswoff << 4) | (cson << 28) | (rdon << 16) | (wron << 20);

    dev::rx_bsc::cs1wcr2 =
	csroff | (cmd_cswoff << 4) | (cson << 28) | (rdon << 16) | (wron << 20);

    // no recovery cycles for normal register access
    dev::rx_bsc::cs2rec = 0;

    // recovery cycles for command write access
    static_assert (cmd_wr_wait_recovery_bclk <= 15, "");

    dev::rx_bsc::cs1rec =
	cmd_wr_wait_recovery_bclk | (cmd_wr_wait_recovery_bclk << 8);

    // enable operation, 16 bit bus, explicit little endian, MPX disable.
    dev::rx_bsc::cs2cr = dev::rx_bsc::cs1cr =
	0b000'0'000'0'00'00'000'1
	| (utils::native_byte_order () != utils::little_endian ? (1 << 8) : 0);


    static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1200>> mtu3_tcr;
    static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1204>> mtu3_tiorh;
    static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1205>> mtu3_tiorl;
    static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0xC1224>> mtu3_tgrc;
    static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1280>> mtu_tstra;

    // count at rising edge, TCNT clear on TGRC match, internal clock PCLK/1
    mtu3_tcr = 0b101'00'000;

    // MTIOC3B: no output, MTIOC3A: no output
    mtu3_tiorh = 0b0000'0000;

    // MTIOC3D: no output, MTIOC3C: initial output is 0, toggle output at compare match.
    mtu3_tiorl = 0b0000'0011;

    static_assert (mtu_tick_count_to_hz (hz_to_mtu_tick_count (mcx514_clock_hz))
		   == mcx514_clock_hz, "");

    mtu3_tgrc = hz_to_mtu_tick_count (mcx514_clock_hz);

    // start MTU3 counter
    mtu_tstra |= 0b01000000;
  }
  #endif


  // LAN9250 external bus setup
  // use CS5 area
  // 0x03000000 - 0x0300001F register access
  // 0x03020000 - 0x0302FFFF fifo access
  #if defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW)
  {
    dev::rx_bsc::cs5mod = 0b0'00000'0'0'0000'0'00'1;

    // read cycle (t_rdcyc)             = 45 ns = 22 mhz
    // gap between read cycles (t_rdrd) = 13 ns
    // rd pulse width (t_rd)            = 32 ns

    // 120 mhz bus clock: t_rd = 32 ns, t_rdrd = 16 ns, t_rdcyc = 48 ns = 20.8 mhz
    //  96 mhz bus clock: t_rd = 32 ns, t_rdrd = 11 ns, t_rdcyc = 42 ns = 23.8 mhz

    // write cycle is the same as the read cycle.

    constexpr unsigned int csrwait = ns_to_bclk (30) - 1;
    constexpr unsigned int csroff = ns_to_bclk (10);

    constexpr unsigned int cswwait = csrwait;
    constexpr unsigned int cswoff = csroff;

    constexpr unsigned int wdoff = 0;
    constexpr unsigned int wdon = 0;
    constexpr unsigned int wron = 0;
    constexpr unsigned int cson = 0;

    static_assert (csrwait <= 31, "");
    static_assert (cswwait <= 31, "");
    dev::rx_bsc::cs5wcr1 = (cswwait << 16) | (csrwait << 24);

    static_assert (csroff <= 7, "");
    static_assert (cswoff <= 7, "");
    static_assert (wdon <= 7, "");
    static_assert (wron <= 7, "");
    static_assert (wdoff <= 7, "");
    static_assert (cson <= 7, "");
    dev::rx_bsc::cs5wcr2 = (wdon << 24) | (wdoff << 8) | (cson << 28) | (wron << 20)
			   | (csroff << 0) | (cswoff << 4);

    dev::rx_bsc::cs5rec = 0;

    // enable operation, 16 bit bus, explicit little endian, MPX disable.
    dev::rx_bsc::cs5cr =
	0b000'0'000'0'00'00'000'1
	| (utils::native_byte_order () != utils::little_endian ? (1 << 8) : 0);
  }
  #endif


  // before PERI_RST release, before changing OE_TRG_OUT IO direction on RX
  // clear trigger output latch by writing 0 to trigger CS area
  // set /OE_TRG_OUT IO value to 1 to enable the latch output.
  // at reset, the latch output is disabled and the outputs are pulled low
  // but latch contents are not reset to zero.
  // FIXME: maybe use trigger_outputs driver class somehow here instead of
  // copy-pasta.
  #if defined (MCB_USE_TRIGGER_IO)
  {
    // byte strobe mode, external write disable, page read disable,
    // page write disable, normal access compatible mode.
    // dev::rx_bsc::cs7mod = 0b0'00000'0'0'0000'0'00'0;

    // the latch is pretty fast.  it seems to be ok to set all wait times
    // to the minimum values.

    constexpr unsigned int wrrd_cs_assert = 0;

    static_assert (wrrd_cs_assert <= 31, "");
    dev::rx_bsc::cs7wcr1 = (wrrd_cs_assert << 16) | (wrrd_cs_assert << 24);

    constexpr unsigned int wdoff = 0;
    constexpr unsigned int wdon = 0;
    constexpr unsigned int wron = 0;
    constexpr unsigned int cson = 0;

    static_assert (wdon <= 7, "");
    static_assert (wron <= 7, "");
    static_assert (wdoff <= 7, "");
    static_assert (cson <= 7, "");
    dev::rx_bsc::cs7wcr2 = (wdon << 24) | (wdoff << 8) | (cson << 28) | (wron << 20);

    dev::rx_bsc::cs7rec = 0;

    // enable external CS area, 8 bit bus, no endian swap.
    dev::rx_bsc::cs7cr = 0b000'0'000'0'00'10'000'1;
  }
  #endif


  // on-chip ROM enable, external bus enable.
  // enable CS recovery cycles.
  #if defined (MCB_USE_PCD4641) || defined (MCB_USE_MCX514) \
      || defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW) \
      || defined (MCB_USE_TRIGGER_IO)
    dev::rx_bsc::csrecen = 0xFFFF;
    dev::rx_sys::syscr0 = 0x5A00 | 0b00000011;
  #endif


  #if defined (MCB_USE_TRIGGER_IO)
    // clear the latch contents before enabling the outputs
    *(volatile uint8_t*)0x01000000 = 0x00;

     // enable the latch outputs (active low output)
     OE_TRG_OUT.clear ();
  #endif

  // start 12 Mhz clock output on MTIOC7D for USB HUB
  #if defined (MCB_USE_USB_HOST)
  {
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1A01>> mtu7_tcr;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1A06>> mtu7_tiorh;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1A07>> mtu7_tiorl;
    static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0xC1A2A>> mtu7_tgrd;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1A80>> mtu_tstrb;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1A0A>> mtu_toerb;

    // count at rising edge, TCNT clear on TGRD match, internal clock PCLK/1
    mtu7_tcr = 0b110'00'000;

    // MTIOC7B: no output, MTIOC7A: no output
    mtu7_tiorh = 0b0000'0000;

    // MTIOC7D: initial output is 0, toggle output at compare match, MTIOC7C: no output
    mtu7_tiorl = 0b0011'0000;

    // some MTU output pins are gated by the timer output mater enable register.
    // MTIOC7D is such an output pin.
    mtu_toerb |= (1 << 5);

    constexpr unsigned int usb_hub_clock_hz = 12'000'000;

    static_assert (mtu_tick_count_to_hz (hz_to_mtu_tick_count (usb_hub_clock_hz))
		   == usb_hub_clock_hz, "");

    mtu7_tgrd = hz_to_mtu_tick_count (usb_hub_clock_hz);

    // start MTU7 counter
    mtu_tstrb |= 0b10000000;
  }
  #endif

  // enable interrupts after IO config.
  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_be0>::enable (dev::interrupt::any_edge, dev::interrupt::priority_6);
  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_bl0>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);
  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_bl1>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);
  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_al0>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);
  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_al1>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);

  __builtin_rx_setpsw ('I');

#endif // MCB_NO_RESET_HW_INIT
}

[[gnu::cold]]
mcb_v13_board::release_peripheral_reset_t
::release_peripheral_reset_t (mcb_v13_board& brd)
{
#ifndef MCB_NO_PERIPHERAL_RESET_RELEASE

  set_peripheral_reset (false);

#endif // MCB_NO_PERIPHERAL_RESET_RELEASE
}

[[gnu::cold]]
const std::array<uint8_t, 6>
mcb_v13_board::board_info_eth0_addr::operator () (void)
{
  auto&& addr = mcb_v13_board_info::inst ().ifconfigs ()[0].hw_addr ();
  return (const std::array<uint8_t, 6>&)addr;
}

[[gnu::cold]]
const std::array<uint8_t, 6>
mcb_v13_board::board_info_eth1_addr::operator () (void)
{
  auto&& addr = mcb_v13_board_info::inst ().ifconfigs ()[1].hw_addr ();
  return (const std::array<uint8_t, 6>&)addr;
}

[[gnu::cold]]
const std::array<uint8_t, 6>
mcb_v13_board::board_info_eth2_addr::operator () (void)
{
  auto&& addr = mcb_v13_board_info::inst ().ifconfigs ()[2].hw_addr ();
  return (const std::array<uint8_t, 6>&)addr;
}




mcb_v13_board::mcb_v13_board (void)
 : devices_begin ()

 , reset_hw_init (*this)
 , system_timer ()
 , release_peripheral_reset (*this)

#if defined (MCB_USE_LEDS)
  , led_outputs ( { 0, 0, 0, 0 } )
#endif

#if defined (MCB_USE_PCA9698)
 , pca9698_i0 (internal_i2c)
 , pca9698_i1 (internal_i2c)
#endif

#if defined (MCB_USE_DAC124S085)
 , dac0 (internal_spi)
 , dac1 (internal_spi)
 , dac2 (internal_spi)
#endif

#if defined (MCB_USE_PCD4641)
 , pcd4641 (&pcd4641_axis_inputs, &pcd4641_axis_outputs)
 , pcd4641_axis_inputs {{
	pcd4641.axis (0), pcd4641.axis (1), pcd4641.axis (2), pcd4641.axis (3) }}
 , pcd4641_axis_outputs {{
	pcd4641.axis (0), pcd4641.axis (1), pcd4641.axis (2), pcd4641.axis (3) }}
#endif

#if defined (MCB_USE_MCX514)
 , mcx51x (&mcx51x_axis_inputs, &mcx51x_axis_outputs)
 , mcx51x_axis_inputs {{
	mcx51x.axis (0), mcx51x.axis (1), mcx51x.axis (2), mcx51x.axis (3) }}
 , mcx51x_axis_outputs {{
	{ mcx51x.axis (0), pca9698_i0, pca9698_i1, 0 },
	{ mcx51x.axis (1), pca9698_i0, pca9698_i1, 1 },
	{ mcx51x.axis (2), pca9698_i0, pca9698_i1, 2 },
	{ mcx51x.axis (3), pca9698_i0, pca9698_i1, 3 } }}
#endif

#if defined (MCB_USE_FCU)
  // fcu expects clock in MHz (rounding up).
 , fcu ((fclk_hz + 1'000'000-1) / 1'000'000)
#endif


#if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC0_RAW)
 , eth0_mac (eth0_etherc)
 , eth0_phy (mdio_sta)
#endif

#if defined (MCB_USE_RX_ETHERC0) && !defined (MCB_USE_RX_ETHERC0_RAW)
 , eth0 (eth0_mac, eth0_phy)
#endif

#if defined (MCB_USE_RX_ETHERC1) || defined (MCB_USE_RX_ETHERC1_RAW)
 , eth1_mac (eth1_etherc)
 , eth1_phy (mdio_sta)
#endif

#if defined (MCB_USE_RX_ETHERC1) && !defined (MCB_USE_RX_ETHERC1_RAW)
 , eth1 (eth1_mac, eth1_phy)
#endif

#if defined (MCB_USE_LAN9250)
 , eth2_phy (eth2_mac)
#endif

#if defined (MCB_USE_LAN9250) && !defined (MCB_USE_LAN9250_RAW)
 , eth2 (eth2_mac, eth2_phy)
#endif

#if defined (MCB_USE_DIGITAL_IO)
 , digital_inputs (pca9698_i0)
 , digital_outputs (pca9698_i1)
#endif

 , devices_end ()
{

#if defined (MCB_USE_FCU)
  static_assert ((fclk_hz + 1'000'000-1) / 1'000'000 >= 4, "");
  static_assert ((fclk_hz + 1'000'000-1) / 1'000'000 <= 60, "");
#endif


#if defined (MCB_USE_PCA9698)

/*
pca9698_i0
bank 0
  EXP1_00 .. EXP1_07 = DI00 .. DI07

bank 1
  EXP1_10 .. EXP1_17 = DI08 .. DI15

bank 2
  EXP1_20 .. EXP1_27 = DI16 .. DI23

bank 3
  EXP1_30 .. EXP1_37 = DI24 .. DI31

bank 4
  EXP1_40 = XOUT5
  EXP1_41 = XOUT6
  EXP1_42 = XOUT7
  EXP1_43 = XOUT8

  EXP1_44 = YOUT5
  EXP1_45 = YOUT6
  EXP1_46 = YOUT7
  EXP1_47 = YOUT8
*/

  pca9698_i0.set_output_config (pca9698_i0_t::output_config_t ()
	.set_port_0_open_drain (false)
	.set_port_1_open_drain (false)
	.set_port_2_open_drain (false)
	.set_port_3_open_drain (false)
	.set_port_4_open_drain (false));
  pca9698_i0.set_io_ports_config (0b00000000'11111111'11111111'11111111'11111111LL);


/*
pca9698_i1
bank 0
  EXP2_00 .. EXP2_07 = DO00 .. DO07

bank 1
  EXP2_10 .. EXP2_17 = DO08 .. DO15

bank 2
  EXP2_20 .. EXP2_27 = DO16 .. DO23

bank 3
  EXP2_30 .. EXP2_37 = DO24 .. DO31

bank 4
  EXP2_40 = ZOUT5
  EXP2_41 = ZOUT6
  EXP2_42 = ZOUT7
  EXP2_43 = ZOUT8

  EXP2_44 = UOUT5
  EXP2_45 = UOUT6
  EXP2_46 = UOUT7
  EXP2_47 = UOUT8
*/

  pca9698_i1.set_output_config (pca9698_i1_t::output_config_t ()
	.set_port_0_open_drain (false)
	.set_port_1_open_drain (false)
	.set_port_2_open_drain (false)
	.set_port_3_open_drain (false)
	.set_port_4_open_drain (false));
  pca9698_i1.set_io_ports_config (0b00000000'00000000'00000000'00000000'00000000LL);

#endif

#if defined (MCB_USE_PCD4641)

// PCD4641 OTS[XYZU]: axis output (amp enable)
// PCD4641 P1[XYZU]: axis output (default after reset)
// PCD4641 P2[XYZU]: axis output (default after reset)
// PCD4641 P3[XYZU]: axis output (default after reset)
// PCD4641 P4[XYZU]: axis output (default after reset)

// PCD4641 STP[XYZU]: axis input (amp alarm)
// PCD4641 U/B[XYZU]: axis input
// PCD4641 F/H[XYZU]: axis input
// PCD4641 STA[XYZU]: axis input
// PCD4641 +SD[XYZU]: axis input
// PCD4641 -SD[XYZU]: axis input

// this uses the chip's default configuration, so there's nothing to do here.

#endif

#if defined (MCB_USE_MCX514)

// MCX514 [XYZU]PIO0: [XYZU]IN5
// MCX514 [XYZU]PIO1: [XYZU]IN6
// MCX514 [XYZU]PIO2: [XYZU]OUT2
// MCX514 [XYZU]PIO3: [XYZU]OUT4
// MCX514 [XYZU]PIO4: [XYZU]IN4
// MCX514 [XYZU]PIO5: [XYZU]IN7
// MCX514 [XYZU]PIO6: [XYZU]OUT1
// MCX514 [XYZU]PIO7: [XYZU]OUT3

// MCX514 PIN0: XIN3
// MCX514 PIN1: YIN3
// MCX514 PIN2: ZIN3
// MCX514 PIN3: UIN3

  for (auto& a : mcx51x.axes ())
    a.set_pio1 (dev::mcx51x::pio_mode1_t ()
		.set_pin (0, dev::mcx51x::general_input)
		.set_pin (1, dev::mcx51x::general_input)
		.set_pin (2, dev::mcx51x::general_output)
		.set_pin (3, dev::mcx51x::general_output)
		.set_pin (4, dev::mcx51x::general_input)
		.set_pin (5, dev::mcx51x::general_input)
		.set_pin (6, dev::mcx51x::general_output)
		.set_pin (7, dev::mcx51x::general_output));

#endif


// the LAN8720 PHYs are reset into power-down mode.
// before we try doing anything with the MACs, need to bring up and
// configure the PHYs first.
// it's a bit odd, but for some reason, if first ETH0 PHY+MAC is brought
// up, then ETH1 PHY will not respond.  have to do ETH0 PHY then ETH1 PHY
// then the MACs.
#if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC0_RAW)

  eth0_phy.set_special_modes (decltype (eth0_phy)::special_modes_t ()
	.set_phy_addr (eth0_phy.mmd_address ())
	.set_mode (decltype (eth0_phy)::all_capable_auto)
	.set_mii_mode (decltype (eth0_phy)::rmii));

  eth0_phy.reset ();

#endif

#if defined (MCB_USE_RX_ETHERC1) || defined (MCB_USE_RX_ETHERC1_RAW)

  eth1_phy.set_special_modes (decltype (eth1_phy)::special_modes_t ()
	.set_phy_addr (eth1_phy.mmd_address ())
	.set_mode (decltype (eth1_phy)::all_capable_auto)
	.set_mii_mode (decltype (eth1_phy)::rmii));

  eth1_phy.reset ();

#endif

#if defined (MCB_USE_RX_ETHERC0) && !defined (MCB_USE_RX_ETHERC0_RAW)

  eth0.set_mac_address (eth0.default_mac_address ());
  eth0.reset ();

#endif

#if defined (MCB_USE_RX_ETHERC1) && !defined (MCB_USE_RX_ETHERC1_RAW)

  eth1.set_mac_address (eth1.default_mac_address ());
  eth1.reset ();

#endif

#if defined (MCB_USE_LAN9250) && !defined (MCB_USE_LAN9250_RAW)

  eth2.set_mac_address (eth2.default_mac_address ());
  eth2.reset ();

#endif
}

void
mcb_v13_board::set_peripheral_reset (bool val)
{
  // peripheral reset line is on P70 for all board variants.
  // output high (default at power-on): reset on (PERI_RST low)
  // output low: reset off (PERI_RST high)
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p7), 0> output;

  if (val == true)
    output.set ();
  else
    output.clear ();
}

using reset_source_1 = reset_source;

[[gnu::cold]] reset_source_1
mcb_v13_board::reset_source (void)
{
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C290>> RSTSR0 = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C291>> RSTSR1 = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x000800C0>> RSTSR2 = { };

  const uint8_t RSTSR2_val = RSTSR2;
  const uint8_t RSTSR0_val = RSTSR0;

  enum
  {
    RSTSR2_SWRF = 1 << 2,
    RSTSR2_WDTRF = 1 << 1,
    RSTSR2_IWDTRF = 1 << 0,

    RSTSR1_CWSF = 1 << 0,

    RSTSR0_PORF = 1 << 0,
    RSTSR0_LVD0RF = 1 << 1,
    RSTSR0_LVD1RF = 1 << 2,
    RSTSR0_LVD2RF = 1 << 3,
    RSTSR0_DPSRSTF = 1 << 7
  };

  if (RSTSR2_val & RSTSR2_SWRF)
    return reset_source_1::soft_reset;

  if (RSTSR0_val & RSTSR0_DPSRSTF)
    return reset_source_1::soft_standby;

  if (RSTSR0_val & RSTSR0_LVD2RF)
    return reset_source_1::vdet2;

  if (RSTSR0_val & RSTSR0_LVD1RF)
    return reset_source_1::vdet1;

  if (RSTSR2_val & RSTSR2_WDTRF)
    return reset_source_1::watchdog_timer;

  if (RSTSR2_val & RSTSR2_IWDTRF)
    return reset_source_1::independent_watchdog_timer;

  if (RSTSR0_val & RSTSR0_LVD0RF)
    return reset_source_1::vdet0;

  if (RSTSR0_val & RSTSR0_PORF)
    return reset_source_1::poweron;

  return reset_source_1::hard_reset;
}

// on RX on-chip RAM starts at address 0 and the first 256 bytes are reserved
// by the linker script.
// if the RX memory protection is ever to be used for nullptr access detection,
// have to disable it when accessing the variable.

int mcb_v13_board::reset_counter (void)
{
  return *(volatile int*)32;
}

void mcb_v13_board::set_reset_counter (int val)
{
  *(volatile int*)32 = val;
}

// -----------------------------------------------------------------------------

void mcb_v13_board::exec (void)
{
  auto cur_time = std::chrono::high_resolution_clock::now ();

  // auto delta_time = cur_time - m_last_exec_time;

  #ifdef MCB_USE_LEDS
    led_outputs.exec ();
  #endif

  #ifdef MCB_USE_DIGITAL_IO
    digital_inputs.exec (cur_time);
    digital_outputs.exec (cur_time);
  #endif


  m_last_exec_time = cur_time;
}

// -----------------------------------------------------------------------------

namespace std { namespace this_thread {

void __sleep_for (std::chrono::high_resolution_clock::duration d)
{
  auto start_time = std::chrono::high_resolution_clock::now ();
  do
  {
  } while (std::chrono::high_resolution_clock::now () - start_time < d);
}

extern "C" uintptr_t istack_start;
extern "C" uintptr_t istack_end;

bool __is_interrupt_context (uintptr_t stack_ptr)
{
  return stack_ptr > (uintptr_t)&istack_start && stack_ptr <= (uintptr_t)&istack_end;
}

} }

// -----------------------------------------------------------------------------

namespace std { namespace chrono { namespace _V2 {

uint64_t system_clock::current_time_ticks (void)
{
  return mcb_v13_board::inst ().system_timer.current_time_ticks ();
}

} } }

// -----------------------------------------------------------------------------

void board_debug_rs232_write (const void* data, unsigned int byte_count)
{
#if defined (MCB_USE_SCI_DEBUG)
  auto& bi = mcb_v13_board::inst ();
  bi.debug_usart.write (data, byte_count);
#endif
}

#if defined (MCB_USE_SCI_DEBUG)

[[gnu::cold]] static void
emergency_puts (const char* str,
		unsigned int max_count = std::numeric_limits<unsigned int>::max ())
{
  auto& sci = mcb_v13_board::inst ().debug_usart;

  for (unsigned int i = 0; i < max_count; ++i)
  {
    char c = *str++;
    if (c == 0)
      break;

    while (! sci.status ().transmit_end ()) { }

    sci.set_control (sci.control ().set_transmit_enable ());
    sci.set_transmit_data (c);
  }
}

#endif


[[noreturn, gnu::noinline, gnu::cold]]
void
int_assert (const char* source_filename, int linenum,
            const char* func_name, const char* expr)
{
  // all LEDs on
  dev::led_outputs led_out ( { 1, 0, 0, 1 } );

#if defined (MCB_USE_SCI_DEBUG)
  auto& sci = mcb_v13_board::inst ().debug_usart;

  // soft-reset SCI
  while (! sci.status ().transmit_end ()) { }

  sci.set_control (sci.control ()
	.set_transmit_end_interrupt_enable (false)
	.set_receive_enable (false)
	.set_receive_interrupt_enable (false)
	.set_transmit_enable (false)
	.set_receive_interrupt_enable (false)
	.set_transmit_interrupt_enable (false));

  while (! sci.status ().transmit_end ()) { }

  // flush the remaining buffer bytes
  auto tx_buffer = sci.tx_buffer_stat ();

  emergency_puts ((const char*)tx_buffer.ptr0, tx_buffer.count0);
  emergency_puts ((const char*)tx_buffer.ptr1, tx_buffer.count1);

  emergency_puts ("\n\n");

  if (source_filename != nullptr)
  {
    emergency_puts (source_filename);
    emergency_puts ("(");
    char tmpbuf[16];
    emergency_puts (utils::itoa (linenum, tmpbuf, 10));
    emergency_puts ("): assertion failed\n");
  }
  else
    emergency_puts ("source_filename null\n");

  emergency_puts (expr);
  emergency_puts ("\n\nCPU stopped\n");

#endif

  for (unsigned int i = 0; ; ++i)
  {
    // for some strange reason, the wait builtin emits two wait insns inside
    // the loop.  outside the loop it seems to be fine though.
    // anyway, in this case we'd like to wait forever, so it doesn't matter.
    // __builtin_rx_wait ();
    led_out.write ((i & (1 << 16)) ? 0b1111 : 0b0000);
  };
}

static const char* int_assert_source_filename;
static int int_assert_linenum;
static const char* int_assert_func_name;
static const char* int_assert_expr;

extern "C" [[noreturn, gnu::noinline, gnu::cold]] void
__assert_func (const char* source_filename, int linenum,
               const char* func_name, const char* expr)
{
  // disable interrupts.  because we might be running in
  // user mode, this has to be done in supervisor mode.

  // the function arguments above are passed in registers
  // and will be available in the interrupt handler... which is shaky.
  // with some compiler versions and optimizations it does not work.  thus
  // pass arguments via global variables.

  int_assert_source_filename = source_filename;
  int_assert_linenum = linenum;
  int_assert_func_name = func_name;
  int_assert_expr = expr;

  __builtin_rx_int (1);

  while (true) { }
}

static void INT_Assert (void)
{
  int_assert (int_assert_source_filename, int_assert_linenum,
	      int_assert_func_name, int_assert_expr);
}

#if defined (MCB_USE_SCI_DEBUG)

extern "C" [[noreturn, gnu::cold, gnu::used]] void abort (void)
{
  __assert_func (nullptr, 0, nullptr, "Abort");
}

[[noreturn]] void INT_Excep_SuperVisorInst (void) { int_assert (nullptr, 0, nullptr, "Priviledged Instruction Exception"); }
[[noreturn]] void INT_Excep_AccessInst (void) { int_assert (nullptr, 0, nullptr, "Access Exception"); }
[[noreturn]] void INT_Excep_UndefinedInst (void) { int_assert (nullptr, 0, nullptr, "Undefined Instruction Exception"); }
[[noreturn]] void INT_Excep_FloatingPoint (void) { int_assert (nullptr, 0, nullptr, "Floating Point Exception"); }
[[noreturn]] void NonMaskableInterrupt (void) { int_assert (nullptr, 0, nullptr, "NMI"); }
[[noreturn]] void INT_Excep_BRK (void) { int_assert (nullptr, 0, nullptr, "BRK"); }

#else

extern "C" [[noreturn, gnu::cold]] void int_assert_isr (void)
{
  int_assert (nullptr, 0, nullptr, "");
}

extern "C" [[gnu::used, gnu::alias ("int_assert_isr")]] void abort (void);

[[noreturn, gnu::alias ("int_assert_isr")]] void INT_Excep_SuperVisorInst (void);
[[noreturn, gnu::alias ("int_assert_isr")]] void INT_Excep_AccessInst (void);
[[noreturn, gnu::alias ("int_assert_isr")]] void INT_Excep_UndefinedInst (void);
[[noreturn, gnu::alias ("int_assert_isr")]] void INT_Excep_FloatingPoint (void);
[[noreturn, gnu::alias ("int_assert_isr")]] void NonMaskableInterrupt (void);
[[noreturn, gnu::alias ("int_assert_isr")]] void INT_Excep_BRK (void);

#endif

// -----------------------------------------------------------------------------

static void icu_group_be0_irq (void);
static void icu_group_bl0_irq (void);
static void icu_group_bl1_irq (void);
static void icu_group_al0_irq (void);
static void icu_group_al1_irq (void);

struct isr_desc
{
  unsigned int i;
  void (*f)(void);

  constexpr isr_desc (void) : i (0), f (nullptr) { }

  template <typename F>
  constexpr isr_desc (unsigned int ii, F ff) : i (ii), f (ff) { }

  constexpr bool operator < (const isr_desc& rhs) const { return i < rhs.i; }

  // unconnected interrupts will be dropped in the end, but will fail the
  // uniqueness check.  hence ignore unconnected isr numbers.
  constexpr bool operator == (const isr_desc& rhs) const
  {
    return i != dev::interrupt::unconnected::isr_num
	   && rhs.i != dev::interrupt::unconnected::isr_num
	   && i == rhs.i;
  }

  static constexpr isr_desc init_empty (size_t i) { return isr_desc (i, dev::interrupt::empty_isr_func); }
  static constexpr bool index_equals (const isr_desc& a, const isr_desc& b) { return a.i == b.i; }

  static constexpr bool not_unconnected_num (const isr_desc& a) { return a.i != dev::interrupt::unconnected::isr_num; }
};

template <typename ConnectedInterrupt>
inline constexpr isr_desc make_isr_desc (void)
{
  return { ConnectedInterrupt::isr_num, ConnectedInterrupt::isr_func };
}

struct isr_func
{
  void (*f)(void);

  constexpr isr_func (const isr_desc& d) : f (d.f) { }

  void operator () (void) const { f (); }
};

static constexpr auto rvectors [[gnu::section (".rvectors"), gnu::used]] =

utils::convert_array<isr_desc, isr_func> (
utils::array_verify_unique (
utils::sort_array (
utils::replace_array_elements<isr_desc, isr_desc::index_equals> (

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
  utils::make_array<isr_desc, dev::rx63_interrupt::count, isr_desc::init_empty> (),
#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
      || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
  utils::make_array<isr_desc, dev::rx64_interrupt::count, isr_desc::init_empty> (),
#endif

utils::array_verify_unique (
utils::sort_array (
utils::partition_array<isr_desc, isr_desc::not_unconnected_num> (
utils::make_array (

  // FIXME: maybe put all instantiated devices into a std::tuple
  // and then iterate over all tuple elements and get an array of isrs from
  // each element and construct the initial isr array from that.

  #if defined (MCB_USE_SCI_DEBUG)
  make_isr_desc<mcb_v13_board::debug_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v13_board::debug_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v13_board::debug_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v13_board::debug_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SCI0_USART)
  make_isr_desc<mcb_v13_board::gpsi0_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi0_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi0_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi0_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SCI3_USART)
  make_isr_desc<mcb_v13_board::gpsi1_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi1_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi1_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi1_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SCI5_USART)
  make_isr_desc<mcb_v13_board::gpsi2_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi2_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi2_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v13_board::gpsi2_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_RS485)
  make_isr_desc<mcb_v13_board::rs485_ch1_t::txi_isr_t> (),
  make_isr_desc<mcb_v13_board::rs485_ch1_t::rxi_isr_t> (),
  make_isr_desc<mcb_v13_board::rs485_ch1_t::tei_isr_t> (),
  make_isr_desc<mcb_v13_board::rs485_ch1_t::eri_isr_t> (),

    #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144) \
        || defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)
    make_isr_desc<mcb_v13_board::rs485_ch2_t::txi_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::rxi_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::tei_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::eri_isr_t> (),

    #elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    make_isr_desc<mcb_v13_board::rs485_ch2_t::bri_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::eri_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::rxi_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::txi_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::tei_isr_t> (),
    make_isr_desc<mcb_v13_board::rs485_ch2_t::dri_isr_t> (),

    #endif
  #endif

  #if defined (MCB_USE_I2C)
  make_isr_desc<mcb_v13_board::internal_i2c_t::txi_isr_t> (),
  make_isr_desc<mcb_v13_board::internal_i2c_t::rxi_isr_t> (),
  make_isr_desc<mcb_v13_board::internal_i2c_t::tei_isr_t> (),
  make_isr_desc<mcb_v13_board::internal_i2c_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SPI)
  make_isr_desc<mcb_v13_board::internal_spi_t::txi_isr_t> (),
  make_isr_desc<mcb_v13_board::internal_spi_t::rxi_isr_t> (),
  make_isr_desc<mcb_v13_board::internal_spi_t::tei_isr_t> (),
  make_isr_desc<mcb_v13_board::internal_spi_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_PCA9698)
  make_isr_desc<mcb_v13_board::pca9698_i0_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_PCD4641)
  make_isr_desc<mcb_v13_board::pcd4641_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_MCX514)
  make_isr_desc<mcb_v13_board::mcx51x_t::isr0_t> (),
  make_isr_desc<mcb_v13_board::mcx51x_t::isr1_t> (),
  #endif

  #if defined (MCB_USE_FCU)
    // fcu also has some interrupts which can be useful for background
    // operations.
  #endif 

  #if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC0_RAW)
  make_isr_desc<mcb_v13_board::rx_edmac0_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_RX_ETHERC1) || defined (MCB_USE_RX_ETHERC1_RAW)
  make_isr_desc<mcb_v13_board::rx_edmac1_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW)
  make_isr_desc<mcb_v13_board::lan9250_t::isr0_t> (),
  #endif


  #if defined (MCB_USE_TRIGGER_IO)
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr0_t> (),
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr1_t> (),
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr2_t> (),
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr3_t> (),
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr4_t> (),
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr5_t> (),
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr6_t> (),
  make_isr_desc<mcb_v13_board::trigger_inputs_t::isr7_t> (),
  #endif

  #if defined (MCB_USE_TPU)
  make_isr_desc<mcb_v13_board::tpu0_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu0_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu0_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu0_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu0_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu0_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v13_board::tpu1_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu1_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu1_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu1_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu1_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu1_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v13_board::tpu2_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu2_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu2_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu2_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu2_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu2_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v13_board::tpu3_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu3_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu3_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu3_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu3_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu3_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v13_board::tpu4_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu4_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu4_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu4_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu4_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu4_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v13_board::tpu5_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu5_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu5_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu5_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu5_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v13_board::tpu5_t::tci_u_isr_t> (),
  #endif

  make_isr_desc<mcb_v13_board::system_timer_t::isr0_t> (),
  make_isr_desc<mcb_v13_board::system_timer_t::isr1_t> (),

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    make_isr_desc<dev::interrupt::connected_isr<dev::rx63_interrupt::line<dev::rx63_interrupt::reserved_0>,
		  dev::interrupt::func<decltype (&INT_Excep_BRK), &INT_Excep_BRK>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx63_interrupt::line<dev::rx63_interrupt::reserved_1>,
		  dev::interrupt::func<decltype (&INT_Assert), &INT_Assert>>> ()

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
	|| defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::reserved_0>,
		  dev::interrupt::func<decltype (&INT_Excep_BRK), &INT_Excep_BRK>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::reserved_1>,
		  dev::interrupt::func<decltype (&INT_Assert), &INT_Assert>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_be0>,
		  dev::interrupt::func<decltype (&icu_group_be0_irq), &icu_group_be0_irq>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_bl0>,
		  dev::interrupt::func<decltype (&icu_group_bl0_irq), &icu_group_bl0_irq>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_bl1>,
		  dev::interrupt::func<decltype (&icu_group_bl1_irq), &icu_group_bl1_irq>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_al0>,
		  dev::interrupt::func<decltype (&icu_group_al0_irq), &icu_group_al0_irq>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_al1>,
		  dev::interrupt::func<decltype (&icu_group_al1_irq), &icu_group_al1_irq>>> ()

  #endif

))))))));


// check each interrupt status bit for the group and invoke the
// respective ISR function from the table.

static inline void do_group_interrupts (uint32_t i, const isr_func* f)
{
  for (; i != 0; ++f, i >>= 1)
    if (i & 1)
      (*f)();
}

[[gnu::flatten]] void icu_group_be0_irq (void)
{
  static constexpr dev::hw_reg_r<uint32_t, dev::const_addr<0x00087600>> GRPBE0 = { };
  do_group_interrupts (GRPBE0, &rvectors[dev::rx64_interrupt::be0_0]);
}

[[gnu::flatten]] void icu_group_bl0_irq (void)
{
  static constexpr dev::hw_reg_r<uint32_t, dev::const_addr<0x00087630>> GRPBL0 = { };
  do_group_interrupts (GRPBL0, &rvectors[dev::rx64_interrupt::bl0_0]);
}

[[gnu::flatten]] void icu_group_bl1_irq (void)
{
  static constexpr dev::hw_reg_r<uint32_t, dev::const_addr<0x00087634>> GRPBL1 = { };
  do_group_interrupts (GRPBL1, &rvectors[dev::rx64_interrupt::bl1_0]);
}

[[gnu::flatten]] void icu_group_al0_irq (void)
{
  static constexpr dev::hw_reg_r<uint32_t, dev::const_addr<0x00087830>> GRPAL0 = { };
  do_group_interrupts (GRPAL0, &rvectors[dev::rx64_interrupt::al0_0]);
}

[[gnu::flatten]] void icu_group_al1_irq (void)
{
  static constexpr dev::hw_reg_r<uint32_t, dev::const_addr<0x00087834>> GRPAL1 = { };
  do_group_interrupts (GRPAL1, &rvectors[dev::rx64_interrupt::al1_0]);
}


// -----------------------------------------------------------------------------

// the board info data always remains in .rodata.  if this is a bootloader
// image, the board info data will be overwritten in the image during/before
// programming.  for normal images, this will contain the all-zero board info
// data, in case there is no real one available.

// when building the boot loader image there's no need to invoke the function
// and could return the reference to the .rodata directly.  however, this will
// allow the compiler to optimize accesses to the board_info_data.
// because the board_info_data is originally blank (all zero), the compiler will
// optimize away the accsesses.  prevent this by always going through the
// function pointer.

extern "C" [[gnu::section (".rodata")]] const std::aligned_storage<sizeof (board_info), 4>::type
board_info_data = { };

[[gnu::cold]] const mcb_v13_board_info& mcb_v13_board_info::inst (void)
{
  // this is RX63N specific.  the fixed vector table in the boot loader
  // contains a pointer to the actual board_info data which usually resides
  // in user boot flash image.
  // the default renesas usb boot loader contains 0xFFFFFFFF at that place.
  // in any case, make sure that the address in the range of the user boot
  // flash area.
/*
  uintptr_t bi = *(const uintptr_t*)0xFF7FFF84;

  if (bi >= 0xFF7FC000 && bi <= 0xFF7FFFFF)
    return *(const mcb_v13_board_info*)bi;

  return *(const mcb_v13_board_info*)&board_info_data;
*/

  // for now use a hard-coded board info, as long as the bootloader thing
  // is not working.
  struct tmp_board_info : public mcb_v13_board_info
  {
    tmp_board_info (void)
    {
      set_board_id (0x00000008);

      auto& ifcfg0 = ifconfigs ()[0];
      ifcfg0.set_hw_addr ({ 0x02, 0x60, 0x95, 0x26, 0x30, 0x00 });
      ifcfg0.set_ipv4_addr ({ 192, 168, 0, 80 });
      ifcfg0.set_ipv4_netmask ({ 255, 255, 255, 0 });

      auto& ifcfg1 = ifconfigs ()[1];
      ifcfg1.set_hw_addr ({ 0x02, 0x60, 0x95, 0x26, 0x30, 0x01 });
      ifcfg1.set_ipv4_addr ({ 192, 168, 0, 81 });
      ifcfg1.set_ipv4_netmask ({ 255, 255, 255, 0 });

      auto& ifcfg2 = ifconfigs ()[2];
      ifcfg2.set_hw_addr ({ 0x02, 0x60, 0x95, 0x26, 0x30, 0x02 });
      ifcfg2.set_ipv4_addr ({ 192, 168, 0, 82 });
      ifcfg2.set_ipv4_netmask ({ 255, 255, 255, 0 });

      auto& ifcfg3 = ifconfigs ()[3];
      ifcfg3.set_hw_addr ({ 0x02, 0x60, 0x95, 0x26, 0x30, 0x03 });
      ifcfg3.set_ipv4_addr ({ 192, 168, 0, 83 });
      ifcfg3.set_ipv4_netmask ({ 255, 255, 255, 0 });
    }
  };

  static tmp_board_info i;
  return i;
}

// =============================================================================


void mcb_v13_board::reset_to_func (void (*func)(void))
{
  dev::this_cpu::save_disable_interrupts ();

  // depending on which devices have been used, we might need a proper
  // shutdown of the drivers to stop the interrupts etc.
  // otherwise the new program might malfunction because of some unexpected
  // hardware event.
  this->~mcb_v13_board ();
  set_peripheral_reset (true);

  func ();
}

void mcb_v13_board::reset (void)
{
  // SWRR software reset register
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x000800C2>> SWRR = { };
  SWRR = 0xA501;
}

[[gnu::cold]] void mcb_v13_board::maybe_reset_config_to_factory_default (void)
{
  // if the FCU is not enabled, we will not be able to clear the configuration
  // data.
  // if the LEDs are not enabled, we will not be able to indicate any success
  // to the user.

  // don't confuse the user in those cases and just do nothing unless both,
  // FCU and LEDs are enabled.

#if defined (MCB_USE_FCU) && defined (MCB_USE_LEDS)

  // the magic reset sequence is:
  // - power on
  // - 3x reset within 500 ms
  auto flash_leds = [this] (int count, int millis = 500)
  {
    for (int i = 0; i < count; ++i)
    {
      led_outputs.write (0b1111);
      std::this_thread::sleep_for (std::chrono::milliseconds (millis));
      led_outputs.write (0b0000);
      std::this_thread::sleep_for (std::chrono::milliseconds (millis));
    }
  };

  auto rsrc = reset_source ();

  if (rsrc == reset_source::poweron)
  {
    set_reset_counter (1);
    flash_leds (1);
  }
  else
  {
    auto rcnt = reset_counter ();

    if (rcnt == 3)
    {
      set_reset_counter (4);
      flash_leds (5, 100);

      fcu.data_flash_dev ().erase_all_blocks ();
    }
    else if (rcnt < 3)
    {
      // increment the reset counter
      set_reset_counter (rcnt + 1);

      flash_leds (1);

      // if the user has pushed the reset button again while the leds are
      // flashing we will not get here.  if we are here user has missed the
      // deadline and we abort the sequence.
    }
  }

  set_reset_counter (4);

#endif // defined (MCB_USE_FCU) && defined (MCB_USE_FCU)
}

// =============================================================================
// non-relocatable fixed vector table
// they are different on RX63 and RX64/RX71.
// from RX64 on, some of the configuration parameters are moved to another
// 48 byte area in flash memory:

//   0x00120040 .. 0x00120043
//     SPPC - setting (serial programmer command control register)
//
//     equivalent RX63 addr: 0xFFFFFF9C (to some extent)
//
//
//   0x00120048 .. 0x0012004B
//     TMEF - TM enable flag register
//
//     equivalent RX63 addr: no TM functions on RX63
//
//
//   0x00120050 .. 0x0012005F
//     OSIS - serial programmer ID setting register
//
//     equivalent RX63 addr: 0xFFFFFFA0 .. 0xFFFFFFAF
//
//
//   0x00120060 .. 0x00120063
//     TMINF - TM identification data register
//
//     equivalent RX63 addr: no TM functions on RX63
//
//
//   0x00120064 .. 0x00120067
//     MDE - endian select register
//
//     equivalent RX63 addr: 0xFFFFFF80
//
//
//   0x00120068 .. 0x0012006B
//     OFS0 - option function select register 0
//
//     equivalent RX63 addr: 0xFFFFFF8C
//
//
//   0x0012006C .. 0x0012006F
//     OFS1 - option function select register 1
//
//     equivalent RX63 addr: 0xFFFFFF88


// flashing those areas requires using special SCI bootloader commands
// and special FCU commands.

// we could locate that table at address 0x00120040 and have the programmer
// tool pick it up automatically as it is suggested by the renesas flash
// memmory programming manual.  however, when creating a binary ROM image,
// it would result in a very big file.

// the fields among RX63 and RX64/RX71 are almost the same, except for the
// new TM fields.  thus we extend the fvectors table with these values.
// the RX64/RX71 bootloader or SCI programming tools will pick up the
// configuration values from the fvectors table and program the flash areas
// with the corresponding special commands.

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
  #define USING_RX63

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
	|| defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
  #define USING_RX64_RX71

#endif

typedef uintptr_t fp;
extern "C" void PowerON_Reset (void);

static constexpr auto fvectors [[gnu::section (".fvectors"), gnu::used, gnu::aligned (4)]] =
utils::make_array (

//  normal    | user boot   |  desc (normal / user boot)
//   addr     |   addr      |
//------------+-------------+--------------------------------------
// 0xFFFFFF80 | 0xFF7FFF80  |  MDES Endian Select Register / -

// in user boot mode this field can be used for something else.
// to allow running user boot images as normal images, keep this setting
// for both variants the same.

#if defined(__RX_LITTLE_ENDIAN__)
  (fp)0xFFFFFFFF,
#elif defined(__RX_BIG_ENDIAN__)
  (fp)0xFFFFFFF8,
#else
  #error unknown endian setting
  (fp)0x00000000,
#endif

//------------+-------------+--------------------------------------
// 0xFFFFFF84 | 0xFF7FFF84  | Reserved

#ifndef RX_USER_BOOT_IMAGE
  (fp)0x00000000,

#else
  (fp)&board_info_data,
#endif

//------------+-------------+--------------------------------------
// 0xFFFFFF88 | 0xFFFFFF88  | OFS1 / -

  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFF8C | 0xFF7FFF8C  | OFS0 / -
  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFF90 | 0xFF7FFF90  | RX63: Reserved
//                          | RX64: use as TMEF
  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFF94 | 0xFF7FFF94  | RX63: Reserved
//                          | RX64: use as TMINF
  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFF98 | 0xFF7FFF98  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFF9C | 0xFF7FFF9C  | RX63: ROM Code Protection
//                          | RX64: use as SPPC

#ifdef USING_RX63
//  (fp)0x00000000,   // user area / user boot area r/w access prohibited
//  (fp)0x00000001,   // user area / user boot area r access prohibited
  (fp)0xFFFFFFFF,     // no restriction

#else
  (fp)0xFFFFFFFF,
#endif


//------------+-------------+--------------------------------------
// 0xFFFFFFA0 | 0xFF7FFFA0  | ID code protection
//                          | control code, ID code 1, ID code 2, ID code 3
  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFFA4 | 0xFF7FFFA4  | ID code protection
//                          | ID code 4, ID code 5, ID code 6, ID code 7
  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFFA8 | 0xFF7FFFA8  | ID code protection
//                          | ID code 8, ID code 9, ID code 10, ID code 11
  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFFAC | 0xFF7FFFAC  | ID code protection
//                          | ID code 12, ID code 13, ID code 14, ID code 15
  (fp)0xFFFFFFFF,

//------------+-------------+--------------------------------------
// 0xFFFFFFB0 | 0xFF7FFFB0  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFB4 | 0xFF7FFFB4  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFB8 | 0xFF7FFFB8  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFBC | 0xFF7FFFBC  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFC0 | 0xFF7FFFC0  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFC4 | 0xFF7FFFC4  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFC8 | 0xFF7FFFC8  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFCC | 0xFF7FFFCC  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFD0 | 0xFF7FFFD0  | Exception(Supervisor Instruction)
  (fp)INT_Excep_SuperVisorInst,

//------------+-------------+--------------------------------------
// 0xFFFFFFD4 | 0xFF7FFFD4  | Exception(Access Instruction)
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFD8 | 0xFF7FFFD8  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFDC | 0xFF7FFFDC  | Exception(Undefined Instruction)
  (fp)INT_Excep_UndefinedInst,

//------------+-------------+--------------------------------------
// 0xFFFFFFE0 | 0xFF7FFFE0  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFFE4 | 0xFF7FFFE4  | Exception(Floating Point)
  (fp)INT_Excep_FloatingPoint,

//------------+-------------+--------------------------------------
// 0xFFFFFFE8 | 0xFF7FFFE8  | Reserved / UB Code A

// note:
// the user boot code is a 8 byte ASCII string which is checked by the SCI1
// bootloader.  if it says "UserBoot" (the official documented value in the
// hardware manual), the SCI1 bootloader will erase it when blanking the
// chip.  if it says "UsbBoot" (terminated with 0xFF), it will not be erased.
// we don't want our bootloader to be erased automatically, which will also
// erase some of the board info.

#ifndef RX_USER_BOOT_IMAGE
  (fp)0x00000000,
#else
//  (fp)0x55736572,       // user boot string
  (fp)0x55736242,         // usb boot string
#endif

//------------+-------------+--------------------------------------
// 0xFFFFFFEC | 0xFF7FFFEC  | Reserved / UB Code A

#ifndef RX_USER_BOOT_IMAGE
  (fp)0x00000000,
#else
//  (fp)0x426F6F74,      // user boot string
  (fp)0x6F6F74FF,        // usb boot string
#endif

//------------+-------------+--------------------------------------
// 0xFFFFFFF0 | 0xFF7FFFF0  | Reserved / UB Code B

#ifndef RX_USER_BOOT_IMAGE
  (fp)0x00000000,
#else
  (fp)0xFFFFFF07,
#endif

//------------+-------------+--------------------------------------
// 0xFFFFFFF4 | 0xFF7FFFF4  |  Reserved / UB Code B

#ifndef RX_USER_BOOT_IMAGE
  (fp)0x00000000,
#else
  (fp)0x0008C04C,
#endif

//------------+-------------+--------------------------------------
// 0xFFFFFFF8 | 0xFF7FFFF8  | NMI / MDEB

#ifndef RX_USER_BOOT_IMAGE
  (fp)NonMaskableInterrupt,
#else

#if defined(__RX_LITTLE_ENDIAN__)
  (fp)0xFFFFFFFF,
#elif defined(__RX_BIG_ENDIAN__)
  (fp)0xFFFFFFF8,
#else
  #error unknown endian setting
  (fp)0x00000000,
#endif

#endif

//------------+-------------+--------------------------------------
// 0xFFFFFFFC | 0xFF7FFFFC  | RESET
  (fp)PowerON_Reset
);

