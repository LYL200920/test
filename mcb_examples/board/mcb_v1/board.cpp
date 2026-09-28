#include <sys/unistd.h>

#include <cstdio>
#include <chrono>
#include <thread>

#include <board/board.hpp>
#include <board/board_info.hpp>

#include <utils/rodata.hpp>
#include <utils/text.hpp>

#include <dev/cpu.hpp>
#include <dev/renesas/rx_bsc.hpp>
#include <dev/renesas/rx_clk.hpp>
#include <dev/renesas/rx_sys.hpp>

// make sure that the board constructor is ran as the very first static
// initializer by specifying the init_priority attribute.
[[gnu::init_priority (101), gnu::used]] mcb_v1_board mcb_v1_board::g_inst;

[[noreturn, gnu::noinline]]
static void board_info_check_ng (void)
{
  while (true) { }
}

[[gnu::cold, gnu::noinline]]
mcb_v1_board::board_id_t mcb_v1_board::board_id (void) const
{
  auto&& bi = mcb_v1_board_info::inst ();

  const auto& uid_override_val = bi.board_uid_override ();

  static_assert (std::tuple_size_v<std::remove_reference_t<decltype (uid_override_val)>> == std::tuple_size_v<board_id_t>);

  uint8_t or_sum = 0x00;
  uint8_t and_sum = 0xFF;

  for (auto&& i : uid_override_val)
  {
    or_sum |= i;
    and_sum &= i;
  }

  if (or_sum != 0x00 && and_sum != 0xFF)
  {
    // the override UID is valid (not all 0x00 and not all 0xFF) -- use it
    return { uid_override_val };
  }

  board_id_t r;
   r.fill (0x00);

  r[0] = 'R';
  r[1] = 'X';
  r[2] = '6';
  r[3] = '3';
  r[4] = 'N';
  r[5] = 1;

  static_assert (r.size () >= 16 + 6);
  std::copy_n ((volatile const uint8_t*)0xFEFFFAC0, 16, r.data () + 6);

  return r;
}

[[gnu::cold]]
mcb_v1_board::reset_hw_init_t::reset_hw_init_t (mcb_v1_board& brd)
{
#ifndef MCB_NO_RESET_HW_INIT

  #if 1
  {
    auto&& bi = mcb_v1_board_info::inst ();
    if (!(bi.board_id () == 0x00000001 || bi.board_id () == 0x10000000))
      board_info_check_ng ();
  }
  #endif


  dev::rx_sys::disable_register_protection ();

  // before enabling the output pin, set the output value to hold
  // the peripheral reset during the init.
  set_peripheral_reset (true);
  dev::rx_gpio::pdr::p9 = 0b0000'1010;	// P2 = EXP_COM_DE2, P93 = /PERI_RST

  // dev::rx_gpio::pdr::p3 = 0b0000'0000;  default after reset
  dev::rx_gpio::pmr::p3 = 0b1100'0000;  // P36 = EXTAL, P37 = XTAL

// these are a bit pointless to check.  if we are here, the chip is already up
// and running.
//  while (SYSTEM.OPCCR.BIT.OPCMTSF == 1);	// b4
//  while (FLASH.FENTRYR.WORD != 0);

  // main clock oscillator wait time = 262144 cycles
  // dev::rx_clk::moscwtcr = 0x0E;  default after reset

  // sub-clock oscillator wait time = 262144 cycles.
  // dev::rx_clk::soscwtcr = 0x0E;  default after reset

  dev::rx_clk::mosccr = 0; // start main clock oscillator
  while (dev::rx_clk::mosccr == 0x01);

  // MCB-27 FIXME: set registers based on the clock values.
  static_assert (xtal_hz == 12000000, "");
  static_assert (iclock_hz == 96000000, "");
  static_assert (pclock_hz == 48000000, "");
  static_assert (bclk_hz == 96000000, "");
  static_assert (bclk_hz_output == 48000000, "");

  // set PLL wait time to wait for 4194304 PLL cycles before the PLL output
  // is supplied to the internal MCU circuits.
  // dev::rx_clk::pllwtcr = 0x0F;  default after reset.

  // enable PLL as div = 1/1, mul = x16
  // resulting PLL output clock = 12*16 = 192 MHz
  dev::rx_clk::pllcr = 0b00'001111'000000'00;

  dev::rx_clk::pllcr2 = 0; // start PLL
  while (dev::rx_clk::pllcr2 == 1);

  constexpr auto pclkb_div = 0b0010; // 1/4 = 48 MHz
  static_assert (pclkb_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto pclka_div = 0b0001; // 1/2 = 96 MHz
  static_assert (pclka_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto bclk_div = 0b0001; // 1/2 = 96 MHz
  static_assert (bclk_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto iclk_div = 0b0001; // 1/2 = 96 MHz
  static_assert (iclk_div != dev::rx_clk::sckcr_div_invalid (), "");

  constexpr auto fclk_div = 0b0010; // 1/4 = 48 MHz
  static_assert (fclk_div != dev::rx_clk::sckcr_div_invalid (), "");

  dev::rx_clk::sckcr =
	    (0b0001 << 0)
	  | (0b0001 << 4)
	  | (pclkb_div << 8)
	  | (pclka_div << 12)
	  | (bclk_div << 16)
	  | (1 << 22) // disable SDCLK pin output
	  | (0 << 23) // enable BCLK pin output
	  | (iclk_div << 24)
	  | (fclk_div << 28);

  // IEBCK (IEBUS clock) = 1/4
  // UCK (USB clock) = 1/4
  dev::rx_clk::sckcr2 = 0x0032;

  // BCLKDIV (BCLK pin output clock) = 1/2 BLCK = 48 MHz
  dev::rx_clk::bckcr = 0x01;

  // CKSEL (clock source select) = PLL circuit
  dev::rx_clk::sckcr3 = 0x0400;

  dev::rx_gpio::pdr::p0 = 0b0010'1000;	// P03 = EXP_COM_DE1, P05 = LD4
  dev::rx_gpio::pdr::p2 = 0b0000'1000;	// P23 = EXP_OUT10
  dev::rx_gpio::pdr::p5 = 0b0111'0000;	// P54 = EXP_OUT11, P55 = EXP_OUT12, P56 = EXP_OUT13
  dev::rx_gpio::pdr::p6 = 0b1111'1111;  // P6[7..0] = trigger outputs
  dev::rx_gpio::pdr::p7 = 0b0000'1001;	// P70 = EXP_OUT23, P73 = EXP_OUT20
  dev::rx_gpio::pdr::pb = 0b1111'0100;	// PB2 = EXP_OUT22, PB4 = DAC_CS3N, PB5 = DAC_CS2N, PB6 = DAC_CS1N, PB7 = EXP_OUT21
  dev::rx_gpio::pdr::pf = 0b0010'0000;	// PF5 = LD3
  dev::rx_gpio::pdr::pj = 0b0010'1000;	// PJ3 = LD2, PJ5 = LD1

  // I2C/SCI2 P12,P13: NMOS open-drain output
  dev::rx_gpio::odr0::p1 = 0b01'01'00'00;

  // I2C/SCI5 PC2,PC3: NMOS open-drain output
  dev::rx_gpio::odr0::pc = 0b01'01'00'00;

  dev::rx_gpio::pfs::p00 = 0b000'01010;  // TXD6
  dev::rx_gpio::pfs::p01 = 0b000'01010;  // RXD6
  dev::rx_gpio::pfs::p20 = 0b000'01010;  // TXD0
  dev::rx_gpio::pfs::p21 = 0b000'01010;  // RXD0
  dev::rx_gpio::pfs::p90 = 0b000'01010;  // TXD7
  dev::rx_gpio::pfs::p92 = 0b000'01010;  // RXD7

  // USB port (disabled for now)
  #if defined (MCB_USE_USB) && 0
    dev::rx_gpio::pfs::p14 = 0b000'10001;	// USB0_DPUPE
    dev::rx_gpio::pfs::p16 = 0b000'10001;	// USB0_VBUS
    dev::rx_gpio::pfs::p22 = 0b000'10011;	// USB0_DRPD
    dev::rx_gpio::pfs::p25 = 0b000'10011;	// USB0_DPRPD
    dev::rx_gpio::pfs::pfusb0 = 0x0C;		// USB0_DPUPE High,USB0_DPRPD Low
  #endif

  #if defined (MCB_USE_I2C)
    // I2C / SCI2
    dev::rx_gpio::pfs::p12 = 0b000'01010;	// RXD2 / SSCL2
    dev::rx_gpio::pfs::p13 = 0b000'01010;	// TXD2 / SSDA2
  #endif

  #if defined (MCB_USE_MCX514_I2C_EXT)
    dev::rx_gpio::pfs::pc2 = 0b000'01010;	// PC2 = RXD5 / SSCL5
    dev::rx_gpio::pfs::pc3 = 0b000'01010;	// PC3 = TXD5 / SSDA5
  #endif

  #if defined (MCB_USE_ETHERC)
    dev::rx_gpio::pfs::p71 = 0b000'10001;	// ET MDIO
    dev::rx_gpio::pfs::p72 = 0b000'10001;	// ET MDC
    dev::rx_gpio::pfs::p74 = 0b000'10010;	// RMII_RXD1
    dev::rx_gpio::pfs::p75 = 0b000'10010;	// RMII_RXD0
    dev::rx_gpio::pfs::p76 = 0b000'10010;	// REF50CK
    dev::rx_gpio::pfs::p77 = 0b000'10010;	// RMII_RX_ER
    dev::rx_gpio::pfs::p80 = 0b000'10010;	// RMII_TXD_EN
    dev::rx_gpio::pfs::p81 = 0b000'10010;	// RMII_TDX0
    dev::rx_gpio::pfs::p82 = 0b000'10010;	// RMII_TDX1
    dev::rx_gpio::pfs::p83 = 0b000'10010;	// RMII_CRS_DV
  #endif

  #if defined (MCB_USE_PCD4641)
    dev::rx_gpio::pfs::p17 = 0b010'00000;	// IRQ7 (PCD4641)
  #endif

  #if defined (MCB_USE_MCX514)
    dev::rx_gpio::pfs::p15 = 0b010'00000;	// IRQ5 (MCX514 INT1N)
    dev::rx_gpio::pfs::p33 = 0b010'00000;	// IRQ3 (MCX514 INT0N)
  #endif

  #if defined (MCB_USE_TRIGGER_IO)
    dev::rx_gpio::pfs::p40 = 0b01'000000; // IRQ8, TRG_IN1
    dev::rx_gpio::pfs::p41 = 0b01'000000; // IRQ9, TRG_IN2
    dev::rx_gpio::pfs::p42 = 0b01'000000; // IRQ10, TRG_IN3
    dev::rx_gpio::pfs::p43 = 0b01'000000; // IRQ11, TRG_IN4
    dev::rx_gpio::pfs::p44 = 0b01'000000; // IRQ12, TRG_IN5
    dev::rx_gpio::pfs::p45 = 0b01'000000; // IRQ13, TRG_IN6
    dev::rx_gpio::pfs::p46 = 0b01'000000; // IRQ14, TRG_IN7
    dev::rx_gpio::pfs::p47 = 0b01'000000; // IRQ15, TRG_IN8
  #endif

  #if defined (MCB_USE_SCI_DEBUG)
    dev::rx_gpio::pfs::p30 = 0b000'01010; // RXD1
    dev::rx_gpio::pfs::p26 = 0b000'01010; // TXD1
  #endif

  #if defined (MCB_USE_PCA9698)
    // conflicts with trigger input.
    // this would require interrupt sharing.
    // dev::rx_gpio::pfs::pc1 = 0b01'000000;	// IRQ12 (PCA9698 U23)
  #endif

  dev::rx_gpio::pmr::p0 = 0b0000'0011;	// P00 = TXD6, P01 = RXD6
  dev::rx_gpio::pmr::p2 = 0b0000'0011;	// P20 = TXD0, P21 = RXD0
  dev::rx_gpio::pmr::p9 = 0b0000'0101;	// P90 = TXD7, P92 = RXD7

  // pull-up sci6 rx/tx pins.
  dev::rx_gpio::pcr::p0 = 0b00000011;

  // pull-up sci7 rx/tx pins.
  dev::rx_gpio::pcr::p9 = 0b00000101;

  #if defined (MCB_USE_I2C)
    dev::rx_gpio::pmr::p1 = 0b00001100;  // P12,P13: use I2C/SCI2
  #endif

  #if defined (MCB_USE_MCX514_I2C_EXT)
    dev::rx_gpio::pmr::pc = 0b00001100	// PC2 = RXD5, PC3 = TXD5
    dev::rx_gpio::dscr::pc = 0b00001100; // high drive capacity
  #endif

  #if defined (MCB_USE_SCI_DEBUG)
    dev::rx_gpio::pmr::p3 |= 0b0000'0001; // P30 = RXD1
    dev::rx_gpio::pmr::p2 |= 0b0100'0000; // P23 = TXD1
  #endif


  #if defined (MCB_USE_ETHERC)
    dev::rx_gpio::pmr::p7 = 0b1111'0110;  // RMII_RX_ER | REF50CK | RMII_RXD0 | RMII_RXD1 | ET MDC | ET MDIO
    dev::rx_gpio::pmr::p8 = 0b0000'1111;  // RMII_CRS_DV | RMII_TDX1 | RMII_TXD0 | RMII_TX_EN
  #endif


  // high-drive for bus WR and RD signals.
  dev::rx_gpio::dscr::p5 = 0b0000'0101;

  // high-drive for data bus
  dev::rx_gpio::dscr::pd = 0b1111'1111;
  dev::rx_gpio::dscr::pe = 0b1111'1111;

  // high-drive for address bus
  dev::rx_gpio::dscr::pa = 0b1111'1111;

  // high-drive for CS signals
  dev::rx_gpio::dscr::pc = 0b0110'0000;

  dev::rx_sys::enable_register_protection ();

  // ---------------------------------------------------------------------------
  // PCD4641 CS1 external bus (0x07000000..0x07FFFFFF)

#if defined (MCB_USE_PCD4641)
/*
// these are the old timings for the PCD4641.
// PCD4641A has a slightly faster bus interface.

  // byte strobe mode, external wait enable, page read/write off,
  // normal access mode
  // the PCD will insert additional bus wait cycles as needed.
  dev::rx_bsc::cs1mod = 0x0008;

  // read cycle: 34 ns of active RD & CS signal and data time
  //             18 ns of data float time (cs/address hold)
  constexpr unsigned int pcd_rd_cs_clk = ns_to_bclk (34);
  constexpr unsigned int pcd_rd_cs_hold = std::max (1u, ns_to_bclk (18));

  // write cycle: 28 ns of active WR & CS signal
  //              (have to wait for 28 ns WAIT output delay time, even though
  //               the min. WR & CS time is 14 ns + a following 14 ns data setup time).
  constexpr unsigned int pcd_wr_cs_clk = ns_to_bclk (28);
  constexpr unsigned int pcd_wr_cs_hold = std::max (1u, ns_to_bclk (0));

  static_assert (pcd_rd_cs_clk <= 31, "");
  static_assert (pcd_wr_cs_clk <= 31, "");

  dev::rx_bsc::cs1wcr1 = (pcd_wr_cs_clk << 16) | (pcd_rd_cs_clk << 24);

  // CSROFF = pcd_rd_cs_hold
  // CSWOFF = pcd_wr_cs_hold
  // WDOFF = AWAIT = RDON = WRON = WDON = CSON = 0
  static_assert (pcd_rd_cs_hold <= 7, "");
  static_assert (pcd_wr_cs_hold <= 7, "");

  dev::rx_bsc::cs1wcr2 = pcd_rd_cs_hold | (pcd_wr_cs_hold << 4);

  // no recovery cycles
  dev::rx_bsc::cs1rec = 0;

  // enable operation, 8 bit bus, chip endian mode (little), MPX disable
  dev::rx_bsc::cs1cr = 0x0021;
*/

  {
    // byte strobe mode, external wait enable, page read/write off,
    // normal access mode
    // the PCD will insert additional bus wait cycles as needed.
    dev::rx_bsc::cs1mod = 0b0'00000'0'0'0000'1'00'0;

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
    // 96 mhz bclk: 24 mhz access speed
    constexpr unsigned int wron = 0;
    constexpr unsigned int wdon = 0;
    constexpr unsigned int wdoff = 0;
    constexpr unsigned int cswoff = 0;
    constexpr unsigned int cswwait = ns_to_bclk (26) - 1;

    // read cycle
    //    32 ns of RD & CS assert (but it seems 31.25 ns is also OK).
    //    10 ns between accesses
    // 96 mhz bclk: 24 mhz access speed
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

    dev::rx_bsc::cs1wcr1 =
	(csrwait << 24) | (cswwait << 16);

    dev::rx_bsc::cs1wcr2 =
	csroff | (cswoff << 4) | (cson << 28) | (rdon << 16) | (wron << 20) | (wdon << 24) | (wdoff << 8);

    // recovery cycles for normal register access
    static_assert (rd_recovery <= 15, "");
    static_assert (wr_recovery <= 15, "");

    dev::rx_bsc::cs1rec = (wr_recovery << 8) | (rd_recovery << 0);
    dev::rx_bsc::cs1cr = 0b000'0'000'0'00'10'000'1;
  }

#endif // defined (MCB_USE_PCD4641)

  // ---------------------------------------------------------------------------
  // MCX514 CS2 external bus (0x06000000..0x06FFFFFF)

#if defined (MCB_USE_MCX514)

  // single write strobe mode, no external wait, page read/write off,
  // normal access mode
  dev::rx_bsc::cs2mod = 0x0001;

  // it seems the MCX needs to see a CS de-assertion between the accesses.
  // for some combinations of the wait values, CS is never de-asserted, which
  // is a problem.  so better to check with a scope.
  // notice that the timing granularity is in 1000/96M = 10.42 ns
  // (internal bus clock).  values are rounded up to multiples of that.
  // so a 4ns delay will actually be 10.42 ns.

  static_assert (std::ratio_greater<std::ratio<bclk_hz,1>, mcx51x_t::clock_hz>::value, "");

  // read cycle
  // continuous register read = 20 MHz.
  constexpr unsigned int mcx_rdon_delay = ns_to_bclk (4);
  constexpr unsigned int mcx_rd_cs_assert = ns_to_bclk (20 + 4);
  constexpr unsigned int mcx_rd_cs_hold = ns_to_bclk (5);

  // write cycle
  // continuous register write = 24 MHz.
  // continuous command write = 42 ns + 125 ns = 167 ns = 6 MHz
  // (2 MCX wait cycles inserted in software after command write).
  constexpr unsigned int mcx_wron_delay = ns_to_bclk (4);
  constexpr unsigned int mcx_wr_cs_assert = ns_to_bclk (30);
  constexpr unsigned int mcx_wr_cs_hold = ns_to_bclk (0);

  static_assert (mcx_rd_cs_assert <= 31, "");
  static_assert (mcx_wr_cs_assert <= 31, "");

  dev::rx_bsc::cs2wcr1 = (mcx_rd_cs_assert << 24) | (mcx_wr_cs_assert << 16);

  // CSROFF = pcd_rd_cs_hold
  // CSWOFF = pcd_wr_cs_hold
  constexpr unsigned int mcx_cson_delay = ns_to_bclk (4);

  static_assert (mcx_rd_cs_hold <= 7, "");
  static_assert (mcx_wr_cs_hold <= 7, "");
  static_assert (mcx_cson_delay <= 7, "");

  dev::rx_bsc::cs2wcr2 = mcx_rd_cs_hold | (mcx_wr_cs_hold << 4) | (mcx_cson_delay << 28)
		     | (mcx_rdon_delay << 16) | (mcx_wron_delay << 20);

  // no recovery cycles
  dev::rx_bsc::cs2rec = 0;

  // enable operation, 16 bit bus, explicit little endian, MPX disable
  dev::rx_bsc::cs2cr = 0x0001 | (utils::native_byte_order () != utils::little_endian
			     ? (1 << 8) : 0);

#endif // defined (MCB_USE_MCX514)

  // ---------------------------------------------------------------------------

  // enable all recovery cycle types.
  dev::rx_bsc::csrecen = 0xFFFF;

  // bus error monitoring (disable all)
  // dev::rx_bsc::beren = 0; reset default

  // bus priority (all buses fixed priority)
  // dev::rx_bsc::buspri = 0x0000; reset default

  // CS0 pin = disable, CS1 pin = enable, CS2 pin = enable
  dev::rx_gpio::pfcse = 0b00000110;
 
  // CS/external bus pin select
  dev::rx_gpio::pfcss0 = 0b00'10'10'00;
  // dev::rx_gpio::pfcss1 = 0b00'00'00'00;  reset default
  // dev::pfaoe0 = 0b00000000;  reset default
  // dev::pfaoe1 = 0b00000000;  reset default
  dev::rx_gpio::pfbcr0 = 0b00010001;
  dev::rx_gpio::pfbcr1 = 0b00000011;

  // on-chip ROM enable, external bus enable
  dev::rx_sys::syscr0 = 0x5A00 | 0b00000011;

  // after this init function the board constructor will be ran, which
  // will initialize all the other devices (board member variables).

#endif // MCB_NO_RESET_HW_INIT
}

[[gnu::cold]]
mcb_v1_board::release_peripheral_reset_t
::release_peripheral_reset_t (mcb_v1_board& brd)
{
#ifndef MCB_NO_PERIPHERAL_RESET_RELEASE
  // toggle the peripheral reset a couple of times.  this is for the LAN8710
  // PHY, which shows some problems during reset after power-on.
  // if the reset toggling is not done, the green indicator led will be
  // inverted.  maybe there are also other subtile things that fail to
  // initialize properly.
  // this is most likely caused by a bug in the PERI_RST line hardware design.
  // during power-on, PERI_RST is not held low properly.
  // it shortly rises to about 1.2V and then drops to 0V.  this pulse seems
  // to be a problem for the LAN8710.
  for (unsigned int i = 0b01010'0; i != 0; )
  {
    i >>= 1;
    set_peripheral_reset (i & 1);
    std::this_thread::sleep_for (std::chrono::microseconds (500));
  }

  std::this_thread::sleep_for (std::chrono::milliseconds (100));
#endif // MCB_NO_PERIPHERAL_RESET_RELEASE
}

[[gnu::cold]]
const std::array<uint8_t, 6>
mcb_v1_board::board_info_eth0_addr::operator () (void)
{
  auto&& addr = mcb_v1_board_info::inst ().ifconfigs ()[0].hw_addr ();
  return (const std::array<uint8_t, 6>&)addr;
}


mcb_v1_board::mcb_v1_board (void)
 : devices_begin ()

 , reset_hw_init (*this)
 , system_timer ()
 , release_peripheral_reset (*this)

#if defined (MCB_USE_LEDS)
  , led_outputs ({0b0000})
#endif

#if defined (MCB_USE_PCA9698)
 , pca9698_i0 (internal_i2c)
 , pca9698_i1 (internal_i2c)
#endif

#if defined (MCB_USE_PCD4641)
#endif

#if defined (MCB_USE_MCX514)
 , mcx51x (&mcx51x_axis_inputs, &mcx51x_axis_outputs)
 , mcx51x_axis_inputs {{
	{ mcx51x.axis (0), pca9698_i0 },
	{ mcx51x.axis (1), pca9698_i0 },
	{ mcx51x.axis (2), pca9698_i0 },
	{ mcx51x.axis (3), pca9698_i0 } }}
 , mcx51x_axis_outputs {{
	{ mcx51x.axis (0), pca9698_i1, 0 },
	{ mcx51x.axis (1), pca9698_i1, 1 },
	{ mcx51x.axis (2), pca9698_i1, 2 },
	{ mcx51x.axis (3), pca9698_i1, 3 } }}
#endif

#if defined (MCB_USE_MCX514_I2C_EXT)
 , mcx514_e0 (mcx514_e0_t::bus_interface (external_i2c))
#endif

#if defined (MCB_USE_PCA9698_I2C_EXT)
  , pca9698_e0 (external_i2c)
  , pca9698_e1 (external_i2c)
#endif

#if defined (MCB_USE_FCU)
  // fcu expects clock in MHz (rounding up).
 , fcu ((pclock_hz + 1'000'000-1) / 1'000'000)
#endif

#if defined (MCB_USE_ETHERC) || defined (MCB_USE_ETHERC_RAW)
 , eth0_phy (mdio_sta)
 , eth0_mac (eth0_etherc)
#endif

#if defined (MCB_USE_ETHERC) && !defined (MCB_USE_ETHERC_RAW)
 , eth0 (eth0_mac, eth0_phy)
#endif

#if defined (MCB_USE_DAC124S085)
 , dac0 (internal_spi)
 , dac1 (internal_spi)
 , dac2 (internal_spi)
#endif

#if defined (MCB_USE_DIPSWITCH)
 , dipswitch_inputs (pca9698_i1)
#endif

#if defined (MCB_USE_DIGITAL_IO)
 , digital_inputs (pca9698_i0)
 , digital_outputs (pca9698_i1)
#endif

 , devices_end ()
{

#if defined (MCB_USE_FCU)
  static_assert (pclock_hz % 1'000'000 == 0, "");
  static_assert (pclock_hz / 1'000'000 >= 4, "");
  static_assert (pclock_hz / 1'000'000 <= 50, "");
#endif


#if defined (MCB_USE_PCA9698)

  // U23 is all inputs
  pca9698_i0.set_io_ports_config (0b11111111'11111111'11111111'11111111'11111111LL);

  // U24 is all totem-pole outputs, except bank 4 which is inputs
  pca9698_i1.set_output_config (pca9698_i1_t::output_config_t ()
				.set_port_0_open_drain (false)
				.set_port_1_open_drain (false)
				.set_port_2_open_drain (false)
				.set_port_3_open_drain (false)
				.set_port_4_open_drain (false));
  pca9698_i1.set_io_ports_config (0b11111111'00000000'00000000'00000000'00000000LL);

#endif

#if defined (MCB_USE_PCA9698_I2C_EXT)

  // U23 is all inputs
  pca9698_e0.set_io_ports_config (0b11111111'11111111'11111111'11111111'11111111LL);

  // U24 is all totem-pole outputs
  pca9698_e1.set_output_config (pca9698_e1_t::output_config_t ()
				.set_port_0_open_drain (false)
				.set_port_1_open_drain (false)
				.set_port_2_open_drain (false)
				.set_port_3_open_drain (false)
				.set_port_4_open_drain (false));

  // U24 is all outputs, except bank 4
  pca9698_e1.set_io_ports_config (0b11111111'00000000'00000000'00000000'00000000LL);

#endif


#if defined (MCB_USE_PCD4641)

  // the only additional axis IOs that are wired are OTS output as amp enable
  // signal and the regular STP input as amp alarm signal.
  // the configurable P1,P2,P3,P4 are not used so there is nothing to do here.

#endif

#if defined (MCB_USE_MCX514)

  // all MCX514 axis GPIOs are wired as outputs.
  for (auto& a : mcx51x.axes ())
    a.set_pio1 (dev::mcx51x::pio_mode1_t ()
		.set_pin (0, dev::mcx51x::general_output)
		.set_pin (1, dev::mcx51x::general_output)
		.set_pin (2, dev::mcx51x::general_output)
		.set_pin (3, dev::mcx51x::general_output)
		.set_pin (4, dev::mcx51x::general_output)
		.set_pin (5, dev::mcx51x::general_output)
		.set_pin (6, dev::mcx51x::general_output)
		.set_pin (7, dev::mcx51x::general_output));

#endif

#if defined (MCB_USE_ETHERC) && !defined (MCB_USE_ETHERC_RAW)

  eth0.set_mac_address (eth0.default_mac_address ());
  eth0.reset ();

#endif

}


void
mcb_v1_board::set_peripheral_reset (bool val)
{
  // the peripheral reset line is an inverting output via Q5.
  // when not driven, it's a pull-down (i.e. reset enabled).
  // val = true: reset line low -> output high
  // val = false: reset line high -> output low
  // on port9 there are also some other outputs, so we need to use atomic
  // bit ops to avoid race conditions with interrupts.
  //
  // P90:  SCI7 TX
  // P91:  SCI7 RS485 transmit enable
  // P92:  SCI7 RX
  // P93:  PERI_RST (output)
  // P94:  -/-
  // P95:  -/-
  // P96:  -/-
  // P97:  -/-

  static constexpr dev::hw_reg_rw <uint8_t, dev::const_addr<0x0008C029>> port9 = { };
  utils::atomic_set_bit (val, 3, &port9);
}

// -----------------------------------------------------------------------------

using reset_source_1 = reset_source;

[[gnu::cold]] reset_source_1
mcb_v1_board::reset_source (void)
{
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C290>> RSTSR0 = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C291>> RSTSR1 = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x000800C0>> RSTSR2 = { };

  const uint8_t RSTSR2_val = RSTSR2;
  const uint8_t RSTSR1_val = RSTSR1;
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

  if ((RSTSR1_val & RSTSR1_CWSF) == 0)
  {
    // the CWSF bit is cleared to 0 on power-on.  so we set it to 1 for the
    // next reset.
    RSTSR1 = RSTSR1_val | RSTSR1_CWSF;
    return reset_source_1::poweron;
  }

  return reset_source_1::hard_reset;
}

// on RX on-chip RAM starts at address 0 and the first 256 bytes are reserved
// by the linker script.
// if the RX memory protection is ever to be used for nullptr access detection,
// have to disable it when accessing the variable.

int mcb_v1_board::reset_counter (void)
{
  return *(volatile int*)32;
}

void mcb_v1_board::set_reset_counter (int val)
{
  *(volatile int*)32 = val;
}


// -----------------------------------------------------------------------------

void mcb_v1_board::exec (void)
{
  auto cur_time = std::chrono::high_resolution_clock::now ();

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
  return mcb_v1_board::inst ().system_timer.current_time_ticks ();
}

} } }


// -----------------------------------------------------------------------------

static volatile bool g_in_assert_abort = false;

void board_debug_uart_write (const void* data, unsigned int byte_count)
{
#if defined (MCB_USE_SCI_DEBUG)
  auto& sci = mcb_v1_board::inst ().debug_usart;

  if (!g_in_assert_abort)
    sci.write (data, byte_count);
  else
  {
    const char* str = (const char*)data;

    for (unsigned int i = 0; i < byte_count; ++i)
    {
      while (! sci.status ().transmit_end ()) { }

      char c = *str++;

      sci.set_control (sci.control ().set_transmit_enable ());
      sci.set_transmit_data (c);
    }
  }
#endif
}

static void
emergency_puts (const char* str, unsigned int max_count)
{
  write (STDOUT_FILENO, str, max_count);
}
static void
emergency_puts (const char* str)
{
  write (STDERR_FILENO, str, std::strlen (str));
}

[[gnu::weak]] void finalize_assert_abort_handler (void)
{
}

[[noreturn, gnu::noinline, gnu::cold]]
void
int_assert (const char* source_filename, int linenum,
            const char* func_name, const char* expr)
{
  // all LEDs on
  g_in_assert_abort = true;

  dev::led_outputs led_out ({0b1001});

#if defined (MCB_USE_SCI_DEBUG)
  auto& sci = mcb_v1_board::inst ().debug_usart;

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

  finalize_assert_abort_handler ();

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
#ifndef BOARD_ASSERT_MSG_NO_FUNCNAME
  static const char* int_assert_func_name;
#endif
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

  #ifndef BOARD_ASSERT_MSG_NO_FUNCNAME
    int_assert_func_name = func_name;
  #endif

  int_assert_expr = expr;

  __builtin_rx_int (1);

  while (true) { }
}

static void INT_Assert (void)
{
  int_assert (int_assert_source_filename, int_assert_linenum,
#ifndef BOARD_ASSERT_MSG_NO_FUNCNAME
  int_assert_func_name,
#else
  ""
#endif
 , int_assert_expr);
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
};


static constexpr auto rvectors [[gnu::section (".rvectors"), gnu::used]] =

utils::convert_array<isr_desc, isr_func> (
utils::array_verify_unique (
utils::sort_array (
utils::replace_array_elements<isr_desc, isr_desc::index_equals> (
utils::make_array<isr_desc, dev::rx63_interrupt::count, isr_desc::init_empty> (),
utils::array_verify_unique (
utils::sort_array (
utils::partition_array<isr_desc, isr_desc::not_unconnected_num> (
utils::make_array (

  // FIXME: maybe put all instantiated devices into a std::tuple
  // and then iterate over all tuple elements and get an array of isrs from
  // each element and construct the initial isr array from that.

  #if defined (MCB_USE_SCI0_RS232C)
  make_isr_desc<mcb_v1_board::rs232c_t::txi_isr_t> (),
  make_isr_desc<mcb_v1_board::rs232c_t::rxi_isr_t> (),
  make_isr_desc<mcb_v1_board::rs232c_t::tei_isr_t> (),
  make_isr_desc<mcb_v1_board::rs232c_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_RS485)
  make_isr_desc<mcb_v1_board::rs485_ch1_t::txi_isr_t> (),
  make_isr_desc<mcb_v1_board::rs485_ch1_t::rxi_isr_t> (),
  make_isr_desc<mcb_v1_board::rs485_ch1_t::tei_isr_t> (),
  make_isr_desc<mcb_v1_board::rs485_ch1_t::eri_isr_t> (),

  make_isr_desc<mcb_v1_board::rs485_ch2_t::txi_isr_t> (),
  make_isr_desc<mcb_v1_board::rs485_ch2_t::rxi_isr_t> (),
  make_isr_desc<mcb_v1_board::rs485_ch2_t::tei_isr_t> (),
  make_isr_desc<mcb_v1_board::rs485_ch2_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_MCX514)
  make_isr_desc<mcb_v1_board::mcx51x_t::isr0_t> (),
  make_isr_desc<mcb_v1_board::mcx51x_t::isr1_t> (),
  #endif

  #if defined (MCB_USE_PCD4641)
  make_isr_desc<mcb_v1_board::pcd4641_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_SCI_DEBUG)
  make_isr_desc<mcb_v1_board::debug_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v1_board::debug_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v1_board::debug_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v1_board::debug_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_I2C)
  make_isr_desc<mcb_v1_board::internal_i2c_t::txi_isr_t> (),
  make_isr_desc<mcb_v1_board::internal_i2c_t::rxi_isr_t> (),
  make_isr_desc<mcb_v1_board::internal_i2c_t::tei_isr_t> (),
  make_isr_desc<mcb_v1_board::internal_i2c_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SPI)
  make_isr_desc<mcb_v1_board::internal_spi_t::txi_isr_t> (),
  make_isr_desc<mcb_v1_board::internal_spi_t::rxi_isr_t> (),
  make_isr_desc<mcb_v1_board::internal_spi_t::tei_isr_t> (),
  make_isr_desc<mcb_v1_board::internal_spi_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_MCX514_I2C_EXT)
  make_isr_desc<mcb_v1_board::external_i2c_t::txi_isr_t> (),
  make_isr_desc<mcb_v1_board::external_i2c_t::rxi_isr_t> (),
  make_isr_desc<mcb_v1_board::external_i2c_t::tei_isr_t> (),
  make_isr_desc<mcb_v1_board::external_i2c_t::eri_isr_t> (),
  #endif


  #if defined (MCB_USE_ETHERC)
  make_isr_desc<mcb_v1_board::rx_edmac_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_TRIGGER_IO)
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr0_t> (),
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr1_t> (),
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr2_t> (),
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr3_t> (),
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr4_t> (),
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr5_t> (),
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr6_t> (),
  make_isr_desc<mcb_v1_board::trigger_inputs_t::isr7_t> (),
  #endif

  #if defined (MCB_USE_TPU)
  make_isr_desc<mcb_v1_board::tpu0_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu0_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu0_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu0_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu0_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu0_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v1_board::tpu1_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu1_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu1_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu1_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu1_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu1_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v1_board::tpu2_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu2_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu2_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu2_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu2_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu2_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v1_board::tpu3_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu3_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu3_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu3_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu3_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu3_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v1_board::tpu4_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu4_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu4_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu4_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu4_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu4_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v1_board::tpu5_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu5_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu5_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu5_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu5_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v1_board::tpu5_t::tci_u_isr_t> (),
  #endif

  make_isr_desc<mcb_v1_board::system_timer_t::isr0_t> (),
  make_isr_desc<mcb_v1_board::system_timer_t::isr1_t> (),

  make_isr_desc<dev::interrupt::connected_isr<dev::rx63_interrupt::line<dev::rx63_interrupt::reserved_0>,
		dev::interrupt::func<decltype (&INT_Excep_BRK), &INT_Excep_BRK>>> (),

  make_isr_desc<dev::interrupt::connected_isr<dev::rx63_interrupt::line<dev::rx63_interrupt::reserved_1>,
		dev::interrupt::func<decltype (&INT_Assert), &INT_Assert>>> ()

))))))));

// -----------------------------------------------------------------------------

#if defined (MCB_USE_DTC)
alignas (1024*4) mcb_v1_board::dtc_t::vector_table_t g_dtc_vector_table;

mcb_v1_board::dtc_t::vector_table_t&
mcb_v1_board::dtc_vector_table_inst (void) { return g_dtc_vector_table; }
#endif // MCB_USE_DTC

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

[[gnu::cold]] const mcb_v1_board_info& mcb_v1_board_info::inst (void)
{
  // this is RX63N specific.  the fixed vector table in the boot loader
  // contains a pointer to the actual board_info data which usually resides
  // in user boot flash image.
  // the default renesas usb boot loader contains 0xFFFFFFFF at that place.
  // in any case, make sure that the address in the range of the user boot
  // flash area.
  uintptr_t bi = *(const uintptr_t*)0xFF7FFF84;

  if (bi >= 0xFF7FC000 && bi <= 0xFF7FFFFF)
    return *(const mcb_v1_board_info*)bi;

  return *(const mcb_v1_board_info*)&board_info_data;
}

// =============================================================================


void mcb_v1_board::reset_to_func (void (*func)(void))
{
  dev::this_cpu::save_disable_interrupts ();

  // depending on which devices have been used, we might need a proper
  // shutdown of the drivers to stop the interrupts etc.
  // otherwise the new program might malfunction because of some unexpected
  // hardware event.
  this->~mcb_v1_board ();
  set_peripheral_reset (true);

  func ();
}

void mcb_v1_board::reset (void)
{
  // SWRR software reset register
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x000800C2>> SWRR = { };
  SWRR = 0xA501;
}


[[gnu::cold]] void mcb_v1_board::maybe_reset_config_to_factory_default (void)
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
// 0xFFFFFF90 | 0xFF7FFF90  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFF94 | 0xFF7FFF94  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFF98 | 0xFF7FFF98  | Reserved
  (fp)0x00000000,

//------------+-------------+--------------------------------------
// 0xFFFFFF9C | 0xFF7FFF9C  | ROM Code Protection

  (fp)0x00000000,   // user area / user boot area r/w access prohibited
//  (fp)0x00000001,   // user area / user boot area r access prohibited
//  (fp)0xFFFFFFFF,     // no restriction

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

