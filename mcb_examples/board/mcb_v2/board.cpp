
/*

GP output ports

                  | 144 pin         | 176 pin
------------------+-----------------+------------------
LD4               |           P07
------------------+-----------------+------------------
LD3               |           P05
------------------+-----------------+------------------
LD2               |           P03
------------------+-----------------+------------------
LD1               |           P02
------------------+-----------------+------------------
RS485_CH0_DE      |           PJ5 (exclusive)
------------------+-----------------+------------------
SCI2_EXT_CS       |           P86 (exclusive)
------------------+-----------------+------------------
LD_COL_SEL        | P56 (exclusive) | P11 (exclusive)
------------------+-----------------+------------------
SCI5_EXT_CS       |           PC0 (exclusive)
------------------+-----------------+------------------
CAN_RS            |           P73
------------------+-----------------+------------------
RS485_CH1_DE      |           PB4 (exclusive)
------------------+-----------------+------------------
PERI_RST          |           P70
------------------+-----------------+------------------
DAC_CS1N          |           P93 (excl)
------------------+-----------------+------------------
DAC_CS2N          |   P92 (excl)    | P96 (excl)
------------------+-----------------+------------------
DAC_CS3N          |   P60 (excl)    | P97 (excl)
------------------+-----------------+------------------
SCI4_EXT_CS       |           P90 (excl)
------------------+-----------------+------------------
BAT_LTEST_EN      |        --       |  PG5
------------------+-----------------+------------------
BAT_VTEST_EN      |        --       |  PG6
------------------+-----------------+------------------
BAT_DCDC_EN       |        --       |  PG7



Clock timer output ports

                  | 144 pin         | 176 pin
------------------+-----------------+------------------
MTIOC3C (MCX)     |           PJ3
------------------+-----------------+------------------
MTIOC4C (USB HUB) |      --         | P87
------------------+-----------------+------------------
BCLK (test point) |           P53
------------------+-----------------+------------------
MTIOC2A (PCD)     |           PB5
------------------+-----------------+------------------


GP input ports

                  | 144 pin         | 176 pin
------------------+-----------------+------------------
DIPSW8            |           MD/FINED
------------------+-----------------+------------------
DIPSW6            |           P35 / NMI
------------------+-----------------+------------------
DIPSW7 / UB       |           PC7
------------------+-----------------+------------------
DIPSW4            |           PC1
------------------+-----------------+------------------
DIPSW3            |           PA7 (shared with unused A7)
------------------+-----------------+------------------
DIPSW2            |           PA6 (shared with unused A6)
------------------+-----------------+------------------
DIPSW1            |           PA5 (shared with unused A5)
------------------+-----------------+------------------
P4[0..7]          |           trigger inputs
------------------+-----------------+------------------
#NAND_BSY         |      --         | PG2



interrupt inputs

                  | 144 pin         | 176 pin
------------------+-----------------+------------------
IRQ0 (MCX INT1)   |      --         | P10
------------------+-----------------+------------------
IRQ1 (unused)     |
------------------+-----------------+------------------
IRQ2 (EMG)        |           P32
------------------+-----------------+------------------
IRQ3 (MCX INT0)   |           P33
------------------+-----------------+------------------
IRQ4 (LAN9250)    |           PF5
------------------+-----------------+------------------
IRQ5 (MCX INT1)   |     P15         | --
------------------+-----------------+------------------
IRQ6 (VBUS/EXP)   |           P16
------------------+-----------------+------------------
IRQ7 (PCD)        |           P17
------------------+-----------------+------------------
IRQ[8..15] (TRG)  |           P4[0..7]
------------------+-----------------+------------------


before MCX reset release:
  - setup timings for CS areas
  - setup clock output on MCX

after MCX reset release:
  - configure GPIO pins for each axis according to wiring of the board.

before PCD reset release:
  - make sure that RX bus RD and WR pull-up is enabled.
    PCD uses RD and WR levels after reset to set the bus mode (8 bit parallel or SPI)
  - setup timings for CS areas
  - setup clock output on PCD

after PCD reset release:
  - configure GPIO pins for each axis according to wiring of the board.


if using ethernet, enable the PHYs.  after power-on/reset they will be in
power-down mode.

LAN9250 will always come up after PERI_RST is released.  if not needed or
desired, turn it off.

if using CAN, select CAN_RS line accordingly.

make sure not to enable NMI, as the pin is used as an input pin.
NMI can only be enabled once and can't be disabled in software afterwards.
only reset can disable NMI again.  so take care.

*/


#include <cstdio>
#include <chrono>
#include <thread>

#include <sys/unistd.h>

#include <board/board.hpp>
#include <board/board_info.hpp>

#include <utils/rodata.hpp>
#include <utils/text.hpp>

#include <dev/cpu.hpp>
#include <dev/renesas/rx_bsc.hpp>
#include <dev/renesas/rx_clk.hpp>
#include <dev/renesas/rx_sys.hpp>



static constexpr bool cfg_use_ext_ad_bus =
#if defined (MCB_USE_PCD4641) \
      || defined (MCB_USE_MCX512) || defined (MCB_USE_MCX514) \
      || defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW) \
      || defined (MCB_USE_TRIGGER_IO) \
      || defined (MCB_USE_DIGITAL_IO) \
      || defined (MCB_USE_NAND_FLASH)
  true;
#else
  false;
#endif

#ifdef MCB_USE_RS485
  static constexpr bool cfg_use_rs485 = true;
#else
  static constexpr bool cfg_use_rs485 = false;
#endif

#ifdef MCB_USE_DAC124S085
  static constexpr bool cfg_use_dac1245 = true;
#else
  static constexpr bool cfg_use_dac1245 = false;
#endif

#ifdef MCB_USE_DIPSWITCH
  static constexpr bool cfg_use_dispswitch = true;
#else
  static constexpr bool cfg_use_dispswitch = false;
#endif

#ifdef MCB_USE_SCI0_USART
  static constexpr bool cfg_use_sci0 = true;
#else
  static constexpr bool cfg_use_sci0 = false;
#endif

#ifdef MCB_USE_SCI_DEBUG
  static constexpr bool cfg_use_sci1 = true;
#else
  static constexpr bool cfg_use_sci1 = false;
#endif

#ifdef MCB_USE_SCI2_USART
  static constexpr bool cfg_use_sci2 = true;
#else
  static constexpr bool cfg_use_sci2 = false;
#endif

#ifdef MCB_USE_SCI3_USART
  static constexpr bool cfg_use_sci3 = true;
#else
  static constexpr bool cfg_use_sci3 = false;
#endif

#if defined (MCB_USE_SCI4_USART) || defined (MCB_USE_SPI)
  static constexpr bool cfg_use_sci4 = true;
#else
  static constexpr bool cfg_use_sci4 = false;
#endif

#ifdef MCB_USE_SCI5_USART
  static constexpr bool cfg_use_sci5 = true;
#else
  static constexpr bool cfg_use_sci5 = false;
#endif

#ifdef MCB_USE_CAN
  static constexpr bool cfg_use_can = true;
#else
  static constexpr bool cfg_use_can = false;
#endif

#ifdef MCB_USE_USB_FUNCTION
  static constexpr bool cfg_use_usb0 = true;
#else
  static constexpr bool cfg_use_usb0 = false;
#endif

#ifdef MCB_USE_USB_HOST
  static constexpr bool cfg_use_usba = true;
#else
  static constexpr bool cfg_use_usba = false;
#endif

#if defined (MCB_USE_MCX514) || defined (MCB_USE_MCX512)
  #define MCB_USE_MCX51x
  static constexpr bool cfg_use_mcx = true;
#else
  #undef MCB_USE_MCX51x
  static constexpr bool cfg_use_mcx = false;
#endif

#ifdef MCB_USE_PCD4641
  static constexpr bool cfg_use_pcd = true;
#else
  static constexpr bool cfg_use_pcd = false;
#endif

#ifdef MCB_USE_TRIGGER_IO
  static constexpr bool cfg_use_trigger_io = true;
#else
  static constexpr bool cfg_use_trigger_io = false;
#endif

#ifdef MCB_USE_EMG_STOP
  static constexpr bool cfg_use_emg_stop = true;
#else
  static constexpr bool cfg_use_emg_stop = false;
#endif

#ifdef MCB_USE_DIPSWITCH
  static constexpr bool cfg_use_dipswitch = true;
#else
  static constexpr bool cfg_use_dipswitch = false;
#endif

#if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC0_RAW)
  static constexpr bool cfg_use_etherc0 = true;
#else
  static constexpr bool cfg_use_etherc0 = false;
#endif

#if defined (MCB_USE_RXxxx_176) && (defined (MCB_USE_RX_ETHERC1) || defined (MCB_USE_RX_ETHERC1_RAW))
  static constexpr bool cfg_use_etherc1 = true;
#else
  static constexpr bool cfg_use_etherc1 = false;
#endif

#if defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW)
  static constexpr bool cfg_use_lan9250 = true;
#else
  static constexpr bool cfg_use_lan9250 = false;
#endif


static void emergency_puts (const char* str, unsigned int max_count);
static void emergency_puts (const char* str);


static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C426>> RCR3 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C427>> RCR4 = { };

// static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0087581>> NMIER = { };

static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x000800A0>> OPCCR = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C282>> DPSIER0 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C283>> DPSIER1 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C284>> DPSIER2 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C285>> DPSIER3 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C286>> DPSIFR0 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C287>> DPSIFR1 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C288>> DPSIFR2 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C289>> DPSIFR3 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C28A>> DPSIEGR0 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C28B>> DPSIEGR1 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C28C>> DPSIEGR2 = { };
static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C28D>> DPSIEGR3 = { };


// global freestanding function, like standard main, can be overridden by
// user code to do something useful when entering standby mode, like persisting
// state into flash memory or standby ram.
extern int enter_standby_mode (void);

[[gnu::weak]] int enter_standby_mode (void) { return 0; }


static bool g_system_timer_initialized;

// make sure that the board constructor is ran as the very first static
// initializer by specifying the init_priority attribute.
[[gnu::init_priority (101), gnu::used]] mcb_v2_board mcb_v2_board::g_inst;


#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144) || defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)
  #define MCB_USE_RXxxx_144
#elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
  #define MCB_USE_RXxxx_176
#else
  #error mcu type
#endif

[[noreturn, gnu::noinline]]
static void board_info_check_ng (void)
{
  while (true) { }
}

[[gnu::cold, gnu::noinline]]
mcb_v2_board::board_id_t mcb_v2_board::board_id (void) const
{
  auto&& bi = mcb_v2_board_info::inst ();

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

#if defined (MCB_USE_RX631_144)
  r[2] = '6';
  r[3] = '3';
  r[4] = '1';
  r[5] = 1;

  static_assert (r.size () >= 16 + 6);
  std::copy_n ((volatile const uint8_t*)0xFEFFFAC0, 16, r.data () + 6);

#elif defined (MCB_USE_RX63N_144)
  r[2] = '6';
  r[3] = '3';
  r[4] = 'N';
  r[5] = 1;

  static_assert (r.size () >= 16 + 6);
  std::copy_n ((volatile const uint8_t*)0xFEFFFAC0, 16, r.data () + 6);

#elif defined (MCB_USE_RX64M_144)
  r[2] = '6';
  r[3] = '4';
  r[4] = 'M';
  r[5] = 1;
  r[6] = 4;
  r[7] = 4;

  static_assert (r.size () >= 12 + 8);
  uint32_t* r_ptr = (uint32_t*)(r.data () + 8);
  *r_ptr++ = *(volatile const uint32_t*)0x007FB174;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E4;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E8;

#elif defined (MCB_USE_RX64M_176)
  r[2] = '6';
  r[3] = '4';
  r[4] = 'M';
  r[5] = 1;
  r[6] = 7;
  r[7] = 6;

  static_assert (r.size () >= 12 + 8);
  uint32_t* r_ptr = (uint32_t*)(r.data () + 8);
  *r_ptr++ = *(volatile const uint32_t*)0x007FB174;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E4;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E8;

#elif defined (MCB_USE_RX71M_144)
  r[2] = '7';
  r[3] = '1';
  r[4] = 'M';
  r[5] = 1;
  r[6] = 4;
  r[7] = 4;

  static_assert (r.size () >= 12 + 8);
  uint32_t* r_ptr = (uint32_t*)(r.data () + 8);
  *r_ptr++ = *(volatile const uint32_t*)0x007FB174;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E4;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E8;

#elif defined (MCB_USE_RX71M_176)
  r[2] = '7';
  r[3] = '1';
  r[4] = 'M';
  r[5] = 1;
  r[6] = 7;
  r[7] = 6;

  static_assert (r.size () >= 12 + 8);
  uint32_t* r_ptr = (uint32_t*)(r.data () + 8);
  *r_ptr++ = *(volatile const uint32_t*)0x007FB174;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E4;
  *r_ptr++ = *(volatile const uint32_t*)0x007FB1E8;

#else
  #error mcu type
#endif

  return r;
}

[[gnu::cold]]
mcb_v2_board::reset_hw_init_t::reset_hw_init_t (mcb_v2_board& brd)
{
#ifndef MCB_NO_RESET_HW_INIT

  #if 1
  {
    // check critical board features which might cause severe malfunction
    // or damage.  some other hardware features can silently fail if they
    // they are unchecked.  for example if the standby battery supply
    // is not there the board will simply power off and fail to transition
    // to standby mode.  it will not cause any damage.
    // however if the software is flashed onto the wrong RX MCU or board ID
    // there might be some issues with wrong pin configs and cause hardware
    // damage.

    auto&& bi = mcb_v2_board_info::inst ();

    if (bi.board_id () != 0x00000010)
      board_info_check_ng ();

    #ifdef MCB_USE_RX631_144
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::rx631_144))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_RX63N_144
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::rx63n_144))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_RX64M_144
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::rx64m_144))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_RX64M_176
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::rx64m_176))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_RX71M_144
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::rx71m_144))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_RX71M_176
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::rx71m_176))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_PCD4641
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::pcd4641))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_MCX514
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::mcx514))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_MCX512
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::mcx512))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_USB_FUNCTION
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::usb0_b))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_USB_HOST
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::usba_a))
	board_info_check_ng ();
    #endif

    #if defined (MCB_USE_RX_ETHERC0) || (MCB_USE_RX_ETHERC0_RAW)
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::eth0))
	board_info_check_ng ();
    #endif

    #if defined (MCB_USE_RX_ETHERC1) || (MCB_USE_RX_ETHERC1_RAW)
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::eth1))
	board_info_check_ng ();
    #endif

    #if defined (MCB_USE_LAN9250) || (MCB_USE_LAN9250_RAW)
      if (!bi.features ().test (mcb_v2_board_info::feature_bit::eth2))
	board_info_check_ng ();
    #endif

    #ifdef MCB_USE_NAND_FLASH
      if ((bi.features () & mcb_v2_board_info::feature_bits ()
				.set (mcb_v2_board_info::feature_bit::nand_flash_128m)
				.set (mcb_v2_board_info::feature_bit::nand_flash_256m)
				.set (mcb_v2_board_info::feature_bit::nand_flash_512m)
				.set (mcb_v2_board_info::feature_bit::nand_flash_1g)
				.set (mcb_v2_board_info::feature_bit::nand_flash_2g)
				.set (mcb_v2_board_info::feature_bit::nand_flash_4g)
				.set (mcb_v2_board_info::feature_bit::nand_flash_8g)).none ())
	board_info_check_ng ();
    #endif
  }
  #endif

  g_system_timer_initialized = false;

  dev::rx_sys::disable_register_protection ();

  // FIXME: use register init maps/scripts to reduce code size of the
  // initialization function.

  // before switching any GPIOs into output mode, make sure they have the
  // appropriate default values for the board.
  // this works for all board variants.
  dev::rx_gpio::podr::p0 = 0xFF;  // all LEDs off

  #ifdef MCB_USE_RXxxx_144
    if (cfg_use_dac1245)
      dev::rx_gpio::podr::p6 = 0xFF; // DAC_CS3N
  #endif

  dev::rx_gpio::podr::p7 = 0xFE;  // PERI_RST = on (low), CAN_RS = low power (high)

  if (cfg_use_sci2)
    dev::rx_gpio::podr::p8 = 0xFF;  // SCI2_EXT_CS off (high)

  if (cfg_use_dac1245 || cfg_use_sci4)
    dev::rx_gpio::podr::p9 = 0xFF;  // DAC_CSxN, SCI4_EXT_CS = off (high)

  if (cfg_use_sci5)
    dev::rx_gpio::podr::pc = 0xFF;  // SCI5_EXT_CS off (high)

  // battery standby circuit is only supported for 176 pin MCU
  // even if built without standby support, need to init these if the
  // circuit is present.
  // PG5 = BAT_LTEST_EN = off
  // PG6 = BAT_VTEST_EN = off
  // PG7 = BAT_DCDC_EN = off (use +3.3V supply as VCC_MCU)
  #ifdef MCB_USE_RXxxx_176
    #ifdef MCB_INVERTED_BAT_LTEST_EN_BAT_VTEST_EN
      dev::rx_gpio::podr::pg = 0b0110'0000;
    #else
      dev::rx_gpio::podr::pg = 0b0000'0000;
    #endif

    dev::rx_gpio::pdr::pg = 0b1110'0000;
  #endif


  // if there is no standby power supply, there is no sub-oscillator crystal.
  // we need to stop the sub-oscillator, as some of its control bits are
  // undefined after reset.  to set the oscillator drive level, the oscillator
  // needs to be stopped first.
  // this code works for RX63, RX64 and RX71.

  // enable the sub-oscillator early.  it will take a while to come up.
  // during that time, we can perform the other initialization steps and check
  // its status later.
  #if !defined (MCB_USE_STANDBY_POWER_SUPPLY)
    RCR3 = 0b0000'110'0;	// sub-osc stop, standard CL drive
    dev::rx_clk::sosccr = 1;	// sub-osc stop
  #else
  {
    RCR4 = 0b0000'0000;		// select sub-osc clock as RTC clock source
    RCR3 = 0b0000'001'1;	// sub-osc stop, low CL drive
				// drive strengh doesn't seem to make a difference
    dev::rx_clk::sosccr = 0;	// sub-osc start

    // use the LVDA temporarily to check that the VCC_MCU voltage is
    // above the battery supply level.  if it's not above the battery supply
    // level then the user might have ...
    // - plugged the battery before applying the main power and somehow
    //   the thing has started.  this should actually not happen normally.
    // - pushed the reset button in deep standby
    // - ...
    // in this case just lock-up and wait for main power and a hard reset.

    // FIXME: OFS1.VDSEL should be set to 2.94V to avoid a power-on when
    //        battery powered.
    mcb_v2_board::lvda_t lvda;

    lvda.set_lvd1_config (dev::rx_lvda::config_t ()
	.set_trigger (dev::rx_lvda::int_on_vcc_less)
	.set_threshold_voltage (dev::rx_lvda::v2_99)
	.set_noise_filter (dev::rx_lvda::filter_off)
	.set_voltage_monitor_enabled (true)
	.set_voltage_comparator_enabled (true));

    lvda.set_lvd1_enable ();

    if (lvda.lvd1_status ().vcc_less_than ())
      enter_deep_standby_mode ();
  }

  #endif

  // FIXME: before proceeding, pre-init the LVDA and check which power source
  //        we're on now.  if the user plugs in the battery first, it should
  //        not try to start the full system.  maybe in this case just hang
  //        around and wait until the voltage raises to 3.3V.

  // ----------------------------------------------------------
  // main clock source setup
  // the on-chip HOCO clock is not very precise.
  // because the MCU timers are used to generate the clocks for various
  // other devices, it's way better to always use an external crystal, which is
  // the default mode of operation.
  //
  // however, in standby mode clocks are not supplied to other devices and
  // the frequency precision does not matter so much.  in this case the HOCO
  // clock source might take less power from the battery.

  // dev::rx_gpio::pdr::p3 = 0b0000'0000;  default after reset
  static constexpr unsigned int init_pmr_p3_value = 0b1100'0000;  // P36 = EXTAL, P37 = XTAL
  dev::rx_gpio::pmr::p3 = init_pmr_p3_value;

#if defined (MCB_USE_RX63)

  // main clock oscillator wait time = 262144 cycles
  // dev::rx_clk::moscwtcr = 0x0E;  default after reset

  // sub-clock oscillator wait time = 262144 cycles.
  // dev::rx_clk::soscwtcr = 0x0E;  default after reset

  dev::rx_clk::mosccr = 0; // start main clock oscillator

  // wait for the main oscillator to become stable.
  while (dev::rx_clk::mosccr == 0x01) { }


  // PLL division ratio for main oscillator is x1/1

  constexpr unsigned int pll_in_clk = xtal_hz / 1;
  constexpr unsigned int pll_mult = (pll_hz / pll_in_clk) - 1;
  static_assert (dev::rx_clk::pllcr_stc_valid_rx63 (pll_mult), "");
  static_assert (dev::rx_clk::pll_freq_valid_rx63 (pll_hz), "");

  static_assert ((pll_in_clk * (pll_mult + 1)) / 1 == pll_hz, "");

  dev::rx_clk::pllcr = pll_mult << 8;

  // start PLL and wait for it to become stable
  dev::rx_clk::pllcr2 = 0;
  while (dev::rx_clk::pllcr2 == 1) { }

  // select clock divisors
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
	    (0b0001 << 0)
	  | (0b0001 << 4)
	  | (pclkb_div << 8)
	  | (pclka_div << 12)
	  | (bclk_div << 16)
	  | (1 << 22) // disable SDCLK pin output
	  | (0 << 23) // enable BCLK pin output
	  | (iclk_div << 24)
	  | (fclk_div << 28);

  static_assert (iclk_hz >= bclk_hz, "");

  constexpr auto uck_div = dev::rx_clk::sckcr_uck_div_rx63 (pll_hz / uck_hz);
  static_assert (uck_div != dev::rx_clk::sckcr_uck_div_invalid (), "");

  constexpr auto iebck_div = dev::rx_clk::sckcr_iebck_div_rx63 (pll_hz / iebck_hz);
  static_assert (iebck_div != dev::rx_clk::sckcr_uck_div_invalid (), "");

  dev::rx_clk::sckcr2 = (iebck_div << 0) | (uck_div << 4);

  // BCLK pin output = 1/2 bclk
  //dev::rx_clk::bckcr = 1;

  // use the PLL as the main system clock
  dev::rx_clk::sckcr3 = 0b100 << 8;




#elif defined (MCB_USE_RX64_RX71)

  // MOFCR = 0b00000001; // 0x00 is default after reset.
  // MOSCWTCR = 0x0E; // 0x53 is default after reset. not sure which value is good enough.
  dev::rx_clk::mosccr = 0; // start main clock oscillator

  // wait for the main oscillator to become stable.
  while ((dev::rx_clk::oscovfsr.read () & 0b00001) == 0) { }


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
    static_assert (dev::rx_clk::pllcr_stc_valid_rx64 (pll_mult), "");
    static_assert (dev::rx_clk::pll_freq_valid_rx64 (pll_hz), "");
    static_assert ((hoco_hz * (pll_mult + 1)) / 2 == pll_hz, "");

    dev::rx_clk::pllcr = (pll_mult << 8) | 0b0001'0000;  // select HOCO as PLL clock source
  #endif


  #ifdef USE_MAIN_OSC_CLOCK

    // PLL division ratio for main oscillator is x1/2

    constexpr unsigned int pll_in_clk = xtal_hz / 2;

    constexpr unsigned int pll_mult = ((pll_hz * 2) / pll_in_clk) - 1;
    static_assert (pll_mult >= 0b010011 && pll_mult <= 0b111011, "");
    static_assert (dev::rx_clk::pll_freq_valid_rx64 (pll_hz), "");
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

#else
  #error not implemented
#endif

  // ----------------------------------------------------------
  // IO pin config

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


  // set output port directions
  // notice that after reset all ports are set to inputs (PDR = 0x00)
  dev::rx_gpio::pdr::p0 = 0b1010'1100;  // P02 = LD1, P03 = LD2, P05 = LD3, P07 = LD4
  dev::rx_gpio::pdr::p7 = 0b0000'1001;  // P73 = CAN_RS, P70 = /PERI_RST

  if (cfg_use_sci2)
    dev::rx_gpio::pdr::p8 = 0b0100'0000;  // P86 = SCI2_EXT_CS

  if (cfg_use_rs485)
  {
    dev::rx_gpio::pdr::pb = 0b0001'0000;  // PB4 = RS485_CH1_DE
    dev::rx_gpio::pdr::pj = 0b0010'0000;  // PJ5 = RS485_CH0_DE
  }

  if (cfg_use_sci5)
    dev::rx_gpio::pdr::pc = 0b0000'0001;  // PC0 = SCI5_EXT_CS

  #if defined (MCB_USE_RXxxx_144)

//    dev::rx_gpio::pdr::p1 = 0b0000'0000;  // P1x = all inputs or special function
    dev::rx_gpio::pdr::p5 = 0b0100'0000;  // P56 = LD_COL_SEL

    if (cfg_use_dac1245)
    {
      dev::rx_gpio::pdr::p6 = 0b0000'0001;  // P60 = DAC_CS3N

      if (cfg_use_sci4)
        dev::rx_gpio::pdr::p9 = 0b0000'1101;  // P93 = DAC_CS1N, P92 = DAC_CS2N, P90 = SCI4_EXT_CS
      else
        dev::rx_gpio::pdr::p9 = 0b0000'1100;  // P93 = DAC_CS1N, P92 = DAC_CS2N
    }
    else if (cfg_use_sci4)
      dev::rx_gpio::pdr::p9 = 0b0000'0001;  // P90 = SCI4_EXT_CS


  #elif defined (MCB_USE_RXxxx_176)

    dev::rx_gpio::pdr::p1 = 0b0000'0010;  // P11 = LD_COL_SEL
    dev::rx_gpio::pdr::p5 = 0b0101'0000;  // Write 1 (output) to bits that correspond to ports 54 to 56 on the 176-pin product.
    dev::rx_gpio::pdr::p7 = 0b0000'1001;  // P73 = CAN_RS, P70 = /PERI_RST
    dev::rx_gpio::pdr::p8 = 0b0100'0000;  // P86 = SCI2_EXT_CS
    dev::rx_gpio::pdr::p9 = 0b1100'1001;  // P93 = DAC_CS1N, P96 = DAC_CS2N, P97 = DAC_CS3N, P90 = SCI4_EXT_CS
    dev::rx_gpio::pdr::pb = 0b0001'0000;  // PB4 = RS485_CH1_DE
    dev::rx_gpio::pdr::pc = 0b0000'0001;  // PC0 = SCI5_EXT_CS
  #else
    #error mcu type
  #endif

  if (cfg_use_ext_ad_bus)
  {
    // high-drive for bus WR, RD, BCLK signals.
    dev::rx_gpio::dscr::p5 = 0b0000'1101;

    // high-drive for bus data signals.
    dev::rx_gpio::dscr::pe = 0b1111'1111;
    dev::rx_gpio::dscr::pd = 0b1111'1111;

    // high-drive for bus address (A0, A1, A2, A3, A4) signals.
    dev::rx_gpio::dscr::pa = 0b0001'1111;

    // high-drive for CS7
    dev::rx_gpio::dscr::p6 = 0b1000'0000;
  }

  // select open drain output
  if (cfg_use_dipswitch)
    dev::rx_gpio::odr1::pa = 0b01'01'01'00;  // PA7, PA6, PA5 = NMOS open-drain (for dipswitch reading)

  // it's not needed to set MDIO to open-drain but it also doesn't harm.
  // both works OK.
  //  dev::rx_gpio::odr0::p7 = 0b00'00'01'00;  // P71 = ET0_MDIO

  // set pull-up resistors (no pull-ups on board)
  if (cfg_use_rs485)
  {
    dev::rx_gpio::pcr::p0 = 0b0000'0011;  // TXD6, RXD6
    dev::rx_gpio::pcr::pb = 0b1100'0000;  // TXD9, RXD9
  }

  // these pull-ups are not really needed.  after the bus setup they will
  // be totem pole driven by RX and during reset the pull-ups don't work anyway.
//  dev::rx_gpio::pcr::p5 = 0b0000'0101;  // WR, RD
//  dev::rx_gpio::pcr::p6 = 0b1111'1110;  // CS1, CS2, CS3, CS4, CS5, CS6, CS7
//  dev::rx_gpio::pcr::pc = 0b0010'0000;  // PCD WAIT (PCD totem-pole output)


  // select pinmux peripherals

  // P0x
  if (cfg_use_rs485)
  {
    dev::rx_gpio::pfs::p00 = 0b00'001010;  // TXD6
    dev::rx_gpio::pfs::p01 = 0b00'001010;  // RXD6
    dev::rx_gpio::pmr::p0 = 0b0000'0011;
  }

  // P1x
  #if defined (MCB_USE_RXxxx_144)
  {
    unsigned int pmr = 0;

    if (cfg_use_sci2)
    {
      dev::rx_gpio::pfs::p12 = 0b00'001010; // TXD2
      dev::rx_gpio::pfs::p13 = 0b00'001010; // RXD2
      pmr |= 0b0000'1100;
    }
    if (cfg_use_usb0)
    {
      dev::rx_gpio::pfs::p14 = 0b00'010001; // USB0_DPUPE
      pmr |= 0b0001'0000;
    }

    if (cfg_use_mcx)
      dev::rx_gpio::pfs::p15 = 0b01'000000; // IRQ5 (MCX INT1)

    if (cfg_use_sci2 || cfg_use_sci4 || cfg_use_sci5 || cfg_use_usb0)
      dev::rx_gpio::pfs::p16 = 0b01'000000; // IRQ6 (USB0_VBUS / EXP_INT)

    if (cfg_use_pcd)
      dev::rx_gpio::pfs::p17 = 0b01'000000; // IRQ7 (PCD)

    if (pmr != 0)
      dev::rx_gpio::pmr::p1 = pmr;
  }
  #elif defined (MCB_USE_RXxxx_176)
    dev::rx_gpio::pfs::p10 = 0b01'000000; // IRQ0 (MCX INT1)
    dev::rx_gpio::pfs::p12 = 0b00'001010; // TXD2
    dev::rx_gpio::pfs::p13 = 0b00'001010; // RXD2
    dev::rx_gpio::pfs::p14 = 0b00'010000; // CTX1
    dev::rx_gpio::pfs::p15 = 0b00'010000; // CRX1
    dev::rx_gpio::pfs::p16 = 0b01'000000; // IRQ6 (USB0_VBUS / EXP_INT)
    dev::rx_gpio::pfs::p17 = 0b01'000000; // IRQ7 (PCD)
    dev::rx_gpio::pmr::p1 = 0b0011'1100;
  #endif

  // P2x
  {
    unsigned int pmr = 0;
    if (cfg_use_sci0)
    {
      dev::rx_gpio::pfs::p20 = 0b00'001010; // TXD0
      dev::rx_gpio::pfs::p21 = 0b00'001010; // RXD0
      dev::rx_gpio::pfs::p22 = 0b00'001010; // SCK0
      pmr |= 0b0000'0111;
    }
    if (cfg_use_sci3)
    {
      dev::rx_gpio::pfs::p23 = 0b00'001010; // TXD3
      dev::rx_gpio::pfs::p24 = 0b00'001010; // SCK3
      dev::rx_gpio::pfs::p25 = 0b00'001010; // RXD3
      pmr |= 0b0011'1000;
    }
    if (cfg_use_sci1)
    {
      dev::rx_gpio::pfs::p26 = 0b00'001010; // TXD1
      dev::rx_gpio::pfs::p27 = 0b00'001010; // SCK1
      pmr |= 0b1100'0000;
    }

    if (pmr != 0)
      dev::rx_gpio::pmr::p2 = 0b1111'1111;
  }

  // P3x
  {
    unsigned int pmr = init_pmr_p3_value;

    if (cfg_use_sci1)
    {
      dev::rx_gpio::pfs::p30 = 0b00'001010; // RXD1
      pmr |= 0b0000'0001;
    }

//  dev::rx_gpio::pfs::p31 = 0b00'001101: // SSLB0 for RSPI mode, add BSP option

    if (cfg_use_emg_stop)
      dev::rx_gpio::pfs::p32 = 0b01'000000; // IRQ2 (EMG)

    if (cfg_use_mcx)
      dev::rx_gpio::pfs::p33 = 0b01'000000; // IRQ3 (MCX INT0)

    if (pmr != init_pmr_p3_value)
      dev::rx_gpio::pmr::p3 = pmr;
  }

  // P4x
  if (cfg_use_trigger_io)
  {
    dev::rx_gpio::pfs::p40 = 0b01'000000; // IRQ8, TRG_IN1
    dev::rx_gpio::pfs::p41 = 0b01'000000; // IRQ9, TRG_IN2
    dev::rx_gpio::pfs::p42 = 0b01'000000; // IRQ10, TRG_IN3
    dev::rx_gpio::pfs::p43 = 0b01'000000; // IRQ11, TRG_IN4
    dev::rx_gpio::pfs::p44 = 0b01'000000; // IRQ12, TRG_IN5
    dev::rx_gpio::pfs::p45 = 0b01'000000; // IRQ13, TRG_IN6
    dev::rx_gpio::pfs::p46 = 0b01'000000; // IRQ14, TRG_IN7
    dev::rx_gpio::pfs::p47 = 0b01'000000; // IRQ15, TRG_IN8
  }

  // P5x
  if (cfg_use_sci2)
  {
    dev::rx_gpio::pfs::p51 = 0b00'001010; // SCK2
    dev::rx_gpio::pmr::p5 = 0b0000'0010;
  }

  // P6x
  // note: CS outputs are selected below
  if (cfg_use_etherc1)
  {
      dev::rx_gpio::pfs::p60 = 0b00'010101; // RMII1_TXD_EN
      dev::rx_gpio::pmr::p6 = 0b0000'0001;  // note: CS outputs are selected below
  }

  // P7x
  if (cfg_use_etherc0)
  {
    dev::rx_gpio::pfs::p71 = 0b00'010001; // ET0_MDIO
    dev::rx_gpio::pfs::p72 = 0b00'010001; // ET0_MDC
    dev::rx_gpio::pfs::p74 = 0b00'010010; // RMII0_RXD1
    dev::rx_gpio::pfs::p75 = 0b00'010010; // RMII0_RXD0
    dev::rx_gpio::pfs::p76 = 0b00'010010; // REF50CK0
    dev::rx_gpio::pfs::p77 = 0b00'010010; // RMII0_RX_ER
    dev::rx_gpio::pmr::p7 = 0b1111'0110;
  }

  // P8x
  {
    unsigned int pmr = 0;

    if (cfg_use_etherc0)
    {
      dev::rx_gpio::pfs::p80 = 0b00'010010; // RMII0_TXD_EN
      dev::rx_gpio::pfs::p81 = 0b00'010010; // RMII0_TXD0
      dev::rx_gpio::pfs::p82 = 0b00'010010; // RMII0_TXD1
      dev::rx_gpio::pfs::p83 = 0b00'010010; // RMII0_CRS_DV
      pmr |= 0b0000'1111;
    }

    if (cfg_use_usba)
    {
     dev::rx_gpio::pfs::p87 = 0b00'001000; // MTIOC4C (USB HUB clock, unused on 144 pin)
     pmr |= 0b1000'0000;
    }

    if (pmr != 0)
      dev::rx_gpio::pmr::p8 = pmr;
  }

  // P9x
  if (cfg_use_etherc1)
  {
    dev::rx_gpio::pfs::p92 = 0b00'010101; // RMII1_CRS_DV
    dev::rx_gpio::pfs::p94 = 0b00'010101; // RMII1_RXD0
    dev::rx_gpio::pfs::p95 = 0b00'010101; // RMII1_RXD1
    dev::rx_gpio::pmr::p9 = 0b0011'0100;
  }

  // PAx
  // used for external bus address outputs, configured below

  // PBx
  {
    unsigned int pmr = 0;

    if (cfg_use_sci4)
    {
      dev::rx_gpio::pfs::pb0 = 0b00'001010; // RXD4
      dev::rx_gpio::pfs::pb1 = 0b00'001010; // TXD4
      dev::rx_gpio::pfs::pb3 = 0b00'001010; // SCK4
      pmr |= 0b0000'1011;
    }
    if (cfg_use_pcd)
    {
      dev::rx_gpio::pfs::pb5 = 0b00'000001; // MTIOC2A (PCD clock)
      pmr |= 0b0010'0000;
    }
    if (cfg_use_rs485)
    {
      dev::rx_gpio::pfs::pb6 = 0b00'001010; // RXD9
      dev::rx_gpio::pfs::pb7 = 0b00'001010; // TXD9
      pmr |= 0b1100'0000;
    }

    if (pmr != 0)
      dev::rx_gpio::pmr::pb = pmr;
  }

  // PCx
  {
    //  dev::rx_gpio::pfs::pc0 = 0b00'001011; // CTS5  can be used as RTS or CTS, or as general CS -> BSP option
    unsigned int pmr = 0;
    if (cfg_use_sci5)
    {
      dev::rx_gpio::pfs::pc2 = 0b00'001010; // RXD5
      dev::rx_gpio::pfs::pc3 = 0b00'001010; // TXD5
      dev::rx_gpio::pfs::pc4 = 0b00'001010; // SCK5
      pmr |= 0b0001'1100;
    }
    if (cfg_use_mcx)
    {
      dev::rx_gpio::pfs::pc6 = 0b00'000010; // MTCLKA (from LAN9250)
      pmr |= 0b0100'0000;
    }

    if (pmr != 0)
      dev::rx_gpio::pmr::pc = pmr;
  }

  // PDx
  // used for external bus data inputs/outputs, configured below

  // PEx
  // used for external bus data inputs/outputs, configured below

  // PFx
  if (cfg_use_lan9250)
    dev::rx_gpio::pfs::pf5 = 0b01'000000;	// IRQ4 (LAN9250)

  // PGx
  if (cfg_use_etherc1)
  {
    dev::rx_gpio::pfs::pg0 = 0b00'010101; // REF50CK1
    dev::rx_gpio::pfs::pg1 = 0b00'010101; // RMII1_RX_ER
    dev::rx_gpio::pfs::pg3 = 0b00'010101; // RMII1_TXD0
    dev::rx_gpio::pfs::pg4 = 0b00'010101; // RMII1_TXD1
    dev::rx_gpio::pmr::pg = 0b0001'1011;
  }

  // PJx
  if (cfg_use_mcx)
  {
    dev::rx_gpio::pfs::pj3 = 0b00'000001; // MTIOC3C (MCX clock A)
    dev::rx_gpio::pmr::pj = 0b0000'1000;
  }


  // ----------------------------------------------------------
  // external bus config

  // dev::rx_sys::enable_register_protection ();

  // enable MTU module to allow outputting clock signals before PERI_RESET
  // is deasserted.
  if (cfg_use_mcx || cfg_use_pcd || cfg_use_usba)
    dev::rx_mstpcra_bit<9> () (true);

  if (cfg_use_ext_ad_bus)
  {
    // enable external bus, if needed.

    // dev::rx_gpio::pfcss0 = 0b00'00'00'00; // CS1 = P61, CS2 = P62, CS3 = P63
    // dev::rx_gpio::pfcss1 = 0b00'00'00'00; // CS4 = P64, CS5 = P65, CS7 = P67

    // enable CS outputs
    dev::rx_gpio::pfcse = 0b11111110;

    // enable A17 output (P91)
    dev::rx_gpio::pfaoe1 = 0b00000010;

    // enable A0-A7 outputs on PA0-PA7
    // enable A16-A23 outputs on P90-P97
    // enable D8-D15 outputs on PE0-PE7
    dev::rx_gpio::pfbcr0 = 0b0001'0'01'1;

    // PC5 = WAIT
    dev::rx_gpio::pfbcr1 = 0b000000'10;
  }


  // PCD4641 clock and external bus setup
  // MTIOC2A = clock output
  // use CS2 area 0x06000000 - 0x06FFFFFF for register access
  // use CS1 area 0x07000000 - 0x07FFFFFF for command write
  #ifdef MCB_USE_PCD4641
  if (cfg_use_pcd)
  {
    // byte strobe mode, external wait enable, page read/write off,
    // normal access mode
    // the PCD will insert additional bus wait cycles as needed.
    dev::rx_bsc::cs2mod = dev::rx_bsc::cs1mod = 0b0'00000'0'0'0000'1'00'0;

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

    dev::rx_bsc::cs2wcr1 =
	(csrwait << 24) | (cswwait << 16);

    dev::rx_bsc::cs2wcr2 =
	csroff | (cswoff << 4) | (cson << 28) | (rdon << 16) | (wron << 20) | (wdon << 24) | (wdoff << 8);

    // recovery cycles for normal register access
    static_assert (rd_recovery <= 15, "");
    static_assert (wr_recovery <= 15, "");

    dev::rx_bsc::cs2rec = (wr_recovery << 8) | (rd_recovery << 0);


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

    dev::rx_bsc::cs1wcr1 =
	(cmd_csrwait << 24) | (cmd_cswwait << 16);

    dev::rx_bsc::cs1wcr2 =
	csroff | (cmd_cswoff << 4) | (cmd_cson << 28) | (cmd_rdon << 16) | (cmd_wron << 20) | (cmd_wdon << 24) | (cmd_wdoff << 8);

    // recovery cycles for command write access
    static_assert (cmd_wr_recovery <= 15, "");

    dev::rx_bsc::cs1rec = (cmd_wr_recovery << 8) | (cmd_wr_recovery << 0);

    // enable operation, 8 bit bus, explicit big endian, MPX disable.
    dev::rx_bsc::cs2cr = dev::rx_bsc::cs1cr =
	0b000'0'000'0'00'10'000'1
	| (utils::native_byte_order () != utils::big_endian ? (1 << 8) : 0);

    // RX64. RX71 use MTU3a unit
    // RX63 use MTU2a unit
    // register addresses are different, but registers generally are compatible
    #ifdef MCB_USE_RX64_RX71
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1400>> mtu2_tcr;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1402>> mtu2_tior;
      static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0xC1408>> mtu2_tgra;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1280>> mtu_tstra;
    #else
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x88800>> mtu2_tcr;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x88802>> mtu2_tior;
      static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x88808>> mtu2_tgra;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x88680>> mtu_tstra;
    #endif

    // count at rising edge, TCNT clear on TGRA match, internal clock PCLK/1
    mtu2_tcr = 0b001'00'000;

    // MTIOC2B: no output, MTIOC2A: initial output is 0, toggle output at compare match.
    mtu2_tior = 0b0000'0011;

    static_assert (mtu_tick_count_to_hz (hz_to_mtu_tick_count_floor (pcd4641_clock_hz))
		   == pcd4641_clock_hz, "");

    mtu2_tgra = hz_to_mtu_tick_count_floor (pcd4641_clock_hz);

    // start MTU2 counter
    mtu_tstra |= 0b00000100;
  }
  #endif

  if (!cfg_use_pcd && cfg_use_dipswitch)
  {
    // in order to read dispswitch bits from A5,A6,A7 we need to do an external
    // bus access with those address bits set to high.
    // the dipswitch driver will do a read in the PCD register area (CS2).
    // enable the CS with some timing, even if PCD is not used.
    dev::rx_bsc::cs2mod = 0b0'00000'0'0'0000'0'00'0;

    constexpr unsigned int csrwait = ns_to_bclk (20);
    constexpr unsigned int csroff = ns_to_bclk (0);

    constexpr unsigned int cswwait = csrwait;
    constexpr unsigned int cswoff = csroff;

    constexpr unsigned int wdoff = 0;
    constexpr unsigned int wdon = 0;
    constexpr unsigned int wron = 0;
    constexpr unsigned int cson = 0;

    static_assert (csrwait <= 31, "");
    static_assert (cswwait <= 31, "");
    dev::rx_bsc::cs7wcr1 = (cswwait << 16) | (csrwait << 24);

    static_assert (csroff <= 7, "");
    static_assert (cswoff <= 7, "");
    static_assert (wdon <= 7, "");
    static_assert (wron <= 7, "");
    static_assert (wdoff <= 7, "");
    static_assert (cson <= 7, "");
    dev::rx_bsc::cs2wcr2 = (wdon << 24) | (wdoff << 8) | (cson << 28) | (wron << 20)
			   | (csroff << 0) | (cswoff << 4);


    // enable operation, 8 bit bus, explicit big endian, MPX disable.
    dev::rx_bsc::cs2cr =
	0b000'0'000'0'00'10'000'1
	| (utils::native_byte_order () != utils::big_endian ? (1 << 8) : 0);
  }


  // MCX514 clock and external bus setup
  // MTIOC3C = clock output
  // use CS4 area 0x04000000 - 0x04FFFFFF for register access
  // use CS3 area 0x05000000 - 0x05FFFFFF for command write
  #ifdef MCB_USE_MCX51x
  if (cfg_use_mcx)
  {
    // single write strobe mode, no external wait, page read/write off,
    // normal access mode
    dev::rx_bsc::cs4mod = dev::rx_bsc::cs3mod = 0b0'00000'0'0'0000'0'00'1;

    // the MCX needs to see a CS de-assertion between the accesses.
    // for some combinations of the wait values, CS is never de-asserted, which
    // is a problem.  so better to check with a scope.
    // notice that the timing granularity is in 1000/96M = 10.42 ns
    // (internal bus clock).  values are rounded up to multiples of that.
    // so a 4ns delay will actually be 10.42 ns.

    // writes to the command area (CS3) will insert a 2 MCX clock wait cycle.
    // 20 MHz MCX clock: 100 ns wait
    // 16 MHz MCX clock: 125 ns wait
    // 12 MHz MCX clock: 166 ns wait

    // max. 15 recovery cycles @  96 MHz BCLK = 156 ns
    // max. 15 recovery cycles @ 120 MHz BCLK = 125 ns

    // 2x 20 MHz clocks = 10x 96 MHz clocks
    // 2x 16 MHz clocks = 12x 96 MHz clocks
    // 2x 12 MHz clocks = 16x 96 MHz clocks

    // use recovery cycles as much as possible.   however, in case of 12 MHz MCX
    // clock 15 recovery cycles might be not enough.  in this case, stretch the
    // write access by one cycle by adjusting the 'cswoff' time.

    // read cycle
    // continuous register read = 19 MHz (96 MHz blck), xx MHz (120 MHz bclk)
    constexpr unsigned int rdon = ns_to_bclk (4);
    constexpr unsigned int csrwait = ns_to_bclk (31);
    constexpr unsigned int csroff = ns_to_bclk (5);

    // write cycle
    // continuous register write = 19 MHz (96 MHz blck)
    constexpr unsigned int wron = ns_to_bclk (4);
    constexpr unsigned int cswwait = ns_to_bclk (30);
    constexpr unsigned int cswoff = ns_to_bclk (1);
    static_assert (csrwait <= 31, "");
    static_assert (cswwait <= 31, "");

    dev::rx_bsc::cs4wcr1 = dev::rx_bsc::cs3wcr1 =
	(csrwait << 24) | (cswwait << 16);

    static_assert (bclk_hz > mcx51x_t::clock_hz_i, "");

    constexpr unsigned int cmd_wr_write_wait_bclk =
	(((200LL * bclk_hz) / mcx51x_t::clock_hz_i) + 99) / 100;

    constexpr unsigned int cmd_wr_wait_recovery_bclk =
	std::min (cmd_wr_write_wait_bclk, 15u);

    constexpr unsigned int cmd_cswoff =
	cswoff + cmd_wr_write_wait_bclk - cmd_wr_wait_recovery_bclk;

    constexpr unsigned int cson = ns_to_bclk (4);

    static_assert (csroff <= 7, "");
    static_assert (cswoff <= 7, "");
    static_assert (cson <= 7, "");
    static_assert (cmd_cswoff <= 7, "");

    dev::rx_bsc::cs4wcr2 =
	csroff | (cswoff << 4) | (cson << 28) | (rdon << 16) | (wron << 20);

    dev::rx_bsc::cs3wcr2 =
	csroff | (cmd_cswoff << 4) | (cson << 28) | (rdon << 16) | (wron << 20);

    // no recovery cycles for normal register access
    dev::rx_bsc::cs4rec = 0;

    // recovery cycles for command write access
    static_assert (cmd_wr_wait_recovery_bclk <= 15, "");

    dev::rx_bsc::cs3rec =
	cmd_wr_wait_recovery_bclk | (cmd_wr_wait_recovery_bclk << 8);

    // enable operation, 16 bit bus, explicit little endian, MPX disable.
    dev::rx_bsc::cs4cr = dev::rx_bsc::cs3cr =
	0b000'0'000'0'00'00'000'1
	| (utils::native_byte_order () != utils::little_endian ? (1 << 8) : 0);


    // RX64. RX71 use MTU3a unit
    // RX63 use MTU2a unit
    // register addresses are different, but registers generally are compatible
    #ifdef MCB_USE_RX64_RX71
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1200>> mtu3_tcr;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1204>> mtu3_tiorh;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1205>> mtu3_tiorl;
      static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0xC1224>> mtu3_tgrc;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0xC1280>> mtu_tstra;
    #else
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x88600>> mtu3_tcr;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x88604>> mtu3_tiorh;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x88605>> mtu3_tiorl;
      static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x88624>> mtu3_tgrc;
      static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x88680>> mtu_tstra;
    #endif


    // count at rising edge, TCNT clear on TGRC match, internal clock PCLK/1
    mtu3_tcr = 0b101'00'000;

    // MTIOC3B: no output, MTIOC3A: no output
    mtu3_tiorh = 0b0000'0000;

    // MTIOC3D: no output, MTIOC3C: initial output is 0, toggle output at compare match.
    mtu3_tiorl = 0b0000'0011;

    static_assert (std::ratio_equal <
	mcx51x_t::clock_hz,
	mtu_tick_count_to_hz_ratio < hz_to_mtu_tick_count_floor < mcx51x_t::clock_hz > () > >::value, "");

    mtu3_tgrc = hz_to_mtu_tick_count_floor < mcx51x_t::clock_hz > ();

    // start MTU3 counter
    mtu_tstra |= 0b01000000;
  }
  #endif

  // LAN9250 external bus setup
  // use CS5 area
  // 0x03000000 - 0x0300001F register access
  // 0x03020000 - 0x0302FFFF fifo access
  if (cfg_use_lan9250)
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

  // always setup the CS7 area (0x0100'0000 - 0x01FF'FFFF)
  // area mapping is:
  // 0x00 - trigger outputs (lower 8 bit)
  // 0x02 - digital outputs [15..0]
  // 0x04 - digital outputs [31..16]
  // 0x06 - mcx axis outputs (XOUT1..8, YOUT1..8)
  // 0x08 - mcx axis outputs (ZOUT1..8, UOUT1..8)
  // 0x0A - digital inputs [15..0]
  // 0x0C - digital inputs [31..16]
  // 0x0E - n/a
  //
  // the latches will be reset to all 0 by the hardware reset.
  if (cfg_use_ext_ad_bus)
  {
    dev::rx_bsc::cs7mod = 0b0'00000'0'0'0000'0'00'1;

    // RX CS7 to latch read/write enable delay (address decoder, xor gates) = 15 ns
    // RX CS7 to data output at latch 22 ns.
    // the CS7 output must not go on before the address has stabilized.
    // this timing is critical as it might result in the wrong latch picking
    // up the data.

    // NAND flash RDY/BSY data rise time about 34 ns -> use 2 successive read cycles.

    //  96 mhz bus clock: 42 ns write access time (23.8 MHz)
    //                    62 ns read access time (16.1 MHz)

    // 120 mhz bus clock: 42 ns write access time (23.8 MHz)
    //                    58 ns read access time (17.2 MHz)

								// blck [MHz]
								// 120   / 96

    constexpr unsigned int cson = ns_to_bclk (15);		// 16.66 / 20.83
//    constexpr unsigned int cson = ns_to_bclk (0);		// seems OK on 05 board
    constexpr unsigned int wdon = cson;
    constexpr unsigned int wron = cson;

    constexpr unsigned int cswwait = ns_to_bclk (20);		// 24.99 / 20.83
//    constexpr unsigned int cswwait = ns_to_bclk (25);		// seems OK on 05 board
    constexpr unsigned int cswoff = 0;
    constexpr unsigned int wdoff = 0;

    // 41 ns is enough, but with 96 MHz blck probing the signal on the board
    // results in wrong reads.  it looks too flaky, better add more margin.
    constexpr unsigned int csrwait = ns_to_bclk (45);		// 49.99 / 52.08
//    constexpr unsigned int csrwait = ns_to_bclk (25);		// seems OK on 05 board
    constexpr unsigned int csroff = 0;

    static_assert (csrwait <= 31, "");
    static_assert (cswwait <= 31, "");
    dev::rx_bsc::cs7wcr1 = (cswwait << 16) | (csrwait << 24);

    static_assert (csroff <= 7, "");
    static_assert (cswoff <= 7, "");
    static_assert (wdon <= 7, "");
    static_assert (wron <= 7, "");
    static_assert (wdoff <= 7, "");
    static_assert (cson <= 7, "");
    dev::rx_bsc::cs7wcr2 = (wdon << 24) | (wdoff << 8) | (cson << 28) | (wron << 20)
			   | (csroff << 0) | (cswoff << 4);

    // writes to the output latches need at least one bus cycle of silence
    // otherwise the latches might pick up wrong data.
    dev::rx_bsc::cs7rec = 0 | (1 << 8);

    // enable operation, 16 bit bus, explicit little endian, MPX disable.
    dev::rx_bsc::cs7cr =
	0b000'0'000'0'00'00'000'1
	| (utils::native_byte_order () != utils::little_endian ? (1 << 8) : 0);
  }

  // on-chip ROM enable, external bus enable.
  // enable CS recovery cycles.
  // always enable external bus.
  if (cfg_use_ext_ad_bus)
  {
    dev::rx_bsc::csrecen = 0xFFFF;
    dev::rx_sys::syscr0 = 0x5A00 | 0b00000011;
  }


  if (cfg_use_usba)
  {
    // NB: only for RX64 / RX71, not for RX63.

    // start 12 MHz clock output on MTIOC4C for USB HUB
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1201>> mtu4_tcr;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1206>> mtu4_tiorh;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1207>> mtu4_tiorl;
    static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0xC1228>> mtu4_tgrc;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC1280>> mtu_tstra;
    static constexpr dev::hw_reg_rw<uint8_t,  dev::const_addr<0xC120A>> mtu_toera;

    // count at rising edge, TCNT clear on TGRC match, internal clock PCLK/1
    mtu4_tcr = 0b101'00'000;

    // MTIOC4B: no output, MTIOC4A: no output
    mtu4_tiorh = 0b0000'0000;

    // MTIOC4D: no output, MTIOC4C: initial output is 0, toggle output at compare match
    mtu4_tiorl = 0b0000'0011;

    // some MTU output pins are gated by the timer output master enable register.
    // MTIOC4C is such an output pin.
    mtu_toera |= (1 << 4);

    constexpr unsigned int usb_hub_clock_hz = 12'000'000;

    static_assert (mtu_tick_count_to_hz (hz_to_mtu_tick_count_floor (usb_hub_clock_hz))
		   == usb_hub_clock_hz, "");

    mtu4_tgrc = hz_to_mtu_tick_count_floor (usb_hub_clock_hz);

    // start MTU4 counter
    mtu_tstra |= 0b10000000;
  }

  // the hw setup could be executed not after a reset, e.g. when an application
  // jumps to the bootloader to run it.  in such a case, this will not work.
  #ifdef MCB_USE_RX64_RX71
    dev::rx64_interrupt::init_sw_cfg_intb_inta ();

    // enable interrupts after IO config.
    dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_be0>::enable (dev::interrupt::any_edge, dev::interrupt::priority_6);
    dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_bl0>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);
    dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_bl1>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);
    dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_al0>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);
    dev::rx64_interrupt::line<dev::rx64_interrupt::icu_group_al1>::enable (dev::interrupt::low_level, dev::interrupt::priority_6);
  #endif

  __builtin_rx_setpsw ('I');

#endif // MCB_NO_RESET_HW_INIT
}

[[gnu::cold]]
mcb_v2_board::release_peripheral_reset_t
::release_peripheral_reset_t (mcb_v2_board& brd)
{
#ifndef MCB_NO_PERIPHERAL_RESET_RELEASE

  set_peripheral_reset (false);

#endif // MCB_NO_PERIPHERAL_RESET_RELEASE
}

mcb_v2_board::set_system_timer_initialized_t
::set_system_timer_initialized_t (void)
{
  g_system_timer_initialized = true;
}

#if defined (MCB_USE_DAC124S085)
mcb_v2_board::init_dac_default_values_t
::init_dac_default_values_t (mcb_v2_board& brd)
{
  // after power-on/reset, PERI_RST will cut the power to the DAC
  // output drivers and the lines will be pulled to AGND.
  // during power-on, the DACs will reset their output to 0V (~ -10V output).

  // before PERI_RST is released, write a 0V output value to all DAC channels.
  // FIXME: these values should come from calibration data.
  // every channel is slightly different and needs a different value to output 0V.
  brd.dac0.set_dac_regs_update_outputs (4096/2, 4096/2, 4096/2, 4096/2);
  brd.dac1.set_dac_regs_update_outputs (4096/2, 4096/2, 4096/2, 4096/2);
  brd.dac2.set_dac_regs_update_outputs (4096/2, 4096/2, 4096/2, 4096/2);
}
#endif


[[gnu::cold]]
const std::array<uint8_t, 6>
mcb_v2_board::board_info_eth0_addr::operator () (void)
{
  auto&& addr = mcb_v2_board_info::inst ().ifconfigs ()[0].hw_addr ();
  return (const std::array<uint8_t, 6>&)addr;
}

[[gnu::cold]]
const std::array<uint8_t, 6>
mcb_v2_board::board_info_eth1_addr::operator () (void)
{
  auto&& addr = mcb_v2_board_info::inst ().ifconfigs ()[1].hw_addr ();
  return (const std::array<uint8_t, 6>&)addr;
}

[[gnu::cold]]
const std::array<uint8_t, 6>
mcb_v2_board::board_info_eth2_addr::operator () (void)
{
  auto&& addr = mcb_v2_board_info::inst ().ifconfigs ()[2].hw_addr ();
  return (const std::array<uint8_t, 6>&)addr;
}




mcb_v2_board::mcb_v2_board (void)
 : devices_begin ()

 , reset_hw_init (*this)
 , system_timer ()

#if defined (MCB_USE_DMACA)
  , dmaca_dispatcher
  ({
    &dmaca0, &dmaca1, &dmaca2, &dmaca3
    #if !defined (MCB_USE_RX63)
    , &dmaca4, &dmaca5, &dmaca6, &dmaca7
    #endif
  })

#endif

#if defined (MCB_USE_SPI)
 , sci4_spi (dmaca_dispatcher)
#endif

#if defined (MCB_USE_DAC124S085)
 , dac0 (sci4_spi)
 , dac1 (sci4_spi)
 , dac2 (sci4_spi)
 , init_dac_default_values (*this)
#endif

#if defined (MCB_USE_MCX514)
 , mcx51x (&mcx51x_axis_inputs, &mcx51x_axis_outputs)
 , mcx51x_axis_inputs {{
	mcx51x.axis (0), mcx51x.axis (1), mcx51x.axis (2), mcx51x.axis (3) }}
 , mcx51x_axis_outputs {{
	{ mcx51x.axis (0), m_mcx_axis_outputs_cache },
	{ mcx51x.axis (1), m_mcx_axis_outputs_cache },
	{ mcx51x.axis (2), m_mcx_axis_outputs_cache },
	{ mcx51x.axis (3), m_mcx_axis_outputs_cache } }}
#endif

#if defined (MCB_USE_MCX512)
 , mcx51x (&mcx51x_axis_inputs, &mcx51x_axis_outputs)
 , mcx51x_axis_inputs {{
	mcx51x.axis (0), mcx51x.axis (1) }}
 , mcx51x_axis_outputs {{
	{ mcx51x.axis (0), m_mcx_axis_outputs_cache },
	{ mcx51x.axis (1), m_mcx_axis_outputs_cache } }}
#endif

 , release_peripheral_reset (*this)

#if defined (MCB_USE_PCD4641)
 , pcd_axis_inputs_sampler (pcd4641.bus_if ())
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

 , devices_end ()
{

#if defined (MCB_USE_FCU)
  static_assert ((fclk_hz + 1'000'000-1) / 1'000'000 >= 4, "");
  static_assert ((fclk_hz + 1'000'000-1) / 1'000'000 <= 60, "");
#endif


#if defined (MCB_USE_PCD4641)

// PCD4641 OTS[XYZU]: axis output (amp enable)
// PCD4641 P1[XYZU]: axis output (default after reset)
// PCD4641 P2[XYZU]: axis output (default after reset)
// PCD4641 P3[XYZU]: axis output (default after reset)
// PCD4641 P4[XYZU]: axis output (default after reset)

// PCD4641 STP[XYZU]: axis input (amp alarm, fixed input)
// PCD4641 U/B[XYZU]: axis input (fixed input)
// PCD4641 F/H[XYZU]: axis input (fixed input)
// PCD4641 STA[XYZU]: axis input (fixed input)

  for (auto& a : pcd4641.axes ())
    a.set_env (dev::pcd4641::env_t ()
	.set_p1p2p3p4_mode (dev::pcd4641::p1p2p3p4_io)
	.set_p1_io_mode (dev::pcd4641::output)
	.set_p2_io_mode (dev::pcd4641::output)
	.set_p3_io_mode (dev::pcd4641::output)
	.set_p4_io_mode (dev::pcd4641::output));
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


#if defined (MCB_USE_DIPSWITCH)
  // get the initial state of the dipswitches after all the other setup is done.
  dipswitch_inputs.read ();
#endif


#if defined (MCB_USE_LEDS) && defined (MCB_USE_STANDBY_POWER_SUPPLY)
  const auto rst_src = reset_source ();
  if (rst_src == reset_source::soft_standby)
  {
    for (unsigned int led_val = 0; led_val != 0b11111;
	 led_val = (led_val << 1) | 1)
    {
      led_outputs.write (led_val & 0b1111);
      std::this_thread::sleep_for (std::chrono::milliseconds (70));
    }

    led_outputs.write (0);
  }
#endif

  maybe_reset_config_to_factory_default ();

#if defined (MCB_USE_STANDBY_POWER_SUPPLY)

  // enable voltage monitoring to get a software trigger when the VCC_MCU
  // voltage drops to about 2.826V (standby VCC_MCU).
  lvda.set_lvd1_config (dev::rx_lvda::config_t ()
	.set_trigger (dev::rx_lvda::int_on_vcc_less)
	.set_threshold_voltage (dev::rx_lvda::v2_99)
	.set_noise_filter (dev::rx_lvda::filter_off)
	.set_voltage_monitor_enabled (true)
	.set_voltage_comparator_enabled (true));

  lvda.set_lvd1_trigger_func ([this] (void) { transition_to_standby_mode_0 (); });
  lvda.set_lvd1_enable ();

  // the sub-oscillator has been enabled at the very beginning.  it takes a
  // while to come up and become stable.  check that it's up and running here
  // as a very last step.
  // when coming back from deep-standby we know that it is up and running,
  // or at least that it was up and running at least once.
  if (rst_src != reset_source::soft_standby)
  {
    while ((dev::rx_clk::oscovfsr.read () & 0b00010) == 0) { }

    // on RX64 and RX71 we can actually measure the frequency of the sub-oscillator
    // and check that it is in the expected range.  currently there is little
    // point to it, as it is not really used for time keeping.  so the frequency
    // does not matter.  moreover, the precision of the sub-oscillator depends
    // on many factors such as CL and temperature.  it's best to measure the
    // average frequency over a longer time period and save that as a calibration
    // value in the board info after manufacturing.
    // if measuring the sub-osc frequency after manufacturing/testing, it's better
    // to use the CACREF input pin (shared with PC7, available on CN1).
    #if defined (MCB_USE_RX64_RX71)
    static_assert (xtal_hz / (xcin_hz / 32) <= cac.counter_max_value (), "");

    cac.set_config (dev::rx_cac::measurement_config_t ()
	.set_target_clock (dev::rx_cac::main_clock)
	.set_target_clock_div (dev::rx_cac::target_div_1)

	.set_use_cacref_pin_input (false)
	.set_reference_clock (dev::rx_cac::sub_clock)
	.set_reference_clock_div (dev::rx_cac::ref_div_32)
	.set_reference_clock_edge_type (dev::rx_cac::rising_edge)
    );

    cac.set_target_frequency_range (
    {
      // because of the inverted equations, min and max is swapped
      xtal_hz / ((xcin_hz + 250) / 32),
      xtal_hz / ((xcin_hz - 250) / 32),
    });

    cac.start_measurement ();

    while (true)
    {
      auto res = cac.measurement_result ();
      if (res.measurement_finished ())
      {
	if (res.frequency_in_range ())
	  break;
	else
	  abort ();
      }
    }
  #endif
  }

#endif
}

mcb_v2_board::~mcb_v2_board (void)
{
  // additional shutdown code here if needed
}


extern "C" alignas (32) const uint8_t ice40_bitstream[];
extern "C" const size_t ice40_bitstream_size;

void
mcb_v2_board::set_peripheral_reset (bool val)
{
  // peripheral reset line is on P70 for all board variants.
  // output low (default at power-on): reset on (PERI_RST high, ~PERI_RST~ low)
  // output high: reset off (PERI_RST low, ~PERI_RST~ high)
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p7), 0> output;

  // the outputs latches on the data bus will fetch whatever is on the data bus
  // during a reset.  try to make sure that they see all bits as zeros by
  // writing zeros to the data bus.
  //
  // also, if the ice40 init follows after reset release, make sure that the
  // SPI_SS_B line on AD16.A4 is pulled low to select SPI-slave configuration
  // mode.

  auto& data_bus16 = *(volatile uint16_t*)0x0100000E;

  if (cfg_use_ext_ad_bus)
  {
    data_bus16 = 0;
    data_bus16 = 0;
    data_bus16 = 0;
  }

  if (val == true)
    output.clear ();
  else
    output.set ();


  // for newer boards starting from revision 05 we need to load the ice40
  // bitstream after reset release.  the ice40 will keep the digital and trigger
  // outputs disabled with its CDONE output until it has been configured.
  #if defined (MCB_USE_RX64_RX71)
  if (val == false && cfg_use_ext_ad_bus
      && this_board::board_info ().features ().test (mcb_v2_board_info::feature_bit::ice40))
  {
    // there is a problem with the power sequencing of 3.3V vs. 1.2V.
    // the ICE40 normally wants to have the 1.2V first.  but the LAN8720 wants
    // 3.3V first.  there is no particular power rail sequencing control and
    // the LAN8720 seems to be doing OK with that, but the ICE40 has an issue
    // after power on.  it needs to see a high -> low on the reset with at
    // least 500 ns duration (not 200 ns as in the datasheet).
    output.clear ();
    std::this_thread::sleep_for (std::chrono::nanoseconds (1500));
    output.set ();
    std::this_thread::sleep_for (std::chrono::nanoseconds (1500));
    output.clear ();
    std::this_thread::sleep_for (std::chrono::nanoseconds (1500));
    output.set ();

    // need to wait at least 1200 usec until ice40 becomes ready for
    // configuration download.
    std::this_thread::sleep_for (std::chrono::microseconds (1500));

    auto&& spi_ss0_clk0 = (volatile uint16_t*)0x01000000;
    auto&& spi_ss0_clk1 = (volatile uint16_t*)0x01000008;

    auto&& spi_ss1_clk0 = (volatile uint16_t*)0x01000010;
    auto&& spi_ss1_clk1 = (volatile uint16_t*)0x01000018;

    // deseclect SPI_SS (AD16.A4), send it 8 clocks on SPI_CLK to
    // begin the configuration
    for (unsigned int i = 0; i < 8; ++i)
    {
      *spi_ss1_clk1 = 0;
      *spi_ss1_clk0 = 0;
    }

    for (size_t i = 0; i < ice40_bitstream_size; ++i)
    {
      uint8_t data_byte = ice40_bitstream[i];

      // send data to SPI_SI (AD16.D3) on the falling edge of SPI_CLK, MSB first.
      for (unsigned int j = 0; j < 8; ++j)
      {
	uint16_t wr_val = (data_byte & 0b1000'0000) ? 0xFFFF : 0x0000;

	*spi_ss0_clk1 = wr_val;
	*spi_ss0_clk0 = wr_val;

	data_byte <<= 1;
      }
    }

    // deseclect SPI_SS (AD16.A4), send it another 100 clocks on SPI_CLK to
    // complete the configuration
    for (unsigned int i = 0; i < 100; ++i)
    {
      *spi_ss1_clk1 = 0;
      *spi_ss1_clk0 = 0;
    }
  }
  #endif


  #if defined (MCB_USE_MCX514) || defined (MCB_USE_MCX512)
  if (val == false && cfg_use_mcx)
  {
    // the MCX trigger outputs are pulled high after the MCX comes out of
    // reset.  the trigger outputs are enabled after a short delay, so have
    // a bit of time to reset them.

    // MCX needs 4 cycles to come out of reset.  wait for about 8 MCX clocks
    // to be on the safe side (we're not synchronizing with the MCX clock timer).
    using iclk_over_mcx_clk = std::ratio_divide < std::ratio<iclk_hz>, mcx_clock_hz >;
    constexpr unsigned int iclk_cycles = (iclk_over_mcx_clk::num + iclk_over_mcx_clk::den - 1) / iclk_over_mcx_clk::den;

    for (unsigned int i = 0; i < 8; ++i)
      cpu_wait_3_iclocks < (iclk_cycles + 2) / 3 > ();

    #if defined (MCB_USE_MCX514)
    for (auto& a : inst ().mcx51x.axes ())
    {
      a.set_pio1 (dev::mcx51x::pio_mode1_t ()
		.set_pin (0, dev::mcx51x::general_input)	// trigger input I00
		.set_pin (1, dev::mcx51x::general_input)	// trigger input I01

		// FIXME: for hardware trigger outputs need to use
		//  sync_pulse_mr_comp_output for the outputs
		.set_pin (2, dev::mcx51x::general_output)	// trigger output I02
		.set_pin (3, dev::mcx51x::general_output)	// trigger output I03

		.set_pin (4, dev::mcx51x::general_input)	// trigger input I04
		.set_pin (5, dev::mcx51x::general_input)	// trigger input I05
		.set_pin (6, dev::mcx51x::general_input)	// axis input IN3
		.set_pin (7, dev::mcx51x::general_input));	// axis input IN4

      a.set_pios (0);
    }
    #endif

    #if defined (MCB_USE_MCX512)
    for (auto& a : inst ().mcx51x.axes ())
    {
      a.set_pio1 (dev::mcx51x::pio_mode1_t ()
		.set_pin (0, dev::mcx51x::general_input)	// trigger input I00
		.set_pin (1, dev::mcx51x::general_input)	// trigger input I01

		// FIXME: for hardware trigger outputs need to use
		//  sync_pulse_mr_comp_output for the outputs
		.set_pin (2, dev::mcx51x::general_output)	// trigger output I02
		.set_pin (3, dev::mcx51x::general_output)	// trigger output I03

		.set_pin (4, dev::mcx51x::general_input)	// trigger input I04
		.set_pin (5, dev::mcx51x::general_input)	// trigger input I05
		.set_pin (6, dev::mcx51x::general_input)	// axis input IN3
		.set_pin (7, dev::mcx51x::general_input));	// axis input IN4

      a.set_pio3 (dev::mcx51x::pio_mode3_t ()
		.set_pin (8, dev::mcx51x::general_input)	// axis input IN5
		.set_pin (9, dev::mcx51x::general_input));	// axis input IN6

      a.set_pios (0);
    }
    #endif
  }
  #endif

  if (val == false && cfg_use_ext_ad_bus)
  {
    // when coming out of reset, keep the data bus low for a while until
    // the IO latches have initialized.
    data_bus16 = 0;

    auto t = std::chrono::high_resolution_clock::now ();
    while (std::chrono::high_resolution_clock::now () - t < std::chrono::milliseconds (10))
      data_bus16 = 0;
  }
}

using reset_source_1 = reset_source;

[[gnu::cold]] reset_source_1
mcb_v2_board::reset_source (void)
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

int mcb_v2_board::reset_counter (void)
{
  return *(volatile int*)32;
}

void mcb_v2_board::set_reset_counter (int val)
{
  *(volatile int*)32 = val;
}

// -----------------------------------------------------------------------------

void mcb_v2_board::exec (void)
{
//  auto cur_time = std::chrono::high_resolution_clock::now ();

  // auto delta_time = cur_time - m_last_exec_time;

//  m_last_exec_time = cur_time;
}

// -----------------------------------------------------------------------------

void mcb_v2_board::set_standby_supply_enable (bool val)
{
  // also disable bat_vtest and bat_ltest

#if defined (MCB_USE_STANDBY_POWER_SUPPLY)

  #ifdef MCB_INVERTED_BAT_LTEST_EN_BAT_VTEST_EN
    dev::rx_gpio::podr::pg = (0b0110'0000 | (val << 7));
  #else
    dev::rx_gpio::podr::pg = (0b0000'0000 | (val << 7));
  #endif

#endif
}

void mcb_v2_board::set_battery_vtest_enable (bool val)
{
#if defined (MCB_USE_STANDBY_POWER_SUPPLY)
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::pg), 6> output;

  #ifdef MCB_INVERTED_BAT_LTEST_EN_BAT_VTEST_EN
    val = !val;
  #endif

  if (val == true)
    output.set ();
  else
    output.clear ();
#endif
}

void mcb_v2_board::set_battery_ltest_enable (bool val)
{
#if defined (MCB_USE_STANDBY_POWER_SUPPLY)
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::pg), 5> output;

  #ifdef MCB_INVERTED_BAT_LTEST_EN_BAT_VTEST_EN
    val = !val;
  #endif

  if (val == true)
    output.set ();
  else
    output.clear ();
#endif
}


#if defined (MCB_USE_STANDBY_POWER_SUPPLY)


[[noreturn, gnu::noinline, gnu::used]]
void mcb_v2_board::transition_to_standby_mode_0 (void)
{
  // when here, a voltage drop on VCC_MCU has been detected.
  // everything that is powered by +3.3V on the board is shutting down
  // and the VCC_MCU is switched from +3.3V to the standby DCDC converter
  // (battery) power supply.  during the transition time, the VCC_MCU is held
  // up by some bulk capacitance, but this capacitance is limited.  the MCU
  // has to quickly turn off all external IOs as it could short out VCC_MCU
  // to +3.3V, which is now going down to become GND.
  set_standby_supply_enable (true);

  // disable the external bus
  dev::rx_sys::syscr0 = 0x5A00 | 0b00000001;

  // pull peripheral reset
  // as +3.3V is going down, the peripheral reset line will eventually
  // be at 0V.  try to pull it earlier to hopefully extend the +3.3V for
  // a little bit longer.
  set_peripheral_reset (true);

  // disable output ports except for LEDs
  // everything else is not usable in standby mode as it's powered off.

#if defined (MCB_USE_RX63) || defined (MCB_USE_RXxxx_144)

  #error standby mode not supported on RX63 or 144-pin MCU

#elif defined (MCB_USE_RXxxx_176)

  // disable all external interrupts
  // because the +3.3V rail is falling, the interrupt lines will also
  // fall and will trigger interrupts and potentially lock-up the cpu.
  dev::rx_gpio::pfs::p10 = 0b00'000000; // IRQ0 (MCX INT1)
  dev::rx_gpio::pfs::p16 = 0b00'000000; // IRQ6 (USB0_VBUS / EXP_INT)
  dev::rx_gpio::pfs::p17 = 0b00'000000; // IRQ7 (PCD)
  dev::rx_gpio::pfs::p33 = 0b00'000000; // IRQ3 (MCX INT0)
  dev::rx_gpio::pfs::p40 = 0b00'000000; // IRQ8, TRG_IN1
  dev::rx_gpio::pfs::p41 = 0b00'000000; // IRQ9, TRG_IN2
  dev::rx_gpio::pfs::p42 = 0b00'000000; // IRQ10, TRG_IN3
  dev::rx_gpio::pfs::p43 = 0b00'000000; // IRQ11, TRG_IN4
  dev::rx_gpio::pfs::p44 = 0b00'000000; // IRQ12, TRG_IN5
  dev::rx_gpio::pfs::p45 = 0b00'000000; // IRQ13, TRG_IN6
  dev::rx_gpio::pfs::p46 = 0b00'000000; // IRQ14, TRG_IN7
  dev::rx_gpio::pfs::p47 = 0b00'000000; // IRQ15, TRG_IN8
  dev::rx_gpio::pfs::pf5 = 0b00'000000;	// IRQ4 (LAN9250)

  // leave LD1,LD2,LD3,LD4 and LD_COL_SEL.
  dev::rx_gpio::pdr::p0 = 0b1010'1100;  // P02 = LD1, P03 = LD2, P05 = LD3, P07 = LD4
  dev::rx_gpio::pdr::p1 = 0b0000'0010;  // P11 = LD_COL_SEL

  dev::rx_gpio::pdr::p7 = 0b0000'0001;  // P73 = CAN_RS, P70 = /PERI_RST
  dev::rx_gpio::pdr::p8 = 0b0000'0000;  // P86 = SCI2_EXT_CS
  dev::rx_gpio::pdr::p9 = 0b0000'0000;  // P93 = DAC_CS1N, P96 = DAC_CS2N, P97 = DAC_CS3N, P90 = SCI4_EXT_CS
  dev::rx_gpio::pdr::pb = 0b0000'0000;  // PB4 = RS485_CH1_DE
  dev::rx_gpio::pdr::pc = 0b0000'0000;  // PC0 = SCI5_EXT_CS

  // select pinmux peripherals (disable most of them)
  dev::rx_gpio::pmr::p0 = 0b0000'0000;
  dev::rx_gpio::pmr::p1 = 0b0000'0000;

  // keep TXD1 and RXD1 alive for now to allow printing something
  // on the debug console.  kill all other peripheral IOs.
  dev::rx_gpio::pmr::p2 = 0b0100'0000;
  dev::rx_gpio::pmr::p5 = 0b0000'0000;
  dev::rx_gpio::pmr::p6 = 0b0000'0000;
  dev::rx_gpio::pmr::p7 = 0b0000'0000;
  dev::rx_gpio::pmr::p8 = 0b0000'0000;
  dev::rx_gpio::pmr::p9 = 0b0000'0000;
  dev::rx_gpio::pmr::pb = 0b0000'0000;
  dev::rx_gpio::pmr::pc = 0b0000'0000;
  dev::rx_gpio::pmr::pg = 0b0000'0000;
  dev::rx_gpio::pmr::pj = 0b0000'0000;

  dev::rx_clk::bckcr = 1;

#else
  #error mcu type

#endif // MCU type

  // switch to the application defined standby-enter function

  // we are in an interrupt context here and we will never return to the
  // interrupted application.  switch back to the user-stack, re-enable
  // interrupts and continue with transition_to_standby_mode_1.

  asm volatile ("\n"
  "	.global _ustack_end"		"\n\t"

  "	mvtc	#_ustack_end, usp"	"\n\t"
  "	mvtc	#0x100, fpsw"		"\n\t"

  // new psw (user stack, interrupts enable, interrupt level mask 0)
  "	mov.l	#((1 << 16) | (1 << 17)), r2"	"\n\t"
  "	push.l	r2"			"\n\t"

  // rte return address
  "	mov.l	#__ZN12mcb_v2_board28transition_to_standby_mode_1Ev, r2" "\n\t"
  "	push.l	r2"	"\n\t"

  "	rte"

  : : : "memory", "cc", "r0", "r2");

  __builtin_unreachable ();
}

[[noreturn, gnu::noinline, gnu::used]]
void mcb_v2_board::transition_to_standby_mode_1 (void)
{
  // at this point it consumes about 50 mA.
  // if the batteries are almost discharged, it will just die after a while.
  if (&enter_standby_mode)
    enter_standby_mode ();

  // the user defined standby function has returned.
  // enter deep standby mode, which will be exited when normal power is applied.

  for (unsigned int led_val = 0b1111; led_val != 0; led_val >>= 1)
  {
    led_outputs.write (led_val);
    std::this_thread::sleep_for (std::chrono::milliseconds (70));
  }

  led_outputs.write (0);


  // configure LVDA to trigger a reset if VCC_MCU rises back,
  // which means normal power is applied again.
  lvda.set_lvd1_config (dev::rx_lvda::config_t ()
	.set_trigger (dev::rx_lvda::int_on_vcc_greater_equal)
	.set_threshold_voltage (dev::rx_lvda::v2_95)
	.set_noise_filter (dev::rx_lvda::filter_off)
	.set_voltage_monitor_enabled (true)
	.set_voltage_comparator_enabled (true)
	.set_reset_release (dev::rx_lvda::after_vcc_greater));

  lvda.set_lvd1_enable ();

  // shutdown the board
  // (the lvda and rtc always stay alive)
  dev::this_cpu::save_disable_interrupts ();

#if defined (MCB_USE_SCI_DEBUG)
  auto& sci = debug_usart;

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
#endif

  this->~mcb_v2_board ();


  // LD_COL_SEL should be turned off normally to save power.
  // to turn on the green LED, turn it on.
  // it is impossible to turn on the red LED in standby mode.
  // turning off the outputs saves about 5 uA in deep-standby
  dev::rx_gpio::pdr::p0 = 0b0000'0000;  // P02 = LD1, P03 = LD2, P05 = LD3, P07 = LD4
  dev::rx_gpio::pdr::p1 = 0b0000'0000;  // P11 = LD_COL_SEL
  dev::rx_gpio::podr::p1 = 0x00;
  dev::rx_gpio::podr::p0 = 0xFF;


  // use the sub-oscillator as the main system clock (35 mA -> 8.9 mA)
  dev::rx_clk::sckcr3 = 0b011 << 8;

  // stop main, loco, hoco oscillators and finally kill sci1
  dev::rx_clk::mosccr = 1;
  dev::rx_clk::lococr = 1;
  dev::rx_clk::hococr = 1;
//  dev::rx_clk::hocopcr = 1;  // must not do that.

  dev::rx_gpio::pmr::p2 = 0b0000'0000;
  dev::rx_gpio::pfs::p26 = 0b00'000000; // TXD1
  dev::rx_gpio::pfs::p30 = 0b00'000000; // RXD1
//  dev::rx_gpio::pdr::p3 = 0b0100'0000;  // pull EXTAL to ground, must not do that.
  dev::rx_gpio::pmr::p3 = 0b0000'0000;  // P36 = EXTAL, P37 = XTAL
  dev::rx_gpio::pdr::pg = 0b1000'0000;  // release BAT_VTEST_EN and BAT_LTEST_EN outputs

  // switch to low power mode 2
  OPCCR = 0b111;
  while (OPCCR & 0b10000) { }

  // select rising edge for LVDA1 interrupt in deep standby
  DPSIEGR2 = 1;

  // allow LVDA1, RTC alarm, RTC cycle interrupts in deep standby
  DPSIER2 = 0b1101;

  // To clear DPSIFR0 to 00h after modifying DPSIER0, wait for at least six PCLKB cycles,
  // read DPSIFR0, and then write 0 to DPSIFR0. Six or more PCLKB cycles can be secured,
  // for example, by reading DPSIER0.
  DPSIFR2 &= 0;

  enter_deep_standby_mode ();

  for (unsigned int i = 0; ; ++i)
  {
    // flash LED for about 500 usec (16 cycles @ 32768 hz)
    // 500 usec = 0.0005 sec
    // 1/256 sec = 0.00390625 sec = 3906.25 usec
    dev::rx_gpio::podr::p0 = 0b0000'0100 ^ 0xFF;
    dev::rx_gpio::podr::p1 = 0xFF;

    for (volatile unsigned int j = 0; j < 16; ++j);  // 500 usec
//    for (volatile unsigned int j = 0; j < 16*8; ++j);  // 4000 usec (~ 1/256 sec)

    dev::rx_gpio::podr::p0 = 0b0000'0000 ^ 0xFF;
    dev::rx_gpio::podr::p1 = 0x00;

    for (volatile unsigned int j = 0; j < 1024*1024*32/(1024*6); ++j)
    {
      // turn the standby supply off for a moment and switch over to +3.3V.
      for (volatile unsigned int jj = 0; jj < 2; ++jj)
        set_standby_supply_enable (false);

      if (!lvda.lvd1_status ().vcc_less_than ())
	reset ();

      set_standby_supply_enable (true);
    }
  }

}

#endif


// -----------------------------------------------------------------------------

[[noreturn]]
void mcb_v2_board::enter_deep_standby_mode (void)
{
  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x80010>> MSTPCRA = { };
  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x80014>> MSTPCRB = { };
  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x80018>> MSTPCRC = { };
  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x8001C>> MSTPCRD = { };
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr <0x0008000C>> SBYCR = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr <0x0008C280>> DPSBYCR = { };

  // standby RAM is enabled/disabled in deep-standby mode only by
  // the DPSBYCR.DEEPCUT setting.  the module-stop setting in MSTPCRC has no
  // effect.

  SBYCR = 1 << 15;
  DPSBYCR = 0b1'0'0000'00; // retain standby RAM contents
  MSTPCRA = 0xFFFFFFFF;
  MSTPCRB = 0xFFFFFFFF;
  MSTPCRC = 0xFFFFFFFF;
  MSTPCRD = 0xFFFFFFFF;

  // make sure that all buffered writes have been completed by reading some
  // register.
  MSTPCRD.read ();

  __builtin_rx_wait ();
  __builtin_unreachable ();

  // when deep standby mode is cancelled, the cpu starts executing the reset
  // vector.  it does resume execution after the wait instruction like normal
  // standby mode.

  // cancellation of deep standby mode via interrupt takes some time.
  // tDSBY (recovery time after cancellation of deep software standby mode) = 0.9 ms
  // tDSBYWT (wait time after cancellation of deep software standby mode) = 32 LOCO cycles
}

// -----------------------------------------------------------------------------

namespace std { namespace this_thread {

void __sleep_for (std::chrono::high_resolution_clock::duration d)
{
  if (likely (g_system_timer_initialized))
  {
     auto start_time = std::chrono::high_resolution_clock::now ();
     while (std::chrono::high_resolution_clock::now () - start_time < d) { }
  }
  else
  {
    const auto cur_clock_src = dev::rx_clk::sckcr3 >> 8;
    int64_t wait_clk_count = 0;

    if (cur_clock_src == 0b000) // LOCO
    {
      typedef std::chrono::duration<int64_t, std::ratio<1, mcb_v2_board::loco_hz>> cur_clk_duration_t;
      wait_clk_count = std::chrono::duration_cast<cur_clk_duration_t> (d).count ();
    }
    else if (cur_clock_src == 0b011) // sub-oscillator
    {
      typedef std::chrono::duration<int64_t, std::ratio<1, mcb_v2_board::xcin_hz>> cur_clk_duration_t;
      wait_clk_count = std::chrono::duration_cast<cur_clk_duration_t> (d).count ();
    }
    else
      assert_unreachable ();

    // waiting for more iclocks at once amortizes the loop overhead, which
    // actually makes the waiting times longer.
    for (; wait_clk_count > 0; wait_clk_count -= 3*10)
      mcb_v2_board::cpu_wait_3_iclocks<10> ();
  }
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
  return mcb_v2_board::inst ().system_timer.current_time_ticks ();
}

} } }

// -----------------------------------------------------------------------------

static volatile bool g_in_assert_abort = false;

void board_debug_uart_write (const void* data, unsigned int byte_count)
{
#if defined (MCB_USE_SCI_DEBUG)
  auto& sci = mcb_v2_board::inst ().debug_usart;

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
  write (STDERR_FILENO, str, max_count);
}

static void
emergency_puts (const char* str)
{
  write (STDERR_FILENO, str, std::strlen (str));
}

[[gnu::weak]] void finalize_assert_abort_handler (void)
{
}


#if defined (NDEBUG) && defined (RX_USER_BOOT_IMAGE)
[[noreturn, gnu::noinline, gnu::cold]]
void
int_assert (const char* source_filename, int linenum,
            const char* func_name, const char* expr)
{
  while (true) { }
}

#else
[[noreturn, gnu::noinline, gnu::cold]]
void
int_assert (const char* source_filename, int linenum,
            const char* func_name, const char* expr)
{
  // when here all interrupts to the CPU are blocked.
  // the LEDs are driven by a timer and a DTC, which will continue its
  // operation.
  g_in_assert_abort = true;

  auto& board = mcb_v2_board::inst ();

  // disable all outputs and everything, or else motors and stuff will
  // continue running and might damage something.
  board.set_peripheral_reset (true);

#if defined (MCB_USE_LEDS)
  board.led_outputs.write (0b00000000);
#endif

#if defined (MCB_USE_SCI_DEBUG)
  auto& sci = mcb_v2_board::inst ().debug_usart;

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

  constexpr unsigned int led_count_mask =
	utils::ceil_pow2 (mcb_v2_board_clk::iclk_hz / 750);

  for (unsigned int i = 0; ; ++i)
  {
    // for some strange reason, the wait builtin emits two wait insns inside
    // the loop.  outside the loop it seems to be fine though.
    // anyway, in this case we'd like to wait forever, so it doesn't matter.
    // __builtin_rx_wait ();
#if defined (MCB_USE_LEDS)
     board.led_outputs.write ((i & led_count_mask) ? 0b1111'0000 : 0b0000'0000);
#endif
  };
}

#endif // NDEBUG

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

#ifdef MCB_USE_RX64_RX71
static void icu_group_be0_irq (void);
static void icu_group_bl0_irq (void);
static void icu_group_bl1_irq (void);
static void icu_group_al0_irq (void);
static void icu_group_al1_irq (void);
#endif

#if defined (MCB_USE_DMACA) && !defined (MCB_USE_RX63)
static void dmac74_irq (void);
#endif

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

#if MCB_USE_RX63
  utils::make_array<isr_desc, dev::rx63_interrupt::count, isr_desc::init_empty> (),
#elif MCB_USE_RX64_RX71
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
  make_isr_desc<mcb_v2_board::debug_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v2_board::debug_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v2_board::debug_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v2_board::debug_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SCI0_USART)
  make_isr_desc<mcb_v2_board::gpsi0_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi0_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi0_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi0_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SCI2_USART)
  make_isr_desc<mcb_v2_board::gpsi3_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi3_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi3_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi3_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SCI3_USART)
  make_isr_desc<mcb_v2_board::gpsi1_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi1_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi1_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi1_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_SCI5_USART)
  make_isr_desc<mcb_v2_board::gpsi2_usart_t::txi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi2_usart_t::rxi_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi2_usart_t::tei_isr_t> (),
  make_isr_desc<mcb_v2_board::gpsi2_usart_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_RS485)
  make_isr_desc<mcb_v2_board::rs485_ch1_t::txi_isr_t> (),
  make_isr_desc<mcb_v2_board::rs485_ch1_t::rxi_isr_t> (),
  make_isr_desc<mcb_v2_board::rs485_ch1_t::tei_isr_t> (),
  make_isr_desc<mcb_v2_board::rs485_ch1_t::eri_isr_t> (),

    #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144) \
        || defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)
    make_isr_desc<mcb_v2_board::rs485_ch2_t::txi_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::rxi_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::tei_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::eri_isr_t> (),

    #elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    make_isr_desc<mcb_v2_board::rs485_ch2_t::bri_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::eri_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::rxi_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::txi_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::tei_isr_t> (),
    make_isr_desc<mcb_v2_board::rs485_ch2_t::dri_isr_t> (),

    #endif
  #endif

  #if defined (MCB_USE_SPI)
  make_isr_desc<mcb_v2_board::sci4_spi_t::txi_isr_t> (),
  make_isr_desc<mcb_v2_board::sci4_spi_t::rxi_isr_t> (),
  make_isr_desc<mcb_v2_board::sci4_spi_t::tei_isr_t> (),
  make_isr_desc<mcb_v2_board::sci4_spi_t::eri_isr_t> (),
  #endif

  #if defined (MCB_USE_PCD4641)
  make_isr_desc<mcb_v2_board::pcd4641_t::isr0_t> (),
  make_isr_desc<mcb_v2_board::cmt3_t::cmi_isr_t> (),
  #endif

  #if defined (MCB_USE_MCX514) || defined (MCB_USE_MCX512)
  make_isr_desc<mcb_v2_board::mcx51x_t::isr0_t> (),
  make_isr_desc<mcb_v2_board::mcx51x_t::isr1_t> (),
  #endif

  #if defined (MCB_USE_FCU)
    // fcu also has some interrupts which can be useful for background
    // operations.
  #endif 

  #if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC0_RAW)
  make_isr_desc<mcb_v2_board::rx_edmac0_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_RX_ETHERC1) || defined (MCB_USE_RX_ETHERC1_RAW)
  make_isr_desc<mcb_v2_board::rx_edmac1_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW)
  make_isr_desc<mcb_v2_board::lan9250_t::isr0_t> (),
  #endif

  #if defined (MCB_USE_TRIGGER_IO)
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr0_t> (),
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr1_t> (),
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr2_t> (),
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr3_t> (),
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr4_t> (),
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr5_t> (),
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr6_t> (),
  make_isr_desc<mcb_v2_board::trigger_inputs_t::isr7_t> (),
  #endif

  #if defined (MCB_USE_EMG_STOP)
  make_isr_desc<mcb_v2_board::emg_stop_inputs_t::isr_t> (),
  #endif

  // used by LED DTC only
  // make_isr_desc<mcb_v2_board::cmt2_t::cmi_isr_t> (),

  #if defined (MCB_USE_TPU)
  make_isr_desc<mcb_v2_board::tpu0_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu0_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu0_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu0_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu0_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu0_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::tpu1_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu1_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu1_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu1_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu1_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu1_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::tpu2_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu2_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu2_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu2_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu2_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu2_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::tpu3_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu3_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu3_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu3_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu3_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu3_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::tpu4_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu4_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu4_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu4_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu4_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu4_t::tci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::tpu5_t::tgi_a_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu5_t::tgi_b_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu5_t::tgi_c_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu5_t::tgi_d_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu5_t::tci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::tpu5_t::tci_u_isr_t> (),
  #endif

  #if defined (MCB_USE_GPT)
  make_isr_desc<mcb_v2_board::gpt0_t::gtci_a_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt0_t::gtci_b_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt0_t::gtci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt0_t::gtci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::gpt1_t::gtci_a_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt1_t::gtci_b_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt1_t::gtci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt1_t::gtci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::gpt2_t::gtci_a_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt2_t::gtci_b_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt2_t::gtci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt2_t::gtci_u_isr_t> (),

  make_isr_desc<mcb_v2_board::gpt3_t::gtci_a_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt3_t::gtci_b_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt3_t::gtci_v_isr_t> (),
  make_isr_desc<mcb_v2_board::gpt3_t::gtci_u_isr_t> (),

  #endif

  make_isr_desc<mcb_v2_board::system_timer_t::isr0_t> (),
  make_isr_desc<mcb_v2_board::system_timer_t::isr1_t> ()

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

    #if !defined (NDEBUG) && !defined (RX_USER_BOOT_IMAGE)
    ,
    make_isr_desc<dev::interrupt::connected_isr<dev::rx63_interrupt::line<dev::rx63_interrupt::reserved_0>,
		  dev::interrupt::func<decltype (&INT_Excep_BRK), &INT_Excep_BRK>>> (),

    make_isr_desc<dev::interrupt::connected_isr<dev::rx63_interrupt::line<dev::rx63_interrupt::reserved_1>,
		  dev::interrupt::func<decltype (&INT_Assert), &INT_Assert>>> ()
    #endif

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
	|| defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    ,
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

    #if defined (MCB_USE_STANDBY_POWER_SUPPLY)
    ,
    make_isr_desc<mcb_v2_board::cac_t::ferrie_isr_t> (),
    make_isr_desc<mcb_v2_board::cac_t::mendie_isr_t> (),
    make_isr_desc<mcb_v2_board::cac_t::ovfie_isr_t> (),

    make_isr_desc<mcb_v2_board::lvda_t::lvd1_isr_t> (),
    make_isr_desc<mcb_v2_board::lvda_t::lvd2_isr_t> ()

    #endif

    #if defined (MCB_USE_FCU) && defined (MCB_USE_RX64_RX71)
    ,
    make_isr_desc<mcb_v2_board::fcu_t::fiferr_isr_t> (),
    make_isr_desc<mcb_v2_board::fcu_t::frdyi_isr_t> ()

    #endif
  #endif // RX64_RX71

  #if defined (MCB_USE_DMACA)
    ,
    make_isr_desc<mcb_v2_board::dmaca0_t::isr_t> (),
    make_isr_desc<mcb_v2_board::dmaca1_t::isr_t> (),
    make_isr_desc<mcb_v2_board::dmaca2_t::isr_t> (),
    make_isr_desc<mcb_v2_board::dmaca3_t::isr_t> ()

    #if defined (MCB_USE_RX64_RX71)
      ,
      make_isr_desc<dev::interrupt::connected_isr<dev::rx64_interrupt::line<dev::rx64_interrupt::dmac74i>,
		    dev::interrupt::func<decltype (&dmac74_irq), &dmac74_irq>>> ()
    #endif
  #endif

))))))));


// check each interrupt status bit for the group and invoke the
// respective ISR function from the table.
#ifdef MCB_USE_RX64_RX71
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
#endif

#if defined (MCB_USE_DMACA) && !defined (MCB_USE_RX63)
[[gnu::flatten]] void dmac74_irq (void)
{
  const unsigned int r = dev::rx_dmaca::dmac74_dmist.read ();

  for (unsigned int i = 0; i < 4; ++i)
    if (r & (1 << (i + 4)))
      (rvectors[dev::rx64_interrupt::dmac4i + i]) ();
}
#endif

// -----------------------------------------------------------------------------

#if defined (MCB_USE_DTC)

// watch out -- alignment constraints for DTC vector table are different
// on RX63 and RX64/RX71.
#if MCB_USE_RX63
  alignas (1024*4)
#else
  alignas (1024*1)
#endif
mcb_v2_board::dtc_t::vector_table_t g_dtc_vector_table;


mcb_v2_board::dtc_t::vector_table_t&
mcb_v2_board::dtc_vector_table_inst (void) { return g_dtc_vector_table; }
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

[[gnu::cold]] const mcb_v2_board_info& mcb_v2_board_info::inst (void)
{
  // this is RX63N specific.  the fixed vector table in the boot loader
  // contains a pointer to the actual board_info data which usually resides
  // in user boot flash image.
  // the default renesas usb boot loader contains 0xFFFFFFFF at that place.
  // in any case, make sure that the address in the range of the user boot
  // flash area.
  uintptr_t bi = *(const uintptr_t*)0xFF7FFF84;

  if (bi >= 0xFF7FC000 && bi <= 0xFF7FFFFF)
    return *(const mcb_v2_board_info*)bi;

  return *(const mcb_v2_board_info*)&board_info_data;
}

// =============================================================================


void mcb_v2_board::reset_to_func (void (*func)(void))
{
  // depending on which devices have been used, we might need a proper
  // shutdown of the drivers to stop the interrupts etc.
  // otherwise the new program might malfunction because of some unexpected
  // hardware event.
  set_peripheral_reset (true);

  dev::this_cpu::save_disable_interrupts ();

  this->~mcb_v2_board ();

  func ();
}

void mcb_v2_board::reset (void)
{
  // SWRR software reset register
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x000800C2>> SWRR = { };
  SWRR = 0xA501;
}

[[gnu::cold]] void mcb_v2_board::maybe_reset_config_to_factory_default (void)
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
  auto flash_leds = [this] (int count, bool red = false, int millis = 500)
  {
    for (int i = 0; i < count; ++i)
    {
      led_outputs.write (0b1111 << red*4);
      std::this_thread::sleep_for (std::chrono::milliseconds (millis));
      led_outputs.write (0b000'00000);
      std::this_thread::sleep_for (std::chrono::milliseconds (millis));
    }
  };

  auto rsrc = reset_source ();

  if (rsrc == reset_source::poweron)
  {
    set_reset_counter (1);
    flash_leds (1);
  }
  else if (rsrc == reset_source::hard_reset)
  {
    auto rcnt = reset_counter ();

    if (rcnt == 3)
    {
      set_reset_counter (4);
      flash_leds (5, true, 100);

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

typedef uintptr_t fp;
extern "C" void PowerON_Reset (void);

// because it's casing constants to pointers newer c++ will reject it if it's
// 'constexpr'.  if 'constexpr' is omitted, there is technically no guarantee
// that the vector table will be evaluated at compile time, but in practice
// it's usually the case.
// static constexpr auto fvectors [[gnu::section (".fvectors"), gnu::used, gnu::aligned (4)]] =
static const auto fvectors [[gnu::section (".fvectors"), gnu::used, gnu::aligned (4)]] =
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

#ifdef MCB_USE_RX63
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
// 0xFFFFFFEC | 0xFF7FFFEC  | Reserved / UB Code A
// 0xFFFFFFF0 | 0xFF7FFFF0  | Reserved / UB Code B
// 0xFFFFFFF4 | 0xFF7FFFF4  | Reserved / UB Code B

// the user boot code (UB Codes in the hardware manual) are checked by the
// MCU's hidden start-up ROM to determine whether it should start executing
// the user boot code or not.
//
// on RX63 the user boot rom is factory pre-programmed with the Renesas USB
// boot loader.  it can be replaced with a custom boot loader and the
// hardware manual says the UB code should be set to "UserBoot".
// the burned-in Renesas SCI bootloader also checks the UB code of the user
// boot area.  if it says "UserBoot", it will always erase the whole block
// after connecting to the MCU with a programmer software and writing to the
// normal flash areas.  however, if it says "UsbBoot" (terminated with 0xFF),
// it will not be erased.
// we don't want our bootloader to be erased automatically, as it can also
// contain factory settings and board information.
//
// on RX64 and RX71 the Renesas USB boot loader is stored in a separate hidden
// ROM in the MCU, like the SCI boot loader.  the SCI boot loader does not
// automatically erase anything on the MCU.  the startup ROM checks the
// UB codes in the user boot area and if it says "UserBoot", it will execute
// the user boot area.  if it doesn't, it will execute the burned-in USB
// bootloader.

#ifndef RX_USER_BOOT_IMAGE
  (fp)0x00000000, (fp)0x00000000, (fp)0x00000000, (fp)0x00000000,

#else
  #if MCB_USE_RX64_RX71
  // "UserBoot" (little endian)
  (fp)0x55736572, (fp)0x426F6F74, (fp)0xFFFFFF07, (fp)0x0008C04C,

  #elif MCB_USE_RX63
  // "UsbBoot\xff" (little endian)
  (fp)0x55736242, (fp)0x6F6F74FF, (fp)0xFFFFFF07, (fp)0x0008C04C,

  #else
    #error unknown MCU type
  #endif
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

