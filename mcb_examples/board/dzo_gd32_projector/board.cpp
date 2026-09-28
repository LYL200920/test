
#include <cstdio>
#include <chrono>
#include <thread>

#include <board/board.hpp>
#include <board/board_info.hpp>

#include <utils/rodata.hpp>
#include <utils/text.hpp>

#include <dev/cpu.hpp>

#include <dev/gd32f30/rcu.hpp>
namespace this_rcu = dev::gd32f30::rcu;

// make sure that the board constructor is ran as the very first static
// initializer by specifying the init_priority attribute.
[[gnu::init_priority (101), gnu::used]] dzo_gd32_proj_board dzo_gd32_proj_board::g_inst;

[[gnu::cold]]
dzo_gd32_proj_board::reset_hw_init_t::reset_hw_init_t (dzo_gd32_proj_board& brd)
{
#ifndef BOARD_NO_RESET_HW_INIT

  // at power-on the CPU runs off the internal 8 MHz oscillator (IRC8) or HXTAL
  // but we might get here also due to soft-reset ...

  #ifdef BOARD_RCU_USE_IRC8M
    // set IRC8MEN (hsi_hz)
    this_rcu::ctl = (this_rcu::ctl & 0b111111'00'1111'0000'11111111'11111'1'00)
		| (1 << 0) // IRC8MEN = 1
		| 0;

    //this_rcu::cfg0 &= 0;
    this_rcu::cfg0 = (this_rcu::cfg0 & 0b0110'1000'00111110'00000000'00001100)
		| (0b00   <<  0)  // SCS[1:0]       = 0b 00    ( CK_SYS  = CK_IRC8M      )
		| (0b0000 <<  4)  // AHBPSC[3:0]    = 0b 0xxx  ( CK_AHB  = CK_SYS        )
		| (0b100  <<  8)  // APB1PSC[2:0]   = 0b 100   ( CK_APB1 = CK_AHB  / 2   )
		| (0b000  << 11)  // APB2PSC[2:0]   = 0b 000   ( CK_APB2 = CK_AHB        )
		| (0b01   << 14)  // ADCPSC[1:0]    = 0b 01    ( CK_ADC  = CK_APB2 / 4   )
		| (0b0    << 16)  // PLLSEL         = 0b 0     ( PLLSEL  = IRC8M   / 2   )
		| (0b10   << 22)  // USBDPSC[1:0]   = 0b 10    ( CK_USBD = CK_PLL  / 2.5 )
		| (0b000  << 24)  // CKOUT0SEL[2:0] = 0b 0xx   ( Disable Clock Out       )
		| 0;

    // HXTALEN = off
    // CKMEN = off
    // PLLEN = off
    this_rcu::ctl &= 0b111111'10'1111'0110'11111111'11111'1'11;

    // HXTALBPS = off
    //   note: can be written only if HXTALEN is 0.
    this_rcu::ctl &= 0xFFF'B'FFFF;

    // PLLPRESEL: HXTAL
    this_rcu::cfg1 &= 0x9FFFFFFF;

    // IRC48MEN = off
    // CK48MSEL = CK_PLL/USBDPSC
    this_rcu::addctl &= 0xFFFEFFFE;

    // disable all interrupts
    this_rcu::intr    &= 0b11111111'00000000'1'000000000000000;
    this_rcu::addintr &= 0b111111111'0'1111111'0'1111111'0'111111;

    // HCLK = SYSCLK: keep CFGR HPRE at 0 (= div 1)
    static_assert (sysclk_hz == hclk_hz, "");

    // PCLK2 = HCLK: keep CFGR PPRE at 0 (= div 1)
    static_assert (pclk2_hz  == hclk_hz, "");

    // PCLK1 = HCLK/2: keep CFGR PPRE at 0b100 (= div 2)
    static_assert (pclk1_hz  == hclk_hz/2, "");

    // ADC clock = pclk2_hz/4
    // ADC max clock = 40 MHz
    static_assert (pclk2_hz/4 <= 40'000'000, "");

    // wait until IRC8M is stable
    while ((this_rcu::ctl & (1 << 1)) == 0) { }

  #else // HXTAL 8M

    // HXTALBPS = off
    //   note: can be written only if HXTALEN is 0.
    this_rcu::ctl &= ~(1 << 18);

    // set HXTALEN (8M external crystal oscillator)
    this_rcu::ctl = (this_rcu::ctl & 0b111111'00'1111'0000'11111111'11111'1'00)
		| (1 << 16) // HXTALEN = 1
		| 0;

    //this_rcu::cfg0 &= 0;
    this_rcu::cfg0 = (this_rcu::cfg0 & 0b0110'1000'00111110'00000000'00001100)
		| (0b00   <<  0)  // SCS[1:0]       = 0b 01    ( CK_SYS  = CK_HXTAL      )
		| (0b0000 <<  4)  // AHBPSC[3:0]    = 0b 0xxx  ( CK_AHB  = CK_SYS        )
		| (0b100  <<  8)  // APB1PSC[2:0]   = 0b 100   ( CK_APB1 = CK_AHB  / 2   )
		| (0b000  << 11)  // APB2PSC[2:0]   = 0b 000   ( CK_APB2 = CK_AHB        )
		| (0b01   << 14)  // ADCPSC[1:0]    = 0b 01    ( CK_ADC  = CK_APB2 / 4   )
		| (0b1    << 16)  // PLLSEL         = 0b 1     ( PLLSEL  = HXTAL         )
		| (0b1    << 17)  // PREDV0         = 0b 1     ( PREDV0  = HXTAL   / 2   )
		| (0b10   << 22)  // USBDPSC[1:0]   = 0b 10    ( CK_USBD = CK_PLL  / 2.5 )
		| (0b000  << 24)  // CKOUT0SEL[2:0] = 0b 0xx   ( Disable Clock Out       )
		| 0;

    // CKMEN    = off
    this_rcu::ctl &= 0b111111'10'1111'0011'11111111'11111'1'11;

    // PLLPRESEL: HXTAL
    this_rcu::cfg1 &= 0x9FFFFFFF;

    // IRC48MEN = off
    // CK48MSEL = CK_PLL/USBDPSC
    this_rcu::addctl &= 0xFFFEFFFE;

    // disable all interrupts
    this_rcu::intr    &= 0b11111111'00000000'1'000000000000000;
    this_rcu::addintr &= 0b111111111'0'1111111'0'1111111'0'111111;

    // HCLK = SYSCLK: keep CFGR HPRE at 0 (= div 1)
    static_assert (sysclk_hz == hclk_hz, "");

    // PCLK2 = HCLK: keep CFGR PPRE at 0 (= div 1)
    static_assert (pclk2_hz  == hclk_hz, "");

    // PCLK1 = HCLK/2: keep CFGR PPRE at 0b100 (= div 2)
    static_assert (pclk1_hz  == hclk_hz/2, "");

    // ADC clock = pclk2_hz/4
    // ADC max clock = 40 MHz
    static_assert (pclk2_hz/4 <= 40'000'000, "");

    // wait until HXTAL is stable
    while ((this_rcu::ctl & (1 << 17)) == 0) { }

  #endif

  // -------------------------------------------------------------------------------------------------
  // we can now be sure to be running off the 4M clock before PLLMF

  // run the GD32 at 120 MHz
  constexpr int pll_mul = sysclk_hz / (hxtal/2);
  static_assert (pll_mul >  0, "");
  static_assert (pll_mul <= 63, "");
  static_assert (pll_mul != 15, "don't have 'x 15' for PLL clock multiplication factor");

  constexpr int pll_mul_reg = pll_mul <= 16 ? (pll_mul - 2) : (pll_mul - 1);

  // PLLMF[5:0] = [ PLLMF[5] : PLLMF[4] : PLLMF[3:0]  ]
  //            = [ CFG0[30] : CFG0[27] : CFG0[21:18] ]
  this_rcu::cfg0 = (this_rcu::cfg0 & 0b1'0'11'0'11111'0000'11'11111111'11111111)
		| (((pll_mul_reg & 0b100000) >> 5) << 30)
		| (((pll_mul_reg & 0b10000)  >> 4) << 27)
		| (( pll_mul_reg & 0b1111)        << 18)
		| 0;

  // enable PLL and wait for it to come up
  this_rcu::ctl |= (1 << 24);
  while ((this_rcu::ctl & (1 << 25)) == 0) { }

  // select PLL as system clock source and wait until the clock switch has finished
  this_rcu::cfg0 |= 0b10;
  while ((this_rcu::cfg0 & 0b1100) != 0b1000) { }

  // -------------------------------------------------------------------------------------------------
  // we can now be sure to be running off the CK_PLL 120 MHz clock.

  // enable AHB clocks for ...
  //this_rcu::ahben = 0;

  // enable APB1 clocks for ...
  this_rcu::apb1en |= 0
	| (1 << 21) // I2C0
	| (1 << 19) // UART3
	| (1 << 17) // USART1
	| 0;

  // enable APB2 clocks for ...
  this_rcu::apb2en |= 0
	| (1 << 14) // USART0
	| (1 << 5)  // PDEN
	| (1 << 4)  // PCEN
	| (1 << 3)  // PBEN
	| (1 << 2)  // PAEN
	// | 0;
	| 1;        // AFEN

  // -------------------------------------------------------------------------------------------------
  // GPIO

  using namespace this_board_gpio;

  this_board_gpio::pa::odr = 0
    | set_odr (11, 1)  // PA11: led_red default off
    | 0;

  this_board_gpio::pa::ctl0 = 0
    | set_ctlx (0,  io_input)  // PA0: reserved
    | set_ctlx (1,  io_output_push_pull | io_output_10M) // PA1: USART1 DE
    | set_ctlx (2,  io_afio_push_pull | io_output_10M)   // PA2: USART1 TX
    | set_ctlx (3,  io_input)                            // PA3: USART1 RX
    | set_ctlx (4,  io_input)
    | set_ctlx (5,  io_analog) // PA5: G LED NTC
    | set_ctlx (6,  io_analog) // PA6: B LED NTC
    | set_ctlx (7,  io_analog) // PA7: R LED NTC
    | 0;

  this_board_gpio::pa::ctl1 = 0
    | set_ctlx (8,  io_input)  // PA8: trigger_in_mcu
    | set_ctlx (9,  io_afio_push_pull | io_output_10M) // PA9:  USART0 TX
    | set_ctlx (10, io_input)                          // PA10: USART0 RX
    | set_ctlx (11, io_output_push_pull | io_output_10M) // PA11: led_red
    | set_ctlx (12, io_input)
    | set_ctlx (13, io_input)
    | set_ctlx (14, io_input)
    | set_ctlx (15, io_input)
    | 0;


  this_board_gpio::pb::odr = 0
    | set_odr (6, true)   // PB6: I2C0 SCL (default high)
    | set_odr (7, true)   // PB7: I2C0 SDA (default high)
    | 0;

  this_board_gpio::pb::ctl0 = 0
    | set_ctlx (0,  io_input) // PB0: mcu_sw_on_off
    | set_ctlx (1,  io_output_push_pull | io_output_10M) // PB1: mcu_out EXT_TRG_OUT_3
    | set_ctlx (2,  io_input) // PB2: mcu_p32
    | set_ctlx (3,  io_input) // PB3: P3P3V_PWR_EN (unused?)
    | set_ctlx (4,  io_input) // PB4: MCU_DPP_RST (unused?)
    | set_ctlx (5,  io_output_push_pull | io_output_10M) // PB5: trigger_out_mcu
    | set_ctlx (6,  io_afio_open_drain | io_output_10M) // PB6: I2C0 SCL
    | set_ctlx (7,  io_afio_open_drain | io_output_10M) // PB7: I2C0 SDA
    | 0;

  this_board_gpio::pb::ctl1 = 0
    | set_ctlx (8,  io_input)
    | set_ctlx (9,  io_input)
    | set_ctlx (10, io_input)
    | set_ctlx (11, io_input)
    | set_ctlx (12, io_input)
    | set_ctlx (13, io_input)
    | set_ctlx (14, io_input)
    | set_ctlx (15, io_input)
    | 0;


  this_board_gpio::pc::odr = 0
    | set_odr (0, 1) // PC0: pull-up enable
    | set_odr (1, 1) // PC1: pull-up enable
    | set_odr (12, 1) // PC12: led white default off
    | 0;

  this_board_gpio::pc::ctl0 = 0
    | set_ctlx (0,  io_input_pupd) // PC0: m_host_irq
    | set_ctlx (1,  io_input_pupd) // PC1: s_host_irq
    | set_ctlx (2,  io_output_push_pull | io_output_10M) // PC2: proj_power_on
    | set_ctlx (3,  io_output_push_pull | io_output_10M) // PC3: mcu_ack
    | set_ctlx (4,  io_input) // PC4: mcu_req
    | set_ctlx (5,  io_output_push_pull | io_output_10M) // PB5: trigger_out_mcu
    | set_ctlx (6,  io_input)
    | set_ctlx (7,  io_input)
    | 0;

  this_board_gpio::pc::ctl1 = 0
    | set_ctlx (8,  io_input)
    | set_ctlx (9,  io_input)
    | set_ctlx (10, io_afio_push_pull | io_output_10M) // PC10: UART3 TX
    | set_ctlx (11, io_input)                          // PC11: UART3 RX
    | set_ctlx (12, io_output_push_pull | io_output_10M) // PC12: led white
    | set_ctlx (13, io_input)
    | set_ctlx (14, io_input)
    | set_ctlx (15, io_input)
    | 0;


  this_board_gpio::pd::ctl0 = 0
    | set_ctlx (0, io_input)
    | set_ctlx (1, io_input)
    | set_ctlx (2, io_input) // PD2: pattern_rdy_mcu
    | set_ctlx (3, io_input)
    | set_ctlx (4, io_input)
    | set_ctlx (5, io_input)
    | set_ctlx (6, io_input)
    | set_ctlx (7, io_input)
    | 0;


  this_board_gpio::afr::extiss0 = 0
    | set_extissx (0, exti_pc)  // pc0 -> exti0
    | set_extissx (1, exti_pc)  // pc1 -> exti1
    | set_extissx (2, exti_pd)  // pd2 -> exti2
    | set_extissx (3, exti_pd)  // pd3 -> exti3 (unused)
    | 0;

  this_board_gpio::afr::extiss1 = 0
    | set_extissx (4, exti_pc)  // pc4 -> exti4
    | set_extissx (5, exti_pa)  // pa5 -> exti5 (unused)
    | set_extissx (6, exti_pa)  // pa6 -> exti6 (unused)
    | set_extissx (7, exti_pa)  // pa7 -> exti7 (unused)
    | 0;

  this_board_gpio::afr::extiss2 = 0
    | set_extissx (8, exti_pa)   // pa8 -> exti8
    | set_extissx (9, exti_pd)   // pd9 -> exti9 (unused)
    | set_extissx (10, exti_pd)  // pd10 -> exti10 (unused)
    | set_extissx (11, exti_pd)  // pd11 -> exti11 (unused)
    | 0;

  this_board_gpio::afr::extiss3 = 0
    | set_extissx (12, exti_pa)  // pa12 -> exti12 (unused)
    | set_extissx (13, exti_pa)  // pa13 -> exti13 (unused)
    | set_extissx (14, exti_pa)  // pa14 -> exti14 (unused)
    | set_extissx (15, exti_pa)  // pa15 -> exti15 (unused)
    | 0;

#endif // ifndef BOARD_NO_RESET_HW_INIT
}

// when the GD32 is running in normal mode, from user flash memory,
// the flash memory that starts at 0x08000000 is also mapped to 0x00000000.
// for programming the flash memory and using it as data flash, remap it
// into the 0x08000000 area.

// reserve the last 3*2-KB pages of the flash memory for the data flash
// and never erase it during flashing by using stm32flash option "-e n"
// and also by using flash write protection.
// this will preserve the data flash area across firmware updates.

static constexpr uintptr_t data_flash_rom_area_start (void)
{
  // assuming flash size of 256 KB for the STM32F0 that is used on this board.
  return (uintptr_t)(1024*(256-6)) | 0x08000000;
}

static constexpr uintptr_t data_flash_rom_area_end (void)
{
  return data_flash_rom_area_start () + (1024*6);
}

dzo_gd32_proj_board::dzo_gd32_proj_board (void)
 : devices_begin ()

 , reset_hw_init (*this)

#ifndef BOARD_MCU_COMMS_ONLY

 , dlpc (i2c_master)

 , data_flash (data_flash_rom_area_start (), data_flash_rom_area_end ())

#endif  // ifndef BOARD_MCU_COMMS_ONLY

 , devices_end ()
{

#ifndef BOARD_NO_RESET_HW_INIT

  #if defined (BOARD_USE_DEBUG_USART)

    // initial debug usart configuration
    debug_usart.set_config (debug_usart.config ()
				//.set_baud_rate (1'500'000)
				.set_baud_rate (500'000));

    // initial cmd uart configuration
    cmd_uart.set_config (cmd_uart.config ()
				//.set_baud_rate (1'500'000)
				.set_baud_rate (500'000));

  #endif

  // initial rs485 usart configuration
  //   - initial baudrate is 250'000 (for MCB1 backwards compatibility)
  //     FIXME: i just copied this baudrate from nld_firmware,
  //            maybe we can set faster value.
  rs485_usart.set_config (rs485_usart.config ()
				.set_baud_rate (250'000));
#endif

}

// -----------------------------------------------------------------------------
// board using common funcs.

#ifndef BOARD_MCU_COMMS_ONLY

  dzo_gd32_proj_board::status_t dzo_gd32_proj_board::status (void)
  {
    if (!dlpc.is_power_on ())
      return dlp_power_off;

    return normal;
  }

  namespace std
  {

  std::string_view to_string (dzo_gd32_proj_board::status_t val) noexcept
  {
    switch (val)
    {
    case dzo_gd32_proj_board::normal: return "normal";
    case dzo_gd32_proj_board::dlp_power_off: return "dlp_power_off";
    case dzo_gd32_proj_board::other_error: return "other error";
    default: return { };
    }
  }

  }

  void dzo_gd32_proj_board::set_led_on (void)
  {
    dlpc.set_on ();
  }

  void dzo_gd32_proj_board::set_led_off (void)
  {
    dlpc.set_off ();
  }

  void dzo_gd32_proj_board::set_led_ready (void)
  {
    dlpc.set_ready ();
  }

#endif // ifndef BOARD_MCU_COMMS_ONLY

// -----------------------------------------------------------------------------

int dzo_gd32_proj_board::reset_counter (void)
{
  return 0;
}

void dzo_gd32_proj_board::set_reset_counter (int val)
{
}

dzo_gd32_proj_board::board_id_t dzo_gd32_proj_board::board_id (void) const
{
  constexpr auto&& board_id_str = "DZO";

  constexpr unsigned int board_id_str_len = std::strlen (board_id_str);

  board_id_t r;

  static_assert ((board_id_str_len + 96/8) <= r.size ());

  auto* i = r.begin ();

  std::memcpy (i, board_id_str, board_id_str_len);
  i += board_id_str_len;

  // on GD32 it has to be accessed as 32-bit words..
  std::memcpy (i, (const void*)0x1FFFF7E8, 96/8);
  i += 96/8;

  std::memset (i, 0, r.end () - i);

  return r;
}

// -----------------------------------------------------------------------------

void dzo_gd32_proj_board::exec (std::chrono::high_resolution_clock::time_point cur_time)
{
}

// -----------------------------------------------------------------------------

namespace std { namespace this_thread {

void __sleep_for (std::chrono::high_resolution_clock::duration d)
{
  auto start_time = std::chrono::high_resolution_clock::now ();
  while (std::chrono::high_resolution_clock::now () - start_time < d) { }
}

} }

// -----------------------------------------------------------------------------

namespace std { namespace chrono { namespace _V2 {

uint64_t system_clock::current_time_ticks (void)
{
  return dzo_gd32_proj_board::inst ().system_timer.current_time_ticks ();
}

} } }

// -----------------------------------------------------------------------------

void board_debug_uart_write (const void* data, unsigned int byte_count)
{
#if defined (BOARD_USE_DEBUG_USART)
  this_board::inst ().debug_usart.write (data, byte_count);
#endif
}


/*#if defined (BOARD_USE_DEBUG_USART)

[[gnu::cold]] static void
emergency_puts (const char* str, unsigned int max_count)
{
}

#endif*/


[[noreturn, gnu::noinline, gnu::cold]]
void
int_assert (const char* source_filename, int linenum,
            const char* func_name, const char* expr)
{
  // disable interrupts.
  this_board_cpu::save_disable_interrupts ();

#if defined (BOARD_USE_DEBUG_USART)

/*
  auto& usart = dzo_gd32_proj_board::inst ().debug_usart;

  // soft-reset the usart
  // wat for any current character transmission to finish.
  while (!usart.transmit_data_empty ()) { }

  usart.regs ().cr1 = 0;
  usart.regs ().icr = 0xFFFFFFFF;
  usart.regs ().cr2 = 0;
  usart.regs ().cr3 = 0;
  usart.regs ().cr1 = (1 << 3) | (1 << 0);  // transmitter enable, USART enable

  // flush the remaining buffer bytes
  auto tx_buffer = usart.tx_buffer_stat ();

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
*/
#endif

#ifndef BOARD_MCU_COMMS_ONLY

  // when here all interrupts to the CPU are blocked.
  // the LEDs are driven by a timer, which will continue its operation.
  auto& board = dzo_gd32_proj_board::inst ();

  //board.led_outputs.write (0b00000000);

  constexpr unsigned int led_count_mask =
	utils::ceil_pow2 (dzo_gd32_proj_board_clk::hclk_hz / 750);

  for (unsigned int i = 0; ; ++i)
  {
    board.led_white.write (i & led_count_mask ? true : false);
    board.led_red.write (i & led_count_mask ? true : false);
  };

#else
  while (true) { }

#endif // ifndef BOARD_MCU_COMMS_ONLY
}


#if defined (BOARD_USE_DEBUG_USART)

extern "C" [[noreturn, gnu::cold, gnu::used]] void abort (void)
{
  int_assert (nullptr, 0, nullptr, "Abort");
}

[[noreturn]] void INT_NMI (void) { int_assert (nullptr, 0, nullptr, "NMI"); }
[[noreturn]] void INT_HardFault (void) { int_assert (nullptr, 0, nullptr, "Hard Fault"); }

#else  // undef BOARD_USE_DEBUG_USART

extern "C" [[noreturn, gnu::cold]] void int_assert_isr (void)
{
  int_assert (nullptr, 0, nullptr, "");
}

extern "C" [[gnu::used, gnu::alias ("int_assert_isr")]] void abort (void);

[[noreturn, gnu::alias ("int_assert_isr")]] void INT_NMI (void);
[[noreturn, gnu::alias ("int_assert_isr")]] void INT_HardFault (void);

#endif // if defined (BOARD_USE_DEBUG_USART)


extern "C" [[noreturn, gnu::noinline, gnu::cold]] void
__assert_func (const char* source_filename, int linenum,
               const char* func_name, const char* expr)
{
  int_assert (source_filename + 4, linenum,
  #ifndef BOARD_ASSERT_MSG_NO_FUNCNAME
    func_name
  #else
    ""
  #endif
  , expr);
}

extern "C" void __aeabi_atexit (void)
{
  // FIXME: do something
}

[[noreturn]]
void dzo_gd32_proj_board::reset_to_func (void (*func)(void))
{
  this_board_cpu::save_disable_interrupts ();

  // depending on which devices have been used, we might need a proper
  // shutdown of the drivers to stop the interrupts etc.
  // otherwise the new program might malfunction because of some unexpected
  // hardware event.
  this->~dzo_gd32_proj_board ();

  func ();
  while (true) { }
}

[[noreturn]]
void dzo_gd32_proj_board::reset (void)
{
  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0xE000ED0C>> AIRCR = { };

  asm volatile ("dsb" : : : "memory");

  AIRCR = 0x05FA0000 | (1 << 2);

  asm volatile ("dsb" : : : "memory");
  while (true) { }
}

[[gnu::cold]] const dzo_gd32_proj_board_info& dzo_gd32_proj_board_info::inst (void)
{
  // for details, see board_info_data.cpp
  const dzo_gd32_proj_board_info* bi = *(const dzo_gd32_proj_board_info**)0x00000010;
  return *bi;
}

extern "C" unsigned int
__atomic_exchange_4 (volatile void* val, unsigned int new_val, int)
{
  auto i = this_board_cpu::save_disable_interrupts ();

  auto prev_val = *(volatile unsigned int*)val;
  *(volatile unsigned int*)val = new_val;

  this_board_cpu::restore_interrupts (i);
  return prev_val;
}

// =============================================================================

struct isr_desc
{
  int i;
  void (*f)(void);

  constexpr isr_desc (void) : i (0), f () { }

  constexpr isr_desc (int ii, void(*ff)(void)) : i (ii), f (ff) { }

  constexpr bool operator < (const isr_desc& rhs) const { return i < rhs.i; }

  // unconnected interrupts will be dropped in the end, but will fail the
  // uniqueness check.  hence ignore unconnected isr numbers.
  constexpr bool operator == (const isr_desc& rhs) const
  {
    return i != dev::interrupt::unconnected::isr_num
	   && rhs.i != dev::interrupt::unconnected::isr_num
	   && i == rhs.i;
  }

  static constexpr isr_desc init_empty (size_t i)
  {
    return isr_desc (i - 16, dev::interrupt::empty_isr_func);
  }

  static constexpr bool index_equals (const isr_desc& a, const isr_desc& b)
  {
    return a.i == b.i;
  }

  static constexpr bool not_unconnected_num (const isr_desc& a)
  {
    // filter out virtual interrupts, on this board with number >= 256
    // do not want to have ISR pointers of virtual interrupts in the ISR table.
    return a.i < 256 && a.i != dev::interrupt::unconnected::isr_num;
  }
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

extern "C" void PowerON_Reset (void);
extern "C" void _istack_end (void);
extern "C" void board_info_data (void);

static constexpr auto fvectors [[gnu::section (".fvectors"), gnu::used, gnu::aligned (4)]] =

utils::convert_array<isr_desc, isr_func> (
utils::array_verify_unique (
utils::sort_array (
utils::replace_array_elements<isr_desc, isr_desc::index_equals> (

// 2 more entries for the first 2 reset SP + reset PC
utils::make_array<isr_desc, this_board_interrupts::count + 2, isr_desc::init_empty> (),

utils::array_verify_unique (
utils::sort_array (
utils::partition_array<isr_desc, isr_desc::not_unconnected_num> (
utils::make_array (

  isr_desc (-16, _istack_end),  // initial MSP value after reset
  isr_desc (-15, PowerON_Reset),
#ifndef BOARD_MCU_COMMS_ONLY
  isr_desc (-9, board_info_data),
#endif // ifndef BOARD_MCU_COMMS_ONLY

  make_isr_desc<dzo_gd32_proj_board::system_timer_t::isr_t> (),

#ifndef BOARD_MCU_COMMS_ONLY
  make_isr_desc<dzo_gd32_proj_board::m_host_irq_t::isr_t> (),
  make_isr_desc<dzo_gd32_proj_board::s_host_irq_t::isr_t> (),
  make_isr_desc<dzo_gd32_proj_board::mcu_req_t::isr_t> (),
  make_isr_desc<dzo_gd32_proj_board::trigger_in_mcu_t::isr_t> (),
  make_isr_desc<dzo_gd32_proj_board::pattern_rdy_mcu_t::isr_t> (),
#endif // ifndef BOARD_MCU_COMMS_ONLY

#if defined (BOARD_USE_DEBUG_USART)
  make_isr_desc<dzo_gd32_proj_board::debug_usart_t::isr_t> (),
  make_isr_desc<dzo_gd32_proj_board::cmd_uart_t::isr_t> (),
#endif

  make_isr_desc<dzo_gd32_proj_board::rs485_usart_t::isr_t> ()

))))))));


