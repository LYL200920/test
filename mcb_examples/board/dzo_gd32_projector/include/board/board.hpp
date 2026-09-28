#ifndef includeguard_dzo_gd32_proj_board_hpp_includeguard
#define includeguard_dzo_gd32_proj_board_hpp_includeguard
#ifdef __cplusplus

#include <board/board_clk.hpp>
#include <board/reset_source.hpp>

#include <dev/cpu.hpp>
#include <dev/arm/cortex_m4_interrupt.hpp>
#include <dev/arm/armv6m_systick.hpp>

#include <dev/gd32f30/gd32f30_gpio.hpp>
#include <dev/gd32f30/gd32f30_usart.hpp>
#include <dev/gd32f30/gd32f30_tim.hpp>

#include <dev/gd32f1/gd32f1_i2c.hpp>

#include <dev/stm32f0/stm32f0_flash.hpp>
#include <dev/stm32f0/stm32f0_digital_output.hpp>
#include <dev/stm32f0/stm32f0_digital_input.hpp>

using     this_board_cpu        = dev::this_cpu;
namespace this_board_interrupts = dev::cortex_m4_interrupt;
namespace this_board_gpio       = dev::gd32f30_gpio;
namespace this_board_usart      = dev::gd32f30_usart;
namespace this_board_timer      = dev::gd32f30_tim;
namespace this_board_i2c        = dev::gd32f1_i2c;
namespace this_board_flash      = dev::stm32f0_flash;

#include <dev/dlpc3479.hpp>

class dzo_gd32_proj_board : public dzo_gd32_proj_board_clk
{
public:
  enum status_t
  {
    normal,
    dlp_power_off,
    other_error
  };

private:
  dzo_gd32_proj_board (void);

  static dzo_gd32_proj_board g_inst;

  struct reset_hw_init_t { reset_hw_init_t (dzo_gd32_proj_board& brd); };


  struct devices_begin_t { };
  struct devices_end_t { };

  static constexpr unsigned int interrupt_priority_bits = 4;
  static constexpr unsigned int max_interrupt_priority = (1u << interrupt_priority_bits) - 1;

public:
  static constexpr dzo_gd32_proj_board& inst (void) { return g_inst; }

  // soft-reset and transfer CPU excution control to the specified address.
  void reset_to_func (void (*func)(void));

  // soft-reset to bootmode.
  // this simply jumps to the user boot rom and might not always work.
  [[noreturn]]
  void reset_to_bootmode (void);

  // hard reset.
  // this triggers a hardware reset.
  [[noreturn]]
  void reset (void);

  // run periodic tasks of the board
  void exec (std::chrono::high_resolution_clock::time_point cur_time = std::chrono::high_resolution_clock::now ());

  // --------------------------------------------------------------------------------
  // board using common funcs.

#ifndef BOARD_MCU_COMMS_ONLY

  status_t status (void);

  void set_led_on (void);
  void set_led_off (void);
  void set_led_ready (void);

#endif // ifndef BOARD_MCU_COMMS_ONLY

  // --------------------------------------------------------------------------------
  // does some board-specific things like checking for a magic user input
  // sequence and reset the configuration partition to clear all configuration
  // data.
  void maybe_reset_config_to_factory_default (void);

  static enum reset_source reset_source (void);
  static int reset_counter (void);
  static void set_reset_counter (int val);

  using board_id_t = std::array<uint8_t, 128/8>;
  board_id_t board_id (void) const;

  devices_begin_t devices_begin;

  reset_hw_init_t reset_hw_init;

  using system_timer_t = dev::armv6m_systick<
	dzo_gd32_proj_board_clk::system_clock_frequency_hz,
	this_board_interrupts::line<this_board_interrupts::systick>,
	max_interrupt_priority>;

  system_timer_t system_timer;

#if defined (BOARD_USE_DEBUG_USART)
  using debug_usart_t = this_board_usart::hw_inst<
	0x40013800, dzo_gd32_proj_board_clk::pclk2_hz,
	this_board_interrupts::line<this_board_interrupts::usart0>,
	32, 32, 3'000'000
      #ifdef BOARD_NO_RESET_HW_INIT
	, dev::default_dummy_transceiver
	, true
      #endif
	>;

  debug_usart_t debug_usart;

  using cmd_uart_t = this_board_usart::hw_inst<
	0x40004C00, dzo_gd32_proj_board_clk::pclk1_hz,
	this_board_interrupts::line<this_board_interrupts::uart3>,
	32, 32, 3'000'000
      #ifdef BOARD_NO_RESET_HW_INIT
	, dev::default_dummy_transceiver
	, true
      #endif
	>;

  cmd_uart_t cmd_uart;
#endif

  // this is used for rs485 DE pin control.
  // DZO connect PA1 to DE pin.
  using rs485_tx_en_t = dev::gpio_transceiver < dev::stm32f0_digital_output< this_board_gpio::pa, 1> >;

  using rs485_usart_t = this_board_usart::hw_inst<
	0x40004400, dzo_gd32_proj_board_clk::pclk1_hz,
	this_board_interrupts::line<this_board_interrupts::usart1>,
	32, 32, 3'000'000
	, rs485_tx_en_t
      #ifdef BOARD_NO_RESET_HW_INIT
	, true
      #endif
	>;

  rs485_usart_t rs485_usart;

#ifndef BOARD_MCU_COMMS_ONLY
  using i2c_master_t = this_board_i2c::i2c_master <
	this_board_i2c::reg_base_i2c0,
	dzo_gd32_proj_board_clk::pclk1_hz/10*8, // 60M / 10 * 8 = 48 MHz
	100'000 // i2c clock = 100K HZ (DLPC3479 MAX)
	>;

  i2c_master_t i2c_master;

  using dlpc3479_t = dev::dlpc3479 < 0x36 >;

  dlpc3479_t dlpc;

  // the page offset address and size of the flash is initialized in the
  // constructor
  using data_flash_t = this_board_flash::hw_inst<1024*2>;
  data_flash_t data_flash;

  using m_host_irq_t = dev::stm32f0_digital_input <
	this_board_gpio::pc, 0, 0,
	this_board_interrupts::line< this_board_interrupts::exti0 >,
	dev::digital_io_port::negative_logic >;

  m_host_irq_t m_host_irq;

  using s_host_irq_t = dev::stm32f0_digital_input <
	this_board_gpio::pc, 1, 1,
	this_board_interrupts::line< this_board_interrupts::exti1 >,
	dev::digital_io_port::negative_logic >;

  s_host_irq_t s_host_irq;

  using proj_power_on_t = dev::stm32f0_digital_output <
	this_board_gpio::pc, 2 >;

  proj_power_on_t proj_power_on;

  using mcu_ack_t = dev::stm32f0_digital_output <
	this_board_gpio::pc, 3 >;

  mcu_ack_t mcu_ack;

  using mcu_req_t = dev::stm32f0_digital_input <
	this_board_gpio::pc, 4, 4,
	this_board_interrupts::line< this_board_interrupts::exti4 >>;

  mcu_req_t mcu_req;

  using mcu_sw_on_off_t = dev::stm32f0_digital_input <
	this_board_gpio::pb, 0, 0,
	dev::interrupt::unconnected >;

  mcu_sw_on_off_t mcu_sw_on_off;

  //  mcu_out EXT_TRG_OUT_3
  using mcu_out_t = dev::stm32f0_digital_input <
	this_board_gpio::pb, 1, 1,
	dev::interrupt::unconnected >;

  mcu_out_t mcu_out;

  using mcu_p32_t = dev::stm32f0_digital_input <
	this_board_gpio::pb, 2, 2,
	dev::interrupt::unconnected >;

  mcu_p32_t mcu_p32;

  using trigger_in_mcu_t = dev::stm32f0_digital_input <
	this_board_gpio::pa, 8, 8,
	this_board_interrupts::line<this_board_interrupts::exti9_5 >>;

  trigger_in_mcu_t trigger_in_mcu;

  // TRIGGER_MCU_O, connect to 3DR for DLP Trigger In Mode.
  using mcu_trigger_out_t = dev::stm32f0_digital_output<
	this_board_gpio::pb, 5 >;

  mcu_trigger_out_t trigger_out;

  using pattern_rdy_mcu_t = dev::stm32f0_digital_input <
	this_board_gpio::pd, 2, 2,
	this_board_interrupts::line< this_board_interrupts::exti2 >>;

  pattern_rdy_mcu_t pattern_rdy_mcu;


#endif // ifndef BOARD_MCU_COMMS_ONLY

  using led_red_t = dev::stm32f0_digital_output <
	this_board_gpio::pa, 11,
	dev::digital_io_port::negative_logic >;

  led_red_t led_red;

  using led_white_t = dev::stm32f0_digital_output <
	this_board_gpio::pc, 12,
	dev::digital_io_port::negative_logic >;

  led_white_t led_white;

  devices_end_t devices_end;
};

namespace this_board
{
using type = dzo_gd32_proj_board;
inline constexpr dzo_gd32_proj_board& inst (void) { return dzo_gd32_proj_board::inst (); }

}

namespace std
{
std::string_view to_string (dzo_gd32_proj_board::status_t val) noexcept;

} // namespace std

#endif // __cplusplus
#endif // includeguard_dzo_gd32_proj_board_hpp_includeguard
