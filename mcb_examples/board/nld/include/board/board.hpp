#ifndef includeguard_nld_board_hpp_includeguard
#define includeguard_nld_board_hpp_includeguard
#ifdef __cplusplus

#include <board/board_clk.hpp>
#include <board/reset_source.hpp>

#if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)
  #include <dev/arm/cortex_m0_interrupt.hpp>
  namespace this_board_interrupts = dev::cortex_m0_interrupt;
#elif defined (BOARD_VARIANT_A04_GD32)
  #include <dev/arm/cortex_m3_interrupt.hpp>
  namespace this_board_interrupts = dev::cortex_m3_interrupt;
#endif

#include <dev/arm/armv6m_systick.hpp>
#include <dev/stm32f0/stm32f0_tim.hpp>
#include <dev/stm32f0/stm32f0_dac.hpp>
#include <dev/stm32f0/stm32f0_comp.hpp>
#include <dev/stm32f0/stm32f0_usart.hpp>

#if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)
  #include <dev/stm32f0/stm32f0_adc.hpp>
  #include <dev/stm32f0/stm32f0_i2c.hpp>
#elif defined (BOARD_VARIANT_A04_GD32)
  #include <dev/gd32f1/gd32f1_adc.hpp>
  #include <dev/gd32f1/gd32f1_i2c.hpp>
#endif

#include <dev/stm32f0/stm32f0_flash.hpp>
#include <dev/stm32f0/stm32f0_crc.hpp>

#include <board/dev/led_output.hpp>
#include <board/dev/zxld_adj.hpp>
#include <board/dev/zxld_pwm.hpp>
#include <dev/stm32f0/stm32f0_gpio.hpp>
#include <dev/stm32f0/stm32f0_digital_output.hpp>
#include <dev/stm32f0/stm32f0_digital_input.hpp>
#include <dev/virtual_interrupt_demux.hpp>

#include <utils/averaging_buffer.hpp>
#include <utils/simple_iir_filter.hpp>
#include <utils/value_range.hpp>
#include <utils/digital_denoise.hpp>

#include <string_view>

class nld_board : public nld_board_clk
{
public:
  enum status_t
  {
    normal,
    standby,
    ready,
    vaux_undervoltage,
    vin_undervoltage,
    stall_out_of_regulation,
    overtemperature,
    overcurrent,
    leda_overvoltage,
    ext_overtemperature,
    other_error,
  };

private:
  nld_board (void);

  static nld_board g_inst;

  struct reset_hw_init_t { reset_hw_init_t (nld_board& brd); };


  struct devices_begin_t { };
  struct devices_end_t { };

  static constexpr unsigned int interrupt_priority_bits = 2;
  static constexpr unsigned int max_interrupt_priority = (1u << interrupt_priority_bits) - 1;

  // pre defined value for the comparator settings.
  static constexpr unsigned int comp_csr_value = 0
  | (0b01 << 12)   // low hysteresis
  | (0 << 11)      // output not inverted
  | (0b001 << 8)   // output = timer 1 break input
  | (0b100 << 4)   // inverting input select = COMP1_INM4 (PA4 with DAC_OUT1)
  | (0b00 << 2)    // high speed, full power
  | (0 << 1)       // PA1-PA4 switch open
  | (0 << 0)       // comparator 1 enable (set later)
  | 0;

#ifndef BOARD_MCU_COMMS_ONLY

  #if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)
    const uint32_t m_vrefint_cal = *(const uint16_t*)0x1FFFF7BA;
    const uint32_t m_vrefint_cal_33 = *(const uint16_t*)0x1FFFF7BA * 3300000/4;
  #endif

  uint16_t m_board_ovt = 800;
  bool m_ignore_errors = false;
  bool m_en_ext_adc = false;
  bool m_ext_ovt_error = false;

  status_t m_sticky_error = normal;

  utils::averaging_buffer<unsigned int, 32> m_vdda_avg;
  utils::simple_iir_filter<uint32_t, uint32_t, utils::simple_iir_filter_one_256i> m_vdda_iir;

  utils::averaging_buffer<uint16_t, 32, uint32_t> m_vin_avg;
  utils::simple_iir_filter<uint32_t, uint32_t, utils::simple_iir_filter_one_256i> m_vin_iir;

  #ifndef APP_DISABLE_POWEROFF_HANDLING
  utils::averaging_buffer<int16_t, 16, int32_t> m_vin_delta;
  utils::simple_iir_filter<int32_t, int32_t, utils::simple_iir_filter_one_256i> m_vin_delta_iir;
  #endif

  utils::averaging_buffer<uint32_t, 32> m_leda_avg;
  utils::simple_iir_filter<uint32_t, uint32_t, utils::simple_iir_filter_one_256i> m_leda_iir;

  utils::averaging_buffer<uint32_t, 64> m_zxld_vref_avg;
  utils::simple_iir_filter<uint32_t, uint32_t, utils::simple_iir_filter_one_256i> m_zxld_vref_iir;

  utils::averaging_buffer<uint8_t, 16, uint32_t> m_zxld_status_avg;

  utils::averaging_buffer<uint16_t, 32, uint32_t> m_temp_avg;
  utils::simple_iir_filter<uint32_t, uint32_t, utils::simple_iir_filter_one_256i> m_temp_iir;

  utils::averaging_buffer<uint32_t, 32> m_ext_adc_avg;
  utils::simple_iir_filter<uint32_t, uint32_t, utils::simple_iir_filter_one_256i> m_ext_adc_iir;

  std::chrono::high_resolution_clock::time_point m_last_adc_update_time;

  // FLAG output needs least 100 microseconds filter.
  static constexpr auto zxld_flag_update_interval = std::chrono::microseconds (50);
  std::chrono::high_resolution_clock::time_point m_last_zxld_flag_sample_time;
  utils::digital_denoise< 16 > m_zxld_flag;

  std::optional<std::chrono::high_resolution_clock::time_point> m_led_on_time;

  #ifndef APP_DISABLE_POWEROFF_HANDLING
  std::chrono::high_resolution_clock::time_point m_last_vin_delta_update_time;
  int m_last_vin_delta_vin = 0;
  #endif

  // the ADJ and GI target values are compared against the actual measured
  // values to adjust the PWM settings.
  uint32_t m_zxld_adj_target_uv = 0;
  uint32_t m_zxld_gi_target_permille = 0;
#endif

  bool m_debug_console_enable = false;

  void update_adc_measurements (void);
  void update_adj_gi_adc_measurements (void);
  void adjust_zxld_adj_gi_pwm (void);
  void update_zxld_status_flag (std::chrono::high_resolution_clock::time_point cur_time);

  struct vdda_now_t
  {
    unsigned int uv_value;
    unsigned int adc_value;
  };
  vdda_now_t vdda_now (void) noexcept;
  unsigned int temperature_ddeg_c_now (unsigned int vdda_adc_val) noexcept;


  unsigned int zxld_adj_uv_to_pwm (uint32_t uv);
  unsigned int zxld_adj_uv_to_pwm (uint32_t uv, uint32_t vdda_uv);
  uint32_t zxld_adj_pwm_to_uv (unsigned int pwm_val, uint32_t vdda_uv);

  unsigned int zxld_gi_permille_to_pwm (unsigned int permille);
  unsigned int zxld_gi_permille_to_pwm (unsigned int permille, unsigned int adj_pwm);

  unsigned int ext_adc_uv_now (void) noexcept;

public:
  static constexpr nld_board& inst (void) { return g_inst; }

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

  // does some board-specific things like checking for a magic user input
  // sequence and reset the configuration partition to clear all configuration
  // data.
  void maybe_reset_config_to_factory_default (void);


  // convert relative raw 12 bit adc value to absolute micro volts
  unsigned int adc_to_uv (unsigned int adc_val)
  {
    // avoid 64 bit calculations
    //return ((uint64_t)vdda () * adc_val) / 4095;
    //return ((vdda ()/8) * adc_val) / (4096/8);
    return ((vdda_uv () >> 3) * adc_val) >> 9;
  }

  // convert a voltage value (mV) to VIN / LEDA DAC or ADC value
  // (DAC and ADC are 12 bit and use 150K/4.7K dividers)
  unsigned int vin_to_adc_value (unsigned int millivolt)
  {
    // (millivolt/1000 * (4700/(150000+4700))) / ((vdda/1000000) / 4096)
    // millivolt/1000 * 4700/154700 * 4096/(vdda/1000000)
    // millivolt * 4700/154700 * 4096/(vdda/1000)
    // millivolt/1 * 4700/154700 * 4096/1 * (1000/vdda)
    // (millivolt * 4700 * 4096 * 1000) / (154700 * vdda)
    // (millivolt * 4700 * 4096/8 * 10) / (1547 * vdda/8)
    // return ((uint64_t)millivolt * (47000/8) * 4096) / (1547 * (vdda ()/8));

    // avoid 64 bit calculations
    return (millivolt * ((47000 * 4096)/16384)) / ((1547 * (vdda_uv ()/8))/(16384/8));
  }

  unsigned int leda_to_adc_value (unsigned int millivolt)
  {
    return vin_to_adc_value (millivolt);
  }

  unsigned int leda_to_dac_value (unsigned int millivolt)
  {
    return vin_to_adc_value (millivolt);
  }

  // convert ADC value to millivolts on LEDA/Vin scale
  unsigned int adc_to_leda (unsigned int adc_val)
  {
    // vout = (vin * r2) / (r1 + r2)
    // vout * (r1 + r2) / r2 = vin
    // vin [V] = vout [V] * (150000 + 4700) / 4700
    // vin [mV] = vout [mV] * (150000 + 4700) / 4700
    // vin [mV] = (vout [uV] / 1000) * (154700 / 4700)
    // vin [mV] = (vout [uV] * 154700) / (1000 * 4700)
    // vin [mV] = (vout [uV] * 1547) / (10 * 4700)

    return ((adc_to_uv (adc_val)/4) * (1547)) / (47000/4);
  }

  unsigned int adc_to_vin (unsigned int adc_val)
  {
    return adc_to_leda (adc_val);
  }

  void enable_ext_adc (bool val);
  void enable_ext_i2c (bool val);

  // returns true if both i2c lines are high
  bool i2c_idle (void);


#ifndef BOARD_MCU_COMMS_ONLY
  bool is_ext_ovt_error (void) const { return m_ext_ovt_error; }
  void set_ext_ovt_error (bool val) { m_ext_ovt_error = val; }
#else
  bool is_ext_ovt_error (void) const { return false; }
  void set_ext_ovt_error (bool) { }
#endif

#ifndef BOARD_MCU_COMMS_ONLY
  bool is_ignore_errors (void) const { return m_ignore_errors; }
  void set_ignore_errors (bool val) { m_ignore_errors = val; }
#else
  bool is_ignore_errors (void) const { return false; }
  void set_ignore_errors (bool) { }
#endif

  static constexpr bool is_error (status_t s) { return s > ready; }

  status_t status (void);

  unsigned int zxld_status_mv (void);
  unsigned int zxld_status_mv_raw (void);
  bool zxld_status_flag (void);
  bool zxld_status_flag_now (void);

  void led_on (void);
  void led_off (void);
  void led_ready (void);


  // ADC readings of various channels
  unsigned int leda_mv (void) noexcept;
  unsigned int vin_mv (void) noexcept;
  int vin_delta_mv (void) noexcept;
  unsigned int zxld_vref_uv (void) noexcept;
  unsigned int zxld_adj_uv (void) noexcept;
  unsigned int zxld_gi_uv (void) noexcept;
  unsigned int adc_vrefint (void) noexcept;

  unsigned int ext_adc_uv (void) noexcept;
#ifndef BOARD_MCU_COMMS_ONLY
  unsigned int ext_adc_iir_k_one (void) const noexcept { return m_ext_adc_iir.k_one_value (); }
  void set_ext_adc_iir_k (unsigned int val) { m_ext_adc_iir.set_k (val); }
#else
  unsigned int ext_adc_iir_k_one (void) const noexcept { return 1; }
  void set_ext_adc_iir_k (unsigned int) { }
#endif


  // converted GI uV value to permille
  unsigned int zxld_gi (void) noexcept;

  // deci-degrees celcius
  unsigned int temperature_ddeg_c (void) noexcept;

  unsigned int vdda_uv (void) noexcept;

  // LEDA overvoltage threshold in millivolts
  unsigned int leda_ovp (void) noexcept;
  void set_leda_ovp (unsigned int mv) noexcept;

  // overtemperature threshold in degrees celcius
  // uses the periodic MCU core temperature measurement.
  // if the threshold is exceeded, the zxld pwm is stopped.
#ifndef BOARD_MCU_COMMS_ONLY
  unsigned int board_ovt (void) const noexcept { return m_board_ovt / 10; }
  void set_board_ovt (utils::clamped_value<unsigned int, 0, 100> deg_c) { m_board_ovt = deg_c * 10; }
#else
  unsigned int board_ovt (void) const noexcept { return 0; }
  void set_board_ovt (utils::clamped_value<unsigned int, 0, 100>) { }
#endif

  void reset_error (void);


  // when the ADJ target value is changed, the GI value is re-calculated so
  // that it effectively stays the same.
#ifndef BOARD_MCU_COMMS_ONLY
  unsigned int zxld_target_adj (void) noexcept { return m_zxld_adj_target_uv; }
#else
  unsigned int zxld_target_adj (void) noexcept { return 0; }
#endif
  void set_zxld_target_adj (unsigned int uv) noexcept;

#ifndef BOARD_MCU_COMMS_ONLY
  unsigned int zxld_target_gi (void) noexcept { return m_zxld_gi_target_permille; }
#else
  unsigned int zxld_target_gi (void) noexcept { return 0; }
#endif
  void set_zxld_target_gi (unsigned int permille);

  void set_zxld_target_adj_gi (unsigned int adj_uv, unsigned int gi_permille);

  static enum reset_source reset_source (void);
  static int reset_counter (void);
  static void set_reset_counter (int val);

  using board_id_t = std::array<uint8_t, 128/8>;
  board_id_t board_id (void) const;

  devices_begin_t devices_begin;

  reset_hw_init_t reset_hw_init;

  using system_timer_t = dev::armv6m_systick<
	nld_board_clk::system_clock_frequency_hz,
	this_board_interrupts::line<this_board_interrupts::systick>,
	max_interrupt_priority>;

  system_timer_t system_timer;

#ifndef BOARD_MCU_COMMS_ONLY
  using tim2_t = dev::stm32f0_tim::hw_inst<
	0x40000000, 4, nld_board_clk::pclk_hz,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	this_board_interrupts::line<this_board_interrupts::tim2>>;

  tim2_t tim2;
#endif

#ifndef BOARD_MCU_COMMS_ONLY
  using tim3_t = dev::stm32f0_tim::hw_inst<
	0x40000400, 4, nld_board_clk::pclk_hz,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	this_board_interrupts::line<this_board_interrupts::tim3>>;

  tim3_t tim3;

  struct tim3_inst_func { constexpr tim3_t& operator () (void) const { return nld_board::inst ().tim3; } };
#endif

#ifndef BOARD_MCU_COMMS_ONLY
  using tim6_t = dev::stm32f0_tim::hw_inst<
	0x40001000, 1, nld_board_clk::pclk_hz,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	this_board_interrupts::line<this_board_interrupts::tim6_dac>>;

  tim6_t tim6;
#endif

#ifndef BOARD_MCU_COMMS_ONLY
  using tim14_t = dev::stm32f0_tim::hw_inst<
	0x40002000, 1, nld_board_clk::pclk_hz,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	this_board_interrupts::line<this_board_interrupts::tim14>>;

  tim14_t tim14;

  struct tim14_inst_func { constexpr tim14_t& operator () (void) const { return nld_board::inst ().tim14; } };
#endif


#ifndef BOARD_MCU_COMMS_ONLY
  using tim1_t = dev::stm32f0_tim::hw_inst<
	0x40012C00, 4, nld_board_clk::pclk_hz,
	this_board_interrupts::line<this_board_interrupts::tim_brk_up_trg_com>,
	this_board_interrupts::line<this_board_interrupts::tim1_cc>,
	dev::interrupt::unconnected>;

  tim1_t tim1;

  struct tim1_inst_func { constexpr tim1_t& operator () (void) const { return nld_board::inst ().tim1; } };
#endif

#ifndef BOARD_MCU_COMMS_ONLY
  using tim15_t = dev::stm32f0_tim::hw_inst<
	0x40014000, 2, nld_board_clk::pclk_hz,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	this_board_interrupts::line<this_board_interrupts::tim15>>;

  tim15_t tim15;

  struct tim15_inst_func { constexpr tim15_t& operator () (void) const { return nld_board::inst ().tim15; } };
#endif

#if 1
  using tim16_t = dev::stm32f0_tim::hw_inst<
	0x40014400, 1, nld_board_clk::pclk_hz,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,

	// indicator LED PWM timer interrupts are not needed by software
	//this_board_interrupts::line<this_board_interrupts::tim16>>;
	dev::interrupt::unconnected>;

  tim16_t tim16;

  struct tim16_inst_func { constexpr tim16_t& operator () (void) const { return nld_board::inst ().tim16; } };
#endif

#ifndef BOARD_MCU_COMMS_ONLY
  using tim17_t = dev::stm32f0_tim::hw_inst<
	0x40014800, 1, nld_board_clk::pclk_hz,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,

	// indicator LED PWM timer interrupts are not needed by software
	//this_board_interrupts::line<this_board_interrupts::tim17>>;
	dev::interrupt::unconnected>;

  tim17_t tim17;

  struct tim17_inst_func { constexpr tim17_t& operator () (void) const { return nld_board::inst ().tim17; } };
#endif

// max. current per IO = 25 mA
#if defined (BOARD_VARIANT_A02)
  using white_led_t = dev::led_output<tim14_inst_func, 800, 2000>;
  using red_led_t = dev::led_output<tim17_inst_func, 1000, 2000>;
#elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
  using white_led_t = dev::led_output<tim16_inst_func, 800, 2000>;

  #ifndef BOARD_MCU_COMMS_ONLY
    using red_led_t = dev::led_output<tim17_inst_func, 1000, 2000>;
  #endif
#endif

  white_led_t white_led;
  struct white_led_inst { constexpr white_led_t& operator () (void) const { return nld_board::inst ().white_led; } };

  #ifndef BOARD_MCU_COMMS_ONLY
    red_led_t red_led;
    struct red_led_inst { constexpr red_led_t& operator () (void) const { return nld_board::inst ().red_led; } };
  #endif


#ifndef BOARD_MCU_COMMS_ONLY

  // use 12 kHz PWM carrier frequency for ADJ and GI outputs
  using zxld_adj_t = dev::zxld_adj<tim15_inst_func, 12'000>;
  zxld_adj_t zxld_adj;

  #if defined (BOARD_VARIANT_A02)
    using zxld_pwm_t = dev::zxld_pwm_A02<tim1_inst_func, tim16_inst_func>;
  #elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
    using zxld_pwm_t = dev::zxld_pwm_A03<tim1_inst_func, tim3_inst_func>;
  #endif
  zxld_pwm_t zxld_pwm;

  using dac_t = dev::stm32f0_dac::hw_inst<0x40007400>;
  dac_t dac;

  using comp_t = dev::stm32f0_comp::hw_inst<0x4001001C>;
  comp_t comp;

  #if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)
    using adc_t = dev::stm32f0_adc::hw_inst<0x40012400, pclk_hz/4>;
  #elif defined (BOARD_VARIANT_A04_GD32)
    using adc_t = dev::gd32f1_adc::hw_inst<0x40012400, pclk_hz/4>;
  #endif

  adc_t adc;

#endif

#ifdef BOARD_USE_DEBUG_USART
  using debug_usart_t =
	dev::stm32f0_usart::hw_inst<0x40004400, nld_board::pclk_hz,
	this_board_interrupts::line<this_board_interrupts::usart2>,
	32, 32, 3'000'000

  #ifdef BOARD_NO_RESET_HW_INIT
        , true
  #endif
  >;

  debug_usart_t debug_usart;
#endif

  // the debug usart is shared between debug console and external rs232.
  // if external rs232 comms is used, user code should turn off the debug console
  // which will stop redirecting all stdout/stderr messages to usart2.
  bool debug_console_enabled (void) const { return m_debug_console_enable; }
  void enable_debug_console (bool val) { m_debug_console_enable = val; }

  enum struct debug_usart_detection_result
  {
    not_connected,
    ext_rs232,
    debug_terminal
  };

  debug_usart_detection_result check_debug_usart_connection (void) const;


  using rs485_usart_t =
	dev::stm32f0_usart::hw_inst<0x40013800, nld_board::pclk_hz,
	this_board_interrupts::line<this_board_interrupts::usart1>,
	32, 32, 3'000'000
   #ifdef BOARD_NO_RESET_HW_INIT
        , true
   #endif
   >;

  rs485_usart_t rs485_usart;

#ifndef BOARD_MCU_COMMS_ONLY
  #if defined (BOARD_VARIANT_A02)
    using trigout_output_t = dev::stm32f0_digital_output<
        dev::stm32f0_gpio::pb, 5 >;
  #elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
    using trigout_output_t = dev::stm32f0_digital_output<
        dev::stm32f0_gpio::pf, 1 >;
  #endif


  #if defined (BOARD_VARIANT_A02)
    using extrun_output_t = dev::stm32f0_digital_output<
        dev::stm32f0_gpio::pb, 6 >;
  #elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
    using extrun_output_t = dev::stm32f0_digital_output<
        dev::stm32f0_gpio::pa, 13 >;
  #endif


  extrun_output_t extrun;
  trigout_output_t trigger_out;

  #if defined (BOARD_VARIANT_A02)
    using trigin_input_t = dev::stm32f0_igital_input<
	dev::stm32f0_gpio::pb, 3,
	this_board_interrupts::line<this_board_interrupts::exti2_3>>;

    using leden_input_t = dev::stm32f0_digital_input<
	dev::stm32f0_gpio::pb, 4, 4,
	dev::interrupt::virtual_line<256>>;

    using zxldflag_input_t = dev::stm32f0_digital_input<
	dev::stm32f0_gpio::pa, 11, 11,
	dev::interrupt::virtual_line<257>>;

    // interrupt demux is used for critical input trigger lines,
    // set its priority to the highest.
    using exti4_15_demux_t = dev::virtual_interrupt_demux<
	this_board_interrupts::line<this_board_interrupts::exti4_15>,
	dev::interrupt::max_priority,
	dev::interrupt::virtual_line<256>,
	dev::interrupt::virtual_line<257>>;

    exti4_15_demux_t exti4_15_demux;

  #elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
    using trigin_input_t = dev::stm32f0_digital_input<
	dev::stm32f0_gpio::pa, 11, 11,
	this_board_interrupts::line<this_board_interrupts::exti4_15>>;

    using leden_input_t = dev::stm32f0_digital_input<
	dev::stm32f0_gpio::pb, 3, 3,
	dev::interrupt::virtual_line<256>>;

    using zxldflag_input_t = dev::stm32f0_digital_input<
	dev::stm32f0_gpio::pb, 2, 2,
	dev::interrupt::virtual_line<257>>;

    // interrupt demux is used for critical input trigger lines,
    // set its priority to the highest.
    using exti2_3_demux_t = dev::virtual_interrupt_demux<
	this_board_interrupts::line<this_board_interrupts::exti2_3>,
	dev::interrupt::max_priority,
	dev::interrupt::virtual_line<256>,
	dev::interrupt::virtual_line<257>>;

    exti2_3_demux_t exti2_3_demux;
  #endif

  trigin_input_t trigger_in;
  leden_input_t led_en;
  zxldflag_input_t zxld_flag;

  // the page offset address and size of the flash is initialized in the
  // constructor
  using data_flash_t = dev::stm32f0_flash::hw_inst<1024>;
  data_flash_t data_flash;


#if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)

  using i2c_master_t = dev::stm32f0_i2c::i2c_master<0x40005400, nld_board::pclk_hz>;

#elif defined (BOARD_VARIANT_A04_GD32)

  using i2c_master_t = dev::gd32f1_i2c::i2c_master<0x40005400, nld_board::pclk_hz>;

#endif

  i2c_master_t i2c_master;

#endif // BOARD_MCU_COMMS_ONLY

#ifndef BOARD_NO_CRC
  using crc_t = dev::stm32f0_crc::hw_inst<>;
  crc_t crc;
#endif

  devices_end_t devices_end;
};

namespace this_board
{
using type = nld_board;
inline constexpr nld_board& inst (void) { return nld_board::inst (); }

}

namespace std
{
std::string_view to_string (nld_board::status_t val) noexcept;

} // namespace std

#endif // __cplusplus
#endif // includeguard_nld_board_hpp_includeguard
