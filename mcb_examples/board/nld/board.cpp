
#include <cstdio>
#include <chrono>
#include <thread>

#include <board/board.hpp>
#include <board/board_info.hpp>

#include <utils/rodata.hpp>
#include <utils/text.hpp>
#include <utils/math.hpp>

#include <dev/cpu.hpp>

#if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)
  #include <dev/stm32f0/stm32f0_rcc.hpp>
#elif defined (BOARD_VARIANT_A04_GD32)
  #include <dev/gd32f1/gd32f1_rcu.hpp>
#endif

#include <dev/stm32f0/stm32f0_gpio.hpp>
#include <dev/stm32f0/stm32f0_exti.hpp>
#include <dev/stm32f0/stm32f0_syscfg.hpp>
#include <dev/stm32f0/stm32f0_dac.hpp>


static constexpr unsigned int adc_ch_leda = 1;
static constexpr unsigned int adc_ch_zxld_vref = 5;

#if defined (BOARD_VARIANT_A02)
  static constexpr unsigned int adc_ch_vin = 8;
  static constexpr unsigned int adc_ch_zxld_status = 6;
  static constexpr unsigned int adc_ch_zxld_adj = 3;
  static constexpr unsigned int adc_ch_zxld_gi = 2;
#elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
  static constexpr unsigned int adc_ch_vin = 9;
  static constexpr unsigned int adc_ch_zxld_status = 8;
  static constexpr unsigned int adc_ch_zxld_adj = 2;
  static constexpr unsigned int adc_ch_zxld_gi = 3;
  static constexpr unsigned int adc_ch_ext = 0;
#endif

static constexpr unsigned int adc_ch_vdda = 17;
static constexpr unsigned int adc_ch_tempsensor = 16;


[[gnu::cold]] static void
emergency_puts (const char* str,
		unsigned int max_count = std::numeric_limits<unsigned int>::max ());

// make sure that the board constructor is ran as the very first static
// initializer by specifying the init_priority attribute.
[[gnu::init_priority (101), gnu::used]] nld_board nld_board::g_inst;

[[gnu::cold]]
nld_board::reset_hw_init_t::reset_hw_init_t (nld_board& brd)
{
#ifndef BOARD_NO_RESET_HW_INIT

  #if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)
    // set HSION
    dev::stm32f0_rcc::cr |= 1;

    // reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE and MCOSEL[2:0]
    dev::stm32f0_rcc::cfgr &= 0xF8FFB80C;

    // reset HSEON, CSSON and PLLON
    dev::stm32f0_rcc::cr &= 0xFEF6FFFF;

    // reset HSEBYP
    dev::stm32f0_rcc::cr &= 0xFFFBFFFF;

    // reset PLLSRC, PLLXTPRE and PLLMUL[3:0]
    dev::stm32f0_rcc::cfgr &= 0xFFC0FFFF;

    // reset PREDIV1[3:0]
    dev::stm32f0_rcc::cfgr2 &= 0xFFFFFFF0;

    // reset USARTSW[1:0], I2CSW, CECSW and ADCSW
    dev::stm32f0_rcc::cfgr3 &= 0xFFFFFEAC;

    // Reset HSI14
    dev::stm32f0_rcc::cr2 &= 0xFFFFFFFE;

    // disable all interrupts
    dev::stm32f0_rcc::cir = 0;

    // wait until HSI is ready
    while ((dev::stm32f0_rcc::cr & (1 << 1)) == 0) { }

    // we can now be sure to be running off the HSI 8 MHz clock.

    // before switching the clocks, enable flash prefetch buffer and set
    // flash wait state if needed (desired sysclk > 24 MHz)
    static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x40022000>> flash_acr = { };
    flash_acr = (1 << 4) | ((sysclk_hz > 24'000'000) ? 1 : 0);

    // HCLK = SYSCLK: keep CFGR HPRE at 0 (= div 1)
    static_assert (sysclk_hz == hclk_hz, "");

    // PCLK = HCLK: keep CFGR PPRE at 0 (= div 1)
    static_assert (pclk_hz == hclk_hz, "");

    constexpr int pll_mul = sysclk_hz / (hsi_hz/2) - 2;
    static_assert (pll_mul >= 0, "");
    static_assert (pll_mul <= 14, "");

    dev::stm32f0_rcc::cfgr |= (pll_mul << 18);

    // enable PLL and wait for it to come up
    dev::stm32f0_rcc::cr |= (1 << 24);
    while ((dev::stm32f0_rcc::cr & (1 << 25)) == 0) { }

    // select PLL as system clock source and wait until it is ready
    dev::stm32f0_rcc::cfgr |= 0b10;
    while ((dev::stm32f0_rcc::cfgr & 0b1100) != 0b1000) { }


    // enable AHB clocks for GPIO port A, GPIO port B, GPIO port F, DMA, DMA2, CRC
    dev::stm32f0_rcc::ahbenr |= 0b000'0100'0110'0000'0000'0100'0011;

    // enable APB clocks for ...
    dev::stm32f0_rcc::apb2enr = 0
	| (1 << 18)	// TIM17
	| (1 << 17)	// TIM16
	| (1 << 16)	// TIM15
	| (1 << 14)	// USART1
	| (1 << 11)	// TIM1
	| (1 <<  9)	// ADC
	| (1 <<  0)	// SYSCFG/COMP
	| 0;

    // enable APB clocks for ...
    dev::stm32f0_rcc::apb1enr = 0
	| (1 << 29)	// DAC
	| (1 << 28)	// PWR
	| (1 << 21)	// I2C1EN
	| (1 << 17)	// USART2
	| (1 <<  8)	// TIM14
	| (1 <<  5)	// TIM7
	| (1 <<  4)	// TIM6
	| (1 <<  1)	// TIM3
	| (1 <<  0)	// TIM2
	| 0;

  #elif defined (BOARD_VARIANT_A04_GD32)

    // at power-on the CPU runs off the internal 8 MHz oscillator (IRC8)
    // but we might get here also due to soft-reset ...
    dev::gd32f1_rcu::ctl0 = (dev::gd32f1_rcu::ctl0 & 0b11111100'11110000'11111111'11111100)
      | (1 << 0) // IRC8MEN = 1
      | 0;

    // SCS[1:0]     = 0b00 (use CK_IRC8M)
    // AHBPSC[3:0]  = 0b0000 (use CK_SYS)
    // APB1PSC[2:0] = 0b000 (use CK_AHB)
    // APB2PSC[2:0] = 0b000 (use CK_AHB)
    // ADCPSC[1:0]  = 0b01 (use CK_APB2/4)
    // CKOUTSEL[2:0] = 0b00 (no clock out)
    dev::gd32f1_rcu::cfg0 = (dev::gd32f1_rcu::cfg0 & 0xF8FF000C) | (0b01 << 14);
//    dev::gd32f1_rcu::cfg0 = (dev::gd32f1_rcu::cfg0 & 0xF8FF000C) | (0b11 << 14);   // ADC = 6 MHz

    // ADC clock = PCLK/4
    // ADC max clock = 14 MHz
    static_assert (pclk_hz/4 <= 14'000'000, "");

    // HXTALEN = off
    // CKMEN = off
    // PLLEN = off
    dev::gd32f1_rcu::ctl0 &= 0xFEF6FFFF;

    // HXTALBPS = off
    dev::gd32f1_rcu::ctl0 &= 0xFFFBFFFF;

    // PLLSEL = 0 (CK_IRC8M/2)
    // PLLPREDV = 0 (HXTAL)
    // PLLMF = 0b00000 (PLL x 2)
    dev::gd32f1_rcu::cfg0 &= 0xF700FFFF;

    // HXTALPREDV: HXTAL = PLL
    dev::gd32f1_rcu::cfg1 &= 0xFFFFFFF0;

    // USART0SEL = 0b00 (CK_APB2)
    // ADCSEL    = 1 (CK_APB2 / {2|4|6|8}
    dev::gd32f1_rcu::cfg2 = (dev::gd32f1_rcu::cfg2 & 0xFFFFFEFC) | (1 << 8);

    // IRC14MEN = off
    dev::gd32f1_rcu::ctl1 &= 0xFFFFFFFE;

    // disable all interrupts
    dev::gd32f1_rcu::int_ = 0;

    // wait until IRC8M is stable
    while ((dev::gd32f1_rcu::ctl0 & (1 << 1)) == 0) { }

    // we can now be sure to be running off the IRC8M 8 MHz clock.

    // on GD32 we don't need to use flash wait states when accessing the first
    // 32 KB flash.  it actually copies the 32 KB from flash to internal SRAM
    // at boot-up and executes from there.

    // GD32 CK_AHB = STM32 HCLK
    // CK_AHB = CK_SYS
    static_assert (sysclk_hz == hclk_hz, "");

    // GD32 CK_APB1, CK_APB2 = PCLK = CK_AHB

    // PCLK = HCLK: keep CFGR PPRE at 0 (= div 1)
    static_assert (pclk_hz == hclk_hz, "");

    // could run the GD32 at 72 MHz, but let's keep it at 48 MHz to be more
    // like STM32.
    constexpr int pll_mul = sysclk_hz / (hsi_hz/2);
    static_assert (pll_mul >= 2);
    static_assert (pll_mul <= 32);

    constexpr int pll_mul_reg = pll_mul <= 16 ? (pll_mul - 2) : (pll_mul - 1);

    dev::gd32f1_rcu::cfg0 |= ((pll_mul_reg & 0b10000) << (27 - 4)) | ((pll_mul_reg & 0b01111) << 18);

    // enable PLL and wait for it to come up
    dev::gd32f1_rcu::ctl0 |= (1 << 24);
    while ((dev::gd32f1_rcu::ctl0 & (1 << 25)) == 0) { }

    // select PLL as system clock source and wait until the clock switch has finished
    dev::gd32f1_rcu::cfg0 |= 0b10;
    while ((dev::gd32f1_rcu::cfg0 & 0b1100) != 0b1000) { }

    // enable AHB clocks for ...
    dev::gd32f1_rcu::ahben |= 0
	| (1 << 22)	// PFEN
	| (1 << 20)	// PDEN
	| (1 << 19)	// PCEN
	| (1 << 18)	// PBEN
	| (1 << 17)	// PAEN
	| (1 << 6)	// CRCEN
	| (1 << 4)	// FMCSPEN
	| (1 << 2)	// SRAMSPEN
	| (1 << 0)	// DMAEN
	| 0;

    // enable APB clocks for ...
   dev::gd32f1_rcu::apb1en |= 0
	| (1 << 29)	// DACEN
	| (1 << 28)	// PMUEN
	| (1 << 21)	// I2C0EN
	| (1 << 17)	// USART1EN
	| (1 <<  8)	// TIMER13EN
	| (1 <<  4)	// TIMER5EN
	| (1 <<  1)	// TIMER2EN
	| (1 <<  0)	// TIMER1EN
	| 0;

   // enable APB clocks for ...
   dev::gd32f1_rcu::apb2en |= 0
	| (1 << 18)	// TIMER16EN
	| (1 << 17)	// TIMER15EN
	| (1 << 16)	// TIMER14EN
	| (1 << 14)	// USART0EN
	| (1 << 11)	// TIMER0EN
	| (1 <<  9)	// ADCEN
	| (1 <<  0)	// CFGCMPEN
	| 0;

  #endif


#if defined (BOARD_VARIANT_A02)
  // PA9:  pull-down
  // PA10: pull-up
  // PA11: pull-up
  // PA12: pull-down
  // PA13: pull-up
  // PA14: pull-up
  dev::stm32f0_gpio::pupdr::pa = 0b00'01'01'10'01'01'10'00'00'00'00'00'00'00'00'00;

  // PA0  = ADC_IN0 (analog mode)
  // PA1  = COMP1_INP (analog mode)
  // PA2  = TIM15_CH1 (AF0)
  // PA3  = TIM15_CH2 (AF0)
  // PA4  = DAC_OUT1 (switched automatically when DAC enable, set to anaolog mode first)
  // PA5  = ADC_IN5 (analog mode)
  // PA6  = ADC_IN6 (analog mode)
  // PA7  = TIM17_CH1 (AF5)
  // PA8  = TIM1_CH1 (AF2)
  // PA9  = USART1_TX (AF1)
  // PA10 = USART1_RX (AF1)
  // PA11 = general input
  // PA12 = USART1_RTS (AF1)
  // PA13 = -/-
  // PA14 = USART2_TX (AF1)
  // PA15 = USART2_RX (AF1)
  dev::stm32f0_gpio::afrl::pa = 0b0101'0000'0000'0000'0000'0000'0000'0000;
  dev::stm32f0_gpio::afrh::pa = 0b0001'0001'0000'0001'0000'0001'0001'0010;
  dev::stm32f0_gpio::moder::pa = 0b10'10'00'10'00'10'10'10'10'11'11'11'10'10'11'11;

  // can't use on-chip pull-down for LED_EN because the input is 5V.
  // with on-chip pull-up/down enabled, the max. pin voltage is 4V.
  // PB4 (LED_EN): pull-down
  // dev::stm32f0_gpio::pupdr::pb = 0b00'00'00'00'00'00'00'00'00'00'00'10'00'00'00'00;

  // PB7: output high (open drain = open)
  dev::stm32f0_gpio::odr::pb = 0b10000000;

  // PB0  = ADC_IN8 (analog mode)
  // PB1  = TIM14_CH1 (AF0, white LED)
  // PB2  = -/-
  // PB3  = general input
  // PB4  = general input (switch to TIM3_CH1 input capture after timer setup)
  // PB5  = push-pull output (TRIGGER_OUT_PROJ+)
  // PB6  = push-pull output (EXT_RUN+)
  // PB7  = open-drain output (FT9885 SHDN)  (* try later)
  // PB8  = TIM16_CH1 (AF2)
  // PB9  = -/-
  dev::stm32f0_gpio::afrl::pb = 0b0000'0000'0000'0000'0000'0000'0000'0000;
  dev::stm32f0_gpio::afrh::pb = 0b0000'0010;
  dev::stm32f0_gpio::otyper::pb = 0b0010000000;
  dev::stm32f0_gpio::moder::pb = 0b00'10'01'01'01'00'00'00'10'11;
//  dev::stm32f0_gpio::moder::pb = 0b00'10'00'01'01'00'00'00'10'11;

  // PB3 -> EXTI3
  dev::stm32f0_syscfg::exticr1 = 0b0001 << 12;

  // PB4 -> EXTI4
  dev::stm32f0_syscfg::exticr2 = 0b0001 << 0;

  // PA11 -> EXTI11
  dev::stm32f0_syscfg::exticr3 = 0b0000 << 12;

#elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)

  // PA9:  pull-down
  // PA10: pull-up
  // PA12: pull-down
  // PA13: pull-up
  // PA14: pull-up
  // PA15: pull-up
  dev::stm32f0_gpio::pa::pupdr = 0b01'01'01'10'00'01'10'00'00'00'00'00'00'00'00'00;

  // PB2:  pull-up
  // PB0:  pull-down
  // dev::stm32f0_gpio::pb::pupdr = 0b00'00'00'00'00'00'00'00'00'00'00'00'00'01'00'10;
  dev::stm32f0_gpio::pb::pupdr = 0b00'00'00'00'00'00'00'00'00'00'00'00'00'01'00'00;

  // PA0  = ADC_IN0 (analog mode)
  // PA1  = COMP1_INP (analog mode)
  // PA2  = TIM15_CH1 (AF0)
  // PA3  = TIM15_CH2 (AF0)
  // PA4  = DAC_OUT1 (switched automatically when DAC enable, set to anaolog mode first)
  // PA5  = ADC_IN5 (analog mode)
  // PA6  = TIM3_CH1 (AF1)
  // PA7  = TIM17_CH1 (AF5)
  // PA8  = TIM1_CH1 (AF2) (ZXLD_PWM)
  // PA9  = USART1_TX (AF1)
  // PA10 = USART1_RX (AF1)
  // PA11 = general input (TRIGGER_IN)
  // PA12 = USART1_RTS (AF1)
  // PA13 = general output (EXT_RUN)
  // PA14 = USART2_TX (AF1)
  // PA15 = USART2_RX (AF1)
  dev::stm32f0_gpio::pa::afrl = 0b0101'0001'0000'0000'0000'0000'0000'0000;
  dev::stm32f0_gpio::pa::afrh = 0b0001'0001'0000'0001'0000'0001'0001'0010;
  dev::stm32f0_gpio::pa::moder = 0b10'10'01'10'00'10'10'10'10'10'11'11'10'10'11'11;
//  dev::stm32f0_gpio::pa::ospeedr = 0b00'00'11'00'00'00'00'11'00'00'00'00'00'00'00'00;

  // PB6: open drain (i2c) output high (sinking output open)
  // PB7: open drain (i2c) output high (sinking output open)
  dev::stm32f0_gpio::pb::odr = 0b11000000;

  // PB0  = ADC_IN8 (analog mode)
  // PB1  = ADC_IN9 (analog mode)
  // PB2  = general input (ZDLD FLAG)
  // PB3  = general input (LED_EN, can switch to TIM2_CH2 input capture after timer setup)
  // PB4  = general input (RS232_EN_SUP)
  // PB5  = -/- (set as output low)
  // PB6  = I2C1_SCL (AF1), open-drain output (i2c)
  // PB7  = I2C1_SDA (AF1), open-drain output (i2c)
  // PB8  = TIM16_CH1 (AF2)
  dev::stm32f0_gpio::pb::afrl = 0b0001'0001'0000'0000'0000'0000'0000'0000;
  dev::stm32f0_gpio::pb::afrh = 0b0010;
  dev::stm32f0_gpio::pb::otyper = 0b011000000;
  dev::stm32f0_gpio::pb::moder = 0b10'01'01'01'00'00'00'11'11;

  // PF0 = sourcing output / high-z (disable output by default)
  // PF1 = push-pull output
  dev::stm32f0_gpio::pf::odr = 0b01;
  dev::stm32f0_gpio::pf::afrl = 0b0000'0000;
  dev::stm32f0_gpio::pf::moder = 0b01'00;

  // PB2 -> EXTI2 (ZXLD_FLAG)
  // PB3 -> EXTI3 (LED_EN)
  dev::stm32f0_syscfg::exticr1 = (0b0001 << (3*4)) | (0b0001 << (2*4));

  // PA11 -> EXTI11 (TRIGGER_IN)
  dev::stm32f0_syscfg::exticr3 = 0b0000 << 12;

  #if defined (BOARD_VARIANT_A04_GD32)

    // enable PB0, PB1, PB2 decoupling capacitors
    static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x40010004>> syscfg_cfg1 = { };

    syscfg_cfg1 |= 0b1110;

  #endif


#endif

#endif // BOARD_NO_RESET_HW_INIT
}

// when the STM32 is running in normal mode, from user flash memory,
// the flash memory that starts at 0x08000000 is also mapped to 0x00000000.
// for programming the flash memory and using it as data flash, remap it
// into the 0x08000000 area.

// reserve the last 2 1-KB pages of the flash memory for the data flash
// and never erase it during flashing by using stm32flash option "-e n"
// and also by using flash write protection.
// this will preserve the data flash area across firmware updates.

static constexpr uintptr_t data_flash_rom_area_start (void)
{
  // assuming flash size of 32 KB for the STM32F0 that is used on this board.
  return (uintptr_t)(1024*(32-2)) | 0x08000000;
}

static constexpr uintptr_t data_flash_rom_area_end (void)
{
  return data_flash_rom_area_start () + (1024*2);
}

nld_board::nld_board (void)
 : devices_begin ()
 , reset_hw_init (*this)

#ifndef BOARD_MCU_COMMS_ONLY
 , data_flash (data_flash_rom_area_start (), data_flash_rom_area_end ())
#endif

 , devices_end ()
{
#ifndef BOARD_NO_RESET_HW_INIT

  // select PCLK/4 clock for ADC
  static_assert (pclk_hz/4 <= 14'000'000, "");

  #ifndef BOARD_MCU_COMMS_ONLY


  #if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)

    adc.regs ().cfgr1 = 0;
    adc.regs ().cfgr2 = 0b10 << 30;

    adc.regs ().ccr = 0
	| (0 << 24)		// VBAT EN = off
	| (1 << 23)		// temperature sensor on
	| (1 << 22)		// VREFINT enabled
	| 0;

    adc.calibrate ();
    adc.enable ();

  #elif defined (BOARD_VARIANT_A04_GD32)

    adc.calibrate ();
//    adc.enable ();

  #endif

  #endif

#endif // BOARD_NO_RESET_HW_INIT

#ifndef BOARD_MCU_COMMS_ONLY

  for (unsigned int i = 0; i < m_vdda_avg.max_size (); ++i)
    m_vdda_avg.push_back (vdda_now ().uv_value);

  m_vdda_iir.reset (m_vdda_avg.value ());

  // channel 1: software trigger, trigger disabled, output buffer disabled,
  //            channel enabled
  dac.regs ().dhr12r1 = 0;
  dac.regs ().cr = 0b111'0'1'1;

  // supported max. LED voltage is 60V and max. Vin = 24V.
  // 60V + 24V = 84V
  // allow some overshoot which is needed to flash the LED quickly.

  // set some value now, set it later again when the voltages have stabilized
  // after power-on.
  set_leda_ovp (95'000);

  comp.regs ().csr = comp_csr_value | (1 << 0); // enable comparator

  #ifndef APP_DISABLE_POWEROFF_HANDLING
  m_last_adc_update_time = m_last_vin_delta_update_time = m_last_zxld_flag_sample_time =
	std::chrono::high_resolution_clock::now ();
  #endif

  m_ext_adc_iir.set_k (m_ext_adc_iir.k_one_value () / 32);
  m_temp_iir.set_k (m_temp_iir.k_one_value () / 24);
  m_vdda_iir.set_k (m_vdda_iir.k_one_value () / 32);
  m_vin_iir.set_k (m_vin_iir.k_one_value () / 32);

  #ifndef APP_DISABLE_POWEROFF_HANDLING
  m_vin_delta_iir.set_k (m_vin_delta_iir.k_one_value () / 32);
  #endif

  m_leda_iir.set_k (m_leda_iir.k_one_value () / 32);
  m_zxld_vref_iir.set_k (m_zxld_vref_iir.k_one_value () / 32);

#endif // BOARD_MCU_COMMS_ONLY


#ifndef BOARD_NO_RESET_HW_INIT
  // initial rs485 usart configuration
  rs485_usart.set_config (rs485_usart.config ()
	.set (dev::usart::flow_ctrl::de_high));
#endif
}

int nld_board::reset_counter (void)
{
  return 0;
}

void nld_board::set_reset_counter (int val)
{
}

nld_board::debug_usart_detection_result nld_board::check_debug_usart_connection (void) const
{
#ifdef BOARD_USE_DEBUG_USART

  #ifdef BOARD_VARIANT_A03
    return debug_usart_detection_result::debug_terminal;
  #endif

  // determine whether RS232_EN_SUP is supplied externally or whether debug
  // usart is connected.
  // notice that RS232_EN_SUP comes up 40 ms after MCU start.
  //
  // if nothing is connected, RS232_EN_SUP will be supplied through the TXD
  // pull-up or active drive.  so before checking RS232_EN_SUP, TXD should be
  // set to output low, then RS232_EN_SUP input should be sampled.

  // PA15 = USART2_RX (AF1)
  // if the pin is open and we change the on-chip pull up / pull down
  // resistor configuration, the pin should toggle in some way.  if it does,
  // nothing is connected.  if it does not, most likely the debug usart
  // cable is connected.

  debug_usart_detection_result test_result = debug_usart_detection_result::not_connected;

  // disable usart rx/tx during the test otherwise it might receive wrong ghost
  // characters.
  auto& usart = debug_usart;
  auto prev_cr1 = usart.regs ().cr1.read ();
  usart.regs ().cr1 = 0;

  uint32_t prev_pupdr = dev::stm32f0_gpio::pa::pupdr;
  uint32_t prev_moder = dev::stm32f0_gpio::pa::moder;

  // PA14 (TX), PA15 (RX) set moder to 0b01 (general output mode)
  // and output low level.  this is to avoid reverse supplying the RS232
  // level-shifter chip which will pull-up RS232_EN_SUP.
  dev::stm32f0_gpio::pa::moder = (prev_moder & ~(0b1111 << (14*2))) | (0b0101 << (14*2));

  // wait for a while because the external supply that powers RS232_EN_SUP
  // might come on a while alter.  if it's high, it's probably powered
  // by an external source and the RS232 level-shifter for external RS232
  // is enabled.
  std::this_thread::sleep_for (std::chrono::milliseconds (100));

  if (dev::stm32f0_gpio::pb::idr & (1 << 4))
    test_result = debug_usart_detection_result::ext_rs232;
  else
  {
    // restore values and wait a while
    dev::stm32f0_gpio::pa::pupdr = prev_pupdr;
    dev::stm32f0_gpio::pa::moder = prev_moder;

    std::this_thread::sleep_for (std::chrono::milliseconds (10));

    // PA15 (RX) set moder to 0b00 (general input mode)
    dev::stm32f0_gpio::pa::moder = prev_moder & ~(0b11 << (15*2));

    // PA15 (RX) turn on pull-up resistor
    dev::stm32f0_gpio::pa::pupdr = prev_pupdr | (0b01 << (15*2));

    // read input for a while (it takes some time to settle)
    bool t0;
    for (int i = 0; i < 16; ++i)
      t0 = dev::stm32f0_gpio::pa::idr & (1 << 15);

    // PA15 (RX) turn on pull-down resistor
    dev::stm32f0_gpio::pa::pupdr = prev_pupdr | (0b10 << (15*2));

    // read input for a while (it takes some time to settle)
    bool t1;
    for (int i = 0; i < 16; ++i)
      t1 = dev::stm32f0_gpio::pa::idr & (1 << 15);

    if (!(t0 == true && t1 == false))
      test_result = debug_usart_detection_result::debug_terminal;
  }

  // restore IO config and re-enable usart.

  dev::stm32f0_gpio::pa::pupdr = prev_pupdr;
  dev::stm32f0_gpio::pa::moder = prev_moder;

  usart.regs ().cr1 = prev_cr1;

  return test_result;

#else
  return debug_usart_detection_result::not_connected;
#endif
}

nld_board::board_id_t nld_board::board_id (void) const
{
  #if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)

  constexpr auto&& board_id_str = "NLDA";

  #elif defined (BOARD_VARIANT_A04_GD32)

  constexpr auto&& board_id_str = "NLGD";

  #endif

  constexpr unsigned int board_id_str_len = std::strlen (board_id_str);

  board_id_t r;

  static_assert ((board_id_str_len + 96/8) <= r.size ());

  auto* i = r.begin ();

  std::memcpy (i, board_id_str, board_id_str_len);
  i += board_id_str_len;

  // on GD32 it has to be accessed as 32-bit words..
  std::memcpy (i, (const void*)0x1FFFF7AC, 96/8);
  i += 96/8;

  std::memset (i, 0, r.end () - i);

  return r;
}

// -----------------------------------------------------------------------------

unsigned int nld_board::leda_mv (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_leda_iir.value ();
#else
  return 0;
#endif
}

unsigned int nld_board::vin_mv (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_vin_iir.value ();
#else
  return 0;
#endif
}

int nld_board::vin_delta_mv (void) noexcept
{
#if !defined (BOARD_MCU_COMMS_ONLY) && !defined (APP_DISABLE_POWEROFF_HANDLING)
  return m_vin_delta_iir.value ();
#else
  return 0;
#endif
}

unsigned int nld_board::zxld_vref_uv (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_zxld_vref_iir.value ();
#else
  return 0;
#endif
}

unsigned int nld_board::zxld_adj_uv (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return zxld_adj_pwm_to_uv (zxld_adj.adj_value (), vdda_uv ());
#else
  return 0;
#endif
}

unsigned int nld_board::zxld_gi_uv (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return zxld_adj_pwm_to_uv (zxld_adj.gi_value (), vdda_uv ());
#else
  return 0;
#endif
}

unsigned int nld_board::zxld_gi (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  const auto zxld_adj = zxld_adj_uv ();
  const auto zxld_gi = zxld_gi_uv ();
  return zxld_adj == 0 ? 0 : ((zxld_gi * 1000) / zxld_adj);
#else
  return 0;
#endif
}

unsigned int nld_board::adc_vrefint (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return adc_to_uv (adc.convert_single (adc_ch_vdda, std::chrono::microseconds (10)));
#else
  return 0;
#endif
}

unsigned int nld_board::ext_adc_uv (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_en_ext_adc ? m_ext_adc_iir.value () : 0;
#else
  return 0;
#endif
}

unsigned int nld_board::temperature_ddeg_c (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_temp_iir.value ();
#else
  return 0;
#endif
}

unsigned int nld_board::temperature_ddeg_c_now (unsigned int vdda_adc_val) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY

  #if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)

    // TS_CAL1: raw data at 30 degC, VDDA = 3.3V
    // TS_CAL2: raw data at 110 degC, VDDA = 3.3V
    const int ts_cal1 = *(const uint16_t*)0x1FFFF7B8;
    const int ts_cal2 = *(const uint16_t*)0x1FFFF7C2;

    const int ts_data = (adc.convert_single (adc_ch_tempsensor, std::chrono::microseconds (10)) * m_vrefint_cal) / vdda_adc_val;

    int32_t slope16_16 = (10*(110 - 30) << 16) / (ts_cal2 - ts_cal1);
    return (slope16_16 * (ts_data - ts_cal1) + (30*10 << 16)) >> 16;

  #elif defined (BOARD_VARIANT_A04_GD32)

    // temp °C = ( (V25 - Vtemp) / avg_slope ) + 25
    //
    //  V25: Vtemp value at 25°C.  typical value is 1.43V
    //  avg_slope: averge slope curve of real temperature vs. Vtemp.  typical 4.3 mV/°C

    // FIXME: GD32 doesn't have factory calibration of VDDA VREF and temperature sensor
    // for now use those constants.
//    const int ts_cal1 = 1430;
//    const int slope16_16 = (int)(4.3 * 65536);

//    const int ts_data = adc.convert_single (adc_ch_tempsensor, std::chrono::microseconds (18));

//    return (slope16_16 * (ts_data - ts_cal1) + (25*10 << 16)) >> 16;

    const int vtemp_mv = adc_to_uv (adc.convert_single (adc_ch_tempsensor, std::chrono::microseconds (18))) / 1000;
    const int v25 = 1430;

    return ((((v25 - vtemp_mv) * 10) << 16) / ((int)(4.3 * 65536))) + 250;

  #endif


#else
  return 0;
#endif
}

nld_board::vdda_now_t nld_board::vdda_now (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY

  #if defined (BOARD_VARIANT_A02) || defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04_STM32)
    unsigned int vref_data = adc.convert_single (adc_ch_vdda, std::chrono::microseconds (10));
    return { (m_vrefint_cal_33 / vref_data) << 2, vref_data };

  #elif defined (BOARD_VARIANT_A04_GD32)

    // GD32 doesn't have factory calibration of VREF
    // we can only derive it as follows

    // vref data = adc value for 1.2V

    //  vdda data   3.3V
    //  --------- = ----
    //  vref data   1.2V
    //
    //  vdda data = 3.3V/1.2V * vref data

    //  max data =  VDDA
    // ---------   ----
    // vref data = 1.2V
    //
    // max data = max adc value 4095
    //
    //  4095 * 1.2V
    // ------------  = VDDA
    //   vref data

    //unsigned int vref_data = adc.convert_single (adc_ch_vdda, std::chrono::microseconds (18));

    return
    {
      // to avoid 64-bit arithmetic, make sure the numerator fits into uint32_t
//      (uint32_t(4095 * (1'200'000/4)) / vref_data) * 4,
//      (uint32_t(4095 * (1'600'000/4)) / vref_data) * 4,
      3310000,

      // adc raw value is not really used anywhere, except for temperature reading.
      4095
    };


  #endif

#else
  return { 0, 0 };
#endif
}

unsigned int nld_board::vdda_uv (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_vdda_iir.value ();
#else
  return 0;
#endif
}

unsigned int nld_board::leda_ovp (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return adc_to_leda (dac.regs ().dhr12r1);
#else
  return 0;
#endif
}

void nld_board::set_leda_ovp (unsigned int mv) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  dac.regs ().dhr12r1 = std::min (leda_to_dac_value (mv), 4095u);
#endif
}

unsigned int nld_board::zxld_status_mv (void)
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_zxld_status_avg.value () * (16384/500);
#else
  return 0;
#endif
}

unsigned int nld_board::zxld_status_mv_raw (void)
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_zxld_status_avg.value ();
#else
  return 0;
#endif
}

bool nld_board::zxld_status_flag_now (void)
{
#ifndef BOARD_MCU_COMMS_ONLY

  #if defined (BOARD_VARIANT_A02)
    return (dev::stm32f0_gpio::idr::pa & (1 << 11)) != 0;
  #elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
    return (dev::stm32f0_gpio::pb::idr & (1 << 2)) != 0;
  #endif

#else
  return false;
#endif
}

bool nld_board::zxld_status_flag (void)
{
#ifndef BOARD_MCU_COMMS_ONLY
  return m_zxld_flag.value ();
#else
  return false;
#endif
}


nld_board::status_t nld_board::status (void)
{
#ifndef BOARD_MCU_COMMS_ONLY
  // when comparing the ZXLD status threshold voltages, pre-scale the constant
  // values to match the scaling of the values in the averaging buffer and
  // the 150K/150K resistor divider.  because of the ADC measurement and the
  // averaging the voltage values will always be a bit less than the nominal
  // values in the datasheet.
  auto status_volt_to_val = [] (float val) { return (int)((val * 1'000'000) / 32768); };

  const int status_val = m_zxld_status_avg.value ();
  const auto pwm_st = zxld_pwm.status ();

  if (is_ext_ovt_error ())
    return ext_overtemperature;

  if (is_error (m_sticky_error))
    return m_sticky_error;

  else if (pwm_st == zxld_pwm.ready)
  {
    // in ready-mode it might drop from normal operation to the stalled
    // state.  ignore it.
    // when the zxld is starting up, it might report the standby state for
    // a while.
    if (status_val >= status_volt_to_val (3.0))
      return ready;
    else if (status_val >= status_volt_to_val (0.5))
    {
      // perhaps it's just starting up.
      return ready;
    }
    else
      return other_error;
  }

  else if (pwm_st == zxld_pwm.break_cond)
    return leda_overvoltage;

  else if (zxld_status_flag ())
  {
    // flag is de-asserted (high)
    return status_val < status_volt_to_val (0.5) ? standby : normal;
  }

  else
  {
    // flag is asserted (low), decode detailed error code

    // after many long trials and attempts at the hardware level, we could not
    // stabilize this condition.  something else must be missing there but
    // we can't figure it out.
    // on some boards the handling of the 'vaux_undervoltage',
    // 'stall_out_of_regulation', 'vin_undervoltage' causes a spurious error
    // to be detected after a longer operational time.
    // the best thing we could come up with is to ignore it :(

    if (status_val > status_volt_to_val (4.3))
//      return vaux_undervoltage;
      return normal;  // ignore this error

    if (status_val > status_volt_to_val (3.3))
//      return stall_out_of_regulation;
      return normal;  // ignore this error

    if (status_val > status_volt_to_val (1.9))
//      return vin_undervoltage;
      return normal;

    if (status_val > status_volt_to_val (1.0))
      return overtemperature;

    if (status_val > status_volt_to_val (0.5))
      return overcurrent;

    return other_error;
  }

#else // #ifndef BOARD_MCU_COMMS_ONLY
  return normal;
#endif
}


namespace std
{

std::string_view to_string (nld_board::status_t val) noexcept
{
  switch (val)
  {
  case nld_board::normal: return "normal";
  case nld_board::standby: return "standby";
  case nld_board::ready: return "ready";
  case nld_board::vaux_undervoltage: return "vaux undervoltage";
  case nld_board::vin_undervoltage: return "vin undervoltage";
  case nld_board::stall_out_of_regulation: return "stall / out of regulation";
  case nld_board::overtemperature: return "overtemperature";
  case nld_board::overcurrent: return "overcurrent";
  case nld_board::leda_overvoltage: return "leda overvoltage / led disconnected";
  case nld_board::ext_overtemperature: return "external over temperature";
  case nld_board::other_error: return "other error";
  default: return { };
  }
}

}

// -----------------------------------------------------------------------------

#ifndef BOARD_MCU_COMMS_ONLY
void nld_board::update_adc_measurements (void)
{
  const auto vdda = vdda_now ();
  m_vdda_avg.push_back (vdda.uv_value);
  m_vdda_iir.push_back (m_vdda_avg.value ());

  m_vin_avg.push_back ((uint16_t)adc_to_vin (adc.convert_single (adc_ch_vin, std::chrono::microseconds (10))));
  m_vin_iir.push_back (m_vin_avg.value ());

  // when ADC sampling starts it will output a significant pulse because no
  // buffers are used for the analog inputs.  normally this is not a problem,
  // but in this case the comparator will wrongly be triggered.
  // to avoid the false triggering, disable comparator during ADC sampling.
  // if an ISR runs while the comparator is disabled, and for some reason it
  // deadlocks the CPU, it will disable over-voltage protection.  thus, do not
  // allow any CPU interrupts during that time.
  {
    auto s = dev::this_cpu::save_disable_interrupts ();

      comp.regs ().csr = comp_csr_value | (0 << 0);
      const auto adc_val = adc.convert_single (adc_ch_leda, std::chrono::microseconds (2));
      comp.regs ().csr = comp_csr_value | (1 << 0); // enable comparator again

    dev::this_cpu::restore_interrupts (s);

    m_leda_avg.push_back (adc_to_leda (adc_val));
    m_leda_iir.push_back (m_leda_avg.value ());
  }

  m_zxld_vref_avg.push_back (adc_to_uv (adc.convert_single (adc_ch_zxld_vref, std::chrono::microseconds (10))));
  m_zxld_vref_iir.push_back (m_zxld_vref_avg.value ());

  m_temp_avg.push_back ((uint16_t)temperature_ddeg_c_now (vdda.adc_value));
  m_temp_iir.push_back (m_temp_avg.value ());

  // the STATUS output comes through a 150K/150K resistor divider
  // the max. STATUS voltage is about 4.5V
  m_zxld_status_avg.push_back (
	(uint8_t)(adc_to_uv (adc.convert_single (adc_ch_zxld_status, std::chrono::microseconds (10))) >> 14));

  if (m_en_ext_adc)
  {
    m_ext_adc_avg.push_back (ext_adc_uv_now ());
    m_ext_adc_iir.push_back (m_ext_adc_avg.value ());
  }

  // make the overtemperature condition sticky
  // if another error is already set, keep it.
  // on GD32 the temperature sensor is not calibrated and the reading can cause
  // false alarms.  just don't use it for now.
  #ifndef BOARD_VARIANT_A04_GD32
  if (m_temp_avg.value () > m_board_ovt && !is_error (m_sticky_error))
    m_sticky_error = overtemperature;
  #endif
}

void nld_board::update_adj_gi_adc_measurements (void)
{
  // in order to measure the ADJ and GI values we need to turn off the
  // timer's PWM output, sample with the ADC and then turn the PWM output
  // back on.
  // FIXME NLD-50: to reduce measurement errors, this should be done synchronized
  //               with the timer.

  // this function is not used at the moment.
  // it was used before to measure the ADJ and GI values in the hope to increase
  // the precision and compensate for component variations.  but it turned out
  // to cause more problems than it would solve.  the PWM output values are
  // almost spot-on with a small error of ± 2 mV.

  // the only thing that an ADC measurement of ADJ and GI values could detect
  // is a broken ZXLD, when its input impedance changes so badly that the
  // external RC filter can't drive the input anymore.  but this error can
  // also be detected by measuring the resulting LED current.  if the ADJ and GI
  // inputs are broken, the voltages will be too low and the LED current error
  // will be too large.

  uint32_t pa_mode = dev::stm32f0_gpio::pa::moder;

  dev::stm32f0_gpio::pa::moder = pa_mode | (0b11 << adc_ch_zxld_adj*2);
  //unsigned int adj_val = adc.convert_single (adc_ch_zxld_adj, std::chrono::microseconds (3));

  dev::stm32f0_gpio::pa::moder = pa_mode | (0b11 << adc_ch_zxld_gi*2);
  //unsigned int gi_val = adc.convert_single (adc_ch_zxld_gi, std::chrono::microseconds (3));

  dev::stm32f0_gpio::pa::moder = pa_mode;

//  m_zxld_adj_avg.push_back (adc_to_uv (adj_val));
//  m_zxld_gi_avg.push_back (adc_to_uv (gi_val));
}
#endif // BOARD_MCU_COMMS_ONLY

unsigned int nld_board::zxld_adj_uv_to_pwm (uint32_t uv)
{
#ifndef BOARD_MCU_COMMS_ONLY
  return zxld_adj_uv_to_pwm (uv, vdda_uv ());
#else
  return 0;
#endif
}

unsigned int nld_board::zxld_adj_uv_to_pwm (uint32_t uv, uint32_t vdda_uv)
{
#ifndef BOARD_MCU_COMMS_ONLY
  return (uv * zxld_adj.max_value ()) / (vdda_uv / 2);
#else
  return 0;
#endif
}

uint32_t nld_board::zxld_adj_pwm_to_uv (unsigned int pwm_val, uint32_t vdda_uv)
{
#ifndef BOARD_MCU_COMMS_ONLY
  return ((vdda_uv / 2) * pwm_val) / zxld_adj.max_value ();
#else
  return 0;
#endif
}

unsigned int nld_board::zxld_gi_permille_to_pwm (unsigned int permille)
{
#ifndef BOARD_MCU_COMMS_ONLY
  return zxld_gi_permille_to_pwm (permille, zxld_adj.adj_value ());
#else
  return 0;
#endif
}

unsigned int
nld_board::zxld_gi_permille_to_pwm (unsigned int permille, unsigned int adj_pwm_value)
{
  return (permille * adj_pwm_value + 999) / 1000;
}

[[gnu::cold]]
void nld_board::enable_ext_adc ([[maybe_unused]] bool val)
{
  #if (defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)) && !defined (BOARD_MCU_COMMS_ONLY)

  if (val)
  {
    // disable I2C pull-up on shared PB6 line
    dev::stm32f0_gpio::pf::moder = dev::stm32f0_gpio::pf::moder & 0b11'00;

    // PB6, PB7: digital output (open-drain), default output high
    dev::stm32f0_gpio::pb::moder = (dev::stm32f0_gpio::pb::moder
				    & 0b11'00'00'11'11'11'11'11'11)
				   |  0b00'01'01'00'00'00'00'00'00;

    m_ext_adc_avg.fill (ext_adc_uv_now ());
    m_ext_adc_iir.reset (m_ext_adc_avg.value ());

    m_en_ext_adc = true;
  }
  else
  {
    // enable I2C pull-up on shared PB6 line
    dev::stm32f0_gpio::pf::moder = dev::stm32f0_gpio::pf::moder | 0b00'01;

    // PB6, PB7: AF1 (I2C)
    dev::stm32f0_gpio::pb::moder = (dev::stm32f0_gpio::pb::moder
				    & 0b11'00'00'11'11'11'11'11'11)
				   |  0b00'10'10'00'00'00'00'00'00;

    m_en_ext_adc = false;
  }

  #endif
}

void nld_board::enable_ext_i2c (bool val)
{
  enable_ext_adc (!val);
}

bool nld_board::i2c_idle (void)
{
  // PB6  = I2C1_SCL (AF1), open-drain output (i2c)
  // PB7  = I2C1_SDA (AF1), open-drain output (i2c)
  return (dev::stm32f0_gpio::pb::idr & ((1 << 6) | (1 << 7))) == ((1 << 6) | (1 << 7));
}

unsigned int nld_board::ext_adc_uv_now (void) noexcept
{
#ifndef BOARD_MCU_COMMS_ONLY
  return adc_to_uv (adc.convert_single (adc_ch_ext, std::chrono::microseconds (20)));
#else
  return 0;
#endif
}

void nld_board::set_zxld_target_adj ([[maybe_unused]] unsigned int uv)
{
#ifndef BOARD_MCU_COMMS_ONLY
  // re-calculate GI whenever ADJ is changed to keep GI constant.
  // the initial values set here will be adjusted later by the control loop.

  m_zxld_adj_target_uv = uv;

  const auto new_adj_pwm = zxld_adj_uv_to_pwm (uv);
  zxld_adj.set_value (new_adj_pwm, zxld_gi_permille_to_pwm (m_zxld_gi_target_permille, new_adj_pwm));
#endif
}

void nld_board::set_zxld_target_gi ([[maybe_unused]] unsigned int permille)
{
#ifndef BOARD_MCU_COMMS_ONLY
  m_zxld_gi_target_permille = permille;
  zxld_adj.set_gi_value (zxld_gi_permille_to_pwm (permille));
#endif
}


void nld_board::set_zxld_target_adj_gi (unsigned int adj_uv, unsigned int gi_permille)
{
#ifndef BOARD_MCU_COMMS_ONLY

  m_zxld_adj_target_uv = adj_uv;
  m_zxld_gi_target_permille = gi_permille;

  const auto new_adj_pwm = zxld_adj_uv_to_pwm (adj_uv);
  const auto new_gi_pwm = zxld_gi_permille_to_pwm (m_zxld_gi_target_permille, new_adj_pwm);

  // to accelerate the transition of the RC filter on the PWM output, output
  // a constant high / low voltage until it is in the target range.
  // because we know the RC component values that are used on the board (10K / 1u)
  // we could calculate the required waiting time until the target value
  // has settled.  since we're going to wait here anyway, we can as well just
  // measure the values with the ADC and see if they are in range.

  int adj_change = (int)new_adj_pwm - (int)zxld_adj.adj_value ();
  int gi_change = (int)new_gi_pwm - (int)zxld_adj.gi_value ();

  const unsigned int cur_vdda = vdda_uv ();
  const int adj_target_uv = zxld_adj_pwm_to_uv (new_adj_pwm, cur_vdda);
  const int gi_target_uv = zxld_adj_pwm_to_uv (new_gi_pwm, cur_vdda);

  zxld_adj.set_value (new_adj_pwm, new_gi_pwm);

  unsigned int adj_threshold, gi_threshold;

  if (adj_change > 0)
  {
    zxld_adj.set_adj_output_const_high ();
    adj_threshold = 20000;
  }
  else if (adj_change < 0)
  {
    zxld_adj.set_adj_output_const_low ();
    adj_threshold = 7500;
  }

  if (gi_change > 0)
  {
    zxld_adj.set_gi_output_const_high ();
    gi_threshold = 20000;
  }
  else if (gi_change < 0)
  {
    zxld_adj.set_gi_output_const_low ();
    gi_threshold = 7500;
  }

  auto start_time = std::chrono::high_resolution_clock::now ();


  while (adj_change != 0 || gi_change != 0)
  {
    auto cur_time = std::chrono::high_resolution_clock::now ();

    // we know from simulation that the max. RC settling time is about 50 ms.
    // in case the ADC measurements oscillate or something like that, just
    // use that as a upper limit.
    if (cur_time - start_time > std::chrono::milliseconds (60))
    {
      zxld_adj.set_adj_output_pwm ();
      zxld_adj.set_gi_output_pwm ();
      break;
    }

    // FIXME: copy-pasted from 'update_adj_gi_adc_measurements'

    auto iiii = dev::this_cpu::save_disable_interrupts ();

    uint32_t pa_mode = dev::stm32f0_gpio::pa::moder;

    dev::stm32f0_gpio::pa::moder = pa_mode | (0b11 << adc_ch_zxld_adj*2);
    unsigned int adj_adc_val = adc.convert_single (adc_ch_zxld_adj, std::chrono::microseconds (10));

    dev::stm32f0_gpio::pa::moder = pa_mode | (0b11 << adc_ch_zxld_gi*2);
    unsigned int gi_adc_val = adc.convert_single (adc_ch_zxld_gi, std::chrono::microseconds (10));

    dev::stm32f0_gpio::pa::moder = pa_mode;

    dev::this_cpu::restore_interrupts (iiii);

    if (adj_change != 0)
    {
      int cur_adj_uv = adc_to_uv (adj_adc_val);
      if (std::abs (cur_adj_uv - adj_target_uv) < adj_threshold)
      {
	adj_change = 0;
	zxld_adj.set_adj_output_pwm ();
      }
    }

    if (gi_change != 0)
    {
      int cur_adj_gi = adc_to_uv (gi_adc_val);
      if (std::abs (cur_adj_gi - gi_target_uv) < gi_threshold)
      {
	gi_change = 0;
	zxld_adj.set_gi_output_pwm ();
      }
    }
  }

#endif
}


void nld_board
::update_zxld_status_flag ([[maybe_unused]] std::chrono::high_resolution_clock::time_point cur_time)
{
#ifndef BOARD_MCU_COMMS_ONLY

  // there are some spurious FLAG triggers shortly after the LED is being turned
  // on.  on the GD32 board variant it was causing problems and the digital
  // noise filtering was not enough.  ignore the FLAG status for a short time
  // when the LEd is being turned on.
  if (m_led_on_time.has_value ()
      && cur_time - m_led_on_time.value () < std::chrono::microseconds (500))
    return;

  if (cur_time - m_last_zxld_flag_sample_time >= zxld_flag_update_interval)
  {
    m_last_zxld_flag_sample_time = cur_time;

    const bool prev_flag = m_zxld_flag.value ();

    m_zxld_flag.push_back (zxld_status_flag_now ());

    const bool new_flag = m_zxld_flag.value ();

    if (prev_flag != new_flag && new_flag == false)
    {
      // make certain errors sticky
      status_t st = status ();

      if (st == overtemperature
	  || (!m_ignore_errors && (st == vaux_undervoltage || st == vin_undervoltage
	      || st == overcurrent || st == leda_overvoltage
	      || st == stall_out_of_regulation)))
        m_sticky_error = st;
    }
  }
#endif
}

void nld_board::led_on (void)
{
#ifndef BOARD_MCU_COMMS_ONLY

  if (is_error (m_sticky_error))
    return;

  m_led_on_time = std::chrono::high_resolution_clock::now ();
  zxld_pwm.set_on ();
#endif
}

void nld_board::led_off (void)
{
#ifndef BOARD_MCU_COMMS_ONLY
  zxld_pwm.set_off ();
#endif
}

void nld_board::led_ready (void)
{
#ifndef BOARD_MCU_COMMS_ONLY
  if (is_error (m_sticky_error))
    return;

  zxld_pwm.set_ready ();
#endif
}

void nld_board::reset_error (void)
{
#ifndef BOARD_MCU_COMMS_ONLY
  zxld_pwm.reset_break_condition ();
  m_sticky_error = normal;
  status ();	// update the status, which might potentially set the sticky
		// error again in case it still persists.
#endif
}

void nld_board::exec (std::chrono::high_resolution_clock::time_point cur_time)
{
#ifndef BOARD_MCU_COMMS_ONLY
  update_zxld_status_flag (cur_time);

  constexpr auto adc_update_interval = std::chrono::microseconds (100);

  if (cur_time - m_last_adc_update_time >= adc_update_interval)
  {
    m_last_adc_update_time = cur_time;
    update_adc_measurements ();
  }

  #ifndef APP_DISABLE_POWEROFF_HANDLING
  if (cur_time - m_last_vin_delta_update_time >= std::chrono::milliseconds (10))
  {
    m_last_vin_delta_update_time = cur_time;
    int new_vin = vin_mv ();

    m_vin_delta.push_back ((int16_t)utils::clamp ((new_vin - m_last_vin_delta_vin), -50000, 50000));
    m_vin_delta_iir.push_back (m_vin_delta.value ());
    m_last_vin_delta_vin = new_vin;
  }
  #endif

  // all sticky errors turn off the zxld
  if (is_error (m_sticky_error))
    zxld_pwm.set_off ();
#endif
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
  return nld_board::inst ().system_timer.current_time_ticks ();
}

} } }

// -----------------------------------------------------------------------------

void board_debug_uart_write (const void* data, unsigned int byte_count)
{
#if defined (BOARD_USE_DEBUG_USART)
  if (this_board::inst ().debug_console_enabled ())
    this_board::inst ().debug_usart.write (data, byte_count);
#endif
}


#if defined (BOARD_USE_DEBUG_USART)

[[gnu::cold]] static void
emergency_puts (const char* str, unsigned int max_count)
{
  if (!nld_board::inst ().debug_console_enabled ())
    return;

  auto& usart = nld_board::inst ().debug_usart;

  for (unsigned int i = 0; i < max_count; ++i)
  {
    char c = *str++;
    if (c == 0)
      break;

    // wat for any current character transmission to finish.
    while (!usart.transmit_data_empty ()) { }
    usart.write_transmit_data (c);
  }
}

#endif


#ifndef NDEBUG
[[noreturn, gnu::noinline, gnu::cold]]
void
int_assert (const char* source_filename, int linenum,
            const char* func_name, const char* expr)
{
  // disable interrupts.
  dev::this_cpu::save_disable_interrupts ();

  // when here all interrupts to the CPU are blocked.
  // the LEDs are driven by a timer, which will continue its operation.
  auto& board = nld_board::inst ();

#ifndef BOARD_MCU_COMMS_ONLY
  // disable ZXLD PWM output
  board.zxld_pwm.set_off ();
#endif

#if defined (BOARD_USE_DEBUG_USART)
  if (nld_board::inst ().debug_console_enabled ())
  {
    auto& usart = nld_board::inst ().debug_usart;

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
  }
#endif

#ifndef BOARD_MCU_COMMS_ONLY
  constexpr unsigned int led_count_mask =
	utils::ceil_pow2 (nld_board_clk::hclk_hz / 750);

  for (unsigned int i = 0; ; ++i)
  {
    board.red_led.write (i & led_count_mask ? true : false);
    board.white_led.write (i & led_count_mask ? true : false);
  };
#endif
}

#else
[[noreturn]]
void int_assert (const char*, int, const char*, const char*)
{
  while (true) { }
}
#endif


#if defined (BOARD_USE_DEBUG_USART)

extern "C" [[noreturn, gnu::cold, gnu::used]] void abort (void)
{
  int_assert (nullptr, 0, nullptr, "Abort");
}

[[noreturn]] void INT_NMI (void) { int_assert (nullptr, 0, nullptr, "NMI"); }
[[noreturn]] void INT_HardFault (void) { int_assert (nullptr, 0, nullptr, "Hard Fault"); }

#else

extern "C" [[noreturn, gnu::cold]] void int_assert_isr (void)
{
  int_assert (nullptr, 0, nullptr, "");
}

extern "C" [[gnu::used, gnu::alias ("int_assert_isr")]] void abort (void);

[[noreturn, gnu::alias ("int_assert_isr")]] void INT_NMI (void);
[[noreturn, gnu::alias ("int_assert_isr")]] void INT_HardFault (void);

#endif


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
void nld_board::reset_to_func (void (*func)(void))
{
  dev::this_cpu::save_disable_interrupts ();

  // depending on which devices have been used, we might need a proper
  // shutdown of the drivers to stop the interrupts etc.
  // otherwise the new program might malfunction because of some unexpected
  // hardware event.
  this->~nld_board ();

  func ();
  while (true) { }
}

[[noreturn]]
void nld_board::reset (void)
{
  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0xE000ED0C>> AIRCR = { };

  asm volatile ("dsb" : : : "memory");

  AIRCR = 0x05FA0000 | (1 << 2);

  asm volatile ("dsb" : : : "memory");
  while (true) { }
}

[[gnu::cold]] const nld_board_info& nld_board_info::inst (void)
{
  // for details, see board_info_data.cpp
  const nld_board_info* bi = *(const nld_board_info**)0x00000010;
  return *bi;
}

extern "C" unsigned int
__atomic_exchange_4 (volatile void* val, unsigned int new_val, int)
{
  auto i = dev::this_cpu::save_disable_interrupts ();

  auto prev_val = *(volatile unsigned int*)val;
  *(volatile unsigned int*)val = new_val;

  dev::this_cpu::restore_interrupts (i);
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
  isr_desc (-12, board_info_data),
#endif

  make_isr_desc<nld_board::system_timer_t::isr_t> (),

#ifndef BOARD_MCU_COMMS_ONLY
  make_isr_desc<nld_board::tim2_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim2_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim2_t::global_isr_t> (),

  make_isr_desc<nld_board::tim3_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim3_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim3_t::global_isr_t> (),

  make_isr_desc<nld_board::tim6_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim6_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim6_t::global_isr_t> (),

  make_isr_desc<nld_board::tim14_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim14_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim14_t::global_isr_t> (),

  make_isr_desc<nld_board::tim1_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim1_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim1_t::global_isr_t> (),

  make_isr_desc<nld_board::tim15_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim15_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim15_t::global_isr_t> (),

  make_isr_desc<nld_board::trigin_input_t::isr_t> (),
  make_isr_desc<nld_board::leden_input_t::isr_t> (),
  make_isr_desc<nld_board::zxldflag_input_t::isr_t> (),

#if 0
  // indicator LED PWM timer interrupts are not needed by software
  make_isr_desc<nld_board::tim16_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim16_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim16_t::global_isr_t> (),

  make_isr_desc<nld_board::tim17_t::break_update_trigger_com_isr_t> (),
  make_isr_desc<nld_board::tim17_t::capture_compare_isr_t> (),
  make_isr_desc<nld_board::tim17_t::global_isr_t> (),
#endif

  #if defined (BOARD_VARIANT_A02)
    make_isr_desc<nld_board::exti4_15_demux_t::isr_t> (),
  #elif defined (BOARD_VARIANT_A03) || defined (BOARD_VARIANT_A04)
    make_isr_desc<nld_board::exti2_3_demux_t::isr_t> (),
  #endif

#endif

#ifdef BOARD_USE_DEBUG_USART
  make_isr_desc<nld_board::debug_usart_t::isr_t> (),
#endif

  make_isr_desc<nld_board::rs485_usart_t::isr_t> ()


))))))));


