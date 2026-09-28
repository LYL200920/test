#ifndef includeguard_mcbv2_board_clk_hpp_includeguard
#define includeguard_mcbv2_board_clk_hpp_includeguard

#include <ratio>

class mcb_v2_board_clk
{
public:

  // sub-clock oscillator frequency is the same for all board variants,
  // if it is present.  this is just a reference value.  the actual value
  // is determined during runtime.
  static constexpr unsigned int xcin_hz = 32'768;

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

  // RX LOCO frequency (approx)
  static constexpr unsigned int loco_hz = 125'000;

  // RX main xtal frequency
  static constexpr unsigned int xtal_hz = 12'000'000;

  static constexpr unsigned int pll_hz = xtal_hz * 16;          // 192 mhz


  // RX internal clock
  static constexpr unsigned int iclk_hz = pll_hz / 2;       // = 96 mhz
  static constexpr unsigned int fclk_hz = pll_hz / 4;       // = 48 mhz


  // RX peripheral clock PCLKA for etherc, edmac, deu
  static constexpr unsigned int pclka_hz = pll_hz / 2;       // = 96 mhz

  // RX peripheral clock PCLKB for other than etherc, edmac, deu
  static constexpr unsigned int pclkb_hz = pll_hz / 4;        // = 48 mhz

  // RX external bus clock
  static constexpr unsigned int bclk_hz = pll_hz /  2;          // = 96 mhz

  // RX external bus clock pin output
  static constexpr unsigned int bclk_hz_output = bclk_hz / 2;   // = 48 mhz

  // USB clock frequency
  static constexpr unsigned int uck_hz = 48'000'000;

  // IEBUS clock frequency
  static constexpr unsigned int iebck_hz = pll_hz / 4;  // = 48 mhz

  // free running timer frequency, used as the main system clock.
  static constexpr unsigned int system_clock_frequency_hz = pclkb_hz / 8; // = 6 mhz

  static constexpr unsigned int mtu_clock_hz = pclkb_hz;

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) || \
      defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)

  // RX LOCO frequency (approx)
  static constexpr unsigned int loco_hz = 240'000;

  // when using USBA (176 pin MCU) with internal clock supply,
  // UCLK must be 48 mhz and PCLKB must be 60 mhz.
  // on RX71M, this does not support high-speed, only full-speed operation.

  // when using USBA with external clock supply,
  // USBMCLK must be 20 or 24 mhz.  for that, the external crystal/osc
  // must be 20 or 24 mhz.  on RX71M this supports high-speed operation.

  // RX71M does the same setup, but the iclk_hz (cpu clock) can be 240 instead
  // of 120 mhz.  if doing so, MEMWAIT has to be set to insert a wait-cycle
  // on all memory accesses.

  static constexpr unsigned int hoco_hz = 16'000'000;
  static constexpr unsigned int xtal_hz = 24'000'000;

  static constexpr unsigned int pll_hz = 240'000'000;
//  static constexpr unsigned int pll_hz = 192'000'000;

  static constexpr unsigned int fclk_hz = pll_hz / 4;

#if defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)
  static constexpr unsigned int iclk_hz = pll_hz;
#else
  static constexpr unsigned int iclk_hz = pll_hz / 2;
#endif

  static constexpr unsigned int pclka_hz = pll_hz / 2;        // = 120 mhz
  static constexpr unsigned int pclkb_hz = pll_hz / 4;        // = 60 mhz
  static constexpr unsigned int pclkc_hz = pll_hz / 4;        // = 60 mhz
  static constexpr unsigned int pclkd_hz = pll_hz / 4;        // = 60 mhz
  static constexpr unsigned int bclk_hz = pll_hz / 2;         // = 120 mhz
  static constexpr unsigned int bclk_hz_output = bclk_hz / 2; // = 60 mhz
  static constexpr unsigned int uck_hz = 48'000'000;
  static constexpr unsigned int usbmclk_hz = xtal_hz;         // = 24 mhz

  // use CMT at 1/8 frequency as free running system timer, which gives 7.5 mhz.
  // to get rid of the .5 use 1/4 as the clock frequency.  it will still use
  // CMT 1/8 as the timer frequency, but report a clock multiplied by 2.
  static constexpr unsigned int system_clock_frequency_hz = pclkb_hz / 4; // = 7.5 x 2 = 15 mhz

  static constexpr unsigned int mtu_clock_hz = pclka_hz;

#else
  #error board variant not selected
#endif

  // convert nanoseconds to a number of bus clock cycles
  static constexpr inline unsigned int ns_to_bclk (unsigned int ns)
  {
    // 1 bus clock in ns = 1000000000 / 96000000 = 10.4166

    // 20 ns = 20 / (1000000000 / 96000000) = 1.92

    // 200 ns = 200 / (1000000000 / 96000000) = 19.2
    //          (200 * 96000000) / 1000000000 = 19.2
    //          (200 * 96000000 + 1000000000 - 1) / 1000000000 = 20.1999

    constexpr uint64_t hz_to_ns = 1000000000LL;

    uint64_t r = ((uint64_t)ns * (uint64_t)bclk_hz + hz_to_ns - 1) / hz_to_ns;

    return (unsigned int)r;
  }


  // convert frequency [hz] to number of MTU counter ticks

  // output frequency = 1 / ((counter + 1)*2 / timer_clk)
  // ( timer_clk / (2 * output frequency) ) - 1 = counter
  // 96 MHz timer_clk, 20 MHz output freq. counter = 1.4 => 1
  // 120 MHz timer_clk, 20 MHz output freq. counter = 2 => 2

  static constexpr inline unsigned int hz_to_mtu_tick_count_floor (unsigned int f)
  {
    return ( mtu_clock_hz / (2 * f) ) - 1;
  }

  template <typename RatioHz>
  static constexpr inline unsigned int hz_to_mtu_tick_count_floor (void)
  {
    using r = std::ratio_divide <
		std::ratio < mtu_clock_hz, 1 >,
		std::ratio_multiply < std::ratio<2, 1>, RatioHz > >;

    return (r::num / r::den) - 1;
  }

  template <typename RatioHz>
  static constexpr inline unsigned int hz_to_mtu_tick_count_ceil (void)
  {
    using r = std::ratio_divide <
		std::ratio < mtu_clock_hz, 1 >,
		std::ratio_multiply < std::ratio<2, 1>, RatioHz > >;

    return ((r::num + r::den - 1) / r::den) - 1;
  }


  // convert MTU counter ticks to frequency [hz]
  static constexpr inline unsigned int mtu_tick_count_to_hz (unsigned int c)
  {
    return mtu_clock_hz / ((c + 1)*2);
  }

  template <unsigned int MtuTickCount>
  using mtu_tick_count_to_hz_ratio = std::ratio < mtu_clock_hz, (MtuTickCount + 1) * 2>;


};

namespace this_board
{
  using clk_type = mcb_v2_board_clk;
}

#endif // includeguard_mcbv2_board_clk_hpp_includeguard
