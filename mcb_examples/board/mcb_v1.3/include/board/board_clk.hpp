#ifndef includeguard_board_clk_hpp_includeguard
#define includeguard_board_clk_hpp_includeguard

class mcb_v13_board_clk
{
public:

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

  // RX main xtal frequency
  static constexpr unsigned int xtal_hz = 12'000'000;

  static constexpr unsigned int pll_hz = xtal_hz * 16;          // 192 mhz


  // RX internal clock
  static constexpr unsigned int iclk_hz = pll_hz / 1;       // = 96 mhz
  static constexpr unsigned int fclk_hz = pll_hz / 4;       // = 48 mhz


  // RX peripheral clock PCLKA for etherc, edmac, deu
  static constexpr unsigned int pclka_hz = pll_hz / 2;       // = 96 mhz

  // RX peripheral clock PCLKB for other than etherc, edmac, deu
  static constexpr unsigned int pclkb_hz = pll_hz / 4;        // = 48 mhz

  // RX external bus clock
  static constexpr unsigned int bclk_hz = pll_hz /  2;          // = 96 mhz

  // RX external bus clock pin output
  static constexpr unsigned int bclk_hz_output = bclk_hz / 2;   // = 48 mhz

  // free running timer frequency, used as the main system clock.
  static constexpr unsigned int system_clock_frequency_hz = pclkb_hz / 8; // = 6 mhz

  // the possible PCD and MCX clock rates depend on the PCLK / PCLKA which
  // drives the MTU.
  static constexpr unsigned int pcd4641_clock_counter_val = 3;
  static constexpr unsigned int pcd4641_clock_hz = 6'000'000;

  // if we use the xor clock doubler, we can also get 16 mhz
  static constexpr unsigned int mcx514_clock_counter_val = 1;
  static constexpr unsigned int mcx514_clock_hz = 12'000'000;
  // static constexpr unsigned int mcx514_clock_counter_val = 2;  // = 8 mhz base clock
  // static constexpr unsigned int mcx514_clock_hz = 16'000'000;  // doubled externally

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) || \
      defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)

  // use RX64's on-chip oscillator (HOCO) as the main system clock.

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

//  static constexpr unsigned int pll_hz = 240'000'000;
  static constexpr unsigned int pll_hz = 192'000'000;

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

  // use CMT at 1/8 frequency as free running system timer.
  static constexpr unsigned int system_clock_frequency_hz = pclkb_hz / 8; // = 7.5 mhz

  static constexpr unsigned int mtu_clock_hz = pclka_hz;

  // 120 mhz mtu clock / 8 = 15 mhz pcd clock
  //  96 mhz mtu clock / 8 = 12 mhz pcd clock
  // the PCD clock has an impact on the number of bus wait cycles
  // that need to be inserted after a command write.
  // although overclocking the PCD to e.g. 12 MHz seems to work OK,
  // there are subtle issues regarding its IO readings and other things.
  // it does run and output pulse signals, but some other things seem to
  // go out the window when clock speed is higher than the specified 10 MHz.
  // and actually it seems it already starts happening at 10 MHz so run it
  // at a lower frequency.

  // 120 mhz mtu clock / 16 = 7.5 mhz pcd clock
  //  96 mhz mtu clock / 16 =   6 mhz pcd clock
  static constexpr unsigned int pcd4641_clock_hz = mtu_clock_hz / 16;
  static_assert (pcd4641_clock_hz * 16 == mtu_clock_hz, "");

  // 120 mhz mtu clock / 6 = 20 mhz mcx clock
  //  96 mhz mtu clock / 6 = 16 mhz mcx clock
  static constexpr unsigned int mcx514_clock_hz = mtu_clock_hz / 6;
  static_assert (mcx514_clock_hz * 6 == mtu_clock_hz, "");

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

  // output frequency = 1 / ((counter + 1)*2 / timer_clk)
  // ( timer_clk / (2 * output frequency) ) - 1 = counter
  // 96 MHz timer_clk, 20 MHz output freq. counter = 1.4 => 1
  // 120 MHz timer_clk, 20 MHz output freq. counter = 2 => 2

  // convert frequency [hz] to number of MTU counter ticks
  static constexpr inline unsigned int hz_to_mtu_tick_count (unsigned int f)
  {
    return ( mtu_clock_hz / (2 * f) ) - 1;
  }

  // convert MTU counter ticks to frequency [hz]
  static constexpr inline unsigned int mtu_tick_count_to_hz (unsigned int c)
  {
    return mtu_clock_hz / ((c + 1)*2);
  }

};

namespace this_board
{
  using clk_type = mcb_v13_board_clk;
}

#endif // includeguard_board_clk_hpp_includeguard
