#ifndef includeguard_nld_board_clk_hpp_includeguard
#define includeguard_nld_board_clk_hpp_includeguard

class nld_board_clk
{
public:
  // internal high-speed oscillator
  static constexpr unsigned int hsi_hz = 8'000'000;

  // internal low-speed oscillator
  static constexpr unsigned int lsi_hz = 40'000;

  // internal ADC high-speed oscillator
  static constexpr unsigned int hsi14_hz = 14'000'000;

  // external crystal frequency (4-32 Mhz, there is no crystal on this board)
  // static constexpr unsigned int hse_hz = 8'000'000;

  static constexpr unsigned int sysclk_hz = 48'000'000;
  static constexpr unsigned int hclk_hz = 48'000'000;
  static constexpr unsigned int pclk_hz = 48'000'000;


  static constexpr unsigned int system_clock_frequency_hz = hclk_hz;
};

namespace this_board
{
  using clk_type = nld_board_clk;
}

#endif // includeguard_nld_board_clk_hpp_includeguard
