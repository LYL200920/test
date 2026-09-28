#ifndef includeguard_dzo_gd32_proj_board_clk_hpp_includeguard
#define includeguard_dzo_gd32_proj_board_clk_hpp_includeguard

class dzo_gd32_proj_board_clk
{
public:
  // internal high-speed oscillator
  static constexpr unsigned int hsi_hz = 8'000'000;

  // internal low-speed oscillator
  static constexpr unsigned int lsi_hz = 40'000;

  // internal ADC high-speed oscillator
  static constexpr unsigned int hsi40_hz = 40'000'000;

  // external crystal oscillator
  static constexpr unsigned int hxtal = 8'000'000;

  static constexpr unsigned int sysclk_hz = 120'000'000;

  static constexpr unsigned int hclk_hz   = 120'000'000;
  static constexpr unsigned int pclk2_hz  = 120'000'000;
  static constexpr unsigned int pclk1_hz  =  60'000'000;

  // alias for system clock
  static constexpr unsigned int system_clock_frequency_hz = sysclk_hz;
};

namespace this_board
{
  using clk_type = dzo_gd32_proj_board_clk;
}

#endif // includeguard_dzo_gd32_proj_board_clk_hpp_includeguard
