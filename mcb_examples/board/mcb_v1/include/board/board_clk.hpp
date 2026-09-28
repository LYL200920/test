#ifndef includeguard_board_clk_hpp_includeguard
#define includeguard_board_clk_hpp_includeguard

class mcb_v1_board_clk
{
public:
  // RX main xtal frequency
  static constexpr unsigned int xtal_hz = 12000000;

  // RX internal clock
  static constexpr unsigned int iclock_hz = xtal_hz * 8;
  static constexpr unsigned int iclk_hz = iclock_hz;

  // RX peripheral clock  
  static constexpr unsigned int pclock_hz = xtal_hz * 4;

  // RX external bus clock
  static constexpr unsigned int bclk_hz = xtal_hz * 8;

  // RX external bus clock pin output
  static constexpr unsigned int bclk_hz_output = bclk_hz / 2;

  // free running timer frequency, used as the main system clock.
  static constexpr unsigned int system_clock_frequency_hz = pclock_hz / 8;
};

namespace this_board
{
  using clk_type = mcb_v1_board_clk;
}

#endif // includeguard_board_clk_hpp_includeguard
