
#include <cstdio>
#include <chrono>

#define log_all
#include <logging/logging.hpp>

#include <board/board.hpp>
#include <dev/rx_gpio.hpp>


using namespace std::chrono_literals;


int main (void)
{
  auto&& board = this_board::inst ();
  #ifdef MCB_USE_LEDS
  auto&& led_out = board.led_outputs;
  #endif

  auto last_led_update_time = std::chrono::high_resolution_clock::now ();

  static constexpr uint8_t led_lut[] =
	{ 0b0001, 0b0010, 0b0100, 0b1000, 0b0100, 0b0010 };

  unsigned int led_i = 0;

  // set peripheral reset
  board.set_peripheral_reset (true);

  // switch SCI2_TXD (P13 port) direction to output
  dev::rx_gpio::pdr::p1 |= 0b0000'1000;

  // set SCI2_TXD (P13 port) output data
  dev::rx_gpio::podr::p1 &= ~0b0000'1000;

  // switch SCI2_TXD (P13 port) to GPIO output mode (if it was in peripheral mode)
  dev::rx_gpio::pmr::p1 &= ~0b0000'1000;

  // release peripheral reset
  board.set_peripheral_reset (false);


  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    auto cur_time = std::chrono::high_resolution_clock::now ();
    if (cur_time - last_led_update_time > 500ms)
    {
      ++led_i;

      if (led_i >= std::extent<decltype (led_lut)>::value)
        led_i = 0;

      #ifdef MCB_USE_LEDS
      led_out.write (led_lut[led_i]);
      #endif

      last_led_update_time = cur_time;
    }
  }

  return 0;
}
