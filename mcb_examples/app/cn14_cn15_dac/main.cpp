
#include <cstdio>
#include <chrono>

#define log_all
#include <logging/logging.hpp>

#include <board/board.hpp>

using namespace std::chrono_literals;


int main (void)
{
  auto&& board = this_board::inst ();
  auto&& led_out = board.led_outputs;

  auto last_led_update_time = std::chrono::high_resolution_clock::now ();

  static constexpr uint8_t led_lut[] =
	{ 0b0001, 0b0010, 0b0100, 0b1000, 0b0100, 0b0010 };

  unsigned int led_i = 0;

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    auto cur_time = std::chrono::high_resolution_clock::now ();
    if (cur_time - last_led_update_time > 500ms)
    {
      ++led_i;

      if (led_i >= std::extent<decltype (led_lut)>::value)
        led_i = 0;

      led_out.write (led_lut[led_i]);
      last_led_update_time = cur_time;
    }
  }

  return 0;
}
