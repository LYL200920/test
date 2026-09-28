
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
  led_out.write (0b1111'1111);

#if 1 //test set_brightness && write
  int8_t step = 1;
  uint8_t led_green_on = 0;
  uint8_t led_red_on = 255;

  auto last_led_update_time = std::chrono::high_resolution_clock::now ();

  static constexpr uint8_t led_lut[] =
	{ 0b00110011, 0b01100110, 0b11001100, 0b01100110 };

  unsigned int led_i = 0;

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
  	led_green_on += step;
  	led_red_on += -step;

    std::printf ("led_green_on = %u, led_red_on = %u, step = %d\n", led_green_on, led_red_on ,step);

  	led_out.set_brightness(led_green_on,led_red_on);

  	if(led_green_on == 0 || led_green_on == 255)
	    step = -step;

  	std::this_thread::sleep_for(20ms);

    auto cur_time = std::chrono::high_resolution_clock::now ();
    if (cur_time - last_led_update_time > 1000ms)
    {
      ++led_i;

      if (led_i >= std::extent<decltype (led_lut)>::value)
        led_i = 0;

      led_out.write (led_lut[led_i]);
      last_led_update_time = cur_time;
    }

  }
#endif

#if 0 //test set_brightness
  int8_t step = 1;
  uint8_t led_green_on = 0;
  uint8_t led_red_on = 255;

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
  	led_green_on += step;
  	led_red_on += -step;

    std::printf ("led_green_on = %u, led_red_on = %u, step = %d\n", led_green_on, led_red_on ,step);

  	led_out.set_brightness(led_green_on,led_red_on);

  	if(led_green_on == 0 || led_green_on == 255)
	    step = -step;

  	std::this_thread::sleep_for(8ms);

  }
#endif

#if 0 //test write
  auto last_led_update_time = std::chrono::high_resolution_clock::now ();

  static constexpr uint8_t led_lut[] =
	{ 0b00010001, 0b00100010, 0b01000100, 0b10001000, 0b01000100, 0b00100010 };

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
#endif
  return 0;
}
