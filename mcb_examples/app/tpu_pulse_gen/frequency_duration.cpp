
#include <cstdio>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <utility>

using timer_duration = std::chrono::duration <int32_t, std::ratio<1, 60'000'000>>;
using timer_duration_long = std::chrono::duration <uint64_t, timer_duration::period>;

std::pair<unsigned int, timer_duration> freq_to_ticks (float f_hz)
{
  static constexpr auto max_timer_duration = timer_duration (65535);

  const auto d = timer_duration_long ((uint64_t)timer_duration_long::period::den * 100 / (uint32_t)(f_hz * 100));

  const unsigned int n = d / max_timer_duration;

  return { n, d - (n * max_timer_duration) };
}

/*
    60'000'000'000 / (f * 100)

    1 hz = 1s    =   1s / 1/60'000'000 = 60'000'000
    2 hz = 0.5s  = (1/2) / (1/60'000'000) = 30'000'000
                 = 1/2 * 60'000'000
                 = 60'000'000 / 2

    0.1 hz = 10/1 / 1/60'000'000 = 10/1 * 60'000'000 = 600000000
    600000000 / 65536 = 9155

    400 hz = 1/400 / 1/60'000'000
           = 60'000'000 / 400 = 150000

    60000 hz = 1/60000 / 1/60'000'000 = 60'000'000 / 60000 = 1000
*/

void test_hz (float val)
{
  const auto x = freq_to_ticks (val);
  std::cout << val << " hz: " << x.first << ", " << x.second.count () << std::endl;
}


int main (void)
{
  static const float values[] =
  {
    1.0f, 2.0f, 3.0f, 1.1f, 0.1f, 400.0f, 60000.0f, 1'200'000.0f, 3.7f
  };

  for (auto&& v : values)
    test_hz (v);

  return 0;
}
