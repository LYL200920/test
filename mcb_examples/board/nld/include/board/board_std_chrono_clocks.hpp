
// this file is included by the overriding <chrono> include in the
// toolchain directory.

#ifndef expand_std_chrono_clocks

#include <board/board_clk.hpp>

#else

#ifdef have_std_chrono_system_clock
  #undef have_std_chrono_system_clock
#endif
#define have_std_chrono_system_clock 1

struct system_clock
{
  // the duration is the number of seconds per tick, i.e. the duration of
  // one tick measured in seconds.
  // e.g. if pclock_hz = 6,000,000 (6 MHz), then
  // 6,000,000 ticks = 1 second
  //                 = 1,000 milliseconds
  //                 = 1,000,000 microseconds
  //         6 ticks = 1 microsecond
  //         1 tick  = 1/6 microseconds
  //                 = 1/6,000,000 seconds

  static constexpr unsigned int tick_frequency = this_board::clk_type::system_clock_frequency_hz;

  typedef std::chrono::duration<int64_t, std::ratio<1, tick_frequency>> duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<system_clock, duration> time_point;

  static constexpr bool is_steady = true;

  // FIXME (MCB-122): actually it's supposed to be noexcept, but since rx-elf does not
  // support dwarf exception handling, but only SJLJ, this always results in
  // extra code.
  static uint64_t current_time_ticks (void);

  static time_point now (void) // noexcept
  {
    return time_point (duration (current_time_ticks ()));
  }

  static std::time_t
  to_time_t (const time_point& val) // noexcept
  {
    return std::time_t (duration_cast<chrono::seconds> (val.time_since_epoch()).count());
  }

  static time_point
  from_time_t (std::time_t val) // noexcept
  {
    typedef chrono::time_point<system_clock, seconds> from_t;
    return time_point_cast<system_clock::duration> (from_t(chrono::seconds(val)));
  }
};

#ifdef have_std_chrono_steady_clock
  #undef have_std_chrono_steady_clock
#endif
#define have_std_chrono_steady_clock 1

using steady_clock = system_clock;

#ifdef have_std_chrono_high_resolution_clock
  #undef have_std_chrono_high_resolution_clock
#endif
#define have_std_chrono_high_resolution_clock 1

using high_resolution_clock = system_clock;

#endif // expand_std_chrono_clocks
