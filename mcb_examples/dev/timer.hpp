/*

a timer is a counter that runs at some frequency and has one or more trigger
points.  these trigger points are often implemented in hardware as
compare-match functions.

the timer can be implemented in software or be actually one hardware device.

at the trigger points, a callback function is invoked
(potentially in an ISR context), this can then do further processng.

in addition to that, it can also be used to trigger other hardware devices in
a system specific way, e.g. by passing the timer instance to some other device.

// stop and clear all current trigger points
timer.stop ();
timer.clear_trigger_points ();

// set the timer period in hertz or in duration
timer.set_period (1MHz);
timer.set_period (std::chrono::nanoseconds (1000));
timer.set_period (std::chrono::microseconds (1));

// add a trigger point.  in this case to the same value
// as the period.  for hardware timers, this will utilize the counter overflow
// interrupt, if possible.
timer.add_trigger_point (timer::periodic, std::chrono::microseconds (1),
			 [] () { some code here });

// add another trigger point, which should trigger another hardware device.
// this will not involve the CPU.
// the period is 1000ns as set above.  the trigger point is 100ns from the
// start of the period.

timer.add_trigger_point (timer::periodic, std::chrono::nanoseconds (100),
			 timer::trigger_link (...))

// set the timer base frequency in hertz or in duration
//timer.set_base_frequency (1MHz);

the trigger points can be single-shot or repeating (periodic).
for hardware timers, the period is defined by the timer running frequency.
other periods still can be implemented with some additional software handling,
but if the timer is used to trigger some other hardware automatically, the
timer's base frequency needs to be adjusted.
software timers mimick this behavior, but they don't have a defined running
frequency.  instead that value is just used to make certain time calculations.

*/


// CAUTION
//  this class is not finished yet.  it's not used by anything yet.

// timer period / auto-reload value
// RX GPT, STM32 timers: user defined value, reset by this special trigger point
// RX TMU: fixed at 0, can be reset by compare-match trigger point
// RX MTU: fixed at 0, can be reset by compare-match trigger point
// RX CMT: fixed at 0, always reset by compare-match trigger point
// RX CMTW: fixed at 0, can be reset by compare-match trigger point
// RX TMR: fixed at 0, can be reset by compare-match trigger point

// has_counter_clear_by_period
// has_counter_clear_by_trigger_a
// has_counter_clear_by_trigger_b

#ifndef includeguard_dev_timer_includeguard
#define includeguard_dev_timer_includeguard

#include <cstdint>
#include <functional>
#include <chrono>
#include <dev/interrupt.hpp>
#include <utils/langcomp.hpp>

namespace dev
{
namespace timer
{

typedef std::function<void (void)> trigger_callback_func_t;

enum trigger_disable_tag { disable };
enum trigger_enable_tag { enable };

void empty_func (void);

struct no_user_trigger_callback
{
  template <typename T> auto& operator = (T&&) { return *this; }
  explicit operator bool (void) { return true; }
  void operator () (void) { }
};

struct nonnull_trigger_callback : trigger_callback_func_t
{
  explicit operator bool (void) const { return true; }

  template <typename T> nonnull_trigger_callback& operator = (T&& other)
  {
    timer::trigger_callback_func_t::operator = (std::forward<T> (other));
    return *this;
  }

  void operator () (void)
  {
    // this does not completely eliminate the nullptr check, but is a bit
    // better than without.
    if (*this == nullptr)
      assert_unreachable ();

    timer::trigger_callback_func_t::operator () ();
  }
};

template <typename Func>
struct fixed_trigger_callback
{
  explicit operator bool (void) const { return true; }

  template <typename T>
  fixed_trigger_callback& operator = (T&&) { return *this; }

  void operator () (void)
  {
    Func ()();
  }
};



#if 0
class timer
{
public:
  // using high_resolution_clock for all timers in the system ...
  // that would work fine if all timers in the system are derived from the same
  // base frequency.  it will not work if the system has a timers
  // for e.g. 25 Mhz and 24 MHz, which might be the case.
  // to support that, the BSP would need to use a common ratio of 1/25 and 1/24
  // which is 1/534.
  typedef std::chrono::high_resolution_clock::duration duration;
  typedef std::chrono::high_resolution_clock::rep rep;
  typedef std::chrono::high_resolution_clock::period period;
  typedef std::chrono::time_point<timer, duration> time_point;


  // start the timer
  virtual void start (void) = 0;

  // stop the timer
  virtual void stop (void) = 0;

  // BLEH: timer resolution ... duration type is usually a template arg.
  // idea: put it into the BSP's board_clk and refer to it as
  //   this_board::clk_type::timer::max_clock
  //   as a common highest possible clock
  //
  // or tie the timer time scale to std::high_resolution_clock.
  // if the free running system timer uses a lower resolution.

  // if this is a hardware timer and the priod exceeds the hardware
  // counter limit, it could reduce the clock base frequency which would
  // also reduce the clock resolution.
  // another way is to insert intermediate trigger points and only
  // invoke the user callback when the time has actually expired.
  // however, that would only work for CPU-callback trigger points.  it will
  // not work for trigger points to trigger other hardware (event link, DTC/DMA
  // trigger).
  virtual void set_frequency (duration d) = 0;

  virtual void set_period (duration d) = 0;


  // adds a trigger point
  enum trigger_point_type_t
  {
    periodic,
    one_shot
  };

//  void add_trigger_point (trigger_point_type_t t, std::chrono::duration ...


protected:

};
#endif

} // namespace timer
} // namespace dev
#endif // includeguard_dev_timer_includeguard
