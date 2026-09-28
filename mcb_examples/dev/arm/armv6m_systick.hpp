/*
   system timer implementation using the ARMv6-M SysTick device

   the SysTick is a 24 bit decrementing wrap-around/reload counter.

   for now this supports only processor clock frequency.

*/

#ifndef includeguard_armv6m_systick_hpp_includeguard
#define includeguard_armv6m_systick_hpp_includeguard

#include <cstdint>

#include <dev/hwreg.hpp>
#include <dev/interrupt.hpp>

namespace dev
{

template <unsigned int ProcessorClockHz,
	  typename InterruptLine, unsigned int InterruptPriority>
class armv6m_systick
{
private:
  static constexpr uintptr_t reg_base_addr = 0xE000E010;

  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_addr + 0x00>> syst_csr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_addr + 0x04>> syst_rvr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_addr + 0x08>> syst_cvr = { };

  // contains number of ticks per 10 ms of the other clock source than the
  // processor clock is used. (not used here)
  static constexpr hw_reg_r <uint32_t, const_addr<reg_base_addr + 0x0C>> syst_calib = { };


  void isr (void)
  {
    m_counter_highpart += 1;
  }


public:
  typedef interrupt::connected_isr<InterruptLine,
	interrupt::func<decltype (&armv6m_systick::isr), &armv6m_systick::isr>> isr_t;

  armv6m_systick (void) : m_counter_highpart (0), m_isr (this)
  {
    // disable timer, in case it has been enabled before by something.
    syst_csr = 0;

    m_isr.enable (dev::interrupt::falling_edge, InterruptPriority);

    // reset counters to 0x00FFFFFF (max value)
    // the timer interrupt will be triggered when it counts down to 0.
    syst_rvr = 0x00FFFFFF;
    syst_cvr = 0x00000000;

    // enable timer and interrupt
    // CLKSOURCE = 1 (processor clock)
    // TICKINT   = 1 (enable interrupt)
    // ENABLE    = 1 (enable timer)
    syst_csr = 0b111;
  }

  ~armv6m_systick (void)
  {
    // stop timer
    syst_csr = 0;
  }

  uint64_t current_time_ticks (void)
  {
    // first read the low part hardware counter and high
    // part software counter.  then read the low part hardware counter again.
    // when there is no wrap-around the new low part counter is less than
    // the old value.  if that's not the case, then there was a wrap-around
    // while we were reading and should try again.
    // even if the new low part counter is less than the old one, it still
    // could be that this function has been pre-empted for very long and
    // it has missed a potential wrap-around and modification of the high part.
    // in those cases, keep trying to read the counters.
    // effectively this is a spinlock.

    uint32_t low_part0;
    uint32_t low_part1;
    uint64_t high_part0;
    uint64_t high_part1;
    do
    {
      low_part0 = syst_cvr;
      high_part0 = m_counter_highpart;
      low_part1 = syst_cvr;
      high_part1 = m_counter_highpart;

    } while (low_part1 > low_part0 || high_part0 != high_part1);

    // the hardware is a down-counter, need to convert it to an up-counter.
    // 0x00FFFFFF -> 0x00000000
    //            ...
    // 0x00000000 -> 0x00FFFFFF
    return (high_part1 << 24) | (0x00FFFFFF - low_part1);
  }

private:
  // free running timer counter bits [23:0] are read directly
  // from the hardware counter.  bits [63:24] are counted using
  // the timer underflow ISR and stored in a global variable.
  volatile uint64_t m_counter_highpart;
  isr_t m_isr;
};

} // namespace dev
#endif // includeguard_armv6m_systick_hpp_includeguard
