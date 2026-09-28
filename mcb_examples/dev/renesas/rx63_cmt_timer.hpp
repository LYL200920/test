/*
   system timer implementation using the RX63 CMT device.

   use CMT0 as free-running timer
   use CMT1 for software timers (not implemented)

   FIXME: this has a hardcoded use of CMT0 and CMT1.  make it configurable.
	  CMT0,CMT1 is one CMT unit and CMT2,CMT3 is another hw instance of
	  the same device.
*/

#ifndef includeguard_rx63_cmt_timer_hpp_includeguard
#define includeguard_rx63_cmt_timer_hpp_includeguard

#include <cstdint>

#include <dev/hwreg.hpp>
#include <dev/renesas/rx63_interrupt.hpp>

namespace dev
{

template <unsigned int PeripheralClockHz, unsigned int TimerClockHz>
class rx63_cmt_timer
{
  void cmt0_isr (void)
  {
    // leave the lower bits [15:0] always at zero.  they will be
    // OR'ed in when reading the composed timer counter.
    m_counter_highpart += 0x10000;
  }

  void cmt1_isr (void)
  {
  }

public:
  typedef interrupt::connected_isr<rx63_interrupt::line<rx63_interrupt::cmt0_cmi>,
	interrupt::func<decltype (&rx63_cmt_timer::cmt0_isr), &rx63_cmt_timer::cmt0_isr>> isr0_t;

  typedef interrupt::connected_isr<rx63_interrupt::line<rx63_interrupt::cmt1_cmi>,
	interrupt::func<decltype (&rx63_cmt_timer::cmt1_isr), &rx63_cmt_timer::cmt1_isr>> isr1_t;

  rx63_cmt_timer (void) : m_counter_highpart (0), m_isr0 (this), m_isr1 (this)
  {
    // release CMT unit 0 module stop (CMT0, CMT1).
    MSTPCRA &= ~(1 << 15);

    // stop the counters.
    CMT0_CMT1_CMSTR = 0;

    // CMT0, CMT1 clock = PCLK/8, CMIE enable
    // for the free running timer to work correctly, the system clock frequency
    // must match the possible timer clock rates.
    static_assert (  TimerClockHz == PeripheralClockHz / 4
		   || TimerClockHz == PeripheralClockHz / 8
		   || TimerClockHz == PeripheralClockHz / 32
		   || TimerClockHz == PeripheralClockHz / 128
		   || TimerClockHz == PeripheralClockHz / 512, "Impossible TimerClockHz");

    unsigned int clocksel =   TimerClockHz == (PeripheralClockHz /   4) ? 0b00
			    : TimerClockHz == (PeripheralClockHz /   8) ? 0b00
			    : TimerClockHz == (PeripheralClockHz /  32) ? 0b01
			    : TimerClockHz == (PeripheralClockHz / 128) ? 0b10
			    : TimerClockHz == (PeripheralClockHz / 512) ? 0b11
			    : 0;

    CMT0_CMCR = clocksel | (1 << 6);
    CMT1_CMCR = clocksel | (1 << 6);

    CMT0_CMCNT = 0;
    CMT1_CMCNT = 0;

    // for the free running CMT0 we're interested in
    // the overflow ISR.  set the match constant to max. value.
    // the CMT1 match constant will be set dynamically when
    // software timers are started.
    CMT0_CMCOR = 0xFFFF;

    m_isr0.enable (interrupt::rising_edge, interrupt::priority_6);
    m_isr1.enable (interrupt::rising_edge, interrupt::priority_1);

    // start CMT0.
    // leave CMT1 stopped.  it will be enabled when needed.
    CMT0_CMT1_CMSTR = 1;
  }

  ~rx63_cmt_timer (void)
  {
    // stop counters and set module stop.
    CMT0_CMT1_CMSTR = 0;
    MSTPCRA |= 1 << 15;
  }

  uint64_t current_time_ticks (void)
  {
    // first read the read low part hardware counter and high
    // part software counter.  if there was a CMT0 interrupt while
    // we were reading the counters, the high part counter value
    // might be wrong, because it was incremented while we were
    // reading it.  to avoid returning wrong values we keep reading
    // the counters until the high part is stable.
    // this is effectively a spinlock.

    uint16_t low_part;
    uint64_t high_part;
    do
    {
      low_part = CMT0_CMCNT;
      high_part = m_counter_highpart;
    } while (high_part != m_counter_highpart);

    return (high_part | low_part) << ((TimerClockHz == PeripheralClockHz / 4) ? 1 : 0);
  }


private:
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00080010>> MSTPCRA = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x00088000>> CMT0_CMT1_CMSTR = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x00088002>> CMT0_CMCR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x00088004>> CMT0_CMCNT = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x00088006>> CMT0_CMCOR = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x00088008>> CMT1_CMCR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x0008800A>> CMT1_CMCNT = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x0008800C>> CMT1_CMCOR = { };

  // free running timer counter bits [15:0] are read directly
  // from the hardware counter.  bits [63:16] are counted using
  // the timer overflow ISR and stored in a global variable. 
  volatile uint64_t m_counter_highpart;
  isr0_t m_isr0;
  isr1_t m_isr1;

};

} // namespace dev
#endif // includeguard_rx63_cmt_timer_hpp_includeguard
