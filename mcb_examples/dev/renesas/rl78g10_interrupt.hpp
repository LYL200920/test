
#ifndef includeguard_dev_rl78g10_interrupt_hpp_includeguard
#define includeguard_dev_rl78g10_interrupt_hpp_includeguard

#include <cstdint>
#include <dev/interrupt.hpp>

namespace dev
{
namespace rl78g10_interrupt
{

enum isr_num_t
{
  // these numbers reflect the vector table offsets in 16 bit units.

  // reset
  reset_spor_wdt_trap = 0x0000/2,	// reset pin input, selectable power-on reset,
					// watchdog timer overflow, illegal instruction

  // reserved_0x0002 (debug monitor area)

  // maskable
  intwdti = 0x0004/2,	// internal - watchdog timer interval

  intp0 = 0x0006/2,	// external - pin input edge detection
  intp1 = 0x0008/2,	// external - pin input edge detection

  intst0 = 0x000A/2,	// internal - UART0 Tx end or Tx buffer empty
  intsr0 = 0x000C/2,	// internal - UART0 Rx transfer end
  intsre0 = 0x000E/2,	// internal - UART0 Rx communication error

  inttm01h = 0x0010/2,	// internal - end of counting or start of operations by
			// timer channel 1 (at higher 8-bit timer operation)

  inttm00 = 0x0012/2,	// internal - end of counting, completion of capture, or
			// start of operations by timer channel 0

  inttm01 = 0x0014/2,	// internal - end of counting, completion of capture, or
			// start of operations by timer channel 1 (at 16-bit or
			// lower 8-bit timer operation)

  intad = 0x0016/2,	// internal - end of A/D conversion

  intkr = 0x0018/2,	// external - key return signal detection

  intp2 = 0x001A/2,	// external - pin input edge detection (16 pin only)
  intp3 = 0x001C/2,	// external - pin input edge detection (16 pin only)

  inttm03h = 0x001E/2,	// internal - End of counting or start of operations by
			// timer channel 3 (at higher 8-bit timer operation)
			// (16 pin only)

  intiica0 = 0x0020/2,	// internal - end of IICA communication (16 pin only)

  inttm02 = 0x0022/2,	// end of counting, completion of capture, or start of
			// operations by timer channel 2 (16 pin only)

  inttm03 = 0x0024/2,	// end of counting, completion of capture, or start of
			// operations by timer channel 3 (at 16-bit or lower
			// 8-bit timer operation) (16 pin only)

  intit = 0x0026/2,	// signal detection by the internal timer

  intcmp0 = 0x0028/2,	// valid edge detection by the comparator

  // 0x002A...0x007C: reserved

  // software
  brk = 0x007E/2	// execution of brk instruction
};

static constexpr unsigned int count = 64;

template <isr_num_t IsrNum> struct line
{
  static constexpr unsigned int isr_num = IsrNum;

  static void connect_func (void(*)(void))
  {
    // interrupt functions are collected and put into the ISR table during
    // compile time.  thus this function does nothing.
  }

  static void enable (interrupt::trigger_type tt, unsigned int priority)
  {
    if (isr_num <= intcmp0 && isr_num >= intwdti)
    {
      if (priority == 0)
	disable ();
      else
      {
	// the number for the priority and mask bits starts at isr num 2.
	auto n = isr_num - 2;

	// there are only 4 priorities: 0, 1, 2, 3
	// 0 = highest, 3 = lowest
	auto p = priority / ((interrupt::max_priority + 1) / 4);

	auto p0 = (p >> 0) & 1;
	auto p1 = (p >> 1) & 1;

	auto&& pr0_reg = (volatile uint8_t*)0xFFFE8;
	auto&& pr1_reg = (volatile uint8_t*)0xFFFEC;

	pr0_reg[isr_num / 8] = (pr0_reg[n / 8] & ~(1 << (n % 8))) | (p0 << (n % 8));
	pr1_reg[isr_num / 8] = (pr1_reg[n / 8] & ~(1 << (n % 8))) | (p1 << (n % 8));

	auto&& mk_reg = (volatile uint8_t*)0xFFFE4;
	mk_reg[isr_num / 8] |= 1 << (isr_num % 8);
      }
    }
  }

  static void disable (void)
  {
    if (isr_num <= 19)
    {
      auto&& mk_reg = (volatile uint8_t*)0xFFFE4;
      mk_reg[isr_num / 8] &= ~(1 << (isr_num % 8));
    }
  }

};

} // namespace rl78g10_interrupt
} // namespace dev

#endif // includeguard_dev_rl78g10_interrupt_hpp_includeguard
