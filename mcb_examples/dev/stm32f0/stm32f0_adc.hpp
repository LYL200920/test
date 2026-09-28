/*

STM32F0x ADC device

*/


#ifndef includeguard_dev_stm32f0_adc_hpp_includeguard
#define includeguard_dev_stm32f0_adc_hpp_includeguard

#include <chrono>

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_adc
{

template <uintptr_t RegBaseAddr, unsigned int ClockHz>
class hw_inst
{
public:
  static constexpr unsigned int clock_hz = ClockHz;

  typedef std::chrono::duration<int32_t, std::ratio<1, clock_hz>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;

  struct regs_t
  {
    hw_reg_rw<uint32_t> isr;		// 0x00
    hw_reg_rw<uint32_t> ier;		// 0x04
    hw_reg_rw<uint32_t> cr;		// 0x08
    hw_reg_rw<uint32_t> cfgr1;		// 0x0C
    hw_reg_rw<uint32_t> cfgr2;		// 0x10
    hw_reg_rw<uint32_t> smpr;		// 0x14
    uint32_t res_0x18;
    uint32_t res_0x1C;
    hw_reg_rw<uint32_t> tr;		// 0x20
    uint32_t res_0x24;
    hw_reg_rw<uint32_t> chselr;		// 0x28
    uint32_t res_0x2C;
    uint32_t res_0x30;
    uint32_t res_0x34;
    uint32_t res_0x38;
    uint32_t res_0x3C;
    hw_reg_r<uint32_t> dr;		// 0x40

    uint32_t res_0x44_0x304[(0x304 - 0x44)/4 + 1];

    hw_reg_rw<uint32_t> ccr;		// 0x308
  };

  static_assert (sizeof (regs_t) == 0x30C, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

  hw_inst (void)
  {
    disable ();

    regs ().ier = 0;  // disable all interrupts (we're not using any yet).
    regs ().cr = 0;
  }

  bool is_enabled (void) const { return (regs ().cr & (1 << 0)) != 0; }
  bool is_disabled (void) const { return !is_enabled (); }
  bool is_converting (void) const { return (regs ().cr & (1 << 2)) != 0; }
  bool is_ready (void) const
  {
    auto cr = regs ().cr;
    return (cr & (1 << 2)) == 0 && (cr & (1 << 1)) == 0 && (cr & (1 << 0)) != 0;
  }

  void stop_conversion (void)
  {
    if (is_converting ())
    {
      // stop conversion and wait until it's ready
      regs ().cr |= (1 << 4);
      while ((regs ().cr & (1 << 4)) != 0) { }
    }
  }

  void disable (void)
  {
    stop_conversion ();

    if (is_enabled ())
    {
      // send disable command and wait until it's off.
      regs ().cr |= (1 << 1);
      while ((regs ().cr & ((1 << 1) | (1 << 0))) != 0) { }

      regs ().ier &= ~(1 << 0);
    }
  }

  void enable (void)
  {
    // ensure that ADRY = 0 and clear it if needed.
    if ((regs ().isr & (1 << 0)) != 0)
      regs ().isr |= (1 << 0);

    // enable ADC
    regs ().cr |= (1 << 0);

    // wait until ADC is ready
    while ((regs ().isr & (1 << 0)) == 0) { }
  }

  void calibrate (void)
  {
    if (is_disabled ())
    {
      regs ().cr = (1 << 31);
      while ((regs ().cr & (1 << 31)) != 0) { }
    }
  }

  unsigned int
  convert_single (unsigned int ch, duration sampling_time = std::chrono::microseconds (1))
  {
    if (is_converting ())
      return 0;

    regs ().cfgr1 = 0
	| (1 << 16)		// discontinuous mode enable
	| (0 << 15)		// auto-off disable
	| (0 << 14)		// wait conversion mode disable
	| (0 << 13)		// continuous mode disable
	| (0 << 12)		// preserve old data on overrun
	| (0b00 << 10)		// hardware trigger disable
	| (0b000 << 6)		// external trigger selection (ignore)
	| (0 << 5)		// right-align data
	| (0b00 << 3)		// 12 bit resolution
	| (0 << 2)		// upward scan direction
	| (0 << 1)		// dma one-shot mode (ignore)
	| (0 << 0)		// dma disable
	| 0;

    regs ().smpr = convert_sample_time (sampling_time.count ());
    regs ().chselr = 1 << ch;

    // start conversion and wait for it to finish
    regs ().cr |= 1 << 2;
    while ((regs ().cr & (1 << 2)) != 0) { }

    return regs ().dr;
  }

private:
  static constexpr unsigned int convert_sample_time (unsigned int clock_cycles)
  {
    if (clock_cycles < 3)
      return 0b000; // 1.5 cycles

    if (clock_cycles < 13)
      return 0b001; // 7.5 cycles

    if (clock_cycles < 28)
      return 0b010;  // 13.5 cycles

    if (clock_cycles < 41)
      return 0b011;  // 28.5 cycles

    if (clock_cycles < 55)
      return 0b100;  // 41.5 cycles

    if (clock_cycles < 71)
      return 0b101;  // 55.5 cycles

    if (clock_cycles < 239)
      return 0b110;  // 71.5 cycles

    return 0b111; // 239.5 cycles
  }
};

} // namespace stm32f0_adc
} // namespace dev

#endif // includeguard_dev_stm32f0_adc_hpp_includeguard
