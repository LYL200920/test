/*

GD32F1x0 ADC device

*/


#ifndef includeguard_dev_gd32f1_adc_hpp_includeguard
#define includeguard_dev_gd32f1_adc_hpp_includeguard

#include <chrono>
#include <thread>

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace gd32f1_adc
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
    hw_reg_rw<uint32_t> stat;		// 0x00
    hw_reg_rw<uint32_t> ctl0;		// 0x04
    hw_reg_rw<uint32_t> ctl1;		// 0x08
    hw_reg_rw<uint32_t> sampt0;		// 0x0C
    hw_reg_rw<uint32_t> sampt1;		// 0x10
    hw_reg_rw<uint32_t> ioff0;		// 0x14
    hw_reg_rw<uint32_t> ioff1;		// 0x18
    hw_reg_rw<uint32_t> ioff2;		// 0x1C
    hw_reg_rw<uint32_t> ioff3;		// 0x20
    hw_reg_rw<uint32_t> wdht;		// 0x24
    hw_reg_rw<uint32_t> wdlt;		// 0x28
    hw_reg_rw<uint32_t> rsq0;		// 0x2C
    hw_reg_rw<uint32_t> rsq1;		// 0x30
    hw_reg_rw<uint32_t> rsq2;		// 0x34
    hw_reg_rw<uint32_t> isq;		// 0x38
    hw_reg_r<uint32_t> idata0;		// 0x3C
    hw_reg_r<uint32_t> idata1;		// 0x40
    hw_reg_r<uint32_t> idata2;		// 0x44
    hw_reg_r<uint32_t> idata3;		// 0x48
    hw_reg_r<uint32_t> rdata;		// 0x4C

    uint32_t res_0x50_0x7C[(0x7C - 0x50)/4 + 1];

    hw_reg_rw<uint32_t> ovsampctl;	// 0x80
  };

  static_assert (sizeof (regs_t) == 0x84, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

  hw_inst (void)
  {
    disable ();

    // disable all interrupts (we're not using any yet).
    regs ().ctl0 = 0;
  }

  bool is_enabled (void) const
  {
    return (regs ().ctl1 & (1 << 0)) != 0; // ADCON
  }

  bool is_disabled (void) const { return !is_enabled (); }

  bool is_converting (void) const
  {
    // there is no direct way of checking for an ongoing conversion.
    // STRC / STIC is set by the hardware when a conversion starts
    // EOIC / EOC is set by the hardware when a conversion finishes
    // 
    // STRC / STIC  |  EOIC / EOC  |  assumed condition
    //     0        |       0      |     not started, not converting, cleared
    //     0        |       1      |     not started / cleared, finished converting
    //     1        |       0      |     started, converting
    //     1        |       1      |     started, finished convering

    auto r = regs ().stat;

    // merge bits STIC -> STRC, EOC -> EOIC
    // resulting bits are in 4 and 2
    r = (r | (r << 1)) & 0b000010100;

    switch (r)
    {
      case 0b000010000:
        return true;

      default:
      case 0b000000000:
      case 0b000000100:
      case 0b000010100:
        return false;
    }
  }

  bool is_ready (void) const
  {
    return !is_converting () && is_enabled ();
  }

  void enable (void)
  {
    if (!is_enabled ())
      regs ().ctl1 |= 1 << 0;  // ADCON = 1

    // enable temperature and internal vref channels
    // set trigger to software enable (ETSRC = 0b111 = SWRCST)
    regs ().ctl1 |= (1 << 23) | (0b111 << 17);

/*
    // ensure that ADRY = 0 and clear it if needed.
    if ((regs ().isr & (1 << 0)) != 0)
      regs ().isr |= (1 << 0);

    // enable ADC
    regs ().cr |= (1 << 0);

    // wait until ADC is ready
    while ((regs ().isr & (1 << 0)) == 0) { }
*/
  }

  void stop_conversion (void)
  {
    // it seems there is no way to stop an ongoing conversion
  }

  void disable (void)
  {
    stop_conversion ();

    if (is_enabled ())
    {
      regs ().ctl1 &= ~(1 << 0);  // ADCON = 0
    }
  }

  unsigned int
  convert_single (unsigned int ch, duration sampling_time = std::chrono::microseconds (1))
  {
//    regs ().ctl0 &= ~(1 << 8); // clear SM
//    regs ().ctl0 |= (1 << 11); // set DISRC

    regs ().ctl0 &= ~((1 << 11) | (1 << 8)); // set DISRC, clear SM
    regs ().ctl1 &= ~(1 << 1); // clear CNT

    regs ().rsq0 = 0;   // conversion group length = 0+1
    regs ().rsq2 = ch;

    auto reg_time_val = convert_sample_time (sampling_time.count ());

    if (ch >= 10)
      regs ().sampt0 = reg_time_val << ((ch - 10) * 3);
    else
      regs ().sampt0 = reg_time_val << (ch * 3);

    // assume default settings in CTL1
    //  * right-align data
    //  * 12-bit mode

    regs ().ctl1 |= (1 << 22); // set SWRCST to start the conversion

    // wait for conversion to finish (EOC)
    while ((regs ().stat & (1 << 1)) == 0) { }

    unsigned int conversion_result = regs ().rdata;

    regs ().stat &= ~(1 << 1); // clear EOC
    return conversion_result;

/*
    // clear STAT.STRC bit ???

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
*/
  }

  void calibrate (void)
  {
    enable (); // make sure ADCON = 1

    // delay 14 ADCCLK
    std::this_thread::sleep_for (duration (14));

    regs ().ctl1 |= 0
       | (1 << 3) // set RSTCLB (optional)
       | (1 << 2) // set CLB (start calibration)
       | 0;

    // wait for calibration
    while ((regs ().ctl1 & (1 << 2)) != 0) { }
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

} // namespace gd32f1_adc
} // namespace dev

#endif // includeguard_dev_gd32f1_adc_hpp_includeguard
