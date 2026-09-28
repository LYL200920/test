/*

digital output port for STM32-like GPIO

instantiation example for port PA5:

  stm32f0_digital_output< stm32f0_gpio::pa, 5 >

*/

#ifndef includeguard_dev_stm32f0_digital_output_hpp_includeguard
#define includeguard_dev_stm32f0_digital_output_hpp_includeguard

#include <dev/digital_io_port.hpp>

namespace dev
{

template <typename PortRegs, unsigned int PortBitNum,
          enum digital_io_port::logic Logic = digital_io_port::positive_logic>
class stm32f0_digital_output
{
public:
  stm32f0_digital_output (void) { }

  // implement the functions  of digital_io_port::read, ::write, ::set_trigger_func
  // if a real instance of digital_io_port is required, need to create
  // a wrapper object around it to implement the corresponding dev_if.
  //operator digital_io_port (void) const;

  bool read (void) const
  {
    if constexpr (Logic == digital_io_port::positive_logic)
      return (port ().idr & (1 << PortBitNum)) != 0;
    else
      return (port ().idr & (1 << PortBitNum)) == 0;
  }

  void write (bool val)
  {
    //port ().bsrr = val ? (1 << PortBitNum) : (1 << (PortBitNum + 16));
    //port ().bsrr = 1 << (val ? PortBitNum : PortBitNum + 16u);

    // this gives smaller code size in most cases.

    if constexpr (Logic == digital_io_port::positive_logic)
      port ().bsrr = 1 << (PortBitNum + (1u-val) * 16u);
    else
      port ().bsrr = 1 << (PortBitNum + val * 16u);
  }

  template <typename F>
  void set_trigger_func (digital_io_port::trigger_type, F&&) { }

  void sync (void) { }
  enum digital_io_port::logic logic  (void) const { return Logic; }

  void set (void)
  {
    if constexpr (Logic == digital_io_port::positive_logic)
      port ().bsrr = 1 << PortBitNum;
    else
      port ().brr = 1 << PortBitNum;
  }

  void clear (void)
  {
    if constexpr (Logic == digital_io_port::positive_logic)
      port ().brr = 1 << PortBitNum;
    else
      port ().bsrr = 1 << PortBitNum;
  }

private:
  static constexpr PortRegs port (void) { return PortRegs (); }
};

} // namespace dev

#endif // includeguard_dev_stm32f0_digital_output_hpp_includeguard
