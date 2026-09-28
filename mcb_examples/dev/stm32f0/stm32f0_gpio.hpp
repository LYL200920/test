/*

STM32Fx GPIO registers

*/


#ifndef includeguard_dev_stm32f0_gpio_hpp_includeguard
#define includeguard_dev_stm32f0_gpio_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_gpio
{

template <uintptr_t port_base_addr> struct port_regs
{
  // mode register
  // 2 bits for each pin in the port
  //
  // 0b00 - input mode (reset state)
  // 0b01 - general purpose output mode
  // 0b10 - alternate function mode
  // 0b11 - analog mode
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x00>> moder = { };

  // output type register
  // 1 bit for each pin in the port
  //
  // 0b0 - push-pull output
  // 0b1 - open-drain output
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x04>> otyper = { };


  // output speed register
  // 2 bits for each pin in the port
  //
  // 0bx0 - low speed
  // 0b01 - medium speed
  // 0b11 - high speed
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x08>> ospeedr = { };

  // pull-up/pull-down register
  // 2 bits for each pin in the port
  //
  // 0b00 - no pull-up/pull-down
  // 0b01 - pull-up
  // 0b10 - pull-down
  // 0b11 - reserved
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x0C>> pupdr = { };

  // input data register
  // 1 bit for each pin in the port
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x10>> idr = { };

  // output data register
  // 1 bit for each pin in the port
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x14>> odr = { };

  // bit set/reset register
  // high 16 bits: write bit number to reset the corresponding bit in odr
  // low 16 bits:  write bit number to set the corresponding bit in odr
  static constexpr hw_reg_w<uint32_t, const_addr<port_base_addr + 0x18>> bsrr = { };

  // lock register
  static constexpr hw_reg_w<uint32_t, const_addr<port_base_addr + 0x1C>> lckr = { };

  // alternatve function low register (pins 0-7)
  // 4 bits for each pin in the port
  // 0b0000: AF0
  // 0b0001: AF1
  // 0b0010: AF2
  // 0b0011: AF3
  // 0b0100: AF4
  // 0b0101: AF5
  // 0b0110: AF6
  // 0b0111: AF7
  // 0b1xxx: reserved
  static constexpr hw_reg_w<uint32_t, const_addr<port_base_addr + 0x20>> afrl = { };

  // alternatve function low register (pins 8-15)
  // 4 bits for each pin in the port
  // 0b0000: AF0
  // 0b0001: AF1
  // 0b0010: AF2
  // 0b0011: AF3
  // 0b0100: AF4
  // 0b0101: AF5
  // 0b0110: AF6
  // 0b0111: AF7
  // 0b1xxx: reserved
  static constexpr hw_reg_w<uint32_t, const_addr<port_base_addr + 0x24>> afrh = { };

  // bit reset register
  // 1 bit for each pin in the port
  //
  // writing 1 resets the corresponding bit in ODR
  static constexpr hw_reg_w<uint32_t, const_addr<port_base_addr + 0x28>> brr = { };
};

using pa = port_regs<0x48000000>;
using pb = port_regs<0x48000400>;
using pc = port_regs<0x48000800>;
using pd = port_regs<0x48000C00>;
using pe = port_regs<0x48001000>;
using pf = port_regs<0x48001400>;


} // namespace stm32f0_gpio
} // namespace dev

#endif // includeguard_dev_stm32f0_gpio_hpp_includeguard
