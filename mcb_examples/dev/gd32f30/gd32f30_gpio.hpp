/*

GD32F30x GPIO registers

*/

#ifndef includeguard_dev_gd32f30_gpio_hpp_includeguard
#define includeguard_dev_gd32f30_gpio_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace gd32f30_gpio
{

// IO mode control bits - configure how an IO is used
static constexpr uint32_t io_analog     = 0b00'00;
static constexpr uint32_t io_input      = 0b01'00;

// use ODR to select pull up or pull down
static constexpr uint32_t io_input_pupd = 0b10'00;

// set these for output type
static constexpr uint32_t io_output_push_pull  = 0b00'00;
static constexpr uint32_t io_output_open_drain = 0b01'00;
static constexpr uint32_t io_afio_push_pull    = 0b10'00;
static constexpr uint32_t io_afio_open_drain   = 0b11'00;

static constexpr uint32_t io_output_10M  = 0b01;
static constexpr uint32_t io_output_2M   = 0b10;
static constexpr uint32_t io_output_50M  = 0b11;

static constexpr inline uint32_t set_ctlx (unsigned int io_num, uint32_t mode)
{
  return mode << ((io_num & 7) * 4);
}

static constexpr inline uint32_t set_odr (unsigned int io_num, bool val)
{
  return (uint32_t)val << (io_num & 31);
}

template <uintptr_t port_base_addr> struct port_regs
{
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x00>> ctl0 = { };

  // control register high (pins 8-15)
  // refer to control register low description.
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x04>> ctl1 = { };

  // input state register
  // 1 bit for each pin in the port
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x08>> istatr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x08>> idr = { };  // for compatibility with stm32 code

  // output control register
  // 1 bit for each pin in the port
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x0C>> octlr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x0C>> odr = { };  // for compatibility with stm32 code

  // bit operate register
  // high 16 bits: write bit number to reset the corresponding bit in odr
  // low 16 bits:  write bit number to set the corresponding bit in odr
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x10>> bopr = { };
  static constexpr hw_reg_w<uint32_t, const_addr<port_base_addr + 0x10>> bsrr = { };  // for compatibility with stm32 code

  // bit clear register
  // 1 bit for each pin in the port
  // writing 1 resets the corresponding bit in ODR
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x14>> bcr = { };
  static constexpr hw_reg_w<uint32_t, const_addr<port_base_addr + 0x14>> brr = { };   // for compatibility with stm32 code


  // configuration lock register
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x18>> lckr = { };


  /*

  bit speed register
    1 bit for each pin in the port

    set very high output speed(120MHz) when md[1:0] == 0b11.

    if the Pin output speed is more than 50 MHz,
    set corresponding bit to 1 and set md[1:0] to 0b11

    - 0: no effect
    - 1: max speed more than 50 MHz.( md[1:0] required to be set to 0b11 together ).

    Note: When the pin output speed is more than 50 MHz,
          the user should enable theI/O compensation cell.
          refer to CPS_EN bit in AFIO_CPSCTL register.
  */
  static constexpr hw_reg_rw<uint32_t, const_addr<port_base_addr + 0x3C>> bspeedr = { };

};

using pa = port_regs<0x40010800>;
using pb = port_regs<0x40010C00>;
using pc = port_regs<0x40011000>;
using pd = port_regs<0x40011400>;
using pe = port_regs<0x40011800>;
using pf = port_regs<0x40011C00>;
using pg = port_regs<0x40012000>;



/*

  alternate-function register

*/
static constexpr uintptr_t reg_base_gpioaf = 0x40010000;

static constexpr uint32_t exti_pa = 0b0000;
static constexpr uint32_t exti_pb = 0b0001;
static constexpr uint32_t exti_pc = 0b0010;
static constexpr uint32_t exti_pd = 0b0011;
static constexpr uint32_t exti_pe = 0b0100;
static constexpr uint32_t exti_pf = 0b0101;
static constexpr uint32_t exti_pg = 0b0110;

static constexpr inline uint32_t set_extissx (unsigned int io_num, uint32_t val)
{
  return val << ((io_num & 3) * 4);
}

struct afr
{
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x00>> ec = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x04>> pcf0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x08>> extiss0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x0C>> extiss1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x10>> extiss2 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x14>> extiss3 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x1c>> pcf1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<reg_base_gpioaf + 0x20>> cpsctl = { };
};


} // namespace gd32f30_gpio
} // namespace dev

#endif // includeguard_dev_gd32f30_gpio_hpp_includeguard
