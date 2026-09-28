/*

various system related things for the renesas RX MCU series

*/

#ifndef includeguard_dev_rx_sys_hpp_includeguard
#define includeguard_dev_rx_sys_hpp_includeguard

#include <dev/hwreg.hpp>

namespace dev
{
struct rx_sys
{

  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x000803FE>> prcr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C11F>> pwpr = { };

  static void disable_register_protection (void)
  {
    prcr = 0xA500 | 0b0000'1011;
    pwpr = 0;
    pwpr = 0b0100'0000;
  }

  static void enable_register_protection (void)
  {
    pwpr = 0b1000'0000;
  }

  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x00080006>> syscr0 = { };
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x00080008>> syscr1 = { };


}; // struct rx_sys

} // namespace dev
#endif // includeguard_dev_rx_sys_hpp_includeguard
