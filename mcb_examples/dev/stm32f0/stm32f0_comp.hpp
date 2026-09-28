/*

STM32F0x COMP device

*/


#ifndef includeguard_dev_stm32f0_comp_hpp_includeguard
#define includeguard_dev_stm32f0_comp_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_comp
{

template <uintptr_t RegBaseAddr>
class hw_inst
{
public:

  struct regs_t
  {
    hw_reg_rw<uint32_t> csr;		// 0x00
  };

  static_assert (sizeof (regs_t) == 0x04, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

private:

};

} // namespace stm32f0_comp
} // namespace dev

#endif // includeguard_dev_stm32f0_comp_hpp_includeguard
