/*

STM32F0x DAC device

*/


#ifndef includeguard_dev_stm32f0_dac_hpp_includeguard
#define includeguard_dev_stm32f0_dac_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_dac
{

template <uintptr_t RegBaseAddr>
class hw_inst
{
public:

  struct regs_t
  {
    hw_reg_rw<uint32_t> cr;		// 0x00
    hw_reg_rw<uint32_t> swtrigr;	// 0x04

    hw_reg_rw<uint32_t> dhr12r1;	// 0x08
    hw_reg_rw<uint32_t> dhr12l1;	// 0x0C
    hw_reg_rw<uint32_t> dhr8r1;		// 0x10

    hw_reg_rw<uint32_t> dhr12r2;	// 0x14
    hw_reg_rw<uint32_t> dhr12l2;	// 0x18
    hw_reg_rw<uint32_t> dhr8r2;		// 0x1C

    hw_reg_rw<uint32_t> dhr12rd;	// 0x20
    hw_reg_rw<uint32_t> dhr12ld;	// 0x24
    hw_reg_rw<uint32_t> dhr8rd;		// 0x28

    hw_reg_r<uint32_t> dor1;		// 0x2C
    hw_reg_r<uint32_t> dor2;		// 0x30

    hw_reg_rw<uint32_t> sr;		// 0x34
  };

  static_assert (sizeof (regs_t) == 0x38, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

private:

};

} // namespace stm32f0_dac
} // namespace dev

#endif // includeguard_dev_stm32f0_dac_hpp_includeguard
