/*

STM32F0x EXTI registers

*/


#ifndef includeguard_dev_stm32f0_exti_hpp_includeguard
#define includeguard_dev_stm32f0_exti_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_exti
{
static constexpr uintptr_t reg_base = 0x40010400;

static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x00>> imr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x04>> emr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x08>> rtsr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x0C>> ftsr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x10>> swier = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x14>> pr = { };


} // namespace stm32f0_exti
} // namespace dev

#endif // includeguard_dev_stm32f0_exti_hpp_includeguard
