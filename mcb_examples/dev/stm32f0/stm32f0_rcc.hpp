/*

STM32Fx RCC registers

*/


#ifndef includeguard_dev_stm32f0_rcc_hpp_includeguard
#define includeguard_dev_stm32f0_rcc_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_rcc
{
static constexpr uintptr_t reg_base = 0x40021000;

static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x00>> cr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x04>> cfgr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x08>> cir = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x0C>> apb2rstr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x10>> apb1rstr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x14>> ahbenr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x18>> apb2enr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x1C>> apb1enr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x20>> bdcr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x24>> csr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x28>> ahbrstr = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x2C>> cfgr2 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x30>> cfgr3 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x34>> cr2 = { };


} // namespace stm32f0_rcc
} // namespace dev

#endif // includeguard_dev_stm32f0_rcc_hpp_includeguard
