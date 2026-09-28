/*

GD32F30 RCU registers ( reset and clock unit )

  High-and extra-density eset and clock control unit (RCU)

*/

#ifndef includeguard_dev_gd32f30_rcc_hpp_includeguard
#define includeguard_dev_gd32f30_rcc_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace gd32f30
{
namespace rcu
{
static constexpr uintptr_t reg_base = 0x40021000;

// control register (RCU_CTL)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x00>> ctl = { };

// configuration register 0 (RCU_CFG0)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x04>> cfg0 = { };

// interrupt register (RCU_INT)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x08>> intr = { };

// APB2 reset register (RCU_APB2RST)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x0C>> apb2rst = { };

// APB1 reset register (RCU_APB1RST)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x10>> apb1rst = { };

// AHB clock enable register (RCU_AHBEN)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x14>> ahben = { };

// APB2 clock enable register (RCU_APB2EN)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x18>> apb2en = { };

// APB1 clock enable register (RCU_APB1EN)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x1C>> apb1en = { };

// Backup domain control register (RCU_BDCTL)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x20>> bdctl = { };

// reset source/clock register (RCU_RSTSCK)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x24>> rstsck = { };

// AHB reset register (RCU_AHBRST)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x28>> ahbrst = { };

// configuration register 1 (RCU_CFG1)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x2C>> cfg1 = { };

// deep-sleep mode voltage register (RCU_DSV)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x34>> dsv = { };

// Additional clock control register (RCU_ADDCTL)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xC0>> addctl = { };

// Additional clock interrupt register (RCU_ADDINT)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xCC>> addintr = { };

// APB1 additional reset register (RCU_ADDAPB1RST)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xE0>> addapb1rst = { };

// APB1 additional enable register (RCU_ADDAPB1EN)
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xE4>> addapb1en = { };

} // namespace rcu
} // namespace gd32f30
} // namespace dev

#endif // includeguard_dev_gd32f30_rcc_hpp_includeguard
