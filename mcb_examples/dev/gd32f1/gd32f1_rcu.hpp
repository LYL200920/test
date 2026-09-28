/*

GD32F1 RCU registers

*/


#ifndef includeguard_dev_gd322f1_rcu_hpp_includeguard
#define includeguard_dev_gd322f1_rcu_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace gd32f1_rcu
{
static constexpr uintptr_t reg_base = 0x40021000;

static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x00>> ctl0 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x04>> cfg0 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x08>> int_ = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x0C>> apb2rst = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x10>> apb1rst = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x14>> ahben = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x18>> apb2en = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x1C>> apb1en = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x20>> bdctl = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x24>> rstsck = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x28>> ahbrst = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x2C>> cfg1 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x30>> cfg2 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x34>> ctl1 = { };


} // namespace gd32f1_rcu
} // namespace dev

#endif // includeguard_dev_gd322f1_rcu_hpp_includeguard
