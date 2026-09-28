/*

STM32F0x SYSCFG registers

*/


#ifndef includeguard_dev_stm32f0_syscfg_hpp_includeguard
#define includeguard_dev_stm32f0_syscfg_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_syscfg
{
static constexpr uintptr_t reg_base = 0x40010000;

static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x00>> cfgr1 = { };

static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x08>> exticr1 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x0C>> exticr2 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x10>> exticr3 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x14>> exticr4 = { };

static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x18>> cfgr2 = { };

static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x80>> itline0 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x84>> itline1 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x88>> itline2 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x8C>> itline3 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x90>> itline4 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x94>> itline5 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x98>> itline6 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0x9C>> itline7 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xA0>> itline8 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xA4>> itline9 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xA8>> itline10 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xAC>> itline11 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xB0>> itline12 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xB4>> itline13 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xB8>> itline14 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xBC>> itline15 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xC0>> itline16 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xC4>> itline17 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xC8>> itline18 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xCC>> itline19 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xD0>> itline20 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xD4>> itline21 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xD8>> itline22 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xDC>> itline23 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xE0>> itline24 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xE4>> itline25 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xE8>> itline26 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xEC>> itline27 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xF0>> itline28 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xF4>> itline29 = { };
static constexpr hw_reg_rw<uint32_t, const_addr<reg_base + 0xF8>> itline30 = { };


} // namespace stm32f0_syscfg
} // namespace dev

#endif // includeguard_dev_stm32f0_syscfg_hpp_includeguard
