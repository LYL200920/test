
/*

ARM Cortex-M4 interrupts

notice that the max. available priority values depend on how many
priority bits the MCU implementation uses in the NVIC.
on cortex-m3 it could be theoretically up to 256 priority levels.

*/

#ifndef includeguard_dev_arm_cortex_m4_interrupt_includeguard
#define includeguard_dev_arm_cortex_m4_interrupt_includeguard

#include <cstdint>
#include <dev/hwreg.hpp>
#include <dev/interrupt.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace cortex_m4_interrupt
{

#if __has_include (<dev/arm/cortex_m4_interrupt.priority.hpp>)
  #include <dev/arm/cortex_m4_interrupt.priority.hpp>
#else
  #warning Cortex M4 interrupt priorites are MCU specific and require priority mapping
  constexpr inline unsigned int remap_priority (unsigned int val)
  {
    return val;
  }
#endif

enum isr_num_t
{
  // arm cortex-m4 processor defined exceptions

  //                  | our logical number | vector table offset
  // -----------------+--------------------+------------------------
  // reset stack ptr  |  = -16             // 0x4
  // reset PC         |  = -15             // 1x4
  nmi                    = -14,            // 2x4
  hard_fault             = -13,            // 3x4
  mem_manage             = -12,            // 4x4
  bus_fault              = -11,            // 5x4
  usage_fault            = -10,            // 6x4
  svcall                 = -5,             // 11x4
  debug_monitor          = -4,             // 12x4
  pendsv                 = -2,             // 14x4
  systick                = -1,             // 15x4

  // ext_interrupt_0     = 0               // 16x4

  // all other external IRQs are MCU specific

  #if __has_include (<dev/arm/cortex_m4_interrupt.x.hpp>)
    #include <dev/arm/cortex_m4_interrupt.x.hpp>
  #else
    max_interrupts = 240
  #endif
};

  // do not include the reset SP and reset PC entries in the vector table
  // there are max. 68 external IRQs.
  static constexpr unsigned int count = 14 + max_interrupts;

  // SCB registers (interrupt relevant only)
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED00>> CPUID = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED04>> ICSR  = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED08>> VTOR  = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED0C>> AIRCR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED10>> SCR   = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED14>> CCR   = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED18>> SHPR1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED1C>> SHPR2 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED20>> SHPR3 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED24>> SHCSR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED28>> CFSR  = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED2C>> HFSR  = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED34>> MMAR  = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED38>> BFAR  = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED3C>> AFSR  = { };

  // NVIC registers
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000EF00>> STIR = { };

template <isr_num_t IsrNum> struct line
{
  static constexpr int isr_num = IsrNum;

  static_assert (isr_num < max_interrupts);

  // -------------------------------------------------------------------------------------------
  // NVIC registers array

  // ISER array:
  // Note: ISER0 bits 0 to 31 are for interrupt 0 to 31, respectively
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E100 + (isr_num/32)*4 >> ISER = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E180 + (isr_num/32)*4 >> ICER = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E200 + (isr_num/32)*4 >> ISPR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E280 + (isr_num/32)*4 >> ICPR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E300 + (isr_num/32)*4 >> IABR = { };
  // IPR array
  // Note: each external interrupt has an 8 bit priority field in the
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E400 + (isr_num/4)*4 >> IPR = { };

  static void connect_func (void(*)(void))
  {
    // interrupt functions are collected and put into the ISR table during
    // compile time.  thus this function does nothing.
  }

  static void enable (interrupt::trigger_type tt, unsigned int priority)
  {
    priority = remap_priority (priority);

    if (isr_num < 0)
    {
      // FIXME: internal exceptions/interrupts can't be enabled/disabled,
      //        but if we can use a function to set the priority
    }
    else
    {
      // external interrupts use the IPR array
      unsigned int n = (unsigned int)isr_num & 3;
      IPR = (IPR & (0xFFFFFF00 << (n*8))) | ((priority & 0xFF) << (n*8));

      ISER = 1 << (unsigned int)(isr_num % 32);
    }
  }

  static void disable (void)
  {
    if (isr_num < 0)
    {
      // internal exceptions/interrupts can't be enabled/disabled
    }
    else
    {
      ICER = 1 << (unsigned int)(isr_num % 32);
    }
  }

  static bool status (void)
  {
    if (isr_num < 0)
      return false;
    else
      return utils::get_bit (ISPR.read (), (unsigned int)(isr_num % 32));
  }

  static void clear (void)
  {
    if (isr_num < 0)
    {
    }
    else
      ICPR = 1 << (unsigned int)(isr_num % 32);
  }

  static void trigger (void)
  {
    if (isr_num < 0)
    {
    }
    else
      ISPR = 1 << (unsigned int)(isr_num % 32);
  }
};


} // namespace cortex_m4_interrupt
} // namespace dev

#endif // includeguard_dev_arm_cortex_m4_interrupt_includeguard
