
/*

ARM Cortex-M0 interrupts

notice that the max. available priority values depend on how many
priority bits the MCU implementation uses in the NVIC.

for example the STM32F0xx uses 2 bits for the priority levels
(#define __NVIC_PRIO_BITS 2)

*/

#ifndef includeguard_cortex_m0_interrupt_hpp_includeguard
#define includeguard_cortex_m0_interrupt_hpp_includeguard

#include <cstdint>
#include <dev/hwreg.hpp>
#include <dev/interrupt.hpp>
#include <utils/bits.hpp>

namespace dev
{

namespace cortex_m0_interrupt
{

#if __has_include (<dev/arm/cortex_m0_interrupt.priority.hpp>)
  #include <dev/arm/cortex_m0_interrupt.priority.hpp>
#else
  #warning Cortex M0 interrupt priorites are MCU specific and require priority mapping
  constexpr inline unsigned int remap_priority (unsigned int val)
  {
    return val;
  }
#endif

enum isr_num_t
{
  // arm cortex-m0 processor defined exceptions

  //                  | our logical number | vector table offset
  // -----------------+--------------------+------------------------
  // reset stack ptr  |  = -16             // 1x4
  // reset PC         |  = -15             // 1x4
  nmi                    = -14,            // 2x4
  hard_fault             = -13,            // 3x4
  svcall                 = -5,             // 11x4
  pendsv                 = -2,             // 14x4
  systick                = -1,             // 15x4

  // ext_interrupt_0     = 0               // 16x4

  // all other external IRQs are MCU specific
  // the board library is suppoed to provide the corresponding inlcude
  // file that adds all the interrupt numbers of the system here.

  #if __has_include (<dev/arm/cortex_m0_interrupt.x.hpp>)
    #include <dev/arm/cortex_m0_interrupt.x.hpp>
  #else
    max_interrupts = 32
  #endif
};

// do not include the reset SP and reset PC entries in the vector table
// there are max. 32 external IRQs.
static constexpr unsigned int count = 14 + max_interrupts;

  // SCB registers (interrupt relevant only)
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED04>> ICSR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED0C>> AIRCR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED1C>> SHPR2 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000ED20>> SHPR3 = { };

  // NVIC registers
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E100>> ISER = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E180>> ICER = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E200>> ISPR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0xE000E280>> ICPR = { };

template <isr_num_t IsrNum> struct line
{
  static constexpr int isr_num = IsrNum;

  static_assert (isr_num < max_interrupts, "");


  // each external interrupt has an 8 bit priority field in the
  // IPR array (0xE000E400-0xE000E41C)
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
      // internal exceptions/interrupts can't be enabled/disabled, but
      // we can use this function to set the priority
      if (isr_num == svcall)
	SHPR2 = priority << 24;
      else if (isr_num == pendsv)
	SHPR3 = (SHPR3 & 0xFF00FFFF) | ((priority & 0xFF) << 16);
      else if (isr_num == systick)
	SHPR3 = (SHPR3 & 0x00FFFFFF) | ((priority & 0xFF) << 24);
    }
    else
    {
      // external interrupts use the IPR array
      unsigned int n = (unsigned int)isr_num & 3;
      IPR = (IPR & (0xFFFFFF00 << (n*8))) | ((priority & 0xFF) << (n*8));

      ISER = 1 << (unsigned int)isr_num;
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
      ICER = 1 << (unsigned int)isr_num;
    }
  }

  static bool status (void)
  {
    if (isr_num < 0)
      return false;
    else
      return utils::get_bit (ISPR.read (), (unsigned int)isr_num);
  }

  static void clear (void)
  {
    if (isr_num < 0)
    {
    }
    else
      ICPR = 1 << (unsigned int)isr_num;
  }

  static void trigger (void)
  {
    if (isr_num < 0)
    {
    }
    else
      ISPR = 1 << (unsigned int)isr_num;
  }
};


} // namespace cortex_m0_interrupt

} // namespace dev

#endif // includeguard_cortex_m0_interrupt_hpp_includeguard
