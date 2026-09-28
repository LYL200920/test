/*

software interface for accessing some of the cpu features.  the minimum things
that should be provided by all cpu implementations are:

struct my_cpu
{
  // the value of the status register.  can be a simple integer or a more
  // complex structure.
  typedef ... status_reg_value;

  // capture the current cpu status register and disable the interrupts on this
  // cpu.  normally the status register contains a field that indicates whether
  // interrupts are enabled or disabled and/or the interrupt priority mask.
  static inline status_reg_value save_disable_interrupts (void);

  // restore the previously captured cpu status register, which will re-enable
  // interrupts if they were enabled before.
  static inline restore_interrupts (status_reg_value val);
};

*/

#ifndef includeguard_dev_cpu_includeguard
#define includeguard_dev_cpu_includeguard

#if defined (__CORTEX_M) && __CORTEX_M == 0

  // for STM32F0 board variant
  #include <dev/arm/cortex_m0_cpu.hpp>
  namespace dev { using this_cpu = dev::cortex_m0_cpu; }

#elif defined (__CORTEX_M) && __CORTEX_M == 3

  // for GD32F15 board variant
  #include <dev/arm/cortex_m3_cpu.hpp>
  namespace dev { using this_cpu = dev::cortex_m3_cpu; }

#else

  #error unexpected CPU type for this board __CORTEX_M

#endif

#endif // includeguard_dev_cpu_includeguard
