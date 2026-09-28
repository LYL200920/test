
#ifndef includeguard_dev_arm_cortex_m4_cpu_includeguard
#define includeguard_dev_arm_cortex_m4_cpu_includeguard

namespace dev
{
namespace arm
{

struct cortex_m4_cpu
{
  typedef unsigned int status_reg_value;

  #if defined (__CORTEX_M) && __CORTEX_M == 4

  static inline status_reg_value
  save_disable_interrupts (void)
  {
    status_reg_value r;
    asm volatile (
	"mrs	%0,primask"	"\n\t"
	"cpsid	i"
	: "=r" (r) : : "memory");
    return { r };
  }

  static inline void
  restore_interrupts (status_reg_value r)
  {
    asm volatile ("msr	primask,%0" : : "r" (r) : "memory");
  }

  #endif
};

} // namespace arm
} // namespace dev
#endif // includeguard_dev_arm_cortex_m4_cpu_includeguard
