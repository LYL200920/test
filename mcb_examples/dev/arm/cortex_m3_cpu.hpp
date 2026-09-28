
#ifndef includeguard_dev_cortex_m3_cpu_includeguard
#define includeguard_dev_cortex_m3_cpu_includeguard

namespace dev
{

struct cortex_m3_cpu
{
  typedef unsigned int status_reg_value;


  #if defined(__CORTEX_M) && __CORTEX_M == 3

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

} // namespace dev
#endif // includeguard_dev_cortex_m3_cpu_includeguard
