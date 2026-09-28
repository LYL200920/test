
#ifndef includeguard_dev_rl78_cpu_includeguard
#define includeguard_dev_rl78_cpu_includeguard

namespace dev
{

struct rl78_cpu
{
  typedef unsigned char status_reg_value;

  #ifdef __RL78__

  static inline status_reg_value
  save_disable_interrupts (void)
  {
    // if the result variable of the asm block is unused, the compiler
    // might eliminate it completely...
    unsigned char psw;
    asm ( "mov	a,psw"		"\n\t"
	  "mov	%0,a"		"\n\t"
	 : "=r" (psw) : : "a");

    // ... that's ok, but we really want to have the interrupts off,
    // so make it a separate asm block, which will remain even if the psw
    // save block gets eliminated.
    asm ("di");

    return { psw };
  }

  static inline void
  restore_interrupts (status_reg_value psw)
  {
    asm ( "mov	a,%0\n\t"
	  "mov	psw,a"
	 : : "r" (psw) : "a", "memory");
  }

  #endif
};

} // namespace dev
#endif // includeguard_dev_rl78_cpu_includeguard
