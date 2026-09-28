
#ifndef includeguard_dev_rx_cpu_includeguard
#define includeguard_dev_rx_cpu_includeguard

namespace dev
{

struct rx_cpu
{
  typedef int status_reg_value;


  #ifdef __RX__

  static inline status_reg_value
  save_disable_interrupts (void)
  {
    int psw = __builtin_rx_mvfc (0);
    __builtin_rx_clrpsw ('I');
    return { psw };
  }

  static inline void
  restore_interrupts (status_reg_value r)
  {
    __builtin_rx_mvtc (0, r);
  }

  #endif
};

} // namespace dev
#endif // includeguard_dev_rx_cpu_includeguard
