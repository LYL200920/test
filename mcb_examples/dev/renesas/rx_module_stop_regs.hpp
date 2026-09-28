/*

a small functor wrapper around the module stop registers so that it can be
used as template arguments for device drivers to enable/disable the module.

the MSTPCRA, MSTPCRB, MSTPCRC, MSTPCRD are mapped at the same addresses
on various RX MCUs (RX100, RX63, RX64, RX71M ...) but the bits have slightly
different meanings, depending on the present peripherals.
*/

#ifndef includeguard_rx_module_stop_regs_includeguard
#define includeguard_rx_module_stop_regs_includeguard

#include <dev/hwreg.hpp>

namespace dev
{

template <unsigned int RegAddr, unsigned int BitNumber>
class rx_module_stop_reg_set_bit_func
{
public:
  void operator () (bool val)
  {
    // notice that the bits are negative logic
    //   0 = module on
    //   1 = module stopped/off
    reg = (reg & ~(1u << BitNumber)) | ((unsigned int)(!val) << BitNumber);
  }

private:
  static constexpr hw_reg_rw<uint32_t, const_addr<RegAddr>> reg = { };
};

template <unsigned int BitNumber> using rx_mstpcra_bit = rx_module_stop_reg_set_bit_func < 0x80010, BitNumber >;
template <unsigned int BitNumber> using rx_mstpcrb_bit = rx_module_stop_reg_set_bit_func < 0x80014, BitNumber >;
template <unsigned int BitNumber> using rx_mstpcrc_bit = rx_module_stop_reg_set_bit_func < 0x80018, BitNumber >;
template <unsigned int BitNumber> using rx_mstpcrd_bit = rx_module_stop_reg_set_bit_func < 0x8001C, BitNumber >;

} // namespace dev
#endif // includeguard_rx_module_stop_regs_includeguard
