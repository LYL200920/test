
/*

the SCId device is used in the RX63 MCU as SCI12.  it has some extended
functions over the SCIc devices.

so far, there was no use for SCId, so it is not really implemented.

*/


#ifndef includeguard_dev_rx_scic_hpp_includeguard
#define includeguard_dev_rx_scic_hpp_includeguard

#include <dev/rx_scic.hpp>

namespace dev
{

template <uintptr_t RegAddress, unsigned int SciNum,
	  unsigned int PeripheralClock, unsigned int MaxBitRate,
	  typename TXI_InterruptLine,
	  typename RXI_InterruptLine,
	  typename TEI_InterruptLine,
	  typename ERI_InterruptLine,
	  typename ModuleEnableDisableFunc >
class rx_scid : public rx_scic < RegAddress, SciNum, PeripheralClock, MaxBitRate,
				 TXI_InterruptLine, RXI_InterruptLine,
				 TEI_InterruptLine, ERI_InterruptLine,
				 ModuleEnableDisableFunc >
{
public:

protected:
  struct regs_t : rx_scic < RegAddress, SciNum, PeripheralClock, MaxBitRate,
				 TXI_InterruptLine, RXI_InterruptLine,
				 TEI_InterruptLine, ERI_InterruptLine,
				 ModuleEnableDisableFunc >::regs_t
  {
    uint8_t padding[18];
    volatile uint8_t esmer;   // extended serial module enable reg.   sci12 = 0x0008B320
    volatile uint8_t cr0;     // control register 0, sci12 = 0x0008B321
    volatile uint8_t cr1;     // control register 1, sci12 = 0x0008B322
    volatile uint8_t cr2;     // control register 2, sci12 = 0x0008B323
    volatile uint8_t cr3;     // control register 3, sci12 = 0x0008B324
    volatile uint8_t pcr;     // port control register, sci12 = 0x0008B325
    volatile uint8_t icr;     // interrupt control register, sci12 = 0x0008B326
    volatile uint8_t str;     // status register, sci12 = 0x0008B327
    volatile uint8_t stcr;    // status clear register, sci12 = 0x0008B328
    volatile uint8_t cf0dr;   // control field 0 data register, sci12 = 0x0008B329
    volatile uint8_t cf0cr;   // control field 0 compare enable, sci12 = 0x0008B32A
    volatile uint8_t cf0rr;   // control field 0 receive data register, sci12 = 0x0008B32B
    volatile uint8_t pcf1dr;  // primary control field 1 data register, sci12 = 0x0008B32C
    volatile uint8_t scf1dr;  // secondary control field 1 data register, sci12 = 0x0008B32D
    volatile uint8_t cf1cr;   // control field 1 compare enable register, sci12 = 0x0008B32E
    volatile uint8_t cf1rr;   // control field 1 receive data register, sci12 = 0x0008B32F
    volatile uint8_t tcr;     // timer control register, sci12 = 0x0008B330
    volatile uint8_t tmr;     // timer mode register, sci12 = 0x0008B331
    volatile uint8_t tpre;    // timer prescaler register, sci12 = 0x0008B332
    volatile uint8_t tcnt;    // timer count register, sci12 = 0x0008B333
  };

  static_assert (sizeof (regs_t) == 14 + 18 + 20, "");
};

} // namespace dev
#endif // includeguard_dev_rx_scic_hpp_includeguard
