/*

RX MCU input/output ports.

the RX MCUs are register compatible in this regard, although some MCU series
don't have some registers.  e.g. RX210 has PORTH.PODR register but no PORTG.PODR
register, while the RX64M has PORTG but not PORTH.  but still, the memory
mappings of the registers are the same.


it's possible to make an object oriented API for the IO ports, where each
IO port is represented by an object and a set of functions to make the necessary
register settings.  however, because of the hardware register layout, it
would result in bloated hardware initialization code because it would require
modifying single bits.  with this API we're always modifying bit vectors, i.e.
make settings for several ports at once.

*/


#ifndef includeguard_dev_rx_gpio_hpp_includeguard
#define includeguard_dev_rx_gpio_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace rx_gpio
{

// port direction register.
// 0: use as input pin
// 1: use as output pin
struct pdr
{
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C000>> p0 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C001>> p1 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C002>> p2 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C003>> p3 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C004>> p4 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C005>> p5 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C006>> p6 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C007>> p7 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C008>> p8 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C009>> p9 = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C00A>> pa = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C00B>> pb = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C00C>> pc = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C00D>> pd = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C00E>> pe = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C00F>> pf = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C010>> pg = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C011>> ph = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C012>> pj = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C013>> pk = { };
 static constexpr hw_reg_rw<uint8_t, const_addr<0x8C014>> pl = { };
};


// port output data register.
// if used as general output pin, sets the output value of the pin.
struct podr
{
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C020>> p0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C021>> p1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C022>> p2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C023>> p3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C024>> p4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C025>> p5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C026>> p6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C027>> p7 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C028>> p8 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C029>> p9 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C02A>> pa = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C02B>> pb = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C02C>> pc = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C02D>> pd = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C02E>> pe = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C02F>> pf = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C030>> pg = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C031>> ph = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C032>> pj = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C033>> pk = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C034>> pl = { };
};


// port input data register.
// returns the current state of the pin.  the current state of the pin can
// be read at any time, regardless of the pdr setting.
struct pidr
{
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C040>> p0 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C041>> p1 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C042>> p2 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C043>> p3 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C044>> p4 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C045>> p5 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C046>> p6 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C047>> p7 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C048>> p8 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C049>> p9 = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C04A>> pa = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C04B>> pb = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C04C>> pc = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C04D>> pd = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C04E>> pe = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C04F>> pf = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C050>> pg = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C051>> ph = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C052>> pj = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C053>> pk = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x8C054>> pl = { };
};


// port mode register.
// 0: use the pin as GPIO pin.
// 1: use the pin for peripheral module.
struct pmr
{
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C060>> p0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C061>> p1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C062>> p2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C063>> p3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C064>> p4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C065>> p5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C066>> p6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C067>> p7 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C068>> p8 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C069>> p9 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C06A>> pa = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C06B>> pb = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C06C>> pc = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C06D>> pd = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C06E>> pe = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C06F>> pf = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C070>> pg = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C071>> ph = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C072>> pj = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C073>> pk = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C074>> pl = { };
};

// port open-drain control register 0
// open drain output control for port bits 0,1,2,3
//
// ports other than PE
//    x0: CMOS output
//    x1: NMOS open-drain (sinking output)
//
// PE port:
//    00: CMOS output
//    01: NMOS open-drain (sinking output)
//    10: PMOS open-drain (sourcing output)
struct odr0
{
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C080>> p0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C082>> p1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C084>> p2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C086>> p3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C088>> p4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C08A>> p5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C08C>> p6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C08E>> p7 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C090>> p8 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C092>> p9 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C094>> pa = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C096>> pb = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C098>> pc = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C09A>> pd = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C09C>> pe = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C09E>> pf = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A0>> pg = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A2>> ph = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A4>> pj = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A6>> pk = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A8>> pl = { };
};

// port open-drain control register 1
// open drain output control for port bits 4,5,6,7
// 0: CMOS output
// 1: NMOS open-drain (sinking output)
struct odr1
{
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C081>> p0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C083>> p1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C085>> p2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C087>> p3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C089>> p4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C08B>> p5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C08D>> p6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C08F>> p7 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C091>> p8 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C093>> p9 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C095>> pa = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C097>> pb = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C099>> pc = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C09B>> pd = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C09D>> pe = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C09F>> pf = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A1>> pg = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A3>> ph = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A5>> pj = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A7>> pk = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0A9>> pl = { };
};


// pull-up resistor control register
// 0: disable input pull-up
// 1: enable input pull-up
struct pcr
{
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C0>> p0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C1>> p1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C2>> p2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C3>> p3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C4>> p4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C5>> p5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C6>> p6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C7>> p7 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C8>> p8 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0C9>> p9 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0CA>> pa = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0CB>> pb = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0CC>> pc = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0CD>> pd = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0CE>> pe = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0CF>> pf = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0D0>> pg = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0D1>> ph = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0D2>> pj = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0D3>> pk = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0D4>> pl = { };
};

// drive capacity control register
// 0: normal drive output
// 1: high-drive output
struct dscr
{
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E0>> p0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E1>> p1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E2>> p2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E3>> p3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E4>> p4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E5>> p5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E6>> p6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E7>> p7 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E8>> p8 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0E9>> p9 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0EA>> pa = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0EB>> pb = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0EC>> pc = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0ED>> pd = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0EE>> pe = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0EF>> pf = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0F0>> pg = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0F1>> ph = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0F2>> pj = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0F3>> pk = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C0F4>> pl = { };
};

// pin function select control registers
// those are actually located in the MPC unit.  before accessing them,
// make sure to enable write access in the MPC PWPR.
// the values of the PSEL bits for each pin depend on the MCU.
struct pfs
{
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C140>> p00 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C141>> p01 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C142>> p02 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C143>> p03 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C144>> p04 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C145>> p05 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C146>> p06 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C147>> p07 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C148>> p10 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C149>> p11 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C14A>> p12 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C14B>> p13 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C14C>> p14 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C14D>> p15 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C14E>> p16 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C14F>> p17 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C150>> p20 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C151>> p21 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C152>> p22 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C153>> p23 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C154>> p24 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C155>> p25 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C156>> p26 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C157>> p27 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C158>> p30 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C159>> p31 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C15A>> p32 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C15B>> p33 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C15C>> p34 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C15D>> p35 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C15E>> p36 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C15F>> p37 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C160>> p40 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C161>> p41 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C162>> p42 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C163>> p43 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C164>> p44 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C165>> p45 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C166>> p46 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C167>> p47 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C168>> p50 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C169>> p51 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C16A>> p52 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C16B>> p53 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C16C>> p54 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C16D>> p55 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C16E>> p56 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C16F>> p57 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C170>> p60 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C171>> p61 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C172>> p62 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C173>> p63 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C174>> p64 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C175>> p65 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C176>> p66 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C177>> p67 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C178>> p70 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C179>> p71 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C17A>> p72 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C17B>> p73 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C17C>> p74 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C17D>> p75 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C17E>> p76 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C17F>> p77 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C180>> p80 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C181>> p81 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C182>> p82 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C183>> p83 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C184>> p84 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C185>> p85 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C186>> p86 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C187>> p87 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C188>> p90 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C189>> p91 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C18A>> p92 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C18B>> p93 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C18C>> p94 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C18D>> p95 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C18E>> p96 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C18F>> p97 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C190>> pa0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C191>> pa1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C192>> pa2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C193>> pa3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C194>> pa4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C195>> pa5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C196>> pa6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C197>> pa7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C198>> pb0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C199>> pb1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C19A>> pb2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C19B>> pb3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C19C>> pb4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C19D>> pb5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C19E>> pb6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C19F>> pb7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A0>> pc0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A1>> pc1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A2>> pc2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A3>> pc3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A4>> pc4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A5>> pc5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A6>> pc6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A7>> pc7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A8>> pd0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1A9>> pd1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1AA>> pd2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1AB>> pd3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1AC>> pd4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1AD>> pd5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1AE>> pd6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1AF>> pd7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B0>> pe0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B1>> pe1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B2>> pe2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B3>> pe3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B4>> pe4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B5>> pe5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B6>> pe6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B7>> pe7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B8>> pf0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1B9>> pf1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1BA>> pf2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1BB>> pf3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1BC>> pf4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1BD>> pf5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1BE>> pf6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1BF>> pf7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C0>> pg0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C1>> pg1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C2>> pg2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C3>> pg3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C4>> pg4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C5>> pg5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C6>> pg6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C7>> pg7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C8>> ph0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1C9>> ph1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1CA>> ph2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1CB>> ph3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1CC>> ph4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1CD>> ph5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1CE>> ph6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1CF>> ph7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D0>> pj0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D1>> pj1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D2>> pj2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D3>> pj3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D4>> pj4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D5>> pj5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D6>> pj6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D7>> pj7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D8>> pk0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1D9>> pk1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1DA>> pk2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1DB>> pk3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1DC>> pk4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1DD>> pk5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1DE>> pk6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1DF>> pk7 = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F0>> pl0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F1>> pl1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F2>> pl2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F3>> pl3 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F4>> pl4 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F5>> pl5 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F6>> pl6 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x8C1F7>> pl7 = { };
};

// CS output enable control
// also part of the MPC unit, but somehow related to GPIO control.
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C100>> pfcse = { };

static constexpr hw_reg_rw<uint8_t, const_addr<0x8C102>> pfcss0 = { };
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C103>> pfcss1 = { };

// address output control
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C104>> pfaoe0 = { };
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C105>> pfaoe1 = { };

// external bus control
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C106>> pfbcr0 = { };
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C107>> pfbcr1 = { };

// ethernet control
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C10E>> pfenet = { };

// USB control
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C114>> pfusb0 = { };
static constexpr hw_reg_rw<uint8_t, const_addr<0x8C115>> pfusb1 = { };

/*

a small functor wrapper around the PODR registers.

there are two ways how an output port can be used.

1) the output bit is shared with other output bits.
   in this case, special care has to be taken when modifying the bit values,
   in particular with regard to interrupt handling and re-entrancy.
   shared bits can't be written as part of a DTC transfer chain.

2) the output bit is shared with other output bits in an exclusive way or
   is a single output on the port block.
   in this case, a single byte write is sufficient to modify the bit value.
   this can be used as part of a DTC transfer chain.

the output port registers are basically the same for all RX MCUs.
*/

template <typename PortOutputReg, unsigned int BitNumber>
class shared_output_port
{
public:
  static constexpr unsigned int bit_number = BitNumber;

  void operator () (bool val) { utils::atomic_set_bit (val, BitNumber, &reg); }

  void set (void) { utils::atomic_set_bit (BitNumber, &reg); }
  void clear (void) {  utils::atomic_clear_bit (BitNumber, &reg); }

  // copy the bit from the specified location to this shared output port.
  void set_from (unsigned int src_bit_num, unsigned int src_bits)
  {
    utils::atomic_copy_bit (src_bit_num, src_bits, BitNumber, &reg);
  }

private:
  static constexpr PortOutputReg reg = { };
};


template <typename PortOutputReg, uint8_t OnValue, uint8_t OffValue>
class exclusive_output_port
{
public:
  static constexpr uint8_t on_value = OnValue;
  static constexpr uint8_t off_value = OffValue;

  void operator () (bool val) { reg = val ? OnValue : OffValue; }

  void set (void) { reg = OnValue; }
  void clear (void) { reg = OffValue; }

  // test the bit at the specified location and set the output port to either
  // OnValue or OffValue
  void set_from (unsigned int src_bit_num, unsigned int src_bits)
  {
    reg = utils::get_bit (src_bits, src_bit_num) ? OnValue : OffValue;
  }

private:
  static constexpr PortOutputReg reg = { };
};

} // namespace rx_gpio
} // namespace dev

#endif // includeguard_dev_rx_gpio_hpp_includeguard
