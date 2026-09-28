/*

RX MCU bus state controller related registers

*/

#ifndef includeguard_dev_rx_bsc_hpp_includeguard
#define includeguard_dev_rx_bsc_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
struct rx_bsc
{
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83002>> cs0mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83004>> cs0wcr1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83008>> cs0wcr2 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83802>> cs0cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8380A>> cs0rec = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83012>> cs1mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83014>> cs1wcr1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83018>> cs1wcr2 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83812>> cs1cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8381A>> cs1rec = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83022>> cs2mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83024>> cs2wcr1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83028>> cs2wcr2 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83822>> cs2cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8382A>> cs2rec = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83032>> cs3mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83034>> cs3wcr1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83038>> cs3wcr2 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83832>> cs3cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8383A>> cs3rec = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83042>> cs4mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83044>> cs4wcr1 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83842>> cs4cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8384A>> cs4rec = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83048>> cs4wcr2 = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83052>> cs5mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83054>> cs5wcr1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83058>> cs5wcr2 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83852>> cs5cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8385A>> cs5rec = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83062>> cs6mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83064>> cs6wcr1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83068>> cs6wcr2 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83862>> cs6cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8386A>> cs6rec = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83072>> cs7mod = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83074>> cs7wcr1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83078>> cs7wcr2 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83872>> cs7cr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8387A>> cs7rec = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x83880>> csrecen = { };

  static constexpr hw_reg_rw<uint8_t,  const_addr<0x83C00>> sdccr = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x83C01>> sdcmod = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x83C02>> sdamod = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x83C10>> sdself = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83C14>> sdrfcr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83C16>> sdrfen = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x83C20>> sdicr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83C24>> sdir = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x83C40>> sdadr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x83C44>> sdtr = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x83C48>> sdmod = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x83C50>> sdsr = { };

  static constexpr hw_reg_w<uint8_t,   const_addr<0x81300>> berclr = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x81304>> beren = { };
  static constexpr hw_reg_rw<uint8_t,  const_addr<0x81308>> bersr1 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x8130A>> bersr2 = { };

  static constexpr hw_reg_rw<uint16_t, const_addr<0x81310>> buspri = { };

};

} // namespace dev

#endif // includeguard_dev_rx_bsc_hpp_includeguard
