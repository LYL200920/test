/*

various clock related things for the renesas RX MCU series.

*/

#ifndef includeguard_dev_rx_clk_hpp_includeguard
#define includeguard_dev_rx_clk_hpp_includeguard

#include <dev/hwreg.hpp>

namespace dev
{
struct rx_clk
{

  static constexpr unsigned int sckcr_div_invalid (void) { return 0b1111; }
  static constexpr unsigned int sckcr_div (unsigned int pll_over_clk_ratio)
  {
    switch (pll_over_clk_ratio)
    {
      case 1: return 0b0000;
      case 2: return 0b0001;
      case 4: return 0b0010;
      case 8: return 0b0011;
      case 16: return 0b0100;
      case 32: return 0b0101;
      case 64: return 0b0110;
      default: return sckcr_div_invalid ();
    }
  }


  static constexpr unsigned int sckcr_uck_div_invalid (void) { return 0b1111; }

  static constexpr unsigned int sckcr_uck_div_rx63 (unsigned int pll_over_uck_ratio)
  {
    switch (pll_over_uck_ratio)
    {
      case 3: return 0b0010;
      case 4: return 0b0011;
      default: return sckcr_uck_div_invalid ();
    }
  }

  static constexpr unsigned int sckcr_iebck_div_invalid (void) { return 0b1111; }

  static constexpr unsigned int sckcr_iebck_div_rx63 (unsigned int pll_over_iebck_ratio)
  {
    switch (pll_over_iebck_ratio)
    {
      case 2: return 0b0001;
      case 4: return 0b0010;
      case 8: return 0b0011;
      case 16: return 0b0100;
      case 32: return 0b0101;
      case 64: return 0b0110;
      case 6: return 0b1100;
      default: return sckcr_iebck_div_invalid ();
    }
  }

  static constexpr unsigned int sckcr_uck_div_rx64 (unsigned int pll_over_uck_ratio)
  {
    switch (pll_over_uck_ratio)
    {
      case 2: return 0b0001;
      case 3: return 0b0010;
      case 4: return 0b0011;
      case 5: return 0b0100;
      default: return sckcr_uck_div_invalid ();
    }
  }

  static constexpr bool pllcr_stc_valid_rx63 (unsigned int stc_bits_val)
  {
    switch (stc_bits_val)
    {
      case 0b000111: return true;  // x8
      case 0b001001: return true;  // x10
      case 0b001011: return true;  // x12
      case 0b001111: return true;  // x16
      case 0b010011: return true;  // x20
      case 0b010111: return true;  // x24
      case 0b011000: return true;  // x25
      case 0b110001: return true;  // x50
      default: return false;
    }
  }

  static constexpr bool pll_freq_valid_rx63 (unsigned int freq_hz)
  {
    return freq_hz >= 104'000'000 && freq_hz <= 200'000'000;
  }

  static constexpr bool ppcr_stc_valid_rx64 (unsigned int stc_bits_val)
  {
    switch (stc_bits_val)
    {
      case 0b010011: return true;  // x10.0
      case 0b010100: return true;  // x10.5
      case 0b010101: return true;  // x11.0
      case 0b010110: return true;  // x11.5
      case 0b010111: return true;  // x12.0
      case 0b011000: return true;  // x12.5
      case 0b011001: return true;  // x13.0
      case 0b011010: return true;  // x13.5
      case 0b011011: return true;  // x14.0
      case 0b011100: return true;  // x14.5
      case 0b011101: return true;  // x15.0
      case 0b011110: return true;  // x15.5
      case 0b011111: return true;  // x16.0
      case 0b100000: return true;  // x16.5
      case 0b100001: return true;  // x17.0
      case 0b100010: return true;  // x17.5
      case 0b100011: return true;  // x18.0
      case 0b100100: return true;  // x18.5
      case 0b100101: return true;  // x19.0
      case 0b100110: return true;  // x19.5
      case 0b100111: return true;  // x20.0
      case 0b101000: return true;  // x20.5
      case 0b101001: return true;  // x21.0
      case 0b101010: return true;  // x21.5
      case 0b101011: return true;  // x22.0
      case 0b101100: return true;  // x22.5
      case 0b101101: return true;  // x23.0
      case 0b101110: return true;  // x23.5
      case 0b101111: return true;  // x24.0
      case 0b110000: return true;  // x24.5
      case 0b110001: return true;  // x25.0
      case 0b110010: return true;  // x25.5
      case 0b110011: return true;  // x26.0
      case 0b110100: return true;  // x26.5
      case 0b110101: return true;  // x27.0
      case 0b110110: return true;  // x27.5
      case 0b110111: return true;  // x28.0
      case 0b111000: return true;  // x28.5
      case 0b111001: return true;  // x29.0
      case 0b111010: return true;  // x29.5
      case 0b111011: return true;  // x30.0
      default: return false;
    }
  }

  static constexpr bool pll_freq_valid_rx64 (unsigned int freq_hz)
  {
    return freq_hz >= 120'000'000 && freq_hz <= 240'000'000;
  }


  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x00080020>> sckcr = { };
  static constexpr dev::hw_reg_rw<uint32_t, dev::const_addr<0x00086610>> memwait = { };
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x00080024>> sckcr2 = { };
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x00080026>> sckcr3 = { };
  static constexpr dev::hw_reg_rw<uint16_t, dev::const_addr<0x00080028>> pllcr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008002A>> pllcr2 = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080030>> bckcr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080032>> mosccr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080033>> sosccr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080034>> lococr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080035>> ilococr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080036>> hococr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080037>> hococr2 = { };
  static constexpr dev::hw_reg_r<uint8_t, dev::const_addr<0x0008003C>> oscovfsr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080040>> ostdcr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080041>> ostdsr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080042>> moscwtcr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00080043>> soscwtcr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C293>> mofcr = { };
  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x0008C293>> hocopcr = { };

  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x000800A6>> pllwtcr = { }; // RX63

}; // struct rx_clk

} // namespace dev
#endif // includeguard_dev_rx_clk_hpp_includeguard
