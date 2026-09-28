
/*

RX63 CRC Calculator (CRC)

*/

#ifndef includeguard_dev_rx63_crc_hpp_includeguard
#define includeguard_dev_rx63_crc_hpp_includeguard

#include <cstddef>
#include <cstdint>

namespace dev
{

class rx63_crc
{
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00080014>> MSTPCRB = { };

public:
  rx63_crc (const rx63_crc&) = delete;
  rx63_crc (rx63_crc&&) = delete;
  rx63_crc& operator = (const rx63_crc&) = delete;
  rx63_crc& operator = (rx63_crc&&) = delete;

  rx63_crc (void)
  {
    // disable module standby
    MSTPCRB &= ~(1u << 23);
  }

  ~rx63_crc (void)
  {
    // enable module standby
    MSTPCRB |= (1u << 23);
  }

  enum crc_type_t
  {
    crc_8_2_1_0 = 1,
    crc_16_15_2_0 = 2,
    crc_16_12_5_0 = 3
  };

  enum crc_bitorder_t
  {
    lsb_first = 0,
    msb_first = 1
  };

  class crccr_t
  {
  public:
    crccr_t (void) : m_value (0) { }
    explicit crccr_t (uint8_t val) : m_value (val) { }

    uint8_t value (void) const { return m_value; }


    crc_type_t crc_type (void) const { return (crc_type_t)(m_value & 0b11); }
    crccr_t& set_crc_type (crc_type_t val) { m_value = (m_value & ~0b11) | val; return *this; }

    crc_bitorder_t bitorder (void) const { return m_value & (1 << 2) ? msb_first : lsb_first; }

    crccr_t& set_bitorder (crc_bitorder_t val)
    {
      m_value = (m_value & ~(1 << 2)) | ((val == msb_first) << 2);
      return *this;
    }

  private:
    uint8_t m_value;
  };

  crccr_t control (void) const
  {
    return crccr_t (*(volatile uint8_t*)0x00088280);
  }

  // notice that setting the control register always automatically clears
  // the crc result register to 0 (DORCLR bit can only be written as 1)
  void set_control (crccr_t val)
  {
    *(volatile uint8_t*)0x00088280 = (val.value () & 0b00000111) | 0b10000000;
  }


  uint8_t data_input (void) const { return *(volatile uint8_t*)0x00088281; }
  void set_data_input (uint8_t val) { *(volatile uint8_t*)0x00088281 = val; }

  uint16_t data_output (void) const { return *(volatile uint16_t*)0x00088282; }
  void set_data_output (uint16_t val) { *(volatile uint16_t*)0x00088282 = val; }
};

} // namespace dev
#endif // includeguard_dev_rx63_crc_hpp_includeguard
