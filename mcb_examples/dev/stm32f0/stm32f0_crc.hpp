
/*

STM32F0xx CRC device

*/

#ifndef includeguard_dev_stm32f0_crc_hpp_includeguard
#define includeguard_dev_stm32f0_crc_hpp_includeguard

#include <cstddef>
#include <cstdint>

namespace dev
{

namespace stm32f0_crc
{

enum output_bitorder_t
{
  output_non_reversed = 0,
  output_reversed = 1
};

enum input_bitorder_t
{
  input_non_reversed = 0b00,
  input_reverse_8 = 0b01,
  input_reverse_16 = 0b10,
  input_reverse_32 = 0b11
};

enum polynomial_size_t
{
  polynomial_32bit = 0b00,  // the only type supported by STM32F03x, STM32F04x, and STM32F05x
  polynomial_16bit = 0b01,
  polynomial_8bit = 0b10,
  polynomial_7bit = 0b11
};

class control_t
{
public:
  constexpr control_t (void) : m_value (0) { }
  constexpr control_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr output_bitorder_t output_bitorder (void) const { return (output_bitorder_t)((m_value >> 7) & 1); }
  constexpr control_t& set_output_bitorder (output_bitorder_t val)
  {
    m_value = (m_value & ~(1 << 7)) | (val << 7);
    return *this;
  }

  constexpr input_bitorder_t input_bitorder (void) const { return (input_bitorder_t)((m_value >> 5) & 0b11); }
  constexpr control_t& set_input_bitorder (input_bitorder_t val)
  {
    m_value = (m_value & ~(0b11 << 5)) | (val << 5);
    return *this;
  }

  constexpr polynomial_size_t polynomial_size (void) const { return (polynomial_size_t)((m_value >> 3) & 0b11); }
  constexpr control_t& set_polynomial_size (polynomial_size_t val)
  {
    m_value = (m_value & ~(0b11 << 3)) | (val << 3);
    return *this;
  }

  constexpr control_t& set_reset (bool val = true)
  {
    m_value = (m_value & ~(1 << 0)) | val;
    return *this;
  }

private:
  uint32_t m_value;
};



template < uintptr_t RegBaseAddr = 0x40023000 >
class hw_inst
{
  static constexpr uintptr_t reg_base_addr = RegBaseAddr;

  // the data register can be accessed with different word sizes
  static constexpr hw_reg_rw<uint32_t, const_addr < reg_base_addr + 0x00 >> dr32 = { };
  static constexpr hw_reg_rw<uint16_t, const_addr < reg_base_addr + 0x00 >> dr16 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr < reg_base_addr + 0x00 >> dr8 = { };

  static constexpr hw_reg_rw<uint32_t, const_addr < reg_base_addr + 0x04 >> idr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr < reg_base_addr + 0x08 >> cr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr < reg_base_addr + 0x0C >> init = { };
  static constexpr hw_reg_rw<uint32_t, const_addr < reg_base_addr + 0x14 >> pol = { };

public:
  hw_inst (void) { }

  control_t control (void) const { return control_t (cr.read ()); }
  void set_control (control_t val) { cr = val.value (); }

  // programmable polynomial is only available on STM32F07x and STM32F09x.
  uint32_t polynomial (void) const { return pol; }
  void set_polynomial (uint32_t val) { pol = val; }

  // the crc remainder value that will be loaded upon reset (through control register)
  uint32_t reset_init_value (void) const { return init; }
  void set_reset_init_value (uint32_t val) { init = val; }

  void write_data (uint8_t val) { dr8 = val; }
  void write_data (uint16_t val) { dr16 = val; }
  void write_data (uint32_t val) { dr32 = val; }

  uint32_t current_value (void) const { return dr32; }

private:
};


} // namespace dev
} // stm32f0_crc

#endif // includeguard_dev_stm32f0_crc_hpp_includeguard
