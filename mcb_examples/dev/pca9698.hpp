/*

  PCA9698 I2C bus expander

  the driver caches the output ports status to allow fast output read-back
  and update of partial bytes.
*/

#ifndef includeguard_dev_pca9698_includeguard
#define includeguard_dev_pca9698_includeguard

#include <cstdint>
#include <cstring>
#include <bitset>
#include <type_traits>
#include <dev/i2c.hpp>
#include <dev/interrupt.hpp>
#include <utils/bits.hpp>

namespace dev
{

template < unsigned int SlaveAddr, unsigned int MaxBaudrate,
	   typename InterruptLine >
class pca9698
{
public:
  static constexpr unsigned int i2c_manufacturer_id = dev::i2c_manufacturer_id::nxp_semiconductrors;
  static constexpr unsigned int i2c_device_id = 0b000000000;

  pca9698 (dev::i2c_master& d) : m_i2c (&d), m_isr (this)
  {
    // get the current values of the output ports.
    const uint8_t wr_data[] = { 0x08 | reg_auto_inc };
    m_i2c->send_recv (SlaveAddr, wr_data, m_output_ports_shadow);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // inputs

  // read all inputs from the specified bank.
  template <unsigned int N> std::bitset<8> input_bank (void) const
  {
    static_assert (N <= 4, "");
    return { read_reg (0x00 + N) };
  }

  // read contiguous range of input ports, including the first port,
  // but excluding the last port.
  // e.g. reading ports <0, 40> will read all 40 ports, not 41 ports.
  template <unsigned int N0, unsigned int N1> std::bitset<N1 - N0>
  input_ports (void) const
  {
    static_assert (N0 <= 40, "");
    static_assert (N1 <= 40, "");
    static_assert (N1 - N0 > 0, "");
    static_assert (N1 > N0, "");

    constexpr unsigned int read_bit_count = N1 - N0;
    constexpr unsigned int read_byte_count = (N1-1)/8 - (N0/8) + 1;
    constexpr unsigned int read_byte_offset = N0 / 8;
    constexpr unsigned int bit_shift_offset = N0 % 8;

    static_assert (read_byte_count > 0, "");

    const uint8_t wr_data[] = { (0x00 + read_byte_offset) | reg_auto_inc };
    uint8_t rd_data[read_byte_count];
    m_i2c->send_recv (SlaveAddr, wr_data, rd_data);

    return extract_bits<read_bit_count, bit_shift_offset, read_byte_count> (rd_data);
  }

  template <unsigned int N> std::bitset<8> input_bank_inversion (void) const
  {
    static_assert (N <= 4, "");
    return { read_reg (0x10 + N) };
  }

  template <unsigned int N> void set_input_bank_inversion (std::bitset<8> val)
  {
    static_assert (N <= 4, "");
    write_reg (0x10 + N, (uint8_t)val.to_ulong ());
  }

  template <unsigned int N> std::bitset<8> input_bank_imask (void) const
  {
    static_assert (N <= 4, "");
    return { read_reg (0x20 + N) };
  }

  template <unsigned int N> void set_input_bank_imask (std::bitset<8> val)
  {
    static_assert (N <= 4, "");
    write_reg (0x20 + N, (uint8_t)val.to_ulong ());
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // outputs

  // read all outputs of the specified bank.
  template <unsigned int N> std::bitset<8> output_bank (void) const
  {
    static_assert (N <= 4, "");
    return m_output_ports_shadow[N];
  }

  // write all outputs of the specified bank.
  template <unsigned int N> void set_output_bank (std::bitset<8> val)
  {
    static_assert (N <= 4, "");
    m_output_ports_shadow[N] = (uint8_t)val.to_ulong ();
    write_reg (0x08 + N, (uint8_t)val.to_ulong ());
  }

  // read contiguous range of outputs, including the first port,
  // exlucding the last port.
  template <unsigned int N0, unsigned int N1> std::bitset<N1 - N0>
  output_ports (void) const
  {
    static_assert (N0 <= 40, "");
    static_assert (N1 <= 40, "");
    static_assert (N1 - N0 > 0, "");
    static_assert (N1 > N0, "");

    constexpr unsigned int bit_count = N1 - N0;
    constexpr unsigned int byte_count = (N1-1)/8 - (N0/8) + 1;
    constexpr unsigned int byte_offset = N0 / 8;
    constexpr unsigned int bit_shift_offset = N0 % 8;

    static_assert (byte_count > 0, "");
    return extract_bits<bit_count, bit_shift_offset, byte_count> (
					m_output_ports_shadow + byte_offset);
  }

  // write contiguous range of outputs, including the first port, excluding the
  // last port.  this works for arbitrary bit positions even if they are not
  // byte aligned.  however, because it requires merging bits with the current
  // output state it's a bit slower.
  template <unsigned int N0, unsigned int N1> void
  set_output_ports (std::bitset<N1 - N0> val)
  {
    static_assert (N0 <= 40, "");
    static_assert (N1 <= 40, "");
    static_assert (N1 - N0 > 0, "");
    static_assert (N1 > N0, "");

    constexpr unsigned int first_byte_offset = (N0/8);
    constexpr unsigned int last_byte_offset = (N1-1)/8;
    constexpr unsigned int byte_count = last_byte_offset - first_byte_offset + 1;
    constexpr unsigned int bit_shift_offset = N0 % 8;

    if ((N0 | N1) & 7)
    {
      // we're not overwriting whole bytes.  so get the current output state
      // and combine with the new bits.

      // note: clamp N1 to 40 because it is invoked recursively and although
      // the path is never executed, the compiler will instantate the code first.
      auto cur_out_bits = output_ports< (N0/8)*8, std::min ((N1/8)*8 + 8, 40u) > ();

      // widen the input bitset so that it is byte-aligned and shift it into place.
      auto new_out_bits = decltype (cur_out_bits) (
					val.to_ullong ()) << bit_shift_offset;

      // construct the bitmask which has '1' for the new bits
      auto new_out_mask = decltype (cur_out_bits) (
			decltype (val)().set ().to_ullong ()) << bit_shift_offset;

      // overwrite bits, for which there is a '1' in the mask and keep the other bits
      set_output_ports < (N0/8)*8, std::min ((N1/8)*8 + 8, 40u) > (
		utils::merge_bits (cur_out_bits, new_out_bits, new_out_mask));
    }
    else
    {
      // overwriting whole bytes.
      // note: the if-else will be actually evaluated at compile time

      // update the bytes in the output shadow register
      if (byte_count == 1)
        m_output_ports_shadow[first_byte_offset + 0] = (uint8_t)val.to_ullong ();

      else if (byte_count == 2)
      {
        m_output_ports_shadow[first_byte_offset + 0] = (uint8_t)(val.to_ullong () >> 0);
        m_output_ports_shadow[first_byte_offset + 1] = (uint8_t)(val.to_ullong () >> 8);
      }
      else if (byte_count == 3)
      {
        m_output_ports_shadow[first_byte_offset + 0] = (uint8_t)(val.to_ullong () >> 0);
        m_output_ports_shadow[first_byte_offset + 1] = (uint8_t)(val.to_ullong () >> 8);
        m_output_ports_shadow[first_byte_offset + 2] = (uint8_t)(val.to_ullong () >> 16);
      }
      else if (byte_count == 4)
      {
        m_output_ports_shadow[first_byte_offset + 0] = (uint8_t)(val.to_ullong () >> 0);
        m_output_ports_shadow[first_byte_offset + 1] = (uint8_t)(val.to_ullong () >> 8);
        m_output_ports_shadow[first_byte_offset + 2] = (uint8_t)(val.to_ullong () >> 16);
        m_output_ports_shadow[first_byte_offset + 3] = (uint8_t)(val.to_ullong () >> 24);
      }
      else if (byte_count == 5)
      {
        m_output_ports_shadow[first_byte_offset + 0] = (uint8_t)(val.to_ullong () >> 0);
        m_output_ports_shadow[first_byte_offset + 1] = (uint8_t)(val.to_ullong () >> 8);
        m_output_ports_shadow[first_byte_offset + 2] = (uint8_t)(val.to_ullong () >> 16);
        m_output_ports_shadow[first_byte_offset + 3] = (uint8_t)(val.to_ullong () >> 24);
        m_output_ports_shadow[first_byte_offset + 4] = (uint8_t)(val.to_ullong () >> 32);
      }

      // synchronize the hardware registers with the shadow registers
      uint8_t wr_data[byte_count + 1];
      wr_data[0] = (0x08 + first_byte_offset) | reg_auto_inc;

      std::memcpy (wr_data + 1, m_output_ports_shadow + first_byte_offset, byte_count);

      m_i2c->send (SlaveAddr, wr_data);
    }
  }


  class all_outputs_ctrl_t
  {
  public:
    constexpr all_outputs_ctrl_t (void) : m_value (0) { }
    constexpr explicit all_outputs_ctrl_t (uint8_t val) : m_value (val) { }
    constexpr uint8_t value (void) const { return m_value; }

    // the bank selection controls whether a bank will have all its outputs
    // set to 0 or 1 or will continue using the output values from the output
    // register.
    // a '1' in the selection bits means use the 0/1 value
    // a '0' means use the output register value.
    std::bitset<5> bank_select (void) const
    {
      std::bitset<5> r (m_value & 0b11111);
      if (get_bit (7) == false)
        r = ~r;
      return r;
    }

    all_outputs_ctrl_t& set_bank_select (std::bitset<5> val)
    {
      if (get_bit (7) == false)
        val = ~val;
      m_value = (m_value & 0b10000000) | val.to_ulong ();
      return *this;
    }

    // set the output value for those banks which have been selected.
    bool output_value (void) const { return get_bit (7); }
    all_outputs_ctrl_t& set_output_value (bool val)
    {
      // depending on the new BSEL bit we might need to invert the bank
      // selection mask...
      auto sel = bank_select ();
      set_bit (7, val);
      set_bank_select (sel);
      return *this;
    }

  private:
    uint8_t m_value;

    constexpr bool get_bit (unsigned int pos) const { return m_value & (1 << pos); }

    constexpr all_outputs_ctrl_t& set_bit (unsigned int pos, bool val)
    {
      m_value = (m_value & ~(1<<pos)) | (val << pos);
      return *this;
    }
  };

  all_outputs_ctrl_t all_outputs_ctrl (void) const { return all_outputs_ctrl_t (read_reg (0x29)); }
  void set_all_outputs_ctrl (all_outputs_ctrl_t val) { write_reg (0x29, val.value ()); }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // IO port configuration
  // set make sure to set these before using the device.
  // after device reset, all IOs are configured as inputs and are open-drained.

  // 0: port pin is an output
  // 1: port pin is an input
  // support only setting the configuration for all ports at once, assuming
  // that this will have to be done during board/device init only once.
  // could also change the configuration of individual blocks
  std::bitset<40> io_ports_config (void) const
  {
    const uint8_t wr_data[] = { 0x18 | reg_auto_inc };
    uint8_t rd_data[] = { 0, 0, 0, 0, 0 };
    m_i2c->send_recv (SlaveAddr, wr_data, rd_data);

    return {   ((unsigned long long)rd_data[0] << 8*0)
	     | ((unsigned long long)rd_data[1] << 8*1)
	     | ((unsigned long long)rd_data[2] << 8*2)
	     | ((unsigned long long)rd_data[3] << 8*3)
	     | ((unsigned long long)rd_data[3] << 8*4) };
  }

  void set_io_ports_config (std::bitset<40> val)
  {
    const uint8_t wr_data[] = { 0x18 | reg_auto_inc,
	(uint8_t)((val.to_ullong () >> 8*0) & 0xFF),
	(uint8_t)((val.to_ullong () >> 8*1) & 0xFF),
	(uint8_t)((val.to_ullong () >> 8*2) & 0xFF),
	(uint8_t)((val.to_ullong () >> 8*3) & 0xFF),
	(uint8_t)((val.to_ullong () >> 8*4) & 0xFF) };
    m_i2c->send (SlaveAddr, wr_data);
  }

  // 0: output port is open-drain
  // 1: output port is totem-pole
  // after hardware reset the default value is all 1's
  class output_config_t
  {
  public:
    constexpr output_config_t (void) : m_value (0xFF) { }
    constexpr explicit output_config_t (uint8_t val) : m_value (val) { }
    constexpr uint8_t value (void) const { return m_value; }

    constexpr bool port_00_01_open_drain (void) const { return !get_bit (0); }
    constexpr output_config_t& set_port_00_01_open_drain (bool val = true) { return set_bit (0, !val); }

    constexpr bool port_02_03_open_drain (void) const { return !get_bit (1); }
    constexpr output_config_t& set_port_02_03_open_drain (bool val = true) { return set_bit (1, !val); }

    constexpr bool port_04_05_open_drain (void) const { return !get_bit (2); }
    constexpr output_config_t& set_port_04_05_open_drain (bool val = true) { return set_bit (2, !val); }

    constexpr bool port_06_07_open_drain (void) const { return !get_bit (3); }
    constexpr output_config_t& set_port_06_07_open_drain (bool val = true) { return set_bit (3, !val); }

    constexpr output_config_t& set_port_0_open_drain (bool val = true) { return set_bit (0, !val), set_bit (1, !val), set_bit (2, !val), set_bit (3, !val); }

    constexpr bool port_1_open_drain (void) const { return !get_bit (4); }
    constexpr output_config_t& set_port_1_open_drain (bool val = true) { return set_bit (4, !val); }

    constexpr bool port_2_open_drain (void) const { return !get_bit (5); }
    constexpr output_config_t& set_port_2_open_drain (bool val = true) { return set_bit (5, !val); }

    constexpr bool port_3_open_drain (void) const { return !get_bit (6); }
    constexpr output_config_t& set_port_3_open_drain (bool val = true) { return set_bit (6, !val); }

    constexpr bool port_4_open_drain (void) const { return !get_bit (7); }
    constexpr output_config_t& set_port_4_open_drain (bool val = true) { return set_bit (7, !val); }

  private:
    uint8_t m_value;

    constexpr bool get_bit (unsigned int pos) const { return m_value & (1 << pos); }

    constexpr output_config_t& set_bit (unsigned int pos, bool val)
    {
      m_value = (m_value & ~(1<<pos)) | (val << pos);
      return *this;
    }
  };

  output_config_t output_config (void) const { return output_config_t (read_reg (0x28)); }
  void set_output_config (output_config_t cfg) { write_reg (0x28, cfg.value ()); }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // (operation) mode selection

  enum oe_pin_mode_t
  {
    active_low = 0,  // default after reset
    active_high = 1
  };

  enum outputs_set_mode_t
  {
    set_on_stop = 0,
    set_on_ack = 1  // default after reset
  };

  class mode_t
  {
  public:
    constexpr mode_t (void) : m_value (0b00000010) { }
    constexpr explicit mode_t (uint8_t val) : m_value (val) { }
    constexpr uint8_t value (void) const { return m_value; }

    // if the !OE pin is tied to some output and can't be controlled, it can
    // be turned on or off here.
    oe_pin_mode_t oe_pin_mode (void) const { return (oe_pin_mode_t)get_bit (0); }
    mode_t& set_oe_pin_mode (oe_pin_mode_t val) { return set_bit (0, val); }

    // if all outputs should switch at the same time this can be used
    // to tell the PCA to switch them after the full I2C multi-register
    // write command has finished.
    outputs_set_mode_t outputs_set_mode (void) const { return (outputs_set_mode_t)get_bit (1); }
    mode_t& set_outputs_set_mode (outputs_set_mode_t val) { return set_bit (1, val); }

    bool gpio_all_call_enable (void) const { return get_bit (3); }
    mode_t& set_gpio_all_call_enable (bool val = true) { return set_bit (3, val); }

    bool smba_alert_enable (void) const { return get_bit (4); }
    mode_t& set_smba_alert_enable (bool val = true) { return set_bit (4, val); }

  private:
    uint8_t m_value;

    constexpr bool get_bit (unsigned int pos) const { return m_value & (1 << pos); }

    constexpr mode_t& set_bit (unsigned int pos, bool val)
    {
      m_value = (m_value & ~(1<<pos)) | (val << pos);
      return *this;
    }
  };

  mode_t mode (void) const { return mode_t (read_reg (0x2A)); }
  void set_mode (mode_t val) { write_reg (0x2A, val.value ()); }

private:
  static constexpr uint8_t reg_auto_inc = 0b1000'0000;

  void isr_func (void)
  {
    // FIXME: some inputs have changed.  issue an i2c read for the bitrange
    // for which interrupts are enabled (not masked).
    // however, because this here is possibly an interrupt context, can't
    // access i2c directly ...
  }

public:
  typedef interrupt::connected_isr<InterruptLine,
	interrupt::func<decltype (&pca9698::isr_func), &pca9698::isr_func>> isr0_t;

private:
  dev::i2c_master* m_i2c;
  uint8_t m_output_ports_shadow[5];
  isr0_t m_isr;


  uint8_t read_reg (uint8_t regno) const
  {
    const uint8_t wr_data[] = { (uint8_t)(regno & ~reg_auto_inc) };
    uint8_t rd_data[] = { 0 };
    m_i2c->send_recv (SlaveAddr, wr_data, rd_data);
    return rd_data[0];
  }

  void write_reg (uint8_t regno, uint8_t val)
  {
    const uint8_t wr_data[] = { (uint8_t)(regno & ~reg_auto_inc), val };
    m_i2c->send (SlaveAddr, wr_data);
  }

  template < unsigned int BitCount, unsigned int BitOffset, unsigned int ByteCount >
  static typename std::enable_if < ByteCount == 1, std::bitset<BitCount> >::type
  extract_bits (const uint8_t* bytes)
  {
    return { (unsigned long long)(bytes[0] >> BitOffset) };
  }

  template < unsigned int BitCount, unsigned int BitOffset, unsigned int ByteCount >
  static typename std::enable_if < ByteCount == 2, std::bitset<BitCount> >::type
  extract_bits (const uint8_t* bytes)
  {
    return { (unsigned long long)(((uint16_t)bytes[0]
				   | ((uint16_t)bytes[1] << 8)) >> BitOffset) };
  }

  template < unsigned int BitCount, unsigned int BitOffset, unsigned int ByteCount >
  static typename std::enable_if < ByteCount == 3, std::bitset<BitCount> >::type
  extract_bits (const uint8_t* bytes)
  {
    return { (unsigned long long)(((uint32_t)bytes[0]
				   | ((uint32_t)bytes[1] << 8)
				   | ((uint32_t)bytes[2] << 16)) >> BitOffset) };
  }

  template < unsigned int BitCount, unsigned int BitOffset, unsigned int ByteCount >
  static typename std::enable_if < ByteCount == 4, std::bitset<BitCount> >::type
  extract_bits (const uint8_t* bytes)
  {
    return { (unsigned long long)(((uint32_t)bytes[0]
				   | ((uint32_t)bytes[1] << 8)
				   | ((uint32_t)bytes[2] << 16)
				   | ((uint32_t)bytes[3] << 24)) >> BitOffset) };
  }

  template < unsigned int BitCount, unsigned int BitOffset, unsigned int ByteCount >
  static typename std::enable_if < ByteCount == 5, std::bitset<BitCount> >::type
  extract_bits (const uint8_t* bytes)
  {
    return { ((uint32_t)bytes[0]
	      | ((uint32_t)bytes[1] << 8)
	      | ((uint32_t)bytes[2] << 16)
	      | ((uint32_t)bytes[3] << 24)
	      | ((uint64_t)bytes[4] << 32)) >> BitOffset };
  }
};

} // namespace dev
#endif // includeguard_dev_pca9698_includeguard
