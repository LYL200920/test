/*

GD32F1 I2C registers and I2C master implementation

*/


#ifndef includeguard_dev_gd322f1_i2c_hpp_includeguard
#define includeguard_dev_gd322f1_i2c_hpp_includeguard

#include <chrono>

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>
#include <dev/i2c.hpp>

namespace dev
{
namespace gd32f1_i2c
{
static constexpr uintptr_t reg_base_i2c0 = 0x40005400;
static constexpr uintptr_t reg_base_i2c1 = 0x40005800;

struct regs_t
{
  hw_reg_rw<uint32_t> ctl0;    // 0x00
  hw_reg_rw<uint32_t> ctl1;    // 0x04
  hw_reg_rw<uint32_t> saddr0;  // 0x08
  hw_reg_rw<uint32_t> saddr1;  // 0x0C
  hw_reg_rw<uint32_t> data;    // 0x10
  hw_reg_rw<uint32_t> stat0;   // 0x14
  hw_reg_rw<uint32_t> stat1;   // 0x18
  hw_reg_rw<uint32_t> ckcfg;   // 0x1C
  hw_reg_rw<uint32_t> rt;      // 0x20
};

enum smbus_type_t
{
  smbus_device = 0,
  smbus_host = 1
};

enum bus_mode_t
{
  mode_i2c = 0,
  mode_smbus = 1
};

class ctl0_t
{
public:
  constexpr ctl0_t (void) : m_value (0) { }
  constexpr ctl0_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool soft_reset (void) const { return utils::get_bit (m_value, 15); }
  ctl0_t& set_soft_reset (bool val) { m_value = utils::set_bit (m_value, 15, val); return *this; }

  constexpr bool smbus_alert (void) const { return utils::get_bit (m_value, 13); }
  ctl0_t& set_smbus_alert (bool val) { m_value = utils::set_bit (m_value, 13, val); return *this; }

  constexpr bool pec_transfer (void) const { return utils::get_bit (m_value, 12); }
  ctl0_t& set_pec_transfer (bool val) { m_value = utils::set_bit (m_value, 12, val); return *this; }

  constexpr bool ack_pec_after_rx (void) const { return utils::get_bit (m_value, 11); }
  ctl0_t& set_ack_pec_after_rx (bool val) { m_value = utils::set_bit (m_value, 11, val); return *this; }

  constexpr bool ack_enable (void) const { return utils::get_bit (m_value, 10); }
  ctl0_t& set_ack_enable (bool val) { m_value = utils::set_bit (m_value, 10, val); return *this; }

  constexpr bool gen_stop (void) const { return utils::get_bit (m_value, 9); }
  ctl0_t& set_gen_stop (bool val) { m_value = utils::set_bit (m_value, 9, val); return *this; }

  constexpr bool gen_start (void) const { return utils::get_bit (m_value, 8); }
  ctl0_t& set_gen_start (bool val) { m_value = utils::set_bit (m_value, 8, val); return *this; }

  constexpr bool scl_stretch_enable (void) const { return utils::get_bit (m_value, 7); }
  ctl0_t& set_scl_stretch_enable (bool val) { m_value = utils::set_bit (m_value, 7, val); return *this; }

  constexpr bool general_call_enable (void) const { return utils::get_bit (m_value, 6); }
  ctl0_t& set_general_call_enable (bool val) { m_value = utils::set_bit (m_value, 6, val); return *this; }

  constexpr bool pec_enable (void) const { return utils::get_bit (m_value, 5); }
  ctl0_t& set_pec_enable (bool val) { m_value = utils::set_bit (m_value, 5, val); return *this; }

  constexpr bool arp_enable (void) const { return utils::get_bit (m_value, 4); }
  ctl0_t& set_arp_enable (bool val) { m_value = utils::set_bit (m_value, 4, val); return *this; }

  constexpr smbus_type_t smbus_type (void) const { return (smbus_type_t)utils::get_bit (m_value, 3); }
  ctl0_t& set_smbus_type (smbus_type_t val) { m_value = utils::set_bit (m_value, 3, (bool)val); return *this; }

  constexpr bus_mode_t bus_mode (void) const { return (bus_mode_t)utils::get_bit (m_value, 1); }
  ctl0_t& set_bus_mode (bus_mode_t val) { m_value = utils::set_bit (m_value, 1, (bool)val); return *this; }

  constexpr bool peripheral_enable (void) const { return utils::get_bit (m_value, 0); }
  ctl0_t& set_peripheral_enable (bool val) { m_value = utils::set_bit (m_value, 0, val); return *this; }

private:
  uint32_t m_value;
};


class ctl1_t
{
public:
  constexpr ctl1_t (void) : m_value (0) { }
  constexpr ctl1_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool dma_last_transfer (void) const { return utils::get_bit (m_value, 12); }
  ctl1_t& set_dma_last_transfer (bool val) { m_value = utils::set_bit (m_value, 12, val); return *this; }

  constexpr bool dma_enable (void) const { return utils::get_bit (m_value, 11); }
  ctl1_t& set_dma_enable (bool val) { m_value = utils::set_bit (m_value, 11, val); return *this; }

  constexpr bool buffer_interrupt_enable (void) const { return utils::get_bit (m_value, 10); }
  ctl1_t& set_buffer_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 10, val); return *this; }

  constexpr bool event_interrupt_enable (void) const { return utils::get_bit (m_value, 9); }
  ctl1_t& set_event_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 9, val); return *this; }

  constexpr bool error_interrupt_enable (void) const { return utils::get_bit (m_value, 8); }
  ctl1_t& set_error_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 8, val); return *this; }

  unsigned int peripheral_clock (void) const { return m_value & 127u; }
  ctl1_t& set_peripheral_clock (unsigned int val) { m_value = utils::merge_bits (m_value, (uint32_t)val, (uint32_t)127); return *this; }

private:
  uint32_t m_value;
};


enum address_format_t
{
  address_7bit = 0,
  address_10bit = 1
};

class saddr0_t
{
public:
  constexpr saddr0_t (void) : m_value (0) { }
  constexpr saddr0_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr address_format_t address_format (void) const { return (address_format_t)utils::get_bit (m_value, 15); }
  saddr0_t& set_address_format (address_format_t val) { m_value = utils::set_bit (m_value, 15, (bool)val); return *this; }

  constexpr unsigned int address (void) const { return m_value & 1023u; }
  saddr0_t& set_address (unsigned int val) { m_value = (m_value & ~((uint32_t)1023u)) | (val & 1023u); return *this; }
  
private:
  uint32_t m_value;
};


class saddr1_t
{
public:
  constexpr saddr1_t (void) : m_value (0) { }
  constexpr saddr1_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr unsigned int second_address (void) const { return (m_value >> 1) & 127; }
  saddr1_t& set_second_address (unsigned int val) { m_value = (m_value & ~((uint32_t)127 << 1)) | ((val & 127) << 1); return *this; }

  constexpr bool dual_address_enable (void) const { return utils::get_bit (m_value, 0); }
  saddr1_t& set_dual_address_enable (bool val) { m_value = utils::set_bit (m_value, 0, val); return *this; }

private:
  uint32_t m_value;
};

class stat0_t
{
public:
  constexpr stat0_t (void) : m_value (0) { }
  constexpr stat0_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool smbus_alert (void) const { return utils::get_bit (m_value, 15); }
  stat0_t& set_smbus_alert (bool val) { m_value = utils::set_bit (m_value, 15, val); return *this; }

  constexpr bool smbus_timeout (void) const { return utils::get_bit (m_value, 14); }
  stat0_t& set_smbus_timeout (bool val) { m_value = utils::set_bit (m_value, 14, val); return *this; }

  constexpr bool pec_error (void) const { return utils::get_bit (m_value, 12); }
  stat0_t& set_pec_error (bool val) { m_value = utils::set_bit (m_value, 12, val); return *this; }

  constexpr bool overrun_error (void) const { return utils::get_bit (m_value, 11); }
  stat0_t& set_overrun_error (bool val) { m_value = utils::set_bit (m_value, 11, val); return *this; }

  constexpr bool ack_error (void) const { return utils::get_bit (m_value, 10); }
  stat0_t& set_ack_error (bool val) { m_value = utils::set_bit (m_value, 10, val); return *this; }

  constexpr bool arbitration_lost (void) const { return utils::get_bit (m_value, 9); }
  stat0_t& set_arbitration_lost (bool val) { m_value = utils::set_bit (m_value, 9, val); return *this; }

  constexpr bool bus_error (void) const { return utils::get_bit (m_value, 8); }
  stat0_t& set_bus_error (bool val) { m_value = utils::set_bit (m_value, 8, val); return *this; }

  constexpr bool transmit_byte_empty (void) const { return utils::get_bit (m_value, 7); }
  stat0_t& set_transmit_byte_empty (bool val) { m_value = utils::set_bit (m_value, 7, val); return *this; }

  constexpr bool receive_byte_not_empty (void) const { return utils::get_bit (m_value, 6); }
  stat0_t& set_receive_byte_not_empty (bool val) { m_value = utils::set_bit (m_value, 6, val); return *this; }

  constexpr bool stop_condition_detected (void) const { return utils::get_bit (m_value, 4); }
  stat0_t& set_stop_condition_detected (bool val) { m_value = utils::set_bit (m_value, 4, val); return *this; }

  constexpr bool sending_10_bit_addr_header (void) const { return utils::get_bit (m_value, 3); }
  stat0_t& set_sending_10_bit_addr_header (bool val) { m_value = utils::set_bit (m_value, 3, val); return *this; }

  constexpr bool byte_transmission_completed (void) const { return utils::get_bit (m_value, 2); }
  stat0_t& set_byte_transmission_completed (bool val) { m_value = utils::set_bit (m_value, 2, val); return *this; }

  constexpr bool address_sent_received (void) const { return utils::get_bit (m_value, 1); }
  stat0_t& set_address_sent_received (bool val) { m_value = utils::set_bit (m_value, 1, val); return *this; }

  constexpr bool sending_start_condition (void) const { return utils::get_bit (m_value, 0); }
  stat0_t& set_sending_start_condition (bool val) { m_value = utils::set_bit (m_value, 0, val); return *this; }

private:
  uint32_t m_value;
};

enum rx_tx_role_t
{
  role_receiver = 0,
  role_transmitter = 1
};

enum bus_role_t
{
  role_slave = 0,
  role_master = 1
};

class stat1_t
{
public:
  constexpr stat1_t (void) : m_value (0) { }
  constexpr stat1_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr unsigned int pec_value (void) const { return (m_value >> 8) & 0xFF; }
  constexpr bool dual_flag (void) const { return utils::get_bit (m_value, 7); }
  constexpr bool smbus_host_header_detected (void) const { return utils::get_bit (m_value, 6); }
  constexpr bool default_smbus_device_address_received (void) const { return utils::get_bit (m_value, 5); }
  constexpr bool general_call_address_received (void) const { return utils::get_bit (m_value, 4); }
  constexpr rx_tx_role_t rx_tx_role (void) const { return (rx_tx_role_t)utils::get_bit (m_value, 2); }
  constexpr bool busy (void) const { return utils::get_bit (m_value, 1); }
  constexpr bus_role_t bus_role (void) const { return (bus_role_t)utils::get_bit (m_value, 0); }

private:
  uint32_t m_value;
};

enum speed_mode_t
{
  standard_speed = 0,
  fast_speed = 1
};

enum duty_cycle_mode_t
{
  duty_2 = 0,
  duty_16_9 = 1
};

class ckcfg_t
{
public:
  constexpr ckcfg_t (void) : m_value (0) { }
  constexpr ckcfg_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr speed_mode_t speed_mode (void) const { return (speed_mode_t)utils::get_bit (m_value, 15); }
  ckcfg_t& set_speed_mode (speed_mode_t val) { m_value = utils::set_bit (m_value, 15, (bool)val); return *this; }

  constexpr duty_cycle_mode_t duty_cylce (void) const { return (duty_cycle_mode_t)utils::get_bit (m_value, 14); }
  ckcfg_t& set_duty_cycle (duty_cycle_mode_t val) { m_value = utils::set_bit (m_value, 14, (bool)val); return *this; }

  constexpr unsigned int clock_ctrl (void) const { return m_value & 4095u; }
  ckcfg_t& set_clock_ctrl (unsigned int val) { m_value = (m_value & ~(uint32_t)4095) | (val & 4095); return *this; }

private:
  uint32_t m_value;
};



template<uint32_t RegBaseAddress,
	 unsigned int PeripheralClockHz,
	 unsigned int I2cCLockHz = 400'000>
class i2c_master : public dev::i2c_master
{
public:
  i2c_master (void)
  {
    set_ctl0 (ctl0 ()
	.set_soft_reset (false)
	.set_smbus_alert (false)
	.set_pec_transfer (false)
	.set_ack_pec_after_rx (false)
	.set_ack_enable (false)
	.set_gen_stop (false)
	.set_gen_start (false)
	.set_scl_stretch_enable (false)
	.set_general_call_enable (false)
	.set_pec_enable (false)
	.set_arp_enable (false)
	.set_smbus_type (smbus_device)
	.set_bus_mode (mode_i2c)
	.set_peripheral_enable (true)
    );

    set_ctl1 (ctl1 ()
    	.set_dma_last_transfer (false)
    	.set_dma_enable (false)
    	.set_buffer_interrupt_enable (false)
    	.set_event_interrupt_enable (false)
    	.set_error_interrupt_enable (false)
    	.set_peripheral_clock (PeripheralClockHz / 1'000'000)
    );

    set_ckcfg (ckcfg ()
    	.set_speed_mode (fast_speed)
    	.set_duty_cycle (duty_2)

    	// T(high) = CLKC*T(PCLK1) , T(low) = 2*CLKC*T(PCLK1)
    	.set_clock_ctrl ((PeripheralClockHz / I2cCLockHz)/3)
    );
  }

  virtual dev::i2c_device_id query_device_id (uint8_t slave_addr) override
  {
    return { };
  }

  virtual bool
  send (uint8_t slave_addr, const void* tx_data, unsigned int count_bytes) override
  {
    const uint8_t* tx_data_u8 = (const uint8_t*)tx_data;
    bool ret = true;

    if (!send_start_condition ())
      goto return_error;

    if (!send_address (slave_addr))
      goto return_error;

    while (count_bytes > 0)
    {
      auto s0 = stat0 ();
      //s1 = stat1 ();
      //debug_printf ("%d  st0: %d   st1: %d\n", count_bytes, s0.value (), s1.value ());

      if (s0.transmit_byte_empty ())
      {
        set_data (*tx_data_u8++);
        --count_bytes;
      }

      if (s0.ack_error () | s0.bus_error () | s0.arbitration_lost ())
        goto return_error;
    }

    goto return_ok;


return_error:
    set_stat0 (stat0 ()
       .set_ack_error (false)
       .set_bus_error (false)
       .set_arbitration_lost (false)
    );
    ret = false;

return_ok:
    send_stop_condition ();
    return ret;
  }

  using dev::i2c_master::send;


  virtual bool
  recv (uint8_t slave_addr, void* rx_data, unsigned int count_bytes) override
  {
    slave_addr |= 1; // make sure to set the read bit of the address.

    uint8_t* rx_data_u8 = (uint8_t*)rx_data;
    bool ret = true;
    bool stop_generated = false;

    set_ctl0 (ctl0 ()
    	.set_ack_pec_after_rx (count_bytes == 2)
      	.set_ack_enable (true)
    );

    if (!send_start_condition ())
      goto return_error;

    if (!send_address (slave_addr, count_bytes > 2))
      goto return_error;


    if (count_bytes == 1)
    {
      stop_generated = true;
      set_ctl0 (ctl0 ().set_gen_stop (true));
    }

    for (auto start_time = std::chrono::high_resolution_clock::now (); count_bytes > 0;)
    {
      auto s0 = stat0 ();

      if (s0.receive_byte_not_empty ())
      {
        *rx_data_u8++ = data ();
        --count_bytes;

        if (count_bytes == 1)
          set_ctl0 (ctl0 ().set_ack_enable (false));

        start_time = std::chrono::high_resolution_clock::now ();
      }
      else if (std::chrono::high_resolution_clock::now () - start_time
               > std::chrono::milliseconds (10))
      {
        //debug_printf ("timeout  %d received\n", rx_byte_count);
        goto return_error;
      }
    }

    if (!stop_generated)
      set_ctl0 (ctl0 ().set_gen_stop (true));

    goto return_ok;

return_error:
    set_stat0 (stat0 ()
       .set_ack_error (false)
       .set_bus_error (false)
       .set_arbitration_lost (false)
    );
    ret = false;

return_ok:
    while (stat1 ().busy ()) { }

    return ret;
  }

  using dev::i2c_master::recv;
  using dev::i2c_master::send_recv;


private:
  bool send_start_condition (void)
  {
    set_stat0 (stat0 ()
       .set_ack_error (false)
       .set_bus_error (false)
       .set_arbitration_lost (false)
    );

    set_ctl0 (ctl0 ().set_gen_start (true));

    for (auto start_time = std::chrono::high_resolution_clock::now (); ;)
    {
      auto s0 = stat0 ();
      [[maybe_unused]] auto s1 = stat1 ();

      //debug_printf ("st0: %d   st1: %d\n", s0.value (), s1.value ());

      if (s0.sending_start_condition ())
        return true;

      if (s0.ack_error () | s0.bus_error () | s0.arbitration_lost ())
      {
        //debug_printf ("bus error\n");
        return false;
      }

      if (std::chrono::high_resolution_clock::now () - start_time
      	  > std::chrono::milliseconds (10))
      {
      	//debug_printf ("timeout\n");
      	return false;
      }
    }
  }

  bool send_stop_condition (void)
  {
    set_ctl0 (ctl0 ().set_gen_stop (true));

    for (auto start_time = std::chrono::high_resolution_clock::now (); ;)
    {
      auto s0 = stat0 ();
      auto s1 = stat1 ();
      //debug_printf ("st0: %d   st1: %d\n", s0.value (), s1.value ());

      if (!s1.busy ())
        return true;

      if (s0.ack_error () | s0.bus_error () | s0.arbitration_lost ())
      	return false;

      if (std::chrono::high_resolution_clock::now () - start_time
      	  > std::chrono::milliseconds (10))
      {
      	//debug_printf ("timeout\n");
      	return false;
      }
    }
  }

  bool send_address (uint8_t val, bool next_ack_enable = true)
  {
    set_data (val);

    for (auto start_time = std::chrono::high_resolution_clock::now (); ;)
    {
      if (!next_ack_enable)
        set_ctl0 (ctl0 ().set_ack_enable (false));

      // need to read stat0 + stat1 to clear the "address sent received flag"
      auto s0 = stat0 ();
      [[maybe_unused]] auto s1 = stat1 ();
      //debug_printf ("st0: %d   st1: %d\n", s0.value (), s1.value ());

      if (s0.address_sent_received ())
        return true;

      if (s0.ack_error () | s0.bus_error () | s0.arbitration_lost ())
      {
        //debug_printf ("ack error\n");
        return false;
      }

      if (std::chrono::high_resolution_clock::now () - start_time
      	  > std::chrono::milliseconds (10))
      {
      	//debug_printf ("timeout\n");
      	//goto i2cm_error_timeout;
      	return false;
      }
    }
  }

  const regs_t& regs (void) const { return *(const regs_t*)RegBaseAddress; }
  regs_t& regs (void) { return *(regs_t*)RegBaseAddress; }

  ctl0_t ctl0 (void) { return { regs ().ctl0 }; }
  void set_ctl0 (const ctl0_t& val) { regs ().ctl0 = val.value (); }

  ctl1_t ctl1 (void) { return { regs ().ctl1 }; }
  void set_ctl1 (const ctl1_t& val) { regs ().ctl1 = val.value (); }

  saddr0_t saddr0 (void) { return { regs ().saddr0 }; }
  void set_saddr0 (const saddr0_t& val) { regs ().saddr0 = val.value (); }

  saddr1_t saddr1 (void) { return { regs ().saddr1 }; }
  void set_saddr1 (const saddr1_t& val) { regs ().saddr1 = val.value (); }

  uint8_t data (void) { return regs ().data; }
  void set_data (uint8_t val) { regs ().data = val; }

  stat0_t stat0 (void) { return { regs ().stat0 }; }
  void set_stat0 (const stat0_t& val) { regs ().stat0 = val.value  (); }

  stat1_t stat1 (void) { return { regs ().stat1 }; }

  ckcfg_t ckcfg (void) { return { regs ().ckcfg }; }
  void set_ckcfg (const ckcfg_t& val) { regs ().ckcfg = val.value (); }

  unsigned int rt (void) { return regs ().rt & 127u; }
  void set_rt (unsigned int val) { regs ().rt = val & 127u; }
};


} // namespace gd32f1_i2c
} // namespace dev

#endif // includeguard_dev_gd322f1_i2c_hpp_includeguard