/*

STM32F0 I2C registers and I2C master implementation

*/


#ifndef includeguard_dev_stm32f0_i2c_hpp_includeguard
#define includeguard_dev_stm32f0_i2c_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>
#include <dev/i2c.hpp>

namespace dev
{
namespace stm32f0_i2c
{
static constexpr uintptr_t reg_base_i2c1 = 0x40005400;
static constexpr uintptr_t reg_base_i2c2 = 0x40005800;

struct regs_t
{
  hw_reg_rw<uint32_t> cr1;       // 0x00
  hw_reg_rw<uint32_t> cr2;       // 0x04
  hw_reg_rw<uint32_t> oar1;      // 0x08
  hw_reg_rw<uint32_t> oar2;      // 0x0C
  hw_reg_rw<uint32_t> timingr;   // 0x10
  hw_reg_rw<uint32_t> timeoutr;  // 0x14
  hw_reg_rw<uint32_t> isr;       // 0x18
  hw_reg_rw<uint32_t> icr;       // 0x1C
  hw_reg_rw<uint32_t> pecr;      // 0x20
  hw_reg_rw<uint32_t> rxdr;      // 0x24
  hw_reg_rw<uint32_t> txdr;      // 0x28
};


class cr1_t
{
public:
  constexpr cr1_t (void) : m_value (0) { }
  constexpr cr1_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool pec_enable (void) const { return utils::get_bit (m_value, 23); }
  cr1_t& set_pec_enable (bool val) { m_value = utils::set_bit (m_value, 23, val); return *this; }

  constexpr bool smbus_alert (void) const { return utils::get_bit (m_value, 22); }
  cr1_t& set_smbus_alert (bool val) { m_value = utils::set_bit (m_value, 22, val); return *this; }

  constexpr bool smbus_default_address_enable (void) const { return utils::get_bit (m_value, 21); }
  cr1_t& set_smbus_default_address_enable (bool val) { m_value = utils::set_bit (m_value, 21, val); return *this; }

  constexpr bool smbus_host_address_enable (void) const { return utils::get_bit (m_value, 20); }
  cr1_t& set_smbus_host_address_enable (bool val) { m_value = utils::set_bit (m_value, 20, val); return *this; }

  constexpr bool general_call_enable (void) const { return utils::get_bit (m_value, 19); }
  cr1_t& set_general_call_enable (bool val) { m_value = utils::set_bit (m_value, 19, val); return *this; }

  constexpr bool wakeup_from_stop_enable (void) const { return utils::get_bit (m_value, 18); }
  cr1_t& set_wakeup_from_stop_enable (bool val) { m_value = utils::set_bit (m_value, 18, val); return *this; }

  constexpr bool scl_stretch_disable (void) const { return utils::get_bit (m_value, 17); }
  cr1_t& set_scl_stretch_disable (bool val) { m_value = utils::set_bit (m_value, 17, val); return *this; }

  constexpr bool slave_byte_enable (void) const { return utils::get_bit (m_value, 16); }
  cr1_t& set_slave_byte_enable (bool val) { m_value = utils::set_bit (m_value, 16, val); return *this; }

  constexpr bool rxdma_enable (void) const { return utils::get_bit (m_value, 15); }
  cr1_t& set_rxdma_enable (bool val) { m_value = utils::set_bit (m_value, 15, val); return *this; }

  constexpr bool txdma_enable (void) const { return utils::get_bit (m_value, 14); }
  cr1_t& set_txdma_enable (bool val) { m_value = utils::set_bit (m_value, 14, val); return *this; }

  constexpr bool analog_noise_filter_disable (void) const { return utils::get_bit (m_value, 12); }
  cr1_t& set_analog_noise_filter_disable (bool val) { m_value = utils::set_bit (m_value, 12, val); return *this; }

  constexpr unsigned int digital_noise_filter (void) const { return (m_value >> 8) & 0b1111; }
  cr1_t& set_digital_noise_filter (unsigned int val) { m_value = (m_value & ~(0b1111u << 8)) | ((val & 0b1111) << 8); return *this; }

  constexpr bool error_interrupt_enable (void) const { return utils::get_bit (m_value, 7); }
  cr1_t& set_error_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 7, val); return *this; }

  constexpr bool transfer_complete_interrupt_enable (void) const { return utils::get_bit (m_value, 6); }
  cr1_t& set_transfer_complete_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 6, val); return *this; }

  constexpr bool stop_interrupt_enable (void) const { return utils::get_bit (m_value, 5); }
  cr1_t& set_stop_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 5, val); return *this; }

  constexpr bool nack_interrupt_enable (void) const { return utils::get_bit (m_value, 4); }
  cr1_t& set_nack_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 4, val); return *this; }
  
  constexpr bool address_match_interrupt_enable (void) const { return utils::get_bit (m_value, 3); }
  cr1_t& set_address_match_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 3, val); return *this; }

  constexpr bool rx_interrupt_enable (void) const { return utils::get_bit (m_value, 2); }
  cr1_t& set_rx_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 2, val); return *this; }

  constexpr bool tx_interrupt_enable (void) const { return utils::get_bit (m_value, 1); }
  cr1_t& set_tx_interrupt_enable (bool val) { m_value = utils::set_bit (m_value, 1, val); return *this; }

  constexpr bool peripheral_enable (void) const { return utils::get_bit (m_value, 0); }
  cr1_t& set_peripheral_enable (bool val) { m_value = utils::set_bit (m_value, 0, val); return *this; }
  
private:
  uint32_t m_value;
};


enum master_transfer_dir_t
{
  master_req_write = 0,
  master_req_read = 1
};

class cr2_t
{
public:
  constexpr cr2_t (void) : m_value (0) { }
  constexpr cr2_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool pec_byte (void) const { return utils::get_bit (m_value, 26); }
  cr2_t& set_pec_byte (bool val) { m_value = utils::set_bit (m_value, 26, val); return *this; }

  constexpr bool auto_end (void) const { return utils::get_bit (m_value, 25); }
  cr2_t& set_auto_end (bool val) { m_value = utils::set_bit (m_value, 25, val); return *this; }

  constexpr bool nbytes_reload (void) const { return utils::get_bit (m_value, 24); }
  cr2_t& set_nbytes_reload (bool val) { m_value = utils::set_bit (m_value, 24, val); return *this; }

  constexpr unsigned int nbytes (void) const { return (m_value >> 16) & 255; }
  cr2_t& set_nbytes (unsigned int val) { m_value = (m_value & ~(255u << 16)) | ((val & 255) << 16); return *this; }

  constexpr bool generate_nack (void) const { return utils::get_bit (m_value, 15); }
  cr2_t& set_generate_nack (bool val) { m_value = utils::set_bit (m_value, 15, val); return *this; }

  constexpr bool generate_stop (void) const { return utils::get_bit (m_value, 14); }
  cr2_t& set_generate_stop (bool val) { m_value = utils::set_bit (m_value, 14, val); return *this; }

  constexpr bool generate_start (void) const { return utils::get_bit (m_value, 13); }
  cr2_t& set_generate_start (bool val) { m_value = utils::set_bit (m_value, 13, val); return *this; }

  constexpr bool head10r (void) const { return utils::get_bit (m_value, 12); }
  cr2_t& set_head10r (bool val) { m_value = utils::set_bit (m_value, 12, val); return *this; }

  constexpr bool add10 (void) const { return utils::get_bit (m_value, 11); }
  cr2_t& set_add10 (bool val) { m_value = utils::set_bit (m_value, 11, val); return *this; }

  constexpr enum master_transfer_dir_t master_trasfer_direction (void) const { return (master_transfer_dir_t)utils::get_bit (m_value, 10); }
  cr2_t& set_master_transfer_direction (master_transfer_dir_t val) { m_value = utils::set_bit (m_value, 10, (bool)val); return *this; }

  constexpr unsigned int slave_address (void) const { return m_value & 1023u; }
  cr2_t& set_slave_address (unsigned int val) { m_value = (m_value & ~1023u) | (val & 1023u); return *this; }

private:
  uint32_t m_value;
};

enum own_address_mode_t
{
  own_address_7_bit = 0,
  own_address_10_bit = 1
};

class oar1_t
{
public:
  constexpr oar1_t (void) : m_value (0) { }
  constexpr oar1_t (uint32_t val): m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool own_address_1_enable (void) const { return utils::get_bit (m_value, 15); }
  oar1_t& set_own_address_1_enable (bool val) { m_value = utils::set_bit (m_value, 15, val); return *this; }

  constexpr own_address_mode_t own_address_mode (void) const { return (own_address_mode_t)utils::get_bit (m_value, 10); }
  oar1_t& set_own_address_mode (own_address_mode_t val) { m_value = utils::set_bit (m_value, 10, (bool)val); return *this; }

  constexpr unsigned int interface_address (void) const { return m_value & 1023u; }
  oar1_t& set_interface_addres (unsigned int val) { m_value = (m_value & ~1023u) | (val & 1023); return *this; }

private:
  uint32_t m_value;
};

class oar2_t
{
public:
  constexpr oar2_t (void) : m_value (0) { }
  constexpr oar2_t (uint32_t val): m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool own_address_2_enable (void) const { return utils::get_bit (m_value, 15); }
  oar2_t& set_own_address_2_enable (bool val) { m_value = utils::set_bit (m_value, 15, val); return *this; }

  constexpr unsigned int own_address_2_masks (void) const { return (m_value >> 8) & 0b111; }
  oar2_t& set_own_address_2_masks (unsigned int val) { m_value = (m_value & ~(0b111u << 8)) | ((val & 0b111) << 8); return *this; }

  constexpr unsigned int interface_address (void) const { return m_value & 1023u; }
  oar2_t& set_interface_addres (unsigned int val) { m_value = (m_value & ~1023u) | (val & 1023); return *this; }

private:
  uint32_t m_value;
};


class timing_reg_t
{
public:
  constexpr timing_reg_t (void) : m_value (0) { }
  constexpr timing_reg_t (uint32_t val): m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr unsigned int prescaler (void) const { return (m_value >> 28) & 0b1111u; }
  timing_reg_t& set_prescaler (unsigned int val) { m_value = (m_value & ~(0b1111u << 28)) | ((val & 0b1111) << 28); return *this; }

  constexpr unsigned int data_setup_time (void) const { return (m_value >> 20) & 0b1111u; }
  timing_reg_t& set_data_setup_time (unsigned int val) { m_value = (m_value & ~(0b1111u << 20)) | ((val & 0b1111) << 20); return *this; }

  constexpr unsigned int data_hold_time (void) const { return (m_value >> 16) & 0b1111u; }
  timing_reg_t& set_data_hold_time (unsigned int val) { m_value = (m_value & ~(0b1111u << 16)) | ((val & 0b1111) << 16); return *this; }

  constexpr unsigned int scl_high_period (void) const { return (m_value >> 8) & 255u; }
  timing_reg_t& set_scl_high_preiod (unsigned int val) { m_value = (m_value & ~(255u << 8)) | ((val & 255u) << 8); return *this; }

  constexpr unsigned int scl_low_period (void) const { return m_value & 255u; }
  timing_reg_t& set_scl_low_period (unsigned int val) { m_value = (m_value & ~255u) | (val & 255u); return *this; }

private:
  uint32_t m_value;
};

class timeout_reg_t
{
public:
  constexpr timeout_reg_t (void) : m_value (0) { }
  constexpr timeout_reg_t (uint32_t val): m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr bool extended_clock_timeout_enable (void) const { return utils::get_bit (m_value, 31); }
  timeout_reg_t& set_extended_clock_timeout_enable (bool val) { m_value = utils::set_bit (m_value, 31, val); return *this; }

  constexpr unsigned int bus_timeout_b (void) const { return (m_value >> 16) & 4095u; }
  timeout_reg_t& set_bus_timeout_b (unsigned int val) { m_value = (m_value & ~(4095u << 16)) | ((val & 4095u) << 16); return *this; }

  constexpr bool clock_timeout_enable (void) const { return utils::get_bit (m_value, 15); }
  timeout_reg_t& set_clock_timeout_enable (bool val) { m_value = utils::set_bit (m_value, 15, val); return *this; }

  constexpr bool idle_clock_timeout_detection (void) const { return utils::get_bit (m_value, 12); }
  timeout_reg_t& set_idle_clock_timeout_detection (bool val) { m_value = utils::set_bit (m_value, 12, val); return *this; }

  constexpr unsigned int bus_timeout_a (void) const { return m_value & 4095u; }
  timeout_reg_t& set_bus_timeout_a (unsigned int val) { m_value = (m_value & ~4095u) | (val & 4095u); return *this; }

private:
  uint32_t m_value;
};

enum transfer_direction_t
{
  write_transfer = 0,
  read_transfer = 1
};

class interrupt_status_t
{
public:
  constexpr interrupt_status_t (void) : m_value (0) { }
  constexpr interrupt_status_t (uint32_t val): m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }
  
  constexpr unsigned int address_match_code (void) const { return (m_value >> 17) & 127u; }
  constexpr transfer_direction_t transfer_direction (void) const { return (transfer_direction_t)utils::get_bit (m_value, 16); }
  constexpr bool busy (void) const { return utils::get_bit (m_value, 15); }
  constexpr bool smbus_alert (void) const { return utils::get_bit (m_value, 13); }
  constexpr bool timeout (void) const { return utils::get_bit (m_value, 12); }
  constexpr bool pec_error (void) const { return utils::get_bit (m_value, 11); }
  constexpr bool overrun (void) const { return utils::get_bit (m_value, 10); }
  constexpr bool arbitration_lost (void) const { return utils::get_bit (m_value, 9); }
  constexpr bool bus_error (void) const { return utils::get_bit (m_value, 8); }
  constexpr bool transfer_complete_reload (void) const { return utils::get_bit (m_value, 7); }
  constexpr bool transfer_complete (void) const { return utils::get_bit (m_value, 6); }
  constexpr bool stop_detected (void) const { return utils::get_bit (m_value, 5); }
  constexpr bool nack_received (void) const { return utils::get_bit (m_value, 4); }
  constexpr bool address_matched (void) const { return utils::get_bit (m_value, 3); }
  constexpr bool rx_data_not_empty (void) const { return utils::get_bit (m_value, 2); }
  constexpr bool tx_interrupt_status (void) const { return utils::get_bit (m_value, 1); }
  constexpr bool tx_data_empty (void) const { return utils::get_bit (m_value, 0); }

private:
  uint32_t m_value;
};

class interrupt_clear_t
{
public:
  constexpr interrupt_clear_t (void) : m_value (0) { }
  constexpr interrupt_clear_t (uint32_t val): m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  constexpr interrupt_clear_t& set_alert_clear (bool val) { m_value = utils::set_bit (m_value, 13, val); return *this; }
  constexpr interrupt_clear_t& set_timeout_clear (bool val) { m_value = utils::set_bit (m_value, 12, val); return *this; }
  constexpr interrupt_clear_t& set_pec_clear (bool val) { m_value = utils::set_bit (m_value, 11, val); return *this; }
  constexpr interrupt_clear_t& set_overrun_clear (bool val) { m_value = utils::set_bit (m_value, 10, val); return *this; }
  constexpr interrupt_clear_t& set_arbitration_lost_clear (bool val) { m_value = utils::set_bit (m_value, 9, val); return *this; }
  constexpr interrupt_clear_t& set_bus_error_clear (bool val) { m_value = utils::set_bit (m_value, 8, val); return *this; }
  constexpr interrupt_clear_t& set_stop_clear (bool val) { m_value = utils::set_bit (m_value, 5, val); return *this; }
  constexpr interrupt_clear_t& set_nack_clear (bool val) { m_value = utils::set_bit (m_value, 4, val); return *this; }
  constexpr interrupt_clear_t& set_address_matched_clear (bool val) { m_value = utils::set_bit (m_value, 3, val); return *this; }

private:
 uint32_t m_value;
};

template<uint32_t RegBaseAddress, unsigned int PeripheralClockHz>
class i2c_master : public dev::i2c_master
{
public:
  i2c_master (void)
  {
    set_cr1 (cr1 ()
  	.set_pec_enable (false)
  	.set_smbus_alert (false)
  	.set_smbus_default_address_enable (false)
  	.set_smbus_host_address_enable (false)
  	.set_general_call_enable (false)
  	.set_wakeup_from_stop_enable (false)
  	.set_scl_stretch_disable (false)
  	.set_slave_byte_enable (false)
  	.set_rxdma_enable (false)
  	.set_txdma_enable (false)
  	.set_analog_noise_filter_disable (false)
  	.set_digital_noise_filter (1)
  	.set_error_interrupt_enable (false)
  	.set_transfer_complete_interrupt_enable (false)
  	.set_stop_interrupt_enable (false)
  	.set_nack_interrupt_enable (false)
  	.set_address_match_interrupt_enable (false)
  	.set_rx_interrupt_enable (false)
  	.set_tx_interrupt_enable (false)
  	.set_peripheral_enable (false)
    );

    set_cr2 (cr2 ()
	.set_pec_byte (false)
	.set_auto_end (false)
	.set_nbytes_reload (false)
	.set_nbytes (0)
	.set_generate_nack (false)
	.set_generate_stop (false)
	.set_generate_start (false)
	.set_head10r (false)
	.set_add10 (false)
	.set_master_transfer_direction (master_req_write)
	.set_slave_address (0)
    );

    // 400 khz for i2c clock = default HSI clock = 8 MHz
    static_assert (PeripheralClockHz == 48'000'000);
    set_timingr (timingr ()
	.set_prescaler (0)
  	.set_data_setup_time (3)
  	.set_data_hold_time (1)
  	.set_scl_high_preiod (3)
  	.set_scl_low_period (6)
    );

    set_cr1 (cr1 ().set_peripheral_enable (true));
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

    set_icr (icr ()
        .set_alert_clear (true)
        .set_timeout_clear (true)
        .set_pec_clear (true)
        .set_overrun_clear (true)
        .set_arbitration_lost_clear (true)
        .set_bus_error_clear (true)
        .set_stop_clear (true)
        .set_nack_clear (true)
        .set_address_matched_clear (true)
    );

    set_cr2 (cr2 ()
      .set_add10 (false)
      .set_slave_address (slave_addr)
      .set_master_transfer_direction (master_req_write)
      .set_nbytes (count_bytes)
      .set_auto_end (true)
    );

    set_cr2 (cr2 ().set_generate_start (true));
    
    while (count_bytes > 0)
    {
      auto isr_s = isr ();
      if (isr_s.tx_interrupt_status ())
      {
      	set_tx_dr (*tx_data_u8++);
      	--count_bytes;
      }

      else if (isr_s.timeout () | isr_s.pec_error () | isr_s.overrun ()
               | isr_s.arbitration_lost () | isr_s.bus_error () | isr_s.overrun ()
               | isr_s.nack_received ())
      	return false;
    }

    return true;
  }


  using dev::i2c_master::send;


  virtual bool
  recv (uint8_t slave_addr, void* rx_data, unsigned int count_bytes) override
  {
    slave_addr |= 1; // make sure to set the read bit of the address.

    uint8_t* rx_data_u8 = (uint8_t*)rx_data;

    set_icr (icr ()
        .set_alert_clear (true)
        .set_timeout_clear (true)
        .set_pec_clear (true)
        .set_overrun_clear (true)
        .set_arbitration_lost_clear (true)
        .set_bus_error_clear (true)
        .set_stop_clear (true)
        .set_nack_clear (true)
        .set_address_matched_clear (true)
    );

    set_cr2 (cr2 ()
      .set_add10 (false)
      .set_slave_address (slave_addr)
      .set_master_transfer_direction (master_req_read)
      .set_nbytes (count_bytes)
      .set_auto_end (true)
    );

    set_cr2 (cr2 ().set_generate_start (true));

    for (auto start_time = std::chrono::high_resolution_clock::now (); count_bytes > 0;)
    {
      auto isr_s = isr ();

      if (isr_s.rx_data_not_empty ())
      {
        *rx_data_u8++ = rxdr ();
        --count_bytes;

        start_time = std::chrono::high_resolution_clock::now ();
      }
      else if (std::chrono::high_resolution_clock::now () - start_time
               > std::chrono::milliseconds (10))
        return false;
    }

    return true;
  }

  using dev::i2c_master::recv;
  using dev::i2c_master::send_recv;


private:

  const regs_t& regs (void) const { return *(const regs_t*)RegBaseAddress; }
  regs_t& regs (void) { return *(regs_t*)RegBaseAddress; }

  cr1_t cr1 (void) { return { regs ().cr1 }; }
  void set_cr1 (const cr1_t& val) { regs ().cr1 = val.value (); }

  cr2_t cr2 (void) { return { regs ().cr2 }; }
  void set_cr2 (const cr2_t& val) { regs ().cr2 = val.value (); }

  oar1_t oar1 (void) { return { regs ().oar1 }; }
  void set_oar1 (const oar1_t& val) { regs ().oar1 = val.value (); }

  oar2_t oar2 (void) { return { regs ().oar2 }; }
  void set_oar2 (const oar2_t& val) { regs ().oar2 = val.value (); }

  timing_reg_t timingr (void) { return { regs ().timingr }; }
  void set_timingr (const timing_reg_t& val) { regs ().timingr = val.value (); }

  timeout_reg_t timeoutr (void) { return { regs ().timeoutr }; }
  void set_timeoutr (const timeout_reg_t& val) { regs ().timeoutr = val.value (); }

  interrupt_status_t isr (void) { return { regs ().isr }; }
  void set_isr (const interrupt_status_t& val) { regs ().isr = val.value (); }

  interrupt_clear_t icr (void) { return { regs ().icr }; }
  void set_icr (const interrupt_clear_t& val) { regs ().icr = val.value (); }

  uint8_t pecr (void) { return regs ().pecr; }
  uint8_t rxdr (void) { return regs ().rxdr; }
  uint8_t txdr (void) { return regs ().txdr; }
  void set_tx_dr (uint8_t val) { regs ().txdr = val; }
};

} // namespace stm32f0_i2c
} // namespace dev

#endif // includeguard_dev_stm32f0_i2c_hpp_includeguard