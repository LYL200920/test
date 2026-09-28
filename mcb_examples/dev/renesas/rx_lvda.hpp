/*

RX voltage detection circuit (LVDA)

the hardware found in rx64 and rx71 is a bit different from rx63.

on rx63 interrupts from LVD1 and LVD2 are always multiplexed through the NMI
and have to be demultiplexed by the driver.

on rx64/rx71 there are two dedicated interrupt lines for that.

*/

#ifndef includeguard_dev_rx_lvda_hpp_includeguard
#define includeguard_dev_rx_lvda_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>
#include <functional>
#include <chrono>
#include <thread>

namespace dev
{
namespace rx_lvda
{

enum trigger_type_t
{
  //                           CR1  | CR0 value
  reset_on_vcc_less_or_equal = 0b00 | 0b0100'0000,
  int_on_vcc_greater_equal   = 0b00 | 0b0000'0000,
  int_on_vcc_less            = 0b01 | 0b0000'0000,
  int_on_vcc_drop_or_rise    = 0b10 | 0b0000'0000
};

/*
for now always use maskable interrupts

enum interrupt_type_t
{
  non_maskable_interrupt = 0,

  // not available on RX63
  maskable_interrupt = 1
};
*/

enum voltage_t : uint8_t
{
  v2_99 = 0b1001,  // RX64, RX71 only

  v2_92 = 0b1010,  // RX64, RX71
  v2_95 = 0b1010,  // RX63 (the only supported value for RX63)

  v2_85 = 0b1011,  // RX64, RX71 only
};

enum noise_filter_t
{
  filter_off = -1,       // artificial value
  filter_2_loco  = 0b00, // filter at 1/2 LOCO frequency
  filter_4_loco  = 0b01, // filter at 1/4 LOCO frequency
  filter_8_loco  = 0b10, // filter at 1/8 LOCO frequency
  filter_16_loco = 0b11  // filter at 1/16 LOCO frequency
};

enum reset_release_t
{
  // release reset after a stabilization time tLVD1 after VCC > Vdet1
  after_vcc_greater = 0,

  // release reset after a stabilization time tLVD1 after LVD1 reset assert
  after_reset_delay = 1
};

class config_t
{
public:
  constexpr config_t (void) : m_cr0_value (0), m_cr1_value (0), m_threshold_voltage (v2_95) { }
  constexpr config_t (uint8_t v0, uint8_t v1, voltage_t v2) : m_cr0_value (v0), m_cr1_value (v1), m_threshold_voltage (v2) { }

  constexpr uint8_t cr0_value (void) const { return m_cr0_value; }
  constexpr uint8_t cr1_value (void) const { return m_cr1_value; }

  constexpr trigger_type_t trigger (void) const
  {
    return (m_cr0_value & 0b0100'0000) != 0
	   ? reset_on_vcc_less_or_equal
	   : (trigger_type_t)(m_cr1_value & 0b11);
  }

  constexpr config_t& set_trigger (trigger_type_t val)
  {
    m_cr0_value = (m_cr0_value & 0b1011'1111) | val;
    m_cr1_value = (m_cr1_value & 0b1111'1100) | val;
    return *this;
  }

//  constexpr interrupt_type_t interrupt_type (void) const { return (interrupt_type_t)((m_cr1_value >> 2) & 1); }
//  constexpr config_t& set_interrupt_type (interrupt_type_t val) { m_cr1_value = (m_cr1_value & 0b1111'1011) | (val << 2); return *this; }

  constexpr voltage_t threshold_voltage (void) const { return m_threshold_voltage; }
  constexpr config_t& set_threshold_voltage (voltage_t val) { m_threshold_voltage = val; return *this; }

  constexpr noise_filter_t noise_filter (void) const
  {
    return (m_cr0_value & 0b10) == 0
	   ? (noise_filter_t)((m_cr0_value >> 4) & 0b11)
	   : filter_off;
  }
  constexpr config_t& set_noise_filter (noise_filter_t val)
  {
    m_cr0_value = (m_cr0_value & 0b11000101)
		  | (val == filter_off ? 0b00000010 : (val << 4));
    return *this;
  }

  constexpr bool voltage_monitor_enabled (void) const { return (m_cr0_value & 0b00000001) != 0; }
  constexpr config_t& set_voltage_monitor_enabled (bool val = true) { m_cr0_value = (m_cr0_value & 0b11111110) | (val << 0); return *this; }

  constexpr bool voltage_comparator_enabled (void) const { return (m_cr0_value & 0b00000100) != 0; }
  constexpr config_t& set_voltage_comparator_enabled (bool val = true) { m_cr0_value = (m_cr0_value & 0b11111011) | (val << 2); return *this; }

  constexpr reset_release_t reset_release (void) const { return (reset_release_t)(m_cr0_value >> 7); }
  constexpr config_t& set_reset_release (reset_release_t val) { m_cr0_value = (m_cr0_value & 0b01111111) | (val << 7); return *this; }

private:
  uint8_t m_cr0_value;
  uint8_t m_cr1_value;
  voltage_t m_threshold_voltage;
};

class status_t
{
public:
  constexpr status_t (void) : m_value (0) { }
  constexpr status_t (uint8_t v) : m_value (v) { }

  constexpr bool threshold_passed (void) const { return (m_value & 0b01) != 0; }
  constexpr bool vcc_less_than (void) const { return (m_value & 0b10) == 0; }

private:
  uint8_t m_value;
};

class hw_inst_base
{
public:

  // changing the config should be done while the respective voltage detection
  // circuit is turned off.

  config_t lvd1_config (void) const
  {
    return m_config;
  }
  void set_lvd1_config (const config_t& val)
  {
    m_config = val;
  }

  bool lvd1_enabled (void) const { return (LVCMPCR & 0b00100000) != 0; }
  void set_lvd1_enable (bool val = true)
  {
    LVCMPCR &= 0b11011111;
    LVD1CR0 = 0b10000010;
    LVD1SR = 0;
    LVD1SR.read ();

    if (val)
    {
      // Select the detection voltage by setting the LVDLVLR.LVD1LVL[3:0] bits.
      LVDLVLR = (LVDLVLR & 0b1111'0000) | ((m_config.threshold_voltage () & 0b1111) << 0);

      // Set LVCMPCR.LVD1E = 1 (enabling the voltage detection 1 circuit).* 4
      LVCMPCR |= 0b00100000;

      // Wait for at least td(E-A) (LVD operation stabilization time after LVD is enabled).*
      // td(E-A) is about 10 usec
      std::this_thread::sleep_for (std::chrono::microseconds (15));

      // Select the sampling clock for the digital filter by setting the LVD1CR0.LVD1FSAMP[1:0] bits.
      if (m_config.noise_filter () != filter_off)
      {
	LVD1CR0 = (LVD1CR0 & 0b11000101) | (m_config.cr0_value () & 0b00110010);

	// Wait for at least 2n + 3 cycles of the LOCO
	// (where n = 2, 4, 8, 16, and the sampling clock for the digital filter is the LOCO frequency-divided by n).
	// LOCO = 240 kHz.  2*16 + 3 = 35 cycles = 146 usec
	std::this_thread::sleep_for (std::chrono::microseconds (150));
      }
      else
	LVD1CR0 = (LVD1CR0 & 0b11000101);


      // Set LVD1CR0.LVD1RI = 0 (selecting the voltage monitoring 1 interrupt).
      // Set LVD1CR0.LVD1RI = 1 (selecting the voltage monitoring 1 reset).
      // Select the type of the reset negation by setting the LVD1CR0.LVD1RN bit.
      LVD1CR0 = (LVD1CR0 & 0b00110111) | (m_config.cr0_value () & 0b11000000);

      // Select the timing of interrupt requests by setting the LVD1CR1.LVD1IDTSEL[1:0] bits.
      // Select the type of interrupt by setting the LVD1CR1.LVD1IRQSEL bit.
      LVD1CR1 = (m_config.cr1_value () & 0b11) | 0b100;

      // Set LVD1SR.LVD1DET = 0.
      LVD1SR = 0;

      // Set LVD1CR0.LVD1RIE = 1 (enabling the voltage monitoring 1 interrupt or reset).* 3
      // Set LVD1CR0.LVD1CMPE = 1 (enabling output of the results of comparison by voltage monitoring 1).
      LVD1CR0 = (LVD1CR0 & 0b11110011) | (m_config.cr0_value () & 0b00000101);
    }

    LVD1SR.read ();
  }

  status_t lvd1_status (void) const { return status_t (LVD1SR); }



  config_t lvd2_config (void) const
  {
    return config_t (LVD2CR0, LVD2CR1, (voltage_t)((LVDLVLR >> 4) & 0b1111));
  }
  void set_lvd2_config (const config_t& val)
  {
    LVD2CR0 = val.cr0_value () & 0b11110111;
    LVD2CR1 = (LVD2CR1 & 0b00000100) | (val.cr1_value () & 0b11111011);
    LVDLVLR = (LVDLVLR & 0b0000'1111) | (val.threshold_voltage () << 4);
  }

  bool lvd2_enabled (void) const { return (LVCMPCR & 0b01000000) != 0; }
  void set_lvd2_enable (bool val = true)
  {
    if (val)
    {
      LVCMPCR |= 0b01000000;
    }
    else
    {
      LVCMPCR &= 0b10111111;
    }
  }


protected:
  hw_inst_base (void) { }
//  ~hw_inst_base (void) { LVCMPCR = 0; }


  void lvd1_isr (void)
  {
    LVD1SR = 0;

    if (m_lvd1_func)
      m_lvd1_func ();
  }

  void lvd2_isr (void)
  {
    LVD2SR = 0;

    if (m_lvd2_func)
      m_lvd2_func ();
  }

  static constexpr hw_reg_rw<uint8_t, const_addr<0x000800E0>> LVD1CR1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x000800E1>> LVD1SR = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x000800E2>> LVD2CR1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x000800E3>> LVD2SR = { };

  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008C297>> LVCMPCR = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008C298>> LVDLVLR = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008C29A>> LVD1CR0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008C29B>> LVD2CR0 = { };

  std::function <void(void)> m_lvd1_func;
  std::function <void(void)> m_lvd2_func;

  config_t m_config;
};


template <typename InterruptLine_LVD1,
	  typename InterruptLine_LVD2 >
class hw_inst_rx64 : public hw_inst_base
{
public:
  typedef interrupt::connected_isr<InterruptLine_LVD1,
		interrupt::func<decltype (&hw_inst_rx64::lvd1_isr), &hw_inst_rx64::lvd1_isr>> lvd1_isr_t;

  typedef interrupt::connected_isr<InterruptLine_LVD2,
		interrupt::func<decltype (&hw_inst_rx64::lvd2_isr), &hw_inst_rx64::lvd2_isr>> lvd2_isr_t;

  hw_inst_rx64 (void) : m_lvd1_isr (this), m_lvd2_isr (this)
  {
  }

  template <typename F>
  void set_lvd1_trigger_func (F&& f)
  {
    m_lvd1_func = std::forward<F> (f);
    if (m_lvd1_func)
      m_lvd1_isr.enable (dev::interrupt::falling_edge, dev::interrupt::priority_15);
    else
      m_lvd1_isr.disable ();
  }

  template <typename F>
  void set_lvd2_trigger_func (F&& f)
  {
    m_lvd2_func = std::forward<F> (f);

    if (m_lvd2_func)
      m_lvd2_isr.enable (dev::interrupt::falling_edge, dev::interrupt::priority_15);
    else
      m_lvd2_isr.disable ();
  }


private:
  lvd1_isr_t m_lvd1_isr;
  lvd2_isr_t m_lvd2_isr;
};


class hw_inst_rx63 : public hw_inst_base
{
public:
  hw_inst_rx63 (void) { }

  void on_nmi (void)
  {
    

  }

private:
};


} // namespace rx_lvda
} // namespace dev
#endif // includeguard_dev_rx_lvda_hpp_includeguard
