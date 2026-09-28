/*

RX clock frequency accuracy measurement circuit (CAC)

*/

#ifndef includeguard_dev_rx_cac_hpp_includeguard
#define includeguard_dev_rx_cac_hpp_includeguard

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace rx_cac
{

// CASTR, CACNTBR
class measurement_result_t
{
public:
  constexpr measurement_result_t (void) : m_value0 (0), m_value1 (0) { }
  constexpr measurement_result_t (uint8_t val0, uint16_t val1) : m_value0 (val0), m_value1 (val1) { }

  constexpr bool frequency_in_range (void) const { return (m_value0 & 0b001) == 0; }
  constexpr bool frequency_out_of_range (void) const { return (m_value0 & 0b001) != 0; }

  constexpr bool measurement_in_progress (void) const { return (m_value0 & 0b010) == 0; }
  constexpr bool measurement_finished (void) const { return (m_value0 & 0b010) != 0; }

  constexpr bool counter_overflowed (void) const { return (m_value0 & 0b100) != 0; }

  constexpr uint16_t frequency_counter (void) const { return m_value1; }

private:
  uint8_t m_value0;
  uint16_t m_value1;
};


enum clock_type
{
  main_clock = 0b000,
  sub_clock = 0b001,
  hoco_clock = 0b010,
  loco_clock = 0b011,
  iwdt_clock = 0b100,
  pclk_b = 0b101
};

enum target_clock_div_t
{
  target_div_1 = 0b00,
  target_div_4 = 0b01,
  target_div_8 = 0b10,
  target_div_32 = 0b11
};

enum filter_t
{
  no_filter = 0b00,
  filter_1 = 0b01,
  filter_4 = 0b10,
  filter_16 = 0b11
};

enum edge_type_t
{
  rising_edge = 0b00,
  falling_edge = 0b01,
  any_edge = 0b10
};

enum reference_clock_div_t
{
  ref_div_32 = 0b00,
  ref_div_128 = 0b01,
  ref_div_1024 = 0b10,
  ref_div_8192 = 0b11
};

// CACR1, CACR2
class measurement_config_t
{
public:
  constexpr measurement_config_t (void) : m_value0 (0), m_value1 (1) { }
  constexpr measurement_config_t (uint8_t val0, uint8_t val1) : m_value0 (val0), m_value1 (val1) { }

  constexpr uint8_t value0 (void) const { return m_value0; }
  constexpr uint8_t value1 (void) const { return m_value1; }

  // the selected target clock increments the timer counter during the measurement.
  constexpr clock_type target_clock (void) const { return (clock_type)((m_value0 >> 1) & 0b111); }
  constexpr measurement_config_t& set_target_clock (clock_type val) { m_value0 = (m_value0 & 0b11110001) | (val << 1); return *this; }

  constexpr target_clock_div_t target_clock_div (void) const { return (target_clock_div_t)((m_value0 >> 4) & 0b11); }
  constexpr measurement_config_t& set_target_clock_div (target_clock_div_t val) { m_value0 = (m_value0 & 0b11001111) | (val << 4); return *this; }


  // at each reference clock tick, the measurement finishes and the timer
  // counter value is compared against the high and low thresholds to determine
  // the measurement result.
  constexpr clock_type reference_clock (void) const { return (clock_type)((m_value1 >> 1) & 0b111); }
  constexpr measurement_config_t& set_reference_clock (clock_type val) { m_value1 = (m_value1 & 0b11110001) | (val << 1); return *this; }

  // if the CACREF pin is used as an input it overrides the reference clock type setting.
  constexpr bool use_cacref_pin_input (void) const { return utils::get_bit (m_value0, 0); }
  constexpr measurement_config_t& set_use_cacref_pin_input (bool val = true)
  {
    m_value0 = utils::set_bit (m_value0, 0, val);
    m_value1 = utils::set_bit (m_value1, 0, !val);
    return *this;
  }

  constexpr filter_t cacref_pin_filter (void) const { return (filter_t)((m_value1 >> 6) & 0b11); }
  constexpr measurement_config_t& set_cacref_pin_filter (filter_t val) { m_value1 = (m_value1 & 0b00111111) | (val << 6); return *this; }

  constexpr reference_clock_div_t reference_clock_div (void) const { return (reference_clock_div_t)((m_value1 >> 4) & 0b11); }
  constexpr measurement_config_t& set_reference_clock_div (reference_clock_div_t val) { m_value1 = (m_value1 & 0b11001111) | (val << 4); return *this; }

  constexpr edge_type_t reference_clock_edge_type (void) const { return (edge_type_t)((m_value0 >> 6) & 0b11); }
  constexpr measurement_config_t& set_reference_clock_edge_type (edge_type_t val) { m_value0 = (m_value0 & 0b00111111) | (val << 6); return *this; }



private:
  uint8_t m_value0;
  uint8_t m_value1;
};

struct frequency_range_t
{
  uint16_t low;
  uint16_t high;
};


template <typename InterruptLine_FERRIE,
	  typename InterruptLine_MENDIE,
	  typename InterruptLine_OVFIE,
	  typename ModuleEnableDisableFunc>
class hw_inst
{
private:
  void ferrie_isr (void)
  {
  }

  void mendie_isr (void)
  {

  }

  void ovfie_isr (void)
  {

  }

public:
  typedef interrupt::connected_isr<InterruptLine_FERRIE,
		interrupt::func<decltype (&hw_inst::ferrie_isr), &hw_inst::ferrie_isr>> ferrie_isr_t;

  typedef interrupt::connected_isr<InterruptLine_MENDIE,
		interrupt::func<decltype (&hw_inst::mendie_isr), &hw_inst::mendie_isr>> mendie_isr_t;

  typedef interrupt::connected_isr<InterruptLine_OVFIE,
		interrupt::func<decltype (&hw_inst::ovfie_isr), &hw_inst::ovfie_isr>> ovfie_isr_t;

  hw_inst (void)
  : m_ferrie_isr (this), m_mendie_isr (this), m_ovfie_isr (this)
  {
    set_device_enable (true);

    // FIXME: enable and use interrupts, if they have been connected.
    // if there are unconnected interrupt lines, this driver needs to periodically
    // poll the measurement result.
  }

  ~hw_inst (void)
  {
    set_device_enable (false);
  }

  void set_device_enable (bool val)
  {
    ModuleEnableDisableFunc () (val);
  }


  measurement_config_t config (void) const { return { CACR1, CACR2 }; }
  void set_config (measurement_config_t val) { CACR1 = val.value0 (); CACR2 = val.value1 (); }

  frequency_range_t target_frequency_range (void) const { return { CALLVR, CAULVR }; }
  void set_target_frequency_range (frequency_range_t val) { CAULVR = val.high; CALLVR = val.low; }

  void start_measurement (void)
  {
    // the counter will be cleared by writing 0.  so do that first.
    stop_measurement ();

    do CACR0 = 1; while (CACR0 == 0);
  }

  void stop_measurement (void)
  {
    do CACR0 = 0; while (CACR0 != 0);
  }

  bool measurement_started (void) const { return CACR0 != 0; }

  measurement_result_t measurement_result (void) const { return { CASTR, CACNTBR }; }

  static constexpr auto counter_max_value (void) { return std::numeric_limits< decltype (CACNTBR)::base_type>::max (); }

private:
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008B000>> CACR0 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008B001>> CACR1 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008B002>> CACR2 = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008B003>> CAICR = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x0008B004>> CASTR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x0008B006>> CAULVR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x0008B008>> CALLVR = { };
  static constexpr hw_reg_r<uint16_t, const_addr<0x0008B00A>> CACNTBR = { };

  ferrie_isr_t m_ferrie_isr;
  mendie_isr_t m_mendie_isr;
  ovfie_isr_t m_ovfie_isr;
};


} // namespace rx_cac
} // namespace dev
#endif // includeguard_dev_rx_cac_hpp_includeguard
