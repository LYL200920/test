/*
  DAC124S085

  12 bit micro power quad digital-to-analog converter with rail-to-rail output.

  the DAC settle time is about 8.5 usec, which is about 117 kHz.
  each register write is 16 bits and there are 4 registers, thus 64 bits.
  to update the DAC at its max. frequency, the SPI channel should be able
  to do 117 * 64 = 7488 kHz = 7.5 Mbps.

*/

#ifndef includeguard_dev_dac124s085_includeguard
#define includeguard_dev_dac124s085_includeguard

#include <cstdint>
#include <cstring>
#include <bitset>
#include <type_traits>

#include <dev/spi.hpp>
#include <utils/value_range.hpp>
#include <utils/byte_order.hpp>

namespace dev
{

template < unsigned int CSLine, unsigned int MaxClockSpeedHz >
class dac124s085
{
public:
  static constexpr unsigned int num_channels = 4;
  static constexpr unsigned int cs_line = CSLine;

  dac124s085 (dev::spi::master& d) : m_spi (d)
  {
    spi::transfer_config tc;
    tc.bitrate_hz = MaxClockSpeedHz;

    spi::transfer_config::phase_config ph;
	ph.allow_trailing_clock = true;
	ph.data_dir = spi::write;
	ph.clock_phase = spi::latch_odd_shift_even;
	ph.clock_polarity = spi::positive;
	ph.byte_order = spi::two_byte;
	ph.bit_order = spi::msb_first;
	ph.lane_mode = spi::single_lane_half_duplex;
	ph.data_idle_output_mode = spi::output_high;
	ph.data2_unused_output_mode = spi::high_z;
	ph.data3_unused_output_mode = spi::high_z;
	ph.cs_idle_output_mode = spi::output_high;
	ph.clk_idle_output_mode = spi::output_high;
	ph.bitrate_div = 0;
	ph.cs_line = cs_line;
	ph.transfer_count = 1;
	ph.clock_delay_cycles = 0;
	ph.cs_delay_cycles = 0;
	ph.next_phase_delay_cycles = 1; // need at least one delay cycle to de-assert
					// the CS line, before it gets asserted again
					// in the next phase.

    tc.phases.push_front (ph);

    m_transfer_single = d.prepare_transfer (tc);


    // for multiple transfers we just repeat the same transfer phase for each
    // command word.  since all phases are equal, the order doesn't matter.
    for (unsigned int i = 1; i < num_channels; ++i)
      tc.phases.push_front (ph);

    m_transfer_all = d.prepare_transfer (tc);
  }

  // set the value to the register but do not update the outputs
  void set_dac_reg_keep_outputs (unsigned int channel,
				 utils::clamped_value<uint16_t, 0, 4095> val)
  {
    write_cmd (make_cmd_set_dac_reg_keep_outputs (channel, val));
  }

  // set the value to the register and update the outputs
  void set_dac_reg_update_outputs (unsigned int channel,
				   utils::clamped_value<uint16_t, 0, 4095> val)
  {
    write_cmd (make_cmd_set_dac_reg_update_outputs (channel, val));
  }

  // write the same value to all registers and update the outputs
  void set_all_regs_update_outputs (utils::clamped_value<uint16_t, 0, 4095> val)
  {
    write_cmd ((0b00 << 14) | (0b10 << 12) | val);
  }

  // write all dac registers and update the outputs at the end of the write
  void set_dac_regs_update_outputs (utils::clamped_value<uint16_t, 0, 4095> reg0,
				    utils::clamped_value<uint16_t, 0, 4095> reg1,
				    utils::clamped_value<uint16_t, 0, 4095> reg2,
				    utils::clamped_value<uint16_t, 0, 4095> reg3)
  {
    m_spi.transfer (m_transfer_single,
      {
	utils::native_to (utils::big_endian, make_cmd_set_dac_reg_update_outputs (0, reg0)),
	utils::native_to (utils::big_endian, make_cmd_set_dac_reg_update_outputs (1, reg1)),
	utils::native_to (utils::big_endian, make_cmd_set_dac_reg_update_outputs (2, reg2)),
	utils::native_to (utils::big_endian, make_cmd_set_dac_reg_update_outputs (3, reg3))
      },
      { },
      [] (void) { });

    // there should also be a way to support multiple DACs on the same SPI
    // bus in one SPI transfer.  this will allow updating multiple DAC channels
    // at the same time with minimal delay/skew.

    // -> use a wrapper class (dac group) around the individual dac instances.

    set_dac_reg_keep_outputs (0, reg0);
    set_dac_reg_keep_outputs (1, reg1);
    set_dac_reg_keep_outputs (2, reg2);
    set_dac_reg_update_outputs (3, reg3);
  }

  // power down all outputs and terminate them with a 2.5K pull-down
  void power_down_outputs_pulldown_2k5 (void)
  {
    write_cmd ((0b01 << 14) | (0b11 << 12));
  }

  // power down all outputs and terminate them with a 100K pull-down
  void power_down_outputs_pulldown_100k (void)
  {
    write_cmd ((0b10 << 14) | (0b11 << 12));
  }

  // power down all outputs and put them in high-z
  void tristate_outputs (void)
  {
    write_cmd ((0b11 << 14) | (0b11 << 12));
  }

private:
  dev::spi::master& m_spi;
  std::shared_ptr<spi::prepared_transfer> m_transfer_single;
  std::shared_ptr<spi::prepared_transfer> m_transfer_all;

  static constexpr uint16_t
  make_cmd_set_dac_reg_keep_outputs (unsigned int channel, uint16_t val)
  {
    return ((channel & 0b11) << 14) | (0b00 << 12) | val;
  }

  static constexpr uint16_t
  make_cmd_set_dac_reg_update_outputs (unsigned int channel, uint16_t val)
  {
    return ((channel & 0b11) << 14) | (0b01 << 12) | val;
  }

  void write_cmd (uint16_t val)
  {
    m_spi.transfer (m_transfer_single,
	{ utils::native_to (utils::big_endian, val) }, { },
	[] (void) { });
  }
};

} // namespace dev
#endif // includeguard_dev_pca9698_includeguard
