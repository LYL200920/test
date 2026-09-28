/*

MCB v1.3 board specific driver for the digital inputs and digital outputs.

there are 2x PCA9698, connected via I2C to the MCU.
although the IOs of the PCAs can be freely configured, the circuity
around the IOs is designed either for inputs or outputs.  thus we can
basically use only one fixed configuration for the two PCAs.

the PCA that is used for inputs can also trigger an interrupt to the MCU.

PCA addr 0x40
  IO bank 0: DI 0..7
  IO bank 1: DI 8..15
  IO bank 2: DI 16..23
  IO bank 3: DI 24..31
  IO bank 4: XOUT5, XOUT6, XOUT7, XOUT8
             YOUT5, YOUT6, YOUT7, YOUT8

  IO bank 4 is used as additional servo motor control, which is
  part of the MCX block.  thus we don't provide access to these inputs here.

PCA addr 0x42
  IO bank 0: DO 0..7
  IO bank 1: DO 8..15
  IO bank 2: DO 16..23
  IO bank 3: DO 24..31
  IO bank 3: ZOUT5, ZOUT6, ZOUT7, ZOUT8
             UOUT5, UOUT6, UOUT7, UOUT8

  IO bank 4 is used as additional servo motor control, which is
  part of the MCX block.  thus we don't provide access to these inputs here.


the digital_io class configures all the IOs of the PCAs correctly, but the
non-general-purpose IOs are not exposed here.  they are accessed by the
corresponding classes directly instead.


because of the slow PCA access over I2C, the input/output values are cached
in RAM and periodically synchronized.  this makes the software that uses
the digital IOs simpler, as it can just manipulate the bits in RAM.

*/

#ifndef includeguard_mcbv13_board_dev_digital_io_includeguard
#define includeguard_mcbv13_board_dev_digital_io_includeguard

#include <cstdint>
#include <bitset>
#include <array>
#include <chrono>

#include <dev/pca9698.hpp>
#include <utils/bits.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

/*
I2C access time
  1.5 MBit/sec (1 clock = 666.666666667 ns)
  32 IOs = 4 bytes + 1 address byte = 5 bytes = 40 bits
  -> assume about 48 clocks -> 32000 ns = 32 usec

  because we need to read and write to two different PCAs,
  the access time is double, i.e. about 64 usec

FIXME: change this device to use byte arrays instead of bitsets to represent
       the IO values in memory.
       this will also allow implementing a Timer + DTC pulse output controller
       which can write bytes to memory, but not bits.


FIXME: this whole thing should use I2C DMA/background transfers and a timer
       to schedule the transfers.
       then can aso leave out clock cycles if the outputs have not changed.
*/

static constexpr auto digital_io_min_update_time = std::chrono::microseconds (120);
static constexpr auto digital_io_poll_time = digital_io_min_update_time * 10;

template < typename PCADev >
class digital_inputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 32;

  typedef PCADev pca9698_t;

  digital_inputs (pca9698_t& p) : m_pca9698 (p)
  {
    // inputs are inverted by default.
    m_and_mask.set ();
    m_or_mask.reset ();
    m_xor_mask.set ();

    sync ();
  }

  std::bitset<port_count> and_mask (void) const { return m_and_mask; }
  std::bitset<port_count> or_mask (void) const { return m_or_mask; }
  std::bitset<port_count> xor_mask (void) const { return m_xor_mask; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val; }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val; }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val; }

  std::bitset<port_count> read (void) const
  {
    return ((m_inputs & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

  void write (const std::bitset<port_count>&)
  {
  }

  void exec (std::chrono::high_resolution_clock::time_point cur_time)
  {
    if (cur_time - m_update_time >= digital_io_poll_time)
    {
      m_update_time = cur_time;
      sync_1 ();
    }
  }

  void sync (void)
  {
    m_update_time = std::chrono::high_resolution_clock::now ();
    sync_1 ();
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  pca9698_t& m_pca9698;

  std::bitset<port_count> m_inputs;
  std::chrono::high_resolution_clock::time_point m_update_time;

  std::bitset<port_count> m_and_mask;
  std::bitset<port_count> m_or_mask;
  std::bitset<port_count> m_xor_mask;

  void sync_1 (void)
  {
    using utils::get_bit;

    static_assert (port_count == 32, "");

    uint32_t pca_bits = m_pca9698.template input_ports <0, 32> ().to_ulong ();

    m_inputs = std::bitset<port_count> (pca_bits);
  }
};

// ---------------------------------------------------------------------------

template < typename PCADev >
class digital_outputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 32;

  typedef PCADev pca9698_t;

  digital_outputs (pca9698_t& p) : m_pca9698 (p)
  {
    m_and_mask.set ();
    m_or_mask.reset ();
    m_xor_mask.reset ();

    sync ();
  }

  std::bitset<port_count> and_mask (void) const { return m_and_mask; }
  std::bitset<port_count> or_mask (void) const { return m_or_mask; }
  std::bitset<port_count> xor_mask (void) const { return m_xor_mask; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val; }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val; }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val; }

  // when reading the outputs, re-apply the AOX masks so that
  // write (read ()) will result in a no-operation.
  std::bitset<port_count> read (void) const
  {
    return m_outputs ^ m_xor_mask;
  }

  void write (const std::bitset<port_count>& val)
  {
    m_outputs = ((val & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

  void exec (std::chrono::high_resolution_clock::time_point cur_time)
  {
    if (cur_time - m_update_time >= digital_io_poll_time)
    {
      m_update_time = cur_time;
      sync_1 ();
    }
  }

  void sync (void)
  {
    m_update_time = std::chrono::high_resolution_clock::now ();
    sync_1 ();
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  virtual void write_port (unsigned int n, bool val) override
  {
    if (n < port_count)
      write (read ().set (n, val));
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }


private:
  pca9698_t& m_pca9698;

  std::bitset<port_count> m_outputs;
  std::chrono::high_resolution_clock::time_point m_update_time;

  std::bitset<port_count> m_and_mask;
  std::bitset<port_count> m_or_mask;
  std::bitset<port_count> m_xor_mask;

  void sync_1 (void)
  {
    const auto cur_outputs = m_outputs;

    using utils::set_bit;
    using utils::get_bit;

    m_pca9698.template set_output_ports <0, 32> (cur_outputs.to_ulong ());
  }

};

} // namespace dev
#endif // includeguard_mcbv13_board_dev_digital_io_includeguard
