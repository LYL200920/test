/*

universal asynchronous/synchronous receiver/transmitter (USART)

*/

#ifndef includeguard_usart_hpp_includeguard
#define includeguard_usart_hpp_includeguard

#include <dev/generic_io.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

struct usart : public generic_io, public digital_io_port::dev_if
{
  enum struct flow_ctrl
  {
    none,
    rts_cts,
    dtr_dsr,
    xon_xoff,
    de_high,
    de_low
  };

  enum struct parity
  {
    none,
    odd,
    even
  };

  struct config
  {
    unsigned int baud_rate;  // bits per second
    enum parity parity;
    unsigned int start_stop_bits_4; // number of quarter bits, usually 4 or 8
    unsigned int data_bits;         // usually 7 or 8

    enum flow_ctrl flow_ctrl;
    unsigned char xon_char;       // usually 0x11
    unsigned char xoff_char;      // usually 0x13

    unsigned int timeout_ms;      // timeout in milliseconds

    constexpr config& set_baud_rate (unsigned int val) { baud_rate = val; return *this; }
    constexpr config& set (enum parity val) { parity = val; return *this; }
    constexpr config& set_start_stop_bits_4 (unsigned int val) { start_stop_bits_4 = val; return *this; }
    constexpr config& set_data_bits (unsigned int val) { data_bits = val; return *this; }
    constexpr config& set (enum flow_ctrl val) { flow_ctrl = val; return *this; }
    constexpr config& set_xon_char (unsigned char val) { xon_char = val; return *this; }
    constexpr config& set_xoff_char (unsigned char val) { xoff_char = val; return *this; }
    constexpr config& set_timeout_ms (unsigned int val) { timeout_ms = val; return *this; }
  };

  virtual bool is_valid (void) const noexcept = 0;

  virtual struct config config (void) const noexcept = 0;
  virtual bool set_config (const struct config& val) noexcept = 0;

  // access to IO pins of the serial port.
  enum struct io_port : unsigned int
  {
    // clear to send
    cts,

    // data set ready
    dsr,

    // data carrier detect
    dcd,

    // ring indicator
    ri,

    // receive data (current pin state, if supported)
    rxd,

    // request to send
    rts,

    // data terminal ready
    dtr,

    // transmit data (current pin state, if supported)
    txd
  };

  static constexpr unsigned int io_port_count = (unsigned int)io_port::txd + 1;

  digital_io_port operator [] (io_port n) { return { this, (unsigned int)n }; }

  // FIXME: this does not allow access t o several bits simultaneously.
  // might be a problem for some use cases.
  // these overloads will hide the virtual functions which is not what we want here.
  // bool read_port (io_port n) const { return digital_io_port::dev_if::read_port ((unsigned int)n); }
  // void write_port (io_port n, bool val) { digital_io_port::dev_if::write_port ((unsigned int)n, val); }

protected:
};

} // namespace dev
#endif // includeguard_usart_hpp_includeguard
