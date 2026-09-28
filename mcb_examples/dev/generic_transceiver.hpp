/*

a generic external transceiver control for some communication interface.
e.g. ISL317xx that is connected via general-purpose output ports.

*/

#ifndef includeguard_generic_transceiver_hpp_includeguard
#define includeguard_generic_transceiver_hpp_includeguard

namespace dev
{

struct default_dummy_transceiver
{
  // not sure if it's a good idea to have separate functions vs. flag bits
  // and a single function ... might need to change it later.
  void enable_external_receiver (void) { }
  void disable_external_receiver (void) { }

  void enable_external_transmitter (void) { }
  void disable_external_transmitter (void) { }
};


// a transceiver that is controlled via GPIO pins.
template <typename OutputPort> struct gpio_transceiver : private OutputPort
{
  void enable_external_receiver (void) { }
  void disable_external_receiver (void) { }

  void enable_external_transmitter (void) { OutputPort::set (); }
  void disable_external_transmitter (void) { OutputPort::clear (); }
};

} // namespace dev
#endif // includeguard_generic_transceiver_hpp_includeguard
