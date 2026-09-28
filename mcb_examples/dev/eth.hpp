/*

a generic ethernet device interface.

this interface could also be put into the net library, because it's used
by it.  but either way, dev and net will have a dependency in one way or the
other.  so it doesn't matter much in which library this interface resides.
at least for now.

the 'eth' device is a compound device of some sort.  it covers the
functionality of an ethernet mac and an ethernet phy.

*/

#ifndef includeguard_dev_eth_hpp_includeguard
#define includeguard_dev_eth_hpp_includeguard

#include <cstdint>
#include <array>

#include <utils/enum_bitset.hpp>

namespace dev
{

struct eth
{
  // reads the default mac address of this ethernet device, probably in
  // some board specific way.  for example, the mac address could be stored in
  // some eeprom or something like that.
  virtual const std::array<uint8_t, 6> default_mac_address (void) const noexcept = 0;

  virtual void set_mac_address (const std::array<uint8_t, 6>& val) noexcept = 0;
  virtual std::array<uint8_t, 6> mac_address (void) const noexcept = 0;

  // issue a device soft-reset.
  virtual void reset (void) noexcept = 0;

  // FIXME: add functions for reading a configuration eeprom which often stores the
  // preprogrammed mac address.  almost every ethernet controller has some
  // serial interface to connect an eeprom.

  enum struct link_speed : int8_t
  {
    invalid,
    m10,
    m100,
    m1000
  };

  typedef utils::enum_bitset<link_speed, link_speed::m10, link_speed::m1000> link_speed_set;

  enum struct link_duplex : int8_t
  {
    invalid,
    half,
    full
  };

  typedef utils::enum_bitset<link_duplex, link_duplex::half, link_duplex::full> link_duplex_set;

  enum struct link_status : int8_t
  {
    // the phy is enabled but the link is down.  either because the cable
    // is unplugged, or because there is a problem with the link partner
    // or selected link type.
    down,

    // the link is up
    up,

    // the phy is auto negotiating the link
    negotiating,
  };

  struct link_info
  {
    // the phy is enabled but the cable inputs/outputs are electrically
    // isolated.
    bool isolated : 1;

    link_status status;
    link_speed speed;
    link_duplex duplex;
  };

  virtual struct link_info link_info (void) const noexcept = 0;

  // restarts auto negotiation with the specified parameters.
  virtual void link_autonegotiate (link_speed_set allowed_speeds,
				   link_duplex_set allowed_duplex) noexcept = 0;

  // electrically isolate or unisolate cable pins.
  virtual void link_isolate (bool val) noexcept = 0;

  // explicitly set the link type, overriding the current link setting,
  // even if it has been auto negotiated.
  virtual void link_force_type (link_speed spd, link_duplex dpl) noexcept = 0;


  // best effort packet read and write.
  // if a packet can't be written because the transmit queue is full, drop it.
  // if a packet can't be read because the queue is empty, return 0.
  virtual void write (const void* data, unsigned int byte_count) noexcept = 0;
  virtual unsigned int read (void* data, unsigned int max_byte_count) noexcept = 0;

  // FIXME: best effort packet transmission might not always be a good thing.
  // e.g. ethercat master devices need to make sure that the packet is sent and
  // if it doesn't get sent, at least it needs to give some error feedback.

  // FIXME: add non-blocking packet send and receive interface and receive
  // callbacks.

  // FIXME: add functions to control mac address filter, multicast, boardcast,
  // promiscuous mode, loopback mode, pause frame stuff, magic packet and
  // wake-on-lan stuff.


  // FIXME: remove this and bury it into the PHY.
  virtual void check_link_status (void) const noexcept = 0;
};

} // namespace dev
#endif // includeguard_dev_eth_hpp_includeguard
