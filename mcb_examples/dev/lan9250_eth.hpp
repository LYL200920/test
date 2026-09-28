/*

eth interface adapter for LAN9250 driver.

*/

#ifndef includeguard_lan9250_eth_hpp_includeguard
#define includeguard_lan9250_eth_hpp_includeguard

#include <dev/eth_phy.hpp>
#include <dev/eth.hpp>

namespace dev
{

template < typename Lan9250_Mac, typename Lan9250_Phy, typename DefaultMacAddrFunc >
class lan9250_eth final : public eth
{
public:
  [[gnu::cold]] lan9250_eth (Lan9250_Mac& mac, Lan9250_Phy& phy)
  : m_mac (mac), m_phy (phy)
  {
  }

  virtual const std::array<uint8_t, 6>
  default_mac_address (void) const noexcept override
  {
    return DefaultMacAddrFunc () ();
  }

  [[gnu::cold]] virtual void set_mac_address (const std::array<uint8_t, 6>& val) noexcept override
  {
  }

  virtual std::array<uint8_t, 6> mac_address (void) const noexcept override
  {
    return { 0, 0, 0, 0, 0, 0};
  }

  [[gnu::cold]] virtual void reset (void) noexcept override
  {
  }

  [[gnu::cold]] virtual struct link_info link_info (void) const noexcept override
  {
    return { };
  }

  // explicitly set the link type, overriding the current link setting,
  // even if it has been auto negotiated.
  [[gnu::cold]] virtual void link_force_type (link_speed spd, link_duplex dpl) noexcept override
  {
    // FIXME: implement
  }

  [[gnu::cold]] virtual void
  link_autonegotiate (link_speed_set allowed_speeds,
		      link_duplex_set allowed_duplex) noexcept override
  {
  }

  [[gnu::cold]] virtual void link_isolate (bool val) noexcept override
  {
  }

  virtual void write (const void* data, unsigned int byte_count) noexcept override
  {

  }

  virtual unsigned int read (void* data, unsigned int max_byte_count) noexcept override
  {
    return 0;
  }

  [[gnu::cold]] virtual void check_link_status (void) const noexcept override
  {
  }

private:
  Lan9250_Mac& m_mac;
  Lan9250_Phy& m_phy;
};


} // namespace dev
#endif // includeguard_lan9250_eth_hpp_includeguard
