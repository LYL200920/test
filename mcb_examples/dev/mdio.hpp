
/*
  management data input/output (MDIO)
  serial management interface (SMI)
  media independent interface management (MIIM)

  STA = device that is driving an MDIO bus (master)
  MMD = MDIO manageable device (slave)

  default minimum MDC cycle time is 400 ns (2.5 MHz), but devices might be
  able to go faster.
*/

#ifndef includeguard_dev_mdio_hpp_includegard
#define includeguard_dev_mdio_hpp_includegard

namespace dev
{

struct mdio_sta
{
  virtual uint16_t mdio_read_reg (unsigned int mmd_addr, unsigned int cycle_speed_ns,
				  unsigned int reg) = 0;

  virtual void mdio_write_reg (unsigned int mmd_addr, unsigned int cycle_speed_ns,
			       unsigned int reg, uint16_t data) = 0;
};


/*
struct mdio_mmd
{
};
*/

};

#endif // includeguard_dev_mdio_hpp_includegard

