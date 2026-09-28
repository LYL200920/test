
/*

each PCD axis has some additional general purpose IOs.

FAULTx (in) - PCD STPx		FAULTy (in) - PCD STPy
U/Bx (in)   - PCD U/Bx		U/By (in)   - PCD U/By
F/Hx (in)   - PCD F/Hx		F/Hy (in)   - PCD F/Hy
STAx (in)   - PCD STAx		STAy (in)   - PCD STAy
+SDx (in)   - PCD +SDx		+SDy (in)   - PCD +SDy
-SDx (in)   - PCD -SDx		-SDy (in)   - PCD -SDy

ENx (out)   - PCD OTSx		ENy (out)   - PCD OTSy
P1x (out)   - PCD P1x		P1y (out)   - PCD P1y
P2x (out)   - PCD P2x		P2y (out)   - PCD P2y
P3x (out)   - PCD P3x		P3y (out)   - PCD P3y
P4x (out)   - PCD P4x		P4y (out)   - PCD P4y

--

FAULTz (in) - PCD STPz		FAULTu (in) - PCD STPu
U/Bz (in)   - PCD U/Bz		U/Bu (in)   - PCD U/Bu
F/Hz (in)   - PCD F/Hz		F/Hu (in)   - PCD F/Hu
STAz (in)   - PCD STAz		STAu (in)   - PCD STAu
+SDz (in)   - PCD +SDz		+SDu (in)   - PCD +SDu
-SDz (in)   - PCD -SDz		-SDu (in)   - PCD -SDu

ENz (out)   - PCD OTSz		ENu (out)   - PCD OTSu
P1z (out)   - PCD P1z		P1u (out)   - PCD P1u
P2z (out)   - PCD P2z		P2u (out)   - PCD P2u
P3z (out)   - PCD P3z		P3u (out)   - PCD P3u
P4z (out)   - PCD P4z		P4u (out)   - PCD P4u

*/

#ifndef includeguard_mcbv13_board_dev_pcd_axis_io_includeguard
#define includeguard_mcbv13_board_dev_pcd_axis_io_includeguard

#include <dev/digital_io_port.hpp>

namespace dev
{

template <typename PcdAxis>
class pcd_axis_inputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 6;

  pcd_axis_inputs (PcdAxis& pcd_axis) : m_pcd_axis (pcd_axis) { }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    switch (i)
    {
      default: return false;
      case 0: return m_pcd_axis.ext_status ().stp ();
      case 1: return utils::get_bit (m_pcd_axis.riop (), 4);
      case 2: return utils::get_bit (m_pcd_axis.riop (), 5);
      case 3: return m_pcd_axis.ext_status ().sta ();
      case 4: return m_pcd_axis.ext_status ().psd ();
      case 5: return m_pcd_axis.ext_status ().msd ();
    }
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  PcdAxis& m_pcd_axis;
};

template <typename PcdAxis>
class pcd_axis_outputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 5;

  pcd_axis_outputs (PcdAxis& pcd_axis) : m_pcd_axis (pcd_axis)
  {
    m_cache[0] = m_pcd_axis.ext_status ().ots ();

    std::bitset<4> p (m_pcd_axis.riop ());
    m_cache[1] = p[0];
    m_cache[2] = p[1];
    m_cache[3] = p[2];
    m_cache[4] = p[3];
  }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i >= port_count)
      return false;

    return m_cache[i];
  }

  virtual void write_port (unsigned int i, bool val) override
  {
    if (i >= port_count)
      return;

    m_cache[i] = val;

    if (i == 0)
      m_pcd_axis.set_control_mode (
	m_pcd_axis.control_mode ().set_ots_output (val));
    else
      m_pcd_axis.set_riop ((uint8_t)(m_cache.to_ulong () >> 1));
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  PcdAxis& m_pcd_axis;
  std::bitset<port_count> m_cache;
};



} // namespace dev
#endif // includeguard_mcbv13_board_dev_pcd_axis_io_includeguard
