
/*

each PCD axis has some additional general purpose IOs.

FAULTx (in) - PCD STPx		FAULTy (in) - PCD STPy

ENx (out)   - PCD OTSx		ENy (out)   - PCD OTSy

--

FAULTz (in) - PCD STPz		FAULTu (in) - PCD STPu

ENz (out)   - PCD OTSz		ENu (out)   - PCD OTSu

*/

#ifndef includeguard_mcbv1_board_dev_pcd_axis_io_includeguard
#define includeguard_mcbv1_board_dev_pcd_axis_io_includeguard

#include <dev/digital_io_port.hpp>

namespace dev
{

template <typename PcdAxisFunc>
class pcd_axis_inputs final : public digital_io_port::dev_if
{
public:
  enum
  {
    // see pcd ext_status_t for bit number assignment
    mel = 0,
    pel = 1,
    org = 2,
    stp = 3,

    max_port_count,

    // not connected, for source compatibility with mcb2
    // will always read zero.
    ub,
    fh,
    sta,
    msd,
    psd
  };

  static constexpr unsigned int port_count = max_port_count;

  auto&& pcd_axis (void) const { return PcdAxisFunc () (this); }

  pcd_axis_inputs (void) { }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i < max_port_count)
      return utils::get_bit (pcd_axis ().ext_status ().value (), i);
    else
      return false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
};

template <typename PcdAxisFunc>
class pcd_axis_outputs final : public digital_io_port::dev_if
{
public:
  enum
  {
    ots = 0,

    max_port_count,

    // not connected, for source compatibility with mcb2.
    // will do nothing when written.
    p1,
    p2,
    p3,
    p4
  };

  static constexpr unsigned int port_count = max_port_count;

  pcd_axis_outputs (void) { }

  auto&& pcd_axis (void) const { return PcdAxisFunc () (this); }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i >= port_count)
      return false;

    return pcd_axis ().ext_status ().ots ();
  }

  virtual void write_port (unsigned int i, bool val) override
  {
    if (i >= port_count)
      return;

    pcd_axis ().set_control_mode (
	pcd_axis ().control_mode ().set_ots_output (val));
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
};



} // namespace dev
#endif // includeguard_mcbv1_board_dev_pcd_axis_io_includeguard
