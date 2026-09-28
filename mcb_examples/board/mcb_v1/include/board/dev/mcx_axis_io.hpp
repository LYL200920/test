
/*

each MCX axis has some additional general purpose IOs.

XIN1   - MCX XALARM		YIN1   - MCX YALARM
XIN2   - MCX XINPOS		YIN2   - MCX YINPOS
XIN3   - EXP1_30		YIN3   - EXP1_34
XIN4   - EXP1_31		YIN4   - EXP1_35
XIN5   - EXP1_32		YIN5   - EXP1_36
XIN6   - EXP1_33		YIN6   - EXP1_37

XOUT1  - MCX XPIO0		YOUT1  - MCX YPIO0
XOUT2  - MCX XPIO1		YOUT2  - MCX YPIO1
XOUT3  - MCX XPIO2		YOUT3  - MCX YPIO2
XOUT4  - MCX XPIO3		YOUT4  - MCX YPIO3
XOUT5  - MCX XPIO4		YOUT5  - MCX YPIO4
XOUT6  - MCX XPIO5		YOUT6  - MCX YPIO5
XOUT7  - MCX XPIO6		YOUT7  - MCX YPIO6
XOUT8  - MCX XPIO7		YOUT8  - MCX YPIO7
XOUT9  - MCX XDCC		YOUT9  - MCX YDCC
XOUT10 - EXP2_30		YOUT10 - EXP2_31

--

ZIN1   - MCX ZALARM		UIN1   - MCX UALARM
ZIN2   - MCX ZINPOS		UIN2   - MCX UINPOS
ZIN3   - EXP1_40		UIN3   - EXP1_44
ZIN4   - EXP1_41		UIN4   - EXP1_45
ZIN5   - EXP1_42		UIN5   - EXP1_46
ZIN6   - EXP1_43		UIN6   - EXP1_47

ZOUT1  - MCX ZPIO0		UOUT1  - MCX UPIO0
ZOUT2  - MCX ZPIO1		UOUT2  - MCX UPIO1
ZOUT3  - MCX ZPIO2		UOUT3  - MCX UPIO2
ZOUT4  - MCX ZPIO3		UOUT4  - MCX UPIO3
ZOUT5  - MCX ZPIO4		UOUT5  - MCX UPIO4
ZOUT6  - MCX ZPIO5		UOUT6  - MCX UPIO5
ZOUT7  - MCX ZPIO6		UOUT7  - MCX UPIO6
ZOUT8  - MCX ZPIO7		UOUT8  - MCX UPIO7
ZOUT9  - MCX ZDCC		UOUT9  - MCX UDCC
ZOUT10 - EXP2_32		UOUT10 - EXP2_33

*/

#ifndef includeguard_mcbv1_board_dev_mcx_axis_io_includeguard
#define includeguard_mcbv1_board_dev_mcx_axis_io_includeguard

#include <utils/bits.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

template <typename McxAxis, typename PCADev>
class mcx_axis_inputs final : public digital_io_port::dev_if
{
public:
  enum
  {
    // INx numbers as in the schematic / connector pinout
    in1 = 0,
    alarm = in1,

    in2 = 1,
    inpos = in2,

    in3 = 2,
    in4 = 3,
    in5 = 4,
    in6 = 5,

    // see mcx51x::signal_status_t bits
    stop0 = 8 + 0,
    stop1 = 8 + 1,
    stop2 = 8 + 2,
    enc_a = 8 + 3,
    enc_b = 8 + 4,
    enc_z = stop2,
    pos_limit = 8 + 7,
    neg_limit = 8 + 8,

    max_port_count
  };

  static constexpr unsigned int port_count = max_port_count;

  mcx_axis_inputs (McxAxis& mcx_axis, PCADev& pca)
  : m_mcx_axis (mcx_axis), m_pca (pca)
  {
    // read only the minimal number of 4 bits for each axis.
    if (mcx_axis.num () == 0)
      m_read_in3456 = [] (const PCADev& pca) { return pca.template input_ports<24, 28> (); };
    else if (mcx_axis.num () == 1)
      m_read_in3456 = [] (const PCADev& pca) { return pca.template input_ports<28, 32> (); };
    else if (mcx_axis.num () == 2)
      m_read_in3456 = [] (const PCADev& pca) { return pca.template input_ports<32, 36> (); };
    else if (mcx_axis.num () == 3)
      m_read_in3456 = [] (const PCADev& pca) { return pca.template input_ports<36, 40> (); };
    else
      assert_unreachable ();
  }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i >= port_count)
      return false;

    if (i <= stop0)
    {
      if (i == in1)
        return m_mcx_axis.signal_status ().alarm ();
      else if (i == in2)
        return m_mcx_axis.signal_status ().in_position ();
      else
      {
	// FIXME: also cache it in the PCA driver?
	return m_read_in3456 (m_pca)[i - 2];
      }
    }
    else if (i <= neg_limit)
      return utils::get_bit (m_mcx_axis.signal_status ().value (), i - stop0);
    else
      return false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  McxAxis& m_mcx_axis;
  PCADev& m_pca;

  // don't use std::function for smaller code
  std::bitset<4> (*m_read_in3456)(const PCADev&);
};

template <typename McxAxis, typename PCADev>
class mcx_axis_outputs final : public digital_io_port::dev_if
{
public:
  enum
  {
    out1 = 0,
    out2 = 1,
    out3 = 2,
    out4 = 3,
    out5 = 4,
    out6 = 5,
    out7 = 6,
    out8 = 7,
    out9 = 8,
    dcc = out9,
    out10 = 9,

    max_port_count
  };

  static constexpr unsigned int port_count = max_port_count;

  mcx_axis_outputs (McxAxis& mcx_axis, PCADev& pca, unsigned int pca_bit_offset)
  : m_mcx_axis (mcx_axis), m_pca (pca), m_pca_bit_offset (pca_bit_offset)
  {
    m_axis_pios_cache = m_mcx_axis.pios ();
  }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i >= port_count)
      return false;

    if (i == 9)
      return m_pca.template output_ports < 24, 28 > ()[m_pca_bit_offset];
    else if (i == 8)
    {
      // DCC output normally outputs pulses.  can't actually read the current
      // output state of the pin.
      return false;
    }
    else
      return utils::get_bit (m_axis_pios_cache, i);
  }

  virtual void write_port (unsigned int i, bool val) override
  {
    if (i >= port_count)
      return;

    if (i == 9)
    {
      auto bits = m_pca.template output_ports < 24, 28 > ();
      bits[m_pca_bit_offset] = val;
      m_pca.template set_output_ports < 24, 28 > (bits);
    }
    else if (i == 8 && val)
      m_mcx_axis.clear_deviation_counter ();
    else
    {
      m_axis_pios_cache = utils::set_bit (m_axis_pios_cache, i, val);
      m_mcx_axis.set_pios (m_axis_pios_cache);
    }
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  McxAxis& m_mcx_axis;
  PCADev& m_pca;
  const unsigned int m_pca_bit_offset;
  uint8_t m_axis_pios_cache;
};



} // namespace dev
#endif // includeguard_mcbv1_board_dev_mcx_axis_io_includeguard
