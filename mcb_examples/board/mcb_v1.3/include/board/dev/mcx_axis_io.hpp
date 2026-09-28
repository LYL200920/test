
/*

each MCX axis has some additional general purpose IOs.

XIN1   - MCX XALARM		YIN1   - MCX YALARM
XIN2   - MCX XINPOS		YIN2   - MCX YINPOS
XIN3   - MCX PIN0		YIN3   - MCX PIN1
XIN4   - MCX XPIO4		YIN4   - MCX YPIO4
XIN5   - MCX XPIO0		YIN5   - MCX YPIO0
XIN6   - MCX XPIO1		YIN6   - MCX YPIO1
XIN7   - MCX XPIO5		YIN7   - MCX YPIO5

XOUT1  - MCX XPIO6		YOUT1  - MCX YPIO6
XOUT2  - MCX XPIO2		YOUT2  - MCX YPIO2
XOUT3  - MCX XPIO7		YOUT3  - MCX YPIO7
XOUT4  - MCX XPIO3		YOUT4  - MCX YPIO3
XOUT5  - EXP1_40		YOUT5  - EXP1_44
XOUT6  - EXP1_41		YOUT6  - EXP1_45
XOUT7  - EXP1_42		YOUT7  - EXP1_46
XOUT8  - EXP1_43		YOUT8  - EXP1_47
XOUT9  - MCX XDCC		YOUT9  - MCX YDCC

--

ZIN1   - MCX ZALARM		UIN1   - MCX UALARM
ZIN2   - MCX ZINPOS		UIN2   - MCX UINPOS
ZIN3   - MCX PIN2		UIN3   - MCX PIN3
ZIN4   - MCX ZPIO4		UIN4   - MCX UPIO4
ZIN5   - MCX ZPIO0		UIN5   - MCX UPIO0
ZIN6   - MCX ZPIO1		UIN6   - MCX UPIO1
ZIN7   - MCX ZPIO5		UIN7   - MCX UPIO5

ZOUT1  - MCX ZPIO6		UOUT1  - MCX UPIO6
ZOUT2  - MCX ZPIO2		UOUT2  - MCX UPIO2
ZOUT3  - MCX ZPIO7		UOUT3  - MCX UPIO7
ZOUT4  - MCX ZPIO3		UOUT4  - MCX UPIO3
ZOUT5  - EXP2_40		UOUT5  - EXP2_44
ZOUT6  - EXP2_41		UOUT6  - EXP2_45
ZOUT7  - EXP2_42		UOUT7  - EXP2_46
ZOUT8  - EXP2_43		UOUT8  - EXP2_47
ZOUT9  - MCX ZDCC		UOUT9  - MCX UDCC

*/

#ifndef includeguard_mcbv13_board_dev_mcx_axis_io_includeguard
#define includeguard_mcbv13_board_dev_mcx_axis_io_includeguard

#include <utils/bits.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

template <typename McxAxis>
class mcx_axis_inputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 7;

  mcx_axis_inputs (McxAxis& mcx_axis) : m_mcx_axis (mcx_axis) { }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    switch (i)
    {
      case 0: return m_mcx_axis.signal_status ().alarm ();
      case 1: return m_mcx_axis.signal_status ().in_position ();
      case 2: return utils::get_bit (m_mcx_axis.container ().gpi_values (), m_mcx_axis.num ());
      case 3: return utils::get_bit (m_mcx_axis.pios (), 4);
      case 4: return utils::get_bit (m_mcx_axis.pios (), 0);
      case 5: return utils::get_bit (m_mcx_axis.pios (), 1);
      case 6: return utils::get_bit (m_mcx_axis.pios (), 5);
      default: return false;
    }
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  McxAxis& m_mcx_axis;
};

template <typename McxAxis, typename PCADev0, typename PCADev1>
class mcx_axis_outputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 9;

  mcx_axis_outputs (McxAxis& mcx_axis, PCADev0& pca0, PCADev1& pca1,
		    unsigned int pca_idx)
  : m_mcx_axis (mcx_axis), m_pca0 (pca0), m_pca1 (pca1), m_pca_idx (pca_idx)
  {
    {
      const std::bitset<8> b (m_mcx_axis.pios ());
      m_cache[0] = b[6];
      m_cache[1] = b[2];
      m_cache[2] = b[7];
      m_cache[3] = b[3];
    }
    {
      const auto b = m_pca_idx & 0b10
		     ? m_pca1.template output_ports < 32, 40 > ()
		     : m_pca0.template output_ports < 32, 40 > ();

      const unsigned int i = (m_pca_idx & 0b01) << 4;

      m_cache[4] = b[i + 0];
      m_cache[5] = b[i + 1];
      m_cache[6] = b[i + 2];
      m_cache[7] = b[i + 3];
    }
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

    if (i == 8 && val)
      m_mcx_axis.clear_deviation_counter ();
    else
    {
      m_cache[i] = val;

      if (i < 4)
      {
	std::bitset<8> b;
	b[6] = m_cache[0];
	b[2] = m_cache[1];
	b[7] = m_cache[2];
	b[3] = m_cache[3];

	// this will overwrite unrelated PIO bits, but it doesn't matter
	// since they are configured as inputs.
	m_mcx_axis.set_pios (b.to_ulong ());
      }
      else
      {
	auto b = m_pca_idx & 0b10
		 ? m_pca1.template output_ports < 32, 40 > ()
		 : m_pca0.template output_ports < 32, 40 > ();

	const unsigned int ii = (m_pca_idx & 0b01) << 4;

	b[(i - 4) + ii] = val;

	m_pca_idx & 0b10
	? m_pca1.template set_output_ports < 32, 40 > (b)
	: m_pca0.template set_output_ports < 32, 40 > (b);
      }
    }
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  McxAxis& m_mcx_axis;
  PCADev0& m_pca0;
  PCADev1& m_pca1;
  const unsigned int m_pca_idx;
  std::bitset<port_count> m_cache;
};



} // namespace dev
#endif // includeguard_mcbv13_board_dev_mcx_axis_io_includeguard
