
/*

each MCX axis has some additional general purpose IOs.

the axis outputs are latches on the RX data bus and memory mapped in the CS7
memory area (0x0100'0000 - 0x01FF'FFFF).

to avoid conflicts with dipswitch readings, always set the unconnected address
bits PA6, PA6, PA7 during the bus access, i.e. use 0x0100'00E0 as a base
address for CS7.

RX CS7 area mapping:
0x00 - trigger outputs (lower 8 bit)
0x02 - digital outputs [15..0]
0x04 - digital outputs [31..16]
0x06 - mcx axis outputs (XOUT1..8, YOUT1..8)
0x08 - mcx axis outputs (ZOUT1..8, UOUT1..8)
0x0A - digital inputs [15..0]
0x0C - digital inputs [31..16]
0x0E - NAND flash RDY/BSY (read only)


0x0100'0006 low byte:  X axis outputs (XOUT1...XOUT8)
0x0100'0006 high byte: Y axis outputs (YOUT1...YOUT8)

0x0100'0008 low byte:  Z axis outputs (ZOUT1...ZOUT8)
0x0100'0008 high byte: U axis outputs (UOUT1...UOUT8)

in addition to those 8 outputs, there is one MCX DCC output.


the inputs are wired differently depending on whether MCX514 or MCX512 is used

XIN1   - MCX XALARM			YIN1   - MCX YALARM
XIN2   - MCX XINPOS			YIN2   - MCX YINPOS
XIN3   - MCX XPIO6			YIN3   - MCX YPIO6
XIN4   - MCX XPIO7			YIN4   - MCX YPIO7
XIN5   - MCX514 PIN0, MCX512 XPIO8	YIN5   - MCX514 PIN2, MCX512 YPIO8
XIN6   - MCX514 PIN1, MCX512 XPIO9	YIN6   - MCX514 PIN3, MCX512 YPIO9

--

ZIN1   - MCX ZALARM			UIN1   - MCX UALARM
ZIN2   - MCX ZINPOS			UIN2   - MCX UINPOS
ZIN3   - MCX ZPIO6			UIN3   - MCX UPIO6
ZIN4   - MCX ZPIO7			UIN4   - MCX UPIO7
ZIN5   - MCX514 PIN4			UIN5   - MCX514 PIN6
ZIN6   - MCX514 PIN5			UIN6   - MCX514 PIN7


there are also 4 trigger inputs and 2 trigger outputs on each axis PIOs:
  PIO0_IN
  PIO1_IN
  PIO4_IN
  PIO5_IN
  PIO2_OUT
  PIO3_OUT

and 5 sensor inputs on each axis, which can also be used as general purpose
inputs.
  STOP0
  STOP1
  STOP2
  +LIMT
  -LIMT

*/

#ifndef includeguard_mcbv2_board_dev_mcx_axis_io_includeguard
#define includeguard_mcbv2_board_dev_mcx_axis_io_includeguard

#include <utils/langcomp.hpp>
#include <utils/bits.hpp>
#include <utils/byte_order.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

template <typename McxAxis>
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

    trigger_pio0 = 8 + 0,
    trigger_pio1 = 8 + 1,
    trigger_pio4 = 8 + 4,
    trigger_pio5 = 8 + 5,

    // see mcx51x::signal_status_t bits
    stop0 = 16 + 0,
    stop1 = 16 + 1,
    stop2 = 16 + 2,
    enc_a = 16 + 3,
    enc_b = 16 + 4,
    enc_z = stop2,
    pos_limit = 16 + 7,
    neg_limit = 16 + 8,

    max_port_count
  };

  static constexpr unsigned int port_count = max_port_count;

  mcx_axis_inputs (McxAxis& mcx_axis) : m_mcx_axis (mcx_axis) { }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i < trigger_pio0)
    {
      switch (i)
      {
	case in1: return m_mcx_axis.signal_status ().alarm ();
	case in2: return m_mcx_axis.signal_status ().in_position ();
	case in3: return utils::get_bit (m_mcx_axis.pios (), 6);
	case in4: return utils::get_bit (m_mcx_axis.pios (), 7);

	#if defined (MCB_USE_MCX514)
	  case in5: return utils::get_bit (m_mcx_axis.container ().gpi_values (),
					   m_mcx_axis.num () * 2 + 0);
	  case in6: return utils::get_bit (m_mcx_axis.container ().gpi_values (),
					   m_mcx_axis.num () * 2 + 1);
	#elif defined (MCB_USE_MCX512)
	  case in5: return utils::get_bit (m_mcx_axis.pios (), 8);
	  case in6: return utils::get_bit (m_mcx_axis.pios (), 9);
	#else
	  // nothing.  probably MCX functions are not enabled in the BSP but
	  // but the file is still included.
	#endif
	default: return false;
      }
    }
    else if (i < stop0)
    {
      return utils::get_bit (m_mcx_axis.pios (), i - trigger_pio0);
    }
    else if (i <= neg_limit)
    {
      return utils::get_bit (m_mcx_axis.signal_status ().value (), i - stop0);
    }
    else
      return false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  McxAxis& m_mcx_axis;
};

// axis outputs are memory mapped
// 0x0100'0006 - mcx axis outputs (XOUT1..8, YOUT1..8)
// 0x0100'0008 - mcx axis outputs (ZOUT1..8, UOUT1..8)
//
// however, we can't access individual bytes, which is needed for
// individual axis output writes.  to do that, the outputs are cached
// in RAM outside the mcx_axis_outputs class (e.g. in the board class).
//

template <typename McxAxis>
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

    trigger_pio2 = 16 + 2,
    trigger_pio3 = 16 + 3,

    max_port_count
  };

  static constexpr unsigned int port_count = max_port_count;

  mcx_axis_outputs (McxAxis& mcx_axis, uint32_t& all_outputs_cache)
  : m_mcx_axis (mcx_axis), m_all_outputs_cache (all_outputs_cache)
  {
    // after reset all outputs are reset to zero/off in hardware.
    all_outputs_cache = 0;
    m_pio_outputs_cache = 0;
  }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i <= out8)
      return utils::get_bit (m_all_outputs_cache, m_mcx_axis.num () * 8 + i);

    else if (i & 16)
    {
      // always read the pio outputs state from the chip.
      // if it is in pulse-output mode, we can get the actual output state
      // of the pin like that.  maybe it's useful.
      return utils::get_bit (m_mcx_axis.pios (), i & 0b11);
    }
    return false;
  }


  // S4_20mcx_axis_inputs_funcENS4_21mcx_axis_outputs_funcEE6axis_tEE10write_portEjb
  virtual void write_port (unsigned int i, bool val) override
  {
    if (likely (i < dcc))
    {
      const unsigned int byte_num = m_mcx_axis.num () ^ (utils::native_byte_order () == utils::big_endian ? 0b11 : 0b00);
      const unsigned int word_num = byte_num / 2;

      volatile uint8_t* byte_ptr = (volatile uint8_t*)((uintptr_t)&m_all_outputs_cache + byte_num);
      volatile uint16_t* const base_reg_addr = (volatile uint16_t*)0x0100'00E6;

      utils::atomic_set_bit (val, i, byte_ptr);
      std::atomic_signal_fence (std::memory_order_release);
      *(base_reg_addr + word_num) = *((volatile uint16_t*)&m_all_outputs_cache + word_num);
    }
    else if (likely (i & 16))
    {
      // just write all the PIO values.  only PIO2 and PIO3 are configured
      // as outputs.  so writing to the other bits has no effect.

      utils::atomic_set_bit (val, i, &m_pio_outputs_cache);
      m_mcx_axis.set_pios (m_pio_outputs_cache);
    }
    else if (i == dcc && val)
    {
      // the DCC pin can output only a pulse when it is written to.
      // can't control the absolute state of the output in software.
      m_mcx_axis.clear_deviation_counter ();
    }
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  uint8_t m_pio_outputs_cache;
  McxAxis& m_mcx_axis;
  uint32_t& m_all_outputs_cache;
};



} // namespace dev
#endif // includeguard_mcbv13_board_dev_mcx_axis_io_includeguard
