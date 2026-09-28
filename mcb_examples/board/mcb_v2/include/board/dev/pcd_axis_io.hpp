
/*

each PCD axis has some additional general purpose IOs.

FAULTx (in) - PCD STPx		FAULTy (in) - PCD STPy
U/Bx (in)   - PCD U/Bx		U/By (in)   - PCD U/By
F/Hx (in)   - PCD F/Hx		F/Hy (in)   - PCD F/Hy
STAx (in)   - PCD STAx		STAy (in)   - PCD STAy

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

ENz (out)   - PCD OTSz		ENu (out)   - PCD OTSu
P1z (out)   - PCD P1z		P1u (out)   - PCD P1u
P2z (out)   - PCD P2z		P2u (out)   - PCD P2u
P3z (out)   - PCD P3z		P3u (out)   - PCD P3u
P4z (out)   - PCD P4z		P4u (out)   - PCD P4u

STPx, STPy, STPz, STPu are logically ORed with the EMG_INT signal.
if EMG_INT is on we can't tell whether STP is on or off.
assume that STP is off when EMG_INT is on.

in addition to that each axis has some sensor inputs, which can also
be used as general-purpose inputs for software use.
  ORG
  +SD (PSD)
  -SD (MSD)
  +EL (PEL)
  -EL (MEL)

note that the limit sensors (+EL, -EL) always stop the respective axis
pulse output.  this is a hardwired logic in the PCD chip and can't be changed
in software.
*/

#ifndef includeguard_mcbv2_board_dev_pcd_axis_io_includeguard
#define includeguard_mcbv2_board_dev_pcd_axis_io_includeguard

#include <dev/digital_io_port.hpp>
#include <dev/renesas/rx_gpio.hpp>

#include <utils/langcomp.hpp>
#include <utils/byte_order.hpp>
#include <utils/bits.hpp>

namespace dev
{

template <typename PcdAxisFunc>
class pcd_axis_inputs final : public digital_io_port::dev_if
{
public:
  enum
  {
    stp = 0,
    ub = 1,
    fh = 2,

    // see pcd ext_status_t for bit number assignment
    mel = 4 + 0,
    pel = 4 + 1,
    org = 4 + 2,
    sta = 4 + 4,
    msd = 4 + 5,
    psd = 4 + 6,

    max_port_count
  };

  static constexpr unsigned int port_count = max_port_count;

  auto&& pcd_axis (void) const { return PcdAxisFunc () (this); }

  pcd_axis_inputs (void) { }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (i <= fh)
    {
      switch (i)
      {
	default: return false;

	// EMG_INT is active-low.
	// STP = 0, EMG_INT = 0 -> 0
	// STP = 0, EMG_INT = 1 -> 0
	// STP = 1, EMG_INT = 0 -> 0
	// STP = 1, EMG_INT = 1 -> 1
	case stp: return pcd_axis ().ext_status ().stp ()
			 & ((dev::rx_gpio::pidr::p3.read () & (1 << 2)) != 0);

	case ub: return utils::get_bit (pcd_axis ().riop (), 4);
	case fh: return utils::get_bit (pcd_axis ().riop (), 5);
      }
    }
    else if (i < max_port_count)
      return utils::get_bit (pcd_axis ().ext_status ().value (), i - 4);
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
    p1 = 1,
    p2 = 2,
    p3 = 3,
    p4 = 4,

    max_port_count
  };

  static constexpr unsigned int port_count = max_port_count;

  auto&& pcd_axis (void) const { return PcdAxisFunc () (this); }

  pcd_axis_outputs (void)
  {
    m_riop_cache = pcd_axis ().riop ();
  }

  void sync (void) { }

  virtual bool read_port (unsigned int i) const override
  {
    if (unlikely (i >= port_count))
      return false;

    if (i == 0)
      return pcd_axis ().control_mode ().ots_output ();

    else
      return utils::get_bit (m_riop_cache, i - 1);
  }

  // pcd_axis_outputs_axis_funcEE10write_portEjb
  virtual void write_port (unsigned int i, bool val) override
  {
    // unfortunately, the compiler never devirtualizes this function.
    // presumably because the object is buried in a array?

    // there are only 5 valid bits.  bit 0 is in another cached register
    // of the pcd axis object.
    auto&& axis = pcd_axis ();

    if (i == 0)
    {
      utils::atomic_set_bit (val, 4, axis.control_mode_cache_var ());
      axis.control_mode_write_cache_var ();
    }
    else
    {
      utils::atomic_set_bit (val, i - 1, &m_riop_cache);
      axis.set_riop (m_riop_cache);
    }
  }

  digital_io_port operator [] (unsigned int n)
  {
    if (unlikely (n >= port_count))
      return { &digital_io_port::g_null_dev, n };

    return { this, n };
  }

private:
  static_assert (port_count <= 8);
  volatile uint8_t m_riop_cache;
};


// ------------------------------------------------------------------------
// U/B and F/H axis inputs are located in a single PCD register RIOP.
// they can be used as soft-encoder inputs by sampling the signals periodically
// with the DTC.

// the practical hardware limit for the maximum input signal frequency is
// around 33 kHz.  because it needs to support AB phase quadrature pulses with
// quad edge evaluation the signals need to be 4x oversampled.

// the recorded U/B (bit 4) and F/H (bit 5) bit states are packed into
// a sample block with the following format

// 32 bit word 0:
//    bit 0: U/B sample 0
//    bit 1: F/H sample 0
//    bit 2: U/B sample 1
//    bit 3: F/H sample 1
//    ...
//    bit 30: U/B sample 15
//    bit 31: F/H sample 15

// 32 bit word 1:
//    bit 0: U/B sample 16
//    bit 1: F/H sample 16
//    ...
//
// this gives a continuous bit stream of interleaved input channels.
// it can be used buy the upper layer signal decoder to process the bits
// in SIMD-like fashion efficiently.

template < typename PcdBusInterface, typename DtcFunc, typename TimerChannelFunc >
class pcd_axis_inputs_sampler
{
  using dtc = std::remove_reference_t<std::invoke_result_t<DtcFunc>>;
  static constexpr auto& dtc_inst (void) { return std::invoke (DtcFunc ()); }

  using timer = std::remove_reference_t<std::invoke_result_t<TimerChannelFunc>>;
  static constexpr auto& timer_inst (void) { return std::invoke (TimerChannelFunc ()); }

  using dtc_insn = typename dtc::insn;

  using timer_trigger_point_value_type =
    typename std::remove_reference<decltype (timer_inst ().trigger_hwreg ())>::type::base_type;


public:
  constexpr static unsigned int pcd_axis_count = 4;
  constexpr static unsigned int sample_buffer_length = 16;

  constexpr static unsigned int sample_base_freq_hz = 32'000;
  constexpr static unsigned int oversampling = 4;

  using sample_block_t = uint32_t;
  constexpr static unsigned int bits_per_sample = 2;      // = number of signal lines
  constexpr static unsigned int samples_per_block = 16;

  constexpr static unsigned int sample_blocks_per_clb = sample_buffer_length / 16;

  using buffer_full_clb_t = std::function<void (const sample_block_t*, unsigned int)>;

  pcd_axis_inputs_sampler (PcdBusInterface& pcd_bus_if)
  {
    auto& timer = timer_inst ();
    auto& dtc = dtc_inst ();

    // at each timer tick, sample all 4 axes at once.
    // this is faster in total, but blocks the DTC for the whole duration
    // of sampling all 4 axes.
    // PCD register selection and register read accesses can be overlapped
    // for each axis to avoid the PCD wait states.

    // set combf reg sel flags for all 4 axes
    // (2+1) + (4*2+1) + (2) + (2+1) + (2) + 2 = 21 iclk = 87.5 ns @ 240 MHz
    m_dtc_insns[0] = dtc_insn ()
      .set_src_addr (&m_riop_reg_num).set_src_addr_mode (dev::dtca::fixed)
      .set_dst_addr (&(pcd_bus_if.last_combf_reg_sel_values ())).set_dst_addr_mode (dev::dtca::fixed)
      .template set_element_size < uint32_t > ()
      .set_transfer_mode (dev::dtca::normal)
      .set_normal_transfer_count (0)			// it's a chain transfer so
      .set_chain_mode (dev::dtca::always_chain);	// the transfer count doesn't matter

    // select pcd read register for axis 0..3
    // they do not cause pcd wait states and effectively run in parallel.
    // (4*2+1) + (2) + (2+1) + (2) + 2 = 18 iclk = 55.6 ns @ 240 MHz
    for (unsigned int i = 0; i < pcd_axis_count; ++i)
      m_dtc_insns[1 + i] = dtc_insn ()
	.set_src_addr (&(pcd_bus_if.combf_reg_sel_riop_value (i))).set_src_addr_mode (dev::dtca::fixed)
	.set_dst_addr (pcd_bus_if.combf_reg_addr (i)).set_dst_addr_mode (dev::dtca::fixed)
	.template set_element_size < uint8_t > ()
	.set_transfer_mode (dev::dtca::normal)
	.set_normal_transfer_count (0)			// it's a chain transfer so
	.set_chain_mode (dev::dtca::always_chain);	// the transfer count doesn't matter

    // read pcd rb0 register for axis 0..3
    // use a repeat transfer and let DTC automatically roll back the
    // counters and write pointers.
    // (4*2+1) + (2) + (2*0+1) + (2) + 2 = 16 iclk = 62.5 ns @ 240 MHz
    // 62.5 + 42 ns pcd read = 104.5 ns
    for (unsigned int i = 0; i < pcd_axis_count; ++i)
      m_dtc_insns[1 + pcd_axis_count + i] = dtc_insn ()
	.set_src_addr (pcd_bus_if.rb0_reg_addr (i)).set_src_addr_mode (dev::dtca::fixed)
	.set_dst_addr (m_riop_values.data () + sample_buffer_length*i).set_dst_addr_mode (dev::dtca::post_inc)
	.template set_element_size < uint8_t > ()
	.set_transfer_mode (dev::dtca::repeat)
	.set_dst_area_repeat ()
	.set_repeat_transfer_count (sample_buffer_length)
	.set_repeat_transfer_current_count (sample_buffer_length)
	.set_chain_mode (dev::dtca::always_chain);

    // when the last pcd register read transfer insn (4th axis) transfer count
    // drops to 1, it will execute one more insn.
    m_dtc_insns[1 + pcd_axis_count*2 - 1].set_chain_mode (dev::dtca::conditional_chain);

    // dummy transfer insn which is executed every 'sample_buffer_length' times
    // to trigger the CPU ISR.  alternatively could also roll back the pointers
    // and transfer count and re-enable DTC (set ICU.DTCERn) in the ISR, but
    // it would take about the same amount of cycles.
    // so better let the DTC do it.
    // (4*2+1) + (2) + (2+1) + (2) + 2 = 18 iclk = 55.6 ns @ 240 MHz
    m_dtc_insns[1 + pcd_axis_count*2] = dtc_insn ()
	.set_src_addr (&m_dtc_dummy).set_src_addr_mode (dev::dtca::fixed)
	.set_dst_addr (&m_dtc_dummy).set_dst_addr_mode (dev::dtca::fixed)
	.template set_element_size <uint8_t> ()
	.set_transfer_mode (dev::dtca::repeat)
	.set_dst_area_repeat ()
	.set_repeat_transfer_count (1)
	.set_repeat_transfer_current_count (1)
	.set_cpu_irq_at_every_transfer ()
	.set_chain_mode (dev::dtca::no_chain);

    // for sample_buffer_length = 16
    // total time = 87.5 ns + 4*55.6 ns + 4*104.5 ns + 55.6/16 ns = 732 ns = 1.366 MHz
    // = 732 / 4 = 183 ns per axis

    // there are other ways to do it, like with 32 bit accesses to PCD and
    // post-inc address modes in DTC block transfers or DTC repeat transfers.
    // however, overall they are all slower.

    dtc.template insns < typename timer::cmi_interrupt_line > ()
	= m_dtc_insns.data ();

    std::atomic_signal_fence (std::memory_order_release);

    dtc.template enable < typename timer::cmi_interrupt_line > ();


    timer.set_timer_control (dev::rx_cmt::cmcr_t ()
	.set_clock_select (dev::rx_cmt::pclk_8)
	.set_match_interrupt_enabled ());

    using namespace std::chrono_literals;
    using timer_duration = typename timer::duration;


    constexpr unsigned int timer_period_hz = sample_base_freq_hz * oversampling;

    constexpr timer_duration d = std::chrono::duration_cast<timer_duration> (
	std::chrono::duration<unsigned int, std::ratio <1, timer_period_hz>> (1));

    timer.set_counter (timer_duration (0));
    timer.set_trigger (d, dev::timer::enable);

    for (unsigned int i = 0; i < pcd_axis_count; ++i)
      set_buffer_full_clb (i, nullptr);
  }

  // the ISR function is public because it's invoked by an external timer
  // fixed-isr adapter, setup in the board class.  it results in more efficient
  // interrupt code.
  void invoke_isr (void)
  {
    // repack the whole buffer, data from all axes, all at once
    // into a temporary buffer on the stack, then invoke the user callbacks.
    // this makes the buffers available for overwriting by DTC, just in case
    // the user callback takes some time to process the data.

    std::array<uint32_t, sample_buffer_length * pcd_axis_count / 16> packed_sample_words;

    // be careful about ISR stack size limitations.
    static_assert (sizeof (packed_sample_words) <= 512);

    volatile uint32_t* src_ptr = (volatile uint32_t*)m_riop_values.data ();
    uint32_t* dst_ptr = packed_sample_words.data ();

    for (unsigned int i = 0; i < packed_sample_words.size (); ++i)
    {
      // recorded samples in memory, read into registers (little endian)
      // one sample = 2 bits
      static_assert (utils::native_byte_order () == utils::little_endian);
      static_assert (std::is_same<sample_block_t, uint32_t>::value);

      //	  s3	    s2        s1        s0
      // x0	..||....  ..||....  ..||....  ..||....

      //	  s7	    s6        s5        s4
      // x1	..||....  ..||....  ..||....  ..||....

      //	  s11	    s10       s9        s8
      // x2	..||....  ..||....  ..||....  ..||....

      //	  s15	    s14      s13       s12
      // x3	..||....  ..||....  ..||....  ..||....

      uint32_t x0 = *src_ptr++;
      uint32_t x1 = *src_ptr++;
      uint32_t x2 = *src_ptr++;
      uint32_t x3 = *src_ptr++;

      // merge 16 2-bit samples into one 32 bit word

      // bit idx	31 30  29 28  27 26  25 24  23 22  21 20  19 18  17 16  15 14  13 12  11 10   9  8   7  6   5  4   3  2   1  0
      // sample		 s15     s11    s7     s3    s14    s10     s6     s2    s13     s9     s5     s1    s12     s8     s4     s0

      uint32_t x = ((x0 & 0x30303030) >> 4) | ((x1 & 0x30303030) >> 2) | ((x2 & 0x30303030) >> 0) | ((x3 & 0x30303030) << 2);

      // permute/shuffle vector with source bit indices (MSB first)

      // src bit idx	31 30  23 22  15 14   7  6  29 28  21 20  13 12   5  4  27 26  19 18  11 10   3  2  25 24  17 16   9  8   1  0
      // sample		 s15     s14    s13    s12    s11    s10    s9      s8    s7     s6     s5      s4    s3      s2     s1     s0

      // http://programming.sirrida.de/calcperm.php

      x = utils::bit_permute_step (x, 0x00cc00cc, 6);
      x = utils::bit_permute_step (x, 0x0000f0f0, 12);

      // in total, it takes about 23 insns on RX, excl. the 4 mem loads

      *dst_ptr++ = x;
    }

    #pragma GCC unroll pcd_axis_count
    for (unsigned int i = 0; i < pcd_axis_count; ++i)
    {
      assume_always_true (m_buffer_callbacks[i]);

      m_buffer_callbacks[i] (packed_sample_words.data () + i*sample_buffer_length/16, sample_buffer_length/16);
    }
  }

  void start (unsigned int axis)
  {
    if (axis < pcd_axis_count)
      m_axis_enabled[axis] = true;

    if (m_axis_enabled.any ())
      timer_inst ().start ();
  }

  void stop (unsigned int axis)
  {
    if (axis < pcd_axis_count)
      m_axis_enabled[axis] = false;

    if (m_axis_enabled.none ())
      timer_inst ().stop ();
  }

  bool is_running (unsigned int axis) const
  {
    return axis < pcd_axis_count ? m_axis_enabled[axis] : false;
  }

  void set_buffer_full_clb (unsigned int axis, std::nullptr_t)
  {
    if (axis < pcd_axis_count)
      m_buffer_callbacks[axis] = [] (const sample_block_t*, unsigned int) { };

    std::atomic_signal_fence (std::memory_order_release);
  }

  template <typename Func>
  void set_buffer_full_clb (unsigned int axis, Func&& func)
  {
    if (axis < pcd_axis_count)
    {
      // we can't atomically update the function pointer and the user_p argument.
      // thus, first set the function to a null dummy function, then set the
      // user_p, then the real function pointer.

      // this relies on the implementation of std::function and on the order
      // in which it initializes/copies its fields.  the actual function pointer
      // has to be copied as a last value for this to work.
      set_buffer_full_clb (axis, nullptr);
      std::atomic_signal_fence (std::memory_order_release);

      m_buffer_callbacks[axis] = std::forward<Func> (func);
      std::atomic_signal_fence (std::memory_order_release);
    }
  }

private:
  std::bitset<pcd_axis_count> m_axis_enabled;

  // N.B. using a custom callback function thingy is more compact and
  // uses fewer instructions for the invocation.
  std::array<buffer_full_clb_t, pcd_axis_count> m_buffer_callbacks;

  std::array<uint8_t, sample_buffer_length * pcd_axis_count> m_riop_values;

  std::array<dtc_insn, 10> m_dtc_insns;

  const uint32_t m_riop_reg_num =
	  (pcd4641::regno_riop << 24) | (pcd4641::regno_riop << 16)
	| (pcd4641::regno_riop << 8) | (pcd4641::regno_riop << 0);

  uint8_t m_dtc_dummy;
};


} // namespace dev
#endif // includeguard_mcbv2_board_dev_pcd_axis_io_includeguard
