
/*

RX EtherC driver

this driver by itself is not very useful.  it has to be used together with
an EDMAC driver.

although internally the EtherC device has interrupts, the hardware does not
expose any interrupts directly to the CPU interface.  instead it relays all
interrupts to the EDMAC device.

*/

#ifndef includeguard_rx_etherc_hpp_includeguard
#define includeguard_rx_etherc_hpp_includeguard

#include <array>

#include <dev/hwreg.hpp>
#include <utils/value_range.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace rx_etherc
{




class mode_t
{
public:
  enum duplex_t
  {
    half_duplex = 0,
    full_duplex = 1
  };

  enum speed_t
  {
    speed_10m = 0,
    speed_100m = 1
  };

  constexpr mode_t (void) : m_value (0) { }
  explicit constexpr mode_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  bool promiscuous (void) const { return get_bit (0); }
  mode_t& set_promiscuous (bool val = true) { return set_bit (0, val); }

  duplex_t duplex (void) const { return (duplex_t)get_bit (1); }
  mode_t& set_duplex (duplex_t val) { return set_bit (1, val); }

  speed_t speed (void) const { return (speed_t)get_bit (2); }
  mode_t& set_speed (speed_t val) { return set_bit (2, val); }

  bool loop_back (void) const { return get_bit (3); }
  mode_t& set_loop_back (bool val = true) { return set_bit (3, val); }

  bool tx_enable (void) const { return get_bit (5); }
  mode_t& set_tx_enable (bool val = true) { return set_bit (5, val); }

  bool rx_enable (void) const { return get_bit (6); }
  mode_t& set_rx_enable (bool val = true) { return set_bit (6, val); }

  bool magic_packet_detection_enable (void) const { return get_bit (9); }
  mode_t& set_magic_packet_detection_enable (bool val = true) { return set_bit (9, val); }

  bool crc_error_frame_rx_enable (void) const { return get_bit (12); }
  mode_t& set_crc_error_frame_rx_enable (bool val = true) { return set_bit (12, val); }

  // transmitting port flow control
  bool pause_frame_tx_enable (void) const { return get_bit (16); }
  mode_t& set_pause_frame_tx_enable (bool val = true) { return set_bit (16, val); }

  // receiving port flow control
  bool pause_frame_rx_enable (void) const { return get_bit (17); }
  mode_t& set_pause_frame_rx_enable (bool val = true) { return set_bit (17, val); }

  // control whether a PAUSE frame is transferred to EDMAC or not.
  bool pause_frame_edmac_enable (void) const { return get_bit (18); }
  mode_t& set_pause_frame_edmac_enable (bool val = true) { return set_bit (18, val); }

  // PAUSE frames with TIME = 0 or not
  bool pause_frame_zero_time_enable (void) const { return get_bit (19); }
  mode_t& set_pause_frame_zero_time_enable (bool val = true) { return set_bit (19, val); }

  // whether PAUSE frames are transmitted within PAUSE periods or not.
  bool pause_frame_in_pause_period_enable (void) const { return get_bit (20); }
  mode_t& set_pause_frame_in_pause_period_enable (bool val = true) { return set_bit (20, val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  mode_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint32_t m_value;
};

class status_t
{
public:
  constexpr status_t (void) : m_value (0) { }
  explicit constexpr status_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  bool illegal_carrier_detected (void) const { return get_bit (0); }
  status_t& set_illegal_carrier_detected (bool val = true) { return set_bit (0, val); }

  bool magic_packet_detected (void) const { return get_bit (1); }
  status_t& set_magic_packet_detected (bool val = true) { return set_bit (1, val); }

  bool link_signal_changed (void) const { return get_bit (2); }
  status_t& set_link_signal_changed (bool val = true) { return set_bit (2, val); }

  // PAUSE frame retransmit count has (not) exceeded the upper limit.
  bool pause_retransmit_limit_exceeded (void) const { return get_bit (4); }
  status_t& set_pause_retransmit_limit_exceeded (bool val = true) { return set_bit (4, val); }

  // continuous reception of broadcast frames has been (not) detected.
  bool continuous_broadcast_reception (void) const { return get_bit (5); }
  status_t& set_continuous_broadcast_reception (bool val = true) { return set_bit (5, val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  status_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint32_t m_value;
};



template < uintptr_t RegAddress, unsigned int ModuleClockHz >
class hw_inst
{
public:
  using mode_t = rx_etherc::mode_t;
  using status_t = rx_etherc::status_t;

  static constexpr unsigned int clock_hz = ModuleClockHz;

  mode_t mode (void) const { return mode_t (regs ().ecmr); }
  void set_mode (mode_t val) { regs ().ecmr = val.value (); }

  // do not change the maximum receive frame length while reception is enabled.
  // although the register allows setting a value less than 1518, 1518 is the
  // effective minimum value.
  uint32_t receive_max_frame_length (void) const { return regs ().rflr; }
  void set_receive_max_frame_length (utils::clamped_value<uint32_t, 1518, 2048> val) { regs ().rflr = val; }

  // writing the status register will clear the corresponding pending interrupts.
  status_t status (void) const { return status_t (regs ().ecsr); }
  void set_status (status_t val) { regs ().ecsr = val.value (); }

  // specify which bits in the status register will trigger an interrupt.
  status_t status_interrupts (void) const { return status_t (regs ().ecsipr); }
  void set_status_interrupts (status_t val) { regs ().ecsipr = val.value (); }

  // external PHY status line
  bool phy_link_status (void) const { return regs ().psr & 1; }

  // 0x00000: normal operation (upper limit disabled)
  // 0x00001...0xFFFFE: setting for the upper limit for the counter used in the
  //                    random number generation block.
  uint32_t random_gen_counter_upper_limit (void) const { return regs ().rdmlr; }
  void set_random_gen_counter_upper_limit (utils::clamped_value<uint32_t, 0, 0xFFFFE> val) { regs ().rdmlr = val; }

  // inter packet gap time that is used when transmitting frames in nanoseconds.
  // the granularity is 40 ns.
  // standard 10 mbit ethernet: 9.6 us = 9600 ns
  // standard 100 mbit ethernet: 0.96 us = 960 ns
  // the default value after reset is 960 ns.
  uint32_t inter_packet_gap (void) const { return regs ().ipgr * 40 + 160; }
  void set_inter_packet_gap (utils::clamped_value<uint32_t, 160, 1400> val) { regs ().ipgr = (val - 160) / 40; }

  // TIME parameter value of an automatic PAUSE frame in 5120 ns units (?).
  // FIXME: figure out the units.  various etherc manuals say "512-bit time",
  // whatever that is supposed to mean ...
  uint32_t auto_pause_frame_time (void) const { return (uint16_t)(regs ().apr); }
  void set_auto_pause_frame_time (utils::clamped_value<uint32_t, 0, 65535> val) { regs ().apr = val; }

  uint32_t manual_pause_frame_time (void) const { return (uint16_t)(regs ().mpr); }
  void set_manual_pause_frame_time (utils::clamped_value<uint32_t, 0, 65535> val) { regs ().mpr = val; }

  // returns the number of PAUSE frames that have been received
  // (zero extended 8 bit counter).
  uint32_t pause_frame_receive_count (void) const { return regs ().rfcf; }

  // max. PAUSE frame retransmission counter.
  uint32_t pause_frame_retransmit_limit (void) const { return regs ().tpauser; }
  void set_pause_frame_retransmit_limit (utils::clamped_value<uint32_t, 0, 65535> val) { regs ().tpauser = val; }

  // number of PAUSE frames that have been retransmitted
  // (zero extended 8 bit counter).
  uint32_t pause_frame_retransmit_count (void) const { return regs ().tpausecr; }

  // max. number of broadcast frames to be received.
  // value of 0 means no limit.
  uint32_t broadcast_frame_limit (void) const { return regs ().bcfrr; }
  void set_broadcast_frame_limit (utils::clamped_value<uint32_t, 0, 65535> val) { regs ().bcfrr = val; }

  std::array<uint8_t, 6> mac_address (void) const
  {
    uint32_t mahr = regs ().mahr;  // upper 32 bits of the 48 bit mac address
    uint32_t malr = regs ().malr;  // lower 16 bits of the 48 bit mac address

    std::array<uint8_t, 6> r;
    r[0] = (mahr >> 24) & 0xFF;
    r[1] = (mahr >> 16) & 0xFF;
    r[2] = (mahr >>  8) & 0xFF;
    r[3] = (mahr >>  0) & 0xFF;
    r[4] = (malr >>  8) & 0xFF;
    r[4] = (malr >>  0) & 0xFF;

    return r;
  }

  void set_mac_address (std::array<uint8_t, 6> val)
  {
    uint32_t mahr = (val[0] << 24) | (val[1] << 16) | (val[2] << 8) | (val[3] << 0);
    uint32_t malr = (val[4] << 8) | (val[5] << 0);

    regs ().mahr = mahr;
    regs ().malr = malr;
  }

  // transmit retry over counter.
  // indicates the number of frames that were unable to be transmitted in 16
  // transmission attempts including the transfer.
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t transmit_retry_over_count (void) const { return regs ().trocr; }
  void reset_transmit_retry_over_count (void) { regs ().trocr = 0; }

  // delayed collision detect counter.
  // indicates the number of all delayed collisions that occured on the line
  // after the start of data transmission.
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t delayed_collision_count (void) const { return regs ().cdcr; }
  void reset_delayed_collision_count (void) { regs ().cdcr = 0; }

  // lost carrier counter.
  // inidicates the number of times the carrier was lost during data transmission.
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t lost_carrier_count (void) const { return regs ().lccr; }
  void reset_lost_carrier_count (void) { regs ().lccr = 0; }

  // carrier not detected counter.
  // indicates the number of times the carrier was not detected during preamble
  // transmission.
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t carrier_not_detected_count (void) const { return regs ().cndcr; }
  void reset_carrier_not_detected_count (void) { regs ().cndcr = 0; }

  // crc error frame receive counter.
  // indicates the number of times a frame with a crc error was received.
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t crc_error_frame_count (void) const { return regs ().cefcr; }
  void reset_crc_error_frame_count (void) { regs ().cefcr = 0; }

  // frame receive error counter.
  // indicates the number of times a receive error was generated by the
  // signal input to the ET_RX_ER pin from the PHY.
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t phy_frame_receive_error_count (void) const { return regs ().frecr; }
  void reset_phy_frame_receive_error_count (void) { regs ().frecr = 0; }

  // truncated (too short) frame receive counter.
  // indicates the number of times a frame was received that was shorter than
  // 64 bytes (minimum frame length).
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t truncated_frame_receive_count (void) const { return regs ().tsfrcr; }
  void reset_truncated_frame_receive_count (void) { regs ().tsfrcr = 0; }

  // over-long frame receive counter.
  // indicates the number of times a frame was received whose length was longer
  // than the maximum allowed length (see set_receive_max_frame_length function).
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t overlong_frame_receive_count (void) const { return regs ().tlfrcr; }
  void reset_overlong_frame_receive_count (void) { regs ().tlfrcr = 0; }

  // residual-bit frame receive counter.
  // indicates the number of times a frame was received which had residual bits
  // (less than 8 bit unit).
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t residual_bit_frame_receive_count (void) const { return regs ().rfcr; }
  void reset_residual_bit_frame_receive_count (void) { regs ().rfcr = 0; }

  // multicast frame receive counter.
  // indicates the number of times a frame was received which had a multicast
  // destination address.
  // counter stops counting once it reaches 0xFFFFFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint32_t multicast_frame_receive_count (void) const { return regs ().mafcr; }
  void reset_multicast_frame_receive_count (void) { regs ().mafcr = 0; }


private:
  struct regs_t;

  const regs_t& regs (void) const { return *(const regs_t*)RegAddress; }
  regs_t& regs (void) { return *(regs_t*)RegAddress; }

  struct regs_t
  {
    volatile uint32_t ecmr;     // 0x000C0100
    uint32_t res_04;
    volatile uint32_t rflr;     // 0x000C0108
    uint32_t res_0C;
    volatile uint32_t ecsr;     // 0x000C0110
    uint32_t res_14;
    volatile uint32_t ecsipr;   // 0x000C0118
    uint32_t res_1C;
    volatile uint32_t pir;      // 0x000C0120
    uint32_t res_24;
    volatile uint32_t psr;      // 0x000C0128
    uint32_t res_2C;
    uint32_t res_30;
    uint32_t res_34;
    uint32_t res_38;
    uint32_t res_3C;
    volatile uint32_t rdmlr;    // 0x000C0140
    uint32_t res_44;
    uint32_t res_48;
    uint32_t res_4C;
    volatile uint32_t ipgr;     // 0x000C0150
    volatile uint32_t apr;      // 0x000C0154
    volatile uint32_t mpr;      // 0x000C0158
    uint32_t res_5C;
    volatile uint32_t rfcf;     // 0x000C0160
    volatile uint32_t tpauser;  // 0x000C0164
    volatile uint32_t tpausecr; // 0x000C0168
    volatile uint32_t bcfrr;    // 0x000C016C
    uint32_t res_70;
    uint32_t res_74;
    uint32_t res_78;
    uint32_t res_7C;
    uint32_t res_80;
    uint32_t res_84;
    uint32_t res_88;
    uint32_t res_8C;
    uint32_t res_90;
    uint32_t res_94;
    uint32_t res_98;
    uint32_t res_9C;
    uint32_t res_A0;
    uint32_t res_A4;
    uint32_t res_A8;
    uint32_t res_AC;
    uint32_t res_B0;
    uint32_t res_B4;
    uint32_t res_B8;
    uint32_t res_BC;
    volatile uint32_t mahr;     // 0x000C01C0
    uint32_t res_C4;
    volatile uint32_t malr;     // 0x000C01C8
    uint32_t res_CC;
    volatile uint32_t trocr;    // 0x000C01D0
    volatile uint32_t cdcr;     // 0x000C01D4
    volatile uint32_t lccr;     // 0x000C01D8
    volatile uint32_t cndcr;    // 0x000C01DC
    uint32_t res_E0;
    volatile uint32_t cefcr;    // 0x000C01E4
    volatile uint32_t frecr;    // 0x000C01E8
    volatile uint32_t tsfrcr;   // 0x000C01EC
    volatile uint32_t tlfrcr;   // 0x000C01F0
    volatile uint32_t rfcr;     // 0x000C01F4
    volatile uint32_t mafcr;    // 0x000C01F8
    uint32_t res_FC;
  };

  static_assert (sizeof (regs_t) == 0x100, "");

};


} // namespace rx_etherc
} // namespace dev
#endif // includeguard_rx_etherc_hpp_includeguard
