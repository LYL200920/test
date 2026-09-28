
#ifndef includeboard_board_info_includeguard
#define includeboard_board_info_includeguard

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <array>

class board_info
{
public:
  // implemented by the board library / BSP.
  // static const board_info& inst (void);

  // zero initializes all fields.
  board_info (void) { std::memset (this, 0, sizeof (board_info)); }

  // checks if the board info is blank (all zeros).
  bool is_blank (void) const;

  // variant is a 32 bit int value stored as big endian.
  // effectively it is a board-id number.  it can be used by software to check
  // for basic compatibility of the board and the software before doing any
  // IO or CPU initialization.  this is to avoid hardware damage by incompatible
  // board and software combination, if the user flashes the wrong software
  // on the wrong board.

  // FIXME: move the list of known board IDs to some other place.

  // 0x00000000 - first 5 MCB samples (white IO connector, DAC, jtag, no encoder input)
  //              this is software compatible with board ID 0x00000001

  // 0x00000001 - first 100 units lot MCB LI5000 (black IO connector, no DAC, no CN1).
  //              this is software compatible with board ID 0x00000000

  // 0x10000000 - second and third lot produced by DDL.  same as 0x00000001
  //              but there was a mistake when specifying the board id parameter
  //              for factory programming.

  // 0x00000002 - MCBv1.3 protoype with 176 pin RX71M

  // 0x00000010 - MCBv2

  uint32_t board_id (void) const
  {
    return (m_board_id[0] << 24) | (m_board_id[1] << 16) | (m_board_id[2] << 8) | (m_board_id[3] << 0);
  }

  const std::array<uint8_t, 4>& board_id_bytes (void) const { return m_board_id; }

  board_info& set_board_id (uint32_t val)
  {
    m_board_id[0] = (val >> 24) & 0xFF;
    m_board_id[1] = (val >> 16) & 0xFF;
    m_board_id[2] = (val >> 8) & 0xFF;
    m_board_id[3] = (val >> 0) & 0xFF;
    return *this;
  }

  board_info& set_board_id (const std::array<uint8_t, 4>& val)
  {
    m_board_id = val;
    return *this;
  }

  // the feature bits are basically a re-implemented std:bitset, to get a
  // guaranteed memory layout of the bits.
  class feature_bits
  {
  public:
    class reference
    {
    public:
      reference& operator = (bool val) noexcept { m_bits.set (m_i, val); return *this; }
      reference& operator = (const reference& x) noexcept { m_bits.set (m_i, x); return *this; }

      operator bool (void) const noexcept { return m_bits.test (m_i); }

      bool operator ~ (void) const noexcept { return !m_bits.test (m_i); }

      reference& flip (void) noexcept { m_bits.flip (m_i); return *this; }

    private:
      unsigned int m_i;
      feature_bits& m_bits;

      constexpr reference (unsigned int i, feature_bits& bits) : m_i (i), m_bits (bits) { }

      friend class feature_bits;
    };

    static constexpr unsigned int bit_count = 96;

    constexpr feature_bits (void) : m_data () { }
    constexpr feature_bits (const std::array<uint8_t, bit_count / 8>& val) : m_data (val) { }

    constexpr bool test (unsigned int b) const { return m_data[b / 8] & (1 << (b % 8)); }
    feature_bits& set (unsigned int b) { m_data[b / 8] |= (1 << (b % 8)); return *this; }
    feature_bits& set (unsigned int b, bool val) { reset (b); if (val) set (b); return *this; }
    feature_bits& reset (unsigned int b) { m_data[b / 8] &= ~(1 << (b % 8)); return *this; }
    feature_bits& flip (unsigned int b) { m_data[b / 8] ^= (1 << (b % 8)); return *this; }

    bool operator [] (unsigned int i) const { return test (i);  }
    reference operator [] (unsigned int i) { return reference (i, *this); }

    const std::array<uint8_t, bit_count / 8>& bytes (void) const { return m_data; }

    feature_bits& set (void)
    {
      m_data.fill (0xFF);
      return *this;
    }

  private:
    std::array<uint8_t, bit_count / 8> m_data;
  };

  // these can be overloaded by the subclass of a particular board's board_info.
  const feature_bits& features (void) const { return m_features; }
  void set_features (const feature_bits& val) { m_features = val; }

  class ifconfig
  {
  public:
    // init everything to zero.
    ifconfig (void) { std::memset (this, 0, sizeof (ifconfig)); }

    bool is_blank (void) const;

    const std::array<uint8_t, 8>& hw_addr (void) const { return m_hw_addr; }
    ifconfig& set_hw_addr (const std::array<uint8_t, 8>& val) { m_hw_addr = val; return *this; }

    const std::array<uint8_t, 4>& ipv4_addr (void) const { return m_ipv4_addr; }
    ifconfig& set_ipv4_addr (const std::array<uint8_t, 4>& val) { m_ipv4_addr = val; return *this; }

    const std::array<uint8_t, 4>& ipv4_netmask (void) const { return m_ipv4_netmask; }
    ifconfig& set_ipv4_netmask (const std::array<uint8_t, 4>& val) { m_ipv4_netmask = val; return *this; }

  private:
    std::array<uint8_t, 8> m_hw_addr;
    std::array<uint8_t, 4> m_ipv4_addr;
    std::array<uint8_t, 4> m_ipv4_netmask;
  };

  const std::array<ifconfig, 5>& ifconfigs (void) const { return m_ifconfigs; }
  std::array<ifconfig, 5>& ifconfigs (void) { return m_ifconfigs; }

  const std::array<uint8_t, 32>& board_uid_override (void) const { return m_board_uid_override; }
  void set_board_uid_override (const std::array<uint8_t, 32>& val) { m_board_uid_override = val; }

protected:
  std::array<uint8_t, 4> m_board_id;

  feature_bits m_features;
  std::array<ifconfig, 5> m_ifconfigs;
  std::array<uint8_t, 32> m_board_uid_override;

  static_assert (sizeof (feature_bits) == 12, "");
  static_assert (sizeof (ifconfig) == 16, "");
};

static_assert (sizeof (board_info) == 128, "");


#endif // includeboard_board_info_includeguard
