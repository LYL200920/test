#ifndef includeguard_mcb_binary_service_hpp
#define includeguard_mcb_binary_service_hpp

#include <array>
#include <cstddef>
#include <cstdint>

namespace mcb_binary
{

struct axis_snapshot
{
  int32_t logical_position = 0;
  int32_t encoder_position = 0;
  uint32_t speed_pps = 0;
  bool moving = false;
  bool homing = false;
  bool error = false;
  bool positive_limit = false;
  bool negative_limit = false;
};

class read_only_backend
{
public:
  virtual ~read_only_backend () = default;
  virtual bool read_axis (unsigned int index, axis_snapshot& result) const = 0;
};

class session
{
public:
  // A partial maximum-size variable frame plus one complete uIP packet.
  static constexpr std::size_t rx_capacity = 3072;
  static constexpr std::size_t tx_capacity = 16 * 8;
  static constexpr std::size_t max_variable_payload = 1024;
  static constexpr unsigned int frames_per_exec = 4;
  static constexpr uint64_t frame_timeout_ms = 5000;
  static constexpr uint64_t send_timeout_ms = 10000;
  static constexpr uint64_t idle_timeout_ms = 60000;

  void reset (uint64_t now_ms);
  bool receive (const void* data, std::size_t size, uint64_t now_ms);
  void exec (const read_only_backend& backend, uint64_t now_ms);
  const uint8_t* output_data () const { return m_tx.data (); }
  std::size_t output_size () const { return m_tx_size; }
  void consume_output (std::size_t size, uint64_t now_ms);
  std::size_t receive_space () const { return rx_capacity - m_rx_size; }
  bool should_close () const { return m_closed; }

private:
  void dispatch (const read_only_backend& backend, std::array<uint8_t, 8>& response) const;
  void close () { m_closed = true; }

  std::array<uint8_t, rx_capacity> m_rx { };
  std::array<uint8_t, tx_capacity> m_tx { };
  std::size_t m_rx_size = 0;
  std::size_t m_tx_size = 0;
  std::size_t m_partial_offset = rx_capacity;
  uint64_t m_frame_since = 0;
  uint64_t m_send_since = 0;
  uint64_t m_activity_since = 0;
  bool m_closed = false;
};

} // namespace mcb_binary

#endif
