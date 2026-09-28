#include "mcb_binary_service.hpp"
#include "mcb_axis_diagnostics.hpp"

#include <cstring>

namespace mcb_binary
{

void session::reset (uint64_t now_ms)
{
  m_rx_size = 0;
  m_tx_size = 0;
  m_partial_offset = rx_capacity;
  m_frame_since = now_ms;
  m_send_since = now_ms;
  m_activity_since = now_ms;
  m_closed = false;
}

bool session::receive (const void* data, std::size_t size, uint64_t now_ms)
{
  if (m_closed)
    return false;
  if ((m_partial_offset != rx_capacity && now_ms - m_frame_since >= frame_timeout_ms)
      || size > receive_space () || (size != 0 && data == nullptr))
  {
    close ();
    return false;
  }
  if (size == 0)
    return true;

  std::memcpy (m_rx.data () + m_rx_size, data, size);
  m_rx_size += size;
  m_activity_since = now_ms;

  // Locate the trailing incomplete frame even when earlier complete requests
  // are waiting for TX space. Only a new frame gets a new deadline.
  std::size_t offset = 0;
  while (offset < m_rx_size)
  {
    const bool variable = (m_rx[offset] & 0xF0) == 0xE0;
    std::size_t frame_size = 8;
    if (variable)
    {
      if (m_rx_size - offset < 4)
        break;
      frame_size = 4 + (std::size_t (m_rx[offset + 2]) << 8) + m_rx[offset + 3];
    }
    if (m_rx_size - offset < frame_size)
      break;
    offset += frame_size;
  }
  if (offset < m_rx_size)
  {
    if (m_partial_offset != offset)
      m_frame_since = now_ms;
    m_partial_offset = offset;
  }
  else
    m_partial_offset = rx_capacity;
  return true;
}

void session::dispatch (const read_only_backend& backend, std::array<uint8_t, 8>& response) const
{
  response.fill (0);
  response[0] = 0xFF;

  if (m_rx[0] != 0 || (m_rx[2] >> 4) != 0)
    return;

  const uint32_t command = (uint32_t (m_rx[1] & 0x0F) << 8) | (m_rx[2] & 0x0F);
  if (command != 0x0301 && command != 0x0306 && command != 0x0307 && command != 0x0507)
    return;

  const unsigned int axes = m_rx[1] >> 4;
  if (command != 0x0301 && (axes == 0 || (axes & (axes - 1)) != 0))
    return;

  uint8_t status = 0;
  uint32_t value = 0;
  for (unsigned int index = 0; index < 4; ++index)
  {
    if ((axes & (1U << index)) == 0)
      continue;

    axis_snapshot axis;
    if (!backend.read_axis (index, axis))
    {
      response[0] = 0xFE;
      return;
    }

    if (command == 0x0301)
      status |= axis_status (axis);
    else if (command == 0x0306)
      value = static_cast<uint32_t> (axis.logical_position);
    else if (command == 0x0307)
      value = static_cast<uint32_t> (axis.encoder_position);
    else
      value = axis.speed_pps;
  }

  response[0] = 0;
  response[1] = m_rx[1];
  response[2] = m_rx[2];
  response[3] = status;
  response[4] = static_cast<uint8_t> (value >> 24);
  response[5] = static_cast<uint8_t> (value >> 16);
  response[6] = static_cast<uint8_t> (value >> 8);
  response[7] = static_cast<uint8_t> (value);
}

void session::exec (const read_only_backend& backend, uint64_t now_ms)
{
  if (m_closed)
    return;

  if ((m_partial_offset != rx_capacity && now_ms - m_frame_since >= frame_timeout_ms)
      || (m_tx_size != 0 && now_ms - m_send_since >= send_timeout_ms)
      || now_ms - m_activity_since >= idle_timeout_ms)
  {
    close ();
    return;
  }

  for (unsigned int count = 0; count < frames_per_exec && m_rx_size != 0; ++count)
  {
    const bool variable = (m_rx[0] & 0xF0) == 0xE0;
    std::size_t frame_size = 8;
    if (variable && m_rx_size >= 4)
    {
      const std::size_t payload_size = (std::size_t (m_rx[2]) << 8) | m_rx[3];
      if (payload_size > max_variable_payload)
      {
        close ();
        return;
      }
      frame_size = 4 + payload_size;
    }

    if ((variable && m_rx_size < 4) || m_rx_size < frame_size)
    {
      if (now_ms - m_frame_since >= frame_timeout_ms)
        close ();
      return;
    }

    if (tx_capacity - m_tx_size < 8)
      return;

    std::array<uint8_t, 8> response { };
    if (variable)
      response[0] = 0xFF;
    else
      dispatch (backend, response);

    if (m_tx_size == 0)
      m_send_since = now_ms;
    std::memcpy (m_tx.data () + m_tx_size, response.data (), response.size ());
    m_tx_size += response.size ();

    m_rx_size -= frame_size;
    std::memmove (m_rx.data (), m_rx.data () + frame_size, m_rx_size);
    if (m_partial_offset != rx_capacity)
      m_partial_offset -= frame_size;
  }
}

void session::consume_output (std::size_t size, uint64_t now_ms)
{
  if (size > m_tx_size)
  {
    close ();
    return;
  }
  if (size == 0)
    return;

  m_tx_size -= size;
  std::memmove (m_tx.data (), m_tx.data () + size, m_tx_size);
  m_send_since = now_ms;
  m_activity_since = now_ms;
}

} // namespace mcb_binary
