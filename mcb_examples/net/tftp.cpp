
/*

atftp --put --local-file "udp_socket_test_small.mot" --remote-file "user_flash.mot" --option "tsize `stat -c %s udp_socket_test_small.mot`" --option "blksize 1024" 192.168.0.80

*/

#include "tftp.hpp"

#include <vector>
#include <cstring>
#include <algorithm>

namespace net
{
namespace tftp
{


message::message (message&& rhs)
: m_data (std::move (rhs.m_data)),
  m_len (std::move (rhs.m_len)),
  m_allocated_data (std::move (rhs.m_allocated_data)),
  m_allocated_size (std::move (rhs.m_allocated_size))
{
  rhs.m_allocated_data = nullptr;
  rhs.m_allocated_size = 0;
}

message& message::operator = (message&& rhs)
{
  delete[] m_allocated_data;
  m_data = std::move (rhs.m_data);
  m_len = std::move (rhs.m_len);
  m_allocated_data = std::move (rhs.m_allocated_data);
  m_allocated_size = std::move (rhs.m_allocated_size);
  return *this;
}

message::message (unsigned int allocate_size, alloc_data_tag)
{
  auto* d = new char[allocate_size];
  if (d != nullptr)
  {
    m_allocated_data = m_data = d;
    m_allocated_size = m_len = allocate_size;
  }
  else
  {
    m_allocated_data = m_data = nullptr;
    m_allocated_size = m_len = 0;
  }
}


#if 0
std::string_view message::get_string (unsigned int offset) const
{
  // usually strings in TFTP messages are zero terminated.  however, do not
  // bindly rely on that.  a packet might be cut off and the zero char might
  // be absent.
  const char* const max_ptr = &data_at<const char> (m_len);

  const char* begin_ptr = &data_at<const char> (offset);
  const char* end_ptr = begin_ptr;

  while (end_ptr < max_ptr)
  {
    if (*end_ptr == 0)
      break;
    ++end_ptr;
  }

  return { begin_ptr, (uintptr_t)end_ptr - (uintptr_t)begin_ptr };
}

#else

std::string_view message::get_string (unsigned int offset) const
{
  // usually strings in TFTP messages are zero terminated.  however, do not
  // bindly rely on that.  a packet might be cut off and the zero char might
  // be absent.

  // note: std::find unrolls the loop.
  const char* begin_ptr = &data_at<const char> (offset);
  const char* end_ptr = std::find (begin_ptr, &data_at<const char> (m_len), 0);

  return { begin_ptr, (uintptr_t)end_ptr - (uintptr_t)begin_ptr };
}

#endif

std::list<std::string_view>
message::get_strings (unsigned int offset) const
{
  std::list<std::string_view> result;

  const char* begin_ptr = &data_at<const char> (offset);
  const char* end_ptr = &data_at<const char> (m_len);

  while (begin_ptr < end_ptr)
  {
    const char* e = std::find (begin_ptr, end_ptr, 0);
    unsigned int l = (uintptr_t)e - (uintptr_t)begin_ptr;
    if (l == 0)
      break;

    result.emplace_back (begin_ptr, l);
    begin_ptr = e + 1;
  }

  return result;
}

void read_write_request::parse_option_strings (void) const
{
  if (!m_option_strings.empty ())
    return;

  m_option_strings = get_strings (sizeof (opcode));
}

std::string_view
read_write_request::filename (void) const
{
  parse_option_strings ();
  if (m_option_strings.size () < 1)
    return { };

  return m_option_strings.front ();
}

std::string_view
read_write_request::mode (void) const
{
  parse_option_strings ();
  if (m_option_strings.size () < 2)
    return { };

  return *std::next (m_option_strings.begin (), 1);
}


std::list<std::string_view>::const_iterator
read_write_request::options_begin (void) const
{
  parse_option_strings ();
  if (m_option_strings.size () < 3)
    return m_option_strings.end ();

  return std::next (m_option_strings.begin (), 2);
}

std::list<std::string_view>::const_iterator
read_write_request::options_end (void) const
{
  parse_option_strings ();
  return m_option_strings.end ();
}

// --------------------------

ack_response::ack_response (uint16_t bn)
: message (sizeof (opcode) + sizeof (uint16_t), alloc_data)
{
  if (raw_data () == nullptr)
    return;

  set_op (opcode::ack);
  set_block_number (bn);
}

void options_ack_response::parse_option_strings (void) const
{
  if (!m_option_strings.empty ())
    return;

  m_option_strings = get_strings (sizeof (opcode));
}

std::list<std::string_view>::const_iterator
options_ack_response::options_begin (void) const
{
  parse_option_strings ();
  return m_option_strings.begin ();
}

std::list<std::string_view>::const_iterator
options_ack_response::options_end (void) const
{
  parse_option_strings ();
  return m_option_strings.end ();
}

// --------------------------

error_response::error_response (error_code c, std::string_view msg)
: message (sizeof (opcode) + sizeof (error_code) + msg.size () + 1, alloc_data)
{
  if (raw_data () == nullptr)
    return;

  set_op (opcode::error);
  set_error (c);

  std::copy (msg.begin (), msg.end (),
	     &data_at<char> (sizeof (opcode) + sizeof (error_code)));

  // zero terminate the copied string.
  data_at<char> (sizeof (opcode) + sizeof (error_code) + msg.size ()) = 0;
}

} // namespace tftp
} // namespace net

