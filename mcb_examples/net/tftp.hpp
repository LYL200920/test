/*
  TFTP - trivial file transfer protocol

  http://www.rfc-base.org/txt/rfc-1350.txt
  http://networksorcery.com/enp/default.htm

  option extension
  http://networksorcery.com/enp/rfc/rfc2347.txt

  blocksize option
  http://networksorcery.com/enp/rfc/rfc2348.txt

  timeout interval and transfer size options
  http://networksorcery.com/enp/rfc/rfc2349.txt

  tftp uri
  http://networksorcery.com/enp/rfc/rfc3617.txt
*/

#ifndef includeguard_net_tftp_includeguard
#define includeguard_net_tftp_includeguard

#include "net.hpp"

#include <string_view>
#include <list>

namespace net
{
namespace tftp
{

// instead of byteswapping the opcode value in the header byteswap
// the constant definitions.
enum struct opcode : uint16_t
{
  // read request
  rrq = hton<uint16_t> (1),

  // write request
  wrq = hton<uint16_t> (2),

  // read/write next block of data
  data = hton<uint16_t> (3),

  // acknowledgement
  ack = hton<uint16_t> (4),

  // error message
  error = hton<uint16_t> (5),

  // option acknowledgement
  oack = hton<uint16_t> (6)
};

// likewise for the error codes.  byteswap the constants instead of the
// stored values.  on the other hand, on little endian systems the constants
// get bigger if we do so.
enum struct error_code : uint16_t
{
  undefined_error = hton<uint16_t> (0),
  file_not_found = hton<uint16_t> (1),
  access_violation = hton<uint16_t> (2),
  disk_full = hton<uint16_t> (3),
  illegal_operation = hton<uint16_t> (4),
  unknown_transfer_id = hton<uint16_t> (5),
  file_already_exists = hton<uint16_t> (6),
  no_such_user = hton<uint16_t> (7),
  terminate_option_negotiation = hton<uint16_t> (8)
};

// -----------------------------------------------------------------------------

class message
{
public:
  constexpr message (void)
  : m_data (nullptr), m_len (0), m_allocated_data (0), m_allocated_size (0) { }

  constexpr message (const void* d, unsigned int l)
  : m_data ((char*)d), m_len (l), m_allocated_data (0), m_allocated_size (0) { }

  ~message (void) { delete[] m_allocated_data; }

  // for now we can't copy messages, because the underlying buffer can't be
  // shared and thus we'd need to copy the whole data.
  message (const message& rhs) = delete;
  message& operator = (const message& rhs) = delete;

  message (message&& rhs);
  message& operator = (message&& rhs);

  opcode op (void) const { return data_at<opcode> (0); }
  void set_op (opcode val) { data_at<opcode> (0) = val; }

  const void* raw_data (void) const { return m_data; }
  unsigned int raw_size (void) const { return m_len; }

protected:
  enum alloc_data_tag { alloc_data };
  message (unsigned int allocate_size, alloc_data_tag);

  char* m_data;
  unsigned int m_len;

  char* m_allocated_data;
  unsigned int m_allocated_size;

  template <typename T> const T& data_at (unsigned int offset) const
  {
    return *reinterpret_cast<const T*> (m_data + offset);
  }

  template <typename T> T& data_at (unsigned int offset)
  {
    return *reinterpret_cast<T*> (m_data + offset);
  }

  std::string_view get_string (unsigned int offset) const;
  std::string_view get_string2 (unsigned int offset) const;

  std::list<std::string_view> get_strings (unsigned int offset) const;
};

// -----------------------------------------------------------------------------

class read_write_request : public message
{
public:
  read_write_request (message&& m) : message (std::move (m)) { }
  read_write_request (const void* d, unsigned int l) : message (d, l) { }

  bool is_read (void) const { return op () == opcode::rrq; }
  bool is_write (void) const { return op () == opcode::wrq; }

  std::string_view filename (void) const;

  // defined in rfc1350: "netascii", "octet", "mail"
  std::string_view mode (void) const;

  std::list<std::string_view>::const_iterator options_begin (void) const;
  std::list<std::string_view>::const_iterator options_end (void) const;

private:
  mutable std::list<std::string_view> m_option_strings;

  void parse_option_strings (void) const;
};

// -----------------------------------------------------------------------------

class data_packet : public message
{
public:
  data_packet (message&& m) : message (std::move (m)) { }
  data_packet (const void* d, unsigned int l) : message (d, l) { }

  data_packet (uint16_t bn, const void* d, unsigned int l);


  uint16_t block_number (void) const
  {
    return hton (data_at<uint16_t> (sizeof (opcode)));
  }

  const void* data (void) const
  {
    return &data_at<char> (sizeof (opcode) + sizeof (uint16_t));
  }

  unsigned int size (void) const
  {
    return m_len - (sizeof (opcode) + sizeof (uint16_t));
  }
};

// -----------------------------------------------------------------------------

class ack_response : public message
{
public:
  ack_response (message&& m) : message (std::move (m)) { }
  ack_response (const void* d, unsigned int l) : message (d, l) { }

  ack_response (uint16_t bn);

  uint16_t block_number (void) const
  {
    return hton (data_at<uint16_t> (sizeof (opcode)));
  }

private:
  void set_block_number (uint16_t val)
  {
    data_at<uint16_t> (sizeof (opcode)) = hton (val);
  }
};

// -----------------------------------------------------------------------------

class options_ack_response : public message
{
public:
  options_ack_response (message&& m) : message (std::move (m)) { }
  options_ack_response (const void* d, unsigned int l) : message (d, l) { }

  uint16_t block_number (void) const
  {
    return hton (data_at<uint16_t> (sizeof (opcode)));
  }

  std::list<std::string_view>::const_iterator options_begin (void) const;
  std::list<std::string_view>::const_iterator options_end (void) const;

private:
  mutable std::list<std::string_view> m_option_strings;

  void parse_option_strings (void) const;
};

// -----------------------------------------------------------------------------

class error_response : public message
{
public:
  error_response (message&& m) : message (std::move (m)) { }
  error_response (const void* d, unsigned int l) : message (d, l) { }

  error_response (error_code c, std::string_view msg = { });

  error_code error (void) const
  {
    return data_at<error_code> (sizeof (opcode));
  }

  std::string_view error_message (void) const
  {
    return get_string (sizeof (opcode) + sizeof (error_code));
  }

private:
  void set_error (error_code c)
  {
    data_at<error_code> (sizeof (opcode)) = c;
  }

};



} // namespace tftp
} // namespace net
#endif // includeguard_net_tftp_includeguard
