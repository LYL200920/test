/*

this is basically an iterator pair which allows reading or writing data elements
and incrementing the read/write iterator.

*/

#ifndef includeguard_utils_byte_stream_hpp_includeguard
#define includeguard_utils_byte_stream_hpp_includeguard

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <type_traits>
#include <utils/byte_order.hpp>
#include <utils/langcomp.hpp>

namespace utils
{

template <typename T, typename Enable = void> struct byte_stream_traits;

class byte_stream
{
public:
  byte_stream (void)
  : m_end_ptr (nullptr), m_cur_ptr (nullptr), m_begin_ptr (nullptr) { }

  byte_stream (const void* ptr, size_t sz)
  : m_end_ptr ((char*)ptr + sz),
    m_cur_ptr ((char*)ptr),
    m_begin_ptr ((char*)ptr)
  {
  }

  byte_stream (const void* begin_ptr, const void* end_ptr)
  : m_end_ptr ((char*)end_ptr),
    m_cur_ptr ((char*)begin_ptr),
    m_begin_ptr ((char*)begin_ptr)
  {
  }

  bool is_end (void) const { return m_cur_ptr == m_end_ptr; }
  bool is_begin (void) const { return m_cur_ptr == m_begin_ptr; }

  // 'empty' is ambiguous.
  // when using byte_stream to read bytes from a buffer, empty = cur_ptr == end_ptr
  // when using byte_stream to write bytes into a buffer, empty = cur_ptr == begin_ptr
  // because of this, leave it out.
  //  bool empty (void) const { return m_cur_ptr == m_begin_ptr; }

  char* cur_ptr (void) const { return m_cur_ptr; }
  char* end_ptr (void) const { return m_end_ptr; }
  char* begin_ptr (void) const { return m_begin_ptr; }

  unsigned int remaining_size (void) const { return m_end_ptr - m_cur_ptr; }
  unsigned int size (void) const { return m_cur_ptr - m_begin_ptr; }

  // read the element at the current position and increment the current pointer
  // by the size of the element.  if the read would exceed the buffer limit
  // (end pointer), the read is not done.
  // FIXME: throw exception or something.
  template <typename T> T read (byte_order_t b)
  {
    if (remaining_size () < sizeof (T))
      return { };
    else
      return read_unchecked<T> (b);
  }

  template <typename T> T read_unchecked (byte_order_t b)
  {
    auto p = m_cur_ptr;
    m_cur_ptr += sizeof (T);

    if (b == little_endian)
      return byte_stream_traits<T>::read_le (p);
    else //if (b == big_endian)
      return byte_stream_traits<T>::read_be (p);
  }

  unsigned int skip_read (unsigned int byte_count)
  {
    byte_count = std::min (remaining_size (), byte_count);
    m_cur_ptr += byte_count;
    return byte_count;
  }

  unsigned int read (void* bytes_out, unsigned int byte_count)
  {
    byte_count = std::min (remaining_size (), byte_count);
    std::memcpy (bytes_out, m_cur_ptr, byte_count);
    m_cur_ptr += byte_count;
    return byte_count;
  }

  // write the specified element at the current position and increment the
  // current pointer by the size of the element.  if it would exceed the buffer
  // limit (end pointer), the write is not done.
  // FIXME: throw exception or something.
  template <typename T> void write (byte_order_t b, const T& val)
  {
    if (remaining_size () < sizeof (T))
      return;
    else
      write_unchecked (b, val);
  }

  template <typename T> void write_unchecked (byte_order_t b, const T& val)
  {
    if (b == little_endian)
      byte_stream_traits<T>::write_le (m_cur_ptr, val);
    else // if (b == big_endian)
      byte_stream_traits<T>::write_be (m_cur_ptr, val);

    m_cur_ptr += sizeof (T);
  }

  unsigned int write (const void* bytes_in, unsigned int byte_count)
  {
    byte_count = std::min (remaining_size (), byte_count);
    std::memcpy (m_cur_ptr, bytes_in, byte_count);
    m_cur_ptr += byte_count;
    return byte_count;
  }

private:
  char* m_end_ptr;    // end pointer AFTER the last valid byte of the buffer.
  char* m_cur_ptr;
  char* m_begin_ptr;
};


// --------------------------------------------------------------------------

// we could use std::is_trivially_copyable for serialization, but not across
// different architectures.  e.g. the byte order and alignment requirements
// might be different.

template <typename T>
struct byte_stream_traits<T, typename std::enable_if <
	   (std::is_same<char, T>::value && sizeof (char) == 1)
	   || (std::is_same<wchar_t, T>::value && sizeof (wchar_t) == 1)
	   || std::is_same<int8_t, T>::value
	   || std::is_same<uint8_t, T>::value
	   || std::is_same<bool, T>::value>::type>
{
  static void write_le (char* buf_out, const T& val)
  {
    uint8_t* p = (uint8_t*)buf_out;
    *p = (uint8_t)val;
  }

  static void write_be (char* buf_out, const T& val)
  {
    uint8_t* p = (uint8_t*)buf_out;
    *p = (uint8_t)val;
  }

  static T read_le (const char* buf_in)
  {
    const uint8_t* p = (const uint8_t*)buf_in;
    return (T)*p;
  }

  static T read_be (const char* buf_in)
  {
    const uint8_t* p = (const uint8_t*)buf_in;
    return (T)*p;
  }
};

template <typename T>
struct byte_stream_traits<T, typename std::enable_if <
	   (std::is_same<wchar_t, T>::value && sizeof (wchar_t) == 2)
	   || std::is_same<char16_t, T>::value
	   || std::is_same<int16_t, T>::value
	   || std::is_same<uint16_t, T>::value>::type>
{
  static void write_le (char* buf_out, const T& val)
  {
    auto v = utils::native_to (utils::little_endian, (uint16_t)val);
    std::memcpy (buf_out, &v, sizeof (v));
  }

  static void write_be (char* buf_out, const T& val)
  {
    auto v = utils::native_to (utils::big_endian, (uint16_t)val);
    std::memcpy (buf_out, &v, sizeof (v));
  }

  static T read_le (const char* buf_in)
  {
    uint16_t v;
    std::memcpy (&v, buf_in, sizeof (v));
    return (T)utils::to_native (utils::little_endian, v);
  }

  static T read_be (const char* buf_in)
  {
    uint16_t v;
    std::memcpy (&v, buf_in, sizeof (v));
    return (T)utils::to_native (utils::big_endian, v);
  }
};

template <typename T>
struct byte_stream_traits<T, typename std::enable_if <
	   (std::is_same<wchar_t, T>::value && sizeof (wchar_t) == 4)
	   || std::is_same<char32_t, T>::value
	   || std::is_same<int32_t, T>::value
	   || std::is_same<uint32_t, T>::value>::type>
{
  static void write_le (char* buf_out, const T& val)
  {
    auto v = utils::native_to (utils::little_endian, (uint32_t)val);
    std::memcpy (buf_out, &v, sizeof (v));
  }

  static void write_be (char* buf_out, const T& val)
  {
    auto v = utils::native_to (utils::big_endian, (uint32_t)val);
    std::memcpy (buf_out, &v, sizeof (v));
  }

  static T read_le (const char* buf_in)
  {
    uint32_t v;
    std::memcpy (&v, buf_in, sizeof (v));
    return (T)utils::to_native (utils::little_endian, v);
  }

  static T read_be (const char* buf_in)
  {
    uint32_t v;
    std::memcpy (&v, buf_in, sizeof (v));
    return (T)utils::to_native (utils::big_endian, v);
  }
};


} // namespace utils
#endif // includeguard_utils_byte_stream_hpp_includeguard
