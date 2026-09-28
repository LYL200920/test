#ifndef includeguard_dev_i2c_hpp_includeguard
#define includeguard_dev_i2c_hpp_includeguard

#include <cstdint>
#include <exception>
#include <string_view>
#include <array>

namespace dev
{

struct i2c_error : public std::exception
{
  i2c_error (void) { }
  virtual const char* what (void) const noexcept override { return "i2c error"; }
};

namespace i2c_manufacturer_id
{
enum : unsigned int
{
  #define expand_i2c_manufacturer_id(enum_name, int_val, str_val) enum_name = int_val,
    #include "i2c_manufacturer_id.x.hpp"
};

std::string_view to_string (unsigned int id);

} // namespace i2c_manufacturer_id

class i2c_device_id
{
public:

  // 12 bits
  unsigned int manufacturer (void) const
  {
    return (m_data[0] << 4) | (m_data[1] >> 4);
  }

  // 9 bits
  unsigned int part_id (void) const
  {
    return ((m_data[1] & 0b11110000) << 1) | (m_data[2] >> 3);
  }

  // 3 bits
  unsigned int revision (void) const
  {
    return m_data[2] & 0b111;
  }

  constexpr unsigned int size (void) const { return 3; }

  const uint8_t* data (void) const { return m_data; }
  uint8_t* data (void) { return m_data; }

private:
  uint8_t m_data[3];
};



struct i2c_master
{
  virtual i2c_device_id query_device_id (uint8_t slave_addr) = 0;

  // return true if successful or false if there was an error.
  virtual bool send (uint8_t slave_addr,
		     const void* tx_data, unsigned int count_bytes) = 0;

  // return true if successful or false if there was an error.
  virtual bool recv (uint8_t slave_addr,
		     void* rx_data, unsigned int count_bytes) = 0;

  // return true if successful or false if there was an error.
  virtual bool send_recv (uint8_t slave_addr,
			  const void* tx_data, unsigned int tx_count_bytes,
			  void* rx_data, unsigned int rx_count_bytes)
  {
    if (!send (slave_addr, tx_data, tx_count_bytes))
      return false;

    return recv (slave_addr | 1, rx_data, rx_count_bytes);
  }

  template <unsigned int N> bool
  send (uint8_t slave_addr, const unsigned char(&data)[N])
  {
    return send (slave_addr, data, N);
  }

  template <unsigned int N> bool
  send (uint8_t slave_addr, const std::array<uint8_t, N>& data)
  {
    return send (slave_addr, data.data (), data.size ());
  }

  template <unsigned int N> bool
  recv (uint8_t slave_addr, unsigned char(&data)[N])
  {
    return recv (slave_addr, data, N);
  }

  template <unsigned int N> bool
  recv (uint8_t slave_addr, std::array<uint8_t, N>& data)
  {
    return recv (slave_addr, data.data (), data.size ());
  }

  template <unsigned int WriteN, unsigned int ReadN> bool
  send_recv (uint8_t slave_addr,
	     const unsigned char(&wr_data)[WriteN],
	     unsigned char(&rd_data)[ReadN])
  {
    return send_recv (slave_addr, wr_data, WriteN, rd_data, ReadN);
  }

  template <unsigned int WriteN, unsigned int ReadN> bool
  send_recv (uint8_t slave_addr,
             const std::array<uint8_t, WriteN>& wr_data,
             std::array<uint8_t, ReadN>& rd_data)
  {
    return send_recv (slave_addr, wr_data.data (), WriteN, rd_data.data (), ReadN);
  }
};


} // namespace dev
#endif // includeguard_dev_i2c_hpp_includeguard
