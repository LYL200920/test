
#ifndef includeguard_win32_userspace_usart_includeguard
#define includeguard_win32_userspace_usart_includeguard

#include <dev/usart.hpp>
#include <string>
#include <string_view>

namespace dev
{

class win32_userspace_usart : public usart
{
public:
  // open with something like "COM1", "COM2" ...
  win32_userspace_usart (const std::string_view& dev_name);

  ~win32_userspace_usart (void);

  virtual bool is_valid (void) const noexcept override;

  virtual struct config config (void) const noexcept override;
  virtual bool set_config (const struct config& val) noexcept override;

  virtual void reset_receiver (void) noexcept override;
  virtual void reset_transmitter (void) noexcept override;

  virtual void write (const void* data, unsigned int byte_count) override;
  virtual unsigned int read (void* data, unsigned int max_byte_count) override;

  virtual void reset_rx_buffer (void) noexcept override;

  virtual struct buffer_stat buffer_stat (void) const noexcept override;

  virtual void set_recv_clb (void f (void* user_p, unsigned int byte_count), void* user_p) noexcept override;

  virtual bool read_port (unsigned int n) const override;
  virtual void write_port (unsigned int n, bool val) override;

private:
  std::string m_dev_name;
  void* m_dev_handle = nullptr;
};

} // namespace dev
#endif // includeguard_win32_userspace_usart_includeguard
