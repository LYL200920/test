
#ifndef includeguard_linux_userspace_usart_includeguard
#define includeguard_linux_userspace_usart_includeguard

#include <dev/usart.hpp>
#include <string>
#include <string_view>

namespace dev
{

class linux_userspace_usart : public usart
{
public:
  linux_userspace_usart (const std::string_view& dev_path);
  ~linux_userspace_usart (void);

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
  std::string m_dev_path;
  unsigned int m_baud_rate;
  int m_fd = -1;
  unsigned int m_timeout_ms;
};

} // namespace dev
#endif // includeguard_linux_userspace_usart_includeguard

