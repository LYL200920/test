
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <iostream>
#include <thread>
#include <cassert>

#include <utils/langcomp.hpp>

#include "linux_userspace_usart.hpp"

namespace dev
{

linux_userspace_usart::linux_userspace_usart (const std::string_view& dev_path)
: m_dev_path (dev_path)
{
  // notice, this might actually toggle the IO lines

  m_fd = open (m_dev_path.c_str (), O_RDWR | O_NOCTTY | O_NDELAY);
  if (!is_valid ())
  {
    std::cerr << "could not open device '" << m_dev_path << "'" << std::endl;
    return;
  }

  fcntl (m_fd, F_SETFL, 0);

  struct termios attr;
  tcgetattr (m_fd, &attr);

  // zero baud-rate is used to terminate/reset a connection.
  cfsetispeed (&attr, B0);
  cfsetospeed (&attr, B0);
  tcsetattr (m_fd, TCSAFLUSH, &attr);

  std::this_thread::sleep_for (std::chrono::milliseconds (100));

  // set initial speed to 9600 baud and switch to raw-mode.
  m_baud_rate = 9600;
  tcgetattr (m_fd, &attr);
  cfsetispeed (&attr, B9600);
  cfsetospeed (&attr, B9600);
  cfmakeraw (&attr);

  attr.c_iflag &= ~(IXON | IXOFF | IXANY);
  attr.c_iflag |= IGNBRK;

  attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  attr.c_cflag &= ~(PARENB | CSIZE | CRTSCTS | CSTOPB);
  attr.c_cflag |= CS8 | CLOCAL;

  m_timeout_ms = 5000;
  attr.c_cc[VTIME] = m_timeout_ms/100;  // read timeout in deciseconds (100 millisecond steps)
  attr.c_cc[VMIN] = 0;

  tcsetattr (m_fd, TCSANOW, &attr);
  tcflush (m_fd, TCIOFLUSH);
}

linux_userspace_usart::~linux_userspace_usart (void)
{
  close (m_fd);
}

bool linux_userspace_usart::is_valid (void) const noexcept
{
  return m_fd >= 0;
}

struct linux_userspace_usart::config
linux_userspace_usart::config (void) const noexcept
{
  if (!is_valid ())
    return { };

  struct termios attr;
  tcgetattr (m_fd, &attr);

  struct config cfg;

  cfg.baud_rate = m_baud_rate;

  if ((attr.c_cflag & PARENB) != 0 && (attr.c_cflag & PARODD) == 0)
    cfg.parity = parity::even;
  else if ((attr.c_cflag & PARENB) != 0 && (attr.c_cflag & PARODD) != 0)
    cfg.parity = parity::odd;
  else
    cfg.parity = parity::none;

  if (attr.c_cflag & CSTOPB)
    cfg.start_stop_bits_4 = 2*4;
  else
    cfg.start_stop_bits_4 = 1*4;

  if (attr.c_cflag & CS8)
    cfg.data_bits = 8;
  else if (attr.c_cflag & CS7)
    cfg.data_bits = 7;
  else if (attr.c_cflag & CS6)
    cfg.data_bits = 6;
  else if (attr.c_cflag & CS5)
    cfg.data_bits = 5;

  if ((attr.c_cflag & CRTSCTS) != 0)
    cfg.flow_ctrl = flow_ctrl::rts_cts;
  else if ((attr.c_iflag & IXON) != 0 && (attr.c_iflag & IXON) != 0)
    cfg.flow_ctrl = flow_ctrl::xon_xoff;
  else
    cfg.flow_ctrl = flow_ctrl::none;

  // FIXME: implement
  cfg.xon_char = 0;
  cfg.xoff_char = 0;

  cfg.timeout_ms = m_timeout_ms;
  return cfg;
}

bool linux_userspace_usart::set_config (const struct config& cfg) noexcept
{
  if (!is_valid ())
    return false;

  tcdrain (m_fd);

  struct termios attr;
  tcgetattr (m_fd, &attr);

  attr.c_iflag &= ~(IXON | IXOFF | IXANY);
  attr.c_cflag &= ~(PARENB | CSIZE | CRTSCTS | CSTOPB);

  m_baud_rate = cfg.baud_rate;

  {
    auto br = B0;
    switch (m_baud_rate)
    {
      case 300: br = B300; break;
      case 600: br = B600; break;
      case 1200: br = B1200; break;
      case 2400: br = B2400; break;
      case 4800: br = B4800; break;
      case 9600: br = B9600; break;
      case 19200: br = B19200; break;
      case 38400: br = B38400; break;
      case 57600: br = B57600; break;
      case 115200: br = B115200; break;
      case 230400: br = B230400; break;
      case 460800: br = B460800; break;
      case 500000: br = B500000; break;
      case 576000: br = B576000; break;
      case 921600: br = B921600; break;
      case 1000000: br = B1000000; break;
      case 1152000: br = B1152000; break;
      case 1500000: br = B1500000; break;
      case 2000000: br = B2000000; break;
      case 2500000: br = B2500000; break;
      case 3000000: br = B3000000; break;
      case 3500000: br = B3500000; break;
      case 4000000: br = B4000000; break;
      default:
      {
	struct serial_struct ss;
	ioctl (m_fd, TIOCGSERIAL, &ss);

	ss.custom_divisor = ss.baud_base / m_baud_rate;
	if (ss.custom_divisor == 0)
	  ss.custom_divisor = 1;

	m_baud_rate = ss.baud_base / ss.custom_divisor;
	ss.flags &= ~ASYNC_SPD_MASK;
	ss.flags |= ASYNC_SPD_CUST;

	ioctl (m_fd, TIOCSSERIAL, &ss);

	// i guess just any value would be fine, since it's overridden anyway..
	attr.c_cflag |= B38400;
      }
    }

    if (br != B0)
    {
      cfsetispeed (&attr, br);
      cfsetospeed (&attr, br);
    }
  }

  if (cfg.parity == parity::odd)
    attr.c_cflag |= PARENB | PARODD;
  else if (cfg.parity == parity::even)
    attr.c_cflag |= PARENB;

  if (cfg.start_stop_bits_4 == 2*4)
    attr.c_cflag |= CSTOPB;
  else if (cfg.start_stop_bits_4 == 1*4)
  {
  }
  else
  {
    std::cout << "unsupported number of stop bits" << std::endl;
    return false;
  }

  if (cfg.data_bits == 8)
    attr.c_cflag |= CS8;
  else if (cfg.data_bits == 7)
    attr.c_cflag |= CS7;
  else if (cfg.data_bits == 6)
    attr.c_cflag |= CS6;
  else if (cfg.data_bits == 5)
    attr.c_cflag |= CS5; 
  else
  {
    std::cout << "unsupported number of data bits" << std::endl;
    return false;
  }

  if (cfg.flow_ctrl == flow_ctrl::rts_cts)
    attr.c_cflag |= CRTSCTS;
  else if (cfg.flow_ctrl == flow_ctrl::xon_xoff)
    attr.c_cflag |= IXON | IXOFF;
  else
    attr.c_cflag |= CLOCAL;

  // FIXME: unsupported xon/xoff chars.
  m_timeout_ms = cfg.timeout_ms;
  attr.c_cc[VTIME] = m_timeout_ms/100;  // read timeout in deciseconds (100 millisecond steps)
  attr.c_cc[VMIN] = 0;

  tcsetattr (m_fd, TCSANOW, &attr);
  tcflush (m_fd, TCIOFLUSH);
  
  return true;
}

void linux_userspace_usart::reset_receiver (void) noexcept
{
  if (m_fd < 0)
    return;

  tcflush (m_fd, TCIFLUSH);
}

void linux_userspace_usart::reset_transmitter (void) noexcept
{
  if (m_fd < 0)
    return;

  tcflush (m_fd, TCOFLUSH);
}

void linux_userspace_usart::write (const void* data, unsigned int byte_count)
{
  if (m_fd < 0)
    return;

 [[gnu::unused]] auto r = ::write (m_fd, data, byte_count);
}

unsigned int linux_userspace_usart::read (void* data, unsigned int max_byte_count)
{
  if (m_fd < 0)
    return 0;

  auto r = ::read (m_fd, data, max_byte_count);
  return r >= 0 ? r : 0;
}

void linux_userspace_usart::reset_rx_buffer (void) noexcept
{
  if (m_fd < 0)
    return;

  // it seems when using USB based serial adapters, there are issues and
  // a sleep is required...
//  std::this_thread::sleep_for (std::chrono::seconds (2));
  tcflush (m_fd, TCIFLUSH);
}

struct linux_userspace_usart::buffer_stat linux_userspace_usart::buffer_stat (void) const noexcept
{
  int bytes_avail;
  ioctl (m_fd, FIONREAD, &bytes_avail);

  struct buffer_stat st;
  st.rx_error = false;
  st.tx_error = false;
  st.rx_fifo_size = bytes_avail;
  st.rx_fifo_capacity = 1024;
  st.tx_fifo_size = 0;
  st.tx_fifo_capacity = 1024;
  return st;
}

void linux_userspace_usart::set_recv_clb (void f (void* user_p, unsigned int byte_count), void* user_p) noexcept
{
  assert_unreachable ();
}

void linux_userspace_usart::write_port (unsigned int i, bool val)
{
  unsigned int status;
  if (ioctl (m_fd, TIOCMGET, &status) == -1)
  {
    std::cout << "TIOCMGET error" << std::endl;
    return;
  }

  switch ((io_port)i)
  {
    case io_port::rts:
      status = val ? status | TIOCM_RTS : status & ~TIOCM_RTS;
      break;

    case io_port::dtr:
      status = val ? status | TIOCM_DTR : status & ~TIOCM_DTR;
      break;

    default:
      return;
  }

  if (ioctl (m_fd, TIOCMSET, &status) == -1)
    std::cout << "TIOCMSET error" << std::endl;
}

bool linux_userspace_usart::read_port (unsigned int i) const
{
  unsigned int status;
  if (ioctl (m_fd, TIOCMGET, &status) == -1)
  {
    std::cout << "TIOCMGET error" << std::endl;
    return false;
  }

  switch ((io_port)i)
  {
    case io_port::rts: return (status & TIOCM_RTS) != 0;
    case io_port::dtr: return (status & TIOCM_DTR) != 0;
    case io_port::txd: return false;
    case io_port::cts: return (status & TIOCM_CTS) != 0;
    case io_port::dsr: return (status & TIOCM_DSR) != 0;
    case io_port::dcd: return (status & TIOCM_CAR) != 0;
    case io_port::ri: return (status & TIOCM_RI) != 0;
    case io_port::rxd: return false;
    default: return false;
  }
}

} // namespace dev
