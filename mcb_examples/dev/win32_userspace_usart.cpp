
#include <iostream>
#include <thread>
#include <cassert>
#include <cstring>

#include <utils/langcomp.hpp>
#include <utils/text.hpp>

#include "win32_userspace_usart.hpp"

#include <windows.h>

namespace dev
{

win32_userspace_usart::win32_userspace_usart (const std::string_view& dev_name)
: m_dev_name (dev_name)
{
  static_assert (std::is_same<decltype (m_dev_handle), HANDLE>::value, "");

  m_dev_handle = CreateFileA ((std::string ("\\\\.\\") + (std::string)dev_name).c_str (),
	GENERIC_READ | GENERIC_WRITE,
	0,
	nullptr,
	OPEN_EXISTING,
	0,
	nullptr);

  if (!is_valid ())
    std::cerr << "could not open device '" << dev_name << "'" << std::endl;
}

win32_userspace_usart::~win32_userspace_usart (void)
{
  CloseHandle (m_dev_handle);
}

bool win32_userspace_usart::is_valid (void) const noexcept
{
  return m_dev_handle != INVALID_HANDLE_VALUE;
}

struct usart::config win32_userspace_usart::config (void) const noexcept
{
  if (!is_valid ())
    return { };

  struct config cfg;

  DCB dcb;
  GetCommState (m_dev_handle, &dcb);

  switch (dcb.BaudRate)
  {
  case CBR_110: cfg.baud_rate = 110; break;
  case CBR_300: cfg.baud_rate = 300; break;
  case CBR_600: cfg.baud_rate = 600; break;
  case CBR_1200: cfg.baud_rate = 1200; break;
  case CBR_2400: cfg.baud_rate = 2400; break;
  case CBR_4800: cfg.baud_rate = 4800; break;
  case CBR_9600: cfg.baud_rate = 9600; break;
  case CBR_14400: cfg.baud_rate = 14400; break;
  case CBR_19200: cfg.baud_rate = 19200; break;
  case CBR_38400: cfg.baud_rate = 38400; break;
  case CBR_57600: cfg.baud_rate = 57600; break;
  case CBR_115200: cfg.baud_rate = 115200; break;
  case CBR_128000: cfg.baud_rate = 128000; break;
  case CBR_256000: cfg.baud_rate = 256000; break;
  default: cfg.baud_rate = dcb.BaudRate;
  }

  if (dcb.fParity)
  {
    switch (dcb.Parity)
    {
    case EVENPARITY:
    case MARKPARITY:
      cfg.parity = parity::even;
      break;

    case ODDPARITY:
    case SPACEPARITY:
      cfg.parity = parity::odd;
      break;

    case NOPARITY:
    default: cfg.parity = parity::none;
    }
  }
  else
    cfg.parity = parity::none;

  switch (dcb.StopBits)
  {
  case ONESTOPBIT: cfg.start_stop_bits_4 = 4; break;
  case ONE5STOPBITS: cfg.start_stop_bits_4 = 6; break;
  case TWOSTOPBITS: cfg.start_stop_bits_4 = 8; break;
  default: cfg.start_stop_bits_4 = 0; break;
  }

  cfg.data_bits = dcb.ByteSize;

  if (dcb.fOutxCtsFlow && dcb.fRtsControl == RTS_CONTROL_HANDSHAKE)
    cfg.flow_ctrl = flow_ctrl::rts_cts;
  else if (dcb.fOutxDsrFlow && dcb.fDtrControl == DTR_CONTROL_HANDSHAKE)
    cfg.flow_ctrl = flow_ctrl::dtr_dsr;
  else if (dcb.fOutX && dcb.fInX)
    cfg.flow_ctrl = flow_ctrl::xon_xoff;
  else
    cfg.flow_ctrl = flow_ctrl::none;

  cfg.xon_char = dcb.XonChar;
  cfg.xoff_char = dcb.XoffChar;

  // FIXME: implement
  cfg.timeout_ms = 0;
  return cfg;
}

bool win32_userspace_usart::set_config (const struct config& cfg) noexcept
{
  if (!is_valid ())
    return false;

  DCB dcb;
  std::memset (&dcb, 0, sizeof (DCB));

  dcb.DCBlength = sizeof (DCB);
  dcb.fBinary = 1;

  dcb.BaudRate = cfg.baud_rate;

  switch (cfg.parity)
  {
  default:
  case parity::none:
    dcb.fParity = 0;
    dcb.Parity = NOPARITY;
    break;

  case parity::odd:
    dcb.fParity = 1;
    dcb.Parity = ODDPARITY;
    break;

  case parity::even:
    dcb.fParity = 1;
    dcb.Parity = EVENPARITY;
  }

  if (cfg.start_stop_bits_4 == 4)
    dcb.StopBits = ONESTOPBIT;
  else if (cfg.start_stop_bits_4 == 6)
    dcb.StopBits = ONE5STOPBITS;
  else if (cfg.start_stop_bits_4 == 8)
    dcb.StopBits = TWOSTOPBITS;
  else
    dcb.StopBits = ONESTOPBIT;

  dcb.ByteSize = cfg.data_bits;

  switch (cfg.flow_ctrl)
  {
  default:
  case flow_ctrl::none:
    // leave default values at zero, i.e. off
    break;

  case flow_ctrl::rts_cts:
    dcb.fOutxCtsFlow = 1;
    dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
    break;

  case flow_ctrl::dtr_dsr:
    dcb.fOutxDsrFlow = 1;
    dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;
    break;

  case flow_ctrl::xon_xoff:
    dcb.fOutX = 1;
    dcb.fInX = 1;
    break;
  }

  dcb.XonChar = cfg.xon_char;
  dcb.XoffChar = cfg.xoff_char;

  if (!SetCommState (m_dev_handle, &dcb))
  {
    std::cerr << "SetCommState NG" << std::endl;
    return false;
  }

  COMMTIMEOUTS timeouts;
  timeouts.ReadIntervalTimeout        = cfg.timeout_ms;
  timeouts.ReadTotalTimeoutConstant   = cfg.timeout_ms;
  timeouts.ReadTotalTimeoutMultiplier = cfg.timeout_ms;

  if (!SetCommTimeouts(m_dev_handle, &timeouts))
  {
    std::cerr << "set timeout failed!" << std::endl;
    return false;
  }

  return true;
}

void win32_userspace_usart::reset_receiver (void) noexcept
{
  FlushFileBuffers (m_dev_handle);
  PurgeComm (m_dev_handle, PURGE_RXABORT | PURGE_RXCLEAR);
}

void win32_userspace_usart::reset_transmitter (void) noexcept
{
  FlushFileBuffers (m_dev_handle);
  PurgeComm (m_dev_handle, PURGE_TXABORT | PURGE_TXCLEAR);
}

void win32_userspace_usart::write (const void* data, unsigned int byte_count) 
{
  DWORD bytes_written = 0;
  if (!WriteFile (m_dev_handle, data, byte_count, &bytes_written, nullptr))
  {
    std::cerr << "error: WriteFile" << std::endl;
    throw std::runtime_error ("error: WriteFile");
  }
  if (bytes_written != byte_count)
  {
    std::string err = utils::printf_format ("error: WriteFile bytes written got %lu, expected %u\n",
                                             bytes_written, byte_count);
    std::cerr << err << std::endl;
    throw std::runtime_error (err);
  }
}

unsigned int win32_userspace_usart::read (void* data, unsigned int max_byte_count) 
{
  DWORD bytes_read = 0;
  if (!ReadFile (m_dev_handle, data, max_byte_count, &bytes_read, nullptr))
  {
    std::cerr << "error: FileRead" << std::endl;
    throw std::runtime_error("error: FileRead");
  }
  return bytes_read;
}

void win32_userspace_usart::reset_rx_buffer (void) noexcept
{
  FlushFileBuffers (m_dev_handle);
  PurgeComm (m_dev_handle, PURGE_RXABORT | PURGE_RXCLEAR);
}

struct win32_userspace_usart::buffer_stat win32_userspace_usart::buffer_stat (void) const noexcept
{
  COMSTAT st;
  DWORD err;
  ClearCommError (m_dev_handle, &err, &st);

  struct buffer_stat st_out;
  st_out.rx_error = err & (CE_FRAME | CE_OVERRUN | CE_RXOVER | CE_RXPARITY);
  st_out.tx_error = false;
  st_out.rx_fifo_size = st.cbInQue;
  st_out.rx_fifo_capacity = 1024;
  st_out.tx_fifo_size = st.cbOutQue;
  st_out.tx_fifo_capacity = 1024;
  return st_out;
}

void win32_userspace_usart::set_recv_clb (void f (void* user_p, unsigned int byte_count), void* user_p) noexcept
{
  assert_unreachable ();
}

bool win32_userspace_usart::read_port (unsigned int i) const
{
  switch ((io_port)i)
  {
    case io_port::cts:
    {
      DWORD r = 0;
      GetCommModemStatus (m_dev_handle, &r);
      return r & MS_CTS_ON;
    }
    case io_port::dsr:
    {
      DWORD r = 0;
      GetCommModemStatus (m_dev_handle, &r);
      return r & MS_DSR_ON;
    }
    case io_port::ri:
    {
      DWORD r = 0;
      GetCommModemStatus (m_dev_handle, &r);
      return r & MS_RING_ON;
    }

    case io_port::dcd:
    case io_port::rxd:
    case io_port::txd:
    case io_port::rts:
    case io_port::dtr:

    default:
      return false;
  }

  return false;
}

void win32_userspace_usart::write_port (unsigned int i, bool val)
{
  switch ((io_port)i)
  {
    case io_port::rts:
      EscapeCommFunction (m_dev_handle, val ? SETRTS : CLRRTS);
      break;

    case io_port::dtr:
      EscapeCommFunction (m_dev_handle, val ? SETDTR : CLRDTR);
      break;

    default:
      break;
  }
}


} // namespace dev
