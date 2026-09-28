/*

a generic device interface which contains functionality that is normally
present on each IO or data communications device.

*/

#ifndef includeguard_dev_generic_io_hpp_includeguard
#define includeguard_dev_generic_io_hpp_includeguard

#include <type_traits>

namespace dev
{

struct generic_io
{
  // reset the device or clear the current errors to bring it back to
  // an operational state.
  virtual void reset_receiver (void) noexcept { }
  virtual void reset_transmitter (void) noexcept { }

  // synchronous write.  blocks the caller until all the specified bytes
  // have been copied to the transmit buffer or put on the wire.
  // not safe to use from within ISRs.
  virtual void write (const void* data, unsigned int byte_count) = 0;

  template <typename T, unsigned int N, std::enable_if_t<sizeof (T) == 1, bool> = true>
  void write (const T (&bytes)[N])
  {
    write (bytes, N);
  }


  // synchronous read.  blocks the caller until at least one byte is read.
  // returns the total number of bytes read.
  virtual unsigned int read (void* data, unsigned int max_byte_count) = 0;

  // set the data received callback function.  the function will be invoked by
  // the driver whenever some new data has been read.
  // the callback function can be invoked from within some ISR context, so
  // it must not block or invoke other functions which can't be used from
  // within an ISR context.

  // FIXME: maybe also receive callback threshold to avoid too many callback calls.

  virtual void set_recv_clb (void f (void* user_p, unsigned int byte_count), void* user_p) noexcept = 0;

  template <typename PtmfCallback>
  void set_recv_clb (typename PtmfCallback::class_type* p, PtmfCallback) noexcept
  {
    set_recv_clb (PtmfCallback::template invoke<unsigned int>, p);
  }

  virtual void reset_rx_buffer (void) noexcept { }

// FIXME: implement later.
//  virtual void set_receive_enable (bool val = true) noexcept { }
//  virtual void set_transmit_enable (bool val = true) noexcept { }

  struct buffer_stat
  {
    bool rx_error : 1;
    bool tx_error : 1;
    unsigned int rx_fifo_size;     // number of bytes currently in the rx fifo.
    unsigned int rx_fifo_capacity;
    unsigned int tx_fifo_size;     // number of bytes currently in the tx fifo.
    unsigned int tx_fifo_capacity;
  };

  virtual struct buffer_stat buffer_stat (void) const noexcept = 0;

  // return the number of bytes currently in the receive/transmit fifo
  unsigned int rx_fifo_size (void) const noexcept { return buffer_stat ().rx_fifo_size; }
  unsigned int tx_fifo_size (void) const noexcept { return buffer_stat ().tx_fifo_size; }

  // return the receive/transmit fifo max counts
  unsigned int rx_fifo_capacity (void) const noexcept { return buffer_stat ().rx_fifo_capacity; }
  unsigned int tx_fifo_capacity (void) const noexcept { return buffer_stat ().tx_fifo_capacity; }

  bool rx_fifo_full (void) const noexcept { return rx_fifo_size () >= rx_fifo_capacity (); }
  bool tx_fifo_full (void) const noexcept { return tx_fifo_size () >= tx_fifo_capacity (); }

  bool rx_fifo_empty (void) const noexcept { return rx_fifo_size () == 0; }
  bool tx_fifo_empty (void) const noexcept { return tx_fifo_size () == 0; }

  unsigned int rx_fifo_remaining_size (void) const noexcept { return rx_fifo_capacity () - rx_fifo_size (); }
  unsigned int tx_fifo_remaining_size (void) const noexcept { return tx_fifo_capacity () - tx_fifo_size (); }

  bool is_receive_error (void) const noexcept { return buffer_stat ().rx_error; }
  bool is_transmit_error (void) const noexcept { return buffer_stat ().tx_error; }
};

} // namespace dev
#endif // includeguard_dev_generic_io_hpp_includeguard
