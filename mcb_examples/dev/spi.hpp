/*

SPI (Serial Peripheral Interface) bus interface classes and definitions.

the spi_master interface class is implemented by some concrete spi master
device which can initiate send and receive operations to one or more
spi slave devices.

the actual SS/CS (slave-select / chip-select) line assertion is handled in
an implementation defined way.  there can be zero or many SS/CS lines, which
are addressed by an integer in the transfer operation.  how exactly the spi
master asserts those lines, depends on its implementation.  some options are:

 * an SS line that is asserted automatically by an spi hardware peripheral

 * an SS line that is asserted by changing the state of an GPIO before/after
   starting the send/receive operation in software

 * an SS line that is asserted by changing the state of an GPIO before/after
   starting the send/receive operation in hardware via DMA/DTC.

 * when using an SPI hub, multiple SS lines are controlled by a prefix
   command which is automatically injected by the spi master.

typical spi memory transfers consist of a command, address and data phase.
depending on the command, the spi lines can be used in different
modes during each phase.  for example when accessing memory devices,
6-wire qspi can be used for the data phase only, or also for the address
phase, while the command byte is usually transferred in single-spi mode.
the details of that are usually defined by the spi device and can vary.
to support multiple device types, the spi master allows the definition of
multiple transfer phases.

an spi device driver usually will configure and prepare the transfer setups
that it needs and store the resulting prepared transfer objects.  this gives
the spi master driver a chance to optimize the transfer setup and decide
how to conduct the transfer.  the returned prepared transfer objects then
reflect the actually used parameters, which might deviate from the originally
requested parameters.  for instance, an spi device driver might request
a transfer bitrate of 50 Mbps, but the actually used bitrate will be set
as 6.3333 Mbps due to capabilities and restrictions of the spi master device.

when performing transfers the spi device driver specifies the prepared transfer
object along with some buffer sequences to send/receive the actual data.

*/


#ifndef includeguard_dev_spi_hpp_includeguard
#define includeguard_dev_spi_hpp_includeguard

#include <cstdint>
#include <utils/buffer.hpp>
#include <vector>
#include <exception>
#include <future>
#include <memory>
#include <forward_list>

namespace dev
{
namespace spi
{

enum lane_mode_t
{
  // use single-lane in both directions at the same time.
  // this will use both lines, MOSI and MISO.
  single_lane_full_duplex,

  // use single-lane in one direction at the time,
  // as specified in the data direction in the phase setting.
  single_lane_half_duplex,

  // dual and quad lane are always half duplex, according to the
  // data direction in the phase setting.
  dual_lane,
  quad_lane,
};

enum data_direction_t
{
  // read and write data during a phase
  // for full-duplex (single lane) only.
  read_write,

  // read (receive) data during a phase.
  read,

  // write (send) data during a phase.
  write,

  // disable data transfer.
  // see ESP32 "dummy phase", e.g. for RAM access time delay
  no_data
};

enum clock_phase_t
{
  latch_odd_shift_even,  // data latch on odd clock edge, data shift on even clock edge
  latch_even_shift_odd   // data latch on even edge, data shift on odd edge
};

enum clock_polarity_t
{
  positive,
  negative
};

enum bit_order_t
{
  msb_first,
  lsb_first
};

enum byte_order_t
{
  // send/receive single bytes.
  single_byte       = 0b000,

  // send/receive 2-byte words from the bit stream.
  // store them in the order they were received.
  two_byte = 0b001,

  // send/receive 4-byte words from the bit stream.
  // store them in the order they were received.
  quad_byte = 0b010,

  // send/receive 2-byte words from the bit stream.
  // store them in the reverse order they were received.
  two_byte_swapped  = 0b101,

  // send/receive 4-byte words from the bit stream.
  // store them in the reverse order they were received.
  quad_byte_swapped = 0b110
};

enum line_output_mode_t
{
  // keep the line at its last output value
  output_last_value,

  // output a low signal
  output_low,

  // output a high signal
  output_high,

  // let the line float
  high_z
};


struct transfer_config
{
  struct phase_config
  {
    struct [[gnu::packed]]
    {
      // allow some trailing clock (and data) output after this phase.
      // some SPI devices are OK with that, as they use only a fixed number
      // of data/clock cycles and ignore everything else that follows.
      // if trailing clock output is allowed, it gives the SPI driver
      // more freedom to optimize transfers that consist of muliple phases.
      bool allow_trailing_clock : 1;

      data_direction_t data_dir : 2;
      clock_phase_t clock_phase : 1;
      clock_polarity_t clock_polarity : 1;
      byte_order_t byte_order : 3;
      bit_order_t bit_order : 1;
      lane_mode_t lane_mode : 3;

      // how all the data IOs should behave after this phase
      line_output_mode_t data_idle_output_mode : 2;

      // how unused data lines should behave during this phase
      line_output_mode_t data2_unused_output_mode : 2;
      line_output_mode_t data3_unused_output_mode : 2;

      // how the CS output should behave after this phase
      line_output_mode_t cs_idle_output_mode : 2;

      // how the clock output should behave after this phase
      line_output_mode_t clk_idle_output_mode : 2;
    };

    // divides the base bitrate by this number
    uint8_t bitrate_div;

    // which CS line number to use (enable/disable) for this phase.
    uint8_t cs_line;

    // number of transfer units (bytes/words, ..)
    // the last phase can have 0 to transfer as many units as there is
    // data buffer capacity left.  the last transfer can also be set to
    // non-zero to always transfer a fixed amount of units.
    // setting 0 to other phases than the last phase will effectively make
    // the first phase with 0 the last one and the following phases will be
    // ignored.
    uint32_t transfer_count;

    // clock output delay for this phase
    // 0 for disable.  unit is system clock cycles (implementation defined).
    // some devices might only allow a non-zero clock delay for the first
    // phase and will output a continuous clock for all subsequent phases.
    // some devices might allow only the same delay for all phases
    // or no delay.
    uint16_t clock_delay_cycles;

    // CS output delay for this phase.
    // 0 for disable.  unit is system clock cycles (implementation defined).
    // some devices might only allow a non-zero clock delay for the first
    // phase and will output a continuous clock for all subsequent phases.
    // some devices might allow only the same delay for all phases
    // or no delay.
    uint16_t cs_delay_cycles;

    // delay until the next phase.
    // 0 for disable.  unit is system clock cycles (implementation defined).
    // some devices might allow only the same delay for all phases
    // or no delay.
    uint16_t next_phase_delay_cycles;
  };

  // base bitrate for the whole transfer
  unsigned int bitrate_hz;

  // a transfer consists of at least 1 or more phases.
  std::forward_list<phase_config> phases;
};


class prepared_transfer : public std::enable_shared_from_this<prepared_transfer>
{
public:
  // the config parameters that were requested by the user
  const transfer_config requested_config;

  // the config parameters that are actually going to be used
  const transfer_config using_config;

  prepared_transfer (void) = delete;
  prepared_transfer (const prepared_transfer&) = delete;
  prepared_transfer (prepared_transfer&&) = delete;

  virtual ~prepared_transfer (void) { }

protected:
  prepared_transfer (const transfer_config& r, const transfer_config& u)
  : requested_config (r), using_config (u) { }

  // device can append more information here through sub-class.
};

enum completion_status
{
  pending,
  finished,
  cancelled,
  failed
};

struct transfer_status
{
  completion_status completion;
  unsigned int bytes_sent;
  unsigned int bytes_received;
};

} // namespace spi
} // namespace dev

namespace std
{

template <> class future<dev::spi::transfer_status>
{
public:
  future (void) noexcept { }
  future (future&& rhs) noexcept { }
  future (const future&) = delete;
  ~future (void) { }

  future& operator = (const future&) = delete;
  future& operator = (future&&) noexcept;

  // shared_future<R> share();

  // does not block or wait.  just gets the live status and byte counts
  // if supported by the device.
  dev::spi::transfer_status peek (void) const;

  // like future::get
  // blocks until completion
  dev::spi::transfer_status get (void);

  void cancel (void);

  bool valid (void) const;
  void wait (void) const;

//  template <class Rep, class Period>
//  std::future_status wait_for (const std::chrono::duration<Rep, Period>& timeout_duration) const;

//  template< class Clock, class Duration >
//  std::future_status wait_until (const std::chrono::time_point<Clock,Duration>& timeout_time ) const;
};

} // namespace std


namespace dev
{
namespace spi
{


using buffer = utils::buffer;
using buffer_sequence_t = std::vector<buffer>;

struct master
{
  // prepare a transfer with the specified configuration settings.
  // the spi master device will not own this object if there is no
  // transfer active.  when starting a transfer with a prepared_transfer
  // object, the spi master device takes temporary shared ownership.
  // use the returned object for subsequent transfer operations.
  // might throw an std::invalid_argument.
  virtual std::shared_ptr<prepared_transfer>
  prepare_transfer (const transfer_config& cfg) const = 0;

  // enqueue a transfer.
  // if there is currently no other transfer pending, the transfer
  // is started immediately.  otherwise it is queued.
  //
  // the buffers in the specified buffer sequences are filled up/drained
  // completely before going to the next buffer in the sequence.
  //
  // if the send buffer sequence is empty, no data or dummy data will be
  // sent.
  // if the receive buffer sequence is empty, no data will be received
  // or received data will be discarded.

  virtual std::future<transfer_status>
  transfer (const std::shared_ptr<prepared_transfer>& transfer,
	    buffer_sequence_t&& send_buffer,
	    buffer_sequence_t&& recv_buffer,
	    std::function<void (void)> completion_clb = nullptr) = 0;
};


} // namespace spi
} // namespace dev
#endif // includeguard_dev_spi_hpp_includeguard
