#ifndef includeguard_dev_dma_includeguard
#define includeguard_dev_dma_includeguard

#include <mutex>
#include <vector>
#include <functional>

namespace dev
{
namespace dma
{

class channel;
class dispatcher;

} // namespace dma
} // namespace dev


namespace dev
{
namespace dma
{

/*
  direction
	bidirectional
	to device
	from device
	none

  control
	pause
	resume
	terminate all
	slave config

  scheduled dma
	interrupt management
	lli related
	scheduling hints
	channel management



*/


struct transfer_desc
{
  void* src_addr;
  void* dst_addr;

};

struct transfer
{
  virtual ~transfer (void) { }
  virtual void begin (void) = 0;
  virtual void end (void) = 0;
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
  unsigned int bytes_transferred;
};



class channel
{
public:

  // Lockable
  // locks until a lock can be obtained for the current
  // execution agent (thread, process, task). If an exception is thrown,
  // no lock is obtained.
  virtual void lock (void) = 0;

  // Releases the lock held by the execution agent. Throws no exceptions.
  virtual void unlock (void) = 0;

  // Attempts to acquire the lock for the current execution agent
  // (thread, process, task) without blocking. If an exception is thrown,
  // no lock is obtained.
  virtual bool try_lock (void) = 0;


  // request a lock of this channel.  when it becomes available,
  // the lock is obtained and passed to the callback function.
//  virtual void
//  request_lock (std::function<void (std::unique_lock<dev::dma::channel>)> clb) = 0;

  // perform an asynchronous transfer.
  // transfers are processed by an internal queue.
  virtual std::future<transfer_status>
  transfer (const transfer_desc& desc,
	    std::function<void (void)> completion_clb) = 0;

//  virtual std::future<transfer_status>
//  transfer (

protected:
};


class dispatcher
{
public:
  dispatcher (std::vector<channel*> channels)
  : m_channels (std::move (channels))
  {
  }

protected:
  std::vector<channel*> m_channels;
};

} // namespace dma
} // namespace dev



#endif // includeguard_dev_dma_includeguard
