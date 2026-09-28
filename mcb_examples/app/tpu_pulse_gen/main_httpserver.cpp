
#include <signal.h>

#include <cstdio>
#include <chrono>
#include <thread>
#include <array>

#include <net/net.hpp>
#include <fs/romfs.hpp>
#include <net/http_server.hpp>
#include <utils/text.hpp>

#define log_all
#include <logging/logging.hpp>

#include "app_http_server.hpp"

//--------------------------------------------------------------------------
// a dummy pulse generator

class dummy_pulse_gen : public pulse_gen
{
public:
  virtual void start (void) override { }
  virtual void stop (void) override { }
  virtual void reset (void) override { }
  virtual bool running (void) const override { return false; }
};

static std::array<dummy_pulse_gen, 4> g_pulse_gens;

//--------------------------------------------------------------------------

static auto g_romfs = fs::romfs::mount_this_image_partition ();
static volatile bool g_exit_request = false;

int main (void)
{
  signal (SIGINT, [] (int)
  {
    std::printf ("exiting ... \n");
    g_exit_request = true;
  });

  log_level::enable (log_level::warn);
  log_level::enable (log_level::error);

  app_http_server httpsrv_delegate (g_romfs.get (),
	std::begin (g_pulse_gens), std::end (g_pulse_gens));

  net::http::server httpsrv (8000, httpsrv_delegate);

  httpsrv_delegate.set_log_requests ();

  std::printf ("running http server ... \n");

  for (unsigned int main_loop_count = 0; !g_exit_request; ++main_loop_count)
  {
    net::exec ();
    httpsrv.exec ();

    std::this_thread::sleep_for (std::chrono::milliseconds (50));
  }

  return 0;
}
