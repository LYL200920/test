
#ifndef includeguard_app_http_server_includeguard
#define includeguard_app_http_server_includeguard

#include <functional>
#include <vector>

#include <fs/fs.hpp>
#include <net/http_server.hpp>

#include "pulse_gen.hpp"

class app_http_server : public net::http::server::delegate
{
public:
  template <typename Iter>
  app_http_server (fs::partition* fs, Iter gens_begin, Iter gens_end)
  : m_fs (fs), m_pulse_gens (gens_begin, gens_end) { }

  virtual std::unique_ptr<fs::file>
  handle_file_request (const net::http::request& req,
		       std::experimental::string_view req_uri) override;


  virtual net::http::response
  handle_request (const net::http::request& req,
		  std::experimental::string_view req_uri) override;

  void set_log_requests (bool val = true) { m_log_requests = val; }

private:
  fs::partition* m_fs;
  bool m_log_requests;

  std::vector<std::reference_wrapper<pulse_gen>> m_pulse_gens;

  net::http::response html_root (const net::http::request& req);
};

#endif // includeguard_app_http_server_includeguard


