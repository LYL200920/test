
#include <cstdio>
#include "app_http_server.hpp"

std::unique_ptr<fs::file>
app_http_server::handle_file_request (const net::http::request& req,
				      std::experimental::string_view req_uri)
{
  if (m_fs != nullptr)
      return m_fs->open_file (req_uri);

  return { };
}

net::http::response
app_http_server::handle_request (const net::http::request& req,
				 std::experimental::string_view req_uri)
{
/*
  if (req_uri == "/" && req.request_line ().method () == "GET")
    return html_root (req);
*/

  if (m_log_requests)
    std::printf ("%s  %s\n", req.request_line ().method ().to_string ().c_str (),
			     req_uri.to_string ().c_str ());

  return { };
}

net::http::response
app_http_server::html_root (const net::http::request& req)
{
  std::string out;
  out.reserve (1024*8);

  out += R"raw(

<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN" "DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en" lang="en">
<head><title>MCB TPU Pulse Gen</title></head>

<body onload="javascript:on_page_load()">
<div class="content"><div class="centered">

</div>
</body></html>

)raw";

  return net::http::make_200_ok_response (req, "text/html", std::move (out));
}

