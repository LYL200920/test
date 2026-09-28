
// the http server needs TCP
#ifndef NET_NO_TCP

#include <malloc.h>
#include <limits>

#include "http_server.hpp"

//#define log_all
#include <logging/logging.hpp>

namespace net
{
namespace http
{

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

server::response::response (http::response&& res, connection_close_type ct)
: m_response (std::move (res)),
  m_close_connection (ct == connection_close),
  m_tx_state (tx_begin)
{
  log_info ("response::response %s %s %s\n",
	std::string (m_response.status_line ().http_version ()).c_str (),
	std::string (m_response.status_line ().status_code ()).c_str (),
	std::string (m_response.status_line ().reason_phrase ()).c_str ());

  log_info ("response header data:\n-----------\n%s\n-----------\n", m_response.header_data ().c_str ());
}

bool server::response::send (tcp_socket& socket)
{
  for (int retry = 0;
       retry < 2 && m_tx_state != tx_done && socket.send_max_count () > 0;
       ++retry)
  {
    switch (m_tx_state)
    {
      default:
	break;

      case tx_begin:
	m_tx_i = 0;
	m_tx_state = tx_sending_headers;

	[[fallthrough]];

      case tx_sending_headers:
	{
	  unsigned int copy_count =
	    std::min ((unsigned int)m_response.header_data ().size () - m_tx_i,
		      socket.send_max_count ());

	  if (copy_count > 0)
	  {
	    log_info ("http response sending %u header bytes\n", copy_count);

	    socket.send (&m_response.header_data ()[m_tx_i], copy_count,
			 net::tcp_socket::queue_write);
	    m_tx_i += copy_count;
	  }
	}

	if (m_tx_i == m_response.header_data ().size ())
	{
	  m_tx_state = tx_sending_content;
	  m_tx_i = 0;
	  m_cur_tx_buffer_tx_i = 0;
	  m_content_size = utils::sum_buffer_sizes (m_response.content_buffers ());
	  m_cur_tx_buffer = m_response.content_buffers ().begin ();

	  log_info ("http response sent all header data.  first content buffer: %p %u\n",
		    m_cur_tx_buffer->data (), m_cur_tx_buffer->size ());
	}
	break;

      case tx_sending_content:
	while (socket.send_max_count () > 0)
	{
	  unsigned int copy_count =
	    std::min ((unsigned int)(m_cur_tx_buffer != m_response.content_buffers ().end ()
				     ? (m_cur_tx_buffer->size () - m_cur_tx_buffer_tx_i)
				     : 0),
		      socket.send_max_count ());
	  if (copy_count > 0)
	  {
	    log_info ("http response sending %u content bytes   tx_i = %u\n", copy_count, m_cur_tx_buffer_tx_i);
	    socket.send ((const char*)m_cur_tx_buffer->data () + m_cur_tx_buffer_tx_i,
			 copy_count);

	    m_cur_tx_buffer_tx_i += copy_count;
	    m_tx_i += copy_count;

	    if (m_cur_tx_buffer_tx_i == m_cur_tx_buffer->size ())
	    {
	      // finished transmitting the current buffer.
	      // null it out to free memory and go to next.
	      *m_cur_tx_buffer = { };
	      ++m_cur_tx_buffer;
	      m_cur_tx_buffer_tx_i = 0;
	    }
	  }
	  else if (/*copy_count == 0 && */
		   m_cur_tx_buffer != m_response.content_buffers ().end ()
		   && m_cur_tx_buffer->empty ())
	  {
	    // the buffer was empty and thus nothing got copied.
	    // go to the next buffer, if it's not already at the end.
	    *m_cur_tx_buffer = { };
	    ++m_cur_tx_buffer;
	    m_cur_tx_buffer_tx_i = 0;
	  }
	  else
	    break;
	}

	if (m_tx_i == m_content_size
	    || m_cur_tx_buffer == m_response.content_buffers ().end ())
	{
	  m_tx_state = tx_closing;
	  m_tx_i = 0;
	}
	break;

      case tx_closing:
	if (m_close_connection)
	{
	  if (socket.send_buffer_pending_count () > 0)
	      log_info ("http response waiting to close connection\n");
	  else
	  {
	    log_info ("http response closing connection\n");
	    socket.close ();
	    m_tx_state = tx_done;
	  }
	}
	else
	{
	  log_info ("http response sent all\n");
	  m_tx_state = tx_done;
	}
	break;
    }
  }

  socket.flush_send_buffer ();
  return m_tx_state == tx_done;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

[[gnu::cold]] server::connection
::connection (delegate& delegate, std::unique_ptr<net::tcp_socket> socket)
: m_delegate (delegate), m_socket (std::move (socket))
{
  log_info ("new http connection %p\n", this);

  // null out the socket, which will make exec return false.
  // the http server container will then remove the connection object.
  m_socket->on_close ([this] (net::tcp_socket*)
  {
    m_socket = nullptr;
  });

  using namespace std::placeholders;
  m_socket->on_receive (std::bind (&connection::recv, this, _2, _3));
}

server::connection::~connection (void)
{
  if (m_socket != nullptr)
    m_socket->on_close (nullptr);
}

void server::connection::close (void)
{
  // once the socket has been closed, the connection will be deleted
  // automatically.
  m_socket->close ();
  m_closing = true;

  m_request_queue.clear ();
}

void server::connection::recv (const void* data, unsigned int len)
{
  #ifdef __cpp_exceptions
  try
  #endif
  {
    log_info ("\n\nhttp %p received %u bytes\n\n", this, len);

    const char* in = (const char*)data;
    const char* const in_end = in + len;

    for (; in != in_end; )
    {
      auto&& new_in = m_parsing_message.parse (in, in_end);

      log_info ("http parse message in %p -> %p (%u)\n", in, new_in,
		(unsigned int)(new_in - in));

      in = new_in;

      if (m_parsing_message.is_parse_error ())
      {
	log_info ("http parsing error, closing connection\n");

	// FIXME: drain pending responses.  this might have been not the first
	// message, but the n-th message, for which we have already generated
	// some good responses.
	close ();
      }

      else if (m_parsing_message.is_parsing_done ())
      {
	log_info ("http message parsing done\n");

	if (m_parsing_message.is_parse_error ())
	{
	  log_info ("http message parsing error, closing connection\n");
	  close ();
	  return;
	}

	// even if this is an invalid or bad request, we need to process
	// all requests in-order and there might be other requests in the
	// queue.  otherwise we might generate an out-of-order response if
	// the bad request was handled immediately.
	m_request_queue.emplace_front (std::move (m_parsing_message), this);
	m_parsing_message = { };
      }

      else if (m_parsing_message.is_parsing_headers_done ())
      {
	log_info ("http message parsing headers done\n");
/*
	// do some early checks before receiving and buffering more stuff.
	if (!m_parsing_message.is_valid_request ())
	{
	  log_info ("http invalid request, closing connection\n");
	  // FIXME: see above.
	  close ();
	}
*/
      }
    }
  }
  #ifdef __cpp_exceptions
  catch (const std::exception&)
  {
    // there was some exception while processing input data.
    // most likely it ran out of memory and the message parsing state could
    // be broken.  close the socket (which in turn will close this connection).
    close ();
  }
  #endif
}

bool server::connection::exec (void)
{
  if (m_socket == nullptr)
    return false;

  if (m_closing)
    return true;

  if (m_delegate.is_awaiting_result ())
  {
    if (std::chrono::high_resolution_clock::now () - m_delegate.m_awaiting_result_start_time
	>= m_delegate.m_awaiting_result_duration)
    {
      m_delegate.m_awaiting_result_duration = std::chrono::milliseconds (0);
      // retry this request in the below loop.
    }
    else
    {
      // keep waiting
      return true;
    }
  }

  while (!m_request_queue.empty ())
  {
    auto& r = m_request_queue.back ();
    const auto& rl = r.http_request ().request_line ();

    if (!r.is_valid ()
	|| !(rl.http_version () == "HTTP/1.0" || rl.http_version () == "HTTP/1.1"))
    {
      log_info ("bad request %d %d %d\n",
		!r.is_valid (),
		rl.http_version () != "HTTP/1.0",
		rl.http_version () != "HTTP/1.1");

      m_response_queue.emplace_front (
      net::http::response ("HTTP/1.1", "400", "Bad Request"),
			   response::connection_close);
    }
    else
    {
      bool handled = false;
      std::string_view req_uri = r.http_request ().request_line ().request_uri ();

      // FIXME: for now remove everything that is behind a '?'.
      // should actually parse those parameters inside the http request line.
      // currently this is only used for cache busting ajax requests.
      req_uri = req_uri.substr (0, req_uri.find ('?'));

      if (!handled && !r.http_request ().header ("Expect").empty ())
      {
	send_error_response (r,
	  net::http::response (rl.http_version (), "417", "Expectation Failed"));
	handled = true;
      }

      if (!handled)
      {
	#if __cpp_exceptions
	try
	#endif
	{
	  // handle connection upgrades in a special way, because we might
	  // need to transform this connection object into something else.
	  // if there is some problem (e.g. out of memory), this might actually
	  // fail and it will be treated like any other error.
	  if (r.http_request ().header ("Connection").value () == "Upgrade")
	  {
	    // FIXME: could use generic list of "upgrade handler/delegate"
	    // approach that tries them all.  but actually there aren't that
	    // many official connection upgrade options.
	    if (r.http_request ().header ("Upgrade").value () == "websocket")
	    {
	      auto resp = try_upgrade_to_websocket (r.http_request (), req_uri);
	      if (resp.is_valid_response ())
	      {
		send_response (r, std::move (resp));
		handled = true;
	      }
	    }
	    else
	    {
	      // ignore unknown connection upgrade requests.
	      // e.g. requests to upgrade to http2 can be silently ignored
	      // if they are not supported by the server.
	    }
	  }

	  if (!handled)
	  {
	    auto resp = m_delegate.handle_request (r, req_uri);

	    if (resp.is_valid_response ())
	    {
	      send_response (r, std::move (resp));
	      handled = true;
	    }
	    else if (m_delegate.is_awaiting_result ())
	    {
	      // delegate is processing the current request and is waiting
	      // for some other result.  keep the current request in the
	      // queue, it will be processed later again.
	      break;
	    }
	    else
	    {
	      // the delegate didn't handle the request.
	      if (r.http_request ().request_line ().method () == "HEAD")
	      {
		// we didn't find a direct handler for the uri and the method.
		// if it's a HEAD request, try to find a GET handler and strip
		// off the content.

		// N.B. the original request method string_view points into the
		// received string data.  we replace that with a string_view to
		// constant data.  if the delegate wants to clone the request
		// (for whatever purpose), it's still possible.
		// the request here is going to be thrown away anyway after
		// processing, so it's OK to modify its contents.
		auto prev_req_method = r.http_request ().request_line ().method ();
		r.http_request ().request_line ().method () = "GET";
		auto resp = m_delegate.handle_request (r, req_uri);

		if (resp.is_valid_response ())
		{
		  send_response (r, std::move (resp));
		  handled = true;
		}
		else
		{
		  // put back the original request method data.
		  r.http_request ().request_line ().method () = prev_req_method;
		}
	      }
	    }
	  }
	}
	#if __cpp_exceptions
	catch (const std::bad_alloc&)
	{
	  // sometimes we run out of memory ...
	  // with current browsers the "Retry-After" is just a hint and
	  // will not automatically try to reload the page.  the "Refresh"
	  // will do that usually.
	  net::http::response resp (rl.http_version (), "503", "Service Unavailable");
	  resp.add_header ({"Retry-After", "2"});
	  resp.add_header ({"Refresh", std::string ("2; url=") + std::string (rl.request_uri ())});

	  send_error_response (r, std::move (resp));
	  handled = true;

	  log_warn ("http handler bad_alloc\n");
	}
	catch (const std::exception& e)
	{
	  send_error_response (r,
		net::http::response (rl.http_version (), "500", "Internal Server Error"),
		e.what ());
	  handled = true;
	  log_warn ("http handler exception: %s\n", e.what ());
	}
	#endif
      }

      if (!handled)
      {
	if (r.http_request ().request_line ().method () == "GET")
	  on_http_get_or_head (r, true, req_uri);
	else if (r.http_request ().request_line ().method () == "HEAD")
	  on_http_get_or_head (r, false, req_uri);
	else if (r.http_request ().request_line ().method () == "POST")
	  on_http_post (r);
	else
	{
	  send_error_response (r,
		net::http::response (rl.http_version (), "501", "Not Implemented"));
	}
      }
    }

    m_request_queue.pop_back ();
  }

  while (!m_response_queue.empty ()
	 && m_socket->state () == net::tcp_socket::connected)
  {
    auto& r = m_response_queue.back ();

    if (r.send (*m_socket))
    {
      // sending is done.  proceed with the next.
      m_response_queue.pop_back ();
    }
    else
      break;
  }

  return true;
}

void server::connection
::send_response (const request& req, net::http::response&& res)
{
  log_info ("send response %s %s %s\n",
	    std::string (res.status_line ().http_version ()).c_str (),
	    std::string (res.status_line ().status_code ()).c_str (),
	    std::string (res.status_line ().reason_phrase ()).c_str ());

  auto server_name = m_delegate.server_name ();
  if (!server_name.empty ())
    res.add_header ({ "Server", server_name });

  // notice: must not add a content-length header if a transfer encoding has
  // been applied.
  if (!res.header ("Content-Length"))
  {
    auto len_val = utils::sum_buffer_sizes (res.content_buffers ());

    char str[std::numeric_limits< decltype (len_val) >::digits10 + 3];
    std::size_t str_len = std::sprintf (str, "%zu", len_val);

    res.add_header ({ "Content-Length", { str, str_len } });
  }

  // a connection is not explicitly closed, leave it open.
  // FIXME: things like "Connection: keep-alive, close" are also possible...
  // keep-alive is HTTP/1.0.  in HTTP/1.1 keep-alive is implicit.
  response::connection_close_type cc =
	req.http_request ().header ("Connection").value () == "close"
	? response::connection_close
	: response::connection_keepalive;

  if (cc == response::connection_close)
    res.add_header ({ "Connection", "close" });

  m_response_queue.emplace_front (std::move (res), cc);
}

void server::connection
::send_error_response (const request& req, net::http::response&& res,
		       std::string_view user_err_msg)
{
  log_info ("send response %s %s %s\n",
	    std::string (res.status_line ().http_version ()).c_str (),
	    std::string (res.status_line ().status_code ()).c_str (),
	    std::string (res.status_line ().reason_phrase ()).c_str ());

  std::string err_msg = std::string (res.status_line ().status_code ()) + " "
			+ std::string (res.status_line ().reason_phrase ());
  std::string cont;
  cont.reserve (256 + user_err_msg.size ());

  cont += "<html><head><title>";
  cont += err_msg;
  cont += "</title></head><body bgcolor=\"white\"><center><h1>";
  cont += err_msg;
  cont += "</h1></center><hr/><center>";
  cont.append (user_err_msg.data (), user_err_msg.size ());
  cont += " </center></body></html>";

  res.add_header ({ "Content-Type", "text/html" });
  res.add_header ({ "Content-Length", std::to_string (cont.size ()) });
  res.set_content (cont);

  auto server_name = m_delegate.server_name ();
  if (!server_name.empty ())
    res.add_header ({ "Server", server_name });

  res.add_header ({ "Connection", "close" });

  // a connection is not explicitly closed, leave it open.
  // FIXME: things like "Connection: keep-alive, close" are also possible...
  // keep-alive is HTTP/1.0.  in HTTP/1.1 keep-alive is implicit.
  m_response_queue.emplace_front (std::move (res), response::connection_close);
}


/*
An origin server that does differentiate resources based on the host requested
(sometimes referred to as virtual hosts or vanity host names) MUST use the
following rules for determining the requested resource on an HTTP/1.1 request:

1. If Request-URI is an absoluteURI, the host is part of the Request-URI.
Any Host header field value in the request MUST be ignored.

2. If the Request-URI is not an absoluteURI, and the request includes a Host
header field, the host is determined by the Host header field value.

3. If the host as determined by rule 1 or 2 is not a valid host on the server,
the response MUST be a 400 (Bad Request) error message.

*/

void server::connection
::on_http_get_or_head (const request& req, bool is_get, std::string_view req_uri)
{
  log_info ("on_http_get_or_head %d uri = %s\n", is_get,
	    std::string (req_uri).c_str ());

  if (req_uri == "/" || req_uri.empty ())
    req_uri = "index.html";

  else if (req_uri.front () == '/')
    req_uri.remove_prefix (1);

#ifdef NET_HTTP_SERVER_ENABLE_HANDLE_FILE_REQUEST
  auto f = m_delegate.handle_file_request (req, req_uri);

  if (f == nullptr)
  {
    log_info ("file not found\n");

    send_error_response (req, net::http::response (
	req.http_request ().request_line ().http_version (),
	"404", "Not Found"));
  }
  else
  {
    log_info ("file found size = %zu\n", (size_t)f->size ());

    net::http::response res;
    bool send_content = false;

    auto ti = f->time_info ();

    if (!ti.modified_str.empty ()
	&& ti.modified_str == req.http_request ().header ("if-modified-since").value ())
    {
      res = net::http::response (req.http_request ().request_line ().http_version (),
				 "304", "Not Modified");
    }
    else
    {
      send_content = true;
      res = net::http::response  (req.http_request ().request_line ().http_version (),
				  "200", "OK");
    }

    if (!ti.modified_str.empty ())
    {
      res.add_header ({ "Last-Modified", ti.modified_str });
      res.add_header ({ "Cache-Control", "public, max-age=30" });
    }

    res.add_header ({ "Content-Type", f->mime_type () });
    res.add_header ({ "Content-Length", std::to_string (f->size ()) });

    // allow cross-site access of hosted file resources
    // this can be useful for debugging and developing.
    //res.add_header ({ "Access-Control-Allow-Origin", "*" });

    if (is_get && send_content)
    {
      #if __cpp_exceptions
      try
      #endif
      {
	if (const void* mapped_file_data = f->mmap_rd ())
	{
	  struct mapped_file_buffer : public utils::buffer_data
	  {
	    std::unique_ptr<fs::file> m_file;
	    const void* m_data;

	    mapped_file_buffer (std::unique_ptr<fs::file> file,
				const void* mapped_ptr)
	    : m_file (std::move (file)), m_data (mapped_ptr) { }

	    virtual ~mapped_file_buffer (void) override { }
	    virtual const void* data (void) const override { return m_data; }
	    virtual size_type size (void) const override { return m_file->size (); }
	  };

	  res.set_content (http::response::content_buffers_type {
		utils::buffer (utils::make_ref_counted<mapped_file_buffer> (
				std::move (f), mapped_file_data)) } );
	}
	else
	{
	  std::string str (f->size (), 0);
	  f->read ((void*)str.data (), f->size ());
	  res.set_content (std::move (str));
	}
      }
      #if __cpp_exceptions
      catch (const std::exception& e)
      {
	log_info ("set content exception: %s\n", e.what ());
	struct mallinfo mi [[gnu::unused]] = mallinfo ();
	log_info ("  mallinfo.arena    = %zu\n"
		  "  mallinfo.ordblks  = %zu\n"
		  "  mallinfo.smblks   = %zu\n"
		  "  mallinfo.hblks    = %zu\n"
		  "  mallinfo.hblkhd   = %zu\n"
		  "  mallinfo.usmblks  = %zu\n"
		  "  mallinfo.fsmblks  = %zu\n"
		  "  mallinfo.uordblks = %zu\n"
		  "  mallinfo.fordblks = %zu\n"
		  "  mallinfo.keepcost = %zu\n", mi.arena, mi.ordblks,
		  mi.smblks, mi.hblks, mi.hblkhd, mi.usmblks, mi.fsmblks,
		  mi.uordblks, mi.fordblks, mi.keepcost);
      }
      #endif
    }

    send_response (req, std::move (res));
  }

#else // NET_HTTP_SERVER_ENABLE_HANDLE_FILE_REQUEST

  log_info ("file not found\n");

  send_error_response (req, net::http::response (
	req.http_request ().request_line ().http_version (),
	"404", "Not Found"));

#endif
}

void server::connection::on_http_post (const request& req)
{
  // if it's a POST request without a content length, reject it.
  // we can't receive streaming data of unknown content length yet.
  // it would trigger an out-of-memory.
  // send a "411 Length Required" response.

  log_info ("on_http_post\n");
  send_error_response (req,
    net::http::response (req.http_request ().request_line ().http_version (),
    "501", "Not Implemented"));
}

http::response
server::connection::try_upgrade_to_websocket (const http::request& req,
					      std::string_view req_uri)
{
  std::printf ("try_upgrade_to_websocket %p\n", this);
  return { };
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

[[gnu::cold]] server::server (uint16_t port, delegate& d)
: m_delegate (d)
{
  m_listening_port = net::tcp_listening_port (port,
  [this] (std::unique_ptr<net::tcp_socket> new_socket)
  {
    // allow the system to close the connections automatically.
    new_socket->set_auto_close_enabled ();

    #ifdef __cpp_exceptions
    try
    #endif
    {
      m_connections.emplace_back (
	std::make_shared<connection> (m_delegate, std::move (new_socket)));
    }
    #ifdef __cpp_exceptions
    catch (const std::exception&)
    {
      // something went wrong and the connection could not be created.
      // the socket will be deleted automatically.
      return;
    }
    #endif
  });
}

[[gnu::cold]] server::~server (void)
{

}

void server::exec (void)
{
  for (auto i = std::begin (m_connections); i != std::end (m_connections); )
  {
    #ifdef __cpp_exceptions
    try
    #endif
    {
      if ((*i)->exec () == false)
      {
	log_info ("http removing dead connection %p\n", (*i).get ());
	m_delegate.connection_closed (i->get ());
	i = m_connections.erase (i);
	continue;
      }
    }
    #ifdef __cpp_exceptions
    catch (const std::exception&)
    {
      // if the processing of a connection propagates and exception to here
      // then something went quite wrong and it wasn't able to deal with it.
      // the best thing we can do here is to close the connection.
      (*i)->close ();
    }
    #endif

    ++i;
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// server delegate default functions

http::response
http::server::delegate::handle_request (const net::http::server::request&/* req*/,
					std::string_view/* req_uri*/)
{
  return { };
}

#ifdef NET_HTTP_SERVER_ENABLE_HANDLE_FILE_REQUEST
std::unique_ptr<fs::file>
http::server::delegate::handle_file_request (const net::http::server::request&/* req*/,
					     std::string_view/* req_uri*/)
{
  return { };
}
#endif

std::string_view
http::server::delegate::server_name (void)
{
  return { };
}

void
http::server::delegate::connection_closed (net::http::server::connection* /*conn*/)
{
}

} // namespace http
} // namespace net

#endif // NET_NO_TCP
