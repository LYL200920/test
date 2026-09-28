
#ifndef includeguard_net_http_server_includeguard
#define includeguard_net_http_server_includeguard

// the http server needs TCP
#ifndef NET_NO_TCP

#include <memory>
#include <list>
#include <exception>
#include <net/net.hpp>
#include <net/http.hpp>

#if __has_include (<fs/fs.hpp>)
  #include <fs/fs.hpp>
  #define NET_HTTP_SERVER_ENABLE_HANDLE_FILE_REQUEST
#endif

#include <chrono>

namespace net
{
namespace http
{


class server
{
public:
  struct delegate;
  class connection;

  server (void) = delete;
  server (const server&) = delete;
  server (server&&) = delete;
  server& operator = (const server&) = delete;
  server& operator = (server&&) = delete;

  server (uint16_t port, delegate& d);
  ~server (void);

  void exec (void);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // a request that is being handled inside of the http server.
  // this used to be a composition (http::request stored as a member variable)
  // but later on was changd to inheritance structure.  switching to inheritance
  // structure allowed for easier addition of connection related state handling
  // in the server delegate.

  class request : public http::request
  {
  public:
    request (http::request&& req, connection* conn)
    : http::request (std::move (req)), m_connection (conn) { }

    bool is_valid (void) const { return http::request::is_valid_request (); }

    // FIXME: leftovers from the composition structure.  can be removed but
    // it would require retouch of the code that uses it.
    const http::request& http_request (void) const { return *this; }
    http::request& http_request (void) { return *this; }

    connection* current_connection (void) const { return m_connection; }

  private:
    connection* m_connection;
  };

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  class response
  {
  public:
    enum connection_close_type
    {
      connection_keepalive,
      connection_close
    };

    response (http::response&& res, connection_close_type ct);

    const http::response& http_response (void) const { return m_response; }
    http::response& http_response (void) { return m_response; }

    // try to send some data over the specified socket.
    // return true if this response has been sent completely.
    bool send (net::tcp_socket& socket);

  private:
    enum transmission_state
    {
      tx_begin,
      tx_sending_headers,
      tx_sending_content,
      tx_closing,
      tx_done
    };

    http::response m_response;

    // close the connection after this response has been sent over the socket.
    bool m_close_connection;

    transmission_state m_tx_state;
    unsigned int m_tx_i;

    http::response::content_buffers_type::iterator m_cur_tx_buffer;
    unsigned int m_cur_tx_buffer_tx_i;
    utils::buffer::size_type m_content_size;
  };

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  class connection : public std::enable_shared_from_this<connection>
  {
  public:
    [[gnu::cold]] connection (delegate& delegate, std::unique_ptr<net::tcp_socket> socket);
    ~connection (void);

    void recv (const void* data, unsigned int len);

    // handle pending requests and responses.
    // returns true if the connections is still active or false if it
    // can be cleaned up.
    bool exec (void);

    void send_response (const request& req, http::response&& res);
    void send_error_response (const request& req, net::http::response&& res,
			      std::string_view user_err_msg = { });

    void on_http_get_or_head (const request& req, bool is_get,
			      std::string_view req_uri);

    void on_http_post (const request& req);

    net::tcp_socket* socket (void) const { return m_socket.get (); }

    void close (void);

    http::response
    try_upgrade_to_websocket (const http::request& req, std::string_view req_uri);

  private:
    delegate& m_delegate;
    std::unique_ptr<net::tcp_socket> m_socket;
    bool m_closing = false;

    // request and response queues.
    // front is the input, back is the output.
    // FIXME: use intrusive list for that
    std::list<request> m_request_queue;
    std::list<response> m_response_queue;

    // the message that is currently being received and parsed.
    net::http::message m_parsing_message;

    // if the connection is being upgraded, first all pending responses
    // will be sent and then this function will be called to finalize the
    // connection transform/upgrade
    std::function<void (void)> m_upgrade_func;
  };


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  class delegate_error : public std::exception
  {
  public:
    delegate_error (const char* msg) noexcept : m_msg (msg) { }

    delegate_error (delegate_error&& other)
    {
      m_msg = std::move (other.m_msg);
      m_msg_str = std::move (other.m_msg_str);
    }

    template <typename T>
    delegate_error (T&& str) : m_msg_str (std::forward<T> (str))
    {
      m_msg = m_msg_str.c_str ();
    }


    virtual const char* what (void) const noexcept override { return m_msg; }

    delegate_error& operator = (delegate_error&& other)
    {
      m_msg = std::move (other.m_msg);
      m_msg_str = std::move (other.m_msg_str);
      return *this;
    }

    delegate_error& operator = (const delegate_error& other) = delete;

  private:
    const char* m_msg;
    std::string m_msg_str;
  };


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  struct delegate
  {
    using delegate_error = server::delegate_error;

    // for an incoming http request 'handle_request' is invoked by the
    // http server first.  if the delegate returns a valid response, it will
    // be sent as such.
    //
    // if the delegate returns an invalid/empty response and the request was
    // a HTTP HEAD, the server will convert it into an HTTP GET and ask
    // the delegate again.
    //
    // if the delegate can't return a response because it's waiting for some
    // other thing to complete, it should call "awaiting_result" and return an
    // empty reponse.  the http server will then stall the request processing
    // on that connection and retry it again after some time.
    //
    // otherwise, if the result is not a valid response, the server will try
    // 'handle_file_request'.
    std::chrono::high_resolution_clock::time_point m_awaiting_result_start_time;
    std::chrono::milliseconds m_awaiting_result_duration = std::chrono::milliseconds (0);

    template <typename Rep, typename Period>
    void awaiting_result_retry_after (const std::chrono::duration<Rep, Period>& duration)
    {
      m_awaiting_result_start_time = std::chrono::high_resolution_clock::now ();
      m_awaiting_result_duration = duration;
    }

    bool is_awaiting_result (void) const { return m_awaiting_result_duration.count () != 0; }

    virtual http::response
    handle_request (const net::http::server::request& req, std::string_view req_uri);

    // if the delegate whishes to serve a file for the requested URI, it may
    // do so by returning the according opened file.  the http server will ask
    // the delegate for files after it has tried out normal request handling.
    #ifdef NET_HTTP_SERVER_ENABLE_HANDLE_FILE_REQUEST
    virtual std::unique_ptr<fs::file>
    handle_file_request (const net::http::server::request& req, std::string_view req_uri);
    #endif

    // when sending a response, the http server can include the "Server" header.
    // if the delegate returns a non-empty string_view, the header will be
    // added to the response.
    virtual std::string_view
    server_name (void);

    // when the server closes a connection, notify the delegate, too.
    // if the delegate stores some state based on the connection, or retains
    // the connection pointer, it should release it now.
    virtual void
    connection_closed (net::http::server::connection* conn);
  };


private:
  delegate& m_delegate;

  net::tcp_listening_port m_listening_port;

  std::list<std::shared_ptr<connection>> m_connections;

  friend class connection;
};

} // namespace http
} // namespace net

#endif // NET_NO_TCP
#endif // includeguard_net_http_server_includeguard
