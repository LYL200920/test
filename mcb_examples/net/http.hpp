
/*
  HTTP - hyper text transport protocol

 https://tools.ietf.org/html/rfc2616

*/

#ifndef includeguard_net_http_includeguard
#define includeguard_net_http_includeguard

#include <string_view>
#include <list>
#include <array>
#include <string>
#include <vector>
#include <limits>
#include <utils/buffer.hpp>
#include <net/uri.hpp>

namespace net
{
namespace http
{

// -----------------------------------------------------------------------------
// FIXME: the start_line is usually inside a message and the string_views
// point into the message's string data.
// sometimes user code wants to replace some of the fields by overwriting
// the string_views.  when doing so, the new data might need to be copied into
// the message.  for that the start_line needs to have access to its message
// container.

class start_line : public std::array<std::string_view, 3>
{
public:
  start_line (void) { }

  start_line (const std::string_view& s0,
	      const std::string_view& s1,
	      const std::string_view& s2)
  : std::array<std::string_view, 3> {{ s0, s1, s2 }}
  {
  }

  bool valid (void) const
  {
    const auto& s = *this;
    return !s[0].empty () && !s[1].empty () && !s[2].empty ();
  }
};

// -----------------------------------------------------------------------------

class request_line : public start_line
{
public:
  request_line (void) { }
  request_line (const std::string_view& method,
		const std::string_view& request_uri,
		const std::string_view& http_version)
  : start_line (method, request_uri, http_version)
  {
  }

  const std::string_view& method (void) const { return start_line::operator[] (0); }
  const std::string_view& request_uri (void) const { return start_line::operator[] (1); }
  const std::string_view& http_version (void) const { return start_line::operator[] (2); }

  // modification of the fields is possible, but keep in mind that string_view
  // is just a weak reference to the string data and something else has to
  // keep a strong reference to it.
  std::string_view& method (void) { return start_line::operator[] (0); }
  std::string_view& request_uri (void) { return start_line::operator[] (1); }
  std::string_view& http_version (void) { return start_line::operator[] (2); }

  // FIXME: use uri class
  std::string_view request_uri_query_str (void) const;
  uri::query_params request_uri_query_params (void) const;
};

// -----------------------------------------------------------------------------

class status_line : public start_line
{
public:
  status_line (void) { }
  status_line (const std::string_view& http_version,
	       const std::string_view& status_code,
	       const std::string_view& reason_phrase)
  : start_line (http_version, status_code, reason_phrase)
  {
  }

  const std::string_view& http_version (void) const { return start_line::operator[] (0); }
  const std::string_view& status_code (void) const { return start_line::operator[] (1); }
  const std::string_view& reason_phrase (void) const { return start_line::operator[] (2); }

  // modification of the fields is possible, but keep in mind that string_view
  // is just a weak reference to the string data and something else has to
  // keep a strong reference to it.
  std::string_view& http_version (void) { return start_line::operator[] (0); }
  std::string_view& status_code (void) { return start_line::operator[] (1); }
  std::string_view& reason_phrase (void) { return start_line::operator[] (2); }
};

// -----------------------------------------------------------------------------

// each header is always followed by a CRLF when sending.
// however, all white spaces and the colon are stripped when accessing the
// key/value.

class header
{
public:
  // FIXME: this interface does not allow constructing header objects with
  // non-compile-time-constant strings that have a longer life span than the
  // caller, because string_view is a weak reference to the actual string.
  header (void) { }
  header (const std::string_view& key,
	  const std::string_view& value)
  : m_key (key), m_value (value)
  {
  }

  const std::string_view& key (void) const { return m_key; }
  const std::string_view& value (void) const { return m_value; }

  // modification of the fields is possible, but keep in mind that string_view
  // is just a weak reference to the string data and something else has to
  // keep a strong reference to it.
  std::string_view& key (void) { return m_key; }
  std::string_view& value (void) { return m_value; }

  bool empty (void) const { return m_key.empty (); }

  explicit operator bool (void) const { return !empty (); }

private:
  std::string_view m_key;
  std::string_view m_value;
};

// -----------------------------------------------------------------------------

class message
{
public:
  using content_buffers_type = std::vector<utils::buffer>;

  message (void);

  // FIXME: copying a request is possible, but is a bit complicated.  after copying
  // the raw data, all string views need to be fixed up.
  message (message&) = delete;

  message (message&& rhs);
  message& operator = (message&& rhs);

  ~message (void);

  bool empty (void) const { return m_header_data.empty (); }

  const std::string& header_data (void) const { return m_header_data; }

  // if the content data has been set as (multiple) buffers, it will be
  // converted to a single string and a single weak buffer.
  const std::string& content_data (void) const;

  // if the content data has been set as a string, this will create a
  // buffer out of it.
  const content_buffers_type& content_buffers (void) const;
  content_buffers_type& content_buffers (void);

  // the start line is usually either the request line or the status line.
  // both consist of 3 fields with 2 SP in between.
  const http::start_line& start_line (void) const { return m_start_line; }

  // FIXME: modifying anything in the start_line (or any other header) should
  // allow copying.  if data is replaced with compile-time string constants,
  // that's OK.  but if the new data is something else, it has to be appended
  // to the message.
  http::start_line& start_line (void) { return m_start_line; }

//  const std::list<http::header>& headers (void) const { return m_headers; }

  // lookup a header by the key string.
  // if not found, returns an empty header.
  const http::header& header (const std::string_view& key) const;

  // input some data for parsing.  the message will copy as many
  // characters it needs and return the pointer to the first unprocessed
  // character.  this can be used to parse successive messages stored in one
  // input buffer.
  const char* parse (const char* in, const char* const in_end);

  bool is_parse_error (void) const { return m_parsing_state >= parsing_error; }
  bool is_parsing_done (void) const { return m_parsing_state >= parsing_done; }

  bool is_parsing_headers_done (void) const { return m_parsing_state >= parsing_content; }

  bool is_valid_request (void) const;
  bool is_valid_response (void) const;

  // copy the specified data and fix-up the string_view references.
  // this does not check if the header already exists.
  void add_header (const http::header& h);

  // notice: must not add a content-length header if a transfer encoding has
  // been applied.
  void set_content (const std::string_view& val);
  void set_content (std::string&& val);
  void set_content (content_buffers_type&& buffers);

protected:
  enum parsing_state
  {
    parsing_init,
    parsing_start_line,
    parsing_headers,

    parsing_content,

    parsing_trailer_headers,

    parsing_done,

    // error values
    parsing_error
  };

  enum parse_line_status
  {
    parse_line_continue,
    parse_line_done,
    parse_line_error
  };

  message (parsing_state p) : m_parsing_state (p) { }

  std::string_view
  append_header_data (const std::string_view& d);

  void reserve_header_data (unsigned int len);
  void header_data_shrink_to_fit (void);

  static std::string_view
  relocate_string_view (const char* prev_base, const char* new_base,
			const std::string_view& sv);

  struct parse_line_result
  {
    const char* in;
    parse_line_status status;
    std::string_view line;
  };

  parse_line_result parse_line (const char* in, const char* const in_end);
  static http::header parse_header (const std::string_view& line);
  static http::start_line parse_start_line (const std::string_view& line);

  parsing_state m_parsing_state;

  // header data as it has been received, or as it is to be transmitted.
  std::string m_header_data;
//  std::vector<char> m_header_data;

  // content data as it has been received, or as it is to be transmitted.
  // it can be either a single contiguous string or as a set of buffers.
  // depending on how the data is accessed, it will be converted into
  // one or the other.
  unsigned int m_expected_content_length;
  static constexpr unsigned int infinite_content_length = std::numeric_limits<decltype (m_expected_content_length)>::max ();

  mutable std::string m_content_str;
  mutable content_buffers_type m_content_buffers;

  // the current line being parsed.  used during start line and
  // header parsing.  the string view references data in m_data.
  //std::string_view m_parsing_cur_line;
  unsigned int m_parsing_cur_line_begin;
  unsigned int m_parsing_cur_line_size;

  // extracted fields that reference the data in m_data.
  http::start_line m_start_line;
  std::list<http::header> m_headers;
};

// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// the request and responses are just overlays on top of the message, which
// reinterpret the start line fields.

class request : public message
{
public:
  request (void) : message () { }
  request (message&& msg) : message (std::move (msg)) { }

  const class request_line& request_line (void) const
  {
    return *(const class request_line*)&m_start_line;
  }

  class request_line& request_line (void)
  {
    return *(class request_line*)&m_start_line;
  }

private:
};

class response : public message
{
public:
  response (void) : message () { }
  response (message&& msg) : message (std::move (msg)) { }

  // construct a response by copying the data.
  // FIXME: add a way to do streaming reads for the content data.
  //        like a callback function or some reader object.
  //        otherwise data from ROM always needs to be copied into the buffer.
  response (const std::string_view& http_version,
	    const std::string_view& status_code,
	    const std::string_view& reason_phrase);

  const class status_line& status_line (void) const
  {
    return *(const class status_line*)&m_start_line;
  }

private:
};

// -----------------------------------------------------------------------------

response
make_303_redirect_response (const request& req,
			    std::string_view new_target_url);

response
make_200_ok_response (const request& req);

template <typename... Args> inline response
make_200_ok_response (const request& req,
		      std::string_view content_type,
		      Args&&... content_args)
{
  response res (req.request_line ().http_version (), "200", "OK");
  res.add_header ({ "Content-Type", content_type });
  res.set_content (std::forward<Args> (content_args)...);

  return std::move (res);

}

response
make_204_no_content_response (const request& req);

response
make_415_unsupported_response (const request& req);

response
make_401_unauthorized_response (const request& req);

} // namespace http
} // namespace net


#endif // includeguard_net_http_includeguard
