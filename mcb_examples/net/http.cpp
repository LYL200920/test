/*
need to do: GET HEAD POST

- in responses copy the Host header contents from the request.

- in requests, header values might be extended over multiple lines.
  if a line (after a line break) starts with a SP, it is a continuation
  of the previous line.
  because we try not to copy data when parsing headers, just extend the string
  view of the previous line.  this requires that all code that reads values
  etc, must skip white spaces.

- incoming requests might cross packet borders.  to simplify request processing
  the received data is copied.


Request-Line   = Method SP       Request-URI SP HTTP-Version CRLF

Status-Line    = HTTP-Version SP Status-Code SP Reason-Phrase CRLF



http://www8.org/w8-papers/5c-protocols/key/key.html
*/


#include <algorithm>
#include <cstring>
#include <cassert>

#include "http.hpp"

//#define log_all
#include <logging/logging.hpp>

#include <utils/text.hpp>

namespace net
{
namespace http
{

static constexpr char SP = ' ';
static constexpr char HT = '\t';
static constexpr char CR = '\r';
static constexpr char LF = '\n';
static constexpr char DEL = 127;

#define CRLF "\r\n"

static constexpr bool is_ctl (char c)
{
  return (c >= 0 && c <= 31) | (c == DEL);
}

static constexpr bool is_hex (char c)
{
  switch (c)
  {
    default:
      return false;

    case '0': case '1': case '2': case '3': case '4': case '5': case '6':
    case '7': case '8': case '9':
    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
      return true;
  }
}

static constexpr bool is_alpha (char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

static constexpr char to_lower (char c)
{
  return (c >= 'A' && c <= 'Z') ? (c - 'A' + 'a') : c;
}

static constexpr bool is_digit (char c)
{
  return c >= '0' && c <= '9';
}

static constexpr bool is_space (char c)
{
  switch (c)
  {
    default:
      return false;
    case 0x20: case 0x0c: case 0x0a: case 0x0d: case 0x09: case 0x0b:
      return true;
  }
}

template <typename Iterator>
static std::string_view trim_str (Iterator begin, Iterator end)
{
  return utils::trim_string (begin, end, &is_space);
}

static bool lowercase_strcmp (const std::string_view& a,
			      const std::string_view& b)
{
  /*
    0x00...0x40: no case	0000'0000 - 0100'0000
    0x41...0x5A: upper case	0100'0001 - 0101'1010
    0x5B...0x60: no case	0101'1011 - 0110'0000
    0x61...0x7A: lower case	0110'0001 - 0111'1010
    0x7B...0x7F: no case	0111'1011 - 0111'1111

    0xDF = 1101'1111  0x20 = 0010'0000

    char | 0x20 gives the lower case char if it was an upper case char.
    if it wasn't an upper case char, this will give wrong results, e.g.:
    (0x00 | 0x20) == (0x20 | 0x20)      // NG

    thus we can't use a simplified form of case insensitive comparison like
       (a | 0x20) == (b | 0x20)

    the only thing we could do here to get more speed is using a lookup table.
  */

  if (a.size () != b.size ())
    return false;

  for (unsigned int i = 0; i < a.size (); ++i)
  {
    if (a[i] == b[i])
      continue;

    if (to_lower (a[i]) == to_lower (b[i]))
      continue;

    return false;
  }

  return true;
}

template <typename T> T parse_decimal (const std::string_view& str, T default_val);

template <> unsigned int
parse_decimal (const std::string_view& str, unsigned int default_val)
{
  auto digits_begin = str.begin ();
  auto digits_end = std::find_if_not (digits_begin, str.end (), is_digit);

  if (digits_begin == digits_end)
    return default_val;

  unsigned int val = 0;
  for (auto d = digits_begin; d != digits_end; ++d)
  {
    auto val_1 = (val * 10) + (*d - '0');

    // for unsigned integers, we can detect overflow like that, because it
    // it is defined as wrap-around.
    if (val_1 < val)
      return default_val;

    val = val_1;
  }

  return val;
}

// -----------------------------------------------------------------------------

std::string_view request_line::request_uri_query_str (void) const
{
  auto i0 = request_uri ().find ('?');
  if (i0 == std::string_view::npos)
    return { };

  return request_uri ().substr (i0 + 1, request_uri ().find ('#', i0 + 1));
}

uri::query_params request_line::request_uri_query_params (void) const
{
  return { request_uri_query_str () };
}

// -----------------------------------------------------------------------------


static const http::header empty_header;

// -----------------------------------------------------------------------------

message::message (void)
: m_parsing_state (parsing_init),
  m_expected_content_length (0),
  m_parsing_cur_line_begin (0),
  m_parsing_cur_line_size (0)
{
}

message::message (message&& rhs)
: m_parsing_state (std::move (rhs.m_parsing_state)),
  m_header_data (std::move (rhs.m_header_data)),
  m_content_str (std::move (rhs.m_content_str)),
  m_content_buffers (std::move (rhs.m_content_buffers)),
  m_parsing_cur_line_begin (std::move (rhs.m_parsing_cur_line_begin)),
  m_parsing_cur_line_size (std::move (rhs.m_parsing_cur_line_size)),
  m_start_line (std::move (rhs.m_start_line)),
  m_headers (std::move (rhs.m_headers))
{
}

message::~message (void)
{
}

message& message::operator = (message&& rhs)
{
  m_parsing_state = std::move (rhs.m_parsing_state);
  m_header_data = std::move (rhs.m_header_data);
  m_content_str = std::move (rhs.m_content_str);
  m_content_buffers = std::move (rhs.m_content_buffers);
  m_parsing_cur_line_begin = std::move (rhs.m_parsing_cur_line_begin);
  m_parsing_cur_line_size = std::move (rhs.m_parsing_cur_line_size);
  m_start_line = std::move (rhs.m_start_line);
  m_headers = std::move (rhs.m_headers);
  return *this;
}


bool message::is_valid_request (void) const
{
  return !m_start_line[0].empty () && !m_start_line[1].empty ()
	 && !m_start_line[2].empty ()
	 && utils::starts_with (m_start_line[2], "HTTP/");
}

bool message::is_valid_response (void) const
{
  return utils::starts_with (m_start_line[0], "HTTP/")
	 && !m_start_line[1].empty ()
	 && !m_start_line[2].empty ();
}

const char* message::parse (const char* in, const char* const in_end)
{
  log_info ("http message parse %p -> %p (%u)\n", in, in_end, (unsigned int)(in_end - in));

  // the incoming data might include a full message, multiple messages
  // or some bytes of a message.  because of that, we need to copy the data
  // into an internal buffer.  when doing so, try to minimize allocations
  // on the free store by copying data into a larger buffer.  the message
  // pieces (start line, headers, content) are then stored as string views
  // (weak references).

  for (; in != in_end && !is_parsing_done (); )
  {
    if (m_parsing_state == parsing_init)
    {
      // we are waiting for the start of a new message, which starts with
      // a start line, which always starts with an alpha character.
      // skip everything else until then.  this will also skip leading CRLFs.
      // notice that this also implies that there can't be empty lines during
      // status line parsing.
      in = std::find_if (in, in_end, is_alpha);
      if (in != in_end)
      {
	log_info ("found line start\n");

	m_parsing_state = parsing_start_line;
	m_parsing_cur_line_begin = 0;
	m_parsing_cur_line_size = 0;

	m_header_data.clear ();
	m_header_data.reserve (256);
	m_content_str.clear ();
	m_content_buffers = { };
      }
    }

    if (m_parsing_state == parsing_start_line)
    {
      auto r = parse_line (in, in_end);
      in = r.in;

      if (r.status == parse_line_done)
      {
	log_info ("got line (s): %s\n", r.line.to_string ().c_str ());

	auto s = parse_start_line (r.line);
	if (!s.valid ())
	{
	  log_info ("http message parse start line error\n");
	  m_parsing_state = parsing_error;
	  break;
	}
	else
	{
	  // status line seems to be valid.
	  // well, we don't know for sure, just all 3 required fields are there.
	  // FIXME: should do an early validation of the start line to reject
	  // junk as early as possible.

	  // parse_line copies the input data into m_header_data.
	  // we assume that the received input data is correct at this point.
	  // if not, we should have bailed out earlier.
	  // thus we can create string_view references directly here.
	  m_start_line = s;

	  m_parsing_state = parsing_headers;
	}
      }
      else if (r.status == parse_line_error)
      {
	log_info ("http message parse line error (1)\n");
	m_parsing_state = parsing_error;
	break;
      }
    }

    if (m_parsing_state == parsing_headers)
    {
      auto r = parse_line (in, in_end);
      in = r.in;

      if (r.status == parse_line_done)
      {
	log_info ("got line (h): %s\n", r.line.to_string ().c_str ());

	// each line that we accept ends with a CRLF.  thus the size of an
	// empty line is 2.  an empty line is used as a separator between
	// message header and body/content.
	if (r.line == CRLF)
	{
	  log_info ("http message parsing header done ...\n");

	  log_info ("header data:\n---------------\n%s\n---------------\n",
		    m_header_data.c_str ());

	  log_info ("start line: %s %s %s\n", m_start_line[0].to_string ().c_str (),
					      m_start_line[1].to_string ().c_str (),
					      m_start_line[2].to_string ().c_str ());
	  do_if_log_info (
	    for (auto&& h : m_headers)
	      log_info ("header: %s: %s\n", h.key ().to_string ().c_str (),
					    h.value ().to_string ().c_str ());
	    );

	  m_parsing_state = parsing_content;

	  if (auto h = header ("content-length"))
	  {
	    log_info ("got content length header: %s: %s\n",
		      h.key ().to_string ().c_str (),
		      h.value ().to_string ().c_str ());

	    m_expected_content_length =
		parse_decimal<decltype (m_expected_content_length)> (
			header ("content-length").value (), 0);
	  }
	  else
	    m_expected_content_length = 0;

	  log_info ("expected content length: %u\n", m_expected_content_length);

	  header_data_shrink_to_fit ();

	  if (m_expected_content_length != infinite_content_length)
	  {
	    m_content_str.reserve (m_expected_content_length);
	    m_content_buffers = { };
	  }
	  m_parsing_state = parsing_content;
	}
	else
	{
	  auto h = parse_header (r.line);

	  if (h.empty ())
	    log_info ("http message empty header\n");

	  // parse_line copies the input data into m_header_data.
	  // assuming that the input data is correct, create string_view
	  // refs directly.
	  m_headers.emplace_back (h);
	}
      }
      else if (r.status == parse_line_error)
      {
	log_info ("http message parse line error (2)\n");
	m_parsing_state = parsing_error;
	break;
      }
    }

    if (m_parsing_state == parsing_content)
    {
      // m_parsing_state = parsing_trailer_headers
      log_info ("parsing content remaining length: %u\n", m_expected_content_length);
      if (m_expected_content_length == 0)
	m_parsing_state = parsing_trailer_headers;
      else
      {
	const unsigned int in_available = in_end - in;
	auto count = std::min (in_available, m_expected_content_length);

	log_info ("copying %u bytes content\n", count);

	m_content_str.insert (m_content_str.end (), in, in + count);
	in = in + count;
	m_expected_content_length -= count;

	if (m_expected_content_length == 0)
	  m_parsing_state = parsing_trailer_headers;
      }
    }

    if (m_parsing_state == parsing_trailer_headers)
    {
      // FIXME: implement trailing headers.
      m_parsing_state = parsing_done;

      log_info ("http message parsing done\n");
    }
  }

  return in;
}

// FIXME: the string_view relocation would not be required if we stored
// indices/offsets into the header data.
// however, this makes iterating over headers more complicated, because
// when user code wants to access the header contents, the offset has to
// be applied.  this means we need a custom header iterator and have to use
// a different header class for storage in the message.  the iterator would then
// convert the internal stored offset format into absolute string_view format
// when the iterator is dereferenced.
std::string_view
message::append_header_data (const std::string_view& d)
{
  if (m_header_data.empty ())
  {
    m_header_data.insert (m_header_data.end (), d.begin (), d.end ());
    return { m_header_data.data (), d.size () };
  }

  // data is not empty and thus we need to fix up any string views that
  // are referencing the header data, if the header data reallocates.
  auto* prev_data = m_header_data.data ();

  m_header_data.insert (m_header_data.end (), d.begin (), d.end ());

  auto* new_data = m_header_data.data ();

  if (prev_data != new_data)
  {
    log_info ("http message append header data changed %p -> %p\n", prev_data, new_data);

    m_start_line[0] = relocate_string_view (prev_data, new_data, m_start_line[0]);
    m_start_line[1] = relocate_string_view (prev_data, new_data, m_start_line[1]);
    m_start_line[2] = relocate_string_view (prev_data, new_data, m_start_line[2]);
    for (auto& h : m_headers)
      h = std::move (http::header (relocate_string_view (prev_data, new_data, h.key ()),
				   relocate_string_view (prev_data, new_data, h.value ())));
  }

  return { (m_header_data.data () + m_header_data.size ()) - d.size (), d.size () };
}

void message::reserve_header_data (unsigned int len)
{
  if (m_header_data.empty ())
  {
    m_header_data.reserve (len);
    return;
  }

  // data is not empty and thus we need to fix up any string views that
  // are referencing the header data, if the header data reallocates.
  auto* prev_data = m_header_data.data ();

  m_header_data.reserve (m_header_data.size () + len);

  auto* new_data = m_header_data.data ();

  if (prev_data != new_data)
  {
    log_info ("http message reserve header data changed %p -> %p\n", prev_data, new_data);
    log_info ("                                                                                               \n");

    m_start_line[0] = relocate_string_view (prev_data, new_data, m_start_line[0]);
    m_start_line[1] = relocate_string_view (prev_data, new_data, m_start_line[1]);
    m_start_line[2] = relocate_string_view (prev_data, new_data, m_start_line[2]);
    for (auto& h : m_headers)
      h = std::move (http::header (relocate_string_view (prev_data, new_data, h.key ()),
				   relocate_string_view (prev_data, new_data, h.value ())));
  }
}

void message::header_data_shrink_to_fit (void)
{
  if (m_header_data.empty ())
  {
    m_header_data.shrink_to_fit ();
    return;
  }

  auto* prev_data = m_header_data.data ();

  m_header_data.shrink_to_fit ();

  auto* new_data = m_header_data.data ();

  if (prev_data != new_data)
  {
    log_info ("http message reserve header data changed %p -> %p\n", prev_data, new_data);
    log_info ("                                                                                               \n");

    m_start_line[0] = relocate_string_view (prev_data, new_data, m_start_line[0]);
    m_start_line[1] = relocate_string_view (prev_data, new_data, m_start_line[1]);
    m_start_line[2] = relocate_string_view (prev_data, new_data, m_start_line[2]);
    for (auto& h : m_headers)
      h = std::move (http::header (relocate_string_view (prev_data, new_data, h.key ()),
				   relocate_string_view (prev_data, new_data, h.value ())));
  }
}


std::string_view
message::relocate_string_view (const char* prev_base, const char* new_base,
			       const std::string_view& sv)
{
  if (sv.empty ())
    return { };

  return { new_base + (sv.data () - prev_base), sv.size () };
}

message::parse_line_result
message::parse_line (const char* in, const char* const in_end)
{
  if (in == in_end)
  {
    log_warn ("http message parse_line empty input\n");
    return { in, parse_line_continue, { } };
  }

  // if the current line ends with an CR we expect an LF.
  if (m_parsing_cur_line_size > 0
      && m_header_data[m_parsing_cur_line_begin + m_parsing_cur_line_size - 1] == CR)
  {
    if (*in == LF)
    {
      char lf_str = LF;

      append_header_data ({ &lf_str, 1 });
      auto l = std::string_view (&m_header_data[m_parsing_cur_line_begin],
				 m_parsing_cur_line_size + 1);

      m_parsing_cur_line_begin += m_parsing_cur_line_size + 1;
      m_parsing_cur_line_size = 0;
      return { in + 1, parse_line_done, l };
    }
    else
      return { in, parse_line_error, { } };
  }

  // try to find the next CR in the input
  auto&& in1 = std::find (in, in_end, CR);

  bool line_done = false;

  // if the CR is followed by the LF (usual case), take it.
  if (in1 != in_end)
    ++in1;

  if (in1 != in_end && *in1 == LF)
  {
    ++in1;
    line_done = true;
  }

  unsigned int copy_count = in1 - in;

  if (copy_count > 0)
  {
    log_info ("parse_line copy count = %u\n", copy_count);

    append_header_data ({ in, copy_count });
    m_parsing_cur_line_size += copy_count;
    in += copy_count;
  }

  if (line_done)
  {
    std::string_view l (&m_header_data[m_parsing_cur_line_begin],
				      m_parsing_cur_line_size);
    m_parsing_cur_line_begin += m_parsing_cur_line_size;
    m_parsing_cur_line_size = 0;

    return { in, parse_line_done, l };
  }
  else
    return { in, parse_line_continue, { } };
}

http::header
message::parse_header (const std::string_view& line)
{
  // assume that the input line ends with a CRLF.

  auto key_begin = line.begin ();
  auto key_end = std::find (key_begin, line.end (), ':');

  if (key_end == line.end ())
  {
    // a header without a value ... maybe possible, although not standard.
    return { trim_str (key_begin, key_end), "" };
  }

  auto val_begin = key_end + 1;
  auto val_end = line.end ();

  return { trim_str (key_begin, key_end),
	   trim_str (val_begin, val_end) };
}

http::start_line
message::parse_start_line (const std::string_view& line)
{
  // assume that the input line ends with a CRLF.

  auto s0_begin = line.begin ();
  auto s0_end = std::find (s0_begin, line.end (), SP);

  if (s0_end == line.end())
    return { trim_str (s0_begin, s0_end), { }, { } };

  auto s1_begin = s0_end + 1;
  auto s1_end = std::find (s1_begin, line.end (), SP);

  if (s1_end == line.end())
    return { trim_str (s0_begin, s0_end),
	     trim_str (s1_begin, s1_end), { } };

  auto s2_begin = s1_end + 1;
  auto s2_end = std::find (s2_begin, line.end (), SP);

  return { trim_str (s0_begin, s0_end),
	   trim_str (s1_begin, s1_end),
	   trim_str( s2_begin, s2_end) };
}


const http::header&
message::header (const std::string_view& key) const
{
  // FIXME: convert input key to lower case only once.

  // header names (keys) are case insensitive.
  for (auto&& h : m_headers)
    if (lowercase_strcmp (key, h.key ()))
      return h;

  return empty_header;
}

void message::add_header (const http::header& new_h)
{
  // the header data can't be empty because every message has a start line.
  assert (!m_header_data.empty ());

  // make sure that the header data is always terminated by an additional
  // CRLF.  this makes it easier to write it out when sending.
  // because we always do so, we also must remove the already inserted
  // terminating CRLF when adding new headers.
  m_header_data.erase (m_header_data.end () - 2, m_header_data.end ());

  reserve_header_data (new_h.key ().size () + std::strlen (": ")
		       + new_h.value ().size () + std::strlen (CRLF)
		       + std::strlen (CRLF));

  auto k = append_header_data (new_h.key ());
  append_header_data (": ");
  auto v = append_header_data (new_h.value ());
  append_header_data (CRLF CRLF);

  m_headers.emplace_back (k, v);
}

const std::string& message::content_data (void) const
{
  if (m_content_str.empty () && !m_content_buffers.empty ())
  {
    auto sz = utils::sum_buffer_sizes (m_content_buffers);
    m_content_str.reserve (sz);

    for (auto&& b : m_content_buffers)
    {
      // if string value type is 2 bytes, but buffer data has 3 bytes
      // this won't work.  so string value type must be 1 byte size.
      static_assert (sizeof (std::string::value_type) == 1, "");
      const char* data = (const char*)b.data ();
      m_content_str.insert (m_content_str.end (), data, data + b.size ());
    }

    m_content_buffers = { utils::buffer (utils::buffer::weak, m_content_str) };
  }

  return m_content_str;
}

message::content_buffers_type& message::content_buffers (void)
{
  if (!m_content_str.empty ())
    m_content_buffers = { utils::buffer (utils::buffer::weak, m_content_str) };

  return m_content_buffers;
}

const message::content_buffers_type& message::content_buffers (void) const
{
  return const_cast<message*> (this)->content_buffers ();
}

void message::set_content (const std::string_view& val)
{
  m_content_str = std::string (val);
  m_content_buffers = { utils::buffer (utils::buffer::weak, m_content_str) };
}

void message::set_content (std::string&& val)
{
  m_content_str = std::move (val);
  m_content_buffers = { utils::buffer (utils::buffer::weak, m_content_str) };
}

void message::set_content (content_buffers_type&& buffers)
{
  m_content_buffers = std::move (buffers);
  m_content_str = { };
}

// -----------------------------------------------------------------------------

response::response (const std::string_view& http_version,
		    const std::string_view& status_code,
		    const std::string_view& reason_phrase)
: message (parsing_done)
{
  // always append a CRLF to the header data.

  reserve_header_data (http_version.size () + std::strlen (" ")
		       + status_code.size () + std::strlen (" ")
		       + reason_phrase.size () + std::strlen (CRLF)
		       + std::strlen (CRLF));

  m_start_line[0] = append_header_data (http_version);
  append_header_data (" ");
  m_start_line[1] = append_header_data (status_code);
  append_header_data (" ");
  m_start_line[2] = append_header_data (reason_phrase);
  append_header_data (CRLF CRLF);
}

// -----------------------------------------------------------------------------

response
make_303_redirect_response (const request& req,
			    std::string_view new_target_url)
{
  // FIXME: 303 is supported only with HTTP 1.1.  For HTTP 1.0 have to use "302 Found".
  response res (req.request_line ().http_version (), "303", "See Other");
  res.add_header ({ "Location", new_target_url });
  return res;
}

response
make_200_ok_response (const request& req)
{
  return response (req.request_line ().http_version (), "200", "OK");
}

response
make_204_no_content_response (const request& req)
{
  return response (req.request_line ().http_version (), "204", "No Content");
}

response
make_415_unsupported_response (const request& req)
{
  return response (req.request_line ().http_version (), "415", "Unsupported Media Type");
}

response
make_401_unauthorized_response (const request& req)
{
  return response (req.request_line ().http_version (), "401", "Unauthorized");
}


} // namespace http
} // namespace net
