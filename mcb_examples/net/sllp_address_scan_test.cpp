
#include <array>
#include <cstdint>
#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <cassert>
#include <algorithm>

#include "sllp.hpp"


#if 0
using addr_t = std::array<uint8_t, 8>;

static const addr_t my_addresses[] =
{
  { 0x32,0x46,0x30,0x57,0x00,0x42,0x00,0x16 },
  { 0x32,0x57,0x30,0x57,0x00,0x42,0x00,0x16 },
  { 0x32,0x57,0x30,0x57,0x00,0x42,0x00,0x17 },
  { 0x32,0x57,0x30,0x57,0x00,0x42,0x10,0x16 }
};
#endif


#if 0
using addr_t = std::array<uint8_t, 6>;

static const addr_t my_addresses[] =
{
  { 0x01,0x02,0x03,0x04,0x05,0x06 },
  { 0x32,0x57,0x30,0x57,0x00,0x42 },
  { 0x32,0x57,0x30,0x57,0x00,0x43 },
  { 0xFF,0x57,0x30,0x57,0x01,0x43 },
  { 0xFE,0x57,0x30,0x57,0x01,0x43 },
  { 0xFA,0x01,0x30,0x57,0x01,0x43 },
  { 0x00,0x55,0x30,0x57,0x01,0x43 },
  { 0x01,0x55,0x30,0x57,0x01,0x43 },
  { 0x02,0x45,0x30,0x57,0x01,0x43 },
  { 0x03,0x35,0x30,0x57,0x01,0x43 },
  { 0x04,0x75,0x30,0x57,0x01,0x43 },

  { 0x32,0x57,0x40,0x57,0x00,0x43 },
  { 0xFF,0x57,0x33,0x57,0x01,0x43 },
  { 0xFE,0x57,0x60,0x57,0x01,0x43 },
  { 0xFA,0x01,0x90,0x57,0x01,0x43 },
  { 0x00,0x55,0x09,0x57,0x01,0x43 },
  { 0x01,0x55,0x30,0xA7,0x01,0x43 },
  { 0x02,0x45,0x30,0xB7,0x01,0x43 },
};
#endif


#if 0
using addr_t = std::array<uint8_t, 6>;

static const addr_t my_addresses[] =
{
  { 0x01,0x02,0x03,0x04,0x05,0x06 },
  { 0x32,0x57,0x30,0x57,0x00,0x42 },
  { 0x32,0x57,0x30,0x57,0x00,0x43 },
  { 0xFF,0x57,0x30,0x57,0x01,0x43 }
};

#endif


#if 0
using addr_t = std::array<uint8_t, 6>;

static const addr_t my_addresses[] =
{
  { 0x02,0x60,0x95,0x27,0x07,0x80 },
  { 0x02,0x60,0x96,0x27,0x07,0x81 },
  { 0x02,0x60,0x99,0x27,0x07,0x82 },
  { 0x02,0x60,0x95,0x37,0x07,0x83 }
};
#endif


#if 0
using addr_t = std::array<uint8_t, 2>;

static const addr_t my_addresses[] =
{
  { 0, 1 },
  { 0, 2 },
  { 0, 3 },
  { 0, 4 }
};
#endif


#if 0
using addr_t = std::array<uint8_t, 2>;

static const addr_t my_addresses[] =
{
  { 1, 0 },
  { 2, 0 },
  { 3, 0 },
  { 4, 0 }
};
#endif


#if 1

//using addr_t = std::array<uint8_t, 16>;
using addr_t = net::sllp::devid_addr_n<16>;

static const addr_t my_addresses[] =
{
/*
  { 0x46,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x30,0x38,0x39,0x20,0xD8,0x06 },
  { 0x46,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x30,0x38,0x39,0x20,0xD8,0x07 },
  { 0x46,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x30,0x38,0x39,0x20,0xD8,0x08 },
  { 0x46,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x30,0x38,0x39,0x21,0xD8,0x06 },
*/
/*
  { 0x46,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x3F,0x38,0x39,0x21,0xD8,0x06 },
  { 0x56,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x06 },
  { 0xF6,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x3F,0x38,0x39,0x21,0xD8,0x06 },
  { 0x56,0x30,0xC7,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x06 },
  { 0x46,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x3F,0x38,0x39,0x21,0xD8,0x16 },
  { 0x56,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x07 },
  { 0xF6,0x30,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x3F,0x38,0x39,0x21,0xD8,0x08 },
  { 0x56,0x30,0xC7,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x09 },
*/

  { 0x4E,0x4C,0x44,0x41,0x57,0x00,0x42,0x00,0x16,0x57,0x34,0x53,0x30,0x38,0x39,0x20 }
};
#endif


#if 0
using addr_t = std::array<uint8_t, 12>;

static const addr_t my_addresses[] =
{
  { 0x42,0x00,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x06 },
  { 0x42,0x01,0x16,0x57,0x34,0x53,0x3F,0x38,0x39,0x21,0xD8,0x06 },
  { 0x42,0x02,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x06 },
  { 0x42,0x03,0x16,0x57,0x34,0x53,0x3F,0x38,0x39,0x21,0xD8,0x16 },
  { 0x42,0x04,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x07 },
  { 0x42,0x05,0x16,0x57,0x34,0x53,0x3F,0x38,0x39,0x21,0xD8,0x08 },
  { 0x42,0x06,0x16,0x57,0x34,0x53,0xB0,0x38,0x39,0x21,0xD8,0x09 },

};
#endif



#define log_ops 1






std::string to_string (const addr_t& val)
{
  static const char hex_digit[16+1] = "0123456789ABCDEF";

  std::string r;
  r.resize (val.size () * 2);
  for (unsigned int i = 0; i < val.size (); ++i)
  {
    r[i*2+0] = hex_digit[val[i] >> 4];
    r[i*2+1] = hex_digit[val[i] & 0x0F];
  }

  return r;
}

bool less_than (const addr_t& a, const addr_t& b)
{
  bool r = a < b;

#if log_ops >= 2
  std::cout << "less_than (" << to_string (a) << ", " << to_string (b) << ") = " << r << std::endl;
#endif

  return r;
}

bool is_zero (const addr_t& val)
{
  return val.is_zero ();
}

bool equal (const addr_t& a, const addr_t& b)
{
  bool r = a == b;

#if log_ops >= 2
  std::cout << "equal (" << to_string (a) << ", " << to_string (b) << ") = " << r << std::endl;
#endif

  return r;
}

addr_t shr_1 (const addr_t& val)
{
  addr_t r = val.shr_1 ();
#if log_ops >= 2
  std::cout << "shr_1 (" << to_string (val) << ") = " << to_string (r) << std::endl;
#endif
  return r;
}

addr_t sub (const addr_t& a, const addr_t& b)
{
  addr_t r = a - b;
#if log_ops >= 2
  std::cout << "sub (" << to_string (a) << ", " << to_string (b) << ") = " << to_string (r) << std::endl;
#endif
  return r;
}

addr_t add (const addr_t& a, const addr_t& b)
{
  addr_t r = a + b;

#if log_ops >= 2
  std::cout << "add (" << to_string (a) << ", " << to_string (b) << ") = " << to_string (r) << std::endl;
#endif

  return r;
}

addr_t inc (const addr_t& a)
{
  addr_t r = a;
  ++r;

#if log_ops >= 2
  std::cout << "inc (" << to_string (a) << ") = " << to_string (r) << std::endl;
#endif
  return r;
}


unsigned int query_address_count;
unsigned int query_address_ok_count;

bool query_address_ (const addr_t& a)
{
  for (auto&& aa : my_addresses)
    if (equal (aa, a))
      return true;

  return false;
}

bool query_address (const addr_t& a)
{
  bool r = query_address_ (a);

#if log_ops >= 1
  std::cout << "query_address (" << to_string (a) << ") = " << r << std::endl;
#endif

  ++query_address_count;
  query_address_ok_count += r;
  return r;
}

bool in_range (const addr_t& a, const addr_t& lower_bound, const addr_t& upper_bound)
{
//  return less_than (a, upper_bound) && !less_than (a, lower_bound);
  return (a < upper_bound) && !(a < lower_bound);
}


unsigned int query_address_range_count;
unsigned int query_address_range_ok_response_count;

bool query_address_range_ (const addr_t& lower_bound, const addr_t& upper_bound)
{
  for (auto&& aa : my_addresses)
    if (in_range (aa, lower_bound, upper_bound))
      return true;

  return false;
}

bool query_address_range (const addr_t& lower_bound, const addr_t& upper_bound)
{
  bool r = query_address_range_ (lower_bound, upper_bound);
#if log_ops >=1
  std::cout << "query_address_range (" << to_string (lower_bound) << ", " << to_string (upper_bound) << ") = " << r << std::endl;
#endif

  ++query_address_range_count;
  query_address_range_ok_response_count += r;

  return r;
}

unsigned int scan_address_range_depth_count;
unsigned int scan_address_range_depth_count_max;


// ---------------------------------------------------------------------------
// simple recursive version

#if 0
using scan_addr_range_param = void;

void
scan_address_range (const addr_t& lower_bound,
		    const addr_t& upper_bound,
		    std::vector<addr_t>& found)
{
  ++scan_address_range_depth_count;

  scan_address_range_depth_count_max = std::max (scan_address_range_depth_count, scan_address_range_depth_count_max);


#ifdef log_ops
  std::cout << " ------ scan_address_range (" << to_string (lower_bound) << ", " << to_string (upper_bound) << ")" << std::endl;
#endif

  // if the range contains only 16 addresses, scan each address individually
  addr_t d = sub (upper_bound, lower_bound);

  constexpr unsigned int max_addr_comparisions = 4;

  if (less_than (d, make_const (max_addr_comparisions+1)))
  {
    addr_t a = lower_bound;
    for (unsigned int i = 0; i < max_addr_comparisions; ++i)
    {
      if (std::find (found.begin (), found.end (), a) == found.end ())
	if (query_address (a))
	  found.push_back (a);

      a = inc (a);
    }
  }

  else if (query_address_range (lower_bound, upper_bound))
  {
    // recurse into lower half and upper half
    addr_t midpoint = add (lower_bound, shr_1 (d));

    scan_address_range (lower_bound, midpoint, found);
    scan_address_range (midpoint, upper_bound, found);
  }


  --scan_address_range_depth_count;
}
#endif

// ---------------------------------------------------------------------------
// iterative version 1

#if 0
struct scan_addr_range_param
{
  addr_t lower_bound;
  addr_t upper_bound;

  scan_addr_range_param (const addr_t& l, const addr_t& u) : lower_bound (l), upper_bound (u) { }
};

void
scan_address_range (std::vector<scan_addr_range_param>& param,
		    std::vector<addr_t>& found)
{

  constexpr unsigned int max_addr_comparisions = 2;

  while (!param.empty ())
  {
    scan_address_range_depth_count_max = std::max ((unsigned int)param.size (), scan_address_range_depth_count_max);

    addr_t lower_bound = param.back ().lower_bound;
    addr_t upper_bound = param.back ().upper_bound;

    #ifdef log_ops
    std::cout << " ------ scan_address_range (" << to_string (lower_bound) << ", " << to_string (upper_bound) << ")" << std::endl;
    #endif

    param.pop_back ();

    addr_t d = sub (upper_bound, lower_bound);

    if (less_than (d, make_const (max_addr_comparisions+1)))
    {
      addr_t a = lower_bound;
      for (unsigned int i = 0; i < max_addr_comparisions; ++i)
      {
	if (std::find (found.begin (), found.end (), a) == found.end ())
	  if (query_address (a))
	    found.push_back (a);

	a = inc (a);
      }
    }
    else if (query_address_range (lower_bound, upper_bound))
    {
      addr_t midpoint = add (lower_bound, shr_1 (d));

      param.emplace_back (lower_bound, midpoint);
      param.emplace_back (midpoint, upper_bound);
    }
  }
}

void
scan_address_range (const addr_t& min_val, const addr_t& max_val, std::vector<addr_t>& found)
{
  std::vector<scan_addr_range_param> p;
  p.emplace_back (min_val, max_val);
  scan_address_range (p, found);
}

#endif

// ---------------------------------------------------------------------------
// iterative version 2
// reduces the number of RAM required by the stack by re-using the midpoint value.

#if 1

using scan_addr_range_param = addr_t;

void
scan_address_range (std::vector<scan_addr_range_param>& param,
		    std::vector<addr_t>& found)
{
  constexpr unsigned int max_addr_comparisions = 2;

  while (param.size () >= 2)
  {
    scan_address_range_depth_count_max = std::max ((unsigned int)param.size (), scan_address_range_depth_count_max);

    addr_t upper_bound = param.back ();
    param.pop_back ();

    const addr_t& lower_bound = param.back ();

    #ifdef log_ops
    std::cout << " ------ scan_address_range (" << to_string (lower_bound) << ", " << to_string (upper_bound) << ")" << std::endl;
    #endif

    addr_t d = upper_bound - lower_bound;

    if (d < addr_t (max_addr_comparisions+1))
    {
      addr_t a = lower_bound;
      for (unsigned int i = 0; i < max_addr_comparisions; ++i)
      {
	if (std::find (found.begin (), found.end (), a) == found.end ())
	  if (query_address (a))
	    found.push_back (a);

	++a;
      }
    }
    else if (query_address_range (lower_bound, upper_bound))
    {
      addr_t midpoint = lower_bound + (d >> 1);

      param.emplace_back (midpoint);
      param.emplace_back (upper_bound);
    }
  }
}


void
scan_address_range (const addr_t& min_val, const addr_t& max_val, std::vector<addr_t>& found)
{
  std::vector<scan_addr_range_param> p;
  p.emplace_back (min_val);
  p.emplace_back (max_val);
  scan_address_range (p, found);
}
#endif


int main (void)
{
  std::vector<addr_t> found;

  query_address_count = 0;
  query_address_ok_count = 0;
  query_address_range_count = 0;
  query_address_range_ok_response_count = 0;

  scan_address_range (addr_t::zero_value (), addr_t::max_value (), found);

  std::cout << "found addresses = " << found.size () << std::endl;
  std::cout << "query_address_count = " << query_address_count
	    << "  query_address_range_count = " << query_address_range_count << std::endl;

  for (auto&& i : found)
    std::cout << to_string (i) << std::endl;


  unsigned int query_address_range_ng_response_count = query_address_range_count - query_address_range_ok_response_count;
  unsigned int query_address_ng_count = query_address_count - query_address_ok_count;

  uint64_t addr_query_range_msg_size_bytes = addr_t ().size () * 2 + 2 + 2 + 2;
  uint64_t addr_query_range_ok_resp_size_bytes = 2+2+2;
  uint64_t addr_query_range_ng_resp_size_bytes = 16;  // assume short timeout value of 32 bytes
  uint64_t addr_query_msg_size_wire_bytes = addr_t ().size () + 2 + 2 + 2 ;
  uint64_t addr_query_msg_ok_size_bytes = 2+2+2;
  uint64_t addr_query_msg_ng_size_bytes = 16;  // assume short timeout value of 32 bytes

  unsigned int total_bytes_on_wire = 0
	+ addr_query_range_msg_size_bytes * query_address_count
	+ addr_query_range_ok_resp_size_bytes * query_address_range_ok_response_count
	+ addr_query_range_ng_resp_size_bytes * query_address_range_ng_response_count
	+ addr_query_msg_size_wire_bytes * query_address_count
	+ addr_query_msg_ok_size_bytes * query_address_ok_count
	+ addr_query_msg_ng_size_bytes * query_address_ng_count
	+ 0;

  const unsigned int wire_speed_bps = 250000;
  std::cout << "total bytes on wire = " << total_bytes_on_wire
	    << " = " << (total_bytes_on_wire*10) << " bits"
	    << " @ " << wire_speed_bps << " bps = " << ((total_bytes_on_wire*10)/(double)wire_speed_bps) << " sec" << std::endl;

  std::cout << "scan_address_range_depth_count_max = " << scan_address_range_depth_count_max << std::endl;
  std::cout << "required state memory = "
	    << (scan_address_range_depth_count_max * sizeof (scan_addr_range_param))
	    << " bytes" << std::endl;

  return 0;
}

