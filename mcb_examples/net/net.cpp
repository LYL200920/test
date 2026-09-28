
#include "net.hpp"

namespace net
{

std::optional<std::array<uint8_t, 4>>
parse_ipv4_addr (std::string_view str)
{
  std::array<uint8_t, 4> val = { 0, 0, 0, 0 };

  int x[4] = { -1, -1, -1, -1 };

  // FIXME: this also allows junk like +1.+1.+1.0
  // make a better parser after adding net::ipv4_address and use it via
  // net::ipv4_address::from_string (...);
  std::sscanf (std::string (str).c_str (),
	       "%d.%d.%d.%d",
	       &x[0], &x[1], &x[2], &x[3]);

  for (unsigned int j = 0; j < 4; ++j)
    if (x[j] < 0 || x[j] > 255)
      return { };
    else
      val[j] = (uint8_t)x[j];

  return { val };
}


}
