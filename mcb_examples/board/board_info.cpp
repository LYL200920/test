
#include "board_info.hpp"

bool board_info::is_blank (void) const
{
  const uint32_t* p = (const uint32_t*)this;

  for (unsigned int i = 0; i < sizeof (board_info) / sizeof (uint32_t); ++i)
    if (*p++ != 0)
      return false;

  return true;
}

bool board_info::ifconfig::is_blank (void) const
{
  const uint32_t* p = (const uint32_t*)this;

  for (unsigned int i = 0; i < sizeof (ifconfig) / sizeof (uint32_t); ++i)
    if (*p++ != 0)
      return false;

  return true;
}
