
#include <cassert>
#include <cstdint>

#include "version_tag.hpp"

// these two are provided by the linker script.
extern "C" [[gnu::weak]] char _version_tag_start;
extern "C" [[gnu::weak]] char _version_tag_end;

// this is to just have those symbols on mingw, where there is no concept
// of "weak" symbols which are nulled out if not actually defined.
// we better not define it on elf targets, as the local symbol definitions
// might shadow the global weak definitions.
#ifndef __ELF__
char _version_tag_start = 0;
char _version_tag_end = 0;
#endif

[[gnu::cold]] version_tag::version_tag (void)
: m_data_begin (&_version_tag_start), m_data_end (&_version_tag_end)
{
}

static constexpr bool has_zero (uint32_t x)
{
  return (x - 0x01010101) & ~x & 0x80808080;
}


[[gnu::cold]] static std::string_view
validate_str (const char* s, const void* smax)
{
  const char* s_end = s;
  while (s < smax)
  {
    char c = *s_end;
    if (c == 0)
      break;

    // any non 7 bit character or a non printable character is an indicator
    // for an invalid string.
    if (c < ' ' || c > '~')
      return { };

    ++s_end;
  }

  if (s == smax)
    return { };

  return { s, size_t (s_end - s) };
}

[[gnu::cold]] std::string_view
version_tag::str (str_num n) const
{
  // find the i-th string in the data.  all the strings are 4 byte aligned,
  // so we can accelerate the search a bit.  if one 32 bit word contains any
  // zero byte, it is the last 32 bit word of the string.
  const uint32_t* s = (const uint32_t*)((uintptr_t)m_data_begin & ~uintptr_t(3));
  const uint32_t* send = (const uint32_t*)((uintptr_t)m_data_end & ~uintptr_t(3));
  const uint32_t* r = s;

  for (int i = (int)n; i > 0 && s < send; --i)
  {
    while (s < send && !has_zero (*s++)) { }

    r = s;
  }

  return validate_str ((const char*)r, m_data_end);
}
