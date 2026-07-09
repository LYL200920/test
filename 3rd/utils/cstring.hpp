/*

utilities for MS CString stuff

*/

#ifndef includeguard_utils_cstring_hpp_includeguard
#define includeguard_utils_cstring_hpp_includeguard

#include <string>
#include <stringapiset.h>

#if __has_include (<experimental/string_view>)
  #include <experimental/string_view>
#endif

#if __cplusplus >= 201703L && __has_include (<string_view>)
  #include <string_view>
#endif

namespace utils
{
inline std::string to_utf8 (const CStringW& s)
{
  if (s.IsEmpty ())
    return { };

  auto reqlen = WideCharToMultiByte (CP_UTF8, 0, s, s.GetLength (), nullptr, 0, 0, 0);
  if (reqlen <= 0)
    return { };

  std::string result (reqlen, 0);
  auto reslen = WideCharToMultiByte (CP_UTF8, 0, s, s.GetLength (), &result.front (), reqlen, 0, 0);
  assert (reslen == reqlen);
  return std::move (result);
}

// this one works with
//  std::experimental::string_view
//  std::string_view
//  std::string
//  ...
template <typename T>
inline CStringW to_cstring (const T& str)
{
  if (str.empty ())
    return { };

  auto reqlen = MultiByteToWideChar (CP_UTF8, 0, str.data (), str.size (), nullptr, 0);
  if (reqlen <= 0)
    return { };

  CStringW result;
  auto* outbuf = result.GetBuffer (reqlen);
  MultiByteToWideChar (CP_UTF8, 0, str.data (), str.size (), outbuf, reqlen);
  result.ReleaseBuffer (reqlen);
  return result;
}

} // namespace utils

#endif // includeguard_utils_cstring_hpp_includeguard
