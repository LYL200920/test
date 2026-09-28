
#include "uri.hpp"

namespace net
{

uri::query_params::iterator::iterator (std::string_view::iterator i,
				       std::string_view::iterator end)
{
  m_end = end;
  find_next_kv (i, end);
}

uri::query_params::iterator& uri::query_params::iterator::operator ++ (void)
{
  if (m_i2 == m_end)
    m_i0 = m_i1 = m_end;
  else
    find_next_kv (m_i2 + 1, m_end);

  return *this;
}

void uri::query_params::iterator::find_next_kv (std::string_view::iterator i,
						std::string_view::iterator end)
{
  if (i == end)
  {
    m_i0 = m_i1 = m_i2 = end;
    return;
  }

  m_i0 = i;

  // support ';' and '&' as separators and optional values
  for (; ; ++i)
  {
    if (i == end)
    {
      m_i1 = m_i2 = i;
      return;
    }
    else if (*i == '=' || *i == ';' || *i == '&')
    {
      m_i1 = i++;
      break;
    }
  }

  for (; i != end; ++i)
  {
    if (*i == ';' || *i == '&')
    {
      m_i2 = i;
      return;
    }
  }

  m_i2 = end;
}



} // namespace net
