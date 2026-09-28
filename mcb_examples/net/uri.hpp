
#ifndef includeguard_net_uri_hpp_includeboard
#define includeguard_net_uri_hpp_includeboard

#include <string_view>
#include <iterator>

namespace net
{

class uri
{
public:

  class query_params
  {
  public:

    // construct from string of key=value&key=value ...
    query_params (std::string_view str) : m_str (str) { }

    class iterator
    {
    public:
      using difference_type = std::size_t;
      using value_type = std::pair<std::string_view, std::string_view>;
      using pointer = const value_type*;
      using reference = const value_type&;
      using iterator_category = std::forward_iterator_tag;

      iterator (void) = default;
      iterator (const iterator& other) = default;

      iterator (std::string_view::iterator end)
      : m_i0 (end), m_i1 (end), m_i2 (end), m_end (end) { }

      iterator (std::string_view::iterator i, std::string_view::iterator end);

      iterator& operator ++ (void);

      iterator operator ++ (int)
      {
	iterator r = *this;
	operator++ ();
	return r;
      }

      void swap (iterator& other) noexcept
      {
	std::swap (m_i0, other.m_i0);
	std::swap (m_i1, other.m_i1);
	std::swap (m_i2, other.m_i2);
	std::swap (m_end, other.m_end);
      }

      bool operator == (const iterator& rhs) const
      {
	return m_i0 == rhs.m_i0 && m_i1 == rhs.m_i1 && m_i2 == rhs.m_i2;
      }

      bool operator != (const iterator& rhs) const
      {
	return m_i0 != rhs.m_i1 || m_i1 != rhs.m_i1 || m_i2 != rhs.m_i2;
      }

      const value_type operator * (void) const
      {
	return
        {
	  std::string_view (&*m_i0, m_i1 - m_i0),
	  std::string_view (&*(m_i1+1), m_i1 == m_i2 ? 0 : m_i2 - (m_i1+1))
        };
      }

    private:
      void find_next_kv (std::string_view::iterator i, std::string_view::iterator end);

      std::string_view::iterator m_i0;   // key string begin
      std::string_view::iterator m_i1;   // key string end, '=' char if any
      std::string_view::iterator m_i2;   // value string end
      std::string_view::iterator m_end;
    };


    iterator begin (void) const { return iterator (m_str.begin (), m_str.end ()); }
    iterator end (void) const { return iterator (m_str.end ()); }

  private:
    std::string_view m_str;
  };



};

} // namespace net
#endif // includeguard_net_uri_hpp_includeboard
