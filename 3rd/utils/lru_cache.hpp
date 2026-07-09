#ifndef includeguard_lru_cache_hpp_includeguard
#define includeguard_lru_cache_hpp_includeguard

#include <map>
#include <list>
#include <cassert>

namespace utils
{

template <typename Key, typename Value, typename CreateFunc> class lru_cache
{
public:
  lru_cache (lru_cache&&) = default;
  lru_cache& operator = (lru_cache&&) = default;

  lru_cache (CreateFunc&& f, unsigned int capacity)
  : m_capacity (capacity), m_create_func (std::forward<CreateFunc> (f))
  {
  }

  const Value& get (const Key& k)
  {
    auto i = m_map.find (k);
    if (i == m_map.end ())
    {
      // requested entry is not in the cache.

      if (m_lru_list.empty () || m_map.size () < m_capacity)
      {
	m_lru_list.emplace_front (k, Value ());
	m_map.emplace (k, m_lru_list.begin ());
      }
      else
      {
	// move the least recently used entry in the LRU list from the back
	// to the front.
	m_lru_list.splice (m_lru_list.begin (), m_lru_list,
			   std::prev (m_lru_list.end ()));

	// erase old key
	m_map.erase (m_map.find (m_lru_list.front ().first));

	// update key in list and insert a new into the map.
	m_lru_list.front ().first = k;
	m_map.emplace (k, m_lru_list.begin ());
      }

      m_create_func (k, m_lru_list.front ().second);
    }
    else
    {
      // relink the entry to the front of the list to get the lru effect.
      m_lru_list.splice (m_lru_list.begin (), m_lru_list, i->second);
    }

    assert (m_map.size () == m_lru_list.size ());
    return m_lru_list.front ().second;
  }

  void erase (const Key& k)
  {
    auto i = m_map.find (k);
    if (i == m_map.end ())
      return;

    m_lru_list.erase (i->second);
    m_map.erase (i);
  }

private:
  unsigned int m_capacity;
  std::list<std::pair<Key, Value>> m_lru_list;
  std::map<Key, typename std::list<std::pair<Key, Value>>::iterator> m_map;

  CreateFunc m_create_func;
};

} // namespace utils
#endif // includeguard_lru_cache_hpp_includeguard
