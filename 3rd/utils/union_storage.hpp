/*

a stateful container for storing one out of multiple possible types.
the container reserves enough space for storing one instance of the specified
types.  it uses in-place construction via placement new to initialize the object
and when the container is deleted it will invoke the destructor of the last
type that has been constructed.

*/

#ifndef includeguard_utils_union_storage_includeguard
#define includeguard_utils_union_storage_includeguard

namespace utils
{

template <typename... Ts> class union_storage
{
public:
  union_storage (void) : m_dtor_func (nullptr) { }
  union_storage (const union_storage&) = delete;
  union_storage (union_storage&&) = delete;
  union_storage& operator = (const union_storage&) = delete;
  union_storage& operator = (union_storage&&) = delete;

  ~union_storage (void)
  {
    clear ();
  }

  template <typename T, typename... Args> T* make_new (Args&&... args)
  {
    // delete previous object
    clear ();

    // use placement-new to iniitialize a new object
    T* new_obj = new (m_data) T (std::forward<Args> (args)...);

    // set new delete function
    m_dtor_func = [] (std::byte* data)
    {
      reinterpret_cast<T*> (data)->~T ();
    };

    return new_obj;
  }

  template <typename T> T* get (void)
  {
    return empty () ? nullptr : reinterpret_cast<T*> (m_data);
  }

  template <typename T> const T* get (void) const
  {
    return empty () ? nullptr : reinterpret_cast<const T*> (m_data);
  }

  void clear (void)
  {
    if (m_dtor_func)
      m_dtor_func (m_data);

    m_dtor_func = nullptr;
  }

  bool empty (void) const
  {
    return m_dtor_func == nullptr;
  }

private:
  // FIXME: if all types are trivially destructable, then can omit the dtor function
  void (*m_dtor_func) (std::byte* data) = nullptr;
  alignas (Ts...) std::byte m_data[std::max (sizeof (Ts)...)];
};

} // namespace utils
#endif // includeguard_utils_union_storage_includeguard