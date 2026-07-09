
/*

boost/asio buffers require an external container to make a buffer sequence
for scatter/gather operations.  the good thing about that is free choice
of container.  the downside is repetitive user code to deal with those
containers.

originally, wanted to make the buffer an intrusive forward list
to make it simpler.  however, that becomes a problem when re-using
constant buffers (e.g. Ethernet/IP headers) in multiple operations.
it would require a buffer wrapper around the constant wrapper in order
to put it in another buffer sequence/list.  in that case, we might
as well create the whole container object.

unlike in boost/asio, a buffer object to be a wrapper for a strong
reference to the actual data object (shared_ptr semantics).


the following will automatically create a reference counted owning data
wrapper object, no data will be copied (if possible):
  buffer (std::string (... ))
  buffer (std::array (... ))
  buffer (std::vector (... ))

the following will create a reference counted data wrapper object and
copy the data:
  buffer ("test moon"); // treated as fixed size char array


the following will create a reference counted data wrapper object but
effectively as non-owning, because the container itself is non-owning:
  buffer (std::string_view ("test moon"));

to avoid the wrapper object allocation/creation:
  buffer (buffer::weak, std::string_view ("test moon"));

*/

#ifndef includeguard_utils_buffer_hpp_includeguard
#define includeguard_utils_buffer_hpp_includeguard

#include <utils/ref_counted_obj.hpp>
#include <utils/container.hpp>

namespace utils
{

class buffer_list;

class buffer_data : public ref_counted_obj<buffer_data>
{
public:
  using size_type = std::size_t;

  virtual ~buffer_data (void) noexcept { }
  virtual const void* data (void) const = 0;
  virtual size_type size (void) const = 0;
};


template <typename T, typename Enable = void>
class buffer_data_wrapper : public buffer_data
{
public:
  template <typename TT>
  buffer_data_wrapper (TT&& t) : m_obj (std::forward<TT> (t)) { }

  virtual ~buffer_data_wrapper (void) override { }

  virtual const void* data (void) const override { return &m_obj; }
  virtual size_type size (void) const override { return sizeof (m_obj); }

private:
  T m_obj;
};

template <typename T>
class buffer_data_wrapper<T, typename std::enable_if<is_contiguous_container<T>::value>::type>
: public buffer_data
{
public:
  template <typename... Args>
  buffer_data_wrapper (Args&&... args) : m_obj (std::forward<Args> (args)...) { }

  buffer_data_wrapper (T&& args) : m_obj (std::move (args)) { }

  virtual ~buffer_data_wrapper (void) override { }

  virtual const void* data (void) const override
  {
    return m_obj.data ();
  }

  virtual size_type size (void) const override
  {
    return sizeof (typename T::value_type) * m_obj.size ();
  }

private:
  T m_obj;
};


class buffer
{
  template <typename T, typename Enable = void> struct auto_wrapper_ctor;

  template <typename T> struct auto_wrapper_ctor <T,
	typename std::enable_if<!std::is_convertible <T, ref_counting_ptr<buffer_data>>::value>::type>
  {
    static constexpr bool is_valid = true;
  };

public:
  enum weak_tag { weak };

  using size_type = std::size_t;

  // create an empty buffer.
  buffer (void) : m_data (nullptr), m_size (0) { }

  // create a weak referencing buffer
  buffer (weak_tag, const void* data, size_type size) : m_data (data), m_size (size) { }

  // create a strong referencing buffer
  buffer (const ref_counting_ptr<buffer_data>& data)
  : m_data (data->data ()), m_size (data->size ()),
    m_data_obj (data)
  {
  }

  // automatically create a strong referencing buffer from the specified
  // object.
  template <typename T, bool = auto_wrapper_ctor<T>::is_valid> buffer (T&& obj)
  {
   m_data_obj = make_ref_counted<buffer_data_wrapper<T>> (std::forward<T> (obj));
   m_data = m_data_obj->data ();
   m_size = m_data_obj->size ();
  }

  // create an explicitly weak reference.
  // useful with string literals wrapped in a string_view.
  template <typename T> buffer (weak_tag, T&& obj)
  {
    m_data = obj.data ();
    m_size = obj.size ();
  }

  template <typename T, std::size_t N> buffer (weak_tag, T const (&str)[N])
  {
    m_data = str;
    m_size = sizeof (T) * N;
  }

  // copy the buffer and the data object reference.
  // does not copy the actual data.
  buffer (const buffer& rhs)
  : m_data (rhs.m_data), m_size (rhs.m_size), m_data_obj (rhs.m_data_obj) { }

  // move and make rhs empty
  buffer (buffer&& rhs)
  : m_data (std::move (rhs.m_data)), m_size (std::move (rhs.m_size)),
    m_data_obj (std::move (rhs.m_data_obj))
  {
    rhs.m_data = nullptr;
    rhs.m_size = 0;
    rhs.m_data_obj = { };
  }

  buffer& operator = (const buffer& rhs)
  {
    if (this != &rhs)
    {
      m_data = rhs.m_data;
      m_size = rhs.m_size;
      m_data_obj = rhs.m_data_obj;
    }
    return *this;
  }

  buffer& operator = (buffer&& rhs)
  {
    if (this != &rhs)
    {
      m_data = std::move (rhs.m_data);
      m_size = std::move (rhs.m_size);
      m_data_obj = std::move (rhs.m_data_obj);
      rhs.m_data = nullptr;
      rhs.m_size = 0;
      rhs.m_data_obj = { };
    }
    return *this;
  }

  const void* data (void) const { return m_data; }
  size_type size (void) const { return m_size; }
  bool empty (void) const { return size () == 0; }

private:
  const void* m_data;
  size_type m_size;
  ref_counting_ptr<buffer_data> m_data_obj;
};

template <typename Container>
inline buffer::size_type sum_buffer_sizes (const Container& c)
{
  buffer::size_type r = 0;

  for (auto&& b : c)
    r += b.size ();

  return r;
}

} // namespace utils
#endif // includeguard_utils_buffer_hpp_includeguard
