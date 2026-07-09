
/*

string tokenizer utiliy

it does not decompose the string into actual copies of substrings.
instead it provides a container view that can be iterated.
the strings can be copied out using std::copy into some other container
if needed.

the idea is based on boost tokenizer.

if the input string does not contain a delimiter, the tokenizer will
return one single token.

if empty tokens are to be kept, an empty token will be emitted if
one delimiter follows another delimiter.

usage:

std::string my_string = ...;

tokenizer<std::string::iterator, char_separator<char>>
  tokens (my_string, ' ', keep_empty_tokens);

for (auto&& t : tokens)
{
  ...
}


convenience function:

std::string my_string = ...;

auto tokens = tokenize (my_string, char_separator<char> (' '));


auto tokens = tokenize (my_string, std::isspace);  // this doesn't work!
auto tokens = tokenize (my_string, char_predicate_separator (&isspace)); // have to use this instead

*/


#ifndef includeguard_utils_tokenizer_hpp_includeguard
#define includeguard_utils_tokenizer_hpp_includeguard

#include <cstring>
#include <string>
#include <string_view>
#include <utility>

#include <utils/iterator.hpp>

namespace utils
{

enum empty_token_policy
{
  drop_empty_tokens,
  keep_empty_tokens
};

template < typename InputIterator, typename TokenizerFunc >
class tokenizer
{
  using input_value_type = typename std::iterator_traits<InputIterator>::value_type;
  using string_view_type = std::basic_string_view<input_value_type>;

  struct end_func
  {
    template <typename Iter, typename... FuncArgs>
    end_func (Iter&& i, FuncArgs&&... f)
    : end (std::forward<Iter> (i)), func (std::forward<FuncArgs> (f)...) { }

    InputIterator end;
    TokenizerFunc func;
  };

public:

  using value_type = string_view_type;
  using reference = const value_type&;
  using const_reference = const value_type&;

  // because we use string_view to represent the tokens, it implies that
  // the source data container is some contiguous storage, which is normally
  // true if it supports random access iterators.
  // this works if the source data container is std::vector, std::array,
  // std::string, std::string_view, utils::static_vector, ...
  static_assert (std::is_convertible
	< typename std::iterator_traits<InputIterator>::iterator_category,
	  std::random_access_iterator_tag >::value, "");

  class iterator
  {
  public:
    // std::distance of two of these iterators is the number of token steps.
    // so the distance unit is not related to a pointer.
    using difference_type = std::size_t;
    using value_type = string_view_type;
    using pointer = const value_type*;
    using reference = const value_type&;
    using iterator_category = std::forward_iterator_tag;

    iterator (void) = default;  // singular value, comparisons undefined
    iterator (const iterator& other) = default;

    iterator (InputIterator i, const end_func& ef)
    : m_cur_token_begin (i), m_cur_token_end (i), m_end_func (&ef)
    {
      if (m_cur_token_begin != m_end_func->end)
      {
	// expand the current token until it hits the delimiter
	for (; m_cur_token_end != m_end_func->end; ++m_cur_token_end)
	  if (m_end_func->func.match (m_cur_token_end, m_end_func->end))
	    break;

	// if the input string starts with a delimiter, we'll get an
	// empty token.  skip it.
	if (m_cur_token_begin == m_cur_token_end)
	  operator++ ();
      }

      m_cur_val = string_view_type (&*m_cur_token_begin,
				    m_cur_token_end - m_cur_token_begin);
    }

    iterator& operator ++ (void)
    {
      do
      {
	if (m_cur_token_end != m_end_func->end)
	  m_end_func->func.advance (m_cur_token_end);

	m_cur_token_begin = m_cur_token_end;

	for (m_cur_token_end = m_cur_token_begin;
	     m_cur_token_end != m_end_func->end; ++m_cur_token_end)
	  if (m_end_func->func.match (m_cur_token_end, m_end_func->end))
	    break;

	m_cur_val = string_view_type (&*m_cur_token_begin,
					m_cur_token_end - m_cur_token_begin);

       } while (m_end_func->func.empty_token_policy () == drop_empty_tokens
		&& m_cur_token_begin == m_cur_token_end
		&& m_cur_token_end != m_end_func->end);

      return *this;
    }


    iterator operator ++ (int)
    {
      iterator r = *this;
      operator++ ();
      return r;
    }

    void swap (iterator& other) noexcept
    {
      std::swap (m_cur_token_begin, other.m_cur_token_begin);
      std::swap (m_cur_token_end, other.m_cur_token_end);
      std::swap (m_end_func, other.m_end_func);
      std::swap (m_cur_val, other.m_cur_val);
    }

    bool operator == (const iterator& rhs) const
    {
      return m_cur_token_begin == rhs.m_cur_token_begin;
    }

    bool operator != (const iterator& rhs) const { return !(*this == rhs); }
    

    reference operator * (void) const { return m_cur_val; }
    pointer operator -> (void) const { return &m_cur_val; }

  private:
    InputIterator m_cur_token_begin;
    InputIterator m_cur_token_end;  // = first char of the token separator (or m_end).
    const end_func* m_end_func;
    string_view_type m_cur_val;
  };

  using const_iterator = iterator;
  using difference_type = typename iterator::difference_type;
  using size_type = difference_type;

  template <typename InputIt>
  tokenizer (InputIt first,
	     typename std::enable_if<is_iterator<InputIt>::value, InputIt>::type last,
	     const TokenizerFunc& f)
  : m_first (first), m_end_func (last, f) { }

  template <typename InputIt>
  tokenizer (InputIt first,
	     typename std::enable_if<is_iterator<InputIt>::value, InputIt>::type last,
	     TokenizerFunc&& f)
  : m_first (first), m_end_func (last, std::move (f)) { }

  template <typename InputIt, typename... Args>
  tokenizer (InputIt first,
	     typename std::enable_if<is_iterator<InputIt>::value, InputIt>::type last,
	     Args&&... args)
  : m_first (first), m_end_func (last, std::forward<Args> (args)...) { }


  template <typename Container,
	    typename = typename std::enable_if<!is_iterator<Container>::value>::type>
  tokenizer (const Container& c, const TokenizerFunc& f)
  : m_first (c.begin ()), m_end_func (c.end (), f) { }

  template <typename Container,
	    typename = typename std::enable_if<!is_iterator<Container>::value>::type>
  tokenizer (const Container& c, TokenizerFunc&& f)
  : m_first (c.begin ()), m_end_func (c.end (), std::move (f)) { }

  template <typename Container, typename... Args,
	    typename = typename std::enable_if<!is_iterator<Container>::value>::type>
  tokenizer (const Container& c, Args&&... args)
  : m_first (c.begin ()), m_end_func (c.end (), std::forward<Args> (args)...) { }


  template <typename InputIt>
  void assign (InputIt first,
	       typename std::enable_if<is_iterator<InputIt>::value, InputIt>::type last)
  {
    m_first = first;
    m_end_func.end = last;
  }

  template <typename InputIt>
  void assign (InputIt first,
	       typename std::enable_if<is_iterator<InputIt>::value, InputIt>::type last,
	       const TokenizerFunc& f)
  {
    assign (first, last);
    m_end_func.func = f;
  }

  template <typename InputIt>
  void assign (InputIt first,
	       typename std::enable_if<is_iterator<InputIt>::value, InputIt>::type last,
	       TokenizerFunc&& f)
  {
    assign (first, last);
    m_end_func.func = f;
  }

  template <typename InputIt, typename... Args>
  void assign (InputIt first,
	       typename std::enable_if<is_iterator<InputIt>::value, InputIt>::type last,
	       Args&&... args)
  {
    assign (first, last);
    m_end_func.func = TokenizerFunc (std::forward<Args> (args)...);
  }

  template <typename Container,
	    typename = typename std::enable_if<!is_iterator<Container>::value>::type>
  void assign (const Container& c)
  {
    assign (c.begin(), c.end ());
  }

  template <typename Container,
	    typename = typename std::enable_if<!is_iterator<Container>::value>::type>
  void assign (const Container& c, const TokenizerFunc& f)
  {
    assign (c.begin (), c.end (), f);
  }

  template <typename Container,
	    typename = typename std::enable_if<!is_iterator<Container>::value>::type>
  void assign (const Container& c, TokenizerFunc&& f)
  {
    assign (c.begin (), c.end (), std::move (f));
  }

  template <typename Container, typename... Args,
	    typename = typename std::enable_if<!is_iterator<Container>::value>::type>
  void assign (const Container& c, Args&&... args)
  {
    assign (c.begin (), c.end (), std::forward<Args> (args)...);
  }

  iterator begin (void) const { return iterator (m_first, m_end_func); }
  iterator end (void) const { return iterator (m_end_func.end, m_end_func); }

  iterator cbegin (void) const { return begin (); }
  iterator cend (void) const { return end (); }

  size_type size (void) const
  {
    if (!m_size_known)
    {
      m_size = std::distance (begin (), end ());
      m_size_known = true;
    }
    return m_size;
  }

  bool empty (void) const { return size () == 0; }

private:
  mutable bool m_size_known = false;
  mutable size_type m_size;
  InputIterator m_first;
  end_func m_end_func;
};

// ---------------------------------------------------------

template <typename Char>
class char_separator
{
public:
  char_separator (Char val, empty_token_policy ep = drop_empty_tokens)
  : m_value (val), m_ep (ep) { }

  enum empty_token_policy empty_token_policy (void) const { return m_ep; }

  template <typename InputIterator>
  bool match (InputIterator& i, const InputIterator& end) const
  {
    return *i == m_value;
  }

  template <typename InputIterator>
  void advance (InputIterator& i) const
  {
    ++i;
  }

private:
  Char m_value;
  enum empty_token_policy m_ep;
};


template <typename Func>
class char_predicate_separator
{
public:
  char_predicate_separator (Func&& f, empty_token_policy ep = drop_empty_tokens)
  : m_func (std::move (f)), m_ep (ep) { }

  enum empty_token_policy empty_token_policy (void) const { return m_ep; }

  template <typename InputIterator>
  bool match (InputIterator& i, const InputIterator& end) const
  {
    return m_func (*i);
  }

  template <typename InputIterator>
  void advance (InputIterator& i) const
  {
    ++i;
  }

private:
  Func m_func;
  enum empty_token_policy m_ep;
};

// ---------------------------------------------------------

template <typename Container, typename TokenizerFunc >
inline auto
tokenize (const Container& c, TokenizerFunc&& f)
{
  return tokenizer<typename Container::const_iterator, TokenizerFunc > (
	c, std::forward<TokenizerFunc> (f));
}

template < typename TokenizerFunc >
inline auto
tokenize (const char* str, TokenizerFunc&& f)
{
  return tokenizer<const char*, TokenizerFunc > (
	str, str + std::strlen (str),
	std::forward<TokenizerFunc> (f));
}

// FIXME: add overloads for something "callable" and use
// char_predicate_separator automatically.

#undef impl_utils_tokenizer_use_string_view_type_namespace

} // namespace utils
#endif // includeguard_utils_tokenizer_hpp_includeguard
