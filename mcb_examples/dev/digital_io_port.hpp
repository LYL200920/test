/*

a wrapper object for reading and writing digital IO ports.
the object can be passed, copied and stored by value.
instances of the object are normally returned by devices that implement
digital IO functionalities.

if digital_io_port is used as a temporary object as in

  iodev[IO_NUMBER].read ()

the temporary digital_io_port object will be optimized away (via inlining and
devirtualization).

if the digital_io_port object is stored it acts as a dynamic port binding and
will introduce some runtime overhead.

if the implementing device supports edge triggers for digital inputs,
the "set_trigger_func" function can be used to set a std::function.
the function will then be invoked in some unspecified execution context,
usually from within an ISR.

to utilize compile-time evaluation, functor objects can be created using
  bind_digital_io_port < typename DevInstFn, unsigned int PortNumber>

this will effectively create a unique type for a port on a device in the system.
when passed as template argument to another class or function, port access
can be optimized by the compiler.

*/

#ifndef includeguard_dev_digital_io_port_includeguard
#define includeguard_dev_digital_io_port_includeguard

#include <functional>
#include <utils/var_fn.hpp>
#include <utils/langcomp.hpp>

namespace dev
{

// copyable wrapper interface that can be stored and passed by value,
// assuming that the device that returns objects of this type remains alive.
class digital_io_port
{
public:
  using trigger_callback_func = std::function<void (bool port_state, unsigned int port_number)>;

  enum trigger_type
  {
    // signal goes from logic low to logic high
    rising_edge  = 0b01,

    // signal goes from logic high to logic low
    falling_edge = 0b10,

    // any signal state transition
    any_edge     = 0b11
  };

  enum logic
  {
    positive_logic = 0,
    negative_logic = 1
  };

  static constexpr trigger_type invert_trigger_type (trigger_type t)
  {
    // this is a 2 bit rotate
    return (trigger_type)((((unsigned int)t << 1) | ((unsigned int)t >> 1)) & 0b11);
  }

  struct dev_if
  {
    virtual bool read_port ([[gnu::unused]] unsigned int n) const { return false; }

    virtual void write_port ([[gnu::unused]] unsigned int n, [[gnu::unused]] bool val) { }

    virtual void set_trigger_func ([[gnu::unused]] unsigned int n,
				   [[gnu::unused]] trigger_type t,
				   [[gnu::unused]] trigger_callback_func f) { }

    virtual void sync (void) { }
  };

  digital_io_port (void) : m_impl (&g_null_dev) { }
  constexpr digital_io_port (dev_if* di, unsigned int n, logic l = positive_logic) : m_impl (di), m_io_num (n), m_logic (l) { }
  constexpr digital_io_port (const digital_io_port& p) : m_impl (p.m_impl), m_io_num (p.m_io_num), m_logic (p.m_logic) { }
  constexpr digital_io_port (const digital_io_port& p, logic l) : m_impl (p.m_impl), m_io_num (p.m_io_num), m_logic (l) { }

  bool read (void) const
  {
    assume_always_true (m_impl != nullptr);
    assume_always_true (m_logic == positive_logic || m_logic == negative_logic);
    return m_impl->read_port (m_io_num) ^ (m_logic == negative_logic);
  }

  void write (bool val)
  {
    assume_always_true (m_impl != nullptr);
    assume_always_true (m_logic == positive_logic || m_logic == negative_logic);
    m_impl->write_port (m_io_num, val ^ (m_logic == negative_logic));
  }

  template <typename T>
  void set_trigger_func (trigger_type t, T&& f)
  {
    m_impl->set_trigger_func (m_io_num, t, std::forward<T> (f));
  }

  void sync (void) { m_impl->sync (); }

  enum logic logic (void) const { return m_logic; }

  static dev_if g_null_dev;

  bool operator == (const digital_io_port& other) const
  {
    return m_impl == other.m_impl && m_io_num == other.m_io_num && m_logic == other.m_logic;
  }
  bool operator != (const digital_io_port& other) const
  {
    return !(*this == other);
  }

  constexpr unsigned int port_num (void) const { return m_io_num; }

private:
  dev_if* m_impl;
  unsigned int m_io_num;
  enum logic m_logic = positive_logic;
};

template <enum digital_io_port::logic Logic>
class digital_io_port_logic_wrapper : public digital_io_port
{
public:
  digital_io_port_logic_wrapper (void) : digital_io_port () { }
  constexpr digital_io_port_logic_wrapper (dev_if* di, unsigned int n) : digital_io_port (di, n, Logic) { }
  constexpr digital_io_port_logic_wrapper (const digital_io_port& p) : digital_io_port (p, Logic) { }

  #if 0
  FIXME: moved into digital_io_port base class.
  it results in increased object size and possibly runtime overhead when the
  digital_io_port_logic_wrapper is used as a stored value.  keep this around
  for a while as a reminder.  check it later.

  bool read (void) const
  {
    return digital_io_port::read () ^ (Logic == negative_logic);
  }
  void write (bool val)
  {
    digital_io_port::write (val ^ (Logic == negative_logic));
  }
  #endif
};

template <typename DevInstFunc, unsigned int PortNumber, enum digital_io_port::logic Logic> struct bind_digital_output_port_write_t;
template <typename DevInstFunc, unsigned int PortNumber, enum digital_io_port::logic Logic> struct bind_digital_input_port_read_t;

template <typename DevInstFunc, unsigned int PortNumber,
	  enum digital_io_port::logic Logic = digital_io_port::positive_logic>
class bound_digital_io_port_static : private DevInstFunc
{
public:
  using dev_inst_func = DevInstFunc;
  using dev_type = std::remove_reference_t<std::invoke_result_t<DevInstFunc>>;
  static constexpr unsigned int port_number = PortNumber;
  static constexpr auto logic = Logic;

  using bind_write_t = bind_digital_output_port_write_t < DevInstFunc, PortNumber, Logic >;
  using bind_read_t  = bind_digital_input_port_read_t < DevInstFunc, PortNumber, Logic >;

  bound_digital_io_port_static (DevInstFunc f = { }) : DevInstFunc (f) { }

  static constexpr auto& dev_inst (void) { return std::invoke (DevInstFunc ()); }

  auto operator () (void) const { return digital_io_port_logic_wrapper<Logic> (dev_inst ()[PortNumber]); }
};

template <typename DevInstFunc, unsigned int PortNumber,
	  enum digital_io_port::logic Logic = digital_io_port::positive_logic>
 using bind_digital_io_port_t = bound_digital_io_port_static<DevInstFunc, PortNumber, Logic>;


template <unsigned int PortNumber, typename DevInstFn>
inline auto bind_digital_io_port (DevInstFn f = { })
{
  return bound_digital_io_port_static<DevInstFn, PortNumber> (f);
}

template <typename DevInstFn, unsigned int PortNumber>
inline auto bind_digital_io_port (DevInstFn f = { })
{
  return bound_digital_io_port_static<DevInstFn, PortNumber> (f);
}

/*
// FIXME: conflicts with
//      bound_digital_io_port_dynamic bind_digital_io_port (digital_io_port p)

// enable_if only if specified parameter is a functor
template <typename PortInstFn>
class bound_digital_io_port_static2 : private PortInstFn
{
public:
  bound_digital_io_port_static2 (PortInstFn f = { }) : PortInstFn (f) { }

  auto& operator () (void) { return PortInstFn::operator ()(); }
  const auto& operator () (void) const { return PortInstFn::operator ()(); }
};

template <typename PortInstFn>
inline auto bind_digital_io_port (PortInstFn f = { })
{
  return bound_digital_io_port_static2<PortInstFn> (f);
}
*/


class bound_digital_io_port_dynamic
{
public:
  bound_digital_io_port_dynamic (digital_io_port p) : m_port (p) { }

  digital_io_port& operator () (void) { return m_port; }
  const digital_io_port& operator () (void) const { return m_port; }

private:
  digital_io_port m_port;
};

inline bound_digital_io_port_dynamic bind_digital_io_port (digital_io_port p)
{
  return { p };
}

template <typename DevFn, typename Port>
inline bound_digital_io_port_dynamic bind_digital_io_port (DevFn dev, Port p)
{
  return { dev ()[p] };
}

// ------------------------------------------------------------------------

// bind an output port to a functor

template <typename DevInstFunc, unsigned int PortNumber,
	  enum digital_io_port::logic Logic = digital_io_port::positive_logic>
struct bind_digital_output_port_write_t
{
  using dev_inst_func = DevInstFunc;
  using dev_type = std::remove_reference_t<std::invoke_result_t<DevInstFunc>>;
  static constexpr unsigned int port_number = PortNumber;
  static constexpr auto logic = Logic;

  static constexpr auto& dev_inst (void) { return std::invoke (DevInstFunc ()); }

  void operator () (bool val)
  {
    dev_inst ()[port_number].write (val ^ (logic == digital_io_port::negative_logic));
  }
};

// bind an input port to a functor

template <typename DevInstFunc, unsigned int PortNumber,
	  enum digital_io_port::logic Logic = digital_io_port::positive_logic>
struct bind_digital_input_port_read_t
{
  using dev_inst_func = DevInstFunc;
  using dev_type = std::remove_reference_t<std::invoke_result_t<DevInstFunc>>;
  static constexpr unsigned int port_number = PortNumber;
  static constexpr auto logic = Logic;

  static constexpr auto& dev_inst (void) { return std::invoke (DevInstFunc ()); }

  bool operator () (void) const
  {
    return dev_inst ()[port_number].read () ^ (logic == digital_io_port::negative_logic);
  }
};


// ------------------------------------------------------------------------
// a virtual io port for passing signals in software
// we can make a shallow 'digital_io_port' copy of it.  however copying
// the 'virtual_digital_io_port' is not allowed to ensure that there is
// only one state variable.

class virtual_digital_io_port : public digital_io_port
{
public:
  virtual_digital_io_port (enum logic l = positive_logic)
  : digital_io_port (&m_dev_if_impl, 0, l)
  {
  }

  virtual_digital_io_port (const virtual_digital_io_port&) = delete;
  virtual_digital_io_port& operator = (const virtual_digital_io_port&) = delete;

private:
  struct dev_if_impl : public digital_io_port::dev_if
  {
    bool m_var = false;

    virtual bool read_port ([[gnu::unused]] unsigned int n) const override { return m_var; }
    virtual void write_port ([[gnu::unused]] unsigned int n,  bool val) override { m_var = val; }

  } m_dev_if_impl;
};


// ------------------------------------------------------------------------
// a dummy device and port for representing not connected ports
// this can be useful to reduce application code complexity when dealing
// with different IO port hardware configurations.

class not_connected_io_port_dev final : public digital_io_port::dev_if
{
public:
  static not_connected_io_port_dev inst;

  digital_io_port operator [] (unsigned int n) { return { }; }

private:
};

struct not_connected_io_port_dev_inst
{
  not_connected_io_port_dev& operator () (void) const { return not_connected_io_port_dev::inst; }
};


} // namespace dev
#endif // includeguard_dev_digital_io_port_includeguard
