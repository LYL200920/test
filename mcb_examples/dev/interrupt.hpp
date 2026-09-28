
#ifndef includeguard_dev_interrupt_hpp_includeguard
#define includeguard_dev_interrupt_hpp_includeguard

#include <cassert>
#include <limits>

namespace dev
{
namespace interrupt
{

enum trigger_type
{
  low_level,
  falling_edge,
  rising_edge,
  any_edge
};

enum : unsigned int
{
  priority_0,  // effectively disables the interrupt
  priority_1,
  priority_2,
  priority_3,
  priority_4,
  priority_5,
  priority_6,
  priority_7,
  priority_8,
  priority_9,
  priority_10,
  priority_11,
  priority_12,
  priority_13,
  priority_14,
  priority_15,

  min_priority = priority_0,
  max_priority = priority_15,
};

#if __has_include ("isr_func_attr")
  #include "isr_func_attr"
#else
  #define __isr_func_attr__
  #define expand_empty_isr_func static inline void empty_isr_func(void) { }
#endif

#ifdef expand_empty_isr_func
  expand_empty_isr_func
#else
static inline void __isr_func_attr__ empty_isr_func (void)
{
}
#endif

struct unconnected
{
  static constexpr int isr_num = std::numeric_limits<int>::max ();

  static void connect_func (void(*)(void)) { }
  static void enable (trigger_type, unsigned int) { }
  static void disable (void) { }
  static bool status (void) { return false; }
  static void clear (void) { }

  static void __isr_func_attr__ isr_func (void)
  {
  }
};

// ---------------------------------------------------------------------------

template <typename F, F Func> struct func;


template <void (*FuncPtr)(void*)> struct func <void(*)(void*), FuncPtr>
{
  static void invoke (void* p) { FuncPtr (p); }
};

template <void (*FuncPtr)(void)> struct func <void(*)(void), FuncPtr>
{
  static void invoke (void*) { FuncPtr (); }
};

template <typename C, void (C::*FuncPtr)(void)> struct func <void (C::*)(void), FuncPtr>
{
  static void invoke (void* p) { (((C*)p)->*FuncPtr) (); }
};

// ---------------------------------------------------------------------------

template <typename InterruptLine, typename Func, typename Enable = void> class connected_isr
{
public:
  typedef InterruptLine interrupt_line;

  static constexpr int isr_num = interrupt_line::isr_num;

  connected_isr (void* param)
  {
    // this assumes that for each ISR-InterruptLine connection there is a
    // unique type.  if the same type is constructed multiple times it will
    // be overridden.
    isr_param = param;

    // if it's a virtual interrupt line, need to latch the function pointer.
    // for real interrupts, all function pointers (to isr_func) will be
    // collected and put in the ISR table.
    // if the interrupt line doesn't support the connect_func, this will do
    // nothing.
    interrupt_line::connect_func (isr_func_1);
  }

  static void enable (trigger_type tt, unsigned int priority)
  {
    interrupt_line::enable (tt, priority);
  }

  static void disable (void)
  {
    interrupt_line::disable ();
  }

  static bool status (void)
  {
    return interrupt_line::status ();
  }

  static void clear (void)
  {
    interrupt_line::clear ();
  }

  static void __isr_func_attr__ isr_func (void)
  {
    Func::invoke (isr_param);
  }

private:
  // the function pointer if "isr_func" goes directly into the ISR table
  // in ROM.  if the ISR implementation needs some additional data, that ISR
  // function will use a global variable 'isr_param'.  if the variable is not
  // used, it will be optimized out (hopefully).
  static void* isr_param;

  // same as isr_func but without the interrupt attribute, i.e. normal function.
  static void isr_func_1 (void)
  {
    Func::invoke (isr_param);
  }
};

// this instantiates the static variable in the template class.
// with normal variables there would be multiple definitions, but with templated
// stuff the compiler/linker will make sure there's only one copy of the
// variable.
template <typename InterruptLine, typename Func, typename Enable>
void* connected_isr<InterruptLine, Func, Enable>::isr_param;

// ---------------------------------------------------------------------------
// when using virtual interrupt lines, make sure that the TYPE of the virtual
// line is UNIQUE.
// when used inside another device driver and the driver is a template itself
// (e.g. register base address is a template parameter), then there is no
// problem.
// if used for multiple instances of the same device type, the device driver
// needs to define an array of virtual lines ...
namespace detail
{
typedef void (*isr_func)(void);

inline void empty_func (void) { }
}

template <int Num>
class virtual_line
{
public:
  static constexpr int isr_num = Num;

  static void connect_func (detail::isr_func func)
  {
    g_func = func;
  }

  static void enable (trigger_type tt, unsigned int priority)
  {
    // FIXME: maybe latch the trigger parameter and use it for the trigger
    // function?
  }

  static void disable (void)
  {
  }

  static bool status (void)
  {
    return false;
  }

  static void clear (void)
  {
  }

  static void trigger (/* FIXME: maybe add this?  trigger_type tt*/)
  {
    g_func ();
  }

private:
  static detail::isr_func g_func;
};

// actually could use a lamda as an empty function, but that causes
// initialization order issues.  if the interrupts are constructed in
// the board constructor (which runs first), g_func might get overwritten
// during later construction steps.  assigning a normal function works as
// expected, as variable initialization is trivial.
template <int Num>
detail::isr_func virtual_line<Num>::g_func = detail::empty_func;


// ---------------------------------------------------------------------------


} // namespace interrupt
} // namespace dev
#endif // includeguard_dev_interrupt_hpp_includeguard
