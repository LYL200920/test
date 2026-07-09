#ifndef includeguard_state_machine_hpp_includeguard
#define includeguard_state_machine_hpp_includeguard

#include <tuple>
#include <type_traits>
#include <memory>
#include <limits>
#include <cstdint>
#include <typeinfo>
#include <cassert>
#include <chrono>

#include <utils/langcomp.hpp>
#include <utils/tuple.hpp>
#include <utils/ptm.hpp>

namespace state_machine_impl
{

// use a empty state instance base class, as it makes the code a bit
// easier to read (could also pass void* everywhere).
struct state_base { };

typedef bool (*exec_func_t)(state_base*, void*, void*, bool);

// #define STATE_MACHINE_IMPL_ENABLE_STATE_DOUBLE_BUFFER

#ifdef STATE_MACHINE_IMPL_ENABLE_STATE_DOUBLE_BUFFER

template<unsigned int MaxStateSize>
struct state_buffer
{
  state_base* buffer (void) const { return (state_base*)((char*)m_buf + cur_offset); }

  state_base* swap_buffers (void)
  {
    cur_offset ^= MaxStateSize;
    return buffer ();
  }

  // current state exec function, which is the actual state encoding.
  exec_func_t exec_func = nullptr;

  // name string of the state for debugging purposes.
  // FIXME MCB-33: we could encode the state with the smallest possible integer type
  // and use it as an index into function/string tables.  this would result
  // in one indirection when invoking the state function, but should result
  // in overall smaller code because integer constants can be used to represent
  // the state.
  const char* name_str = "nullptr";

  // offset for the first buffer (current buffer).
  int cur_offset = 0;

  // double buffer to hold two state objects (current and next).
  // if all the state objects don't have any members, the array may be of
  // zero-length.
  char m_buf[MaxStateSize * 2];
};

#else

template<unsigned int MaxStateSize>
struct state_buffer
{
  state_base* buffer (void) const { return (state_base*)(char*)m_buf; }
  state_base* swap_buffers (void) { return buffer (); }

  // current state exec function, which is the actual state encoding.
  exec_func_t exec_func = nullptr;
  const char* name_str = "nullptr";

  // buffer to hold the current state object.
  // if all the state objects don't have any members, the array may be of
  // zero-length.
  char m_buf[MaxStateSize];
};

#endif // STATE_MACHINE_IMPL_ENABLE_STATE_DOUBLE_BUFFER

template<> struct state_buffer<0>
{
  state_base* buffer (void) const { return (state_base*)(char*)m_buf; }
  state_base* swap_buffers (void) { return buffer (); }

  // current state exec function.
  exec_func_t exec_func = nullptr;
  const char* name_str = "nullptr";

#ifdef _MSC_VER
  char m_buf[1];
#else
  char m_buf[0];
#endif
};

template <typename T> using ptm_class = utils::ptm_class<T>;
template <typename T> using ptm_result = utils::ptm_result<T>;

// if the pointer-to-member-function args were function args and not template
// args, we could deduce most of the stuff like std::mem_fn does.
// we could switch from template arguments to function arguments and describe
// the state graph like...
//
// desc_states (
//   state (led_on (), transition (timer_expired (), &myclass::led_off)),
//   state (&myclass::led_off, transition (always (), led_on ()))
//
// ...and capture all necessary stuff via function return types.
// the problem with specifying a pointer-to-member-function as a function
// argument is that we can only capture the pointer, not the function itself.
// this requires storing the pointer for invocation.  specifying the function
// as a template parameter doesn't require pointer storage.

#define stm_fn(x) \
  state_machine_impl::ptmf_state <decltype(x), x>

// a wrapper functor around a pointer-to-member-function.
// notice that 'typename F' is the type of the function and 'func' is the
// actual function itself.  it's like
//    template <typename R, typename C, R (C::*F)(void)>
// but the function type is in 'F'.
//
// this allows us to invoke the function without
// actually storing the pointer-to-member.
// this works for non-const member functions and for const member functions.
// the class type is needed for invoking the pointer-to-member function.
// it's part of the function type, so we can extract it from there.

template <typename F, F func> struct ptmf_state
{
  typedef typename ptm_result<F>::type Result;
  typedef typename ptm_class<F>::type Class;

  // used for state functions
  Result enter (Class& c) const { return ((c.*func)()); }

  // used for transition predicates or for enter/exit state functions
  // used for enter/exit
  Result operator () (Class& c) const { return ((c.*func)()); }
};


// bind a member function or member data and synthesize a functor object
// out of it.  the arguments are used when invoking the function.  they are
// assumed to be constructible and invokable just as bind_fn itself.
template <typename F, F member_func_or_data, typename... Args> struct bind_fn
{
  typedef typename ptm_result<F>::type Result;
  typedef typename ptm_class<F>::type Class;

  // used for state functions
  Result enter (Class& c) const { return ((c.*member_func_or_data)()); }

  // used for transition predicates or for enter/exit state functions
  // used for enter/exit
  Result operator () (Class& c) const
  {
    // FIXME: this works only for functions.  handle variables, too.
    return ((c.*member_func_or_data)(Args ()(c)...));
  }
};


// find the target state type that can be instantiated from the state tuple.
template <typename TargetState, typename AllStates> struct find_target_state;

template <typename TargetState>
struct find_target_state<TargetState, std::tuple<>>
{
  typedef void type;
};

template <typename TargetState, typename S0, typename... SS>
struct find_target_state<TargetState, std::tuple<S0, SS...>> : 
  std::conditional<std::is_same<typename S0::state_type, TargetState>::value,
		   S0,
		   typename find_target_state<TargetState, std::tuple<SS...>>::type>
{ };

template <typename State> struct error_target_state_not_found
{
  static_assert (!std::is_same<State, State>::value, "target state not found");
};

template <typename State> struct error_state_unreachable
{
  static_assert (!std::is_same<State, State>::value, "state unreachable");
};

// the state's 'enter' and 'exit' functions are optional.  if they are
// not there, invoke nothing.
template <typename S, typename SM> struct exec_state_enter
{
  template <typename C> static auto test (decltype(&C::enter)) -> std::true_type;
  template <typename C> static auto test (...) -> std::false_type;

  template<typename SS = S>
  typename std::enable_if< decltype (test<SS> (0))::value >::type
  operator () (SS& s, SM& sm) { s.enter (sm); }

  template<typename SS = S>
  typename std::enable_if< !decltype (test<SS> (0))::value >::type
  operator () (SS& s, SM& sm) { }
};

template <typename S, typename SM> struct exec_state_exit
{
  template <typename C> static auto test (decltype(&C::exit)) -> std::true_type;
  template <typename C> static auto test (...) -> std::false_type;

  template<typename SS = S>
  typename std::enable_if< decltype (test<SS> (0))::value >::type
  operator () (SS& s, SM& sm) { s.exit (sm); }

  template<typename SS = S>
  typename std::enable_if< !decltype (test<SS> (0))::value >::type
  operator () (SS& s, SM& sm) { }
};

template <typename S, typename SM, typename F> struct exec_state_functor
{
  void operator () (S& s, SM& sm) { F () (sm); }
};


// if the state implements a function "name" use it.  otherwise
// return the standard typeid name.
template <typename S> struct get_typename_str
{
  template <typename C> static auto test (decltype(&C::name)) -> std::true_type;
  template <typename C> static auto test (...) -> std::false_type;

  template<typename SS = S>
  auto operator() () -> typename std::enable_if< decltype(test<SS>(0))::value, decltype(SS::name()) >::type
  { return S::name (); }

  template<typename SS = S>
  auto operator() () -> typename std::enable_if< !decltype(test<SS>(0))::value, const char* >::type
  {
    #if __cpp_rtti
      return typeid (S).name ();
    #else
      return "";
    #endif
  }
};

// derive from State and the transitions tuple to utilize empty base class.
template <typename StateMachine,
	  typename State,
	  typename AllStates,
	  unsigned int MaxStateSize,
	  typename EnterFunc,
	  typename ExitFunc,
	  typename Transitions>
struct state_instance : public state_base,
			public State,
			public Transitions
{
  typedef Transitions transitions_tuple;

  State& state (void) { return *this; }
  transitions_tuple& transitions (void) { return *this; }

  static /*__attribute__((noinline)) */ bool exec (state_base* s, void* sm, void* sb, bool abort_state)
  {
    // maybe for hierarchial state machines ...
    // if 'State' is a subclass of 'state_machine' invoke it's exec function,
    // then check the transitions.

    auto* sbb = (state_machine_impl::state_buffer<MaxStateSize>*)sb;
    auto* ss = (state_instance*)s;
    auto* smm = (StateMachine*)sm;

    if (unlikely (abort_state))
    {
      // exit functions may contain code which continues some kind of operation
      // like ... wait for timeout, start/continue motor drive in the exit
      // function and transition to some other state which continues normal
      // "execution flow".  because of this, it's better not to invoke the
      // exit function when the state is aborted (e.g. by a reset).
      ss->destruct (*smm);
      return false;
    }
    else
      return ss->check_transition (*smm, *sbb);
  }

  static const char* name_str (void)
  {
    return get_typename_str<State> () ();
  }

  void /*__attribute__((noinline)) */ enter (StateMachine& sm)
  {
    EnterFunc () (state (), sm);
  }
  void /* __attribute__((noinline)) */ exit_destruct (StateMachine& sm)
  {
    ExitFunc () (state (), sm);
    destruct (sm);
  }
  void destruct (StateMachine& sm)
  {
    this->~state_instance ();
  }

  template <unsigned int I = 0>
  typename std::enable_if<I == std::tuple_size<transitions_tuple>::value, bool>::type
  check_transition (StateMachine&, state_machine_impl::state_buffer<MaxStateSize>&)
  {
    return false;
  }

  template <unsigned int I = 0>
  typename std::enable_if<I < std::tuple_size<transitions_tuple>::value, bool>::type
  check_transition (StateMachine& sm, state_machine_impl::state_buffer<MaxStateSize>& sb)
  {
    if (std::get<I> (transitions ()).template check<AllStates> (this, sm, sb))
      return true;

    return check_transition<I + 1> (sm, sb);
  }
};

struct enter_func_tag { };
struct exit_func_tag { };

// a state is specified by the user as
//    state < typename,
//		{ enter func typename }, { exit func typename },
//            transition typenames ... >
//
// where enter func typename and exit func typename are optional.
// enter and exit are derived from the enter_func_tag and exit_func_tag
// so we specialize the template <State, Transitions...> on the first two
// types of the parameter pack and try to extract the expected subclasses.
  
template <typename State, typename T0, typename... Ts>
struct state_actions_0
{
  typedef typename utils::tuple_remove<void, T0, Ts...>::type transitions;

  template <typename StateMachine> using enter_func = exec_state_enter<State, StateMachine>;
  template <typename StateMachine> using exit_func = exec_state_exit<State, StateMachine>;
};

template <typename State, typename EnterFunc, typename... Ts>
struct state_actions_1
{
  typedef typename utils::tuple_remove<void, Ts...>::type transitions;

  template <typename StateMachine> struct enter_func
  {
    void operator () (State& s, StateMachine& sm)
    {
      EnterFunc () (s, sm);
    }
  };

  template <typename StateMachine> using exit_func = exec_state_exit<State, StateMachine>;
};

template <typename State, typename ExitFunc, typename... Ts>
struct state_actions_2
{
  typedef typename utils::tuple_remove<void, Ts...>::type transitions;

  template <typename StateMachine> struct exit_func
  {
    void operator () (State& s, StateMachine& sm)
    {
      ExitFunc () (s, sm);
    }
  };

  template <typename StateMachine> using enter_func = exec_state_enter<State, StateMachine>;
};

template <typename State, typename EnterFunc, typename ExitFunc, typename... Ts>
struct state_actions_3
{
  typedef typename utils::tuple_remove<void, Ts...>::type transitions;

  template <typename StateMachine> struct enter_func
  {
    void operator () (State& s, StateMachine& sm)
    {
      EnterFunc () (s, sm);
    }
  };

  template <typename StateMachine> struct exit_func
  {
    void operator () (State& s, StateMachine& sm)
    {
      ExitFunc () (s, sm);
    }
  };
};


template <typename State, typename... Transitions> struct state_actions
{
  template <typename StateMachine> using enter_func = exec_state_enter<State, StateMachine>;
  template <typename StateMachine> using exit_func = exec_state_exit<State, StateMachine>;

  typedef utils::tuple_remove<void, Transitions...> transitions;
};

template <typename State, typename MaybeEnterExitFunc, typename... Transitions>
struct state_actions<State, MaybeEnterExitFunc, Transitions...> 
  : public std::conditional<
      std::is_base_of < enter_func_tag, MaybeEnterExitFunc >::value,
      state_actions_1<State, MaybeEnterExitFunc, Transitions...>,
      typename std::conditional < std::is_base_of < exit_func_tag, MaybeEnterExitFunc >::value,
				  state_actions_2<State, MaybeEnterExitFunc, Transitions...>,
				  state_actions_0<State, MaybeEnterExitFunc, Transitions...>>::type >:: type
{ };

template <typename State, typename MaybeEnterExitFunc, typename MaybeExitFunc, typename... Transitions>
struct state_actions<State, MaybeEnterExitFunc, MaybeExitFunc, Transitions...> 
  : public std::conditional<
	// enter func, exit func, transitions ...
	std::is_base_of < enter_func_tag, MaybeEnterExitFunc >::value
	&& std::is_base_of < exit_func_tag, MaybeExitFunc >::value,
	state_actions_3<State, MaybeEnterExitFunc, MaybeExitFunc, Transitions...>,

	// enter func, transitions ...
	// exit func, transitions ...
	typename std::conditional < std::is_base_of < enter_func_tag, MaybeEnterExitFunc >::value,
				    state_actions_1<State, MaybeEnterExitFunc, MaybeExitFunc, Transitions...>,

		typename std::conditional < std::is_base_of < exit_func_tag, MaybeEnterExitFunc >::value,
					    state_actions_2<State, MaybeEnterExitFunc, MaybeExitFunc, Transitions...>,
					    state_actions_0<State, MaybeEnterExitFunc, MaybeExitFunc, Transitions...>
					  >::type
				  >:: type
	>::type
{ };

} // namespace state_machine_impl

class state_machine
{
public:
  // exec function can be overridden by the implementing state machine
  // subclass.  it will be invoked each time the finite_state_machine::exec
  // is invoked, once before iterating over the states.
  void exec (void)
  {
  }

  template <typename... Funcs> struct enter : public state_machine_impl::enter_func_tag
  {
    template <typename State, typename StateMachine>
    void invoke (State& s, StateMachine& sm) { }

    template <typename State, typename StateMachine, typename F, typename... Fs>
    void invoke (State& s, StateMachine& sm)
    {
      F () (sm);
      invoke<State, StateMachine, Fs...> (s, sm);
    }
   
    template <typename State, typename StateMachine>
    void operator () (State& s, StateMachine& sm)
    {
      state_machine_impl::exec_state_enter<State, StateMachine> () (s, sm);
      invoke<State, StateMachine, Funcs...> (s, sm);
    }
  };

  template <typename... Funcs> struct exit : public state_machine_impl::exit_func_tag
  {
    template <typename State, typename StateMachine>
    void invoke (State& s, StateMachine& sm) { }

    template <typename State, typename StateMachine, typename F, typename... Fs>
    void invoke (State& s, StateMachine& sm)
    {
      F () (sm);
      invoke<State, StateMachine, Fs...> (s, sm);
    }
   
    template <typename State, typename StateMachine>
    void operator () (State& s, StateMachine& sm)
    {
      state_machine_impl::exec_state_exit<State, StateMachine> () (s, sm);
      invoke<State, StateMachine, Funcs...> (s, sm);
    }
  };


  // state transition.  named "trans" to reduce the amount of text in state
  // machine descriptions in user code.
  // derive from the predicate type to utilize empty base class optimization.
  template <class Predicate, class State> struct trans : private Predicate
  {
    typedef Predicate condition_func;
    typedef State     target_state;

    Predicate& condition (void) { return *this; }

    template <typename AllStates, unsigned int MaxStateSize,
	      typename StateMachine, typename CurrentState>
    bool check (CurrentState* cs, StateMachine& sm,
		state_machine_impl::state_buffer<MaxStateSize>& sb)
    {
      // locate the new state type in AllStates, instantiate it and return it.
      // if the target state that has been specified in this transition is
      // not present in the AllStates tuple, instantiate an error template.
      // this will usually show the state class name in the compiler error
      // message.
      typedef typename state_machine_impl::find_target_state<target_state, AllStates>::type new_state1;

      typedef typename std::conditional<std::is_void<new_state1>::value,
				        state_machine_impl::error_target_state_not_found<State>,
				        new_state1>::type new_state;

      if (!condition () (sm))
	return false;

    #ifdef STATE_MACHINE_IMPL_ENABLE_STATE_DOUBLE_BUFFER

      // invoke exit function of the current state and destruct it.
      cs->exit_destruct (sm);

      // construct new state instance of successor state type.
      auto* newst =
	new_state::template new_instance<AllStates, MaxStateSize,
					 StateMachine> (sb.swap_buffers ());

      typedef typename std::remove_pointer<decltype (newst)>::type newst_type;

      // invoke enter function of the new state instance.
      newst->enter (sm);

      // update state buffer
      sb.exec_func = &newst_type::exec;
      sb.name_str = newst_type::name_str ();

    #else // STATE_MACHINE_IMPL_ENABLE_STATE_DOUBLE_BUFFER

      cs->exit_destruct (sm);

      auto* newst =
	new_state::template new_instance<AllStates, MaxStateSize,
					 StateMachine> (sb.buffer ());

      typedef typename std::remove_pointer<decltype (newst)>::type newst_type;

      newst->enter (sm);
      sb.exec_func = &newst_type::exec;
      sb.name_str = newst_type::name_str ();

    #endif // STATE_MACHINE_IMPL_ENABLE_STATE_DOUBLE_BUFFER

      return true;
    }
  };

  template <class Predicate, typename... Funcs> struct do_if : private Predicate
  {
    typedef Predicate condition_func;

    Predicate& condition (void) { return *this; }

    template <typename State, typename StateMachine>
    void invoke (State& s, StateMachine& sm) { }

    template <typename State, typename StateMachine, typename F, typename... Fs>
    void invoke (State& s, StateMachine& sm)
    {
      F () (sm);
      invoke<State, StateMachine, Fs...> (s, sm);
    }

    template <typename AllStates, unsigned int MaxStateSize,
	      typename StateMachine, typename CurrentState>
    bool check (CurrentState* cs, StateMachine& sm,
		state_machine_impl::state_buffer<MaxStateSize>& sb)
    {
      if (condition () (sm))
	invoke<CurrentState, StateMachine, Funcs...> (*cs, sm);

      // the do-if is put into the transition list and thus has to return
      // whether it transitions or not.  obviously it never does.
      return false;
    }

    // if the do_if is placed into a list of functions, e.g. enter, exit list
    // those will be invoked via the operator ()

    template <typename StateMachine>
    void invoke_1 (StateMachine& sm) { }

    template <typename StateMachine, typename F, typename... Fs>
    void invoke_1 (StateMachine& sm)
    {
      F () (sm);
      invoke_1<StateMachine, Fs...> (sm);
    }

    template <typename StateMachine>
    void operator () (StateMachine& sm)
    {
      if (condition () (sm))
        invoke_1<StateMachine> (sm);
    }
  };

  template <class Predicate, typename... Funcs> struct do_if_once : private Predicate
  {
    typedef Predicate condition_func;
    bool done = false;

    Predicate& condition (void) { return *this; }

    template <typename State, typename StateMachine>
    void invoke (State& s, StateMachine& sm) { }

    template <typename State, typename StateMachine, typename F, typename... Fs>
    void invoke (State& s, StateMachine& sm)
    {
      F () (sm);
      invoke<State, StateMachine, Fs...> (s, sm);
    }

    template <typename AllStates, unsigned int MaxStateSize,
	      typename StateMachine, typename CurrentState>
    bool check (CurrentState* cs, StateMachine& sm,
		state_machine_impl::state_buffer<MaxStateSize>& sb)
    {
      if (!done && condition () (sm))
      {
	done = true;
	invoke<CurrentState, StateMachine, Funcs...> (*cs, sm);
      }

      // the do-if is put into the transition list and thus has to return
      // whether it transitions or not.  obviously it never does.
      return false;
    }

    // if the do_if_once is placed into a list of functions, e.g. enter, exit list
    // those will be invoked via the operator ()
    // when used in enter or exit function list, it's the same as do_if

    template <typename StateMachine>
    void invoke_1 (StateMachine& sm) { }

    template <typename StateMachine, typename F, typename... Fs>
    void invoke_1 (StateMachine& sm)
    {
      F () (sm);
      invoke_1<StateMachine, Fs...> (sm);
    }

    template <typename StateMachine>
    void operator () (StateMachine& sm)
    {
      if (condition () (sm))
        invoke_1<StateMachine> (sm);
    }
  };



  template <typename State, typename... Transitions>
  struct state
  {
    typedef state_machine_impl::state_actions<State, Transitions...> helper;

    typedef State state_type;

    template <typename StateMachine> using enter_func = typename helper::template enter_func<StateMachine>;
    template <typename StateMachine> using exit_func = typename helper::template exit_func<StateMachine>;
    using transitions = typename helper::transitions;

    template <typename AllStates, unsigned int MaxStateSize, typename StateMachine>
    static state_machine_impl::state_instance<StateMachine, State, AllStates, MaxStateSize,
					      enter_func<StateMachine>,
					      exit_func<StateMachine>, transitions>*
    new_instance (void* buf)
    {
      return new (buf) state_machine_impl::state_instance<StateMachine, State, AllStates, MaxStateSize,
							  enter_func<StateMachine>,
							  exit_func<StateMachine>, transitions> ();
    }

    template <typename AllStates, typename StateMachine>
    static constexpr unsigned int state_size (void)
    {
      // the MaxStateSize value is caluclated by this function.  thus we can't
      // recursively put it there.  set it to zero, as it doesn't have an
      // impact on the state size anyway.
      return sizeof (state_machine_impl::state_instance<StateMachine, State, AllStates, 0,
							enter_func<StateMachine>,
							exit_func<StateMachine>, transitions>);
    }

    template <typename AllStates, typename StateMachine>
    static constexpr unsigned int alignment (void)
    {
      return alignof (state_machine_impl::state_instance<StateMachine, State, AllStates, 0,
							 enter_func<StateMachine>,
							 exit_func<StateMachine>, transitions>);
    }

    template <typename AllStates, typename StateMachine>
    static constexpr bool is_empty (void)
    {
      return std::is_empty<state_machine_impl::state_instance<StateMachine, State, AllStates, 0,
							      enter_func<StateMachine>,
							      exit_func<StateMachine>, transitions>>::value;
    }

    template <typename AllStates, unsigned int MaxStateSize, typename StateMachine>
    static constexpr state_machine_impl::exec_func_t
    state_func (void)
    {
      return &state_machine_impl::state_instance<StateMachine, State, AllStates, MaxStateSize,
						 enter_func<StateMachine>,
						 exit_func<StateMachine>, transitions>::exec;
    }

    template <typename AllStates, unsigned int MaxStateSize, typename StateMachine>
    static constexpr const char*
    name_str (void)
    {
      return state_machine_impl::state_instance<StateMachine, State, AllStates, MaxStateSize,
						enter_func<StateMachine>,
						exit_func<StateMachine>, transitions>::name_str ();
    }
  };

  template <typename... States> using states = std::tuple<States...>;


  // some general purpose transition predicates
  struct always
  {
    template <typename T> bool operator () (T&) const { return true; }
  };

  template <typename A, typename B>
  struct cond_le : public A, public B
  {
    template <typename... Args> bool operator () (Args&&... args)
    {
      A& a = *this;
      B& b = *this;
      return a (std::forward<Args> (args)...) <= b (std::forward<Args> (args)...);
    }
  };

  template <typename Cond>
  struct cond_not : public Cond
  {
    template <typename... Args> bool operator () (Args&&... args)
    {
      Cond* c = this;
      return ! c->operator () (std::forward<Args> (args)...);
    }
  };

  template <typename... Cond>
  struct cond_and
  {
    typedef std::tuple<Cond...> conds_type;
    static constexpr unsigned int conds_size (void) { return sizeof...(Cond); }

    conds_type m_conds;

    template <unsigned int I = 0, typename... Args> 
    typename std::enable_if< I == conds_size (), bool>::type
    eval (Args&&... args)
    {
      // if we have reached the end of the tuple it means all conditions
      // have returned true so far.  to get a logical AND we return true.
      return true;
    }

    template <unsigned int I = 0, typename... Args> 
    typename std::enable_if< I < conds_size (), bool>::type
    eval (Args&&... args)
    {
      return std::get<I> (m_conds) (std::forward<Args> (args)...) == false
	     ? false
	     : eval<I + 1> (std::forward<Args> (args)...);
    }

    template <typename... Args> bool operator () (Args&&... args)
    {
      return eval<0> (std::forward<Args> (args)...);
    }
  };

  template <typename... Cond>
  struct cond_or
  {
    typedef std::tuple<Cond...> conds_type;
    static constexpr unsigned int conds_size (void) { return sizeof...(Cond); }

    conds_type m_conds;

    template <unsigned int I = 0, typename... Args> 
    typename std::enable_if< I == conds_size (), bool>::type
    eval (Args&&... args)
    {
      // if we have reached the end of the tuple it means all conditions
      // have returned false so far.  to get a logical OR we return false.
      return false;
    }

    template <unsigned int I = 0, typename... Args> 
    typename std::enable_if< I < conds_size (), bool>::type
    eval (Args&&... args)
    {
      return std::get<I> (m_conds) (std::forward<Args> (args)...) == true
	     ? true
	     : eval<I + 1> (std::forward<Args> (args)...);
    }

    template <typename... Args> bool operator () (Args&&... args)
    {
      return eval<0> (std::forward<Args> (args)...);
    }
  };


  // edge change conditions
  template < typename Cond, bool PrevVal, bool NewVal >
  struct edge_trigger : public Cond
  {
    // FIXME MCB-39: pass stm ref to constructor (optionally) and get the initial
    // value in the constructor.
    mutable int8_t prev_val = -1;

    template<typename... Args>
    bool operator () (Args&&... args) const
    {
      bool new_val = ((Cond&)*this) (std::forward<Args> (args)...);
      if (prev_val == -1)
      {
	prev_val = new_val;
	return false;
      }

      bool r = (prev_val == PrevVal) & (new_val == NewVal);
      prev_val = new_val;
      return r;
    }
  };

  template <typename Cond> using falling_edge = edge_trigger<Cond, true, false>; 
  template <typename Cond> using rising_edge = edge_trigger<Cond, false, true>; 

  template <typename Cond, typename TrueFunc, typename FalseFunc>
  struct if_then_else : public Cond, TrueFunc, FalseFunc
  {
    template <typename... Args>
    void operator () (Args&&... args) const
    {
      if (((Cond&)*this) (std::forward<Args> (args)...))
        ((TrueFunc&)*this) (std::forward<Args> (args)...);
      else
        ((FalseFunc&)*this) (std::forward<Args> (args)...);
    }
  };

  template <typename Cond, typename TrueFunc>
  struct if_then : public Cond, TrueFunc
  {
    template <typename... Args>
    void operator () (Args&&... args) const
    {
      if (((Cond&)*this) (std::forward<Args> (args)...))
        ((TrueFunc&)*this) (std::forward<Args> (args)...);
    }
  };

  // can't use std::integral_constant because its operator () does not accept
  // any arguments.
  template <unsigned int N> struct const_uint
  {
    template <typename... Args> auto operator () (Args&&...) const { return N; }
  };

  template <int N> struct const_int
  {
    template <typename... Args> auto operator () (Args...) const { return N; }
  };

  template <typename DurationType, typename ValueFunc> struct timeout : public ValueFunc
  {
    std::chrono::high_resolution_clock::time_point start_time =
	std::chrono::high_resolution_clock::now ();

    template <typename... Args>
    bool operator () (Args&&... args) const
    {
      return std::chrono::high_resolution_clock::now () - start_time
		> DurationType (((ValueFunc&)*this) (std::forward<Args> (args)...));
    }
  };

  template <typename ValueFunc> using timeout_ns = timeout<std::chrono::nanoseconds, ValueFunc>;
  template <typename ValueFunc> using timeout_us = timeout<std::chrono::microseconds, ValueFunc>;
  template <typename ValueFunc> using timeout_ms = timeout<std::chrono::milliseconds, ValueFunc>;
  template <typename ValueFunc> using timeout_s = timeout<std::chrono::seconds, ValueFunc>;
  template <typename ValueFunc> using timeout_m = timeout<std::chrono::minutes, ValueFunc>;
  template <typename ValueFunc> using timeout_h = timeout<std::chrono::hours, ValueFunc>;

}; // class state_machine


template <typename StateMachine>
class finite_state_machine : public StateMachine
{
public:
  typedef typename StateMachine::states_tuple states_tuple;

private:
  // FIXME: check if it is still needed with newer libstdc++.
  // probably it was fixed.
  static constexpr unsigned int max (unsigned int a, unsigned int b)
  {
    return a > b ? a : b;
  }

  template <unsigned int I = 0> static constexpr
  typename std::enable_if<I == std::tuple_size<states_tuple>::value,
			  unsigned int>::type
  calc_max_state_size (void)
  {
    return 0;
  }

  template <unsigned int I = 0> static constexpr
  typename std::enable_if<I < std::tuple_size<states_tuple>::value,
			  unsigned int>::type
  calc_max_state_size (void)
  {
    return max (std::tuple_element<I, states_tuple>::type
		:: template state_size<states_tuple, StateMachine> (),
                calc_max_state_size<I + 1> ());
  }

  template <unsigned int I = 0> static constexpr
  typename std::enable_if<I == std::tuple_size<states_tuple>::value,
			  unsigned int>::type
  calc_max_alignment (void)
  {
    return 0;
  }

  template <unsigned int I = 0> static constexpr
  typename std::enable_if<I < std::tuple_size<states_tuple>::value,
			  unsigned int>::type
  calc_max_alignment (void)
  {
    return max (std::tuple_element<I, states_tuple>::type
		:: template alignment<states_tuple, StateMachine> (),
                calc_max_alignment<I + 1> ());
  }

  template <unsigned int I = 0> static constexpr
  typename std::enable_if<I == std::tuple_size<states_tuple>::value,
			  bool>::type
  calc_all_empty (void)
  {
    return true;
  }

  template <unsigned int I = 0> static constexpr
  typename std::enable_if<I < std::tuple_size<states_tuple>::value,
			  bool>::type
  calc_all_empty (void)
  {
    return std::tuple_element<I, states_tuple>::type
		:: template is_empty<states_tuple, StateMachine> ()
	   && calc_all_empty<I + 1> ();
  }

public:
  // maximum alignment required for all the state types.
  static constexpr unsigned int max_state_alignment = calc_max_alignment ();

  // tells if all state types are empty.
  static constexpr bool all_states_empty = calc_all_empty ();

  // aligned maximum size of all possible state objects.
  // if all state types are empty we can pull this to zero.  this is required
  // to get truly zero-sized structs, because sizeof (x) will always be > zero
  // if it has some (non-static) members.  even if it actually has no variables.
  static constexpr unsigned int max_state_size =
    all_states_empty
    ? 0
    : (calc_max_state_size () + max_state_alignment - 1) & ~(max_state_alignment - 1);


  template <typename... Args>
  finite_state_machine (Args&&... args)
  : StateMachine (std::forward<Args> (args)...) { }

  ~finite_state_machine (void)
  {
    // exit and destruct the current state.
    if (likely (m_state_data.state.exec_func != nullptr))
      (m_state_data.state.exec_func)(m_state_data.state.buffer (),
				     (StateMachine*)this,
				     &m_state_data.state, true);
  }

  // execute one step of the state machine.
  // the current state's transition functions are invoked and the first
  // transition condition that is satisfied will transition to the according
  // state.  in this case, the old state's exit function is invoked and the
  // old state is destroyed.  the new state is constructed and the new state's
  // enter function is invoked.  to avoid infinite recursion, the new state's
  // exec function is not invoked.  it will be invoked by the next state machine
  // execution step.
  // if none of the transition conditions are satisfied the current state
  // remains active.
  
  void //__attribute__((noinline))
  exec (unsigned int max_transitions = std::numeric_limits<unsigned int>::max ())
  {
    // there are some cases where state machines might trigger each other by
    // posting commands and executing 1 step.  avoid re-entrant exec calls
    // as they might lead to infinite call loops.
    if (unlikely (m_in_exec))
      return;

    m_in_exec = true;
    StateMachine::exec ();

    if (likely (m_state_data.state.exec_func != nullptr))
    {
Lexec:
      for (unsigned int i = 0; i < max_transitions; ++i)
      {
	// for now abort at too many state transitions.  it's usually
	// an indicator for an infinite loop, which could happen if transition
	// conditions are broken.
	assert (i < 100);
	if (! (m_state_data.state.exec_func)(m_state_data.state.buffer (),
					     (StateMachine*)this,
					     &m_state_data.state, false))
	  break;
      }
    }
    else
    {
      // start first state
      auto* newst =
	std::tuple_element<0, states_tuple>::type
	:: template new_instance<states_tuple, max_state_size, StateMachine> (m_state_data.state.buffer ());

      newst->enter (*(StateMachine*)this);

      typedef typename std::remove_pointer<decltype (newst)>::type newst_type;

      m_state_data.state.exec_func = &newst_type::exec;
      m_state_data.state.name_str = newst_type::name_str ();

      --max_transitions;
      goto Lexec;
    }

     m_in_exec = false;
  }

  void //__attribute__((noinline))
  reset (void)
  {
    if (likely (m_state_data.state.exec_func != nullptr))
    {
      // force the current state to exit.
      (m_state_data.state.exec_func)(m_state_data.state.buffer (),
				     (StateMachine*)this,
				     &m_state_data.state, true);

      m_in_exec = false;
      m_state_data.state = decltype (m_state_data.state) ();
      exec (1);
    }
  }

  class state_desc
  {
  public:
    constexpr state_desc (void) : m_func (nullptr), m_name ("nullptr") { }
    constexpr state_desc (state_machine_impl::exec_func_t f, const char* n)
    : m_func (f), m_name (n) { }

    constexpr state_machine_impl::exec_func_t func (void) { return m_func; }

    const char* name (void) const { return m_name; }

    constexpr bool operator == (const state_desc& rhs) const { return m_func == rhs.m_func; }
    constexpr bool operator != (const state_desc& rhs) const { return m_func != rhs.m_func; }

  private:
    state_machine_impl::exec_func_t m_func;
    const char* m_name;
  };

  state_desc state (void) const
  {
    return { m_state_data.state.exec_func, m_state_data.state.name_str };
  }

  template <typename State> static constexpr state_desc state_for (void)
  {
    typedef typename state_machine_impl::find_target_state<State, states_tuple>::type state1;

    typedef typename std::conditional<std::is_void<state1>::value,
				      state_machine_impl::error_state_unreachable<State>,
				      state1>::type state2;

    return { state2::template state_func<states_tuple, max_state_size, StateMachine> (),
	     state2::template name_str<states_tuple, max_state_size, StateMachine> () };
  }

private:
  volatile bool m_in_exec = false;

  struct data
  {
    state_machine_impl::state_buffer<max_state_size> state;

  } m_state_data;
};

#endif // includeguard_state_machine_hpp_includeguard
