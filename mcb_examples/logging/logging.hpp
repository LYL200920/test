/*

logging utility

the following log levels are supported
  error
  warn
  info
  debug
  trace

in a source code file, before including this file, define the log levels that
should be used.  if a log level is not defined, the output for that (undefined)
log level will be redirected to nowhere (at compile time).  otherwise the
loglevel will be checked at runtime and if enabled the log message will be
written.

*/

#ifndef includeguard_logging_hpp_includeguard
#define includeguard_logging_hpp_includeguard
#ifdef __cplusplus

#include <cstdio>

class log_level
{
public:
  enum value_type
  {
    // critical errors
    // e.g. exceptions, totally unexpected state
    error = 1 << 0,

    // warnings
    // e.g. recovered from unexpected state.
    warn  = 1 << 1,

    // info messages
    // e.g. session/connection open, close
    info  = 1 << 2,

    // debugging messages
    debug = 1 << 3,

    // very verbose messages
    // e.g. individual bytes sent/received
    trace = 1 << 4,

    all = trace | debug | info | warn | error
  };

  // sometimes it's useful to disable logging temporarily, e.g. in interrupt
  // handler contexts.  when temporarily disabling the log levels, we write a
  // all-zero mask and store the mask.  when the saved_enable_mask object goes
  // out of scope it automatically restores the previously set mask bits.
  // to allow move semantics the mask is used as an OR mask.
#if defined (LOGGING_LOGGING_ENABLE)
  struct saved_enable_mask
  {
    char or_mask;

    constexpr saved_enable_mask (void) = delete;

    constexpr saved_enable_mask (char val) : or_mask (val) { }

    constexpr saved_enable_mask (const saved_enable_mask&) = delete;

    constexpr saved_enable_mask (saved_enable_mask&& rhs)
    : or_mask (rhs.or_mask)
    {
      // clear the or mask in rhs so that rhs's destructor won't re-enable
      // the logging.
      rhs.or_mask = 0;
    }

    saved_enable_mask& operator = (const saved_enable_mask&) = delete;

    saved_enable_mask& operator = (saved_enable_mask&& rhs)
    {
      or_mask = rhs.or_mask;
      rhs.or_mask = 0;
      return *this;
    }

    ~saved_enable_mask (void) { g_enabled_mask = or_mask; }
  };

  static bool enabled (value_type v) { return (g_enabled_mask & v) != 0; }

  static void enable (value_type v, bool en = true)
  {
    g_enabled_mask = (g_enabled_mask & ~v) | (en ? v : 0);
  }

  static void disable (value_type v) { enable (v, false); }

  static saved_enable_mask save_disable (void)
  {
    return { g_enabled_mask };
  }

#else
  struct saved_enable_mask
  {
    constexpr saved_enable_mask (void) = delete;
    constexpr saved_enable_mask (char val) { }
    constexpr saved_enable_mask (const saved_enable_mask&) = delete;
    constexpr saved_enable_mask (saved_enable_mask&&) { }
    saved_enable_mask& operator = (const saved_enable_mask&) = delete;
    saved_enable_mask& operator = (saved_enable_mask&&) { return *this; }
  };

  static saved_enable_mask save_disable (void) { return { 0 }; }

  static bool enabled (value_type) { return false; }
  static void enable (value_type, bool en = true) { }
  static void disable (value_type) { }
#endif

private:
  static char g_enabled_mask;
};

struct logging_hook
{
  virtual void logging_write (const void* ptr, size_t len) = 0;
};

#if defined (LOGGING_LOGGING_ENABLE)

bool attach_logging_hook (logging_hook*);
void detach_logging_hook (logging_hook*);

#else

inline bool attach_logging_hook (logging_hook*) { return false; }
inline void detach_logging_hook (logging_hook*) { }

#endif

#ifdef log_all
  #undef log_trace
  #define log_trace

  #undef log_debug
  #define log_debug

  #undef log_info
  #define log_info

  #undef log_warn
  #define log_warn

  #undef log_error
  #define log_error
#endif

#define if_level_level_enabled(lev) if (__builtin_expect (log_level::enabled (log_level::lev), 0))

#if !defined (log_trace) || !defined (LOGGING_LOGGING_ENABLE)
  #undef log_trace
  #define log_trace(...) do { } while (0)
  #define do_if_log_trace(...) do { } while (0)
#else
  #undef log_trace
  #define log_trace(...) do { if_level_level_enabled (trace) printf (__VA_ARGS__); } while (0)
  #define do_if_log_trace(expr) do { if_level_level_enabled (trace) do { expr; } while (0); } while (0)
#endif

#if !defined (log_debug) || !defined (LOGGING_LOGGING_ENABLE)
  #undef log_debug
  #define log_debug(...) do { } while (0)
  #define do_if_log_debug(...) do { } while (0)
#else
  #undef log_debug
  #define log_debug(...) do { if_level_level_enabled (debug) printf (__VA_ARGS__); } while (0)
  #define do_if_log_debug(expr) do { if_level_level_enabled (debug) do { expr; } while (0); } while (0)
#endif

#if !defined (log_info) || !defined (LOGGING_LOGGING_ENABLE)
  #undef log_info
  #define log_info(...) do { } while (0)
  #define do_if_log_info(...) do { } while (0)
#else
  #undef log_info
  #define log_info(...) do { if_level_level_enabled (info) printf (__VA_ARGS__); } while (0)
  #define do_if_log_info(expr) do { if_level_level_enabled (info) do { expr; } while (0); } while (0)
#endif

#if !defined (log_warn) || !defined (LOGGING_LOGGING_ENABLE)
  #undef log_warn
  #define log_warn(...) do { } while (0)
  #define do_if_log_warn(...) do { } while (0)
#else
  #undef log_warn
  #define log_warn(...) do { if_level_level_enabled (warn) printf (__VA_ARGS__); } while (0)
  #define do_if_log_warn(expr) do { if_level_level_enabled (warn) do { expr; } while (0); } while (0)
#endif

#if !defined (log_error) || !defined (LOGGING_LOGGING_ENABLE)
  #undef log_error
  #define log_error(...) do { } while (0)
  #define do_if_log_error(...) do { } while (0)
#else
  #undef log_error
  #define log_error(...) do { if_level_level_enabled (error) printf (__VA_ARGS__); } while (0)
  #define do_if_log_error(expr) do { if_level_level_enabled (error) do { expr; } while (0); } while (0)
#endif


#endif // __cplusplus
#endif // includeguard_logging_hpp_includeguard
