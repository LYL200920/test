
#ifndef includeguard_utils_langcomp_hpp_includeguard
#define includeguard_utils_langcomp_hpp_includeguard

#include <cassert>

#define assert_unreachable() do { assert ("unreachable" && 0); } while (1)

#ifdef _MSC_VER

  #define likely(expr) (expr)
  #define unlikely(expr) (expr)

#else

  #define likely(expr) __builtin_expect (expr, 1)
  #define unlikely(expr) __builtin_expect (expr, 0)

#endif

#define countof(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))


#define linenum_prefix_quote_1(x) #x
#define linenum_prefix_quote(x) linenum_prefix_quote_1(x)
#define linenum_prefix "#line " linenum_prefix_quote(__LINE__) 

// officially, GLSL does not support source file names after #line, as C.
// it works though at least for the nvidia GL / CL compilers.
// doesn't work for vivante GL compiler.
//#define linenum_prefix "#line " linenum_prefix_quote(__LINE__) " " linenum_prefix_quote (__FILE__)


#ifdef _MSC_VER

#endif


#ifdef __GNUC__
#define soft_memory_fence() do { asm volatile ("" : : : "memory"); } while (false)
#endif


#ifdef __GNUC__
#define unreachable __builtin_unreachable()
#endif

#ifdef _MSC_VER
#define unreachable __assume(0)
#endif

#define assume_always_true(expr) if (!(expr)) do { unreachable; } while (false)
#define assume_always_false(expr) if (expr) do { unreachable; } while (false)

#endif // includeguard_utils_langcomp_hpp_includeguard
