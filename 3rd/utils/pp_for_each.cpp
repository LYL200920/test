
/*

a generator for the pp_for_each macro.

*/

#include <iostream>
#include <cstdlib>
#include <cassert>

void make_pp_for_each_n (int n)
{
  std::cout << "#define pp_for_each_" << n << "(e";

  for (int i = 0; i < n; ++i)
    std::cout << ",a" << i;

  if (n == 1)
    std::cout << ") e (a0)" << std::endl;
  else
  {
    std::cout << ") pp_for_each_1(e,a0) pp_for_each_" << (n-1) << "(e";

    for (int i = 1; i < n; ++i)
      std::cout << ",a" << i;

    std::cout << ")" << std::endl;
  }
}

void make_pp_for_each_n_e0 (int n)
{
  std::cout << "#define pp_for_each_i_" << n << "(e,e0";

  for (int i = 0; i < n; ++i)
    std::cout << ",a" << i;

  if (n == 1)
    std::cout << ") e (e0,a0)" << std::endl;
  else
  {
    std::cout << ") pp_for_each_i_1(e,e0,a0) pp_for_each_i_" << (n-1) << "(e,e0";

    for (int i = 1; i < n; ++i)
      std::cout << ",a" << i;

    std::cout << ")" << std::endl;
  }
}

int main (int argc, const char** argv)
{
  assert (argc == 2);

  std::cout << "#ifndef include_guard_pp_for_each_hpp_include_guard" << std::endl
	    << "#define include_guard_pp_for_each_hpp_include_guard" << std::endl
	    << std::endl
	    << "#include \"utils/pp_utils.hpp\"" << std::endl
	    << "#include \"utils/pp_args.hpp\"" << std::endl
	    << std::endl;

  const int count = std::atoi (argv[1]);

  for (int i = 1; i <= count; ++i)
    make_pp_for_each_n (i);

  std::cout << std::endl;

  for (int i = 1; i <= count; ++i)
    make_pp_for_each_n_e0 (i);

  // MSVC expands VA_ARGS differently than GCC or clang.
  // For example:
  //    #define test_1(a, b, c) a, b, c
  //    #define test(...) test_1 (__VA_ARGS__)
  //
  //    test (hello,from,earth)
  //
  //    will expand to
  //       hello,from,earth,,
  //
  //    because all __VA_ARGS__ end up being passed as the first arg 'a' for
  //    macro test_1 and no level of indirection/nesting will help this.
  //    the only thing that seems to work on MSVC is to capture the __VA_ARGS__
  //    in parens, like '(__VA_ARGS__)' and use that to synthesize a macro
  //    invocation:
  //
  //    #define test_1(a, b, c) a, b, c
  //    #define test_2(args) test_1 args
  //    #define test(...) test_2 ((__VA_ARGS__))
  //
  //    test (hello,from,earth)
  //
  //    will expand as:
  //       test (hello,from,earth)
  //         test_2 ((hello,from,earth))
  //           test_1 (hello,from,earth)
  //             hello,from,earth

  std::cout << R"(

#if defined (_MSC_VER) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL)

#define pp_for_each_prefix_args_1(e,...) (e,__VA_ARGS__)
#define pp_for_each_prefix_args_2(e,e0,...) (e,e0,__VA_ARGS__)
#define pp_for_each_expand_args(...) __VA_ARGS__

#define pp_for_each_(e, M, ARGS) M pp_for_each_prefix_args_1 (e, pp_for_each_expand_args ARGS)
#define pp_for_each(expander,...) pp_for_each_(expander, pp_concat(pp_for_each_, pp_args_size(__VA_ARGS__)), (__VA_ARGS__))

#define pp_for_each__i(e, e0, M, ARGS) M pp_for_each_prefix_args_2 (e, e0, pp_for_each_expand_args ARGS)
#define pp_for_each_i(expander,e0, ...) pp_for_each__i(expander, e0, pp_concat(pp_for_each_i_, pp_args_size(__VA_ARGS__)), (__VA_ARGS__))

#else // _MSC_VER

#define pp_for_each_(e, M, ...) M(e, __VA_ARGS__)
#define pp_for_each(expander,...) pp_for_each_(expander, pp_concat(pp_for_each_, pp_args_size(__VA_ARGS__)), __VA_ARGS__)

#define pp_for_each__i(e, e0, M, ...) M(e, e0, __VA_ARGS__)
#define pp_for_each_i(expander,e0, ...) pp_for_each__i(expander, e0, pp_concat(pp_for_each_i_, pp_args_size(__VA_ARGS__)), __VA_ARGS__)

#endif // _MSC_VER
#endif // include_guard_pp_for_each_hpp_include_guard
)" << std::endl;

  return 0;
}
