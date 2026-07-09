
/*
a generator for the pp_args_size macro, which expands to the number of
arguments in the __VA_ARGS__

how it works:

#define VA_NUM_ARGS(...) VA_NUM_ARGS_IMPL(__VA_ARGS__, 5,4,3,2,1,0)

#define VA_NUM_ARGS_IMPL(_1,_2,_3,_4,_5,N,...) N

VA_NUM_ARGS(x,y,z)

VA_NUMARGS_IMPL(x,y,x, 5,4,3,2,1,0)

-> VA_NUM_ARGS_IMPL(_1, _2, _3, _4, _5, N,...) N
                     x   y   z   5   4  3


VA_NUM_ARGS_IMPL(_1, _2, _3, _4, _5, N,...) N
                  5   4   3   2   1  0


some special handling is needed for the case of zero arguments, where
the __VA_ARGS__ expands to a comma character.
an alternative would be to use -std=gnu++11 and ##__VA_ARGS__

see also
http://stackoverflow.com/questions/11317474/macro-to-count-number-of-arguments

*/

#include <iostream>
#include <cstdlib>
#include <cassert>

int main (int argc, const char** argv)
{
  assert (argc == 2);
  int count = std::atoi (argv[1]);

  std::cout << "#ifndef include_guard_pp_args_hpp_include_guard" << std::endl
	    << "#define include_guard_pp_args_hpp_include_guard" << std::endl
	    << std::endl;


  // pp_args_size
  // evaluates to a positive decimal literal which is the number of va args
  // passed to pp_args_size.

  std::cout << "#define pp_args_size_arg_n(";
  for (int i = 1; i < count; ++i)
    std::cout << "a" << i << ",";
  std::cout << "N,...) N" << std::endl;


  std::cout << "#define pp_args_rseq() ";
  for (int i = count - 1; i > 0; --i)
    std::cout << i << ',';
  std::cout << "0" << std::endl;

  std::cout << "#define pp_args_cseq() ";
  for (int i = 0; i < count - 2; ++i)
    std::cout << "1,";
  std::cout << "0,0" << std::endl;

  std::cout << "#define pp_args_comma()    ," << std::endl;

  // MSVC needs an indirection here to work properly.
  // it also works on GCC that way, so no need to check the compiler.
  std::cout << "#define pp_args_expand(x) x" << std::endl;
  std::cout << "#define pp_args_(...) pp_args_expand (pp_args_size_arg_n(__VA_ARGS__))" << std::endl;

  std::cout << "#define pp_args_has_comma(...) pp_args_(__VA_ARGS__, pp_args_cseq())" << std::endl;
  
  std::cout << "#define pp_args_size_impl1(a,b,N) pp_args_size_impl2(a,b,N)" << std::endl;
  std::cout << "#define pp_args_size_impl2(a,b,N) pp_args_size_impl3_ ## a ## b(N)" << std::endl;
  std::cout << "#define pp_args_size_impl3_01(N) 0" << std::endl;
  std::cout << "#define pp_args_size_impl3_00(N) 1" << std::endl;
  std::cout << "#define pp_args_size_impl3_11(N) N" << std::endl;

  std::cout << std::endl;

  std::cout << "#define pp_args_size(...) "
	       "pp_args_size_impl1 (pp_args_has_comma(__VA_ARGS__),"
	                      "pp_args_has_comma(pp_args_comma __VA_ARGS__ ()),"
			      "pp_args_(__VA_ARGS__, pp_args_rseq ()))"
	    << std::endl;

  std::cout << std::endl;

  // pp_args_nth
  // evaluates to the n-th argument of the va args passed to pp_args_nth.

#define pp_arith_seq_narg_0(x,...) x
#define pp_arith_seq_narg_1(a0,x,...) x
#define pp_arith_seq_narg_2(a0,a1,x,...) x
#define pp_arith_seq_narg_3(a0,a1,a2,x,...) x
#define pp_arith_seq_narg_4(a0,a1,a2,a3,x,...) x
#define pp_arith_seq_narg_5(a0,a1,a2,a3,a4,x,...) x

#define pp_arith_seq_narg1(n,...) pp_arith_seq_narg_ ## n (__VA_ARGS__)
#define pp_arith_seq_narg(n,...) pp_arith_seq_narg1(n,__VA_ARGS__)

  for (int i = 0; i < count; ++i)
  {
    std::cout << "#define pp_args_nth_" << i << "(";
    for (int j = 0; j < i; ++j)
      std::cout << "a" << j << ",";
    std::cout << "x,...) x" << std::endl;
  }


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

#define pp_args_nth2(n, args) n args
#define pp_args_nth1(n, ...) pp_args_nth2 (pp_concat (pp_args_nth_, n),  (__VA_ARGS__))
#define pp_args_nth(n, ...) pp_args_nth1(n, __VA_ARGS__)

#else // _MSC_VER

#define pp_args_nth1(n, ...) pp_args_nth_ ## n (__VA_ARGS__)
#define pp_args_nth(n, ...) pp_args_nth1(n, __VA_ARGS__)

#endif // _MSC_VER


  // built-in tests

static_assert (pp_args_size(a) == 1);
static_assert (pp_args_size() == 0);
static_assert (pp_args_size(a, a, a) == 3);

static_assert (pp_args_nth (0, 1, 2, 3) == 1);
static_assert (pp_args_nth (1, 1, 2, 3) == 2);
static_assert (pp_args_nth (2, 1, 2, 3) == 3);


  )" << std::endl;

  std::cout << std::endl;
  std::cout << "#endif // include_guard_pp_args_hpp_include_guard" << std::endl;
  return 0;
}
