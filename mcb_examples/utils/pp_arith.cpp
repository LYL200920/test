/*

provide some preprocessor arithmetic on number literals.  the numbers must
be between 0 and the max value (usually 255)

*/

#include <iostream>
#include <cstdlib>
#include <cassert>

int main (int argc, const char** argv)
{
  assert (argc == 2);
  int count = std::atoi (argv[1]);

  std::cout << R"(
#ifndef include_guard_pp_arith_hpp_include_guard
#define include_guard_pp_arith_hpp_include_guard

#include "utils/pp_args.hpp"
  )" << std::endl;

  // truth table for x == y
  // where y is the index in the argument list (column) and x is the
  // macro number (row)
  for (int i = 0; i < count; ++i)
  {
    std::cout << "#define pp_arith_seq_eq_" << i << "() ";
    for (int j = 0; j < count; ++j)
    {
      if (j > 0)
	std::cout << ',';

      std::cout << (i == j ? '1' : '0');
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;

  // truth table for x < y
  // where y is the index in the argument list (column) and x is the
  // macro number (row)
  for (int i = 0; i < count; ++i)
  {
    std::cout << "#define pp_arith_seq_lt_" << i << "() ";
    for (int j = 0; j < count; ++j)
    {
      if (j > 0)
	std::cout << ',';

      std::cout << (i < j ? '1' : '0');
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;

  // pp_arith_lnot (x)
  // logical not
  // evaluate to '1' if x is '0'.
  // evaluate to '0' if x is '1'.
  // if x is neither '1' nor '0' the result is undefined.
  std::cout << R"(

#if defined (_MSC_VER) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL)

#define pp_arith_lnot_0() 1
#define pp_arith_lnot_1() 0
#define pp_arith_lnot2(x,y) x ## y
#define pp_arith_lnot1(x) pp_arith_lnot2 (pp_arith_lnot_, x) ()
#define pp_arith_lnot(x) pp_arith_lnot1(x)

#else // _MSC_VER

#define pp_arith_lnot_0 1
#define pp_arith_lnot_1 0
#define pp_arith_lnot1(x) pp_arith_lnot_ ## x
#define pp_arith_lnot(x) pp_arith_lnot1(x)

#endif // _MSC_VER
  )" << std::endl;

  // pp_arith_lor (x, y)
  // logical or
  // evaluates to..
  // (x, y) = (0, 0) -> 0
  // (x, y) = (0, 1) -> 1
  // (x, y) = (1, 0) -> 1
  // (x, y) = (1, 1) -> 1
  // if x and y are neither '1' nor '0' the result is undefined.
  std::cout << R"(

#if defined (_MSC_VER) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL)

#define pp_arith_lor_00() 0
#define pp_arith_lor_01() 1
#define pp_arith_lor_10() 1
#define pp_arith_lor_11() 1
#define pp_arith_lor2(m, x,y) m ## x ## y
#define pp_arith_lor1(x,y) pp_arith_lor2 (pp_arith_lor_, x, y)()
#define pp_arith_lor(x,y) pp_arith_lor1(x,y)

#else // _MSC_VER

#define pp_arith_lor_00 0
#define pp_arith_lor_01 1
#define pp_arith_lor_10 1
#define pp_arith_lor_11 1
#define pp_arith_lor1(x,y) pp_arith_lor_ ## x ## y
#define pp_arith_lor(x,y) pp_arith_lor1(x,y)

#endif // _MSC_VER
  )" << std::endl;

  // pp_arith_land (x, y)
  // logical and
  // evaluates to..
  // (x, y) = (0, 0) -> 0
  // (x, y) = (0, 1) -> 0
  // (x, y) = (1, 0) -> 0
  // (x, y) = (1, 1) -> 1
  // if x and y are neither '1' nor '0' the result is undefined.
  std::cout << R"(

#if defined (_MSC_VER) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL)

#define pp_arith_land_00() 0
#define pp_arith_land_01() 0
#define pp_arith_land_10() 0
#define pp_arith_land_11() 1
#define pp_arith_land2(m,x,y) m ## x ## y
#define pp_arith_land1(x,y) pp_arith_land2 (pp_arith_land_, x, y)()
#define pp_arith_land(x,y) pp_arith_land1(x,y)

#else // _MSC_VER

#define pp_arith_land_00 0
#define pp_arith_land_01 0
#define pp_arith_land_10 0
#define pp_arith_land_11 1
#define pp_arith_land1(x,y) pp_arith_land_ ## x ## y
#define pp_arith_land(x,y) pp_arith_land1(x,y)

#endif // _MSC_VER
  )" << std::endl;

  // pp_arith_eq (x, y)
  // evaluates to '1' if x is EQual to y, '0' otherwise.
  std::cout << R"(
#define pp_arith_eq1(y,...) pp_args_nth (y, __VA_ARGS__)
#define pp_arith_eq(x,y) pp_arith_eq1(y, pp_concat (pp_arith_seq_eq_, x) ())
  )" << std::endl;

  // pp_arith_lt (x, y)
  // evaluates to '1' if x is Less Than y, '0' otherwise.
  std::cout << R"(
#define pp_arith_lt1(y,...) pp_args_nth (y, __VA_ARGS__)
#define pp_arith_lt(x,y) pp_arith_lt1(y, pp_concat (pp_arith_seq_lt_, x) ())
  )" << std::endl;

  // pp_arith_ne (x, y)
  // evaluates to '1' if x is Not Equal to y, '0' otherwise.
  std::cout << R"(
#define pp_arith_ne(x,y) pp_arith_lnot(pp_arith_eq (x,y))
  )" << std::endl;

  // pp_arith_le (x, y)
  // evaluates to '1' if x is Less than or Equal to y, '0' otherwise.
  std::cout << R"(
#define pp_arith_le(x,y) pp_arith_lor(pp_arith_lt(x,y), pp_arith_eq(x,y))
  )" << std::endl;

  // pp_arith_gt (x, y)
  // evaluates to '1' if x is Graeater Than y, '0' otherwise.
  std::cout << R"(
#define pp_arith_gt(x,y) pp_arith_lt(y,x)
  )" << std::endl;
 
  // pp_arith_ge (x, y)
  // evaluates to '1' if x is Greater than or Equal to y, '0' otherwise
  std::cout << R"(
#define pp_arith_ge(x,y) pp_arith_le(y,x)
  )" << std::endl;

  // built-in tests
  std::cout << R"(


static_assert (pp_arith_lnot (0) == 1);
static_assert (pp_arith_lnot (1) == 0);

static_assert (pp_arith_lor (0, 0) == 0);
static_assert (pp_arith_lor (0, 1) == 1);
static_assert (pp_arith_lor (1, 0) == 1);
static_assert (pp_arith_lor (1, 1) == 1);

static_assert (pp_arith_land (0, 0) == 0);
static_assert (pp_arith_land (0, 1) == 0);
static_assert (pp_arith_land (1, 0) == 0);
static_assert (pp_arith_land (1, 1) == 1);

static_assert (pp_arith_eq (0, 0) == 1);
static_assert (pp_arith_eq (1, 0) == 0);
static_assert (pp_arith_eq (5, 4) == 0);
static_assert (pp_arith_eq (50, 40) == 0);

static_assert (pp_arith_ne (0, 0) == 0);
static_assert (pp_arith_ne (1, 0) == 1);
static_assert (pp_arith_ne (5, 4) == 1);
static_assert (pp_arith_ne (50, 40) == 1);

static_assert (pp_arith_lor (pp_arith_ne (0, 0), 1) == 1);
static_assert (pp_arith_lor (pp_arith_ne (0, 0), 1) == 1);

static_assert (pp_arith_land (pp_arith_lor (pp_arith_ne (0, 0), 1), 1) == 1);

static_assert (pp_arith_lt (5, 4) == 0);
static_assert (pp_arith_eq (pp_arith_lt (5, 4), 0) == 1);

static_assert (pp_arith_ne (pp_arith_ne (40, 30), 0) == 1);

static_assert (pp_arith_le (4, 5) == 1);
static_assert (pp_arith_eq (pp_arith_le (4, 5), 1) == 1);

static_assert (pp_arith_gt (5, 4) == 1);
static_assert (pp_arith_eq (pp_arith_gt (5, 4), 1) == 1);

static_assert (pp_arith_ge (5, 5) == 1);
static_assert (pp_arith_eq (pp_arith_ge (5, 5), 1) == 1);

#define test_pp_arith_op_eq ==
#define test_pp_arith_op_ne !=
#define test_pp_arith_op_lt <
#define test_pp_arith_op_le <=
#define test_pp_arith_op_gt >
#define test_pp_arith_op_ge >=

#define test_pp_arith_2(op, x, y) static_assert (pp_arith_ ## op (x, y) == (x test_pp_arith_op_ ## op y), "")

#define test_pp_arith_1(op) \
  test_pp_arith_2 (op, 0, 0); \
  test_pp_arith_2 (op, 0, 1); \
  test_pp_arith_2 (op, 1, 0); \
  test_pp_arith_2 (op, 1, 1); \
  test_pp_arith_2 (op, 2, 2); \
  test_pp_arith_2 (op, 2, 1); \
  test_pp_arith_2 (op, 1, 2); \
  test_pp_arith_2 (op, 4, 2); \
  test_pp_arith_2 (op, 4, 0); \
  test_pp_arith_2 (op, 0, 4);

test_pp_arith_1 (eq);
test_pp_arith_1 (ne);
test_pp_arith_1 (lt);
test_pp_arith_1 (le);
test_pp_arith_1 (gt);
test_pp_arith_1 (ge);

#undef test_pp_arith_op_eq
#undef test_pp_arith_op_ne
#undef test_pp_arith_op_lt
#undef test_pp_arith_op_le
#undef test_pp_arith_op_gt
#undef test_pp_arith_op_ge
#undef test_pp_arith_1
#undef test_pp_arith_2

  )" << std::endl;

  std::cout << std::endl;
  std::cout << "#endif // include_guard_pp_arith_hpp_include_guard" << std::endl;
  return 0;
}
