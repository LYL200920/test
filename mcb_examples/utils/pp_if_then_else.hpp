#ifndef includeguard_pp_if_then_else_hpp_includeguard
#define includeguard_pp_if_then_else_hpp_includeguard

#include "pp_utils.hpp"
#include "pp_arith.hpp"

#define pp_if_then_0(expr, ...)
#define pp_if_then_1(expr, ...) expr (__VA_ARGS__)

// if cond is '1' evaluate to expr (args)
// otherwise evaluate to nothing
#define pp_if_then(cond, expr, ...) pp_concat(pp_if_then_, cond) (expr, __VA_ARGS__)

// if cond is '1Å' evaluate to expr_a (args).
// if cond is '0' evaluate to expr_b (args).
#define pp_if_then_else(cond,expr_a,expr_b,...) \
  pp_if_then(cond,expr_a,__VA_ARGS__) \
  pp_if_then(pp_arith_lnot (cond),expr_b,__VA_ARGS__)

#endif // includeguard_pp_if_then_else_hpp_includeguard