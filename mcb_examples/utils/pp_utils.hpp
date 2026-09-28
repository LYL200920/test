#ifndef includeguard_pp_utils_hpp_includeguard
#define includeguard_pp_utils_hpp_includeguard

#define pp_concat_1(a,b) a ## b
#define pp_concat(a,b) pp_concat_1(a,b)

#define pp_quote_1(x) # x
#define pp_quote(x) pp_quote_1(x)

#endif // includeguard_pp_utils_hpp_includeguard