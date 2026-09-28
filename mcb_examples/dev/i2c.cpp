
#include <algorithm>
#include <type_traits>

#include "dev/i2c.hpp"
#include "utils/rodata.hpp"

namespace dev
{
namespace i2c_manufacturer_id
{

typedef utils::str_table_entry<unsigned int> str_desc;

// note: because of the trailing comma of the x-macro expansion, put the
// expanded elements into an additional { }
static constexpr auto str_table = utils::sort_array (utils::make_array ({
  #define expand_i2c_manufacturer_id(enum_name, int_val, str_val) str_desc { int_val, utils::conststr_string_view (str_val) } ,
    #include "dev/i2c_manufacturer_id.x.hpp"
}));

inline std::string_view to_string (unsigned int id)
{
  auto i = std::lower_bound (std::begin (str_table), std::end (str_table), id);

  if (i != std::end (str_table) && *i == str_desc (id))
    return *i;

  return { };
}

} // i2c_manufacturer_id
} // namespace dev
