
#ifndef includeboard_board_info_mcb_v13_includeguard
#define includeboard_board_info_mcb_v13_includeguard

#include <string>
#include <experimental/string_view>

#include "../../board_info.hpp"
#include <utils/enum_bitset.hpp>
#include <utils/rodata.hpp>

class mcb_v13_board_info : public board_info
{
public:
  static const mcb_v13_board_info& inst (void);

  enum feature_bit
  {
    #define expand_feature_bit(enum_num, enum_name, str_name, desctext) enum_name = enum_num,
      #include "board_info_feature_bit.x.hpp"

    feature_bit_count,
    feature_bit_max_val = feature_bit_count-1
  };

  static_assert (feature_bit_count <= board_info::feature_bits::bit_count, "");

  class feature_bits : public utils::enum_bitset < feature_bit, dac124s085, feature_bit_max_val,
						   board_info::feature_bits >
  {
  public:
    const std::array<uint8_t, board_info::feature_bits::bit_count / 8>& bytes (void) const
    {
      return ((const board_info::feature_bits*)this)->bytes ();
    }

  private:
  };

  const feature_bits& features (void) const { return (const feature_bits&)(m_features); }

  feature_bits all_features (void) const
  {
    feature_bits r;
    r.set ();
    return r;
  }

  static unsigned int feature_num (feature_bit b)
  {
    return (unsigned int)b;
  }

  static std::experimental::string_view feature_name (feature_bit b);
  static std::experimental::string_view feature_desc (feature_bit b);

private:
};


// N.B. we'd like to have only one instance of the string tables.  usually
// we'd put that into a .cpp file and have it in a lib.  this here is a
// header-only library and the only standard way of getting only one instance
// is to use static variables in template classes.

template < int I > struct mcb_v13_board_info_str_feature_name_table
{
  typedef utils::str_table_entry<mcb_v13_board_info::feature_bit> str_desc;

  static constexpr auto table =
    utils::convert_array<str_desc, std::experimental::string_view> (utils::sort_array (utils::make_array ({

    #define expand_feature_bit(enum_num, enum_name, str_name, desctext) \
	str_desc { mcb_v13_board_info::enum_name, utils::conststr_string_view (str_name) },

      #include "board_info_feature_bit.x.hpp"
    })));
};

template < int I > constexpr decltype (mcb_v13_board_info_str_feature_name_table<I>::table)
mcb_v13_board_info_str_feature_name_table<I>::table;

inline std::experimental::string_view mcb_v13_board_info::feature_name (feature_bit b)
{
  return mcb_v13_board_info_str_feature_name_table<0>::table[b];
}


template < int I > struct mcb_v13_board_info_str_feature_desc_table
{
  typedef utils::str_table_entry<mcb_v13_board_info::feature_bit> str_desc;

  static constexpr auto table =
    utils::convert_array<str_desc, std::experimental::string_view> (utils::sort_array (utils::make_array ({

    #define expand_feature_bit(enum_num, enum_name, str_name, desctext) \
	str_desc { mcb_v13_board_info::enum_name, utils::conststr_string_view (desctext) },

      #include "board_info_feature_bit.x.hpp"
    })));
};

template < int I > constexpr decltype (mcb_v13_board_info_str_feature_desc_table<I>::table)
mcb_v13_board_info_str_feature_desc_table<I>::table;

inline std::experimental::string_view mcb_v13_board_info::feature_desc (feature_bit b)
{
  return mcb_v13_board_info_str_feature_desc_table<0>::table[b];
}


namespace this_board
{
  using board_info_type = mcb_v13_board_info;

  inline const board_info_type& board_info (void) { return board_info_type::inst (); }
}

#endif // includeboard_board_info_mcb_v13_includeguard
