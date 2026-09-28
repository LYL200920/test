#ifndef includeguard_nld_bsp_hash_crc_includeguard
#define includeguard_nld_bsp_hash_crc_includeguard

#include "../../../hash/crc.hpp"

#ifndef BOARD_NO_CRC

#include <board/board.hpp>

namespace hash
{
namespace crc_impl_detail
{

template <>
inline uint32_t core<32, 0x04C11DB7, false, false>
  ::process_bytes (uint32_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& crc = this_board::inst ().crc;

  crc.set_reset_init_value (rem);
  crc.set_control (dev::stm32f0_crc::control_t ()
	.set_input_bitorder (dev::stm32f0_crc::input_non_reversed)
	.set_output_bitorder (dev::stm32f0_crc::output_non_reversed)
	.set_reset ());

  for (; in != in_end; ++in)
    crc.write_data (*in);

  return crc.current_value ();
}

template <>
inline uint32_t core<32, 0x04C11DB7, true, false>
  ::process_bytes (uint32_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& crc = this_board::inst ().crc;

  crc.set_reset_init_value (rem);
  crc.set_control (dev::stm32f0_crc::control_t ()
	.set_input_bitorder (dev::stm32f0_crc::input_reverse_8)
	.set_output_bitorder (dev::stm32f0_crc::output_non_reversed)
	.set_reset ());

  for (; in != in_end; ++in)
    crc.write_data (*in);

  return crc.current_value ();
}

template <>
inline uint32_t core<32, 0x04C11DB7, false, true>
  ::process_bytes (uint32_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& crc = this_board::inst ().crc;

  crc.set_reset_init_value (rem);
  crc.set_control (dev::stm32f0_crc::control_t ()
	.set_input_bitorder (dev::stm32f0_crc::input_non_reversed)
	.set_output_bitorder (dev::stm32f0_crc::output_reversed)
	.set_reset ());

  for (; in != in_end; ++in)
    crc.write_data (*in);

  return crc.current_value ();
}

template<>
constexpr inline bool core<32, 0x04C11DB7, false, true>
  ::is_remainder_reflected (void)
{
  return true;
}


template <>
inline uint32_t core<32, 0x04C11DB7, true, true>
  ::process_bytes (uint32_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& crc = this_board::inst ().crc;

  crc.set_reset_init_value (rem);
  crc.set_control (dev::stm32f0_crc::control_t ()
	.set_input_bitorder (dev::stm32f0_crc::input_reverse_8)
	.set_output_bitorder (dev::stm32f0_crc::output_reversed)
	.set_reset ());

  for (; in != in_end; ++in)
    crc.write_data (*in);

  return crc.current_value ();
}

template<>
constexpr inline bool core<32, 0x04C11DB7, true, true>
  ::is_remainder_reflected (void)
{
  return true;
}

} // namespace crc_impl_detail
} // hash

#endif // BOARD_NO_CRC
#endif // includeguard_nld_bsp_hash_crc_includeguard
