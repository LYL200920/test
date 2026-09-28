#ifndef includeguard_mcb_v1_bsp_hash_crc_includeguard
#define includeguard_mcb_v1_bsp_hash_crc_includeguard

#include "../../../hash/crc.hpp"

#ifndef MCB_NO_CRC

#include <board/board.hpp>

namespace hash
{
namespace crc_impl_detail
{

template<> inline uint8_t core<8, 0x07, false, false>
  ::process_bytes (uint8_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& rxcrc = this_board::inst ().crc;

  rxcrc.set_control (dev::rx63_crc::crccr_t ()
			.set_crc_type (dev::rx63_crc::crc_8_2_1_0)
			.set_bitorder (dev::rx63_crc::msb_first));

  rxcrc.set_data_output (rem);

  for (; in != in_end; ++in)
    rxcrc.set_data_input (*in);

  return rxcrc.data_output ();
}

template<> inline uint8_t core<8, 0x07, true, true>
  ::process_bytes (uint8_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& rxcrc = this_board::inst ().crc;

  rxcrc.set_control (dev::rx63_crc::crccr_t ()
			.set_crc_type (dev::rx63_crc::crc_8_2_1_0)
			.set_bitorder (dev::rx63_crc::lsb_first));

  rxcrc.set_data_output (rem);

  for (; in != in_end; ++in)
    rxcrc.set_data_input (*in);

  return rxcrc.data_output ();
}

template<> inline uint16_t core<16, 0x8005, false, false>
  ::process_bytes (uint16_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& rxcrc = this_board::inst ().crc;

  rxcrc.set_control (dev::rx63_crc::crccr_t ()
			.set_crc_type (dev::rx63_crc::crc_16_15_2_0)
			.set_bitorder (dev::rx63_crc::msb_first));

  rxcrc.set_data_output (rem);

  for (; in != in_end; ++in)
    rxcrc.set_data_input (*in);

  return rxcrc.data_output ();
}

template<> inline uint16_t core<16, 0x8005, true, true>
  ::process_bytes (uint16_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& rxcrc = this_board::inst ().crc;

  rxcrc.set_control (dev::rx63_crc::crccr_t ()
			.set_crc_type (dev::rx63_crc::crc_16_15_2_0)
			.set_bitorder (dev::rx63_crc::lsb_first));

  rxcrc.set_data_output (rem);

  for (; in != in_end; ++in)
    rxcrc.set_data_input (*in);

  return rxcrc.data_output ();
}

template<> inline uint16_t core<16, 0x1021, false, false>
  ::process_bytes (uint16_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& rxcrc = this_board::inst ().crc;

  rxcrc.set_control (dev::rx63_crc::crccr_t ()
			.set_crc_type (dev::rx63_crc::crc_16_12_5_0)
			.set_bitorder (dev::rx63_crc::msb_first));

  rxcrc.set_data_output (rem);

  for (; in != in_end; ++in)
    rxcrc.set_data_input (*in);

  return rxcrc.data_output ();
}

template<> inline uint16_t core<16, 0x1021, true, true>
  ::process_bytes (uint16_t rem, const uint8_t* in, const uint8_t* in_end)
{
  auto&& rxcrc = this_board::inst ().crc;

  rxcrc.set_control (dev::rx63_crc::crccr_t ()
			.set_crc_type (dev::rx63_crc::crc_16_12_5_0)
			.set_bitorder (dev::rx63_crc::lsb_first));

  rxcrc.set_data_output (rem);

  for (; in != in_end; ++in)
    rxcrc.set_data_input (*in);

  return rxcrc.data_output ();
}

} // namespace crc_impl_detail
} // hash

#endif // MCB_NO_CRC
#endif // includeguard_mcb_v1_bsp_hash_crc_includeguard
