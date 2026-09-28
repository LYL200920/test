#ifndef includeguard_nld_bsp_hash_crc_includeguard
#define includeguard_nld_bsp_hash_crc_includeguard

#include "../../../hash/crc.hpp"

#ifndef BOARD_NO_CRC

#include <board/board.hpp>

namespace hash
{
namespace crc_impl_detail
{

// FIXME: the GD32F30x supports CRC32 as it is used in ethernet

} // namespace crc_impl_detail
} // hash

#endif // MCB_NO_CRC
#endif // includeguard_nld_bsp_hash_crc_includeguard
