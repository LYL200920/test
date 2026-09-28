#ifndef includeguard_li5000_mcb_network_config_includeguard
#define includeguard_li5000_mcb_network_config_includeguard

#include <cstdint>
#include <array>
#include <bitset>

#include <utils/bits.hpp>

// the ipv4 config file with fileid = 0 has a fixed layout.
enum
{
  ipv4_config_fileid = 0,
  io_config_fileid = 1,
  trigger_io_config_fileid = 2,
  conveyor_config_fileid_lane0 = 3,
  gantry_config_fileid = 4,
  conveyor_config_fileid_lane1 = 5,
};

// ----------------------------------------------------------------------------

struct io_config
{
  io_config (void) { }
  io_config (const std::bitset<32>& ia, const std::bitset<32>& io,
	     const std::bitset<32>& ix, const std::bitset<32>& oi,
	     const std::bitset<32>& oa, const std::bitset<32>& oo,
	     const std::bitset<32>& ox)
  : inputs_and_mask_raw (ia.to_ulong ()),
    inputs_or_mask_raw (io.to_ulong ()),
    inputs_xor_mask_raw (ix.to_ulong ()),
    outputs_init_mask_raw (oi.to_ulong ()),
    outputs_and_mask_raw (oa.to_ulong ()),
    outputs_or_mask_raw (oo.to_ulong ()),
    outputs_xor_mask_raw (ox.to_ulong ())
  {
  }

  // std::bitset<32>, stored as little endian byte order in the file.
  uint32_t inputs_and_mask_raw;
  uint32_t inputs_or_mask_raw;
  uint32_t inputs_xor_mask_raw;

  uint32_t outputs_init_mask_raw;
  uint32_t outputs_and_mask_raw;
  uint32_t outputs_or_mask_raw;
  uint32_t outputs_xor_mask_raw;

  std::bitset<32> inputs_and_mask (void) const { return { inputs_and_mask_raw }; }
  std::bitset<32> inputs_or_mask (void) const { return { inputs_or_mask_raw }; }
  std::bitset<32> inputs_xor_mask (void) const { return { inputs_xor_mask_raw }; }

  std::bitset<32> outputs_init_mask (void) const { return { outputs_init_mask_raw }; }
  std::bitset<32> outputs_and_mask (void) const { return { outputs_and_mask_raw }; }
  std::bitset<32> outputs_or_mask (void) const { return { outputs_or_mask_raw }; }
  std::bitset<32> outputs_xor_mask (void) const { return { outputs_xor_mask_raw }; }
};

io_config read_io_config (void);
void write_io_config (const io_config& val);

#endif // includeguard_li5000_mcb_network_config_includeguard
