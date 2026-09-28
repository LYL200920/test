/*

generic flash memory device

*/

#ifndef includeguard_dev_flash_memory_hpp_includeguard
#define includeguard_dev_flash_memory_hpp_includeguard

#include <utility>
#include <cstdint>
#include <utils/bits.hpp>

namespace dev
{

class flash_memory
{
public:
  class block_info
  {
  public:
    constexpr block_info (void) : m_start_addr (0), m_end_addr (0) { }

    constexpr block_info (uintptr_t s, uintptr_t e) : m_start_addr (s), m_end_addr (e) { }

    // start address of the block (cpu read address)
    constexpr uintptr_t start_addr (void) const { return m_start_addr; }

    // end address of the block (cpu read address) plus 1, so that
    // end_addr - start_addr = block size in bytes.
    constexpr uintptr_t end_addr (void) const { return m_end_addr; }

    // block size in bytes.
    constexpr uintptr_t size (void) const { return m_end_addr - m_start_addr; }

    constexpr bool operator == (const block_info& rhs) const { return m_start_addr == rhs.m_start_addr; }
    constexpr bool operator != (const block_info& rhs) const { return m_start_addr != rhs.m_start_addr; }

    // note the end addr might be 0.  e.g. the last block of the RX code flash
    // is 0xFFFFE000 - 0xFFFFFFFF+1, which wraps around to 0.
    constexpr explicit operator bool (void) const { return m_start_addr != 0; }

    // check if the specified address is within this block, taking into
    // account that the end address of this block might wrap around to 0.
    constexpr bool contains_addr (uintptr_t val) const
    {
      return (val - m_start_addr) < size ();
    }

  private:
    // FIXME: to support larger NAND memories, need 64 bit
    uintptr_t m_start_addr;
    uintptr_t m_end_addr;
  };

  // get the erase block for the specified address.
  virtual block_info erase_block_for_addr (uintptr_t addr) const = 0;

  // erase one single block.  the size of the block is determined by the
  // flash memory topology.
  virtual bool erase_block (uintptr_t addr) = 0;
  virtual bool erase_block (const block_info& bi) = 0;

  // erase all blocks
  virtual bool erase_all_blocks (void) = 0;

  // perform a blank check of the specified area.
  // normally the area should be aligned to the read/write block size.
  virtual bool is_area_blank (const block_info& bi) const = 0;

  // assume that the read/write block size is constant across the whole memory.
  // the block infos as returned by "block_for_addr" are for the erase blocks
  // which can be different, depending on the flash memory organization.
  unsigned int rw_block_size (void) const { return m_read_write_block_size; }

  // get the read/write block for an address.
  block_info rw_block_for_addr (uintptr_t addr) const
  {
    auto aligned_addr = utils::floor_pow2 (addr, (uintptr_t)m_read_write_block_size);
    return { aligned_addr, aligned_addr + m_read_write_block_size };
  }

  virtual bool write_block (uintptr_t addr, const void* src_data) = 0;
  virtual bool read_block (uintptr_t addr, void* dst_data) = 0;

  // re-use the block_info for the description of the whole memory.
  // not all devices support memory-mapped read access.
  // if memory mapped access is not supported, an invalid block is returned.
  block_info mmap_info (void) const { return { m_mmap_addr_begin, m_mmap_addr_end }; }

protected:
  // read-only fields initialized by subclass
  const unsigned int m_read_write_block_size;
  const uintptr_t m_mmap_addr_begin;
  const uintptr_t m_mmap_addr_end;

  flash_memory (unsigned int rd_wr_block_size,
		uintptr_t mmap_addr_begin, uintptr_t mmap_addr_end)
  : m_read_write_block_size (rd_wr_block_size),
    m_mmap_addr_begin (mmap_addr_begin),
    m_mmap_addr_end (mmap_addr_end)
  {
  }

};

} // namespace dev
#endif // includeguard_dev_flash_memory_hpp_includeguard
