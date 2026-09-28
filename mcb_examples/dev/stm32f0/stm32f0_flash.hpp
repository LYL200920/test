
/*


STM32F03x, STM32F04x, STM32F05x: page size = 1 KByte

STM32F07x, STM32F09x: page size = 2 KByte

although the base address of the raw flash is always at 0x08000000,
allow specifying it as a parameter.  this allows to split the flash
device into multiple virtual flash devices.

*/

#ifndef includeguard_dev_stm32f0_flash_hpp_includeguard
#define includeguard_dev_stm32f0_flash_hpp_includeguard

#include <utility>
#include <cstdint>
#include <cstring>

#include <dev/hwreg.hpp>
#include <dev/flash_memory.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace stm32f0_flash
{

class hw_inst_base
{
public:
  hw_inst_base (void);


protected:
  // actual read/write flash granularity is 2 bytes.  but that's too small
  // to be useful, so use 32.
  static constexpr unsigned int block_size = 32;

  static constexpr uintptr_t reg_base_addr = 0x40022000;

  struct regs_t
  {					// offset
    hw_reg_rw<uint32_t> acr;		// 0x00
    hw_reg_rw<uint32_t> keyr;		// 0x04
    hw_reg_rw<uint32_t> optkeyr;	// 0x08
    hw_reg_rw<uint32_t> sr;		// 0x0C
    hw_reg_rw<uint32_t> cr;		// 0x10
    hw_reg_rw<uint32_t> ar;		// 0x14
    uint32_t res_0x18;
    hw_reg_rw<uint32_t> obr;		// 0x1C
    hw_reg_rw<uint32_t> wrpr;		// 0x20
  };

  static_assert (sizeof (regs_t) == 0x24, "");
  static constexpr regs_t& regs (void) { return *(regs_t*)reg_base_addr; }

  static bool erase_block_1 (uintptr_t page_addr);
  static bool erase_all_blocks_1 (void);
  static bool write_block_1 (uintptr_t addr, const void* src_data);

  static bool wait_busy (void);

  static bool blank_check_1 (uintptr_t start_addr, uintptr_t end_addr);
};


template <unsigned int PageSize>
class hw_inst final : public hw_inst_base, public dev::flash_memory
{
  static constexpr uintptr_t page_size = PageSize;
  static_assert (utils::is_pow2 (page_size), "");

public:
  hw_inst (uintptr_t start_addr, uintptr_t end_addr)
  : dev::flash_memory (block_size, start_addr, end_addr)
  {
  }

  virtual block_info erase_block_for_addr (uintptr_t addr) const override
  {
    if (!page_address_valid (addr))
      return { };

    addr = utils::floor_pow2 (addr, page_size);
    return { addr, addr + page_size };
  }

  virtual bool erase_block (uintptr_t addr) override
  {
    if (!page_address_valid (addr))
      return false;

    return erase_block_1 (utils::floor_pow2 (addr, page_size));
  }

  virtual bool erase_block (const block_info& bi) override
  {
    return erase_block (bi.start_addr ());
  }

  virtual bool erase_all_blocks (void) override
  {
    return erase_all_blocks_1 ();
  }

  virtual bool is_area_blank (const block_info& bi) const override
  {
    if (bi.start_addr () >= m_mmap_addr_begin && bi.start_addr () < m_mmap_addr_end
        && bi.end_addr () >= m_mmap_addr_begin && bi.end_addr () <= m_mmap_addr_end)
      return blank_check_1 (bi.start_addr (), bi.end_addr ());

    return false;
  }

  virtual bool write_block (uintptr_t addr, const void* src_data) override
  {
    if (!block_address_valid (addr))
      return false;

    return write_block_1 (addr, src_data);
  }

  virtual bool read_block (uintptr_t addr, void* dst_data) override
  {
    if (!block_address_valid (addr))
      return false;

    std::memcpy (dst_data, (const void*)addr, block_size);
    return true;
  }

private:
  bool page_address_valid (uintptr_t addr) const
  {
    return addr >= m_mmap_addr_begin && addr < m_mmap_addr_end;
  }

  bool block_address_valid (uintptr_t addr) const
  {
    return addr >= m_mmap_addr_begin && addr < m_mmap_addr_end;
  }
};

} // namespace stm32f0_flash
} // namespace dev
#endif // includeguard_dev_stm32f0_flash_hpp_includeguard
