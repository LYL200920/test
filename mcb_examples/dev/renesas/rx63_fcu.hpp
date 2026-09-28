#ifndef includeguard_dev_rx63_fcu_hpp_includeguard
#define includeguard_dev_rx63_fcu_hpp_includeguard

#include <utility>
#include <cstdint>

#include <dev/cpu.hpp>
#include <dev/flash_memory.hpp>

namespace dev
{

class rx63_fcu
{
public:
  using block_info = dev::flash_memory::block_info;

  // important: the peripheral clock must match the actual clock, otherwise
  // the on-chip flash might get damaged.
  rx63_fcu (unsigned int peripheral_clock_mhz);

  auto& code_flash_dev (void) { return m_code_flash_dev; }
  auto& data_flash_dev (void) { return m_data_flash_dev; }

private:

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // data flash functions

  block_info data_erase_block_for_addr (uintptr_t data_cpu_addr);

  bool erase_data_block (uintptr_t data_cpu_addr);
  bool erase_data_block (const block_info& bi);

  bool erase_all_data_blocks (void);

  bool is_blank_data_area (const block_info& bi) const;

  // write one data block.  if the block is not blank at that address,
  // writing will fail and the block must be erased first.
  static constexpr unsigned int write_data_block_size (void) { return 32; }
  bool write_data_block (uintptr_t data_cpu_addr, const void* src_data);

  // read one block
  bool read_data_block (uintptr_t data_cpu_addr, void* dst_data) const;

  struct data_flash_dev_impl final : public flash_memory
  {
    rx63_fcu& m_parent;

    data_flash_dev_impl (rx63_fcu& p)
    : flash_memory (32, 0x00100000, 0x00100000 + 32*1024), m_parent (p) { }

    virtual block_info erase_block_for_addr (uintptr_t addr) const override
    {
      return m_parent.data_erase_block_for_addr (addr);
    }

    virtual bool erase_block (uintptr_t addr) override
    {
      return m_parent.erase_data_block (addr);
    }

    virtual bool erase_block (const block_info& bi) override
    {
      return m_parent.erase_data_block (bi);
    }

    virtual bool erase_all_blocks (void) override
    {
      return m_parent.erase_all_data_blocks ();
    }

    virtual bool is_area_blank (const block_info& bi) const override
    {
      return m_parent.is_blank_data_area (bi);
    }

    virtual bool write_block (uintptr_t addr, const void* data) override
    {
      return m_parent.write_data_block (addr, data);
    }

    virtual bool read_block (uintptr_t addr, void* data) override
    {
      return m_parent.read_data_block (addr, data);
    }
  } m_data_flash_dev;


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // code flash functions

  // returns the rom block for the specified address.
  static inline constexpr block_info
  rom_erase_block_for_addr (uintptr_t rom_cpu_addr)
  {
    uintptr_t pe_addr = rom_map_addr (rom_cpu_addr);
    uintptr_t block_size_kb = 0;

    if (pe_addr >= rom_map_addr (0xFFE00000) && pe_addr < rom_map_addr (0xFFF00000))
      block_size_kb = 64;
    else if (pe_addr >= rom_map_addr (0xFFF00000) && pe_addr < rom_map_addr (0xFFF80000))
      block_size_kb = 32;
    else if (pe_addr >= rom_map_addr (0xFFF80000) && pe_addr < rom_map_addr (0xFFFF8000))
      block_size_kb = 16;
    else if (pe_addr >= rom_map_addr (0xFFFF8000) && pe_addr <= rom_map_addr (0xFFFFFFFF))
      block_size_kb = 4;
    else
      return { 0, 0 };

    auto start_addr = rom_cpu_addr & ~(block_size_kb*1024 - 1);
    auto end_addr = start_addr + block_size_kb*1024;

    return { start_addr, end_addr };
  }

  // erase one block.  the block size is defined implicitly by the block address.
  bool erase_rom_block (uintptr_t rom_cpu_addr);
  bool erase_rom_block (const block_info& bi);

  // check if the whole block is blank.  the block size is the same as for
  // erasing blocks.  we could also do a partial blank check but it seems not
  // so useful (except for special cases).  if a block is not blank, usually
  // the whole block will be erased.
  bool is_blank_rom_area (const block_info& bi) const;

  // write one 128 byte block.  if the block is not blank at that address,
  // writing will fail and the block must be erased first.
  static constexpr unsigned int write_rom_block_size (void) { return 128; }
  bool write_rom_block (uintptr_t rom_cpu_addr, const void* src_data);

  // read one 128 byte block.
  bool read_rom_block (uintptr_t rom_cpu_addr, void* dst_data) const;


  struct code_flash_dev_impl final : public flash_memory
  {
    rx63_fcu& m_parent;

    // report the maximum possible value for the code flash area.
    code_flash_dev_impl (rx63_fcu& p)
    : flash_memory (128, 0xFFE00000, 0xFFE00000 + 1024*1024*2), m_parent (p) { }

    virtual block_info erase_block_for_addr (uintptr_t addr) const override
    {
      return m_parent.rom_erase_block_for_addr (addr);
    }

    virtual bool erase_block (uintptr_t addr) override
    {
      return m_parent.erase_rom_block (addr);
    }

    virtual bool erase_block (const block_info& bi) override
    {
      return m_parent.erase_rom_block (bi);
    }

    virtual bool erase_all_blocks (void) override
    {
      return false;
    }

    virtual bool is_area_blank (const block_info& bi) const override
    {
      return m_parent.is_blank_rom_area (bi);
    }

    virtual bool write_block (uintptr_t addr, const void* data) override
    {
      return m_parent.write_rom_block (addr, data);
    }

    virtual bool read_block (uintptr_t addr, void* data) override
    {
      return m_parent.read_rom_block (addr, data);
    }
  } m_code_flash_dev;


  // the current mode is not needed for small applications.
  // it might be required for asynchonous high speed operations such as writing
  // data to the flash in the background etc.
  enum mode_t
  {
    // normal "high speed" read mode, i.e. all E2 / ROM programming is disabled.
    read_mode,

    // data flash program/erase mode
    data_pe_mode,

    // ROM flash program/erase mode
    rom_pe_mode
  };

  // static mode_t g_cur_mode;

  static bool enter_data_pe_mode (void);
  static void enter_read_mode (void);
  static void wait_ready (void);
  static bool is_error (void);
  static bool check_error (void);

  static constexpr uintptr_t data_block_addr (uintptr_t a)
  {
    // FIXME: add static_assert on power-of-two for the blocksize.
    return a & ~(write_data_block_size () - 1);
  }

  static inline constexpr uint16_t rom_map_addr (uintptr_t rom_cpu_addr)
  {
    // RX63N has up to 70 blocks.
    // smallest block is 4 KB = 12 bits
    // max addr = 2 MB = 21 bits
    // 21 - 12 = 9 bits
    return (rom_cpu_addr & 0x00FFFFFF) / 4096;
  }

  static void set_pe_clock (unsigned int clock_mhz);

  // RX ROM area is split into 512 KByte pages for programming.
  //
  // on-chip ROM | ROM reading address |
  //  capacity   |      range          | FENTRY0 = 1         | FENTRY1 = 1         | FENTRY2 = 1         | FENTRY3 = 1         |
  // ------------+---------------------+---------------------+---------------------+---------------------+---------------------+
  //   256 K     | FFFC0000 - FFFFFFFF | 00FC0000 - 00FFFFFF |                     |                     |                     |
  //   384 K     | FFFA0000 - FFFFFFFF | 00FA0000 - 00FFFFFF |                     |                     |                     |
  //   512 K     | FFF80000 - FFFFFFFF | 00F80000 - 00FFFFFF |                     |                     |                     |
  //   768 K     | FFF40000 - FFFFFFFF | 00F80000 - 00FFFFFF | 00F40000 - 00F7FFFF |                     |                     |
  //   1 M       | FFF00000 - FFFFFFFF | 00F80000 - 00FFFFFF | 00F00000 - 00F7FFFF |                     |                     |
  //   1.5 M     | FFE80000 - FFFFFFFF | 00F80000 - 00FFFFFF | 00F00000 - 00F7FFFF | 00E80000 - 00EFFFFF |                     |
  //   2 M       | FFE00000 - FFFFFFFF | 00F80000 - 00FFFFFF | 00F00000 - 00F7FFFF | 00E80000 - 00EFFFFF | 00E00000 - 00E7FFFF |
  // ------------+---------------------+---------------------+---------------------+---------------------+---------------------+

  enum rom_pe_area
  {
    rom_pe_area_invalid = 0,
    rom_pe_area_0 = 1,
    rom_pe_area_1 = 2,
    rom_pe_area_2 = 4,
    rom_pe_area_3 = 8
  };

  // map a "normal read mode" ROM address into a programming address and
  // its corresponding area.
  struct rom_pe_addr
  {
    constexpr rom_pe_addr (uintptr_t a0, uintptr_t a1)
    : addr (a0), area ((rom_pe_area)a1) { }

    uintptr_t addr;
    rom_pe_area area;
  };

  static constexpr rom_pe_addr make_rom_pe_addr (uintptr_t addr)
  {
    return { addr & 0x00FFFFFF,
	     (0x08'04'02'01u >> (((0x00FFFFFF - (addr & 0x00FFFFFF)) / (512*1024)) * 8)) & 0xFF };
  }

  static constexpr uintptr_t rom_pe_area_addr_begin (rom_pe_area a)
  {
    // 1 = 0001 -> 00F80000 = 111 110000000000000000000  11 = 3
    // 2 = 0010 -> 00F00000 = 111 100000000000000000000  10 = 2
    // 4 = 0100 -> 00E80000 = 111 010000000000000000000  01 = 1
    // 8 = 1000 -> 00E00000 = 111 000000000000000000000  00 = 0

    // FIXME: the mapping is a bit unlucky.  the rom_pe_area definitions should
    // be linear instead of logarithmic.
    return a == rom_pe_area_0
	   ? 0x00F80000
	   : a == rom_pe_area_1
	     ? 0x00F00000
	     : a == rom_pe_area_2
	       ? 0x00E80000
	       : a == rom_pe_area_3
		 ? 0x00E00000
		 : 0;
  }

  static constexpr uintptr_t rom_pe_area_addr_end (rom_pe_area a)
  {
    return rom_pe_area_addr_begin (a) + 512*1024;
  }

  static void rom_read_to_pe_static_checks (void);

  struct enter_rom_pe_mode_result
  {
    dev::this_cpu::status_reg_value prev_psw;
    bool success;

    explicit operator bool (void) const { return success; }
  };

  static enter_rom_pe_mode_result
  enter_rom_pe_mode (rom_pe_area area_sel, uintptr_t addr_begin);

  static void leave_rom_pe_mode (dev::this_cpu::status_reg_value prev_psw);

  enum struct op_result
  {
    ok,
    pe_enter_ng,
    erase_ng,
    write_ng,
  };

  static op_result erase_rom_block_1 (rom_pe_area area_sel, uintptr_t addr_begin);

  static op_result write_rom_block_1 (rom_pe_area area_sel, uintptr_t addr_begin,
				      const uint16_t* data);

};

} // namespace dev
#endif // includeguard_dev_rx63_fcu_hpp_includeguard
