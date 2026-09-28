
#ifndef includeguard_dev_rx64_fcu_hpp_includeguard
#define includeguard_dev_rx64_fcu_hpp_includeguard

#include <utility>
#include <cstdint>
#include <cstring>

#include <dev/cpu.hpp>
#include <dev/hwreg.hpp>
#include <dev/flash_memory.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace rx64_fcu
{

class hw_inst_base
{
public:
  using block_info = dev::flash_memory::block_info;

  class code_flash_dev_impl final : public flash_memory
  {
    hw_inst_base& m_parent;

    static bool erase_block_1 (uintptr_t addr_begin);
    static bool write_block_1 (uintptr_t addr_begin, const uint16_t* data);

  public:
    static constexpr unsigned int write_block_size (void) { return 256; }
    static constexpr unsigned int read_block_size (void) { return 256; }

    // actually the flash memory area starts at 0xFF7F8000 with a 32kbyte
    // user boot area.  however, we don't include that here, since that
    // area is somewhat special and should not be accessed by applications
    // directly.
    code_flash_dev_impl (hw_inst_base& p)
    : flash_memory (read_block_size (), 0xFFC00000, 0xFFC00000 + 1024*1024*4),
      m_parent (p) { }

    virtual block_info erase_block_for_addr (uintptr_t addr) const override;
    virtual bool erase_block (uintptr_t addr) override;
    virtual bool erase_block (const block_info& bi) override;
    virtual bool erase_all_blocks (void) override;
    virtual bool is_area_blank (const block_info& bi) const override;
    virtual bool write_block (uintptr_t addr, const void* data) override;
    virtual bool read_block (uintptr_t addr, void* data) override;

  } m_code_flash_dev;


  class data_flash_dev_impl final : public flash_memory
  {
    hw_inst_base& m_parent;

  public:
    static constexpr unsigned int erase_block_size (void) { return 64; }

    // read/write block sizes are just software definitions.
    // hardware for writes is 4 byte blocks.
    static constexpr unsigned int read_write_block_size (void) { return 32; }

    static constexpr uintptr_t erase_block_addr (uintptr_t a)
    {
      static_assert (utils::is_pow2 (erase_block_size ()), "");
      return a & ~(erase_block_size () - 1);
    }

    data_flash_dev_impl (hw_inst_base& p)
    : flash_memory (read_write_block_size (), 0x00100000, 0x00100000 + 64*1024),
      m_parent (p) { }

    virtual block_info erase_block_for_addr (uintptr_t addr) const override;
    virtual bool erase_block (uintptr_t addr) override;
    virtual bool erase_block (const block_info& bi) override;
    virtual bool erase_all_blocks (void) override;
    virtual bool is_area_blank (const block_info& bi) const override;
    virtual bool write_block (uintptr_t addr, const void* data) override;
    virtual bool read_block (uintptr_t addr, void* data) override;

  } m_data_flash_dev;


  auto& code_flash_dev (void) { return m_code_flash_dev; }
  auto& data_flash_dev (void) { return m_data_flash_dev; }

protected:
  hw_inst_base (unsigned int clock_mhz);
  ~hw_inst_base (void);

private:
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008C296>> FWEPROR = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x007FE010>> FASTAT = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x007FE014>> FAEINT = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x007FE018>> FRDYIE = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x007FE030>> FSADDR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x007FE034>> FEADDR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x007FE054>> FCURAME = { };
  static constexpr hw_reg_r<uint32_t, const_addr<0x007FE080>> FSTATR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x007FE084>> FENTRYR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x007FE088>> FPROTR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x007FE08C>> FSUINITR = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x007FE090>> FLKSTAT = { };
  static constexpr hw_reg_r<uint16_t, const_addr<0x007FE0A0>> FCMDR = { };
  static constexpr hw_reg_r<uint16_t, const_addr<0x007FE0C0>> FPESTAT = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x007FE0D0>> FBCCNT = { };
  static constexpr hw_reg_r<uint8_t, const_addr<0x007FE0D4>> FBCSTAT = { };
  static constexpr hw_reg_r<uint32_t, const_addr<0x007FE0D8>> FPSADDR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x007FE0E0>> FCPSR = { };
  static constexpr hw_reg_rw<uint16_t, const_addr<0x007FE0E4>> FPCKAR = { };

  // command write area is 4 bytes long, command writes are always 8 bit,
  // except for programming data, which is transferred using 16 bit writes.
  static constexpr hw_reg_w<uint8_t, const_addr<0x007E0000>> FACI_CMD8 = { };
  static constexpr hw_reg_w<uint16_t, const_addr<0x007E0000>> FACI_CMD16 = { };

  static void enter_code_pe_mode (void);
  static void enter_data_pe_mode (void);
  static void enter_read_mode (void);
  static bool wait_ready (void);
  static void forced_stop_command (void);
  static void set_pe_clock (unsigned int clock_mhz);
  static void wait_fifo_empty (void);

  void fiferr_isr (void);
  void frdyi_isr (void);

  void data_flash_memory_access_voilation_isr (void);
  void command_locked_isr (void);
  void code_flash_memory_access_voilation_isr (void);

};

template <typename InterruptLine_FIFERR,
	  typename InterruptLine_FRDYI>
class hw_inst final : public hw_inst_base
{
public:
  using fiferr_isr_t = interrupt::connected_isr<InterruptLine_FIFERR,
	interrupt::func<decltype (&hw_inst_base::fiferr_isr), &hw_inst_base::fiferr_isr>>;

  using frdyi_isr_t = interrupt::connected_isr<InterruptLine_FRDYI,
	interrupt::func<decltype (&hw_inst_base::frdyi_isr), &hw_inst_base::frdyi_isr>>;


  hw_inst (unsigned int clock_mhz)
  : hw_inst_base (clock_mhz), m_fiferr_isr (this), m_frdyi_isr (this)
  {
    m_fiferr_isr.enable (dev::interrupt::low_level, dev::interrupt::priority_2);
    m_frdyi_isr.enable (dev::interrupt::falling_edge, dev::interrupt::priority_2);
  }

  ~hw_inst (void) { }

private:
  fiferr_isr_t m_fiferr_isr;
  frdyi_isr_t m_frdyi_isr;


};

} // namespace rx64_fcu
} // namespace dev
#endif // includeguard_dev_rx64_fcu_hpp_includeguard
