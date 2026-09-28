
#include "stm32f0_flash.hpp"

namespace dev
{
namespace stm32f0_flash
{

hw_inst_base::hw_inst_base (void)
{
  wait_busy ();

  if (regs ().cr & (1 << 7))
  {
    // flash is locked and should be unlocked
    regs ().keyr = 0x45670123;
    regs ().keyr = 0xCDEF89AB;
  }
}

bool hw_inst_base::wait_busy (void)
{
  while (true)
  {
    auto sr_val = regs ().sr.read ();

    // wait for BSY flag to go off
    // FIXME: add timeout
    if (sr_val & (1 << 0))
      continue;

    // write protection error
    if (sr_val & (1 << 4))
    {
      regs ().sr = 1 << 4;
      return false;
    }

    // programming error
    if (sr_val & (1 << 2))
    {
      regs ().sr = 1 << 2;
      return false;
    }

    // successful end of operation
    if (sr_val & (1 << 5))
    {
      regs ().sr = 1 << 5;
      return true;
    }

    // nothing ... perhaps the BSY flag was not set at all and there
    // was no pending operation.
    return true;
  }
}

bool hw_inst_base::blank_check_1 (uintptr_t start_addr_, uintptr_t end_addr_)
{
  const uint32_t* start_addr = (const uint32_t*)utils::floor_pow2 (start_addr_, (uintptr_t)4);
  const uint32_t* const end_addr = (const uint32_t*)utils::floor_pow2 (end_addr_, (uintptr_t)4);

  while (start_addr < end_addr)
  {
    if (*start_addr++ != 0xFFFFFFFF)
      return false;
  }

  return true;
}

bool hw_inst_base::erase_block_1 (uintptr_t page_addr)
{
  // assume that the address has been validated before we get here.
  auto&& r = regs ();
  r.cr |= 1 << 1;   // set PER
  r.ar = page_addr;
  r.cr |= 1 << 6;   // set STRT

  bool op_result = wait_busy ();
  r.cr &= ~(1 << 1); // clear PER
  return op_result;
}

bool hw_inst_base::erase_all_blocks_1 (void)
{
  auto&& r = regs ();
  r.cr |= 1 << 2;  // set MER
  r.cr |= 1 << 6;  // set STRT

  bool op_result = wait_busy ();
  r.cr &= ~(1 << 2); // clear MER
  return op_result;
}

bool hw_inst_base::write_block_1 (uintptr_t addr, const void* src_data)
{
  // assume that the address has been validated before we get here.
  auto&& r = regs ();
  r.cr = (r.cr & ~(1 << 6)) | (1 << 0); // clear STRT, set PG

  bool op_result = true;

  const uint8_t* src_data8 = (const uint8_t*)src_data;

  for (unsigned int i = 0; i < block_size/2 && op_result; ++i)
  {
    uint16_t v = (src_data8[1] << 8) | src_data8[0];

    *(volatile uint16_t*)(addr) = v;
    addr += 2;
    src_data8 += 2;
    op_result = wait_busy ();
  }

  r.cr &= ~(1 << 0); // clear PG
  return op_result;
}


} // namespace stm32f0_flash
} // namespace dev

