
#define log_all
#include <logging/logging.hpp>

#include <thread>
#include <dev/interrupt.hpp>

#include "rx64_fcu.hpp"


namespace dev
{
namespace rx64_fcu
{

#include "rx_fcu_ramfunc.hpp"


[[gnu::cold]] hw_inst_base::hw_inst_base (unsigned int clock_mhz)
: m_code_flash_dev (*this),
  m_data_flash_dev (*this)
{
  // allow programming/erasure of data, programming/erasure of lock bits,
  // blank checking.
  FWEPROR = 0b01;

  while (FENTRYR != 0)
  {
    FENTRYR = 0xAA00;

    // wait for the reset time to elapse.
    std::this_thread::sleep_for (std::chrono::microseconds (35));
  }

  // enable high-speed write-only FCU RAM access from CPU side
  FCURAME = 0xC403;

  // copy FCU firmware
  // src: 0xFEFFF000 to 0xFEFFFFFF (FCU firmware area)
  // dst: 0x007F8000 to 0x007F8FFF (FCU RAM area)
  // size: 0x1000 = 4096 bytes
  // use 32 bit access
  {
    const uint32_t* src_ptr = (const uint32_t*)0xFEFFF000;
    uint32_t* dst_ptr = (uint32_t*)0x007F8000;
    for (unsigned int i = 0; i < 4096/4; ++i)
      *dst_ptr++ = *src_ptr++;
  }

  // disable FCU RAM access from CPU side
  FCURAME = 0xC400;

  // initialize FCU sequencer by issuing a "forced stop" command
  // in code flash P/E mode.
  enter_data_pe_mode ();

  forced_stop_command ();

  enter_read_mode ();

  // reset/re-init FCU registers to default values
  FSUINITR = 0x2D01;
  FSUINITR = 0x2D00;

  // disable lockbits
  FPROTR = 0x5501;

  set_pe_clock (clock_mhz);

  // interrupts will be enabled on the CPU side by the hw_isnt subclass constructor
  FAEINT = 0;
  FRDYIE = 0;
}

[[gnu::cold]] hw_inst_base::~hw_inst_base (void)
{
  // stop/abort all flash operations and shutdown
  enter_data_pe_mode ();
  forced_stop_command ();
  enter_read_mode ();
}

void hw_inst_base::fiferr_isr (void)
{
  for (unsigned int s = FASTAT; (s & 0b10011000) != 0; s = FASTAT)
  {
    // try to clear the error source bits, which should happen before
    // issuing forced stop command to do error recovery.
    FASTAT = 0;

    if (s & (1 << 3))
      data_flash_memory_access_voilation_isr ();
    if (s & (1 << 4))
      command_locked_isr ();
    if (s & (1 << 7))
      code_flash_memory_access_voilation_isr ();
  }
}

void hw_inst_base::frdyi_isr (void)
{
  // the ready interrupt can be used for background operations
  // of the data flash.

  // for code flash background operations, need a MCU with more than 2 MByte
  // flash and the execution and write areas must not overlap.
}


void hw_inst_base::data_flash_memory_access_voilation_isr (void)
{
}

void hw_inst_base::command_locked_isr (void)
{
}

void hw_inst_base::code_flash_memory_access_voilation_isr (void)
{
}


bool hw_inst_base::wait_ready (void)
{
  // FSTATR access time is 2-4 FCLK.
  // based on that we can calculate the max. number of wait iterations
  // for time-out detection.
  // max. FCLK on RX71 is 60 MHz, max ICLK is 240 MHz.
  // assume best case 1 FSTATR access = 2 FCLK
  // 2 FCLK = 4*2 = 8 ICLK
  // however, if ICLK < FCLK, best case is 2 ICLK.
  // the slowest command time listed in the datasheet is 1040 ms.
  // 1040 ms = 1.040 sec = 249600000 ICLKs -> 249600000/2 = 124800000 ICLKs
  constexpr unsigned int max_wait_count = 124800000;
  for (unsigned int i = 0; i < max_wait_count; ++i)
  {
    auto s = FSTATR.read ();

    // if FCUERR, PRGERR, ERSERR, ILGLERR are set, bail out.
    if (s & ((1 << 7) | (1 << 12) | (1 << 13) | (1 << 14)))
      return false;

    if (s & (1 << 15)) // FRDY
      return true;
  }

  // when here, there was a timeout.
  return false;
}

void hw_inst_base::enter_code_pe_mode (void)
{
  FENTRYR = 0xAA01;
  while (FENTRYR != 0x0001) { }
}

void hw_inst_base::enter_data_pe_mode (void)
{
  FENTRYR = 0xAA80;
  while (FENTRYR != 0x0080) { }
}

void hw_inst_base::enter_read_mode (void)
{
  FENTRYR = 0xAA00;
  while (FENTRYR != 0x0000) { }
}

[[gnu::cold]]
void hw_inst_base::forced_stop_command (void)
{
  FACI_CMD8 = 0xB3;
  while ((FSTATR & (1 << 15)) == 0) { }

  // if there was an address voilation the DFAE or CFAE bits are set
  // and will also set the CMDLK bit.  try clearing those bits first.
  FASTAT = 0x00;

  // if it's in command clocked state, issue a status clear.
  if (FASTAT & (1 << 4))
    FACI_CMD8 = 0x50;

  while ((FSTATR & (1 << 15)) == 0) { }
}

[[gnu::cold]] void
hw_inst_base::set_pe_clock (unsigned int clock_mhz)
{
  // to set the peripheral clock we need to enter any PE mode.
  // use E2 here because it doesn't require execution from RAM.

  enter_data_pe_mode ();

  // change FCU operation clock
  FPCKAR = 0x1E00 | (clock_mhz & 0xFF);

  if (!wait_ready ())
    forced_stop_command ();

  enter_read_mode ();
}

hw_inst_base::block_info
hw_inst_base::code_flash_dev_impl::erase_block_for_addr (uintptr_t cpu_addr) const
{
  // unlike RX63, the RX64/RX71 code rom mapping is simpler.
  // all flash rom resides in the 0xFF........ address range, so we can use
  // the lower 24 bits for comparisions.
  // do not return the block for the user boot area as it can't be erased
  // or programmed in self-programming mode (i.e. by this driver) anyway.
  if (cpu_addr < 0xFFC00000)
    return { };

  // there are 8 8 KByte blocks at the very top.
  // everything else is 32 KByte blocks.
  const uintptr_t block_size_kb = cpu_addr >= 0xFFFF0000 ? 8 : 32;

  auto start_addr = cpu_addr & ~(block_size_kb*1024 - 1);
  auto end_addr = start_addr + block_size_kb*1024;

  return { start_addr, end_addr };
}

bool
hw_inst_base::code_flash_dev_impl::erase_block (uintptr_t addr)
{
  return erase_block (erase_block_for_addr (addr));
}

bool
hw_inst_base::code_flash_dev_impl::erase_block (const block_info& bi)
{
  if (!bi)
    return false;

  auto prev_psw = dev::this_cpu::save_disable_interrupts ();

  auto r = invoke_ramfunc (erase_block_1, bi.start_addr ());

  if (!r)
  {
    enter_data_pe_mode ();
    forced_stop_command ();
    enter_read_mode ();
  }

  dev::this_cpu::restore_interrupts (prev_psw);
  return r;
}

[[__ramfunc_attr__]] bool
hw_inst_base::code_flash_dev_impl::erase_block_1 (uintptr_t addr)
{
  enter_code_pe_mode ();

  FCPSR = 1;
  FSADDR = addr;
  FACI_CMD8 = 0x20;
  FACI_CMD8 = 0xD0;
  bool r = wait_ready ();
  FCPSR = 0;
  enter_read_mode ();
  return r;
}

bool
hw_inst_base::code_flash_dev_impl::erase_all_blocks (void)
{
  // we don't know the size of the flash of the MCU.  so just try all
  // the possible blocks of a 4 MByte device.
  uintptr_t addr = 0xC00000;

  for (; addr < 0xFF0000; addr += 32*1024)
    erase_block (block_info (addr | 0xFF000000, (addr | 0xFF000000) + 32*1024));

  for (; addr < 0x01000000; addr += 8*1024)
    erase_block (block_info (addr | 0xFF000000, (addr | 0xFF000000) + 8*1024));

  return true;
}

bool
hw_inst_base::code_flash_dev_impl::is_area_blank (const block_info& bi) const
{
  // code flash blank checking is like classical NOR flash blank checking.
  // after erasing one block all bits go to '1'.

  // this a bit smaller than std::find_if.
  const uint32_t* p = (const uint32_t*)(bi.start_addr () & ~3u);
  auto count = bi.size () / sizeof (uint32_t);

  for (decltype (count) i = 0; i < count; ++i)
    if (*p++ != 0xFFFFFFFF)
      return false;

  return true;
}

bool
hw_inst_base::code_flash_dev_impl::write_block (uintptr_t addr, const void* data)
{
  // don't check the address -- it will be checked by the FCU
  auto prev_psw = dev::this_cpu::save_disable_interrupts ();

  auto r = invoke_ramfunc (write_block_1, addr, (const uint16_t*)data);

  if (!r)
  {
    enter_data_pe_mode ();
    forced_stop_command ();
    enter_read_mode ();
  }

  dev::this_cpu::restore_interrupts (prev_psw);
  return r;
}

[[gnu::always_inline]] inline void
hw_inst_base::wait_fifo_empty (void)
{
  while (FSTATR & (1 << 10)) { }
}

[[__ramfunc_attr__]] bool
hw_inst_base::code_flash_dev_impl::write_block_1 (uintptr_t addr, const uint16_t* data)
{
  enter_code_pe_mode ();

  FSADDR = addr;
  FACI_CMD8 = 0xE8;
  FACI_CMD8 = 128;  // number of following words to be programmed.

  for (unsigned int i = 0; i < 128; ++i)
  {
    wait_fifo_empty ();
    FACI_CMD16 = *data++;
  }

  FACI_CMD8 = 0xD0;

  bool r = wait_ready ();
  enter_read_mode ();
  return r;
}

bool
hw_inst_base::code_flash_dev_impl::read_block (uintptr_t addr, void* data)
{
  return std::memcpy (data, (const void*)addr, read_block_size ());
}




hw_inst_base::block_info
hw_inst_base::data_flash_dev_impl::erase_block_for_addr (uintptr_t cpu_addr) const
{
  auto start_addr = erase_block_addr (cpu_addr);
  auto end_addr = start_addr + erase_block_size ();

  if (start_addr >= m_mmap_addr_begin && end_addr <= m_mmap_addr_end)
    return { start_addr, end_addr };
  else
    return { };
}

bool
hw_inst_base::data_flash_dev_impl::erase_block (uintptr_t addr)
{
  return erase_block (erase_block_for_addr (addr));
}

bool
hw_inst_base::data_flash_dev_impl::erase_block (const block_info& bi)
{
  // leave the address error checking to the FCU.
  enter_data_pe_mode ();

  FCPSR = 1;
  FSADDR = bi.start_addr ();
  FACI_CMD8 = 0x20;
  FACI_CMD8 = 0xD0;
  bool r = wait_ready ();
  if (!r)
    forced_stop_command ();

  FCPSR = 0;
  enter_read_mode ();
  return r;
}

bool
hw_inst_base::data_flash_dev_impl::erase_all_blocks (void)
{
  bool r = true;

  for (uintptr_t addr = m_mmap_addr_begin; addr < m_mmap_addr_end;
	addr += erase_block_size ())
    r &= erase_block (block_info (addr, addr + erase_block_size ()));

  return r;
}

bool
hw_inst_base::data_flash_dev_impl::is_area_blank (const block_info& bi) const
{
  if (!bi)
    return false;

  enter_data_pe_mode ();

  FBCCNT = 0;  // increment address mode
  FSADDR = bi.start_addr ();
  FEADDR = bi.end_addr () - 1;	// bits [31:19] and [1:0] 1 are ignored.
  FACI_CMD8 = 0x71;		// careful as address can wrap around and
  FACI_CMD8 = 0xD0;		// can check the last and first block.

  // in case of invalid address input it will go into locked error state.
  bool r = wait_ready ();
  if (!r)
    forced_stop_command ();

  enter_read_mode ();
  return r ? FBCSTAT == 0 : false;
}

bool
hw_inst_base::data_flash_dev_impl::write_block (uintptr_t addr, const void* data)
{
  const uint16_t* data16 = (const uint16_t*)data;
  const uintptr_t addr_end = addr + read_write_block_size ();
  bool r = true;

  enter_data_pe_mode ();

  for (; addr != addr_end; addr += 4)
  {
    FSADDR = addr;
    FACI_CMD8 = 0xE8;
    FACI_CMD8 = 2;  // number of following words to be programmed.
    FACI_CMD16 = *data16++;
    FACI_CMD16 = *data16++;
    FACI_CMD8 = 0xD0;

    r = wait_ready ();
    if (!r)
    {
      forced_stop_command ();
      break;
    }
  }
  enter_read_mode ();
  return r;
}

bool
hw_inst_base::data_flash_dev_impl::read_block (uintptr_t addr, void* data)
{
  return std::memcpy (data, (const void*)addr, read_write_block_size ());
}






} // namespace rx64_fcu
} // namespace dev
