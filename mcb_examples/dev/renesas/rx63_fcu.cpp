#include <type_traits>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <thread>

#include <dev/hwreg.hpp>

#define log_all
#include <logging/logging.hpp>

#include "rx63_fcu.hpp"

namespace dev
{
// ---------------------------------------------------------------------------

namespace hw
{
static constexpr hw_reg_rw<uint8_t, const_addr<0x0008C296>> FWEPROR;
static constexpr hw_reg_rw<uint8_t, const_addr<0x007FC402>> FMODR;
static constexpr hw_reg_rw<uint8_t, const_addr<0x007FC410>> FASTAT;
static constexpr hw_reg_rw<uint8_t, const_addr<0x007FC411>> FAEINT;
static constexpr hw_reg_rw<uint8_t, const_addr<0x007FC412>> FRDYIE;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FC440>> DFLRE0;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FC442>> DFLRE1;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FC450>> DFLWE0;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FC452>> DFLWE1;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FC454>> FCURAME;
static constexpr hw_reg_r<uint8_t, const_addr<0x007FFFB0>> FSTATR0;
static constexpr hw_reg_r<uint8_t, const_addr<0x007FFFB1>> FSTATR1;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FFFB2>> FENTRYR;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FFFB4>> FPROTR;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FFFB6>> FRESETR;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FFFBA>> FCMDR;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FFFC8>> FCPSR;
static constexpr hw_reg_rw<uint16_t, const_addr<0x007FFFCA>> DFLBCCNT;
static constexpr hw_reg_r<uint16_t, const_addr<0x007FFFCC>> FPESTAT;
static constexpr hw_reg_r<uint16_t, const_addr<0x007FFFCE>> DFLBCSTAT;
static constexpr hw_reg_rw<uint16_t, const_addr<0x07FFFE8>> PCKAR;
}

#include "rx_fcu_ramfunc.hpp"


// ---------------------------------------------------------------------------

[[gnu::cold]] rx63_fcu::rx63_fcu (unsigned int peripheral_clock_mhz)
: m_data_flash_dev (*this), m_code_flash_dev (*this)
{
  // reset FCU.
  hw::FRESETR = 0xCC01;

  // the reset time is specified in the datasheet as tFCUR min = 35 microsec.
  // it is peripheral/flash clock invariant.
  std::this_thread::sleep_for (std::chrono::microseconds (40));
  hw::FRESETR = 0xCC00;

  // load firmware
  enter_read_mode ();

  // enable FCU RAM access (write only from CPU side).
  hw::FCURAME = 0xC401;

  // copy FCU firmware
  // src: 0xFEFFE000 to 0xFEFFFFFF (FCU firmware area)
  // dst: 0x007F8000 to 0x007F9FFF (FCU RAM area)
  // size: 0x2000 = 8192 bytes
  std::memcpy ((void*)0x007F8000, (void*)0xFEFFE000, 8192);

  // disable FCU RAM access.
  hw::FCURAME = 0xC400;

  // disable lock bit protection
  hw::FPROTR = 0x5501;

  // enable E2 data flash read access (all pages)
  hw::DFLRE0 = 0x2DFF;
  hw::DFLRE1 = 0xD2FF;

  // enable E2 data flash write access (all pages)
  hw::DFLWE0 = 0x1EFF;
  hw::DFLWE1 = 0xE1FF;

  // clear error condition, disable interrupts
  hw::FAEINT = 0;

  set_pe_clock (peripheral_clock_mhz);
}

[[gnu::cold]] void rx63_fcu::set_pe_clock (unsigned int mhz)
{
  // to set the peripheral clock we need to enter any PE mode.
  // use E2 here because it doesn't require execution from RAM.
  enter_data_pe_mode ();

  auto& e2_cmd8 = *((volatile uint8_t*)0x00100000);
  auto& e2_cmd16 = *((volatile uint16_t*)0x00100000);

  // setting the programming clock is required only once, not for every
  // individual program/erase command.
  // maybe should remember the recently set clock and set it only if it
  // changes.  the clock limits for ROM and E2 programming are different...

  mhz = std::max (4u, std::min (50u, mhz));

  // select FCLK for programming / erasure.
  // it seems that for erasure values 1...100 mhz are OK, but
  // for programming only 4 ... 50.
  // in any case, the value is tied to the FlashIF.  if it's different,
  // there might be some hardware damage during programming/erase.
  hw::PCKAR = mhz;

  // send a peripheral notification command sequence.
  e2_cmd8 = 0xE9;
  e2_cmd8 = 0x03;
  e2_cmd16 = 0x0F0F;
  e2_cmd16 = 0x0F0F;
  e2_cmd16 = 0x0F0F;
  e2_cmd8 = 0xD0;

  wait_ready ();

  if (is_error ())
    abort ();

  // go back to normal P/E mode.
  e2_cmd8 = 0xFF;
  wait_ready ();

  if (is_error ())
    abort ();

  enter_read_mode ();
}

// force these to be always inlined because they are used from ROM and from
// RAM functions.
[[gnu::always_inline]] inline void rx63_fcu::wait_ready (void)
{
  // FIXME: When a timeout leads to the FSTATR0.FRDY bit not being set to 1,
  // FRESETR must be used to initialize the FCU.
  while ((hw::FSTATR0 & (1 << 7)) == 0) { }
}

[[gnu::always_inline]] inline bool rx63_fcu::is_error (void)
{
  return (hw::FASTAT & (1 << 4)) != 0;
}

#if 0
static void fcu_print_fstatr0 (void)
{
  log_info ("FCU FSTATR0\n"
	  "  PRGSPD = %d\n"
	  "  ERSSPD = %d\n"
	  "  SUSRDY = %d\n"
	  "  PRGERR = %d\n"
	  "  ERSERR = %d\n"
	  "  ILGLERR = %d\n"
	  "  FRDY = %d\n",
	  FLASH.FSTATR0.BIT.PRGSPD,
	  FLASH.FSTATR0.BIT.ERSSPD,
	  FLASH.FSTATR0.BIT.SUSRDY,
	  FLASH.FSTATR0.BIT.PRGERR,
	  FLASH.FSTATR0.BIT.ERSERR,
	  FLASH.FSTATR0.BIT.ILGLERR,
	  FLASH.FSTATR0.BIT.FRDY);
}

static void fcu_print_fstatr1 (void)
{
  log_info ("FCU FSTATR1\n"
	  "  FLOCKST = %d\n"
	  "  FCUERR = %d\n",
	  FLASH.FSTATR1.BIT.FLOCKST,
	  FLASH.FSTATR1.BIT.FCUERR);
}

bool rx63_fcu::check_error (void)
{
  bool r = is_error ();

//  if (r)
//    printf ("FCU error\n");

  if (FLASH.FASTAT.BIT.CMDLK)
  {
    log_info ("FCU error\n"
	      "  DFLWPE = %d\n"
	      "  DFLRPE = %d\n"
	      "  DFLAE = %d\n"
	      "  CMDLK = %d\n"
	      "  ROMAE = %d\n",
	     FLASH.FASTAT.BIT.DFLWPE,
	     FLASH.FASTAT.BIT.DFLRPE,
	     FLASH.FASTAT.BIT.DFLAE,
	     FLASH.FASTAT.BIT.CMDLK,
	     FLASH.FASTAT.BIT.ROMAE);

    fcu_print_fstatr0 ();
    fcu_print_fstatr1 ();

    for (int i = 0; i < 64; ++i)
      printf ("                ");

    assert_unreachable ();
    return false;
  }
  else
    return true;
}
#endif

void rx63_fcu::enter_read_mode (void)
{
  // enter ROM and E2 data flash high speed read mode.
  // programming and erasure is disabled.

  hw::FENTRYR = 0xAA00;
  asm ("nop");
  asm ("nop");

  // disable P/E of E2 or ROM.
  hw::FWEPROR = 0x02;

  // clear current (error) status flags.
  hw::FASTAT = 0;
}

/*
void fcu_enter_e2_status_read_mode (void)
{
  do FLASH.FENTRYR.WORD = 0xAA00;
    while (FLASH.FENTRYR.WORD != 0x0000);

  FLASH.FWEPROR.BYTE = 0x02;
}
*/

bool rx63_fcu::enter_data_pe_mode (void)
{
  // when in E2 P/E mode ROM read access is OK.  thus we don't need to jump
  // to RAM.

  auto& e2_cmd8 = *((volatile uint8_t*)0x00100000);

  hw::FENTRYR = 0xAA80;
  asm ("nop");
  asm ("nop");
  hw::FASTAT = 0;

  // write status clear cmd to E2 flash area.
  e2_cmd8 = 0x50;

  // enable E2 flash write
  hw::FWEPROR = 0x01;

  wait_ready ();

  if (is_error ())
  {
    enter_read_mode ();
    return false;
  }

  return true;
}

[[__ramfunc_attr__]] rx63_fcu::enter_rom_pe_mode_result
rx63_fcu::enter_rom_pe_mode (rom_pe_area area_sel, uintptr_t addr_begin)
{
  auto& rom_cmd8 = *(volatile uint8_t*)addr_begin;

  // because the interrupt vector table is (usually) located in ROM,
  // we must disable interrupts while in ROM PE mode.  otherwise an interrupt
  // might trigger a ROM read access which will fail.

  // notice also since this function resides in RAM (.data section), we
  // can't safely call other functions which might be not inlined and might
  // not be in RAM but in ROM.  even though we specify the "flatten" function
  // attribute, there is no guarantee that this function will contain only
  // function calls to .data.
  auto prev_psw = dev::this_cpu::save_disable_interrupts ();

  hw::FENTRYR = area_sel | 0xAA00;
  asm ("nop");
  asm ("nop");
  hw::FWEPROR = 0x01;
  hw::FASTAT = 0;

  // write status clear cmd to rom pe address.
  rom_cmd8 = 0x50;

  wait_ready ();

  if (is_error ())
  {
    leave_rom_pe_mode (prev_psw);
    return { prev_psw, false };
  }

  return { prev_psw, true };
}

[[__ramfunc_attr__]] void
rx63_fcu::leave_rom_pe_mode (dev::this_cpu::status_reg_value prev_psw)
{
  enter_read_mode ();
  dev::this_cpu::restore_interrupts (prev_psw);
}

[[__ramfunc_attr__]] rx63_fcu::op_result
rx63_fcu::erase_rom_block_1 (rom_pe_area area_sel, uintptr_t addr_begin)
{
  auto& rom_cmd8 = *(volatile uint8_t*)addr_begin;

  if (auto r = enter_rom_pe_mode (area_sel, addr_begin))
  {
    rom_cmd8 = 0x20;
    rom_cmd8 = 0xD0;

    wait_ready ();

    bool err = is_error ();

    // go back to normal P/E mode.
    rom_cmd8 = 0xFF;

    leave_rom_pe_mode (r.prev_psw);
    return err ? op_result::erase_ng : op_result::ok;
  }
  else
    return op_result::pe_enter_ng;
}

[[__ramfunc_attr__]] rx63_fcu::op_result
rx63_fcu::write_rom_block_1 (rom_pe_area area_sel, uintptr_t addr_begin,
			const uint16_t* data)
{
  auto& rom_cmd8 = *(volatile uint8_t*)addr_begin;
  auto& rom_cmd16 = *(volatile uint16_t*)addr_begin;

  if (auto r = enter_rom_pe_mode (area_sel, addr_begin))
  {
    rom_cmd8 = 0xE8;
    rom_cmd8 = 0x40;

    for (unsigned int i = 0; i < 128 / sizeof (uint16_t); ++i)
      rom_cmd16 = *data++;

    rom_cmd8 = 0xD0;

    wait_ready ();

    bool err = is_error ();

    leave_rom_pe_mode (r.prev_psw);
    return err ? op_result::write_ng : op_result::ok;
  }
  else
    return op_result::pe_enter_ng;
}

bool rx63_fcu::erase_rom_block (const block_info& bi)
{
  return erase_rom_block (bi.start_addr ());
}

bool rx63_fcu::erase_rom_block (uintptr_t rom_read_addr)
{
  auto pe_addr = make_rom_pe_addr (rom_read_addr);

//  __force_reg__ (erase_rom_block_1);
//  auto r = erase_rom_block_1 (pe_addr.area, pe_addr.addr);

  auto r = invoke_ramfunc (erase_rom_block_1, pe_addr.area, pe_addr.addr);

  if (r == op_result::ok)
    return true;
  else
    return false;
}

bool rx63_fcu::write_rom_block (uintptr_t rom_read_addr, const void* src_data)
{
  auto pe_addr = make_rom_pe_addr (rom_read_addr);

//  __force_reg__ (write_rom_block_1);
//  auto r = write_rom_block_1 (pe_addr.area, pe_addr.addr, (const uint16_t*)src_data);

  auto r = invoke_ramfunc (write_rom_block_1,
			   pe_addr.area, pe_addr.addr, (const uint16_t*)src_data);

  if (r == op_result::ok)
    return true;
  else
    return false;
}

bool rx63_fcu::read_rom_block (uintptr_t rom_read_addr, void* dst_data) const
{
  std::memcpy (dst_data, (const void*)rom_read_addr, 128);
  return true;
}

bool rx63_fcu::is_blank_rom_area (const block_info& bi) const
{
  // ROM blank checking is like classical NOR flash blank checking.  after
  // erasing one block all bits go to '1'.

  // this a bit smaller than std::find_if.
  const uint32_t* p = (const uint32_t*)(bi.start_addr () & ~3u);
  auto count = bi.size () / sizeof (uint32_t);

  for (decltype (count) i = 0; i < count; ++i)
    if (*p++ != 0xFFFFFFFF)
      return false;

  return true;
}

void rx63_fcu::rom_read_to_pe_static_checks (void)
{
  static_assert (make_rom_pe_addr (0xFFFFFFFF).area == rom_pe_area_0, "");
  static_assert (make_rom_pe_addr (0xFFFC0000).area == rom_pe_area_0, "");
  static_assert (make_rom_pe_addr (0xFFFA0000).area == rom_pe_area_0, "");
  static_assert (make_rom_pe_addr (0xFFF80000).area == rom_pe_area_0, "");
  static_assert (make_rom_pe_addr (0xFFF40000).area == rom_pe_area_1, "");
  static_assert (make_rom_pe_addr (0xFFF7FFFF).area == rom_pe_area_1, "");
  static_assert (make_rom_pe_addr (0xFFF00000).area == rom_pe_area_1, "");
  static_assert (make_rom_pe_addr (0xFFE80000).area == rom_pe_area_2, "");
  static_assert (make_rom_pe_addr (0xFFEFFFFF).area == rom_pe_area_2, "");
  static_assert (make_rom_pe_addr (0xFFEFFFFF).area == rom_pe_area_2, "");
  static_assert (make_rom_pe_addr (0xFFE00000).area == rom_pe_area_3, "");
  static_assert (make_rom_pe_addr (0xFFE7FFFF).area == rom_pe_area_3, "");
}

rx63_fcu::block_info rx63_fcu::data_erase_block_for_addr (uintptr_t data_cpu_addr)
{
  auto start_addr = data_block_addr (data_cpu_addr);
  auto end_addr = start_addr + write_data_block_size ();

  if (start_addr >= 0x00100000 && end_addr <= 0x00100000 + 32*1024)
    return { start_addr, end_addr };
  else
    return { };
}

bool rx63_fcu::erase_data_block (const block_info& bi)
{
  return erase_data_block (bi.start_addr ());
}

bool rx63_fcu::erase_data_block (uintptr_t data_cpu_addr)
{
  if (!enter_data_pe_mode ())
    return false;

  auto cmd_addr = data_block_addr (data_cpu_addr);
  *(volatile uint8_t*)cmd_addr = 0x20;
  *(volatile uint8_t*)cmd_addr = 0xD0;

  wait_ready ();

  bool err = is_error ();
  enter_read_mode ();
  return !err;
}

bool rx63_fcu::erase_all_data_blocks (void)
{
  if (!enter_data_pe_mode ())
    return false;

  for (uintptr_t a = 0x00100000; data_erase_block_for_addr (a);
       a += write_data_block_size ())
    erase_data_block (a);

  return true;
}

bool rx63_fcu::is_blank_data_area (const block_info& bi) const
{
  if (!enter_data_pe_mode ())
    return false;

  // select cmd 0x71 blank check
  hw::FMODR = (1 << 4);

  bool is_blank = true;

  for (auto addr = bi.start_addr () & ~1u;
	addr < bi.end_addr () && is_blank; addr += 2)
  {
    // we can do a blank check of 2 bytes or 2 kbytes.
    // we are going to use 2 byte checks and thus we need to specify the
    // 2 kb area address to be checked.
    auto cmd_addr = addr & ~0x7FF;

    hw::DFLBCCNT = addr & 0x7FE;

    // write blank check command
    *(volatile uint8_t*)cmd_addr = 0x71;
    *(volatile uint8_t*)cmd_addr = 0xD0;

    wait_ready ();
    if (is_error ())
    {
      // FIXME: report a non-blank block in case of error.
      // not sure what to do here... should actually abort.
      is_blank = false;
    }

    is_blank &= hw::DFLBCSTAT == 0;
  }

  // enter normal P/E mode
  auto& e2_cmd8 = *((volatile uint8_t*)0x00100000);
  e2_cmd8 = 0xFF;

  wait_ready ();
  bool err = is_error ();

  enter_read_mode ();
  return is_blank & !err;
}

bool rx63_fcu::write_data_block (uintptr_t data_cpu_addr, const void* src_data)
{
  const uint16_t* src = (const uint16_t*)src_data;

  if (!enter_data_pe_mode ())
    return false;

  bool err = false;

  for (unsigned int i = 0; i < write_data_block_size (); i += 2)
  {
    auto cmd_addr = data_block_addr (data_cpu_addr) + i;
    auto& cmd8 = *(volatile uint8_t*)cmd_addr;
    auto& cmd16 = *(volatile uint16_t*)cmd_addr;

    auto val = *src++;

    cmd8 = 0xE8;
    cmd8 = 0x01;
    cmd16 = val;
    cmd8 = 0xD0;

    wait_ready ();

    err = is_error ();
    if (err)
      break;
  }

  enter_read_mode ();
  return !err;
}

bool rx63_fcu::read_data_block (uintptr_t data_cpu_addr, void* dst_data) const
{
  std::memcpy (dst_data, (const void*)data_cpu_addr, write_data_block_size ());
  return true;
}

} // namespace dev
