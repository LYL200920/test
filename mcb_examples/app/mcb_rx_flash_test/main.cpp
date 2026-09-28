
#include <cstdio>
#include <memory>
#include <cstring>
#include <cassert>
#include <chrono>

#include <board/board.hpp>

#define log_all
#include <logging/logging.hpp>

class rx_e2_flash
{
public:
  static void init (void);

  // by using functions instead of constants it is possible to create an
  // adapter class which resolves the values at runtime.

  constexpr size_t block_size_bytes (void) const { return 32; }

  constexpr size_t size_bytes (void) const { return (0x00107FFF - 0x00100000) + 1; }
  constexpr size_t size_blocks (void) const { return size_bytes () / block_size_bytes (); }


  const void* block_read_ptr (size_t block) const { return (const void*)(base_address () + block * block_size_bytes ()); }

  // on a normal NOR flash after block erasure all bytes are 0xFF.  a block
  // is blank if all its bytes are 0xFF.  the RX E2 data flash does not reset
  // the bit values to '1' when it is erased.  instead, it keeps the blank
  // state in separate hidden bits.
  bool is_blank_block (size_t block) const;

  bool erase_block (size_t block);

  // assuming that the block is blank, write the specified data to the flash.
  bool write_block (size_t block, const void* data_in);

  bool read_block (size_t block, void* data_out);

  constexpr uintptr_t base_address (void) const { return 0x00100000; }

  constexpr uintptr_t block_address (size_t n) const
  {
    return base_address () + block_size_bytes () * n;
  }

private:
};

void rx_e2_flash::init (void)
{
}

bool rx_e2_flash::is_blank_block (size_t block) const
{
  return this_board::inst ().fcu.is_blank_data_block (block_address (block));
}

bool rx_e2_flash::erase_block (size_t block)
{
  return this_board::inst ().fcu.erase_data_block (block_address (block));
}


bool rx_e2_flash::write_block (size_t block, const void* data_in)
{
  return this_board::inst ().fcu.write_data_block (block_address (block), data_in);
}

bool rx_e2_flash::read_block (size_t block, void* data_out)
{
  return this_board::inst ().fcu.read_data_block (block_address (block), data_out);
}

template <size_t N>
static void print_block (const char(&a)[N])
{
  for (unsigned int i = 0; i < N; ++i)
    printf (i == 0 ? "%02x" : " %02x", a[i]);
}


// --------------------------------------------------------------

static auto&& led_out = this_board::inst ().led_outputs;

int main (void)
{
  led_out.write (0b0000);

  log_level::enable (log_level::info);

  auto prev_time = std::chrono::high_resolution_clock::now ();
  auto last_trigger_time = prev_time;

  bool led_stat = true;

#if 1
  rx_e2_flash::init ();

  rx_e2_flash fl;

{
  char buf[fl.block_size_bytes ()];

  printf ("read block: ");
  fl.read_block (0, buf);
  print_block (buf);
  printf ("\n");


  printf ("check blank block 1\n");
  printf ("block %u (0x%08x) blank = %d\n", 0, fl.block_address (0), fl.is_blank_block (0));

  printf ("erase block (%u): %s\n", 0, fl.erase_block (0) ? "OK" : "NG");

  printf ("check blank block 2\n");
  printf ("block %u (0x%08x) blank = %d\n", 0, fl.block_address (0), fl.is_blank_block (0));

  {
    uint8_t write_data[32];
    for (int i = 0; i < 32; ++i)
      write_data[i] = i;

    printf ("write block (%u): %s\n", 0, fl.write_block (0, write_data) ? "OK" : "NG");
  }

  printf ("check blank block 4\n");
  printf ("block %u (0x%08x) blank = %d\n", 0, fl.block_address (0), fl.is_blank_block (0));
/*
  {
    uint8_t write_data[32];
    for (int i = 0; i < 32; ++i)
      write_data[i] = 0xFF;

    printf ("write block (%u): %s\n", 0, fl.write_block (0, write_data) ? "OK" : "NG");
  }

  printf ("check blank block 5\n");
  printf ("block %u (0x%08x) blank = %d\n", 0, fl.block_address (0), fl.is_blank_block (0));
*/
  printf ("read block: ");
  fl.read_block (0, buf);
  print_block (buf);
  printf ("\n");


/*
  for (unsigned int i = 0; i < fl.size_blocks (); ++i)
    printf ("block %u (0x%08x) blank = %d\n", i, fl.block_address (i), fl.is_blank_block (i));
*/
}


static const uintptr_t rom_addrs[] = { 0xFFF80000 + 128*0, 0xFFF80000 + 128*1, 0xFFF80000 + 128*2 };

for (uintptr_t rom_addr : rom_addrs)
{
  char buf2[128];

  dev::rx63_fcu::block_info bi = this_board::inst ().fcu.rom_block_for_addr (rom_addr);

  printf ("read rom block 0x%08x (0x%08x, 0x%08x, %u): ", rom_addr, bi.start_addr (), bi.end_addr (), bi.size ());
  this_board::inst ().fcu.read_rom_block (rom_addr, buf2);

  print_block (buf2);
  printf ("\n\n");

  printf ("is blank block 0x%08x: %d\n", rom_addr, this_board::inst ().fcu.is_blank_rom_block (rom_addr));

  printf ("erase rom block 0x%08x...", rom_addr);
  bool erase_result = this_board::inst ().fcu.erase_rom_block (rom_addr);
  printf (" erase_result = %d\n", erase_result);

  printf ("is blank block 0x%08x: %d\n", rom_addr, this_board::inst ().fcu.is_blank_rom_block (rom_addr));

  printf ("read rom block 0x%08x: ", rom_addr);
  this_board::inst ().fcu.read_rom_block (rom_addr, buf2);
  print_block (buf2);
  printf ("\n\n");

  printf ("write rom block 0x%08x...", rom_addr);
  for (int i = 0; i < sizeof (buf2); ++i)
    buf2[i] = i;

  bool write_result = this_board::inst ().fcu.write_rom_block (rom_addr, buf2);
  printf (" write_result = %d\n", write_result);

  printf ("is blank block 0x%08x: %d\n", rom_addr, this_board::inst ().fcu.is_blank_rom_block (rom_addr));

  printf ("read rom block 0x%08x: ", rom_addr);
  this_board::inst ().fcu.read_rom_block (rom_addr, buf2);
  print_block (buf2);
  printf ("\n\n");
}

#endif


  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    auto cur_time = std::chrono::high_resolution_clock::now ();

    if (cur_time - last_trigger_time > std::chrono::milliseconds (500))
    {
      this_board::inst ().exec ();
//      printf ("hello\n");

      led_stat = !led_stat;
      led_out.write (utils::set_bit (led_out.read (), 0, led_stat));

      last_trigger_time = cur_time;
    }

    prev_time = cur_time;
  }

  return 0;
}
