
/*

MCB bootloader programmer utility

this is used for programming the user boot flash area of the RX63N on the MCB.
the user boot flash area contains the boot loader (e.g. TFTP) and information
about the board configuration/options and ethernet MAC address information.

this tool reads the binary user boot flash image and rewrites the board_info
data.  the following operations are possible:

- program the initial bootloader (the RX63N does not contain the bootloader)
     the board_info fields are taken from the command line and the image
     is written into the user boot flash.

- update board_info fields
     the user boot flash is downloaded from the RX63N, the board_info data
     is overwritten with the command line values and the resulting image
     is written back to the RX63N user boot flash.

- update bootloader
     the user boot flash is downloaded from the RX63N and the board_info data
     is extracted.  the old board_info data is then copied into the new
     bootloader image and the image is written to the RX63N user boot flash.

*/


/*

---------------------------
read current image and display info:

./mcb_bootloader_prog /dev/ttyUSB1 115200

(writes current image to user_boot.bin.old)

---------------------------
write new image and set the board info parameters:

./mcb_bootloader_prog /dev/ttyUSB1 115200 img=../build_tftp_bootloader/tftp_bootloader.bin \
board_info.board_id=00000001 \
board_info.features=000000000000000000000000 \
board_info.ifconfig0.hw_addr=021953061100 \
board_info.ifconfig0.ipv4_addr=192.168.0.80 \
board_info.ifconfig0.ipv4_netmask=255.255.255.0

(writes current image to user_boot.bin.old,
 writes new image to user_boot.bin.new before erasing/programming)

---------------------------
write new image and replace board info parameters (overwrite everything):

./mcb_bootloader_prog /dev/ttyUSB1 115200 img_raw=../build_tftp_bootloader/tftp_bootloader.bin

(writes current image to user_boot.bin.old,
 writes new image to user_boot.bin.new before erasing/programming)

---------------------------
write new image and keep old board info parameters:

./mcb_bootloader_prog /dev/ttyUSB1 115200 img=../build_tftp_bootloader/tftp_bootloader.bin

(writes current image to user_boot.bin.old,
 writes new image to user_boot.bin.new before erasing/programming)

---------------------------
keep old image and update board info parameters:

./mcb_bootloader_prog /dev/ttyUSB1 115200 \
board_info.board_id=00000001 \
board_info.features=000000000000000000000000 \
board_info.ifconfig0.hw_addr=021953061100 \
board_info.ifconfig0.ipv4_addr=192.168.0.80 \
board_info.ifconfig0.ipv4_netmask=255.255.255.0

(writes current image to user_boot.bin.old,
 writes new image to user_boot.bin.new before erasing/programming)

*/

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <array>
#include <signal.h>

#include <sys/types.h>
#include <sys/time.h>

#include "dj-flash-tool/serial.h"
#include "dj-flash-tool/rxlib.h"

#include "version_tag/version_tag.hpp"
#include "board/board_info.hpp"
#include "utils/byte_order.hpp"
#include "utils/text.hpp"

static constexpr uint32_t RX_USER_BOOT_FLASH_ADDR = 0xFF7FC000;
static constexpr uint32_t RX_USER_BOOT_FLASH_SIZE = 1024*16;
static constexpr uint32_t RX_USER_BOOT_BOARD_INFO_ADDR = 0xFF7FFF84;

int verbose = 0;

static unsigned int
read_userboot_memory_size ()
{
  int n;
  RxMemRange *a;
  rxlib_inquire_boot_flash_size (&n, &a);

  if (verbose > 1/* || test_connection*/)
    for (int i=0; i<n; i++)
      fprintf(stderr, "UBootMem[%02d]:  0x%08lx - 0x%08lx, size 0x%08lx (%ld kb)\n",
	      i, a[i].start, a[i].end, a[i].size, a[i].size/1024);

  if (n != 1)
    return 0;

  return a[0].size;
}

static void
read_part_number ()
{
  int nparts, i;
  char **codes;
  char **products;

  rxlib_inquire_device (&nparts, &codes, &products);
  if (verbose > 1 /*|| test_connection*/)
    for (i=0; i<nparts; i++)
      fprintf(stderr, "Chip %d: code %s product %s\n",
	      i, codes[i], products[i]);

  std::cout << "chip 0: " << codes[0] << " " << products[0] << std::endl;
  rxlib_select_device (codes[0]);
}

static void
read_clock_modes ()
{
  int n, i;
  int *m;
  rxlib_inquire_clock_modes (&n, &m);
  for (i=0; i<n; i++)
    fprintf(stderr, "Clock mode[%d]: %02x\n", i, m[i]);
  rxlib_select_clock_mode (n ? m[0] : 0);
}


uint32_t read32_le (const void* d)
{
  auto x = *(const uint32_t*)__builtin_assume_aligned (d, 4);
  return utils::to_native (utils::little_endian, x);
}

board_info* get_board_info (void* user_boot_flash_data)
{
  // the address of the board_info struct in the image is stored as little endian

  uint32_t addr = read32_le (
	(const uint8_t*)user_boot_flash_data
	+ (RX_USER_BOOT_BOARD_INFO_ADDR - RX_USER_BOOT_FLASH_ADDR));

  if (addr >= RX_USER_BOOT_FLASH_ADDR
      && addr < RX_USER_BOOT_FLASH_ADDR + RX_USER_BOOT_FLASH_SIZE)
  {
    return (board_info*)((uint8_t*)user_boot_flash_data
			 + (addr - RX_USER_BOOT_FLASH_ADDR));
  }

  return nullptr;
}

template <typename T, size_t N>
std::string to_hexstring (const std::array<T, N>& d)
{
  std::string res;

  const size_t byte_count = N * sizeof (T);
  res.reserve (byte_count * 2);

  static const char* hex_str_map = "0123456789ABCDEF";

  const uint8_t* d8 = (const uint8_t*)d.data ();
  for (size_t i = 0; i < byte_count; ++i)
  {
    res.push_back (hex_str_map[ d8[i] >> 4 ]);
    res.push_back (hex_str_map[ d8[i] & 0x0F ]);
  }

  return res;
}

std::string ipv4_addr_to_string (const std::array<uint8_t, 4>& a)
{
  char tmp[64];
  std::sprintf (tmp, "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
  return tmp;
}

std::ostream& operator << (std::ostream& out, const version_tag& vtag)
{
  return out << "version_tag:"
	     << "\n   product:     " << vtag.product_name ()
	     << "\n   tag:         " << vtag.tag ()
	     << "\n   num:         " << vtag.num ()
	     << "\n   tick:        " << vtag.tick ()
	     << "\n   aux:         " << vtag.aux ()
	     << "\n   copyright:   " << vtag.copyright ()
	     << "\n   repository:  " << vtag.repository_uuid ()
	     << "\n   commit hash: " << vtag.commit_hash ()
	     << "\n   commit time: " << vtag.commit_time ()
	     << "\n   build hash:  " << vtag.build_hash ();
}

std::ostream& operator << (std::ostream& out, const board_info& binfo)
{
  out << "board info:"
      << "\n   board ID: " << to_hexstring (binfo.board_id_bytes ())
      << "\n   features: " << to_hexstring (binfo.features ().bytes ());

  for (unsigned int i = 0; i < binfo.ifconfigs ().size (); ++i)
  {
    out << "\n   ifconfig" << i << ":";

    const auto& c = binfo.ifconfigs ()[i];
    out << "\n     hw_addr      : " << to_hexstring (c.hw_addr ())
	<< "\n     ipv4_addr    : " << ipv4_addr_to_string (c.ipv4_addr ())
	<< "\n     ipv4_netmask : " << ipv4_addr_to_string (c.ipv4_netmask ());
  }

  return out;
}

template <size_t N>
std::array<uint8_t, N> parse_hexstring (const std::string& a)
{
  std::array<uint8_t, N> res;
  res.fill (0);

  const size_t max_chars = N * 2;

  for (size_t i = 0; i < max_chars && i < a.size () ; ++i)
  {
    char c = a[i];
    uint8_t val;

    if (c >= '0' && c <= '9')
      val = c - '0';
    else if (c >= 'A' && c <= 'F')
      val = c - 'A' + 0x0A;
    else if (c >= 'a' && c <= 'f')
      val = c - 'a' + 0x0A;
    else
    {
      std::cerr << "error: invalid hexstring character " << c << std::endl;
      exit (1);
    }

    res[i >> 1] |= (i & 1) ? val : (val << 4);
  }

  return res;
}


std::array<uint8_t, 4> parse_ipv4_addr (const std::string& a)
{
  int b[4] = { -1, -1, -1, -1 };

  std::sscanf (a.c_str (), "%u.%u.%u.%u", &b[0], &b[1], &b[2], &b[3]);

  for (int x : b)
    if (x > 255 || x < 0)
    {
      std::cerr << "error: invalid ipv4 address: " << a << std::endl;
      exit (1);
    }

  return { uint8_t (b[0]), uint8_t (b[1]), uint8_t (b[2]), uint8_t (b[3]) };
}


std::array<uint8_t, RX_USER_BOOT_FLASH_SIZE> flash_data;

int main (int argc, const char* argv[])
{
  if (argc < 3)
  {
    std::cerr << "error: not enough arguments" << std::endl;
    std::cerr << utils::printf_format (
R"(usage example: mcb_bootloader_prog <serial dev> <baud rate> [key=value]*
supported key values:

  img=<file name>
    sets the bootloader binary image to be programmed. setting only the image
    will preserve the board_info configuration settings that have been already
    programmed.  this is done by first downloading the image from the board,
    extracting the board_info parameters and inserting them into the new image.

    if the image is not specified, the already programmed image will be used.
    this allows modification of board_info parameters without changing the
    bootloader software.

  img_raw=<file name>
    use the specified binary image as a raw image for programming.  the raw
    image is written 1:1 into the flash memory, without any modifications.

  board_info.board_id=<hex string>
    sets the board ID number.  the value is max. %d bytes.  if a shorter
    value is specified, it will be padded with zeros.

  board_info.features=<hex string>
    sets the board feature bits.  there are max. %d bits, i.e. %d bytes.  if
    a shorter value is specified, it will be padded with zeros.

  board_info.ifconfig<N>
    set the parameters of the N-th network interface configuration.  N can be
    in the range 0..%d.  the following interface parameters are available:

    .hw_addr=<hex string>      hardware (MAC) address, max %d bytes.
    .ipv4_addr=<ipv4 addr>     default IPv4 address
    .ipv4_netmask=<ipv4 addr>  default IPv4 netmask
)",
    (int)board_info ().board_id_bytes ().size (),
    (int)board_info::feature_bits::bit_count,
    (int)board_info ().features ().bytes ().size (),
    (int)board_info ().ifconfigs ().size (),
    (int)board_info::ifconfig ().hw_addr ().size ()
) << std::endl;

    exit (1);
  }

  signal (SIGINT, [] (int)
  {
    std::cerr << std::endl << "aborting" << std::endl;
    exit (1);
  });


  std::cout << "connecting via " << argv[1] << std::endl;

  serial_init (argv[1], 19200);

  rxlib_init ();
  rxlib_reset ();

  rxlib_inquire_operating_frequency ();

  const unsigned int chip_user_boot_size = read_userboot_memory_size ();
  if (chip_user_boot_size != flash_data.size ())
  {
    std::cerr << "error: user boot flash size " << chip_user_boot_size << " NG"
	      << std::endl;
    exit (1);
  }

  int programming_size = 0;
  rxlib_inquire_programming_size (&programming_size);

  read_part_number ();
  read_clock_modes ();

  int use_baud = std::stoi (argv[2]);

  std::cout << "setting baud rate to " << use_baud << std::endl;
  rxlib_set_baud_rate (use_baud);

  // this will erase all data in the chip by the SCI1 bootloader.
  // the user boot area is usually not erased
  // (if it contains the UsbBoot magic numbers)
  rxlib_data_setting_complete ();

  const auto user_boot_flash_size = read_userboot_memory_size ();

  if (user_boot_flash_size != flash_data.size ())
  {
    std::cerr << "error: unexpected user boot flash size: "
	      << user_boot_flash_size << std::endl;
    exit (1);
  }

  std::cout << "reading user boot flash" << std::endl;
  [[gnu::unused]] bool flash_data_blank;
  bool write_back_flash = false;


  if (rxlib_blank_check_boot () == RX_ERR_NONE)
  {
    std::cout << "user boot flash is blank" << std::endl;
    flash_data.fill (0xFF);
    flash_data_blank = true;
  }
  else
  {
    // the rxlib reads from the serial port one byte at a time.  this doesn't
    // really work on win32 with baud rates > 19200 or so.  if we tell the
    // RX to read the whole flash memory at once
    const size_t read_block_size = 256;
    for (size_t i = 0; i < flash_data.size (); i += read_block_size)
    {
      if (rxlib_read_memory (RX_USER_BOOT_FLASH_ADDR + i, read_block_size,
			     flash_data.data () + i) != RX_ERR_NONE)
      {
        std::cerr << "error: user boot flash read" << std::endl;
        exit (1);
      }
    }

    std::fstream fout ("user_boot.bin.old", std::ios::binary | std::ios::out);
    fout.write ((const char*)flash_data.data (), flash_data.size ());

    flash_data_blank = false;
  }


  // get the version tag and board info
  version_tag vtag = version_tag (flash_data.data ());
  board_info binfo;
  [[gnu::unused]] bool version_tag_valid= false;
  bool binfo_valid = false;

  if (vtag.product_name ().empty ())
    std::cout << "version tag not found" << std::endl;
  else
  {
    std::cout << vtag << std::endl;

    binfo = *get_board_info (flash_data.data ());
    binfo_valid = true;
    version_tag_valid = true;
  }

  if (!binfo_valid)
    std::cout << "board info not found" << std::endl;
  else
    std::cout << binfo << std::endl;


  for (int arg_num = 3; arg_num < argc; ++arg_num)
  {
    std::string a = argv[arg_num];

    bool do_img = a.find ("img=") == 0;
    bool do_img_raw = a.find ("img_raw=") == 0;

    if (do_img || do_img_raw)
    {
      auto val = a.substr (a.find ('=') + 1);
      std::cout << "loading image file " << val << std::endl;

      std::fstream fin (val, std::ios::binary | std::ios::in);
      fin.read ((char*)flash_data.data (), flash_data.size ());

      if (!fin)
      {
	std::cerr << "error: reading file" << std::endl;
	exit (1);
      }

      vtag = version_tag (flash_data.data ());
      if (vtag.product_name ().empty ())
      {
	std::cerr << "error: image has no version tag" << std::endl;
	exit (1);
      }

      version_tag_valid = true;

      std::cout << vtag << std::endl;

      if (do_img_raw)
      {
	// overwrite everything from the image as-is.
	write_back_flash = true;
	break;
      }
      else
      {
	// we have overwritten the board info in the flash data with the new
	// image.  restore the old board info in the new flash data.
	auto* new_bi = get_board_info (flash_data.data ());
	if (new_bi == nullptr)
	{
	  std::cerr << "error: image has no board info area" << std::endl;
	  exit (1);
	}

	*new_bi = binfo;
	write_back_flash = true;
      }
    }

    if (a.find ("board_info.") == 0)
    {
      a = a.substr (std::strlen ("board_info."));

      if (a.find ("board_id=") == 0)
      {
	a = a.substr (std::strlen ("board_id="));

	auto val = parse_hexstring<4> (a);
	std::cout << "setting board_info.board_id = "
		  << to_hexstring (val) << std::endl;

	binfo.set_board_id (val);
      }
      else if (a.find ("features=") == 0)
      {
	a = a.substr (std::strlen ("features="));

	auto val = parse_hexstring<board_info::feature_bits::bit_count / 8> (a);
	std::cout << "setting board_info.features = "
		  << to_hexstring (val) << std::endl;

	binfo.set_features (val);
      }
      else if (a.find ("ifconfig") == 0)
      {
	a = a.substr (std::strlen ("ifconfig"));
	if (a.empty ())
	{
	  std::cerr << "error: ifconfig arg" << std::endl;
	  exit (1);
	}

	int ifconfig_num = a.front () - '0';
	if (ifconfig_num < 0 || ifconfig_num >= (int)binfo.ifconfigs ().size ())
	{
	  std::cerr << "error: ifconfig number " << ifconfig_num << " out of range" << std::endl;
	  exit (1);
	}

	a = a.substr (1);

	if (a.find (".hw_addr=") == 0)
	{
	  a = a.substr (std::strlen (".hw_addr="));

	  std::array<uint8_t, 8> val = parse_hexstring<8> (a);
	  std::cout << "setting ifconfig" << ifconfig_num << ".hw_addr = "
		    << to_hexstring (val) << std::endl;

	  binfo.ifconfigs ()[ifconfig_num].set_hw_addr (val);
	}
	else if (a.find (".ipv4_addr=") == 0)
	{
	  a = a.substr (std::strlen (".ipv4_addr="));

	  std::array<uint8_t, 4> val = parse_ipv4_addr (a);
	  std::cout << "setting ifconfig" << ifconfig_num << ".ipv4_addr = "
		    << ipv4_addr_to_string (val) << std::endl;

	  binfo.ifconfigs ()[ifconfig_num].set_ipv4_addr (val);
	}
	else if (a.find (".ipv4_netmask=") == 0)
	{
	  a = a.substr (std::strlen (".ipv4_netmask="));

	  std::array<uint8_t, 4> val = parse_ipv4_addr (a);
	  std::cout << "setting ifconfig" << ifconfig_num << ".ipv4_netmask = "
		    << ipv4_addr_to_string (val) << std::endl;

	  binfo.ifconfigs ()[ifconfig_num].set_ipv4_netmask (val);
	}
	else
	{
	  std::cerr << "error: invalid ifconfig field " << a << std::endl;
	  exit (1);
	}
      }
      else
      {
	std::cerr << "error: invalid board_info field" << a << std::endl;
	exit (1);
      }

      // assume that all the checks above were OK and the field has been
      // set to 'binfo'.  now update the flash data image.
      if (auto* bi = get_board_info (flash_data.data ()))
      {
	*bi = binfo;
         write_back_flash = true;
      }
      else
      {
	std::cerr << "error: could not write board info to flash image" << std::endl;
	exit (1);
      }
    }
  }

  if (!write_back_flash)
    return 0;

  std::cout << "writing user boot flash" << std::endl;

  {
    // in case rewriting the image goes wrong, write it to disk first.
    // the user then has a chance to retry the operation with "img_only=".
    std::fstream fout ("user_boot.bin.new", std::ios::binary | std::ios::out);
    fout.write ((const char*)flash_data.data (), flash_data.size ());
    if (!fout)
    {
      std::cerr << "error: could not write user_boot.bin.new" << std::endl;
      exit (1);
    }
  }

  if (rxlib_blank_check_boot () != RX_ERR_NONE)
  {
    std::cout << "user boot area is not blank.  erasing it." << std::endl;
    if (rxlib_erase_boot_flash () != RX_ERR_NONE)
    {
      std::cerr << "error: can't erase user boot flash." << std::endl;
      exit (1);
    }

    if (rxlib_blank_check_boot () != RX_ERR_NONE)
    {
      std::cerr << "error: user boot area not blank after erasure." << std::endl;
      exit (1);
    }
  }

  rxlib_begin_boot_flash ();

  if (flash_data.size () % (unsigned int)programming_size != 0)
  {
    std::cerr << "error: programming flash size NG." << std::endl;
    exit (1);
  }

  for (size_t i = 0; i < flash_data.size (); i += programming_size)
  {
    std::cout << "\rwriting block " << i << "                       ";
    std::cout.flush ();

    if (rxlib_program_block (RX_USER_BOOT_FLASH_ADDR + i, flash_data.data () + i)
	!= RX_ERR_NONE)
    {
      std::cerr << "\rerror: program block " << i << " NG                      "
		<< std::endl;

      rxlib_done_programming ();
      exit (1);
    }
  }

  std::cout << "\rdone                       " << std::endl;
  rxlib_done_programming ();
  return 0;
}
