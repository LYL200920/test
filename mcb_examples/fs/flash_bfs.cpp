/*

general overview
----------------

this filesystem has been designed for the following criteria:
- works in small storage (16 KB, 32 KB flash)
- storage of small files
- flash wear leveling
- some degree of fault tolerance against power outage during flash write access
- flash memory is mapped into a linear address space of the CPU

the flash storage is subdivided into partitions and each partition is subdivided
into equally sized blocks.

we assume that it is only possible to write to a new block once after it has
been erased.
this is not exactly true for most NOR flash memories, as they allow writing
0s over 1s in the same block several times.  however, some on-chip MCU flash
memories like the RX data flash have some additional restrictions on that.
so the assumption here is that a block can be written only once.

as a consequence, operations on the filesystem are recorded by always appending
new blocks.  for example, if data in a file is to be overwritten, a new block
for that file is appended and the old block remains in the flash.  when reading
the file data, always the most recent blocks are used.  this automatically
results in a journaling file system.  if a file write operation fails, the
old data will still be available.

at some point the whole flash memory is utilized and it has to be garbage
collected to make blocks available again.  this process consists of erasing
blocks in the flash memory and copying around the most recent valid blocks.
this garbage collection process should be fault tolerant.  so that, if there is
a power-outage (or other failure) in the middle of the garbage collection,
the data can still be recovered.

one block in the filesystem partition has the following structure:

     [ integer : file ID ]
     [ integer : block ID ]
        <  payload data >
     [ integer : checksum ]

theoretically, it's possible to use different integer sizes, but at the moment
only one block type is implemented, which uses 1 byte for the file ID, 1 byte
for the block ID and 2 bytes for the checksum fields.  hence the name
"block_type_1_1_2".  the sizes of the integers have an impact on the maximum
number of files that can be stored in a partition and the maximum file size.
the size of the payload data is defined in the partition header and can be
varied by the application when it creates a new partition.

the file ID field identifies the file in the partition.  a file can consists
of multiple data blocks.  so each block that belongs to the same file will have
the same integer ID.  the integer file ID is the primary means of identifying
files in the partition.  text file names are optional.  if the file ID is only
1 byte, the maximum number of different files that can reside in the partition
is 255.

the block ID is effectively the block offset position of the data in the file.
some block ID numbers are reserved for special filesystem operations:

  file_op_set_filename = 0xF0
    this renames a file and gives it a text name.  the data in the block
    is the text string.  the filename is limited by the data block size.

  file_op_set_filesize = 0xF1
    this resizes a file.

  file_op_erase_file = 0xF2,
    undefine / erase a file.

because of those special block IDs, the maximum block ID number in a file
is 0xEF (239).  a block size of 32 bytes results in 28 bytes of user data.
so the maximum file size is 240 * 28 = 6720 bytes.

the checksum field uses CRC to calculate the checksum of the block.
when reading blocks, the CRC is calculated and compared against the stored
value.  if it does not match, the data is considered to be broken in some way
and the block is treated as a bad block.  such bad blocks can be the result of
sudden power loss during flash writing or dead flash cells.


creating a file
---------------

file creation is implicit.  when scanning all blocks of the partition, a
block with a file ID implicitly defines a new file.  so to create a new
file, all that is needed is to append a new block with a file ID that has
not been used yet in the partition.  a named file can be created by appending
a block with a new file ID and the block ID 0xF0.  because a 0xF0 block sets
the filename it does not contain any file data.  so the created file will be
initially empty.


opening a file
--------------

when opening a file, the blocks for that file and the block data offsets need
to be collected and written to a block table in RAM.  when reading data, that
table is used to find the block address for a particular file byte position.
this of course occupies RAM for each opened file.  it could be improved by
implementing some sort of dynamic block cache mechanism and looking up blocks
as needed.


bad block treatment
-------------------

if a flash error is detected during programming the block is marked as bad.
this is usually done by keeping a separate bit vector at the start/end of
the partition.  in NOR flashes, programming is done by writing 0 bits, which
is always possible.  the RX data flash can't do that though.  thus, we ignore
the issue.  if block programming fails, the CRC field will be wrong (normally)
and thus we automatically know that the block is unused the next time it is
accessed.  we can retry to program the block data into the next free block.


garbage collection
-------------------

this is done by moving blocks of same file ID from high addresses to lower
addresses.  if one of the lower address blocks fails to program, the next
higher block is selected.  after moving a block "up" all the blocks with the
same ID that follow the new block are erased.  erasure of obsoleted blocks
happens top -> bottom.  this will try to preserve the most recent version
of a block.  if erasing of the most recent (high address) block completely
fails for some reason (broken flash), it will become a sticky block and writing
to that block ID will not be possible anymore.  if erasing fails partially
(only some bits are erased) the CRC check will indicate a "free block".

normally, the flash memory topology is to have larger physical blocks like
4 KB or 64 KB and only complete flash blocks can be erased.  this is not true
for the RX data flash, as the erase block size is much smaller (32 bytes).
this correspondence of 1 flash block = 1 file block makes it a bit easier
to do the garbage collection.  if we can't erase individual file blocks, but
only whole flash blocks (= multiple file blocks), additional bitvectors can be
used in the partition to record the state of a file block.

*/

#include <cinttypes>
#include <cstring>
#include <type_traits>
#include <cassert>
#include <algorithm>

#include <hash/crc.hpp>
#include <utils/bits.hpp>

#include "flash_bfs.hpp"

#define log_all
#include <logging/logging.hpp>

namespace
{

static constexpr char magic_string[16 + 1] = "__FLASH_BFS_____";
static constexpr size_t magic_string_len = 16;

// ----------------------------------------------------------------------------

static uint32_t checksum_crc16_16_15_2_0 (const void* data, unsigned int len)
{
  hash::crc < 16, 0x8005, 0x0000, 0x0000, true, true > crc;
  return crc (data, len);
}

static uint32_t checksum_crc16_16_12_5_0 (const void* data, unsigned int len)
{
  hash::crc < 16, 0x1021, 0x0000, 0x0000, true, true > crc;
  return crc (data, len);
}

static std::string_view
get_string (const void* data, unsigned int max_len)
{
  const char* zero_chr = (const char*)std::memchr (data, 0, max_len);
  if (zero_chr == nullptr)
    return { };
  else
    return { (const char*)data, (unsigned int)(zero_chr - (const char*)data) };
}

} // anonymous namespace


// ----------------------------------------------------------------------------

namespace fs
{

/*
  struct block_1_1_2
  {
    // each file in the partition has a number assigned.
    // file names are optional.  this allows a maximum of 256 files in
    // the partition.
    uint8_t file_id;

    // the block number of the file.  there are 16 special block numbers
    // 240 (0xF0) .. 255 (0xFF) which are used to encode special file
    // operations.  see enum file_op.
    uint8_t block_num;

    std::array<uint8_t, 28> data;

    // the checksum algorithm is specified in the partition header.
    uint16_t checksum;
  };

  static_assert (sizeof (block_1_1_2) == 32, "");
*/


struct flash_bfs::file_block_1_1_2 : public fs::file
{
  using fs::file::m_numid;
  using fs::file::m_block_size;
  using fs::file::m_name;
  using fs::file::m_size_bytes;
  using fs::file::m_size_blocks;


  file_block_1_1_2 (partition* p) : file (p)
  {
    m_blocks.fill (0);
  }

  virtual ~file_block_1_1_2 (void) override
  {
    #ifndef FS_FLASH_BFS_READ_ONLY
      auto* p = (flash_bfs::partition*)&*m_partition;
      p->m_opened_files.erase(this->m_numid);
    #endif
  }

  virtual size_t read (void* out, size_t num_bytes) const override
  {
    // this file has been opened successfully.  thus we can read from it
    // by looking at the blocks array, which has been built when opening
    // the file, which also included checksum checking.  if a block is not
    // noted in the array, output zeros.
    //const size_t bsz = m_partition->raw_blocksize_bytes ();

    auto* p = (flash_bfs::partition*)&*m_partition;

    const size_t bsz_user = p->user_blocksize_bytes ();

    unsigned int cur_block = 0;
    char* out_ptr = (char*)out;
    size_t bytes_read = 0;

    while (num_bytes > 0 && cur_block < m_blocks.size ())
    {
      size_t bytes_to_read = std::min (bsz_user, num_bytes);

      if (m_blocks[cur_block] == 0)
	std::memset (out_ptr, 0, bytes_to_read);
      else
      {
	// skip 2 header bytes
	std::memcpy (out_ptr, (const void*)(m_blocks[cur_block] + 2), bytes_to_read);
      }

      // bytes_to_read is less than a block size it is the last block and
      // the loop will stop.  thus we can assume that at each iteration
      // we read one full block.
      ++cur_block;

      out_ptr += bytes_to_read;
      bytes_read += bytes_to_read;
      num_bytes -= bytes_to_read;
    }

    return bytes_read;
  }


  virtual size_t
  read (fs::file_size_t start_block, size_t block_count, void* out) const override
  {
  #ifdef FS_FLASH_BFS_DISABLE_BLOCK_READ
    return 0;

  #else
    // this file has been opened successfully.  thus we can read from it
    // by looking at the blocks array, which has been built when opening
    // the file, which also included checksum checking.  if a block is not
    // noted in the array, output zeros.
    //const size_t bsz = m_partition->raw_blocksize_bytes ();

    auto* p = (flash_bfs::partition*)&*m_partition;

    const size_t bsz_user = p->user_blocksize_bytes ();

    fs::file_size_t cur_block = start_block;
    char* out_ptr = (char*)out;
    size_t blocks_read = 0;

    while (block_count > 0 && cur_block < m_blocks.size ())
    {
      if (m_blocks[cur_block] == 0)
	std::memset (out_ptr, 0, bsz_user);
      else
      {
	// skip 2 header bytes
	std::memcpy (out_ptr, (const void*)(m_blocks[cur_block] + 2), bsz_user);
      }

      // bytes_to_read is less than a block size it is the last block and
      // the loop will stop.  thus we can assume that at each iteration
      // we read one full block.
      ++cur_block;

      out_ptr += bsz_user;
      ++blocks_read;
      --block_count;
    }

    return blocks_read;
  #endif // FS_FLASH_BFS_DISABLE_BLOCK_READ
  }

  virtual size_t
  write (const void* in, fs::file_size_t start_block, size_t block_count) override
  {
    return 0;
  }

  virtual size_t
  write (const void* in, size_t num_bytes) override
  {
  #ifdef FS_FLASH_BFS_READ_ONLY
    return 0;

  #else

    auto* p = (flash_bfs::partition*)&*m_partition;

    const auto chkfunc = p->checksum_func ();

    if (chkfunc == nullptr)
    {
      log_info ("invalid checksum function\n");
      return 0;
    }

    const size_t bsz = p->raw_blocksize_bytes ();
    const size_t bsz_user = p->user_blocksize_bytes ();

    const size_t needed_blocks = (num_bytes + bsz_user - 1) / bsz_user;

    // to serve this write request we need 'needed_blocks' number of blank
    // blocks AFTER the highest block address of this file.

    // if there is not enough space in the partition left, bail out.
    // N.B. actually this write request will replace the old blocks of this
    // file and we only need to keep the new blocks.  however, to allow
    // fail-safe writes we need to keep the old blocks, too.  thus, the max.
    // file size will always be limited to 1/2 the size of the partition.
    if (p->partition_free_blocks () < needed_blocks)
    {
      log_error ("not enough space in partition");
      return 0;
    }

    // if there are not enough blocks try doing a garbage collection.
    // try to find that out before starting the file write.
    // however, it's not a guarantee that the file write will be possible.
    // during writing blocks might die or some other error might occur which
    // will effectively reduce the size of the partition.
    // FIXME: record bad blocks and avoid them in the future.

    // find the next free block to be used for writing data.
    // start looking for the next free block after the last block of this file.
    uintptr_t next_new_block_addr =
	std::max (*std::max_element (m_blocks.begin (), m_blocks.end ()),
		  (uintptr_t)p->blocks_begin ());

    {
      size_t avail_blank_blocks = 0;
      uintptr_t a = next_new_block_addr;
      for (; avail_blank_blocks < needed_blocks; ++avail_blank_blocks)
      {
	a = p->find_next_blank_block (a);
	if (a == 0)
	  break;

	a += bsz;
      }

      log_info ("write blocks: %u  avail blank blocks: %u\n",
		(unsigned int)needed_blocks, (unsigned int)avail_blank_blocks);

      if (avail_blank_blocks < needed_blocks)
      {
        next_new_block_addr = p->garbage_collect ();

	if (next_new_block_addr == 0)
	{
	  log_error ("garbage collection failred\n");
	  return 0;
	}
      }
    }

    next_new_block_addr = p->find_next_blank_block (next_new_block_addr);

    std::unique_ptr<uint8_t[]> blockbuf (new uint8_t[bsz * 2]);

    uint8_t* blockbuf0 = blockbuf.get ();
    uint8_t* blockbuf1 = blockbuf0 + bsz;

    unsigned int cur_block = 0;
    const char* in_ptr = (const char*)in;
    size_t bytes_written = 0;

    constexpr int MAX_RETRY_TIMES = 6;
    while (num_bytes > 0 && cur_block < m_blocks.size ())
    {
      size_t bytes_to_write = std::min (bsz_user, num_bytes);

      // write block header
      blockbuf0[0] = m_numid;
      blockbuf0[1] = cur_block;

      // copy user data 
      std::memcpy (&blockbuf0[2], in_ptr, bytes_to_write);

      auto old_block_addr = m_blocks[cur_block];
      if (0 != old_block_addr && !p->m_flash_dev.read_block (old_block_addr, blockbuf1))
      {
        log_error ("block read old block 0x%08x NG\n", (unsigned int)old_block_addr);
        break;
      }
      // compare the data to be written with the old data.
      // if is the same, means this block doesn't need to be written.
      if (std::memcmp (blockbuf0, blockbuf1, bsz - 2) == 0)
      {
        log_info ("data in old block 0x%08x and data to write are same, not writing.\n",
                  (unsigned int)old_block_addr);
      }
      else
      {
        // clear remaining user data and/or block footer (checksum field).
        std::memset (&blockbuf0[2 + bytes_to_write], 0, bsz - (2 + bytes_to_write));

        // calculate and set the checksum
        *(uint16_t*)(&blockbuf0[0] + bsz - 2) = chkfunc (&blockbuf0[0], bsz - 2);

        // a write could fail if a flash block dies.
        // in this case, should try to find a next free block and retry
        // flashing the current block.
        bool write_ok = false;

        for (int i = 0; i < MAX_RETRY_TIMES; ++i)
        {
          // if there are no more free blocks do a garbage collection and retry.
          if (next_new_block_addr == 0)
            next_new_block_addr = p->garbage_collect ();

          if (next_new_block_addr == 0)
          {
            log_error ("can't get free block for writing\n");
            break;
          }

	  write_ok = p->m_flash_dev.write_block (next_new_block_addr, &blockbuf0[0]);
	  if (write_ok)
	  {
	    log_info ("write data block 0x%08x OK\n", (unsigned int)next_new_block_addr);

	    // remember that block for subsequent read operations.
	    // if we have overwritten an old block it will not count as a
	    // used block in the partition (it was already counted when
	    // mounting the partition).  if it was a new block, count it.

	    if (m_blocks[cur_block] != 0)
	    {
	      // there was an old block which has been replaced with a new block.
	      p->m_using_blocks[p->addr_to_index (m_blocks[cur_block])] = false;
	    }
	    else
	    {
	      // a new block was written.
	      p->m_free_blocks -= 1;
	    }

	    m_blocks[cur_block] = next_new_block_addr;
	    p->m_using_blocks[p->addr_to_index (m_blocks[cur_block])] = true;
	    break;
	  }

	  next_new_block_addr = p->find_next_blank_block (next_new_block_addr + bsz);
	}

	if (!write_ok)
	{
	  log_error ("block write 0x%08x NG\n", (unsigned int)next_new_block_addr);
	  break;
	}
      }

      ++cur_block;
      in_ptr += bytes_to_write;
      bytes_written += bytes_to_write;
      num_bytes -= bytes_to_write;

      // find the next block for writing.
      if (num_bytes > 0)
	next_new_block_addr = p->find_next_blank_block (next_new_block_addr + bsz);
    }

    return bytes_written;

  #endif // FS_FLASH_BFS_READ_ONLY
  }

  virtual bool rename (std::string_view name) override
  {
    return false;
  }

  virtual fs::time_info_t time_info (void) const override
  {
    return { };
  }

  // pointers to blocks of the file in the data flash.
  // if some blocks are undefined, it will be a nullptr.
  std::array<uintptr_t, 240> m_blocks;
};

// ----------------------------------------------------------------------------

[[gnu::cold]] flash_bfs::flash_bfs (void)
{
}

const flash_bfs::partition_header*
flash_bfs::find_partition_header (dev::flash_memory& fldev,
				  uintptr_t start_ptr, uintptr_t end_ptr)
{
  // the start_ptr might not be properly aligned, which happens when scanning
  // for partitions.
  const uintptr_t partition_alignment = fldev.rw_block_size ();
  assert (utils::is_pow2 (partition_alignment));

  const uintptr_t aligned_start_ptr = utils::ceil_pow2 (start_ptr, partition_alignment);

  log_info ("find_partition_header start = 0x%08x (-> 0x%08x) end = 0x%08x\n",
	    (unsigned int)start_ptr, (unsigned int)aligned_start_ptr, (unsigned int)end_ptr);

  for (const char* ptr = (const char*)aligned_start_ptr;
       ptr != (const char*)end_ptr; ptr += partition_alignment)
  {
    // check if the erase blocks that are covered by the minimum sized partition
    // header are blank.
    const uintptr_t possible_header_start = (uintptr_t)ptr;
    const uintptr_t possible_header_end =
		utils::ceil_pow2 (possible_header_start + partition_header::min_header_size (),
				  partition_alignment);

    if (!fldev.is_area_blank ({ possible_header_start, possible_header_end })
	&& std::memcmp (ptr, magic_string, magic_string_len) == 0)
      {
	log_info ("found partition at 0x%" PRIXPTR "\n", (uintptr_t)ptr);
	return (const partition_header*)ptr;
      }
  }

  log_info ("no partition found\n");
  return nullptr;
}

bool flash_bfs::validate_supported_partition (const partition_header* hdr)
{
  switch (hdr->m_checksum_algorithm)
  {
    default:
      return false;

    case crc16_16_15_2_0:
    case crc16_16_12_5_0:
      break;
  }

  switch (hdr->m_byte_order)
  {
    default:
      return false;

    case big_endian:
    case little_endian:
      break;
  }

  const auto chkfunc = checksum_func (hdr->m_checksum_algorithm, hdr->m_byte_order);
  const auto chksz = checksum_size_bytes (hdr->m_checksum_algorithm);
  if (chkfunc == nullptr || chksz == 0)
    return false;

  if (chkfunc (hdr, hdr->header_size ()) != 0)
  {
    log_info ("partition header checksum check NG\n");
    return false;
  }

  switch (hdr->m_block_type)
  {
    default:
      return false;

    case block_type_1_1_2:
      break;
  }

  const auto rw_block_size = hdr->rw_block_size ();
  if (rw_block_size == 0 || !utils::is_pow2 (rw_block_size))
    return false;

  const auto psz = hdr->partition_size_rw_blocks ();
  if (psz == 0)
    return false;

  if (hdr->offset_rw_blocks () >= psz)
    return false;

  return true;
}

size_t flash_bfs::get_partition_size (const partition_header* hdr)
{
  switch (hdr->m_byte_order)
  {
    default:
      return 0;

    case big_endian:
    case little_endian:
      break;
  }

  return hdr->rw_block_size () * hdr->partition_size_rw_blocks ();
}

std::unique_ptr<flash_bfs::partition>
flash_bfs::mount_partition (dev::flash_memory& fldev,
			    std::string_view name)
{
  log_info ("mount partition\n");

  const auto partition_alignment = fldev.rw_block_size ();
  assert (utils::is_pow2 (partition_alignment));

  auto mi = fldev.mmap_info ();

  for (uintptr_t addr = mi.start_addr (); addr < mi.end_addr (); )
  {
    const partition_header* h = find_partition_header (fldev, addr, mi.end_addr ());

    // if we hit a nullptr it scanned the whole space but could not find a
    // partition header.  assume that there really is none.
    if (h == nullptr)
      break;

    if (get_string (h->m_partition_name.data (), h->m_partition_name.size ()) == name
	&& validate_supported_partition (h))
    {
       auto r = std::make_unique<partition> (const_cast<partition_header*> (h), fldev);
	log_info ("partition ... \n"
		  "   max num files:          %u\n"
		  "   max file size blocks:   %u\n"
		  "   max file size bytes:    %u\n"
		  "   raw block size bytes:   %u\n"
		  "   user block size bytes:  %u\n"
		  "   partition size blocks:  %u\n"
		  "   partition size bytes:   %u\n"
		  "   partition free blocks:  %u\n"
		  "   blocks begin, end = 0x%" PRIXPTR ", 0x%" PRIXPTR "\n",
		  (unsigned int)r->max_num_files (),
		  (unsigned int)r->max_filesize_blocks (),
		  (unsigned int)r->max_filesize_bytes (),
		  (unsigned int)r->raw_blocksize_bytes (),
		  (unsigned int)r->user_blocksize_bytes (),
		  (unsigned int)r->partition_size_blocks (),
		  (unsigned int)r->partition_size_bytes (),
		  (unsigned int)r->partition_free_blocks (),
		  (uintptr_t)r->blocks_begin (), (uintptr_t)r->blocks_end ());
      return r;
    }

    // try to skip the whole partition.
    auto psz = get_partition_size (h);
    addr += psz != 0 ? psz : partition_alignment;
  }

  return nullptr;
}

std::unique_ptr<flash_bfs::partition>
flash_bfs::create_partition (dev::flash_memory& fldev,
			     std::string_view name,
			     checksum_algorithm_t chkalgo,
			     byte_order_t bo,
			     block_type_t bt,
			     size_t partition_block_count,
			     size_t partition_block_offset)
{
#ifdef FS_FLASH_BFS_READ_ONLY
  return nullptr;
#else
  const auto raw_blocksize_bytes = fldev.rw_block_size ();
  assert (utils::is_pow2 (raw_blocksize_bytes));

  const auto partition_header_size_in_blocks =
	utils::ceil_pow2 (sizeof (partition_header), (size_t)raw_blocksize_bytes) / raw_blocksize_bytes;

  const auto user_block_count = partition_block_count - partition_header_size_in_blocks;

  log_info ("create partition\n"
	    "   block size = %u\n"
	    "   block count = %u\n"
	    "   partition header block count = %u\n"
	    "   user block count = %u\n",
	    (unsigned int)raw_blocksize_bytes, (unsigned int)partition_block_count,
	    (unsigned int)partition_header_size_in_blocks,
	    (unsigned int)user_block_count);


  // FIXME: for now this will always create a partition at the start address
  // and overwrite everything.
  // instead it should look for sufficient contiguous space.
  partition_header hdr;
  std::memset (&hdr, 0, sizeof (hdr));

  std::memcpy (hdr.m_magic_string.data (), magic_string, magic_string_len);

  if (!name.empty ())
  {
    auto namelen = std::min (name.size (), hdr.m_partition_name.size () - 1);
    std::memcpy (hdr.m_partition_name.data (), name.data (), namelen);
  }

  hdr.m_checksum_algorithm = chkalgo;
  hdr.m_byte_order = bo;
  hdr.m_block_type = bt;
  hdr.set_header_size (sizeof (partition_header));
  hdr.set_rw_block_size (raw_blocksize_bytes);
  hdr.set_partition_size_rw_blocks (user_block_count);

  const auto chkfunc = checksum_func (chkalgo, bo);
  const auto chksz = checksum_size_bytes (chkalgo);
  if (chkfunc == nullptr || chksz == 0)
  {
    log_error ("invalid checksum\n");
    return nullptr;
  }

  uint32_t hdr_checksum = chkfunc (&hdr, sizeof (partition_header) - chksz);
  uint8_t* hdr_checksum_ptr = (uint8_t*)&hdr + sizeof (partition_header) - chksz;

  if (chksz == 1)
    *(uint8_t*)hdr_checksum_ptr = (uint8_t)hdr_checksum;
  else if (chksz == 2)
    *(uint16_t*)hdr_checksum_ptr = (uint16_t)hdr_checksum;
  if (chksz == 4)
    *(uint32_t*)hdr_checksum_ptr = (uint32_t)hdr_checksum;

  log_info ("hdr block size = %u  partition size = %u  checksum = 0x%08x\n",
	    (unsigned int)hdr.rw_block_size (),
	    (unsigned int)hdr.partition_size_rw_blocks (),
	    (unsigned int)hdr.m_checksum);

  const uintptr_t partition_start_addr = fldev.mmap_info ().start_addr ()
				+ partition_block_offset * raw_blocksize_bytes;

  const uintptr_t partition_end_addr = partition_start_addr
				+ partition_block_count * raw_blocksize_bytes;

  log_info ("partition addr start, end = 0x%08x, 0x%08x\n",
	    (unsigned int)partition_start_addr, (unsigned int)partition_end_addr);

  if (partition_end_addr > fldev.mmap_info ().end_addr ())
  {
    log_error ("partition too big\n");
    return nullptr;
  }

  // erase all blocks that will be covered by the partition.
  for (auto bi = fldev.erase_block_for_addr (partition_start_addr);
	bi.start_addr () < partition_end_addr && bi;
	bi = fldev.erase_block_for_addr (bi.end_addr ()))
  {
    if (!fldev.is_area_blank (bi))
    {
      log_info ("erasing block 0x%08x\n", (unsigned int)bi.start_addr ());
      if (!fldev.erase_block (bi))
      {
	log_error ("could not erase data block 0x%08x\n", (unsigned int)bi.start_addr ());
	return nullptr;
      }
    }
  }

  // write partition header
  for (uintptr_t offset = 0; offset < sizeof (hdr); offset += raw_blocksize_bytes)
  {
    log_info ("writing block 0x%08x\n", (unsigned int)(partition_start_addr + offset));
    if (!fldev.write_block (partition_start_addr + offset,
			    (const char*)&hdr + offset))
    {
      log_error ("could not write partition header block %u\n", (unsigned int)offset);
      return nullptr;
    }
    else
      log_info ("partition header block %u written\n", (unsigned int)offset);
  }

  auto r = std::make_unique<partition> ((partition_header*)partition_start_addr, fldev);

	log_info ("partition ... \n"
		  "   max num files:          %u\n"
		  "   max file size blocks:   %u\n"
		  "   max file size bytes:    %u\n"
		  "   raw block size bytes:   %u\n"
		  "   user block size bytes:  %u\n"
		  "   partition size blocks:  %u\n"
		  "   partition size bytes:   %u\n"
		  "   partition free blocks:  %u\n"
		  "   blocks begin, end = 0x%" PRIXPTR ", 0x%" PRIXPTR "\n",
		  (unsigned int)r->max_num_files (),
		  (unsigned int)r->max_filesize_blocks (),
		  (unsigned int)r->max_filesize_bytes (),
		  (unsigned int)r->raw_blocksize_bytes (),
		  (unsigned int)r->user_blocksize_bytes (),
		  (unsigned int)r->partition_size_blocks (),
		  (unsigned int)r->partition_size_bytes (),
		  (unsigned int)r->partition_free_blocks (),
		  (uintptr_t)r->blocks_begin (), (uintptr_t)r->blocks_end ());

  return r;
#endif // FS_FLASH_BFS_READ_ONLY
}

flash_bfs::partition::partition (partition_header* h, dev::flash_memory& fldev)
: m_header (h), m_flash_dev (fldev)
{
  const auto part_size_blocks = partition_size_blocks ();

  #ifndef FS_FLASH_BFS_READ_ONLY
  m_free_blocks = part_size_blocks;
  #endif
  m_using_blocks.resize (part_size_blocks, false);

  // here we know that the rw_block_size is a pow2 value.
  // it has been checked in validate_supported_partition or
  // in create_partition.
  m_rw_block_size = m_header->rw_block_size ();
  m_rw_block_size_log2 = utils::pow2_log2 (m_rw_block_size);

  const uintptr_t bsz = m_rw_block_size;

  m_block_addr_begin = utils::ceil_pow2 ((uintptr_t)m_header + m_header->header_size ()
			- (uintptr_t)m_header->offset_rw_blocks () * bsz, bsz);

  m_block_addr_end = m_block_addr_begin + part_size_blocks * bsz;


  log_info ("init_using_blocks...\n");

  const auto chkfunc = checksum_func ();

  // then iterate over all file ids and collect all used blocks by scanning
  // the partition blocks in reverse order and check in the file id bitset
  // whether a file is still in the partition or not.

  // scan the whole partition in the reverse direction and create a bitmap
  // of all used/unused bits.  if a bit is set, it means the block is used.
  // blocks that fulfill the following criteria are regarded as unused:
  //
  //  - block is blank
  //
  //  - block does not pass the checksum test
  //
  //  - { file id, block id } has been seen before in a previous block
  //    this means the block is an old version and can be ignored.
  //
  // FIXME: SP-35
  // need to support deleted files.
  //
  // to do that the { file id, block id } pairs are recorded in a temporary
  // set (sorted vector).  the max size of this vector in the worst case is:
  //    max different 16 bit values = 65536
  //    parition size blocks: 510
  //    -> max 510 different file/block ID values.
  //       = 1020 bytes
  // since the maximum size of the vector is known, we can use an array with a
  // fixed size.  this results in smaller code than using std::vector.
  const auto id_set_max_size =
	std::min (part_size_blocks,
		  (size_t)std::numeric_limits<uint16_t>::max () + 1);

  auto id_set = std::make_unique<uint16_t[]> (id_set_max_size);

  uint16_t* const id_set_begin = id_set.get ();
  [[maybe_unused]] uint16_t* const id_set_end_max = id_set_begin + id_set_max_size;
  uint16_t* id_set_end = id_set_begin;

  auto const b_end = block_addr_end ();

  auto idx = m_using_blocks.size () - 1;
  for (uintptr_t b = b_end - bsz, b_begin = block_addr_begin () - bsz;
      b != b_begin; b -= bsz, --idx)
  {
    auto bi = m_flash_dev.rw_block_for_addr (b);
    log_info ("block 0x%08x (0x%08x, 0x%08x, %d): ", (unsigned int)b,
		(unsigned int)bi.start_addr (), (unsigned int)bi.end_addr (), (bool)b);

    if (m_flash_dev.is_area_blank (bi))
    {
      log_info (" blank\n");
      continue;
    }

    uint8_t* bheader = (uint8_t*)b;
    // run the checksum function over the whole block.  if it's correct,
    // the result should be zero.
    if (chkfunc (bheader, bsz) != 0)
    {
      log_info ("  bad checksum\n");
      continue;
    }

    // the byte order doesn't have any meaning in this case, as we're always
    // comparing the full 16 bit word.  use std::memcpy because of potentially
    // unaligned access.
    uint16_t n_b_id;
    std::memcpy (&n_b_id, bheader, sizeof (n_b_id));

    auto iter = std::lower_bound (id_set_begin, id_set_end, n_b_id, std::greater<> ());

    if (id_set_end == iter || n_b_id != *iter)
    {
      // insert new 'n_b_id' element before 'iter', shifting elements
      // after 'iter' by one to the left and increasing the range size by 1.
      assert (id_set_end != id_set_end_max);

      for (auto* i = id_set_end; i != iter; --i)
	*i = *(i - 1);

      *iter = n_b_id;
      ++id_set_end;

      do_if_log_info (
	log_info ("   block 0x%08x, id set updated: ",  (unsigned int)b);
	std::for_each (id_set_begin, id_set_end, [] (const auto& id)
	{
	  log_info ("[f%03u, b%03u] \t",
	  (unsigned int)(id >> 8),
	  (unsigned int)(id & 0xFF));
	});
	log_info ("\n");
      );
      m_using_blocks[idx] = true;

      #ifndef FS_FLASH_BFS_READ_ONLY
      --m_free_blocks;
      #endif
    }
  }
}

fs::partition::sizes_t flash_bfs::partition::sizes (void) const
{
  return
  {
    raw_blocksize_bytes (),
    user_blocksize_bytes (),
    max_num_files (),
    max_filesize_blocks (),
    partition_size_blocks ()
  };
}

size_t flash_bfs::partition::max_num_files (void) const
{
  size_t r;

  switch (m_header->m_block_type)
  {
    default:
      return 0;

    case block_type_1_1_2:
      r = 256;
  }

  // each file takes at least one block.
  return std::min (partition_size_blocks (), r);
}

size_t flash_bfs::partition::max_filesize_blocks (void) const
{
  size_t r;

  switch (m_header->m_block_type)
  {
    default:
      return 0;

    case block_type_1_1_2:
      r = 240;
      break;
  }

  // when overwriting file data blocks, old blocks are always preserved
  // to provide fail-safe writes.  this limits the file size to 1/2 the
  // total size of the partition.
  return std::min (partition_size_blocks () / 2, r);
}

unsigned int flash_bfs::partition::user_blocksize_bytes (void) const
{
  unsigned int header_footer_size = 0;

  switch (m_header->m_block_type)
  {
    default:
      return 0;

    case block_type_1_1_2:
      header_footer_size = 1 + 1 + 2;
      break;
  }

  return raw_blocksize_bytes () - header_footer_size;
}

flash_bfs::checksum_func_t
flash_bfs::checksum_func (checksum_algorithm_t x, byte_order_t /* bo */)
{
  switch (x)
  {
    default:
      return nullptr;

    case crc16_16_15_2_0:
      return checksum_crc16_16_15_2_0;

    case crc16_16_12_5_0:
      return checksum_crc16_16_12_5_0;
  }
}

flash_bfs::checksum_func_t
flash_bfs::partition::checksum_func (void) const
{
  return flash_bfs::checksum_func (m_header->m_checksum_algorithm, m_header->m_byte_order);
}

size_t
flash_bfs::partition::checksum_size_bytes (void) const
{
  return flash_bfs::checksum_size_bytes (m_header->m_checksum_algorithm);
}

uintptr_t
flash_bfs::partition::find_next_blank_block (uintptr_t bstart) const
{
  // do a forward search and find the first block that is free.
  const uintptr_t block_end = (uintptr_t)blocks_end ();

  log_info ("find_next_blank_block bstart = 0x%08x   bend = 0x%08x\n", (unsigned int)bstart, (unsigned int)block_end);

  if (bstart >= block_end)
    return 0;

  for (auto bi = m_flash_dev.rw_block_for_addr (bstart);
	bi.start_addr () < block_end && bi;
	bi = m_flash_dev.rw_block_for_addr (bi.end_addr ()))
  {
    if (m_flash_dev.is_area_blank (bi))
      return bi.start_addr ();
  }

  return 0;
}

bool
flash_bfs::partition::is_invalid_block (uintptr_t b) const
{
  log_info ("is_invalid_block 0x%08x\n", (unsigned int)b);

  const auto chkfunc = checksum_func ();

  if (chkfunc == nullptr)
  {
    log_info ("invalid checksum function\n");
    return false;
  }

  const size_t bsz = raw_blocksize_bytes ();

  if (chkfunc ((const void*)b, bsz) != 0)
    return true;

  return false;
}


// try to perform a garbage collection by erasing blocks at the lower
// addresses and moving blocks from higher addresses to lower addresses.
// returns the new lowest next free block address that can be used.
uintptr_t
flash_bfs::partition::garbage_collect (void)
{
#ifdef FS_FLASH_BFS_READ_ONLY
  return 0;

#else

  log_info ("partition garbage collecting...\n");
  const auto rw_block_sz = raw_blocksize_bytes ();

  // FIXME: support only 1_1_2 blocks for now.
  if (m_header->m_block_type != block_type_1_1_2)
  {
    log_info ("invalid partition block type\n");
    return 0;
  }

  uintptr_t  free_addr = 0;

  std::unique_ptr<uint8_t[]> tmp_block_buf (new uint8_t[rw_block_sz * 2]);  // !! might throw

  std::vector<uint8_t> tmp_block_buf2;
  tmp_block_buf2.reserve (rw_block_sz * 2);
  unsigned int tmp_block_buf2_rd_i = 0;
  unsigned int tmp_block_buf2_wr_i = 0;

  log_info ("erasing unused blocks\n");
  gc_erase_unused_blocks ();
  dump_log_info_blocks ();


  // iterate over the erase blocks and try to compact them by moving used blocks up.
  for (auto [rw_bi, a] = std::tuple{ 0u, block_addr_begin () }; rw_bi < m_using_blocks.size (); )
  {
    dev::flash_memory::block_info eb = m_flash_dev.erase_block_for_addr (a);

    auto st = erase_block_state (eb);

    if (st == all_used)
    {
      log_info ("rw_bi = %u  a = 0x%08x  all used\n", rw_bi, (unsigned int)a);

      // skip this erase block
      for (; a != eb.end_addr () && rw_bi < m_using_blocks.size (); ++rw_bi, a += rw_block_sz);
    }

    else if (st == all_unused)
    {
  retry_all_unused:
      log_info ("rw_bi = %u  a = 0x%08x  all unused\n", rw_bi, (unsigned int)a);

      // try to find used blocks and move them up into the current erase block's
      // rw blocks.

      free_addr = a;

      for (; a != eb.end_addr () && rw_bi < m_using_blocks.size (); ++rw_bi, a += rw_block_sz)
      {
	if (tmp_block_buf2_rd_i != tmp_block_buf2_wr_i)
	{
	  // get a block from the temporary buffer and write it to flash.
	  log_info ("writing queued tmp block  %u / %u\n", tmp_block_buf2_wr_i, tmp_block_buf2_rd_i);

	  const uint8_t* src_block = tmp_block_buf2.data () + tmp_block_buf2_rd_i;

	  if (!m_flash_dev.write_block (a, src_block))
	  {
	    log_error ("flash write NG %u 0x%08x\n", rw_bi, (unsigned int)a);
	    return 0;
	  }

	  m_using_blocks[rw_bi] = true;
	  update_opened_file_block_map (src_block, a);

	  // step to next in tmp buffer/queue
	  tmp_block_buf2_rd_i += rw_block_sz;
	  if (tmp_block_buf2_rd_i == tmp_block_buf2_wr_i)
	  {
	    tmp_block_buf2_rd_i = tmp_block_buf2_wr_i = 0;
	    tmp_block_buf2.resize (0);
	  }
	}
	else
	{
	  auto next_used_rw_bi = rw_bi;
	  for (; next_used_rw_bi < m_using_blocks.size () && !m_using_blocks[next_used_rw_bi];
	       ++next_used_rw_bi);

	  if (next_used_rw_bi == m_using_blocks.size ())
	    goto no_more_used_blocks;

	  if (!gc_move_rw_block (next_used_rw_bi, rw_bi, tmp_block_buf.get ()))
	  {
	    log_error ("block move NG\n");
	    return 0;
	  }
	}
	free_addr = a + rw_block_sz;
      }

      // after moving blocks, we might be able to erase more blocks that became
      // unused (after this current erase block, no need to check already garbage
      // collected blocks).
      gc_erase_unused_blocks (rw_bi, a);
    }

    else if (st == partially_used)
    {
      log_info ("rw_bi = %u  a = 0x%08x  partially used\n", rw_bi, (unsigned int)a);

      // move all used rw blocks from this erase block somewhere else temporarily,
      // so that the erase block can be erased.

      for (auto [ii, aa] = std::tuple { rw_bi, a };
	   aa != eb.end_addr () && ii < m_using_blocks.size (); ++ii, aa += rw_block_sz)
      {
	if (m_using_blocks[ii])
	{
	  // move the block somewhere else temporarily and remember to restore it
	  // afterwards.
/*
FIXME: should move it to the flash somehwere.  however then the queue management
       becomes more complicated.

	  for (auto [iii, aaa] = std::tuple { ii, aa };
	       iii < m_using_blocks.size (); ++iii, aaa += rw_block_sz)
	  {

	  }
*/
	  log_info ("pushing block %u  0x%08x\n", ii, (unsigned int)aa);
	  tmp_block_buf2.resize (tmp_block_buf2.size () + rw_block_sz);
	  if (!m_flash_dev.read_block (aa, tmp_block_buf2.data () + tmp_block_buf2.size () - rw_block_sz))
	  {
	    log_error ("block read 0x%08x NG\n", (unsigned int)aa);
	    return false;
	  }
	  tmp_block_buf2_wr_i += rw_block_sz;
	  m_using_blocks[ii] = false;
	}
      }

      // all the rw blocks have been moved out from this earse block.
      // erase it and continue with it.
      if (!m_flash_dev.erase_block (eb))
      {
	log_error ("   block erase 0x%08x NG\n", (unsigned int)eb.start_addr ());
	return 0;
      }
      goto retry_all_unused;
    }

    else //if (st == invalid_block)
    {
      log_error (" block 0x%08x state error\n", (unsigned int)eb.start_addr ());
      return 0;
    }
  }

no_more_used_blocks:;

  // the number of free blocks in the partition usually remains the same
  // after a garbage collection.
  dump_log_info_blocks ();

  log_info ("garbage collection done.  next free block = 0x%08x\n", (unsigned int)free_addr);

  return free_addr ==  block_addr_end () ? 0 : free_addr;
#endif // FS_FLASH_BFS_READ_ONLY
}

void flash_bfs::partition::dump_log_info_blocks (void)
{
  const auto rw_block_sz = raw_blocksize_bytes ();

  for (auto [rw_bi, a] = std::tuple{ 0u, block_addr_begin () }; rw_bi < m_using_blocks.size ();
	rw_bi += 1, a += rw_block_sz)
  {
    log_info ("rw block %u  0x%08x  blank: %d\n", rw_bi, (unsigned int)a, m_flash_dev.is_area_blank ({ a, a + rw_block_sz }));
  }
}

[[gnu::noinline]]
flash_bfs::partition::erase_block_state_t
flash_bfs::partition::erase_block_state (dev::flash_memory::block_info eb) const
{
  if (!eb)
    return invalid_block;

  const auto rw_block_sz = raw_blocksize_bytes ();

  unsigned int used_count = 0;
  unsigned int total_count = 0;

  for (auto [a, rw_bi] = std::tuple { eb.start_addr (), (eb.start_addr () - block_addr_begin ()) / rw_block_sz };
	a != eb.end_addr () && rw_bi < m_using_blocks.size (); ++rw_bi, a += rw_block_sz)
  {
    ++total_count;

    if (m_using_blocks[rw_bi])
      ++used_count;
  }

  if (used_count == 0)
    return all_unused;
  else if (used_count == total_count)
    return all_used;
  else
    return partially_used;
}



bool flash_bfs::partition::gc_erase_unused_blocks (void)
{
  return gc_erase_unused_blocks (0, block_addr_begin ());
}

[[gnu::noinline]]
bool flash_bfs::partition::gc_erase_unused_blocks (unsigned int idx, uintptr_t addr)
{
  const auto rw_block_sz = raw_blocksize_bytes ();

  // try to erase blocks that have no used blocks
  for (auto [rw_bi, a] = std::tuple{ idx, addr }; rw_bi < m_using_blocks.size (); )
  {
    dev::flash_memory::block_info eb = m_flash_dev.erase_block_for_addr (a);

    log_info ("rw_bi = %u  a = 0x%08x   eb 0x%08x 0x%08x\n", rw_bi, (unsigned int)a, eb.start_addr (), eb.end_addr ());

    if (!eb)
    {
      log_error ("invalid erase block for rw block i = %u  addr = 0x%08x\n", rw_bi, (unsigned int)a);
      return false;
    }

    bool can_erase = true;

    for (; a != eb.end_addr () && rw_bi < m_using_blocks.size (); ++rw_bi, a += rw_block_sz)
    {
      if (m_using_blocks[rw_bi])
	can_erase = false;
    }

    log_info ("erase block 0x%08x - 0x%08x can erase: %d\n",
		(unsigned int)eb.start_addr (), (unsigned int)eb.end_addr (), can_erase);

    if (can_erase && !m_flash_dev.is_area_blank (eb))
      if (!m_flash_dev.erase_block (eb))
      {
	log_error ("block erase 0x%08x NG\n", (unsigned int)eb.start_addr ());
	return false;
      }
  }

  return true;
}

[[gnu::noinline]]
bool
flash_bfs::partition::gc_move_rw_block (unsigned int block_idx_src,
				        unsigned int block_idx_dst,
				        uint8_t* tmp_buf)
{
  const auto rw_block_sz = raw_blocksize_bytes ();
  uint8_t* blockbuf0 = tmp_buf;
  uint8_t* blockbuf1 = blockbuf0 + rw_block_sz;

  auto b_rd = block_addr_begin () + block_idx_src * rw_block_sz;
  auto b_wr = block_addr_begin () + block_idx_dst * rw_block_sz;

  if (!m_flash_dev.read_block (b_rd, blockbuf0))
  {
    log_error ("block read 0x%08x NG\n", (unsigned int)b_rd);
    return false;
  }
  if (!m_flash_dev.write_block (b_wr, blockbuf0))
  {
    log_error ("block write 0x%08x NG\n", (unsigned int)b_wr);
    return false;
  }
  if (!m_flash_dev.read_block (b_wr, blockbuf1))
  {
    log_error ("block read after write 0x%08x NG\n", (unsigned int)b_wr);
    return false;
  }
  if (std::memcmp (blockbuf0, blockbuf1, raw_blocksize_bytes ()) != 0)
  {
    log_error ("block memcmp after write 0x%08x NG\n", (unsigned int)b_wr);
    return false;
  }

  const auto chkfunc = checksum_func ();

  if (chkfunc == nullptr)
  {
    log_error ("invalid checksum function\n");
    return false;
  }

  if (chkfunc (blockbuf1, raw_blocksize_bytes ()) != 0)
  {
    log_error ("checksum after write 0x%08x NG\n", (unsigned int)b_wr);
    return false;
  }

  // update the block use map.  assume that we only ever move blocks that
  // are actively used.
  m_using_blocks[block_idx_dst] = true;
  m_using_blocks[block_idx_src] = false;

  update_opened_file_block_map (blockbuf0, b_wr);

  log_info ("gc block moved, 0x%08x -> 0x%08x\n", (unsigned int)b_rd, (unsigned int)b_wr);
  return true;
}

#ifndef FS_FLASH_BFS_READ_ONLY
[[gnu::noinline]]
void
flash_bfs::partition::update_opened_file_block_map (const uint8_t* block_data,
						    uintptr_t new_block_addr)
{
  uint8_t block_fid = block_data[0];
  uint8_t block_bn = block_data[1];

  auto f_iter = m_opened_files.find (block_fid);
  if ( f_iter != m_opened_files.end ())
  {
    auto f_ptr = (file_block_1_1_2*)f_iter->second;
    if (0 != f_ptr->m_blocks[block_bn])
      f_ptr->m_blocks[block_bn] = new_block_addr;
  }
}
#endif

std::unique_ptr<fs::file>
flash_bfs::partition::open_file (std::string_view name) const
{
  // FIXME: support only 1_1_2 blocks for now.
  if (m_header->m_block_type != block_type_1_1_2)
  {
    log_info ("invalid partition block type\n");
    return nullptr;
  }

  return nullptr;
}

std::unique_ptr<fs::file>
flash_bfs::partition::open_file (uint32_t numid) const
{
  log_info ("open_file 0x%08x\n", (unsigned int)numid);

  const auto bsz = raw_blocksize_bytes ();
  const auto chkfunc = checksum_func ();
  const auto bsz_user = user_blocksize_bytes ();

  // FIXME: support only 1_1_2 blocks for now.
  if (m_header->m_block_type != block_type_1_1_2)
  {
    log_info ("invalid partition block type\n");
    return nullptr;
  }

  if (chkfunc == nullptr)
  {
    log_info ("invalid checksum function\n");
    return nullptr;
  }

  auto r = std::make_unique<file_block_1_1_2> (const_cast<partition*> (this));

  // scan whole partition and collect all blocks that belong to the file.
  // the file size is determined by the highest block number that we hit,
  // or an explicit set_filesize operation.

  unsigned int filesize_blocks = 0;
  bool got_file = false;
  size_t idx = 0;
  for (uintptr_t b = block_addr_begin (), b_end = block_addr_end ();
       b != b_end; b += bsz, ++idx)
  {
    // FIXME: this loops one bit at a time in the std::vector<bool>
    // need to use more efficient bit search that looks at whole words.
    // e.g. boost::dynamic_bitset::find_next does such an optimization.
    if (!m_using_blocks[idx])
    {
      // log_info ("   block 0x%08x not used\n", (unsigned int)b);
      continue;
    }

    uint8_t* bheader = (uint8_t*)b;

    if (bheader[0] != numid)
    {
      //log_info ("   block 0x%08x numid mismatch (0x%08x)\n",
      //	(unsigned int)b, (unsigned int)bheader[0]);
      continue;
    }

    // the block is valid and the file numid matches.
    const unsigned int blockoff_fileop = bheader[1];

    if (blockoff_fileop < r->m_blocks.size ())
    {
      // it's a valid data block.
      log_info ("got block 0x%08x %u (0x%02x) checksum 0x%04x\n",
		 (unsigned int)b, (unsigned int)blockoff_fileop,
		 (unsigned int)blockoff_fileop,
		 *(const uint16_t*)(bheader + bsz - 2));

      got_file = true;
      filesize_blocks = std::max (filesize_blocks, blockoff_fileop + 1);

      // set the block address
      r->m_blocks[blockoff_fileop] = b;
    }

  #ifndef FS_FLASH_BFS_DISABLE_FILENAMES
    else if (blockoff_fileop == file_op_set_filename)
    {
      auto str = get_string (&bheader[2], bsz_user);
      log_info ("fileop = set_filename got_file = %d", got_file);

      // this is the first block for the file which creates a named empty file.
      got_file = true;
      r->m_name = str;
    }
  #endif

    else if (blockoff_fileop == file_op_set_filesize)
    {
      uint8_t new_filesz = bheader[2];
      log_info ("fileop = set_filesize %u (0x%02x) got_filesize = %d\n",
		new_filesz, new_filesz, filesize_blocks);

      if (new_filesz >= r->m_blocks.size ())
	log_info ("ignoring invalid new filesize value\n");
      else
      {
	// this might be the first block for the file which creates an unnamed
	// sized file.
	got_file = true;
	filesize_blocks = new_filesz;
      }
    }
    else if (blockoff_fileop == file_op_erase_file)
    {
      log_info ("fileop = erase_file got_file = %d\n", got_file);

      // restart and keep looking.
      filesize_blocks = 0;
      got_file = false;
      r->m_name = { };
      r->m_blocks.fill (0);
    }
    else
    {
      // some other unknown file operation command block.
      // ignore it.
      log_info ("skipping unknown fileop 0x%02x\n", blockoff_fileop);
    }
  }
  
  if (!got_file)
  {
    log_info ("file not found\n");
    return nullptr;
  }

  // blank out all blocks that are beyond the filesize.
  // this will make it easier to handle the reads.
  for (auto i = filesize_blocks; i < r->m_blocks.size (); ++i)
  {
    if (r->m_blocks[i] != 0)
      m_using_blocks[addr_to_index (r->m_blocks[i])] = false;
    r->m_blocks[i] = 0;
  }

  log_info ("file block map: ");
  for ([[gnu::unused]] auto b : r->m_blocks)
    log_info ("%08x ", (unsigned int)b);

  log_info ("\n");

  r->m_numid = numid;
  r->m_size_blocks = filesize_blocks;
  r->m_size_bytes = filesize_blocks * bsz_user;
  r->m_block_size = user_blocksize_bytes ();

  #ifndef FS_FLASH_BFS_READ_ONLY
    m_opened_files[numid] = r.get ();
  #endif

  return r;
}

std::unique_ptr<fs::file>
flash_bfs::partition::create_file (std::string_view name)
{
  return nullptr;
}

std::unique_ptr<fs::file>
flash_bfs::partition::create_file (uint32_t numid)
{
  #ifdef FS_FLASH_BFS_READ_ONLY
    return nullptr;
  #endif

  // FIXME: support only 1_1_2 blocks for now.
  if (m_header->m_block_type != block_type_1_1_2)
  {
    log_info ("invalid partition block type\n");
    return nullptr;
  }

  auto ff = open_file (numid);
  if (ff != nullptr)
    return nullptr;

  // the file is not there.
  // create a new file object, but don't write anything to disk.
  // the first block that is written will define the new file.
  auto r = std::make_unique<file_block_1_1_2> (this);

  r->m_numid = numid;
  r->m_size_blocks = 0;
  r->m_size_bytes = 0;
  r->m_block_size = 0;
  return r;
}

bool flash_bfs::partition::erase_file (uint32_t numid)
{
  #ifdef FS_FLASH_BFS_READ_ONLY
    return false;
  #endif

  // FIXME: support only 1_1_2 blocks for now.
  if (m_header->m_block_type != block_type_1_1_2)
  {
    log_info ("invalid partition block type\n");
    return false;
  }

  auto ff = open_file (numid);
  if (ff != nullptr)
  {
    // write an file_op_erase_file block.
  }
  else
  {
    // file does not exist.  nothing to do.
    return true;
  }

  return false;
}

bool flash_bfs::partition::erase_file (std::string_view name)
{
  #ifdef FS_FLASH_BFS_READ_ONLY
    return false;
  #endif

  return false;
}

} // namespace fs
