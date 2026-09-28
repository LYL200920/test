
/*

a block oriented/organized flash file system, targeted at NOR flash storages
of small size.

usually std::shared_ptr is used for keeping strong references to open files
and mounted partitions.  however, it pulls in some error handling code
which is too big for a bootloader.  thus use intrusive ref counting, which is
simpler than std::shared_ptr.

open file objects contain only a weak ref to their owning partition.  this
results in smaller code in some extreme situations like bootloaders.
because of this, the application must ensure that partition objects are not
deleted while there are still open file objects for that partition.

*/

#ifndef includeguard_fs_flash_bfs_includeguard
#define includeguard_fs_flash_bfs_includeguard


#include <cstdint>
#include <array>
#include <memory>
#include <string_view>
#include <map>
#include <vector>

#include <fs/fs.hpp>
#include <dev/flash_memory.hpp>
#include <utils/byte_order.hpp>

namespace fs
{

class flash_bfs
{
private:
  struct partition_header;

  typedef uint32_t (*checksum_func_t)(const void* data, unsigned int len);

public:
  flash_bfs (void);

  enum checksum_algorithm_t : uint8_t
  {
    // the lower nibble encodes the length of the checksum in bytes.
    // valid checksum lengths are 1, 2, 4 bytes.
    // this eliminates the need for a lookup table.

    // crc < 16, 0x8005, 0x0000, 0x0000, true, true >
    crc16_16_15_2_0 = (2 << 4) | (2),

    // crc < 16, 0x1021, 0x0000, 0x0000, true, true >
    crc16_16_12_5_0 = (3 << 4) | (2)
  };

  enum byte_order_t : uint8_t
  {
    little_endian = 0,
    big_endian = 1
  };

  enum block_type_t : uint8_t
  {
    // 1 byte file id, 1 byte block number, 2 bytes checksum
    block_type_1_1_2 = 0,
  };

  class partition;
  class file;

  class partition : public fs::partition
  {
  public:
    partition (const partition&) = delete;
    partition& operator = (const partition&) = delete;

    partition (partition_header* h, dev::flash_memory& fldev);

    // maximum number of files in this parition.
    size_t max_num_files (void) const;

    // maximum filesize in blocks.
    size_t max_filesize_blocks (void) const;

    // maximum filesize in bytes (counts only user data bytes).
    size_t max_filesize_bytes (void) const
    {
      return user_blocksize_bytes () * max_filesize_blocks ();
    }

    // size of the raw block in bytes.
    unsigned int raw_blocksize_bytes (void) const { return m_rw_block_size; }

    // size of the user data in the block.
    unsigned int user_blocksize_bytes (void) const;

    // size of the partition in user blocks, not including the partition header.
    size_t partition_size_blocks (void) const
    {
      return m_header->partition_size_rw_blocks ();
    }

    // number of free blocks in the partition.  a free block is one that
    // is not actively being used, which could be a blank block, a block
    // with invalid checksum (garbage data), old file data block etc.
    size_t partition_free_blocks (void) const
    {
      #ifndef FS_FLASH_BFS_READ_ONLY
	return m_free_blocks;
      #else
	return 0;
      #endif
    }

    size_t partition_size_bytes (void) const
    {
      return partition_size_blocks () * raw_blocksize_bytes ();
    }

    virtual sizes_t sizes (void) const override;

    virtual std::string_view name (void) const override
    {
      return { m_header->m_partition_name.data () };
    }

    virtual std::pair<void*, void*> block_addr_begin_end (void) const override
    {
      return { (void*)block_addr_begin (), (void*)block_addr_end () };
    }

    // stl iterator style range.  the block type and size is determined
    // dynamically, thus the best we can do is return void-star.
    const void* blocks_begin (void) const { return (const void*)block_addr_begin (); }
    const void* blocks_end (void) const { return (const void*)block_addr_end (); }

    void* blocks_begin (void) { return (void*)block_addr_begin (); }
    void* blocks_end (void) { return (void*)block_addr_end (); }

    // open_file opens an existing file.
    // if it does not exist, it returns nullptr.

    // create_file creates a new file.  if it already exists, it fails.
    // if something goes wrong it returns nullptr.

    virtual std::unique_ptr<fs::file> open_file (std::string_view name) const override;
    virtual std::unique_ptr<fs::file> open_file (uint32_t numid) const override;
    virtual std::unique_ptr<fs::file> create_file (std::string_view name) override;
    virtual std::unique_ptr<fs::file> create_file (uint32_t numid) override;

    virtual std::unique_ptr<fs::directory> root_directory (void) const override
    {
      return nullptr;
    }


    // erases the file in the partition.
    // FIXME: if there are still open file objects, this will result in
    // funny things happening.
    bool erase_file (std::string_view name);
    bool erase_file (uint32_t numid);

  private:
    uintptr_t block_addr_begin (void) const { return m_block_addr_begin; }
    uintptr_t block_addr_end (void) const { return m_block_addr_end; }

    checksum_func_t checksum_func (void) const;
    size_t checksum_size_bytes (void) const;
    uintptr_t find_next_blank_block (uintptr_t bstart) const;
    bool is_invalid_block (uintptr_t b) const;

    uintptr_t garbage_collect (void);

    bool gc_move_rw_block (unsigned int block_idx_src,
			   unsigned int block_idx_dst,
			   uint8_t* tmp_buf);

    bool gc_erase_unused_blocks (void);
    bool gc_erase_unused_blocks (unsigned int idx, uintptr_t addr);
    void dump_log_info_blocks (void);

    enum erase_block_state_t
    {
      // all rw blocks in the erase block are used
      all_used,

      // all rw blocks in the erase block are unused
      all_unused,

      // some rw blocks are used, some are unused
      partially_used,

      // something went wrong
      invalid_block
    };

    erase_block_state_t erase_block_state (dev::flash_memory::block_info eb) const;

    void update_opened_file_block_map (const uint8_t* block_data, uintptr_t new_block_addr);

    void init_using_blocks (void);

    uintptr_t addr_to_index (uintptr_t addr) const
    {
      return (addr - m_block_addr_begin) >> m_rw_block_size_log2;
    }

    partition_header* m_header;
    dev::flash_memory& m_flash_dev;

    // first data block address
    uintptr_t m_block_addr_begin;

    // first data block address of the block that follows the last block.
    uintptr_t m_block_addr_end;

    unsigned int m_rw_block_size;
    unsigned int m_rw_block_size_log2;

    // records the number of free blocks in the partition that can be
    // used for writing new data blocks.  a free block can be a blank block,
    // garbage data (invalid checksum), old file data and so on.
    // when building for read-only, we don't care about the free block
    // count in the partition, as this is only needed when writing to files.
    #ifndef FS_FLASH_BFS_READ_ONLY
    size_t m_free_blocks;
    #endif

    // when building for read-only, we don't care about multiple opened
    // files.  because no write operations are performed all file block
    // lists can't be changed (by the garbage collection) after a file has
    // been opened.  this saves some code.
    // FIXME: SP-32 store files by value, support multiple opened files.
    #ifndef FS_FLASH_BFS_READ_ONLY
      mutable std::map<uint32_t, fs::file*> m_opened_files;
    #endif
    mutable std::vector<bool> m_using_blocks;

    friend class flash_bfs;
  };

  // locate the named partition and mount it.
  // returns nullptr if partition does not exist or is otherwise invalid.

  static std::unique_ptr<partition>
  mount_partition (dev::flash_memory& fldev, std::string_view name);

  // create a new partition with the specified parameters and mount it.
  // the read/write block size is set to that of the flash device.
  // the partition size is the number of read/write blocks of the flash device.
  // 'offset_rw_block_count' specifies the offset in the flash device where
  // the partition should be created.
  //
  // all erase blocks of the flash device that will be covered by this
  // partition will be erased first.

  // returns nullptr if something goes wrong.
  static std::unique_ptr<partition>
  create_partition (dev::flash_memory& fldev,
		    std::string_view name,
		    checksum_algorithm_t chkalgo,
		    byte_order_t bo,
		    block_type_t bt,
		    size_t partition_rw_block_count,
		    size_t offset_rw_block_count = 0);

private:
  static constexpr utils::byte_order_t conv_byte_order (byte_order_t val)
  {
    return val == little_endian ? utils::little_endian : utils::big_endian;
  }

  struct partition_header
  {
    static constexpr unsigned int min_header_size (void) { return 48; }

    template <typename T> T read_field (const T& ref) const
    {
      return utils::to_native (conv_byte_order (m_byte_order), ref);
    }

    template <typename T, typename S> void write_field (T& ref, S val)
    {
      ref = utils::native_to (conv_byte_order (m_byte_order), val);
    }

    unsigned int header_size (void) const
    {
      return read_field (m_partition_header_size)*4 + min_header_size ();
    }
    void set_header_size (unsigned int val)
    {
      write_field (m_partition_header_size, (val - min_header_size ())/4);
    }

    uint32_t offset_rw_blocks (void) const
    {
      // do not try to read the field if the actual header size is not big
      // enough to contain it.  this can happen when reading old format
      // partition headers, which don't contain the offset field.
      return offsetof (partition_header, m_offset) + sizeof (decltype (m_offset))
	     > header_size () - sizeof (decltype (m_checksum))
	     ? 0
	     : read_field (m_offset);
    }

    uint32_t partition_size_rw_blocks (void) const  { return read_field (m_partition_size); }
    void set_partition_size_rw_blocks (uint32_t val) { write_field (m_partition_size, val); }

    uint16_t rw_block_size (void) const { return read_field (m_block_size); }
    void set_rw_block_size (uint16_t val) { write_field (m_block_size, val); }


    std::array<char, 16> m_magic_string;

    // zero terminated string.  max string length is 15 chars.
    std::array<char, 16> m_partition_name;

    // the checksum algorithm that is being used.
    checksum_algorithm_t m_checksum_algorithm;

    // the endianness of word fields that follow from here on and in
    // each block.
    byte_order_t m_byte_order;

    // the block type that is being used.
    block_type_t m_block_type;

    // size of the partition header in 4-byte words minus 48.
    // the very first header version had a size of 48 bytes, and this field
    // was reserved and always set to zero.  so 48 is the minimum header size.
    uint8_t m_partition_header_size;

    // size of one raw block in bytes in this partition.  must be power-of-2.
    // must also be the same as the flash device's read/write block size.
    uint16_t m_block_size;

    uint16_t m_reserved1;

    // size of the parition in number of read/write blocks.
    uint32_t m_partition_size;

    // if the partition header is located in an erase block, along with normal
    // data blocks, it will need to be erased during garbage collection.  after
    // the block with the partition header has been erased, there is a chance
    // that it can't be written back properly, which would destroy the whole
    // partition.  to avoid that, allow the partition header to be stored
    // anywhere in the partition area.  the parition header offset relative to
    // the first paritition block number is recorded in the partition header.
    // before erasing a block with the partition header, the parition header
    // will be copied to another location (always aligned on the read/write
    // block size).
    // CAUTION
    // older versions of the parition header don't have this field and the
    // m_checksum field is in this place.
    uint32_t m_offset;

    // reserve max checksum field of 32 bits.
    // if the selected checksum algorithm uses fewer bytes, the leading
    // unused bytes will be filled with zeros.
    // the checksummed data is "partition_header size - checksum_bytes".
    uint32_t m_checksum;
  };

  enum file_op
  {
    // set the name of the file.  the user data of the block is used to store
    // the filename.  the maximum name string length is 27 bytes plus one
    // zero terminating byte.
    // the filename can be set at any point and becomes valid from there on.
    // if a set_filename block is the first (and only) block for the file id,
    // it implicitly creates a new empty file.
    file_op_set_filename = 0xF0,

    // set the size of the file (in blocks) to the new value in the data field.
    // if it appends new blocks to the file, the blocks will be initially
    // implicit all-zero.
    file_op_set_filesize = 0xF1,

    // undefine / erase a file.
    file_op_erase_file = 0xF2,
  };

  struct file_block_1_1_2;

  static const partition_header*
  find_partition_header (dev::flash_memory& fldev, uintptr_t start_ptr, uintptr_t end_ptr);

  static bool validate_supported_partition (const partition_header* hdr);
  static size_t get_partition_size (const partition_header* hdr);

  static size_t checksum_size_bytes (checksum_algorithm_t x)
  {
    return (uint8_t)x & 0x0F;
  }

  static checksum_func_t checksum_func (checksum_algorithm_t x, byte_order_t bo);
};

} // namespace fs
#endif // includeguard_fs_flash_bfs_includeguard
