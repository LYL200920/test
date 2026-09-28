/*

there are at least two ways how a ROM FS image can be stored.

1) the ROM FS image resides in the same address space as the executable ROM
   image.  this means the program can read-access the ROM space at any time.
   -> file mapping functions are possible.

2) the executable ROM image is copied into RAM leaving the ROM FS image in ROM.
   when reading file data from the ROM FS, data blocks have to be copied
   from ROM into RAM.  if necessary, some bus locking can be done and DMA can
   be used (for async reads).
   -> file mapping functions are not possible.  application has to read files
      using read functions.

   -> on systems with MMU it's also possible to have mapping.



for now implement only type 1) above.

type 2) requires a rom device some kind of thing.  e.g. it could be an SPI
flash device or something similar, which needs special hardware access.
in fact, type 1) is a special case of type 2).
*/

#ifndef includeguard_fs_romfs_hpp_includeguard
#define includeguard_fs_romfs_hpp_includeguard

#include <array>
#include <limits>

#include <fs/fs.hpp>
#include <utils/byte_order.hpp>

namespace fs
{

class romfs
{
public:
  static constexpr unsigned int block_size = 32;

  struct partition_header
  {
    // __ROM_FS_1______
    std::array<char, 16> magic_string;

    // zero terminated string.  max string length is 15 chars.
    std::array<char, 16> partition_name;

    // the endianness of word fields that follow from here on and in
    // each block.  0 = little endian, non-zero = big endian.
    int8_t byte_order;

    int8_t reserved_0;
    int8_t reserved_1;
    int8_t reserved_2;

    // the number of file entries that follows this header.
    // the size of a file entry is fixed, so the size of the whole block
    // can be calculated.
    uint32_t file_entries_count;

    // the size of the string table.  strings are dynamically sized.
    uint32_t string_table_size;

    uint32_t partition_size_blocks;

    uint32_t reserved_5;
    uint32_t reserved_6;
    uint32_t reserved_7;
    uint32_t reserved_8;

    // data that follows the partition header:
    // - file entries
    // - string table
    // - file data
  };

  static_assert (sizeof (partition_header) == 64, "");

  struct string_table_entry
  {
    // string length in bytes, excl. zero terminator.
    uint32_t length;

    // zero terminated string data follows here.  an empty string consists
    // of at least one zero terminating character.
    // string data is padded to a 4 byte alignment.
  };

  static_assert (sizeof (string_table_entry) == 4, "");

  struct file_entry
  {
    // all *_ptr fields are data pointers, relative to the start of the
    // partition header.  this allows for simple address calculations when
    // the whole romfs image is mapped, because all values are relative to the
    // image base address.

    // numerical file id.
    uint32_t numid;

    // size of the file data in bytes.
    uint32_t size_bytes;

    // file data.
    uint32_t data_ptr;

    // name string.  if there is no name, will be 0.
    uint32_t name_str_ptr;

    // mime type string.
    uint32_t mime_str_ptr;

    uint32_t reserved_0;

    // numerical time stamps.  a value of 0 means invalid/unknown.
    uint64_t created_timestamp;
    uint64_t modified_timestamp;

    // pre-formatted textual time stamps.  if not present, 0.
    // can be used for things like http "last modified" timestamps.
    uint32_t created_timestamp_str_ptr;
    uint32_t modified_timestamp_str_ptr;
  };

  static_assert (sizeof (file_entry) == 48, "");

  class partition;

  class file : public fs::file
  {
  public:
    file (partition* p, const file_entry* fe);
    file (const file&) = delete;
    file& operator = (const file&) = delete;

    virtual size_t read (file_size_t start_block, size_t block_count, void* out) const override;
    virtual size_t read (void* out, size_t num_bytes) const override;

    virtual size_t write (const void*, file_size_t, size_t) override;
    virtual size_t write (const void*, size_t) override;
    virtual bool rename (std::string_view) override;

    virtual fs::time_info_t time_info (void) const override;

    virtual std::string_view mime_type (void) const override;

    virtual const void* mmap_rd (void) const override;

  protected:
    const file_entry& m_fe;
  };

  class partition : public fs::partition
  {
  public:
    partition (const partition&) = delete;
    partition& operator = (const partition&) = delete;

    partition (const void* start_addr, const void* end_addr);

    virtual sizes_t sizes (void) const override;
    virtual std::string_view name (void) const override;

    virtual std::unique_ptr<fs::file> open_file (std::string_view name) const override;
    virtual std::unique_ptr<fs::file> open_file (uint32_t numid) const override;

    virtual std::unique_ptr<fs::file> create_file (std::string_view name) override;
    virtual std::unique_ptr<fs::file> create_file (uint32_t numid) override;

    virtual std::unique_ptr<fs::directory> root_directory (void) const override;

    virtual std::pair<void*, void*> block_addr_begin_end (void) const override;

  private:
    const partition_header& header (void) const;

    utils::byte_order_t byte_order (void) const;

    const file_entry* files_begin (void) const;
    const file_entry* files_end (void) const;

    const char* strings_begin (void) const;
    const char* strings_end (void) const;

    template <typename OffsetField>
    std::string_view str_field (const OffsetField& offset_ptr) const;


    const char* m_start_ptr;
    const char* m_end_ptr;

    friend class file;
  };

  static const romfs::partition_header*
  find_partition_header (const char* ptr, const char* end_ptr);

  // try to mount a partition in the given address range.  the partition's
  // name must match the specified name.
  // returns nullptr if not found.
  static std::unique_ptr<partition>
  mount_partition (const void* start_addr, const void* end_addr,
		   std::string_view name);

  // try to mount the first partition that is found in the given address range.
  // returns nullptr if not found.
  static std::unique_ptr<partition>
  mount_partition (const void* start_addr, const void* end_addr);

  // try to mount the first partition that is found in the romfs that has
  // been included into the program image which is currenctly running.
  // returns nullptr if not found.
  static std::unique_ptr<partition>
  mount_this_image_partition (void);
};


} // namespace fs
#endif // includeguard_fs_romfs_hpp_includeguard
