
#ifndef includeguard_fs_fs_hpp_includeguard
#define includeguard_fs_fs_hpp_includeguard

#include <cstdint>
#include <memory>
#include <limits>
#include <string_view>

namespace fs
{
typedef uint64_t file_size_t;

struct time_info_t
{
  uint64_t created;
  std::string_view created_str;

  uint64_t modified;
  std::string_view modified_str;

  uint64_t last_accessed;
  std::string_view last_accessed_str;
};

// ---------------------------------------------------------------------

class file;
class directory;
class link;
class partition;

class file
{
public:
  file (const file&) = delete;
  file (file&&) = delete;
  file& operator = (const file&) = delete;
  file& operator = (file&&) = delete;

  virtual ~file (void);

  // the numerical file id.
  uint32_t numid (void) const { return m_numid; }

  // if this is an unnamed file the name will be an empty string.
  std::string_view name (void) const { return m_name; }

  // the current file size in bytes.
  file_size_t size (void) const { return m_size_bytes; }

  // the current file size in blocks.
  file_size_t size_blocks (void) const { return m_size_blocks; }

  uint32_t block_size (void) const { return m_block_size; }

  // read the specified number of bytes and store them at the specified
  // memory location.  returns the number of blocks read.
  virtual size_t read (file_size_t start_block, size_t block_count, void* out) const = 0;

  // read the specified number of bytes and store them at the specified
  // memory location.  returns the number of bytes read.
  // this always reads from file offset 0.
  virtual size_t read (void* out, size_t num_bytes) const = 0;

  // write the specified data into the file.
  // returns the number of blocks written.
  // if the data is bigger than the current file size, the file is extended.
  virtual size_t write (const void* in, file_size_t start_block, size_t block_count) = 0;

  // write the specified data into the file.
  // returns the number of bytes written.
  // this always writes from file offset 0.
  // if the data is bigger than the current file size, the file is extended.
  virtual size_t write (const void* in, size_t num_bytes) = 0;

  // rename the file.
  // returns true on success or false otherwise.
  virtual bool rename (std::string_view name) = 0;

  virtual time_info_t time_info (void) const = 0;

  // virtual ... attributes (void) const = 0;

  // a mime type string like "text/html", if known.
  virtual std::string_view mime_type (void) const { return { }; }

  // if possible, memory map the whole file
  virtual const void* mmap_rd (void) const { return nullptr; }

protected:
  // a partition is expected to keep a list of all open files and invalidate
  // the parition reference if an open file can't be used without an oppen
  // partition.  generally the application should take care of keeping
  // the paritions alive as long as it has live file objects.
  // before ref_counting_ptr was used but that results in increased code
  // size, which is an issue for things like tiny bootloaders.
  file (partition* p) noexcept;

  partition* m_partition;

  uint32_t m_numid;
  uint32_t m_block_size;
  std::string_view m_name;
  file_size_t m_size_bytes;
  file_size_t m_size_blocks;
};

// ---------------------------------------------------------------------
// a directory is different from a file.  it's not possible to read or
// write any user data from/to a directory.  the only operation it supports
// is listing the contents and creating new entries (directories or files).
// so the directory provides access to a file system's meta-data structures.

class directory
{
public:

  // an entry in a directory.
  class entry
  {
  public:
    enum type_t : int8_t
    {
      file,
      directory,
      link
    };

    type_t type (void) const { return m_type; }
    std::string_view name (void) const { return m_name; }

    time_info_t time_info (void) const;

    // attributes, user ownership, access permissions ...
    // ...

  protected:
    type_t m_type;
    std::string_view m_name;
  };

  // how to list the entries in a directory?
  // use some pagination technique.
  // must be able to prefetch pages asynchronously in the background.
  class entries_page
  {
  public:
    const entry* begin (void) const { return m_begin; }
    const entry* end (void) const { return m_end; }
    size_t size (void) const { return m_end - m_begin; }
    bool empty (void) const { return m_end == m_begin; }
    bool last_page (void) const;

  protected:
    const entry* m_begin;
    const entry* m_end;

    // unique_ptr to data block or something
  };

  // returns the first page of entries.
  entries_page entries_begin (void) const;

  // returns the next page of entries.
  entries_page entries_next (entries_page& p) const;

  // open an entries from this directory 
  std::shared_ptr<directory> open (const entry& e) const;

protected:
};

// ---------------------------------------------------------------------
// a link ...

class link
{
public:


protected:

};

// ---------------------------------------------------------------------
// a partition/volume

class partition
{
public:
  class sizes_t
  {
  public:
    constexpr sizes_t (uint32_t raw_blocksize_bytes,
			uint32_t user_blocksize_bytes,
			file_size_t max_num_files,
			file_size_t max_file_size_blocks,
			file_size_t partition_size_blocks)
    : m_raw_blocksize_bytes (raw_blocksize_bytes),
      m_user_blocksize_bytes (user_blocksize_bytes),
      m_max_num_files (max_num_files),
      m_max_file_size_blocks (max_file_size_blocks),
      m_partition_size_blocks (partition_size_blocks)
    {
    }

    // maximum number of files in this parition.
    constexpr file_size_t max_num_files (void) const { return m_max_num_files; }

    // maximum filesize in blocks.
    constexpr file_size_t max_filesize_blocks (void) const { return m_max_file_size_blocks; }

    // maximum filesize in bytes (counts only user data bytes).
    constexpr file_size_t max_filesize_bytes (void) const
    {
      return user_blocksize_bytes () * max_filesize_blocks ();
    }

    // size of one raw block in bytes.
    constexpr unsigned int raw_blocksize_bytes (void) const { return m_raw_blocksize_bytes; }

    // size of the user data in the block.
    constexpr unsigned int user_blocksize_bytes (void) const { return m_user_blocksize_bytes; }

    // size of the partition in blocks.
    constexpr file_size_t partition_size_blocks (void) const { return m_partition_size_blocks; }

    // size of the partition in bytes.
    constexpr file_size_t partition_size_bytes (void) const
    {
      return partition_size_blocks () * raw_blocksize_bytes ();
    }

  protected:
    uint32_t m_raw_blocksize_bytes;
    uint32_t m_user_blocksize_bytes;
    file_size_t m_max_num_files;
    file_size_t m_max_file_size_blocks;
    file_size_t m_partition_size_blocks;
  };

  partition (const partition&) = delete;
  partition& operator = (const partition&) = delete;

  virtual ~partition (void) { }

  virtual sizes_t sizes (void) const = 0;
  virtual std::string_view name (void) const = 0;

  // only for storage devices and partitions that can be memory mapped.
  // might return nullptr.
  std::pair<const void*, const void*> mapped_blocks (void) const
  {
    auto r = block_addr_begin_end ();
    return { (const void*)r.first, (const void*)r.second };
  }

  std::pair<void*, void*> mapped_blocks (void)
  {
    auto r = block_addr_begin_end ();
    return { (void*)r.first, (void*)r.second };
  }

  // try to open an existing file in this partition.
  // the file name can be a path with directories etc.
  // returns nullptr on failure.
  virtual std::unique_ptr<file> open_file (std::string_view name) const = 0;
  virtual std::unique_ptr<file> open_file (uint32_t numid) const = 0;

  // try to create a new file in this partition.
  // the file name can be a path with directories etc.
  // returns nullptr on failure.
  virtual std::unique_ptr<file> create_file (std::string_view name) = 0;
  virtual std::unique_ptr<file> create_file (uint32_t numid) = 0;

  // get the root directory.  it can't be created, as it's always there.
  virtual std::unique_ptr<directory> root_directory (void) const = 0;

protected:
  partition (void) { }

  virtual std::pair<void*, void*> block_addr_begin_end (void) const = 0;
};

// ---------------------------------------------------------------------
// these need the definitions of class partition for the ref_counting_ptr
// ctor and dtor.  hence they are implemented here and not in the class def.
//inline file::file (const utils::ref_counting_ptr<partition>& p) noexcept : m_partition (p) { }
inline file::file (partition* p) noexcept : m_partition (std::move (p)) { }

inline file::~file (void) { }

} // namespace fs
#endif // includeguard_fs_fs_hpp_includeguard
