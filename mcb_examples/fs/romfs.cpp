
#include <cstdlib>
#include <cstring>

#include "romfs.hpp"

#define log_all
#include <logging/logging.hpp>

extern "C" [[gnu::weak]] const uint8_t __romfs_data[];
extern "C" [[gnu::weak]] const size_t __romfs_size;

namespace fs
{

static constexpr char magic_string[16 + 1] = "__ROM_FS_1______";
static constexpr size_t magic_string_len = 16;

template <typename T> static constexpr T
read_field (utils::byte_order_t bo, const T& ref)
{
#ifdef ROMFS_1_NATIVE_BYTE_ORDER_ONLY
  return ref;
#else
  return utils::to_native (bo, ref);
#endif
}

template <typename T, typename S> static constexpr void
write_field (utils::byte_order_t bo, T& ref, S val)
{
#ifdef ROMFS_1_NATIVE_BYTE_ORDER_ONLY
  ref = val;
#else
  ref = utils::native_to (bo, ref);
#endif
}

// ----------------------------------------------------------------------------

const romfs::partition_header*
romfs::find_partition_header (const char* start_ptr, const char* end_ptr)
{
  log_info ("find_partition_header start = 0x%p end = 0x%p\n", start_ptr, end_ptr);

  for (const char* ptr = start_ptr; ptr != end_ptr; ptr += sizeof (partition_header))
    if (std::memcmp (ptr, magic_string, magic_string_len) == 0)
    {
      auto&& h = (const partition_header*)ptr;

      #ifdef ROMFS_1_NATIVE_BYTE_ORDER_ONLY
	if (h->byte_order != utils::native_byte_order ())
	  continue;
      #endif

      return h;
    }

  return nullptr;
}

std::unique_ptr<romfs::partition>
romfs::mount_partition (const void* start_addr, const void* end_addr,
			std::string_view name)
{
  log_info ("ROMFS mount partition start_addr = %p  end_addr = %p\n",
	    start_addr, end_addr);

  for (const char* ptr = (const char*)start_addr, *end_ptr = (const char*)end_addr;
       ptr != end_ptr; )
  {
    auto&& h = find_partition_header (ptr, end_ptr);

    if (h == nullptr)
      break;

    if (partition (ptr, end_ptr).name () == name)
      return std::make_unique<partition> (ptr, end_ptr);
  }

  return nullptr;
}

std::unique_ptr<romfs::partition>
romfs::mount_partition (const void* start_addr, const void* end_addr)
{
  log_info ("ROMFS mount partition start_addr = %p  end_addr = %p\n",
	    start_addr, end_addr);

  for (const char* ptr = (const char*)start_addr, *end_ptr = (const char*)end_addr;
       ptr != end_ptr; )
  {
    auto&& h = find_partition_header (ptr, end_ptr);

    if (h == nullptr)
      break;

    return std::make_unique<partition> (ptr, end_ptr);
  }

  return nullptr;
}

std::unique_ptr<romfs::partition>
romfs::mount_this_image_partition (void)
{
  if (__romfs_data == nullptr)
    return nullptr;

  return fs::romfs::mount_partition (__romfs_data, __romfs_data + __romfs_size);
}

// ----------------------------------------------------------------------------

romfs::partition::partition (const void* start_addr, const void* end_addr)
: m_start_ptr ((const char*)start_addr),
  m_end_ptr ((const char*)end_addr)
{
}

const romfs::partition_header& romfs::partition::header (void) const
{
  return *(const partition_header*)m_start_ptr;
}

utils::byte_order_t romfs::partition::byte_order (void) const
{
#ifdef ROMFS_1_NATIVE_BYTE_ORDER_ONLY
  return utils::native_byte_order ();
#else
  return header ().byte_order ? utils::big_endian : utils::little_endian;
#endif
}

const romfs::file_entry* romfs::partition::files_begin (void) const
{
  return (const romfs::file_entry*)(&header () + 1);
}

const romfs::file_entry* romfs::partition::files_end (void) const
{
  return files_begin ()
	 + read_field (byte_order (), header ().file_entries_count);
}

const char* romfs::partition::strings_begin (void) const
{
  return (const char*)files_end ();
}

const char* romfs::partition::strings_end (void) const
{
  return strings_begin ()
	 + read_field (byte_order (), header ().string_table_size);
}

template <typename OffsetField>
std::string_view
romfs::partition::str_field (const OffsetField& offset_ptr) const
{
  auto bo = byte_order ();

  const char* p = m_start_ptr + read_field (bo, offset_ptr);
  uint32_t len = *(const uint32_t*)p;

  return { p + sizeof (len), len };
}

partition::sizes_t romfs::partition::sizes (void) const
{
  return
  {
    block_size, block_size,
    std::numeric_limits<uint32_t>::max (),
    std::numeric_limits<uint32_t>::max () / block_size,
    read_field (byte_order (), header ().partition_size_blocks)
  };
}

std::string_view romfs::partition::name (void) const
{
  return header ().partition_name.data ();
}

std::unique_ptr<fs::file>
romfs::partition::open_file (std::string_view name) const
{
  for (auto&& f = files_begin (), f_end = files_end (); f != f_end; ++f)
  {
    auto f_name = str_field (f->name_str_ptr);
    if (f_name == name)
      return std::make_unique<file> (const_cast<partition*> (this), f);
  }

  return nullptr;
}

std::unique_ptr<fs::file>
romfs::partition::open_file (uint32_t numid) const
{
  numid = utils::native_to (byte_order (), numid);

  for (auto&& f = files_begin (), f_end = files_end (); f != f_end; ++f)
    if (f->numid == numid)
      return std::make_unique<file> (const_cast<partition*> (this), f);

  return nullptr;
}

std::unique_ptr<fs::file>
romfs::partition::create_file (std::string_view name)
{
  return nullptr;
}

std::unique_ptr<fs::file>
romfs::partition::create_file (uint32_t name)
{
  return nullptr;
}

std::unique_ptr<fs::directory>
romfs::partition::root_directory (void) const
{
  return nullptr;
}

std::pair<void*, void*>
romfs::partition::block_addr_begin_end (void) const
{
  return { (void*)m_start_ptr, (void*)m_end_ptr };
}

// ----------------------------------------------------------------------------

romfs::file::file (partition* p, const file_entry* fe)
: fs::file (p), m_fe (*fe)
{
  auto bo = p->byte_order ();

  m_numid = read_field (bo, fe->numid);
  m_block_size = romfs::block_size;
  m_size_bytes = read_field (bo, fe->size_bytes);
  m_size_blocks = (m_size_bytes + romfs::block_size - 1) / romfs::block_size;
  m_name = p->str_field (fe->name_str_ptr);
}

size_t
romfs::file::read (file_size_t start_block, size_t block_count, void* out) const
{
  partition* p = (partition*)&*m_partition;
  auto bo = p->byte_order ();

  if (start_block >= m_size_blocks)
    return 0;

  file_size_t end_block = std::min (start_block + block_count, m_size_blocks);

  block_count = end_block - start_block;

  const char* file_data_begin = p->m_start_ptr + read_field (bo, m_fe.data_ptr);
  file_data_begin += start_block * romfs::block_size;

  std::memcpy (out, file_data_begin, romfs::block_size * block_count);

  return block_count;
}

size_t romfs::file::read (void* out, size_t num_bytes) const
{
  partition* p = (partition*)&*m_partition;
  auto bo = p->byte_order ();

  if (m_size_bytes < num_bytes)
    num_bytes = (size_t)m_size_bytes;

  const char* file_data_begin = p->m_start_ptr + read_field (bo, m_fe.data_ptr);

  std::memcpy (out, file_data_begin, num_bytes);

  return num_bytes;
}

size_t romfs::file::write (const void*, file_size_t, size_t)
{
  return 0;
}

size_t romfs::file::write (const void*, size_t)
{
  return 0;
}

bool romfs::file::rename (std::string_view)
{
  return false;
}

time_info_t romfs::file::time_info (void) const
{
  partition* p = (partition*)&*m_partition;
  auto bo = p->byte_order ();

  return
  {
    read_field (bo, m_fe.created_timestamp),
    p->str_field (m_fe.created_timestamp_str_ptr),

    read_field (bo, m_fe.modified_timestamp),
    p->str_field (m_fe.modified_timestamp_str_ptr),

    0, ""
  };
}

std::string_view romfs::file::mime_type (void) const
{
  partition* p = (partition*)&*m_partition;

  return p->str_field (m_fe.mime_str_ptr);
}

const void* romfs::file::mmap_rd (void) const
{
  partition* p = (partition*)&*m_partition;
  auto bo = p->byte_order ();
  return p->m_start_ptr + read_field (bo, m_fe.data_ptr);
}


} // namespace fs
