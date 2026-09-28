
/*
the version tag information is stored as plain text, zero terminated strings and
compiled into a special ELF section which is linked into the binary.

dependencies such as versions of libraries and communication protocols
could also be included here.  but on the other hand, this information
is recorded in the commit itself and can be reconstructed from the
commit hash.
still, it is possible to have a product version info in each linked library,
which are then all merged into a single section of the final ELF.  the first
entry is always the main entry, followed by sub entries.

*/

#include <string_view>

class version_tag
{
public:
  // use default version_tag section as start address.
  version_tag (void);

  // construct from the starting address (e.g. in ROM).
  version_tag (const void* data_begin, const void* data_end)
  : m_data_begin (data_begin), m_data_end (data_end) { }

  // if the data size is not known, assume 256.
  version_tag (const void* data_begin)
  : m_data_begin (data_begin), m_data_end ((const char*)data_begin + 256) { }

  std::string_view
  product_name (void) const { return str (str_num::product); }

  std::string_view
  tag (void) const { return str (str_num::version_tag); }

  // the number of changes/commits in the repository.  in svn this is a revision
  // number, in git it's a commit counter.
  std::string_view
  num (void) const { return str (str_num::version_num); }

  // the number of changes/commits that happened after the tag has been assigned.
  std::string_view
  tick (void) const { return str (str_num::version_tick); }

  // if present designates a source tree with some local modifications or
  // untracked files.
  std::string_view
  aux (void) const { return str (str_num::version_aux); }

  std::string_view
  copyright (void) const { return str (str_num::copyright); }

  std::string_view
  repository_uuid (void) const { return str (str_num::repository_uuid); }

  std::string_view
  commit_hash (void) const { return str (str_num::commit_hash); }

  std::string_view
  commit_time (void) const { return str (str_num::date); }

  // MD5 checksum of final ELF file, calculated with the MD5 sum field being
  // set to "00000000000000000000000000000000".  the field is then rewritten
  // in the final ELF by the write_elf_md5.sh tool.
  std::string_view
  build_hash (void) const { return str (str_num::build_hash); }

  enum struct str_num
  {
    product,
    version_tag,
    version_num,
    version_tick,
    version_aux,
    copyright,
    repository_uuid,
    commit_hash,
    date,
    build_hash,
  };

  std::string_view str (unsigned int i) const
  {
    return i <= str_num_max
	   ? str ((str_num)i)
	   : std::string_view ();
  }

  std::string_view str (str_num n) const;


  static constexpr unsigned int str_num_max = (unsigned int)str_num::build_hash;
  static constexpr unsigned int str_num_count = str_num_max + 1;

private:

  const void* m_data_begin;
  const void* m_data_end;
};

