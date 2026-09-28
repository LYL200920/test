
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cstring>
#include <limits>
#include <set>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "romfs.hpp"

static size_t ceil_blocksize (size_t val)
{
  return ((val + fs::romfs::block_size - 1) / fs::romfs::block_size)
	 * fs::romfs::block_size;
}

static size_t ceil_strlen (size_t val)
{
  return ((val + 3) / 4) * 4;
}



int main (int argc, const char* argv[])
{
  if (argc < 4)
  {
    std::cerr << "not enough arguments" << std::endl;
    return -1;
  }

  std::string out_file = argv[1];
  std::string endianess = argv[2];
  std::string files_dir = argv[3];
  std::string in_files = argv[4];

  utils::byte_order_t target_endian;

  if (endianess == "little")
    target_endian = utils::little_endian;
  else if (endianess == "big")
    target_endian = utils::big_endian;
  else
  {
    std::cerr << "error: unknown target endian setting" << std::endl;
    exit (1);
  }

  const std::string partition_name = "romfs";
  const std::string magic_string = "__ROM_FS_1______";

  if (files_dir.back () != '/')
    files_dir += '/';

//  std::cout << "XXX files_dir = " << files_dir << std::endl;
//  std::cout << "XXX out file = " << out_file << std::endl;
//  std::cout << "XXX in files = " << in_files << std::endl;

  // force classic "C" locale, which we need for time string formatting.
  std::locale::global (std::locale::classic ());

  // ----------------------------------------------------------------------

  struct string_table_entry
  {
    std::string str;

    // offset of the string in the romfs image.
    mutable uint32_t offset;

    string_table_entry (const std::string& s) : str (s), offset (0) { }

    bool operator < (const string_table_entry& rhs) const
    {
      return str < rhs.str;
    }
  };

  struct input_file
  {
    fs::romfs::file_entry file_entry;

    std::set<string_table_entry>::const_iterator filename_str;
    std::set<string_table_entry>::const_iterator mimetype_str;
    std::set<string_table_entry>::const_iterator created_time_str;
    std::set<string_table_entry>::const_iterator modified_time_str;

    std::string in_file_name;


    input_file (utils::byte_order_t target_endian,
		uint32_t file_numid,
		const std::string& file_name,
		const std::string& romfs_name,
		const std::string& mime_type,
		std::set<string_table_entry>& string_table)
    {
      std::cout << "make_romfs input_file "
		<< file_name << " " << romfs_name << " " << mime_type << std::endl;

      struct stat fs;
      if (stat (file_name.c_str (), &fs) != 0)
      {
	std::cerr << "error: can't stat " << file_name << std::endl;
	exit (1);
      }

      auto file_size_bytes = fs.st_size;
      auto modified_time = fs.st_mtime;
      char modified_time_str_buf[128];

      auto modified_time_str_len =
	std::strftime (modified_time_str_buf, sizeof (modified_time_str_buf) - 1,
			"%c", std::gmtime (&modified_time));

      // copy the formatted date string and remove double spaces.
      std::string modified_time_str;
      modified_time_str.reserve (modified_time_str_len);

      for (const char* i = modified_time_str_buf; *i != '\0'; ++i)
      {
	modified_time_str += *i;

	if (std::isspace (*i))
	{
	  for (; std::isspace (*i) && *i != '\0'; ++i) { }

	  if (*i != '\0')
	    --i;
	}
      }

      std::cout << "   file size = " << file_size_bytes << " bytes" << std::endl;
      std::cout << "   modified time = " << modified_time
		<< " " << modified_time_str << std::endl;

      std::memset (&file_entry, 0, sizeof (file_entry));
      this->in_file_name = file_name;
      this->filename_str = string_table.emplace (romfs_name).first;
      this->mimetype_str = string_table.emplace (mime_type).first;
      this->created_time_str = string_table.emplace ("").first;
      this->modified_time_str = string_table.emplace (modified_time_str).first;

      if (file_size_bytes > std::numeric_limits<decltype (this->file_entry.size_bytes)>::max ())
      {
	std::cerr << "error: file too big " << file_name << std::endl;
	exit (1);
      }

      this->file_entry.numid = utils::native_to (target_endian, file_numid);
      this->file_entry.size_bytes = utils::native_to (target_endian, file_size_bytes);
      this->file_entry.modified_timestamp = utils::native_to (target_endian, modified_time);
    }
  };

  // ----------------------------------------------------------------------

  std::vector<std::string> in_files_str;

  for (auto i = in_files.begin (); i != in_files.end (); )
  {
    auto ii = std::find (i, in_files.end (), '*');

    in_files_str.emplace_back (i, ii);

    if (ii == in_files.end ())
      break;
    i = std::next (ii);
  }

  if (in_files_str.size () % 3 != 0)
  {
    std::cerr << "input files arguments must be multiple of 3" << std::endl;
    exit (1);
  }

  std::set<string_table_entry> string_table;
  std::vector<input_file> input_files;

  for (size_t i = 0; i < in_files_str.size (); i += 3)
  {
    // if the specified path is absolute, don't add the prefix.
    // FIXME: use std::filesystem
    auto use_filename = in_files_str[i+0].front () == '/'
			? in_files_str[i+0]
			: (files_dir + in_files_str[i+0]);

    input_files.emplace_back (target_endian,
			      i / 3,
			      use_filename,
			      in_files_str[i+1], in_files_str[i+2],
			      string_table);
  }

  size_t file_entries_size = input_files.size () * sizeof (fs::romfs::file_entry);
  file_entries_size = ceil_blocksize (file_entries_size);

  size_t string_table_size = 0;
  for (const auto& s : string_table)
  {
    s.offset = sizeof (fs::romfs::partition_header) + file_entries_size
	       + string_table_size;

    string_table_size += ceil_strlen (s.str.size () + 1)
			 + sizeof (fs::romfs::string_table_entry);
  }
  string_table_size = ceil_blocksize (string_table_size);

  size_t file_data_size = 0;
  for (auto& f : input_files)
  {
    f.file_entry.data_ptr = utils::native_to (target_endian,
			      sizeof (fs::romfs::partition_header)
			      + file_entries_size
			      + string_table_size
			      + file_data_size);

    f.file_entry.name_str_ptr = utils::native_to (target_endian,
				  f.filename_str->offset);
    f.file_entry.mime_str_ptr = utils::native_to (target_endian,
				  f.mimetype_str->offset);
    f.file_entry.created_timestamp_str_ptr = utils::native_to (target_endian,
				  f.created_time_str->offset);
    f.file_entry.modified_timestamp_str_ptr = utils::native_to (target_endian,
				  f.modified_time_str->offset);

    file_data_size += ceil_blocksize (
		utils::to_native (target_endian, f.file_entry.size_bytes));
  }

  std::cout << "file entries size = " << file_entries_size << std::endl;
  std::cout << "string table size = " << string_table_size << std::endl;
  std::cout << "file data size = " << file_data_size << std::endl;


  // ----------------------------------------------------------------------

  fs::romfs::partition_header header;
  std::memset (&header, 0, sizeof (header));

  std::memcpy (header.magic_string.data (), magic_string.data (),
	       std::min (magic_string.size (), header.magic_string.size ()));

  std::memcpy (header.partition_name.data (), partition_name.data (),
	       std::min (partition_name.size (), header.partition_name.size ()));

  header.byte_order = target_endian == utils::little_endian ? 0 : 1;

  header.file_entries_count = utils::native_to (target_endian,
	(uint32_t)input_files.size ());

  header.string_table_size = utils::native_to (target_endian,
	(uint32_t)string_table_size);

  header.partition_size_blocks = utils::native_to (target_endian,
	(sizeof (fs::romfs::partition_header)
	 + file_entries_size
	 + string_table_size
	 + file_data_size) / fs::romfs::block_size);

  std::cout << "partition size blocks = "
	    << utils::to_native (target_endian, header.partition_size_blocks)
	    << std::endl;

//  std::ofstream fout (out_file + ".bin", std::ios_base::out | std::ios_base::binary);
  std::stringstream fout;

  std::vector<int8_t> padding (1024, 0);

  // write partition header
  {
    fout.write ((const char*)&header, sizeof (header));
  }

  // write file entries
  {
    size_t written_count = 0;
    for (const auto& f : input_files)
    {
      fout.write ((const char*)&f.file_entry, sizeof (f.file_entry));
      written_count += sizeof (f.file_entry);
    }

    fout.write ((const char*)padding.data (), file_entries_size - written_count);
  }

  // write string table
  {
    size_t written_count = 0;
    for (const auto& s : string_table)
    {
      fs::romfs::string_table_entry e;
      e.length = utils::native_to (target_endian, s.str.size ());

      fout.write ((const char*)&e, sizeof (e));
      size_t c = sizeof (e);

      fout.write ((const char*)s.str.c_str (), s.str.size () + 1);
      c += s.str.size () + 1;

      fout.write ((const char*)padding.data (), ceil_strlen (c) - c);

      written_count += c + (ceil_strlen (c) - c);
    }

    fout.write ((const char*)padding.data (), string_table_size - written_count);
  }

  // write file data
  {
    size_t written_count = 0;

    std::vector<uint8_t> tmp_buf (1024*4);

    for (const auto& f : input_files)
    {
      std::ifstream fin (f.in_file_name, std::ios_base::in | std::ios_base::binary);

      size_t sz = 0;
      for (; sz < utils::to_native (target_endian, f.file_entry.size_bytes); )
      {
	fin.read ((char*)tmp_buf.data (), tmp_buf.size ());
	size_t read_bytes = fin.gcount ();
	fout.write ((const char*)tmp_buf.data (), read_bytes);

	sz += read_bytes;
	written_count += read_bytes;

	if (read_bytes == 0 || !fin.good ())
	  break;
      }

      if (sz != utils::to_native (target_endian, f.file_entry.size_bytes))
      {
	std::cerr << "error reading/writing file " << f.in_file_name << std::endl;
	exit (1);
      }

      fout.write ((const char*)padding.data (), ceil_blocksize (written_count) - written_count);
      written_count = ceil_blocksize (written_count);
    }
  }


  // convert binary data into c source.

  // if this turns out to be a problem, can also use the target's objdump to
  // convert the binary file to an elf, instead of using the compiler.
  // using the compiler is easier as it will pick up the build's options etc.

  std::ofstream fout_c (out_file, std::ios_base::out | std::ios_base::binary);

  fout_c << R"raw(

#include <cstdint>
#include <cstdlib>

extern "C" alignas (32) const uint8_t __romfs_data[] =
{

)raw";


  {
    fout.seekg (0);
    uint8_t tmpbuf[16];

    while (true)
    {
      fout.read ((char*)tmpbuf, sizeof (tmpbuf));

      int sz = fout.gcount ();

      for (int i = 0; i < sz; ++i)
	fout_c << (unsigned int)tmpbuf[i] << ',';

      fout_c << '\n';

      if (sz < sizeof (tmpbuf))
	break;
    }
  }

  fout_c << R"raw(
};

extern "C" const size_t __romfs_size = sizeof (__romfs_data);

)raw";

  return 0;
}
