
#include <exception>
#include <type_traits>

#include "config.hpp"
#include <fs/flash_bfs.hpp>
#include <board/board_info.hpp>
#include <board/board.hpp>

#include <utils/byte_stream.hpp>

#define log_all
#include <logging/logging.hpp>

struct config_write_exception : public std::exception
{
  config_write_exception (const char* msg) : m_msg (msg) { }
  virtual const char* what (void) const noexcept override { return m_msg; }

  const char* m_msg;
};

// -----------------------------------------------------------------------------

static fs::flash_bfs::partition_ptr config_partition = nullptr;

static int config_partition_does_not_exist = -1;

static bool open_config_partition_rd (void)
{
  if (config_partition == nullptr && config_partition_does_not_exist == -1)
  {
    config_partition = fs::flash_bfs::mount_partition (this_board::inst ().fcu, "config");
    config_partition_does_not_exist = config_partition == nullptr;
  }
  return config_partition != nullptr;
}

static void open_config_partition_wr (void)
{
  if (config_partition == nullptr)
    config_partition = fs::flash_bfs::mount_partition (this_board::inst ().fcu, "config");

  if (config_partition == nullptr)
    config_partition =
	fs::flash_bfs::create_partition (this_board::inst ().fcu, "config",
				     fs::flash_bfs::crc16_16_12_5_0,
				     fs::flash_bfs::little_endian,
				     fs::flash_bfs::block_type_1_1_2,
				     32, 16*1024 / 32 /* = 512 blocks */);

  config_partition_does_not_exist = config_partition == nullptr;

  if (config_partition == nullptr)
  {
    log_info ("config partition NG\n");
    throw config_write_exception ("config partition NG");
  }
}

// -----------------------------------------------------------------------------

static utils::ref_counting_ptr<fs::file> io_config_file = nullptr;
static int io_config_file_does_not_exist = -1;

io_config read_io_config (void)
{
  io_config c;

  c.inputs_and_mask_raw = ~0;
  c.inputs_or_mask_raw = 0;
  c.inputs_xor_mask_raw = ~0;

  c.outputs_init_mask_raw = 0;
  c.outputs_and_mask_raw = ~0;
  c.outputs_or_mask_raw = 0;
  c.outputs_xor_mask_raw = 0;

  if (io_config_file_does_not_exist == true)
    return c;

  if (io_config_file == nullptr)
    if (open_config_partition_rd ())
    {
      if (auto f = config_partition->open_file (io_config_fileid))
      {
	io_config_file_does_not_exist = false;
	io_config_file = f;
      }
      else
	io_config_file_does_not_exist = true;
    }

  if (io_config_file != nullptr)
  {
    io_config cc;
    if (io_config_file->read (&cc, sizeof (cc)) == sizeof (cc))
    {
      // FIXME: use special type for serializing/storing values in
      // specific byte order.
      c.inputs_and_mask_raw = utils::to_native (utils::little_endian, cc.inputs_and_mask_raw);
      c.inputs_or_mask_raw = utils::to_native (utils::little_endian, cc.inputs_or_mask_raw);
      c.inputs_xor_mask_raw = utils::to_native (utils::little_endian, cc.inputs_xor_mask_raw);
      c.outputs_init_mask_raw = utils::to_native (utils::little_endian, cc.outputs_init_mask_raw);
      c.outputs_and_mask_raw = utils::to_native (utils::little_endian, cc.outputs_and_mask_raw);
      c.outputs_or_mask_raw = utils::to_native (utils::little_endian, cc.outputs_or_mask_raw);
      c.outputs_xor_mask_raw = utils::to_native (utils::little_endian, cc.outputs_xor_mask_raw);
    }
  }

  return c;
}

void write_io_config (const io_config& val_in)
{
  // FIXME: use special type for serializing/storing values in
  // specific byte order.
  io_config val;
  val.inputs_and_mask_raw = utils::native_to (utils::little_endian, val_in.inputs_and_mask_raw);
  val.inputs_or_mask_raw = utils::native_to (utils::little_endian, val_in.inputs_or_mask_raw);
  val.inputs_xor_mask_raw = utils::native_to (utils::little_endian, val_in.inputs_xor_mask_raw);
  val.outputs_init_mask_raw = utils::native_to (utils::little_endian, val_in.outputs_init_mask_raw);
  val.outputs_and_mask_raw = utils::native_to (utils::little_endian, val_in.outputs_and_mask_raw);
  val.outputs_or_mask_raw = utils::native_to (utils::little_endian, val_in.outputs_or_mask_raw);
  val.outputs_xor_mask_raw = utils::native_to (utils::little_endian, val_in.outputs_xor_mask_raw);

  if (io_config_file != nullptr)
  {
do_write:
    if (io_config_file->write (&val, sizeof (io_config)) != sizeof (io_config))
     {
      log_error ("io config write file NG\n");
      throw config_write_exception ("file write NG");
     }

    log_info ("io config file write OK\n");
    io_config_file_does_not_exist = false;
  }
  else
  {
    open_config_partition_wr ();
    io_config_file = config_partition->open_file (io_config_fileid);

    if (io_config_file == nullptr)
      io_config_file = config_partition->create_file (io_config_fileid);

    if (io_config_file == nullptr)
    {
      log_error ("io config file create NG\n");
      throw config_write_exception ("config file NG");
    }
    goto do_write;
  }
}
