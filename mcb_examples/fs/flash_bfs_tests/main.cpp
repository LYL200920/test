#include <cstdio>
#include <chrono>

#define log_all
#include "logging/logging.hpp"

#include "board/board.hpp"
#include "config.hpp"

int main(void)
{
  log_level::enable (log_level::all);
  auto& board = this_board::inst ();

  board.exec ();

  constexpr int COUNT = 600;
  io_config cfg;
  for (int i=0;i<COUNT;++i)
  {
    cfg = read_io_config();
    uint32_t inc(1);
    io_config new_cfg { cfg.inputs_and_mask ().to_ulong () + inc,
                        cfg.inputs_or_mask ().to_ulong () + inc,
                        cfg.inputs_xor_mask ().to_ulong () + inc,
                        cfg.outputs_init_mask ().to_ulong () + inc,
                        cfg.outputs_and_mask ().to_ulong () + inc,
                        cfg.outputs_or_mask ().to_ulong () + inc,
                        cfg.outputs_xor_mask ().to_ulong () + inc};
    write_io_config(new_cfg);
    log_info("new_io_cfg: 0x%08lx 0x%08lx 0x%08lx 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n",
            new_cfg.inputs_and_mask ().to_ulong (), new_cfg.inputs_or_mask ().to_ulong (),
            new_cfg.inputs_xor_mask ().to_ulong (), new_cfg.outputs_init_mask ().to_ulong (),
            new_cfg.outputs_and_mask ().to_ulong (), new_cfg.outputs_or_mask ().to_ulong (),
            new_cfg.outputs_xor_mask ().to_ulong ());
  }

  return 0;
}