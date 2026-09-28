
cmake_minimum_required(VERSION 3.0)

project (mcb_bootloader_prog)

import_library (board)
import_library (version_tag)
import_library (board/mcb_v1/tools/dj-flash-tool)

add_target_executable (mcb_bootloader_prog
  main.cpp
)

target_link_libraries (mcb_bootloader_prog
  rxlib
  board_info
  version_tag
)
