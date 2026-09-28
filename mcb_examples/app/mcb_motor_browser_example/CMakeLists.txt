cmake_minimum_required(VERSION 3.0)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED 17)
set (CMAKE_CXX_EXTENSIONS OFF)

project (mcb_motor_browser_example)

import_library (utils)
import_library (board)
import_library (uip)
import_library (net)
import_library (dev)
import_library (hash)
import_library (version_tag)
import_library (fs)
import_library (logging)

set (mcb_motor_browser_example_src
  main.cpp
  mcb_binary_service.cpp
  mcb_binary_server.cpp
)

set (mcb_motor_browser_example_romfs_files
  romfs_files/favicon.ico 	favicon.ico	"image/x-icon"
)

add_target_executable (mcb_motor_browser_example.elf
  ${mcb_motor_browser_example_src}
)

target_link_libraries (mcb_motor_browser_example.elf
  uip
  net
  version_tag
  hash
  fs
)

target_link_library_group (mcb_motor_browser_example.elf
  logging
  utils
  dev
  board board_crt stdc++ m c supc++ gcc
)

set_default_version_tag_product_name ("MCB Motor Browser Example")
set_default_version_tag_copyright_str ("Copyright (C) 2016-2018, Jutze Japan Co.,Ltd.")

add_version_tag (mcb_motor_browser_example.elf)
add_romfs (mcb_motor_browser_example.elf ${TARGET_ENDIAN} ${mcb_motor_browser_example_romfs_files})

add_additional_executable_targets (mcb_motor_browser_example.elf)

