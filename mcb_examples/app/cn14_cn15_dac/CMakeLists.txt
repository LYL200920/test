cmake_minimum_required(VERSION 3.0)

project (cn14_cn15_dac)

import_library (utils)
import_library (board)
import_library (uip)
import_library (net)
import_library (dev)
import_library (hash)
import_library (fs)
import_library (logging)
import_library (version_tag)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED 14)
set (CMAKE_CXX_EXTENSIONS OFF)

add_target_executable (cn14_cn15_dac.elf
  main.cpp
)

target_link_libraries (cn14_cn15_dac.elf
  board board_crt stdc++ m c supc++ gcc
  uip
  net
  version_tag
  hash
  fs
  logging
  utils
  dev
)

set_default_version_tag_product_name ("MCB CN14 CN15 DAC Example")
set_default_version_tag_copyright_str ("Copyright (C) 2018, Jutze")

add_version_tag (cn14_cn15_dac.elf)
#add_romfs (cn14_cn15_dac.elf ${TARGET_ENDIAN} ${mcb_motor_browser_example_romfs_files})

add_additional_executable_targets (cn14_cn15_dac.elf)

