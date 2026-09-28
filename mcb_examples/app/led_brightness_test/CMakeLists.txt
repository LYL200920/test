cmake_minimum_required(VERSION 3.0)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED 17)
set (CMAKE_CXX_EXTENSIONS OFF)

project (led_brightness_test)

import_library (utils)
import_library (board)
import_library (uip)
import_library (net)
import_library (dev)
import_library (hash)
import_library (version_tag)
import_library (fs)
import_library (logging)

set (led_brightness_test_src
  main.cpp
)

add_target_executable (led_brightness_test.elf
  ${led_brightness_test_src}
)

target_link_libraries (led_brightness_test.elf
  uip
  net
  hash
  fs
)

target_link_library_group (led_brightness_test.elf
  logging
  utils
  dev
  board board_crt stdc++ m c supc++ gcc
)

set_default_version_tag_product_name ("MCB led brightness test")
set_default_version_tag_copyright_str ("Copyright (C) 2016-2018, Jutze Japan Co.,Ltd.")

add_version_tag (led_brightness_test.elf)

add_additional_executable_targets (led_brightness_test.elf)

