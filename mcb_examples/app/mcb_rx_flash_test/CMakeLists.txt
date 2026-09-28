cmake_minimum_required(VERSION 3.0)

project (mcb_rx_flash_test)

import_library (utils)
import_library (board)
import_library (dev)
import_library (version_tag)
import_library (logging)

add_target_executable (mcb_rx_flash_test.elf
  main.cpp
)

target_link_libraries (mcb_rx_flash_test.elf
  utils
  version_tag
)

target_link_library_group (mcb_rx_flash_test.elf
  logging
  utils
  dev
  board board_crt stdc++ supc++ m c gcc
)

set_default_version_tag_product_name ("RX Flash Test")
set_default_version_tag_copyright_str ("Copyright (C) 2015, Jutze Japan Co.,Ltd.")

add_version_tag (mcb_rx_flash_test.elf)

add_additional_executable_targets (mcb_rx_flash_test.elf)
