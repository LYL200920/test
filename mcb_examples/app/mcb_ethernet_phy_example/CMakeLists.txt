cmake_minimum_required(VERSION 3.0)

project (mcb_ethernet_phy_example)

import_library (utils)
import_library (board)
import_library (dev)
import_library (logging)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED 14)
set (CMAKE_CXX_EXTENSIONS OFF)

add_target_executable (mcb_ethernet_phy_example.elf
  main.cpp
)

target_link_library_group (mcb_ethernet_phy_example.elf
  logging
  utils
  dev
  board stdc++ supc++ m c board_crt gcc
)

add_additional_executable_targets (mcb_ethernet_phy_example.elf)

