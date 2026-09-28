cmake_minimum_required(VERSION 3.0)

project (tcp_client_example)

import_library (utils)
import_library (dev)
import_library (board)
import_library (uip)
import_library (net)
import_library (logging)

add_target_executable (tcp_client_example.elf
  main.cpp
)

target_link_libraries (tcp_client_example.elf
  utils
  uip
  net
)

target_link_library_group (tcp_client_example.elf
  logging
  utils
  dev
  board board_crt stdc++ supc++ m c gcc
)

add_additional_executable_targets (tcp_client_example.elf)

