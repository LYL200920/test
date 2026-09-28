cmake_minimum_required(VERSION 3.0)

project (flash_bfs_tests)

import_library (utils)
import_library (board)
import_library (dev)
import_library (hash)
import_library (fs)
import_library (logging)


set (flash_bfs_tests_src
  main.cpp
  config.cpp
)

add_target_executable (flash_bfs_tests.elf
  ${flash_bfs_tests_src}
)

target_link_libraries (flash_bfs_tests.elf
  hash
  fs
)

target_link_library_group (flash_bfs_tests.elf
  logging
  utils
  dev
  board
  board_crt stdc++ m c supc++ gcc
)

add_additional_executable_targets (flash_bfs_tests.elf)

