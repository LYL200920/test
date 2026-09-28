cmake_minimum_required(VERSION 3.0)
include (ExternalProject)

project (wifi_demo_1)

ExternalProject_Add (esp32
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/esp32
  PREFIX esp32
  TMP_DIR esp32
  BINARY_DIR esp32
  CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_DIR}/../esp32-idf/toolchain.cmake -DCMAKE_LIBRARIES_DIR=${LIBRARIES_DIR}
  BUILD_ALWAYS 1
  BUILD_COMMAND $(MAKE)
  STEP_TARGETS build flash menuconfig
  EXCLUDE_FROM_ALL TRUE
)

ExternalProject_Add_Step (esp32 flash
  WORKING_DIRECTORY esp32
  COMMAND $(MAKE) flash
  DEPENDEES build
)

ExternalProject_Add_Step (esp32 menuconfig
  WORKING_DIRECTORY esp32
  COMMAND $(MAKE) menuconfig
)

add_executable (wifi_demo_1.esp32.bin IMPORTED)
set_property (TARGET wifi_demo_1.esp32.bin PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/esp32/wifi_demo_1.bin")
add_dependencies (wifi_demo_1.esp32.bin esp32-build)

import_library (utils)
import_library (board)
import_library (uip)
import_library (net)
import_library (dev)
import_library (hash)
import_library (fs)
import_library (logging)
import_library (version_tag)

add_target_executable (wifi_demo_1.elf
  main.cpp
)

add_dependencies (wifi_demo_1.elf wifi_demo_1.esp32.bin)

target_link_libraries (wifi_demo_1.elf
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

set_default_version_tag_product_name ("MCB WiFi Demo 1")
set_default_version_tag_copyright_str ("Copyright (C) Jutze")

add_version_tag (wifi_demo_1.elf)
add_additional_executable_targets (wifi_demo_1.elf)

