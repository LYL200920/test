cmake_minimum_required(VERSION 3.0)

project (board)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 14)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

add_target_library (board_info
  board_info.cpp
)
