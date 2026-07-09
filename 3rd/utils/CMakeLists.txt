cmake_minimum_required(VERSION 3.12)

project (utils)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++17")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 17)
  message (FATAL_ERROR "this library requires at least c++17")
endif ()

add_target_library (utils
  text.cpp
)
