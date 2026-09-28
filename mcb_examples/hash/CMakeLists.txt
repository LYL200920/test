
# various checksum and hash algorithms

project (hash)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 14)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

#set (CMAKE_CXX_STANDARD 14)
#set (CMAKE_CXX_STANDARD_REQUIRED 14)
#set (CMAKE_CXX_EXTENSIONS OFF)

add_target_library (hash
  crc.cpp
)
