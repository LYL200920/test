
project (fs)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++17")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 17)
  message (FATAL_ERROR "this library requires at least c++17")
endif ()

#set (CMAKE_CXX_STANDARD 14)
#set (CMAKE_CXX_STANDARD_REQUIRED 14)
#set (CMAKE_CXX_EXTENSIONS OFF)

add_target_library (fs
  fs.cpp
  romfs.cpp
  flash_bfs.cpp
)
