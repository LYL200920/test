
project (board)
enable_language(ASM)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 14)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

#set (CMAKE_CXX_STANDARD 14)
#set (CMAKE_CXX_STANDARD_REQUIRED 14)
#set (CMAKE_CXX__EXTENSIONS OFF)

board_toolchain_add_linkerscript ("linker_script.c")

add_target_library (board STATIC
  ../board_info.cpp
  board.cpp
  board.S
  debug_write_e1.c

  # the dummy linker script file to create a dependency on the actual
  # linker script.
  linker_script.c
)

# the C runtime support library is separated because some applications
# might not want to use it but something else, e.g. tiny apps like the
# tftp bootloader.
add_target_library (board_crt STATIC
  ../crt.cpp
)
