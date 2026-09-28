
# device drivers and hardware(-abstraction) classes.

project (dev)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 14)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

#set (CMAKE_CXX_STANDARD 14)
#set (CMAKE_CXX_STANDARD_REQUIRED 14)
#set (CMAKE_CXX_EXTENSIONS OFF)

add_target_library (dev
  i2c.cpp
  digital_io_port.cpp
  stm32f0/stm32f0_flash.cpp
  mcx51x_pps_calc_test.cpp
  timer.cpp
)

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "rx")

target_sources (dev
  PRIVATE
    renesas/rx63_fcu.cpp
    renesas/rx64_fcu.cpp
)

endif ()


if (UNIX)

target_sources (dev
  PRIVATE
    linux_userspace_usart.cpp
)

endif ()

if (WIN32)

target_sources (dev
  PRIVATE
    win32_userspace_usart.cpp
)

endif ()
