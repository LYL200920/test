
project (rxlib)

# the C code requires gnu99 for the linux includes to define everything...
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -Werror=return-type -lpthread -pthread")

if (WIN32)

add_target_library (rxlib STATIC
  rxlib.c
  serial_win32.c
)

else ()

add_target_library (rxlib STATIC
  rxlib.c
  serial.c
)

endif ()
