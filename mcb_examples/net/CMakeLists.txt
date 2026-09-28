
project (net)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++17")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 17)
  message (FATAL_ERROR "this library requires at least c++17")
endif ()

add_target_library (net
  net.cpp
  tftp.cpp
  http.cpp
  http_server.cpp
  sllp.cpp
  uri.cpp
)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

target_sources (net
PRIVATE
  net_impl_linux.cpp
)
endif ()
