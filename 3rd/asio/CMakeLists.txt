cmake_minimum_required(VERSION 3.10)
project(asio_test)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(asio_test example/main.cpp)

# Windows 下需要链接 winsock2
if(WIN32)
    target_link_libraries(asio_test ws2_32)
endif()
