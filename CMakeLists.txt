cmake_minimum_required(VERSION 3.21)

project(test VERSION 1.0.0 LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
foreach(config Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER "${config}" config_upper)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_BINARY_DIR}/bin/${config}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_BINARY_DIR}/bin/${config}")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_BINARY_DIR}/lib/${config}")
endforeach()

add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

# Third-party sources retained for this standalone project. OpenCV,
# state_machine and utils are intentionally available but are not built until
# a project target actually needs them.
set(wxBUILD_SHARED ON CACHE BOOL "Build wxWidgets shared libraries" FORCE)
set(wxBUILD_SAMPLES OFF CACHE STRING "Build wxWidgets samples" FORCE)
set(wxBUILD_TESTS OFF CACHE STRING "Build wxWidgets tests" FORCE)
set(wxBUILD_DEMOS OFF CACHE BOOL "Build wxWidgets demos" FORCE)
set(wxBUILD_BENCHMARKS OFF CACHE BOOL "Build wxWidgets benchmarks" FORCE)
set(wxBUILD_INSTALL OFF CACHE BOOL "Create wxWidgets install target" FORCE)
add_subdirectory(3rd/wxWidgets EXCLUDE_FROM_ALL)

add_subdirectory(3rd/pugixml EXCLUDE_FROM_ALL)
target_include_directories(pugixml PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/3rd/pugixml")

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/3rd")
target_compile_definitions(asio INTERFACE ASIO_STANDALONE _WIN32_WINNT=0x0601)

# MV3D RGBD SDK
set(MV3DRGBD_DEV_ROOT "F:/Software/HiViewer/Development"
    CACHE PATH "MV3D RGBD SDK development directory")

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(MV3DRGBD_LIBRARY_ARCH win64)
  set(MV3DRGBD_RUNTIME_ARCH Win64_x64)
else()
  set(MV3DRGBD_LIBRARY_ARCH win32)
  set(MV3DRGBD_RUNTIME_ARCH Win32_i86)
endif()

set(MV3DRGBD_RUNTIME_DIR
    "C:/Program Files (x86)/Common Files/Mv3dRgbdSDK/Runtime/${MV3DRGBD_RUNTIME_ARCH}"
    CACHE PATH "MV3D RGBD SDK runtime directory")
set(MV3D_RUNTIME_DIR
    "C:/Program Files (x86)/Common Files/MV3D/Runtime/${MV3DRGBD_RUNTIME_ARCH}"
    CACHE PATH "MV3D common runtime directory")

set(MV3DRGBD_INCLUDE_DIR "${MV3DRGBD_DEV_ROOT}/Includes")
set(MV3DRGBD_IMPORT_LIBRARY
    "${MV3DRGBD_DEV_ROOT}/Libraries/${MV3DRGBD_LIBRARY_ARCH}/Mv3dRgbd.lib")
set(MV3DRGBD_RUNTIME_LIBRARY "${MV3DRGBD_RUNTIME_DIR}/Mv3dRgbd.dll")
set(MV3DRGBD_RUNTIME_FILES
    "${MV3DRGBD_RUNTIME_LIBRARY}"
    "${MV3DRGBD_RUNTIME_DIR}/MvUdp.dll"
    "${MV3DRGBD_RUNTIME_DIR}/Mv3dRgbdCfg.ini"
    "${MV3D_RUNTIME_DIR}/pthreadVC2.dll")

set(MV3DRGBD_REQUIRED_FILES
    "${MV3DRGBD_INCLUDE_DIR}/Mv3dRgbdApi.h"
    "${MV3DRGBD_INCLUDE_DIR}/Mv3dRgbdDefine.h"
    "${MV3DRGBD_INCLUDE_DIR}/Mv3dRgbdImgProc.h"
    "${MV3DRGBD_IMPORT_LIBRARY}"
    ${MV3DRGBD_RUNTIME_FILES})
foreach(required_file IN LISTS MV3DRGBD_REQUIRED_FILES)
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR "MV3D RGBD SDK file was not found: ${required_file}")
  endif()
endforeach()

add_library(MV3DRGBD::SDK SHARED IMPORTED GLOBAL)
set_target_properties(MV3DRGBD::SDK PROPERTIES
  IMPORTED_IMPLIB "${MV3DRGBD_IMPORT_LIBRARY}"
  IMPORTED_LOCATION "${MV3DRGBD_RUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MV3DRGBD_INCLUDE_DIR}")

# VTK is built and installed by build_on_windows.sh. An explicit VTK_DIR can
# still be supplied when using an existing VTK installation.
set(local_vtk_dir "${CMAKE_CURRENT_SOURCE_DIR}/vtk_build/install/lib/cmake/vtk-9.6")
if((NOT VTK_DIR OR VTK_DIR STREQUAL "VTK_DIR-NOTFOUND") AND EXISTS "${local_vtk_dir}")
  set(VTK_DIR "${local_vtk_dir}" CACHE PATH "VTK config directory" FORCE)
endif()

find_package(VTK REQUIRED COMPONENTS
  CommonCore
  CommonDataModel
  FiltersCore
  FiltersSources
  IOGeometry
  IOPLY
  IOXML
  InteractionStyle
  InteractionWidgets
  RenderingAnnotation
  RenderingCore
  RenderingFreeType
  RenderingOpenGL2
  RenderingUI)

set(project_include_dirs
  "${CMAKE_CURRENT_SOURCE_DIR}/include/camera"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/net"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/panel"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/point_cloud"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model")

file(GLOB camera_sources CONFIGURE_DEPENDS "src/camera/*.cpp")
file(GLOB camera_headers CONFIGURE_DEPENDS "include/camera/*.h" "include/camera/*.hpp")
add_library(camera STATIC ${camera_sources} ${camera_headers})
target_include_directories(camera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/camera")
target_link_libraries(camera PUBLIC MV3DRGBD::SDK wx::base wx::core)

file(GLOB net_sources CONFIGURE_DEPENDS "src/net/*.cpp")
file(GLOB net_headers CONFIGURE_DEPENDS "include/net/*.h" "include/net/*.hpp")
add_library(net STATIC ${net_sources} ${net_headers})
target_include_directories(net PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/net")
target_link_libraries(net PUBLIC asio)

file(GLOB robot_model_sources CONFIGURE_DEPENDS "src/robot_model/*.cpp")
file(GLOB robot_model_headers CONFIGURE_DEPENDS
     "include/robot_model/*.h" "include/robot_model/*.hpp")
add_library(robot_model STATIC ${robot_model_sources} ${robot_model_headers})
target_include_directories(robot_model PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model")
target_link_libraries(robot_model PUBLIC
  pugixml
  wx::base
  wx::core
  wx::gl
  ${VTK_LIBRARIES})

file(GLOB point_cloud_sources CONFIGURE_DEPENDS "src/point_cloud/*.cpp")
file(GLOB point_cloud_headers CONFIGURE_DEPENDS
     "include/point_cloud/*.h" "include/point_cloud/*.hpp")
add_library(point_cloud STATIC ${point_cloud_sources} ${point_cloud_headers})
target_include_directories(point_cloud PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/point_cloud")
target_link_libraries(point_cloud PUBLIC camera robot_model wx::base wx::core ${VTK_LIBRARIES})

file(GLOB panel_sources CONFIGURE_DEPENDS "src/panel/*.cpp")
file(GLOB panel_headers CONFIGURE_DEPENDS "include/panel/*.h" "include/panel/*.hpp")
add_library(panel STATIC ${panel_sources} ${panel_headers})
target_include_directories(panel PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/panel")
target_link_libraries(panel PUBLIC
  camera
  net
  point_cloud
  robot_model
  wx::aui
  wx::base
  wx::core
  wx::gl)

add_executable(test_app WIN32 src/main.cpp)
set_target_properties(test_app PROPERTIES OUTPUT_NAME test)
target_include_directories(test_app PRIVATE ${project_include_dirs})
target_link_libraries(test_app PRIVATE panel)

vtk_module_autoinit(TARGETS test_app MODULES ${VTK_LIBRARIES})

if(MSVC)
  foreach(target camera net robot_model point_cloud panel test_app)
    target_compile_options(${target} PRIVATE /FS /utf-8)
  endforeach()
endif()

if(WIN32)
  add_custom_command(TARGET test_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_app>
            $<TARGET_FILE_DIR:test_app>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${MV3DRGBD_RUNTIME_FILES}
            $<TARGET_FILE_DIR:test_app>
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/Resource"
            "$<TARGET_FILE_DIR:test_app>/Resource"
    COMMAND_EXPAND_LISTS)
endif()

if(BUILD_TESTING)
  function(add_project_test target source)
    add_executable(${target} "tests/${source}")
    target_include_directories(${target} PRIVATE ${project_include_dirs})
    if(MSVC)
      target_compile_options(${target} PRIVATE /FS /utf-8)
    endif()
    add_test(NAME ${target} COMMAND ${target})
  endfunction()

  add_project_test(robot_model_logic_tests robot_model_logic_tests.cpp)
  target_link_libraries(robot_model_logic_tests PRIVATE robot_model)

  add_project_test(camera_frame_buffer_tests camera_frame_buffer_tests.cpp)
  target_link_libraries(camera_frame_buffer_tests PRIVATE camera)

  add_project_test(camera_device_detail_tests camera_device_detail_tests.cpp)
  target_link_libraries(camera_device_detail_tests PRIVATE camera)

  add_project_test(camera_parameter_tests camera_parameter_tests.cpp)
  target_link_libraries(camera_parameter_tests PRIVATE camera)

  add_project_test(camera_stream_config_tests camera_stream_config_tests.cpp)
  target_link_libraries(camera_stream_config_tests PRIVATE camera)

  add_project_test(camera_image_converter_tests camera_image_converter_tests.cpp)
  target_link_libraries(camera_image_converter_tests PRIVATE camera)

  add_project_test(camera_point_cloud_converter_tests camera_point_cloud_converter_tests.cpp)
  target_link_libraries(camera_point_cloud_converter_tests PRIVATE camera)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES
             src/main.cpp ${camera_sources} ${net_sources} ${robot_model_sources}
             ${point_cloud_sources} ${panel_sources})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "Header Files" FILES
             ${camera_headers} ${net_headers} ${robot_model_headers}
             ${point_cloud_headers} ${panel_headers})
