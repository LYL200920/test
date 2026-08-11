cmake_minimum_required(VERSION 3.21)

project(jutze_3d_camera VERSION 1.0.0 LANGUAGES CXX)

option(JUTZE_3D_CAMERA_ENABLE_MV3D
       "Build the MV3D RGBD backend" ON)
option(JUTZE_3D_CAMERA_BUILD_TESTS
       "Build the jutze_3d_camera tests" ${PROJECT_IS_TOP_LEVEL})

add_library(jutze_3d_camera_types STATIC
  src/camera_frame.cpp
  src/camera_ip_config.cpp)
add_library(jutze_3d_camera::types ALIAS jutze_3d_camera_types)
target_compile_features(jutze_3d_camera_types PUBLIC cxx_std_17)
if(MSVC)
  target_compile_options(jutze_3d_camera_types PRIVATE /utf-8)
endif()
target_include_directories(jutze_3d_camera_types PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

if(JUTZE_3D_CAMERA_BUILD_TESTS)
  enable_testing()
  add_executable(jutze_3d_camera_ip_config_tests
    tests/camera_ip_config_tests.cpp)
  target_link_libraries(jutze_3d_camera_ip_config_tests PRIVATE
    jutze_3d_camera::types)
  add_test(NAME jutze_3d_camera_ip_config_tests
           COMMAND jutze_3d_camera_ip_config_tests)
endif()

if(JUTZE_3D_CAMERA_ENABLE_MV3D)
  if(NOT WIN32)
    message(FATAL_ERROR "The MV3D RGBD backend currently supports Windows only")
  endif()

  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(MV3DRGBD_LIBRARY_ARCH win64)
    set(MV3DRGBD_RUNTIME_ARCH Win64_x64)
  else()
    set(MV3DRGBD_LIBRARY_ARCH win32)
    set(MV3DRGBD_RUNTIME_ARCH Win32_i86)
  endif()

  # Retain the established cache variable names so existing build scripts keep
  # working when this directory is consumed as a submodule.
  set(MV3DRGBD_DEV_ROOT "F:/Software/HiViewer/Development"
      CACHE PATH "MV3D RGBD SDK development directory")
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
  set_property(GLOBAL PROPERTY JUTZE_3D_CAMERA_MV3D_RUNTIME_FILES
               "${MV3DRGBD_RUNTIME_FILES}")

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

  add_library(jutze_3d_camera_mv3d_sdk SHARED IMPORTED GLOBAL)
  add_library(jutze_3d_camera::mv3d_sdk ALIAS jutze_3d_camera_mv3d_sdk)
  set_target_properties(jutze_3d_camera_mv3d_sdk PROPERTIES
    IMPORTED_IMPLIB "${MV3DRGBD_IMPORT_LIBRARY}"
    IMPORTED_LOCATION "${MV3DRGBD_RUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MV3DRGBD_INCLUDE_DIR}")

  add_library(jutze_3d_camera STATIC
    src/camera_device_detail.cpp
    src/camera_manager.cpp
    src/camera_mv3d_format_compatibility.cpp
    src/camera_parameter.cpp
    src/camera_parameter_registry.cpp
    src/camera_stream_config.cpp
    src/mv3d_rgbd.cpp)
  add_library(jutze_3d_camera::jutze_3d_camera ALIAS jutze_3d_camera)
  target_compile_features(jutze_3d_camera PUBLIC cxx_std_17)
  target_compile_definitions(jutze_3d_camera PUBLIC NOMINMAX)
  if(MSVC)
    target_compile_options(jutze_3d_camera PRIVATE /utf-8)
  endif()
  target_include_directories(jutze_3d_camera PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
  target_link_libraries(jutze_3d_camera
    PUBLIC
      jutze_3d_camera::types
      jutze_3d_camera::mv3d_sdk)

  # Consumers call this for an executable that needs the vendor runtime.
  function(jutze_3d_camera_copy_runtime target)
    get_property(runtime_files GLOBAL PROPERTY
                 JUTZE_3D_CAMERA_MV3D_RUNTIME_FILES)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${runtime_files}
              $<TARGET_FILE_DIR:${target}>
      COMMAND_EXPAND_LISTS)
  endfunction()

endif()
