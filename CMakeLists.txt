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

option(ENABLE_MV3D_CAMERA "Build the MV3D RGBD camera adapter" ON)
option(ENABLE_HUARAY_CAMERA "Build the HuarayTech 2D camera adapter" ON)
option(BUILD_DESKTOP_APP "Build the hardware-integrated desktop application" ON)

if(BUILD_DESKTOP_APP AND
   (NOT ENABLE_MV3D_CAMERA OR NOT ENABLE_HUARAY_CAMERA))
  message(FATAL_ERROR
    "BUILD_DESKTOP_APP requires both camera adapters. Disable the desktop app "
    "with -DBUILD_DESKTOP_APP=OFF for a hardware-independent core build.")
endif()

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

# Portable frame DTOs shared with the optional Huaray adapter.
add_library(jutze_camera_types INTERFACE)
target_include_directories(jutze_camera_types INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/3rd/jutze_camera"
  "${CMAKE_CURRENT_SOURCE_DIR}/3rd/jutze_camera/include")

if(ENABLE_HUARAY_CAMERA)
  set(HUARAY_SDK_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/3rd/jutze_camera/camera/HuarayTech")
  set(HUARAY_RUNTIME_DIR
      "C:/Program Files/HuarayTech/MV Viewer/Runtime/x64"
      CACHE PATH "HuarayTech MV Viewer x64 runtime directory")
  set(HUARAY_IMPORT_LIBRARY "${HUARAY_SDK_DIR}/lib/MVSDKmd.lib")
  set(HUARAY_RUNTIME_LIBRARY "${HUARAY_RUNTIME_DIR}/MVSDKmd.dll")

  foreach(required_file
      "${HUARAY_SDK_DIR}/include/IMVApi.h"
      "${HUARAY_IMPORT_LIBRARY}"
      "${HUARAY_RUNTIME_LIBRARY}")
    if(NOT EXISTS "${required_file}")
      message(FATAL_ERROR "HuarayTech SDK file was not found: ${required_file}")
    endif()
  endforeach()

  add_library(HuarayTech::MVSDK SHARED IMPORTED GLOBAL)
  set_target_properties(HuarayTech::MVSDK PROPERTIES
    IMPORTED_IMPLIB "${HUARAY_IMPORT_LIBRARY}"
    IMPORTED_LOCATION "${HUARAY_RUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${HUARAY_SDK_DIR}/include")

  add_library(jutze_camera_huaray STATIC
    "${HUARAY_SDK_DIR}/camera_dahua.cpp")
  target_include_directories(jutze_camera_huaray PUBLIC
    "${HUARAY_SDK_DIR}/include")
  target_link_libraries(jutze_camera_huaray PUBLIC
    jutze_camera_types HuarayTech::MVSDK)
endif()

# MV3D RGBD SDK
if(ENABLE_MV3D_CAMERA)
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
endif()

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
  FiltersModeling
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
  "${CMAKE_CURRENT_SOURCE_DIR}/include/application"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/camera"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/kuka"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/net"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/panel"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/point_cloud"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model")

add_library(point_cloud_core INTERFACE)
target_include_directories(point_cloud_core INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include/point_cloud")

add_library(app_paths STATIC
  src/application/app_paths.cpp
  include/application/app_paths.h)
target_include_directories(app_paths PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/application")

add_library(application_core STATIC
  src/application/progress_run_controller.cpp
  include/application/progress_run_controller.h)
target_include_directories(application_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/application")

file(GLOB camera_headers CONFIGURE_DEPENDS "include/camera/*.h" "include/camera/*.hpp")
set(camera_core_sources
  src/camera/camera_2d_cross_template.cpp
  src/camera/camera_2d_image_converter.cpp
  src/camera/camera_frame.cpp
  src/camera/camera_image_converter.cpp
  src/camera/camera_point_cloud_converter.cpp)
add_library(camera_core STATIC ${camera_core_sources})
target_include_directories(camera_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/camera")
target_link_libraries(camera_core PUBLIC
  app_paths point_cloud_core jutze_camera_types pugixml)

add_library(camera INTERFACE)
target_link_libraries(camera INTERFACE camera_core)

if(ENABLE_MV3D_CAMERA)
  set(camera_mv3d_adapter_sources
    src/camera/camera_config_dialog.cpp
    src/camera/camera_device_detail.cpp
    src/camera/camera_manager.cpp
    src/camera/camera_mv3d_format_compatibility.cpp
    src/camera/camera_parameter.cpp
    src/camera/camera_parameter_registry.cpp
    src/camera/camera_service.cpp
    src/camera/camera_stream_config.cpp
    src/camera/mv3d_rgbd.cpp)
  add_library(camera_mv3d_adapter STATIC ${camera_mv3d_adapter_sources})
  target_include_directories(camera_mv3d_adapter PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/camera")
  target_link_libraries(camera_mv3d_adapter PUBLIC
    camera_core MV3DRGBD::SDK wx::base wx::core)
  target_link_libraries(camera INTERFACE camera_mv3d_adapter)
endif()

if(ENABLE_HUARAY_CAMERA)
  add_library(camera_huaray_adapter STATIC
    src/camera/camera_2d_service.cpp)
  target_include_directories(camera_huaray_adapter PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/camera")
  target_link_libraries(camera_huaray_adapter PUBLIC
    camera_core jutze_camera_huaray)
  target_link_libraries(camera INTERFACE camera_huaray_adapter)
endif()

file(GLOB net_sources CONFIGURE_DEPENDS "src/net/*.cpp")
file(GLOB net_headers CONFIGURE_DEPENDS "include/net/*.h" "include/net/*.hpp")
add_library(net STATIC ${net_sources} ${net_headers})
target_include_directories(net PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/net")
target_link_libraries(net PUBLIC asio)

file(GLOB kuka_sources CONFIGURE_DEPENDS "src/kuka/*.cpp")
file(GLOB kuka_headers CONFIGURE_DEPENDS "include/kuka/*.h" "include/kuka/*.hpp")
add_library(kuka STATIC ${kuka_sources} ${kuka_headers})
target_include_directories(kuka PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/kuka")
target_link_libraries(kuka PUBLIC app_paths net)

add_library(robot_connection_controller STATIC
  src/application/robot_connection_controller.cpp
  include/application/robot_connection_controller.h)
target_include_directories(robot_connection_controller PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/application")
target_link_libraries(robot_connection_controller PRIVATE kuka)

file(GLOB robot_model_headers CONFIGURE_DEPENDS
     "include/robot_model/*.h" "include/robot_model/*.hpp")

# Renderer-independent robot data, kinematics and workflow models.
set(robot_model_domain_sources
  src/robot_model/flange_drag_update_coordinator.cpp
  src/robot_model/point_cloud_template_binding.cpp
  src/robot_model/pose_transform.cpp
  src/robot_model/progress_flow_transform.cpp
  src/robot_model/robot_forward_kinematics.cpp
  src/robot_model/robot_inverse_kinematics.cpp
  src/robot_model/robot_joint_state_builder.cpp
  src/robot_model/robot_joint_sweep.cpp
  src/robot_model/robot_kinematic_params.cpp
  src/robot_model/robot_model_state.cpp
  src/robot_model/robot_part_transform.cpp
  src/robot_model/robot_simulation_state.cpp
  src/robot_model/robot_teach_point_store.cpp
  src/robot_model/robot_trajectory.cpp
  src/robot_model/robot_trajectory_planner.cpp
  src/robot_model/robot_trajectory_player.cpp
  src/robot_model/robot_trajectory_session.cpp
  src/robot_model/template_configuration.cpp
  src/robot_model/tool_coordinate.cpp
  src/robot_model/tool_visualization.cpp)
add_library(robot_model_domain STATIC ${robot_model_domain_sources})
target_include_directories(robot_model_domain PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model")

# Filesystem and XML adapters. Public APIs expose domain values, not VTK/wx.
set(robot_model_io_sources
  src/robot_model/eye_to_hand_calibration.cpp
  src/robot_model/point_cloud_template_binding_repository.cpp
  src/robot_model/pose_point_io.cpp
  src/robot_model/robot_model_config_loader.cpp
  src/robot_model/robot_model_repository.cpp
  src/robot_model/robot_model_service.cpp
  src/robot_model/robot_progress_io.cpp
  src/robot_model/robot_trajectory_io.cpp
  src/robot_model/template_configuration_repository.cpp
  src/robot_model/tool_coordinate_repository.cpp
  src/robot_model/tool_visualization_repository.cpp)
add_library(robot_model_io STATIC ${robot_model_io_sources})
target_include_directories(robot_model_io PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model")
target_link_libraries(robot_model_io
  PUBLIC robot_model_domain
  PRIVATE app_paths pugixml)

# Collision owns VTK-backed geometry indexes but has no window-system code.
set(robot_model_collision_sources
  src/robot_model/collision_index_build_service.cpp
  src/robot_model/collision_index_rebuild_coordinator.cpp
  src/robot_model/robot_collision_detector.cpp
  src/robot_model/robot_collision_service.cpp
  src/robot_model/robot_drag_performance_collector.cpp
  src/robot_model/robot_motion_collision_guard.cpp)
add_library(robot_model_collision STATIC ${robot_model_collision_sources})
target_include_directories(robot_model_collision PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model")
target_link_libraries(robot_model_collision PUBLIC
  robot_model_domain ${VTK_LIBRARIES})

# VTK/wx rendering, scene assembly and interactive drag adapters.
set(robot_model_render_sources
  src/robot_model/camera_pose_renderer.cpp
  src/robot_model/coordinate_frame_renderer.cpp
  src/robot_model/flange_axis_translation_dragger.cpp
  src/robot_model/flange_drag_motion_executor.cpp
  src/robot_model/flange_interaction_controller.cpp
  src/robot_model/flange_orientation_dragger.cpp
  src/robot_model/flange_position_dragger.cpp
  src/robot_model/flange_transform_gizmo_renderer.cpp
  src/robot_model/fov_frame_renderer.cpp
  src/robot_model/robot_calibration_builder.cpp
  src/robot_model/robot_mesh_loader.cpp
  src/robot_model/robot_model_view.cpp
  src/robot_model/robot_render_controller.cpp
  src/robot_model/robot_scene_assembly.cpp
  src/robot_model/view_cube_controller.cpp
  src/robot_model/vtk_scene.cpp)
add_library(robot_model_render STATIC ${robot_model_render_sources})
target_include_directories(robot_model_render PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robot_model")
target_link_libraries(robot_model_render
  PUBLIC robot_model_domain robot_model_collision wx::gl ${VTK_LIBRARIES}
  PRIVATE robot_model_io wx::base wx::core)

file(GLOB point_cloud_sources CONFIGURE_DEPENDS "src/point_cloud/*.cpp")
file(GLOB point_cloud_headers CONFIGURE_DEPENDS
     "include/point_cloud/*.h" "include/point_cloud/*.hpp")
if(NOT ENABLE_MV3D_CAMERA)
  list(REMOVE_ITEM point_cloud_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/src/point_cloud/point_cloud_view.cpp")
endif()
add_library(point_cloud STATIC ${point_cloud_sources} ${point_cloud_headers})
target_include_directories(point_cloud PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/point_cloud")
target_link_libraries(point_cloud PUBLIC
  app_paths
  point_cloud_core
  camera
  robot_model_domain
  robot_model_io
  robot_model_render
  wx::base
  wx::core
  ${VTK_LIBRARIES})

set(panel_sources)
set(panel_headers)
if(BUILD_DESKTOP_APP)
  file(GLOB panel_sources CONFIGURE_DEPENDS "src/panel/*.cpp")
  file(GLOB panel_headers CONFIGURE_DEPENDS "include/panel/*.h" "include/panel/*.hpp")
  add_library(panel STATIC ${panel_sources} ${panel_headers})
  target_include_directories(panel PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/panel")
  target_link_libraries(panel PUBLIC
    application_core
    app_paths
    robot_connection_controller
    camera
    net
    point_cloud
    robot_model_domain
    robot_model_io
    robot_model_render
    wx::aui
    wx::base
    wx::core
    wx::gl)

  add_executable(test_app WIN32 src/main.cpp)
  set_target_properties(test_app PROPERTIES OUTPUT_NAME test)
  target_include_directories(test_app PRIVATE ${project_include_dirs})
  target_link_libraries(test_app PRIVATE panel)

  vtk_module_autoinit(TARGETS test_app MODULES ${VTK_LIBRARIES})
endif()

if(MSVC)
  set(msvc_project_targets
      app_paths
      application_core
      robot_connection_controller
      camera_core
      net
      kuka
      robot_model_domain
      robot_model_io
      robot_model_collision
      robot_model_render
      point_cloud)
  if(ENABLE_MV3D_CAMERA)
    list(APPEND msvc_project_targets camera_mv3d_adapter)
  endif()
  if(ENABLE_HUARAY_CAMERA)
    list(APPEND msvc_project_targets
      jutze_camera_huaray camera_huaray_adapter)
  endif()
  if(BUILD_DESKTOP_APP)
    list(APPEND msvc_project_targets panel test_app)
  endif()
  foreach(target IN LISTS msvc_project_targets)
    target_compile_options(${target} PRIVATE /FS /utf-8)
  endforeach()
endif()

if(WIN32 AND BUILD_DESKTOP_APP)
  file(GLOB HUARAY_RUNTIME_FILES CONFIGURE_DEPENDS
       "${HUARAY_RUNTIME_DIR}/*.dll"
       "${HUARAY_RUNTIME_DIR}/*.cti"
       "${HUARAY_RUNTIME_DIR}/*.properties")
  add_custom_command(TARGET test_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_app>
            $<TARGET_FILE_DIR:test_app>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${MV3DRGBD_RUNTIME_FILES}
            $<TARGET_FILE_DIR:test_app>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${HUARAY_RUNTIME_FILES}
            $<TARGET_FILE_DIR:test_app>
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
    if(WIN32)
      add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND}
                "-DRUNTIME_DLLS=$<JOIN:$<TARGET_RUNTIME_DLLS:${target}>,|>"
                "-DDESTINATION=$<TARGET_FILE_DIR:${target}>"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/copy_runtime_dlls.cmake"
        VERBATIM)
    endif()
  endfunction()

  add_project_test(robot_model_logic_tests robot_model_logic_tests.cpp)
  target_link_libraries(robot_model_logic_tests PRIVATE
    robot_model_domain robot_model_io robot_model_collision robot_model_render)

  add_project_test(app_paths_tests app_paths_tests.cpp)
  target_link_libraries(app_paths_tests PRIVATE app_paths)

  add_project_test(progress_run_controller_tests
                   progress_run_controller_tests.cpp)
  target_link_libraries(progress_run_controller_tests PRIVATE application_core)

  add_project_test(robot_connection_controller_tests
                   robot_connection_controller_tests.cpp)
  target_link_libraries(robot_connection_controller_tests PRIVATE
                        robot_connection_controller)

  add_project_test(robot_model_panel_boundary_tests
                   robot_model_panel_boundary_tests.cpp)
  target_link_libraries(robot_model_panel_boundary_tests PRIVATE wx::core)

  add_project_test(robot_kinematics_core_tests robot_kinematics_core_tests.cpp)
  target_link_libraries(robot_kinematics_core_tests PRIVATE robot_model_domain)

  add_project_test(robot_collision_detector_tests
                   robot_collision_detector_tests.cpp)
  target_link_libraries(robot_collision_detector_tests PRIVATE
    robot_model_domain robot_model_io robot_model_collision robot_model_render)
  vtk_module_autoinit(TARGETS robot_collision_detector_tests
                      MODULES ${VTK_LIBRARIES})

  add_project_test(tool_coordinate_tests tool_coordinate_tests.cpp)
  target_link_libraries(tool_coordinate_tests PRIVATE
    robot_model_domain robot_model_io)

  add_project_test(template_configuration_tests
                   template_configuration_tests.cpp)
  target_link_libraries(template_configuration_tests PRIVATE
    robot_model_domain robot_model_io)

  add_project_test(camera_frame_buffer_tests camera_frame_buffer_tests.cpp)
  target_link_libraries(camera_frame_buffer_tests PRIVATE camera)

  if(ENABLE_MV3D_CAMERA)
    add_project_test(camera_device_detail_tests camera_device_detail_tests.cpp)
    target_link_libraries(camera_device_detail_tests PRIVATE camera_mv3d_adapter)

    add_project_test(camera_parameter_tests camera_parameter_tests.cpp)
    target_link_libraries(camera_parameter_tests PRIVATE camera_mv3d_adapter)

    add_project_test(camera_stream_config_tests camera_stream_config_tests.cpp)
    target_link_libraries(camera_stream_config_tests PRIVATE camera_mv3d_adapter)
  endif()

  add_project_test(kuka_protocol_tests kuka_protocol_tests.cpp)
  target_link_libraries(kuka_protocol_tests PRIVATE kuka)

  add_project_test(kuka_robot_service_tests kuka_robot_service_tests.cpp)
  target_link_libraries(kuka_robot_service_tests PRIVATE kuka)

  add_project_test(camera_image_converter_tests camera_image_converter_tests.cpp)
  target_link_libraries(camera_image_converter_tests PRIVATE camera)

  add_project_test(camera_2d_image_converter_tests
                   camera_2d_image_converter_tests.cpp)
  target_link_libraries(camera_2d_image_converter_tests PRIVATE camera)

  add_project_test(camera_point_cloud_converter_tests camera_point_cloud_converter_tests.cpp)
  target_link_libraries(camera_point_cloud_converter_tests PRIVATE camera)

  add_project_test(point_cloud_persistence_tests point_cloud_persistence_tests.cpp)
  target_link_libraries(point_cloud_persistence_tests PRIVATE point_cloud)

  add_project_test(point_cloud_document_tests point_cloud_document_tests.cpp)
  target_link_libraries(point_cloud_document_tests PRIVATE point_cloud)

  add_project_test(point_cloud_selection_tests point_cloud_selection_tests.cpp)
  target_link_libraries(point_cloud_selection_tests PRIVATE point_cloud)

  add_project_test(robot_base_point_cloud_builder_tests
                   robot_base_point_cloud_builder_tests.cpp)
  target_link_libraries(robot_base_point_cloud_builder_tests PRIVATE point_cloud)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES
             src/main.cpp ${camera_core_sources} ${camera_mv3d_adapter_sources}
             ${kuka_sources} ${net_sources}
             ${robot_model_domain_sources} ${robot_model_io_sources}
             ${robot_model_collision_sources} ${robot_model_render_sources}
             ${point_cloud_sources} ${panel_sources})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "Header Files" FILES
             ${camera_headers} ${kuka_headers} ${net_headers} ${robot_model_headers}
             ${point_cloud_headers} ${panel_headers})
