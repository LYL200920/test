# jutze_3d_camera

Standalone C++17 wrapper for the MV3D RGBD SDK. The module owns SDK discovery,
device lifecycle, parameter access, stream metadata and frame acquisition. It
does not depend on wxWidgets, VTK, OpenCV or application UI code.

## Embedding

```cmake
add_subdirectory(3rd/jutze_3d_camera EXCLUDE_FROM_ALL)
target_link_libraries(my_target PRIVATE
  jutze_3d_camera::jutze_3d_camera)
```

Existing builds can continue supplying `MV3DRGBD_DEV_ROOT`,
`MV3DRGBD_RUNTIME_DIR` and `MV3D_RUNTIME_DIR` as CMake cache variables.

For DTO-only or hardware-independent builds, configure with:

```text
-DJUTZE_3D_CAMERA_ENABLE_MV3D=OFF
```

and link `jutze_3d_camera::types`.
