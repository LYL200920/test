#include "camera_formats.h"

#include "Mv3dRgbdDefine.h"

#include <cstdint>

static_assert(camera_formats::Mono8 == static_cast<std::uint32_t>(ImageType_Mono8));
static_assert(camera_formats::Mono16 == static_cast<std::uint32_t>(ImageType_Mono16));
static_assert(camera_formats::Depth == static_cast<std::uint32_t>(ImageType_Depth));
static_assert(camera_formats::Yuv422 == static_cast<std::uint32_t>(ImageType_YUV422));
static_assert(camera_formats::Nv12 == static_cast<std::uint32_t>(ImageType_YUV420SP_NV12));
static_assert(camera_formats::Nv21 == static_cast<std::uint32_t>(ImageType_YUV420SP_NV21));
static_assert(camera_formats::Rgb8_Planar == static_cast<std::uint32_t>(ImageType_RGB8_Planar));
static_assert(camera_formats::Point_Cloud == static_cast<std::uint32_t>(ImageType_PointCloud));
static_assert(camera_formats::Point_Cloud_With_Normals == static_cast<std::uint32_t>(ImageType_PointCloudWithNormals));
static_assert(camera_formats::Textured_Point_Cloud == static_cast<std::uint32_t>(ImageType_TexturedPointCloud));
static_assert(camera_formats::Textured_Point_Cloud_With_Normals == static_cast<std::uint32_t>(ImageType_TexturedPointCloudWithNormals));
static_assert(camera_formats::Depth_Coordinates == static_cast<std::uint32_t>(CoordinateType_Depth));
static_assert(camera_formats::Rgb_Coordinates == static_cast<std::uint32_t>(CoordinateType_RGB));
static_assert(camera_formats::Xyz_Record_Size == sizeof(MV3D_RGBD_POINT_3D_F32));
static_assert(camera_formats::Xyz_Normals_Record_Size == sizeof(MV3D_RGBD_POINT_XYZ_NORMALS));
static_assert(camera_formats::Xyz_Rgba_Record_Size == sizeof(MV3D_RGBD_POINT_XYZ_RGB));
static_assert(camera_formats::Xyz_Rgba_Normals_Record_Size == sizeof(MV3D_RGBD_POINT_XYZ_RGB_NORMALS));
