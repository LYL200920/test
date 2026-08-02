#ifndef includeguard_camera_formats_h_includeguard
#define includeguard_camera_formats_h_includeguard

#include <cstddef>
#include <cstdint>

// Stable wire-format values consumed by the hardware-independent camera core.
// The MV3D adapter verifies these values against the vendor SDK at compile time.
namespace camera_formats
{
inline constexpr std::uint32_t Mono8 = 0x01080001U;
inline constexpr std::uint32_t Mono16 = 0x01100007U;
inline constexpr std::uint32_t Depth = 0x011000B8U;
inline constexpr std::uint32_t Yuv422 = 0x02100032U;
inline constexpr std::uint32_t Nv12 = 0x020C8001U;
inline constexpr std::uint32_t Nv21 = 0x020C8002U;
inline constexpr std::uint32_t Rgb8_Planar = 0x02180021U;

inline constexpr std::uint32_t Point_Cloud = 0x026000C0U;
inline constexpr std::uint32_t Point_Cloud_With_Normals = 0x80C00001U;
inline constexpr std::uint32_t Textured_Point_Cloud = 0x80780002U;
inline constexpr std::uint32_t Textured_Point_Cloud_With_Normals = 0x80D80003U;

inline constexpr std::uint32_t Depth_Coordinates = 1U;
inline constexpr std::uint32_t Rgb_Coordinates = 2U;

inline constexpr std::size_t Xyz_Record_Size = sizeof(float) * 3U;
inline constexpr std::size_t Xyz_Normals_Record_Size = sizeof(float) * 6U;
inline constexpr std::size_t Xyz_Rgba_Record_Size = sizeof(float) * 4U;
inline constexpr std::size_t Xyz_Rgba_Normals_Record_Size = sizeof(float) * 7U;
}

#endif
