#ifndef includeguard_pose_transform_h_includeguard
#define includeguard_pose_transform_h_includeguard

#include <array>
#include <string>

namespace robot_model
{

using XyzabcPose = std::array<double, 6>;
using Point3 = std::array<double, 3>;
using Matrix4 = std::array<std::array<double, 4>, 4>;

bool Parse_Xyzabc_Pose_Text (const std::string& text,
                             XyzabcPose* pose,
                             std::string* error_message);

Matrix4 Build_Zyx_Pose_Matrix (const XyzabcPose& pose);
XyzabcPose Build_Xyzabc_From_Zyx_Matrix (const Matrix4& matrix);

Matrix4 Invert_Rigid_Matrix (const Matrix4& matrix);
Matrix4 Multiply_Matrices (const Matrix4& lhs, const Matrix4& rhs);

Matrix4 Build_Relative_Pose_Transform (const XyzabcPose& from_pose,
                                       const XyzabcPose& to_pose);

Point3 Transform_Position (const Matrix4& matrix, const Point3& point);
XyzabcPose Transform_Xyzabc_Pose (const Matrix4& matrix,
                                  const XyzabcPose& pose);

std::string Format_Xyzabc_Pose (const XyzabcPose& pose);
std::string Format_Xyzabc_Pose_Csv (const XyzabcPose& pose);
std::string Format_Matrix4 (const Matrix4& matrix);

} // namespace robot_model

#endif
