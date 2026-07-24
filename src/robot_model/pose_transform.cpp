#include "pose_transform.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <vector>

namespace robot_model
{
  namespace
  {

    constexpr double PI = 3.14159265358979323846;
    constexpr double GIMBAL_EPSILON = 1.0e-9;

    void set_error(std::string *error_message, const std::string &message)
    {
      if (error_message != nullptr)
      {
        *error_message = message;
      }
    }

    double deg_to_rad(double degree)
    {
      return degree * PI / 180.0;
    }

    double rad_to_deg(double radian)
    {
      return radian * 180.0 / PI;
    }

    std::vector<double> extract_numbers(const std::string &text)
    {
      std::vector<double> numbers;
      const char *cursor = text.c_str();
      while (*cursor != '\0')
      {
        char *end = nullptr;
        const double number = std::strtod(cursor, &end);
        if (end != cursor)
        {
          if (std::isfinite(number))
          {
            numbers.push_back(number);
          }
          cursor = end;
        }
        else
        {
          ++cursor;
        }
      }
      return numbers;
    }

  } // namespace

  bool Parse_Xyzabc_Pose_Text(const std::string &text,
                              XyzabcPose *pose,
                              std::string *error_message)
  {
    if (pose == nullptr)
    {
      set_error(error_message, "Output pose is null.");
      return false;
    }

    const std::vector<double> numbers = extract_numbers(text);
    if (numbers.size() < pose->size())
    {
      std::ostringstream message;
      message << "Expected 6 numeric values, parsed " << numbers.size() << ".";
      set_error(error_message, message.str());
      return false;
    }

    for (size_t i = 0; i < pose->size(); ++i)
    {
      (*pose)[i] = numbers[i];
    }
    return true;
  }

  Matrix4 Build_Zyx_Pose_Matrix(const XyzabcPose &pose)
  {
    const double a = deg_to_rad(pose[3]);
    const double b = deg_to_rad(pose[4]);
    const double c = deg_to_rad(pose[5]);

    const double ca = std::cos(a);
    const double sa = std::sin(a);
    const double cb = std::cos(b);
    const double sb = std::sin(b);
    const double cc = std::cos(c);
    const double sc = std::sin(c);

    Matrix4 matrix = {};
    matrix[0][0] = ca * cb;
    matrix[0][1] = ca * sb * sc - sa * cc;
    matrix[0][2] = ca * sb * cc + sa * sc;
    matrix[0][3] = pose[0];
    matrix[1][0] = sa * cb;
    matrix[1][1] = sa * sb * sc + ca * cc;
    matrix[1][2] = sa * sb * cc - ca * sc;
    matrix[1][3] = pose[1];
    matrix[2][0] = -sb;
    matrix[2][1] = cb * sc;
    matrix[2][2] = cb * cc;
    matrix[2][3] = pose[2];
    matrix[3][3] = 1.0;
    return matrix;
  }

  XyzabcPose Build_Xyzabc_From_Zyx_Matrix(const Matrix4 &matrix)
  {
    XyzabcPose pose = {};
    pose[0] = matrix[0][3];
    pose[1] = matrix[1][3];
    pose[2] = matrix[2][3];

    const double clamped = std::clamp(matrix[2][0], -1.0, 1.0);
    const double b = std::asin(-clamped);
    const double cb = std::cos(b);

    double a = 0.0;
    double c = 0.0;
    if (std::abs(cb) > GIMBAL_EPSILON)
    {
      a = std::atan2(matrix[1][0], matrix[0][0]);
      c = std::atan2(matrix[2][1], matrix[2][2]);
    }
    else
    {
      a = std::atan2(-matrix[0][1], matrix[1][1]);
    }

    pose[3] = rad_to_deg(a);
    pose[4] = rad_to_deg(b);
    pose[5] = rad_to_deg(c);
    return pose;
  }

  Matrix4 Invert_Rigid_Matrix(const Matrix4 &matrix)
  {
    Matrix4 inverse = {};
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        inverse[row][col] = matrix[col][row];
      }
    }

    for (int row = 0; row < 3; ++row)
    {
      inverse[row][3] = -(inverse[row][0] * matrix[0][3] +
                          inverse[row][1] * matrix[1][3] +
                          inverse[row][2] * matrix[2][3]);
    }
    inverse[3][3] = 1.0;
    return inverse;
  }

  Matrix4 Multiply_Matrices(const Matrix4 &lhs, const Matrix4 &rhs)
  {
    Matrix4 result = {};
    for (int row = 0; row < 4; ++row)
    {
      for (int col = 0; col < 4; ++col)
      {
        for (int k = 0; k < 4; ++k)
        {
          result[row][col] += lhs[row][k] * rhs[k][col];
        }
      }
    }
    return result;
  }

  Matrix4 Build_Relative_Pose_Transform(const XyzabcPose &from_pose, const XyzabcPose &to_pose)
  {
    return Multiply_Matrices(Build_Zyx_Pose_Matrix(to_pose),
                             Invert_Rigid_Matrix(Build_Zyx_Pose_Matrix(from_pose)));
  }

  Point3 Transform_Position(const Matrix4 &matrix, const Point3 &point)
  {
    return {matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2] * point[2] + matrix[0][3],
            matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2] * point[2] + matrix[1][3],
            matrix[2][0] * point[0] + matrix[2][1] * point[1] + matrix[2][2] * point[2] + matrix[2][3]};
  }

  XyzabcPose Transform_Xyzabc_Pose(const Matrix4 &matrix,
                                   const XyzabcPose &pose)
  {
    return Build_Xyzabc_From_Zyx_Matrix(Multiply_Matrices(matrix, Build_Zyx_Pose_Matrix(pose)));
  }

  std::string Format_Xyzabc_Pose(const XyzabcPose &pose)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "[" << pose[0] << ", " << pose[1] << ", " << pose[2]
        << ", " << pose[3] << ", " << pose[4] << ", " << pose[5] << "]";
    return out.str();
  }

  std::string Format_Xyzabc_Pose_Csv(const XyzabcPose &pose)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << pose[0] << ','
        << pose[1] << ','
        << pose[2] << ','
        << pose[3] << ','
        << pose[4] << ','
        << pose[5];
    return out.str();
  }

  std::string Format_Matrix4(const Matrix4 &matrix)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    for (int row = 0; row < 4; ++row)
    {
      out << "  [";
      for (int col = 0; col < 4; ++col)
      {
        if (col > 0)
        {
          out << ", ";
        }
        out << std::setw(12) << matrix[row][col];
      }
      out << "]";
      if (row < 3)
      {
        out << "\n";
      }
    }
    return out.str();
  }

} // namespace robot_model
