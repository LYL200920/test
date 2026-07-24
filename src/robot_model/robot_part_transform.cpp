#include "robot_part_transform.h"

#include <cmath>

namespace robot_model
{
  namespace
  {

    constexpr double PI = 3.14159265358979323846;

    Matrix4 identity_matrix()
    {
      Matrix4 matrix = {};
      for (std::size_t index = 0; index < 4; ++index)
        matrix[index][index] = 1.0;
      return matrix;
    }

    Matrix4 rotation_x(double degrees)
    {
      auto matrix = identity_matrix();
      const double radians = degrees * PI / 180.0;
      const double cosine = std::cos(radians);
      const double sine = std::sin(radians);
      matrix[1][1] = cosine;
      matrix[1][2] = -sine;
      matrix[2][1] = sine;
      matrix[2][2] = cosine;
      return matrix;
    }

    Matrix4 rotation_y(double degrees)
    {
      auto matrix = identity_matrix();
      const double radians = degrees * PI / 180.0;
      const double cosine = std::cos(radians);
      const double sine = std::sin(radians);
      matrix[0][0] = cosine;
      matrix[0][2] = sine;
      matrix[2][0] = -sine;
      matrix[2][2] = cosine;
      return matrix;
    }

    Matrix4 rotation_z(double degrees)
    {
      auto matrix = identity_matrix();
      const double radians = degrees * PI / 180.0;
      const double cosine = std::cos(radians);
      const double sine = std::sin(radians);
      matrix[0][0] = cosine;
      matrix[0][1] = -sine;
      matrix[1][0] = sine;
      matrix[1][1] = cosine;
      return matrix;
    }

    Matrix4 translation(const double values[3])
    {
      auto matrix = identity_matrix();
      matrix[0][3] = values[0];
      matrix[1][3] = values[1];
      matrix[2][3] = values[2];
      return matrix;
    }

  } // namespace

  Matrix4 Build_Part_Calibration_Matrix(
      const Robot_Part_Calibration &calibration)
  {
    return Multiply_Matrices(Multiply_Matrices(Multiply_Matrices(rotation_x(calibration.rotate_xyz[0]),
                                                                 rotation_y(calibration.rotate_xyz[1])),
                                               rotation_z(calibration.rotate_xyz[2])),
                             translation(calibration.translate));
  }

  Point3 Transform_Part_Point(const Robot_Part_Calibration &calibration,
                              const Point3 &point)
  {
    return Transform_Position(Build_Part_Calibration_Matrix(calibration), point);
  }

} // namespace robot_model
