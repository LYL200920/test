#include "eye_to_hand_calibration.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string_view>
#include <vector>

namespace robot_model
{
namespace
{

void set_error (std::string* error_message, const std::string& message)
{
  if( error_message )
  {
    *error_message = message;
  }
}

std::vector<double> extract_numbers (std::string_view text)
{
  std::vector<double> values;
  std::string storage (text);
  const char* cursor = storage.c_str ( );
  while( *cursor != '\0' )
  {
    char* end = nullptr;
    const double value = std::strtod (cursor, &end);
    if( end != cursor )
    {
      values.push_back (value);
      cursor = end;
    }
    else
    {
      ++cursor;
    }
  }
  return values;
}

bool values_for_key (
  const std::string& text,
  std::string_view key,
  std::size_t expected_count,
  std::vector<double>* values,
  std::string* error_message)
{
  const auto key_position = text.find (key);
  if( key_position == std::string::npos )
  {
    set_error (error_message, "Missing calibration key: " + std::string (key));
    return false;
  }
  const auto open = text.find ('[', key_position + key.size ( ));
  const auto close = open == std::string::npos
    ? std::string::npos
    : text.find (']', open + 1);
  if( open == std::string::npos || close == std::string::npos )
  {
    set_error (error_message,
               "Calibration key has no complete bracketed value: " +
                 std::string (key));
    return false;
  }

  *values = extract_numbers (
    std::string_view (text).substr (open + 1, close - open - 1));
  if( values->size ( ) != expected_count )
  {
    set_error (error_message,
               "Calibration key " + std::string (key) + " expected " +
                 std::to_string (expected_count) + " values, parsed " +
                 std::to_string (values->size ( )));
    return false;
  }
  for( const double value : *values )
  {
    if( !std::isfinite (value) )
    {
      set_error (error_message,
                 "Calibration key contains a non-finite value: " +
                   std::string (key));
      return false;
    }
  }
  return true;
}

bool is_rotation_matrix (const Matrix4& matrix)
{
  constexpr double orthogonality_tolerance = 1.0e-4;
  constexpr double determinant_tolerance = 1.0e-4;
  for( int row = 0; row < 3; ++row )
  {
    for( int other = 0; other < 3; ++other )
    {
      double dot = 0.0;
      for( int column = 0; column < 3; ++column )
      {
        dot += matrix[row][column] * matrix[other][column];
      }
      const double expected = row == other ? 1.0 : 0.0;
      if( std::abs (dot - expected) > orthogonality_tolerance )
      {
        return false;
      }
    }
  }

  const double determinant =
    matrix[0][0] * (
      matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
    matrix[0][1] * (
      matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
    matrix[0][2] * (
      matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
  return std::abs (determinant - 1.0) <= determinant_tolerance;
}

} // namespace

Matrix4 Eye_To_Hand_Calibration::World_From_Camera ( ) const
{
  return camera_to_robot;
}

bool Load_Eye_To_Hand_Calibration (
  const std::filesystem::path& path,
  Eye_To_Hand_Calibration* calibration,
  std::string* error_message)
{
  if( error_message )
  {
    error_message->clear ( );
  }
  if( calibration == nullptr )
  {
    set_error (error_message, "Eye-to-hand calibration output is null");
    return false;
  }

  std::ifstream input (path);
  if( !input )
  {
    set_error (error_message, "Cannot open eye-to-hand calibration file");
    return false;
  }
  const std::string text {
    std::istreambuf_iterator<char> (input), std::istreambuf_iterator<char> ( ) };

  std::vector<double> rotation;
  std::vector<double> translation;
  if( !values_for_key (text, "RotMat_cam2robot", 9, &rotation,
                       error_message) ||
      !values_for_key (text, "TranMat_cam2robot", 3, &translation,
                       error_message) )
  {
    return false;
  }

  std::vector<double> coordinate_type;
  if( text.find ("CameraCoordinateType") != std::string::npos &&
      !values_for_key (text, "CameraCoordinateType", 1, &coordinate_type,
                       error_message) )
  {
    return false;
  }
  std::vector<double> point_cloud_unit;
  if( text.find ("PointCloudUnitToMm") != std::string::npos &&
      !values_for_key (text, "PointCloudUnitToMm", 1, &point_cloud_unit,
                       error_message) )
  {
    return false;
  }

  Eye_To_Hand_Calibration parsed;
  for( int row = 0; row < 3; ++row )
  {
    for( int column = 0; column < 3; ++column )
    {
      parsed.camera_to_robot[row][column] =
        rotation[static_cast<std::size_t> (row * 3 + column)];
    }
    parsed.camera_to_robot[row][3] = translation[static_cast<std::size_t> (row)];
  }
  parsed.camera_to_robot[3][3] = 1.0;
  if( !coordinate_type.empty ( ) )
  {
    const auto value = coordinate_type.front ( );
    if( value != 1.0 && value != 2.0 )
    {
      set_error (error_message,
                 "CameraCoordinateType must be 1 (Depth) or 2 (RGB)");
      return false;
    }
    parsed.camera_coordinate_type = static_cast<std::uint32_t> (value);
  }
  if( !point_cloud_unit.empty ( ) )
  {
    const auto value = point_cloud_unit.front ( );
    if( value <= 0.0 )
    {
      set_error (error_message, "PointCloudUnitToMm must be positive");
      return false;
    }
    parsed.point_cloud_unit_to_mm = value;
  }

  if( !is_rotation_matrix (parsed.camera_to_robot) )
  {
    set_error (error_message,
               "RotMat_cam2robot is not a valid rotation matrix");
    return false;
  }

  *calibration = parsed;
  return true;
}

} // namespace robot_model
