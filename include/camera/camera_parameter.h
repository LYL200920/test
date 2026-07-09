#ifndef includeguard_camera_parameter_h_includeguard
#define includeguard_camera_parameter_h_includeguard

#include "Mv3dRgbdDefine.h"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

enum class Camera_Parameter_Type
{
  Bool,
  Int,
  Float,
  Enum,
  String
};

enum class Camera_Parameter_Group
{
  Basic,
  Image_Output,
  Trigger
};

using Camera_Parameter_Value = std::variant<
  bool,
  std::int64_t,
  float,
  std::uint32_t,
  std::string>;

struct Camera_Parameter_Choice
{
  std::int64_t value = 0;
  std::string label;
};

struct Camera_Parameter
{
  std::string key;
  std::string display_name;
  Camera_Parameter_Group group = Camera_Parameter_Group::Basic;
  Camera_Parameter_Type type = Camera_Parameter_Type::Int;
  Camera_Parameter_Value current_value = std::int64_t{0};
  std::optional<std::int64_t> int_minimum;
  std::optional<std::int64_t> int_maximum;
  std::optional<std::int64_t> int_increment;
  std::optional<float> float_minimum;
  std::optional<float> float_maximum;
  std::optional<std::uint32_t> string_max_length;
  std::vector<std::uint32_t> enum_values;
  std::vector<Camera_Parameter_Choice> choices;
  bool affects_stream_config = false;
};

struct Camera_Parameter_Update
{
  std::string key;
  Camera_Parameter_Type type = Camera_Parameter_Type::Int;
  Camera_Parameter_Value value = std::int64_t{0};
};

bool Decode_Camera_Parameter (
  const MV3D_RGBD_PARAM& source,
  Camera_Parameter* parameter,
  std::string* error = nullptr);

bool Encode_Camera_Parameter_Update (
  const Camera_Parameter_Update& update,
  MV3D_RGBD_PARAM* destination,
  std::string* error = nullptr);

bool Validate_Camera_Parameter_Update (
  const Camera_Parameter& current,
  const Camera_Parameter_Update& update,
  std::string* error = nullptr);

#endif
