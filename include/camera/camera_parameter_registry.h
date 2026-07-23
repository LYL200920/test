#ifndef includeguard_camera_parameter_registry_h_includeguard
#define includeguard_camera_parameter_registry_h_includeguard

#include "camera_parameter.h"

#include <string>
#include <vector>

struct Camera_Parameter_Definition
{
  std::string key;
  std::string display_name;
  Camera_Parameter_Group group = Camera_Parameter_Group::Basic;
  std::vector<Camera_Parameter_Choice> choices;
  bool affects_stream_config = false;
};

const std::vector<Camera_Parameter_Definition> &
First_Stage_Camera_Parameter_Definitions();

const Camera_Parameter_Definition *Find_Camera_Parameter_Definition(const std::string &key);

std::string Camera_Parameter_Enum_Label(
    const std::string &key,
    std::uint32_t value);

#endif
