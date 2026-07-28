#ifndef includeguard_template_configuration_h_includeguard
#define includeguard_template_configuration_h_includeguard

#include "pose_transform.h"

#include <string>
#include <vector>

namespace robot_model
{

struct Template_Profile
{
  std::string id;
  std::string name;
  XyzabcPose reference_pose = {};
};

struct Template_Configuration
{
  std::vector<Template_Profile> templates;
};

bool Is_Valid_Template_Profile(const Template_Profile &profile);
void Normalize_Template_Configuration(
  Template_Configuration *configuration);

const Template_Profile *Find_Template_Profile(
  const Template_Configuration &configuration,
  const std::string &template_id);
Template_Profile *Find_Template_Profile(
  Template_Configuration *configuration,
  const std::string &template_id);

} // namespace robot_model

#endif
