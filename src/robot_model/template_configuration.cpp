#include "template_configuration.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace robot_model
{

bool Is_Valid_Template_Profile(const Template_Profile &profile)
{
  if (profile.id.empty() || profile.name.empty())
  {
    return false;
  }
  return std::all_of(
    profile.reference_pose.begin(),
    profile.reference_pose.end(),
    [](double value) { return std::isfinite(value); });
}

void Normalize_Template_Configuration(
  Template_Configuration *configuration)
{
  if (!configuration)
  {
    return;
  }

  std::vector<Template_Profile> normalized;
  std::unordered_set<std::string> ids;
  for (Template_Profile &profile : configuration->templates)
  {
    if (!Is_Valid_Template_Profile(profile) ||
        ids.count(profile.id) != 0)
    {
      continue;
    }
    ids.insert(profile.id);
    normalized.push_back(std::move(profile));
  }
  configuration->templates = std::move(normalized);
}

const Template_Profile *Find_Template_Profile(
  const Template_Configuration &configuration,
  const std::string &template_id)
{
  const auto found = std::find_if(
    configuration.templates.begin(),
    configuration.templates.end(),
    [&template_id](const Template_Profile &profile)
    {
      return profile.id == template_id;
    });
  return found == configuration.templates.end() ? nullptr : &*found;
}

Template_Profile *Find_Template_Profile(
  Template_Configuration *configuration,
  const std::string &template_id)
{
  if (!configuration)
  {
    return nullptr;
  }
  const auto found = std::find_if(
    configuration->templates.begin(),
    configuration->templates.end(),
    [&template_id](const Template_Profile &profile)
    {
      return profile.id == template_id;
    });
  return found == configuration->templates.end() ? nullptr : &*found;
}

} // namespace robot_model
