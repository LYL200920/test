#include "mosaic_configuration.h"

#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace robot_model
{
namespace
{

bool Is_Valid_Axis(Coordinate_Axis axis)
{
  return axis == Coordinate_Axis::X ||
         axis == Coordinate_Axis::Y ||
         axis == Coordinate_Axis::Z;
}

} // namespace

bool Validate_Mosaic_Configuration(
  const Mosaic_Configuration &configuration,
  const std::vector<Robot_Teach_Point> &points,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  // An empty configuration keeps the legacy selection behavior.
  if (configuration.groups.empty())
  {
    return true;
  }

  std::unordered_map<std::size_t, const Robot_Teach_Point *> points_by_id;
  for (const auto &point : points)
  {
    const auto inserted = points_by_id.emplace(point.id, &point);
    if (!inserted.second)
    {
      inserted.first->second = nullptr;
    }
  }

  std::unordered_set<std::size_t> group_ids;
  for (const auto &group : configuration.groups)
  {
    const std::string context =
      "Mosaic group " + std::to_string(group.id) + " (" + group.name + "): ";
    const auto fail = [&](const std::string &message)
    {
      if (error_message)
      {
        *error_message = context + message;
      }
      return false;
    };
    if (group.id == 0 || !group_ids.insert(group.id).second)
    {
      return fail("group ID must be nonzero and unique");
    }
    if (group.name.empty())
    {
      return fail("name is empty");
    }
    if (group.point_ids.empty())
    {
      return fail("point selection is empty");
    }
    if (group.reference != Mosaic_Reference::First_Camera &&
        group.reference != Mosaic_Reference::Robot_Base)
    {
      return fail("reference is invalid");
    }
    if (!Is_Valid_Axis(group.horizontal_axis) ||
        !Is_Valid_Axis(group.vertical_axis) ||
        group.horizontal_axis == group.vertical_axis)
    {
      return fail("horizontal and vertical axes must be valid and different");
    }

    std::unordered_set<std::size_t> point_ids;
    for (const auto id : group.point_ids)
    {
      const std::string point_context = "point " + std::to_string(id) + " ";
      if (id == 0 || !point_ids.insert(id).second)
      {
        return fail(point_context + "ID must be nonzero and unique in the group");
      }
      const auto found = points_by_id.find(id);
      if (found == points_by_id.end())
      {
        return fail(point_context + "does not exist");
      }
      if (!found->second)
      {
        return fail(point_context + "ID is ambiguous");
      }
      const auto &point = *found->second;
      if (point.type != Robot_Teach_Point_Type::Motion)
      {
        return fail(point_context + "is not a Motion point");
      }
      if (!point.has_world_pose)
      {
        return fail(point_context + "has no world pose");
      }
      for (const double value : point.world_pose)
      {
        if (!std::isfinite(value))
        {
          return fail(point_context + "has a non-finite world pose");
        }
      }
    }
  }
  return true;
}

} // namespace robot_model
