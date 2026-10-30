#include "mosaic_configuration.h"
#include "robot_progress_io.h"
#include "robot_teach_point_store.h"
#include "pugixml.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using namespace robot_model;

void Require(bool condition, const std::string &message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

struct Temporary_Directory
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("mosaic_configuration_tests_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));

  Temporary_Directory()
  {
    Require(std::filesystem::create_directory(path),
      "Unable to create test directory");
  }

  ~Temporary_Directory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
};

Robot_Progress_File Make_Progress()
{
  Robot_Progress_File progress;
  progress.robot_model_id = "test-robot";
  for (const auto id : {3u, 8u, 12u})
  {
    Robot_Teach_Point point;
    point.id = id;
    point.robot_model_id = progress.robot_model_id;
    point.has_world_pose = true;
    point.world_pose = {double(id), 2.5, -3.0, 4.0, -5.0, 6.0};
    progress.points.push_back(point);
  }
  Mosaic_Group group;
  group.id = 17;
  group.name = "Top & side";
  group.point_ids = {8, 3};
  progress.mosaic_configuration.groups.push_back(group);
  return progress;
}

void Require_Equal_Groups(
  const Mosaic_Configuration &actual,
  const Mosaic_Configuration &expected)
{
  Require(actual.groups.size() == expected.groups.size(),
    "Group count changed");
  for (std::size_t index = 0; index < expected.groups.size(); ++index)
  {
    const auto &a = actual.groups[index];
    const auto &e = expected.groups[index];
    Require(a.id == e.id && a.name == e.name && a.point_ids == e.point_ids &&
      a.reference == e.reference && a.horizontal_axis == e.horizontal_axis &&
      a.vertical_axis == e.vertical_axis &&
      a.reverse_horizontal == e.reverse_horizontal &&
      a.reverse_vertical == e.reverse_vertical,
      "Group identity, order, selection, reference, axes or directions changed");
  }
}

void Test_Validation()
{
  std::string error = "stale error";
  Require(Validate_Mosaic_Configuration({}, {}, &error) && error.empty(),
    "Empty configuration must be valid and clear errors");
  auto progress = Make_Progress();
  Require(Validate_Mosaic_Configuration(
    progress.mosaic_configuration, progress.points), "Valid group rejected");

  const std::vector<std::function<void(Robot_Progress_File &)>> invalid = {
    [](auto &p) { p.mosaic_configuration.groups[0].id = 0; },
    [](auto &p) { p.mosaic_configuration.groups.push_back(
      p.mosaic_configuration.groups[0]); },
    [](auto &p) { p.mosaic_configuration.groups[0].name.clear(); },
    [](auto &p) { p.mosaic_configuration.groups[0].point_ids.clear(); },
    [](auto &p) { p.mosaic_configuration.groups[0].point_ids = {0}; },
    [](auto &p) { p.mosaic_configuration.groups[0].point_ids = {3, 3}; },
    [](auto &p) { p.mosaic_configuration.groups[0].point_ids = {999}; },
    [](auto &p) { p.points[0].type = Robot_Teach_Point_Type::Transition; },
    [](auto &p) { p.points[0].type = Robot_Teach_Point_Type::Wait; },
    [](auto &p) { p.points[0].has_world_pose = false; },
    [](auto &p) { p.points.push_back(p.points[0]); },
    [](auto &p) { p.mosaic_configuration.groups[0].reference =
      static_cast<Mosaic_Reference>(-1); },
    [](auto &p) { p.mosaic_configuration.groups[0].horizontal_axis =
      static_cast<Coordinate_Axis>(-1); },
    [](auto &p) { p.mosaic_configuration.groups[0].vertical_axis =
      static_cast<Coordinate_Axis>(3); },
    [](auto &p) { p.mosaic_configuration.groups[0].vertical_axis =
      p.mosaic_configuration.groups[0].horizontal_axis; }
  };
  for (const auto &mutate : invalid)
  {
    auto candidate = progress;
    mutate(candidate);
    Require(!Validate_Mosaic_Configuration(
      candidate.mosaic_configuration, candidate.points, &error),
      "Invalid configuration accepted");
    Require(error.find("group") != std::string::npos,
      "Validation error must identify a group");
  }
  for (std::size_t index = 0; index < 6; ++index)
  {
    for (const auto value : {std::numeric_limits<double>::infinity(),
         -std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::quiet_NaN()})
    {
      auto candidate = progress;
      candidate.points[0].world_pose[index] = value;
      Require(!Validate_Mosaic_Configuration(
        candidate.mosaic_configuration, candidate.points, &error),
        "Non-finite world pose accepted");
      Require(error.find("group 17") != std::string::npos &&
        error.find("point 3") != std::string::npos,
        "Pose error must identify group and point");
    }
  }
  for (const auto id : {0u, 999u, 3u})
  {
    auto candidate = progress;
    candidate.mosaic_configuration.groups[0].point_ids = {id, id};
    Require(!Validate_Mosaic_Configuration(
      candidate.mosaic_configuration, candidate.points, &error) &&
      error.find("point " + std::to_string(id)) != std::string::npos,
      "Selection error must identify the point");
  }

  // A point can participate in different groups; unused non-Motion points
  // do not invalidate an explicit selection.
  auto second = progress.mosaic_configuration.groups[0];
  second.id = 42;
  progress.mosaic_configuration.groups.push_back(second);
  progress.points[2].type = Robot_Teach_Point_Type::Wait;
  progress.points[2].has_world_pose = false;
  Require(Validate_Mosaic_Configuration(
    progress.mosaic_configuration, progress.points, &error),
    "Shared selections or unused points incorrectly rejected");
}

void Test_Round_Trip(const std::filesystem::path &path)
{
  auto progress = Make_Progress();
  progress.mosaic_configuration.groups.clear();
  for (const auto horizontal : {Coordinate_Axis::X, Coordinate_Axis::Y,
       Coordinate_Axis::Z})
  {
    for (const auto vertical : {Coordinate_Axis::X, Coordinate_Axis::Y,
         Coordinate_Axis::Z})
    {
      if (horizontal == vertical)
      {
        continue;
      }
      for (int flags = 0; flags < 4; ++flags)
      {
        Mosaic_Group group;
        group.id = 100 + progress.mosaic_configuration.groups.size() * 7;
        group.name = u8"拼图 <top> & \"side\"";
        group.point_ids = {12, 3, 8};
        group.reference = flags % 2 == 0
          ? Mosaic_Reference::First_Camera : Mosaic_Reference::Robot_Base;
        group.horizontal_axis = horizontal;
        group.vertical_axis = vertical;
        group.reverse_horizontal = (flags & 1) != 0;
        group.reverse_vertical = (flags & 2) != 0;
        progress.mosaic_configuration.groups.push_back(group);
      }
    }
  }
  // Stable IDs must not be renumbered and must support the entire size_t range.
  progress.mosaic_configuration.groups.back().id =
    std::numeric_limits<std::size_t>::max();
  std::string error;
  Require(Save_Robot_Progress(path, progress, &error), "Save failed: " + error);
  pugi::xml_document document;
  Require(document.load_file(path.wstring().c_str()), "Saved XML is invalid");
  Require(document.child("RobotProgress").attribute("version").as_int() == 5,
    "Progress must be saved as version 5");
  Robot_Progress_File loaded;
  Require(Load_Robot_Progress(path, &loaded, &error), "Load failed: " + error);
  Require(loaded.robot_model_id == progress.robot_model_id &&
    loaded.points.size() == progress.points.size(), "Progress data changed");
  Require_Equal_Groups(loaded.mosaic_configuration, progress.mosaic_configuration);

  progress.mosaic_configuration.groups.clear();
  Require(Save_Robot_Progress(path, progress, &error), "Empty groups save failed");
  Require(Load_Robot_Progress(path, &loaded, &error) &&
    loaded.mosaic_configuration.groups.empty(), "Empty groups did not round-trip");
}

void Test_Legacy(const std::filesystem::path &path)
{
  for (int version = 1; version <= 4; ++version)
  {
    {
      std::ofstream file(path);
      file << "<RobotProgress version=\"" << version
           << "\" robotModel=\"legacy\">"
           << "<Point id=\"1\" a1=\"0\" a2=\"0\" a3=\"0\" "
           << "a4=\"0\" a5=\"0\" a6=\"0\" x=\"0\" y=\"0\" z=\"0\" "
           << "a=\"0\" b=\"0\" c=\"0\" hasWorldPose=\"true\"/>"
           << "</RobotProgress>";
    }
    auto loaded = Make_Progress();
    std::string error;
    Require(Load_Robot_Progress(path, &loaded, &error),
      "Legacy Progress rejected: " + error);
    Require(loaded.mosaic_configuration.groups.empty() &&
      loaded.robot_model_id == "legacy" && loaded.points.size() == 1 &&
      loaded.points[0].type == Robot_Teach_Point_Type::Motion,
      "Legacy Progress must clear any previous configuration");
  }
}

void Require_Rejected_Unchanged(const std::filesystem::path &path)
{
  auto output = Make_Progress();
  output.robot_model_id = "unchanged-output";
  const auto before = output;
  std::string error;
  Require(!Load_Robot_Progress(path, &output, &error) && !error.empty(),
    "Malformed MosaicGroups accepted");
  Require(output.robot_model_id == before.robot_model_id &&
    output.points.size() == before.points.size() &&
    output.points[0].id == before.points[0].id &&
    output.points[0].world_pose == before.points[0].world_pose,
    "Failed load modified output Progress");
  Require_Equal_Groups(output.mosaic_configuration, before.mosaic_configuration);
}

void Test_Invalid_Xml(const std::filesystem::path &path)
{
  const auto original = Make_Progress();
  const auto check = [&](const std::function<void(pugi::xml_node)> &mutate)
  {
    std::string error;
    Require(Save_Robot_Progress(path, original, &error), "Fixture save failed");
    pugi::xml_document document;
    Require(document.load_file(path.wstring().c_str()), "Fixture parse failed");
    mutate(document.child("RobotProgress"));
    Require(document.save_file(path.wstring().c_str()), "Fixture write failed");
    Require_Rejected_Unchanged(path);
  };
  for (const auto text : {"", "0", "-1", "+1", "1tail", "1.5", " 17", "17 ",
       "184467440737095516160000"})
  {
    check([&](auto root) {
      root.child("MosaicGroups").child("Group").attribute("id") = text;
    });
    check([&](auto root) {
      root.child("MosaicGroups").child("Group").child("PointRef")
        .attribute("id") = text;
    });
  }
  for (const auto field : {"reverseHorizontal", "reverseVertical"})
  {
    for (const auto text : {"", "yes", "True", "2", "-1", "truejunk", "false "})
    {
      check([&](auto root) {
        root.child("MosaicGroups").child("Group").attribute(field) = text;
      });
    }
  }
  for (const auto field : {"horizontalAxis", "verticalAxis"})
  {
    for (const auto text : {"", "x", "3", "XY", "Zjunk"})
    {
      check([&](auto root) {
        root.child("MosaicGroups").child("Group").attribute(field) = text;
      });
    }
  }
  for (const auto field : {"id", "name", "reference", "horizontalAxis",
       "verticalAxis", "reverseHorizontal", "reverseVertical"})
  {
    check([&](auto root) {
      root.child("MosaicGroups").child("Group").remove_attribute(field);
    });
  }
  const std::vector<std::function<void(pugi::xml_node)>> invalid = {
    [](auto root) { root.append_child("MosaicGroups"); },
    [](auto root) { root.child("MosaicGroups").append_child("Unknown"); },
    [](auto root) { root.child("MosaicGroups").append_attribute("unknown") = 1; },
    [](auto root) {
      auto groups = root.child("MosaicGroups");
      groups.append_copy(groups.child("Group"));
    },
    [](auto root) { root.child("MosaicGroups").child("Group")
      .attribute("name") = ""; },
    [](auto root) { root.child("MosaicGroups").child("Group")
      .attribute("reference") = "unknown"; },
    [](auto root) { root.child("MosaicGroups").child("Group")
      .attribute("verticalAxis") = "X"; },
    [](auto root) { root.child("MosaicGroups").child("Group")
      .append_attribute("unknown") = 1; },
    [](auto root) { root.child("MosaicGroups").child("Group")
      .append_attribute("id") = 18; },
    [](auto root) { root.child("MosaicGroups").child("Group").remove_children(); },
    [](auto root) { root.child("MosaicGroups").child("Group")
      .append_child("Point").append_attribute("id") = 3; },
    [](auto root) { root.child("MosaicGroups").child("Group")
      .append_child(pugi::node_pcdata).set_value("unexpected text"); },
    [](auto root) {
      auto group = root.child("MosaicGroups").child("Group");
      group.append_copy(group.child("PointRef"));
    },
    [](auto root) { root.child("MosaicGroups").child("Group").child("PointRef")
      .attribute("id") = 999; },
    [](auto root) { root.child("MosaicGroups").child("Group").child("PointRef")
      .remove_attribute("id"); },
    [](auto root) { root.child("MosaicGroups").child("Group").child("PointRef")
      .append_child("Unknown"); },
    [](auto root) { root.child("MosaicGroups").child("Group").child("PointRef")
      .append_attribute("unknown") = 1; },
    [](auto root) { root.child("Point").attribute("type") = "wait"; },
    [](auto root) { root.child("Point").attribute("type") = "transition"; },
    [](auto root) { root.child("Point").attribute("hasWorldPose") = false; },
    [](auto root) { root.child("Point").attribute("x") =
      std::numeric_limits<double>::infinity(); }
  };
  for (const auto &mutate : invalid)
  {
    check(mutate);
  }
  {
    std::ofstream file(path);
    file << "<RobotProgress><MosaicGroups><Group></RobotProgress>";
  }
  Require_Rejected_Unchanged(path);

  // XML boolean numeric spellings are accepted, but not permissive prefixes.
  std::string error;
  Require(Save_Robot_Progress(path, original, &error), "Fixture save failed");
  pugi::xml_document document;
  Require(document.load_file(path.wstring().c_str()), "Fixture parse failed");
  const auto group = document.child("RobotProgress").child("MosaicGroups")
    .child("Group");
  group.attribute("reverseHorizontal") = "1";
  group.attribute("reverseVertical") = "0";
  Require(document.save_file(path.wstring().c_str()), "Fixture write failed");
  Robot_Progress_File loaded;
  Require(Load_Robot_Progress(path, &loaded, &error) &&
    loaded.mosaic_configuration.groups[0].reverse_horizontal &&
    !loaded.mosaic_configuration.groups[0].reverse_vertical,
    "Numeric XML booleans should be accepted");
}

void Test_Invalid_Save(const std::filesystem::path &path)
{
  auto progress = Make_Progress();
  std::string error;
  Require(Save_Robot_Progress(path, progress, &error), "Fixture save failed");
  const auto original = progress;
  progress.mosaic_configuration.groups[0].point_ids = {999};
  Require(!Save_Robot_Progress(path, progress, &error) &&
    error.find("group 17") != std::string::npos &&
    error.find("point 999") != std::string::npos,
    "Invalid save must identify group and point");
  Robot_Progress_File loaded;
  Require(Load_Robot_Progress(path, &loaded, &error), "Invalid save damaged file");
  Require_Equal_Groups(loaded.mosaic_configuration, original.mosaic_configuration);
}

} // namespace

void Test_Stable_Selection_After_Edit()
{
  auto progress = Make_Progress();
  Robot_Teach_Point_Store store;
  store.Replace_Points(progress.robot_model_id, progress.points);
  Mosaic_Configuration configuration;
  Mosaic_Group group;
  group.name = "Selected point";
  group.point_ids = {8};
  configuration.groups.push_back(group);
  store.Insert_Point(progress.robot_model_id, 0, {}, {});
  std::string error;
  Require(Validate_Mosaic_Configuration(configuration,
    store.Points(progress.robot_model_id), &error), error);
  Require(store.Points(progress.robot_model_id)[2].id == 8,
    "Inserting a point retargeted a mosaic reference");
  store.Delete_Point(progress.robot_model_id, 2);
  store.Add_Point(progress.robot_model_id, {}, {});
  Require(!Validate_Mosaic_Configuration(configuration,
    store.Points(progress.robot_model_id), &error),
    "Deleted mosaic point silently bound to a replacement point");
}

int main()
{
  try
  {
    Temporary_Directory temporary;
    const auto path = temporary.path / "progress.xml";
    Test_Validation();
    Test_Stable_Selection_After_Edit();
    Test_Round_Trip(path);
    Test_Legacy(path);
    Test_Invalid_Xml(path);
    Test_Invalid_Save(path);
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
