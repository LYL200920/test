#include "robot_model_config_loader.h"

#include "pugixml.hpp"

#include <Windows.h>

#include <sstream>

namespace robot_model
{
namespace
{

double item_value_as_double (pugi::xml_node root,
                             const char* key,
                             double fallback)
{
  const auto xpath = std::string (".//Item[@keyname='") + key + "']";
  const auto node = root.select_node (xpath.c_str ( )).node ( );
  if( !node )
  {
    return fallback;
  }
  return node.attribute ("value").as_double (fallback);
}

bool item_exists (pugi::xml_node root, const char* key)
{
  const auto xpath = std::string (".//Item[@keyname='") + key + "']";
  return root.select_node (xpath.c_str ( )).node ( );
}

int item_value_as_int (pugi::xml_node root, const char* key, int fallback)
{
  const auto xpath = std::string (".//Item[@keyname='") + key + "']";
  const auto node = root.select_node (xpath.c_str ( )).node ( );
  if( !node )
  {
    return fallback;
  }
  return node.attribute ("value").as_int (fallback);
}

std::string item_value_as_string (pugi::xml_node root,
                                  const char* key,
                                  const char* fallback)
{
  const auto xpath = std::string (".//Item[@keyname='") + key + "']";
  const auto node = root.select_node (xpath.c_str ( )).node ( );
  if( !node )
  {
    return fallback;
  }
  return node.attribute ("value").as_string (fallback);
}

} // namespace

Robot_Kinematic_Params Load_Robot_Kinematic_Params (
  const std::filesystem::path& xml_path,
  const std::string& default_name)
{
  Robot_Kinematic_Params params;
  params.model_name = default_name;

  if( xml_path.empty ( ) || !std::filesystem::exists (xml_path) )
  {
    return params;
  }

  pugi::xml_document doc;
  if( !doc.load_file (xml_path.wstring ( ).c_str ( )) )
  {
    return params;
  }

  const auto root = doc.child ("RobotTemplate");
  params.model_name = item_value_as_string (
    root, "ManipulatorModel", default_name.c_str ( ));
  params.model_number = item_value_as_int (root, "ModelNumber", 0);
  params.model_scale_unit = item_value_as_int (root, "ModelScaleUnit", 1);
  params.base_cs_x = item_value_as_int (root, "BaseCsX", 4);
  params.base_cs_z = item_value_as_int (root, "BaseCsZ", 1);
  params.tool_cs_x = item_value_as_int (root, "ToolCsX", 2);
  params.tool_cs_z = item_value_as_int (root, "ToolCsZ", 4);

  params.link_lengths.clear ( );
  for( int i = 1; i <= 6; ++i )
  {
    const auto key = std::string ("LinkLength_") + std::to_string (i);
    params.link_lengths.push_back (
      item_value_as_double (root, key.c_str ( ), 0.0));
  }

  params.joint_offsets.clear ( );
  params.joint_directions.clear ( );
  params.joint_mins.clear ( );
  params.joint_maxs.clear ( );
  for( int i = 1; i <= 6; ++i )
  {
    const auto offset_key = std::string ("JointOffset_") + std::to_string (i);
    const auto direction_key =
      std::string ("JointDirection_") + std::to_string (i);
    const auto min_key = std::string ("JointMin_") + std::to_string (i);
    const auto max_key = std::string ("JointMax_") + std::to_string (i);
    const auto pivot_prefix = std::string ("JointPivot_") + std::to_string (i);
    const auto axis_prefix = std::string ("JointAxis_") + std::to_string (i);

    params.joint_offsets.push_back (
      item_value_as_double (root, offset_key.c_str ( ), 0.0));
    params.joint_directions.push_back (
      item_value_as_int (root, direction_key.c_str ( ), 1));
    params.joint_mins.push_back (
      item_value_as_double (root, min_key.c_str ( ), -180.0));
    params.joint_maxs.push_back (
      item_value_as_double (root, max_key.c_str ( ), 180.0));

    auto& joint_frame = params.joint_frames[static_cast<size_t> (i - 1)];
    const auto pivot_x_key = pivot_prefix + "_X";
    const auto pivot_y_key = pivot_prefix + "_Y";
    const auto pivot_z_key = pivot_prefix + "_Z";
    const auto axis_x_key = axis_prefix + "_X";
    const auto axis_y_key = axis_prefix + "_Y";
    const auto axis_z_key = axis_prefix + "_Z";

    joint_frame.has_pivot =
      item_exists (root, pivot_x_key.c_str ( )) &&
      item_exists (root, pivot_y_key.c_str ( )) &&
      item_exists (root, pivot_z_key.c_str ( ));
    if( joint_frame.has_pivot )
    {
      joint_frame.pivot[0] = item_value_as_double (
        root, pivot_x_key.c_str ( ), 0.0);
      joint_frame.pivot[1] = item_value_as_double (
        root, pivot_y_key.c_str ( ), 0.0);
      joint_frame.pivot[2] = item_value_as_double (
        root, pivot_z_key.c_str ( ), 0.0);
    }

    joint_frame.has_axis =
      item_exists (root, axis_x_key.c_str ( )) &&
      item_exists (root, axis_y_key.c_str ( )) &&
      item_exists (root, axis_z_key.c_str ( ));
    if( joint_frame.has_axis )
    {
      joint_frame.axis[0] = item_value_as_double (
        root, axis_x_key.c_str ( ), 0.0);
      joint_frame.axis[1] = item_value_as_double (
        root, axis_y_key.c_str ( ), 0.0);
      joint_frame.axis[2] = item_value_as_double (
        root, axis_z_key.c_str ( ), 1.0);
    }
  }

  params.has_neutral_flange_pose =
    item_exists (root, "FlangePose_X") &&
    item_exists (root, "FlangePose_Y") &&
    item_exists (root, "FlangePose_Z");
  if( params.has_neutral_flange_pose )
  {
    params.neutral_flange_pose[0] =
      item_value_as_double (root, "FlangePose_X", 0.0);
    params.neutral_flange_pose[1] =
      item_value_as_double (root, "FlangePose_Y", 0.0);
    params.neutral_flange_pose[2] =
      item_value_as_double (root, "FlangePose_Z", 0.0);
    params.neutral_flange_pose[3] =
      item_value_as_double (root, "FlangePose_A", 0.0);
    params.neutral_flange_pose[4] =
      item_value_as_double (root, "FlangePose_B", 0.0);
    params.neutral_flange_pose[5] =
      item_value_as_double (root, "FlangePose_C", 0.0);
  }

  for( int i = 0; i < 7; ++i )
  {
    const auto prefix = std::string ("Part_") + std::to_string (i);
    params.manual_part_calibration[i].translate[0] =
      item_value_as_double (root, ( prefix + "_OffsetX" ).c_str ( ), 0.0);
    params.manual_part_calibration[i].translate[1] =
      item_value_as_double (root, ( prefix + "_OffsetY" ).c_str ( ), 0.0);
    params.manual_part_calibration[i].translate[2] =
      item_value_as_double (root, ( prefix + "_OffsetZ" ).c_str ( ), 0.0);
    params.manual_part_calibration[i].rotate_xyz[0] =
      item_value_as_double (root, ( prefix + "_RotX" ).c_str ( ), 0.0);
    params.manual_part_calibration[i].rotate_xyz[1] =
      item_value_as_double (root, ( prefix + "_RotY" ).c_str ( ), 0.0);
    params.manual_part_calibration[i].rotate_xyz[2] =
      item_value_as_double (root, ( prefix + "_RotZ" ).c_str ( ), 0.0);
  }

  std::ostringstream dbg;
  dbg << "[RobotModel] Parsed link_lengths: ";
  for( size_t i = 0; i < params.link_lengths.size ( ); ++i )
  {
    dbg << "L" << ( i + 1 ) << "=" << params.link_lengths[i] << " ";
  }
  OutputDebugStringA (dbg.str ( ).c_str ( ));
  OutputDebugStringA ("\n");

  return params;
}

} // namespace robot_model
