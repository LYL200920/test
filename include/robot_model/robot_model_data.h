#ifndef includeguard_robot_model_data_h_includeguard
#define includeguard_robot_model_data_h_includeguard

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace robot_model
{

struct Robot_Model_Info
{
  std::string display_name;
  std::filesystem::path model_dir;
  std::filesystem::path xml_path;
  std::vector<std::filesystem::path> stl_files;

  bool Has_Mesh ( ) const { return !stl_files.empty ( ); }
};

struct Robot_Part_Calibration
{
  double translate[3] = { 0.0, 0.0, 0.0 };
  double rotate_xyz[3] = { 0.0, 0.0, 0.0 };
};

struct Robot_Joint_State
{
  std::array<double, 6> input_angles_deg = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  std::array<double, 6> effective_angles_deg = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  std::array<double, 6> delta_angles_deg = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
};

struct Robot_Joint_Frame_Config
{
  std::array<double, 3> pivot = { 0.0, 0.0, 0.0 };
  std::array<double, 3> axis = { 0.0, 0.0, 1.0 };
  bool has_pivot = false;
  bool has_axis = false;
};

struct Robot_Kinematic_Params
{
  std::string model_name;
  int model_number = 0;
  int model_scale_unit = 1;
  int base_cs_x = 4;
  int base_cs_z = 1;
  int tool_cs_x = 2;
  int tool_cs_z = 4;
  std::vector<double> link_lengths;
  std::vector<double> joint_offsets;
  std::vector<int> joint_directions;
  std::vector<double> joint_mins;
  std::vector<double> joint_maxs;
  std::array<Robot_Joint_Frame_Config, 6> joint_frames;
  std::array<double, 6> neutral_flange_pose = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  };
  bool has_neutral_flange_pose = false;
  std::array<Robot_Part_Calibration, 7> manual_part_calibration;
  double self_collision_clearance_mm = 3.0;
  std::vector<std::array<std::size_t, 2>> self_collision_pairs;
  std::vector<std::size_t> ground_collision_parts;

  bool Has_Link_Lengths ( ) const { return link_lengths.size ( ) >= 6; }
};

struct Robot_Assembly_Calibration
{
  std::array<Robot_Part_Calibration, 7> parts;
};

} // namespace robot_model

#endif
