#include "robot_calibration_builder.h"
#include "robot_kinematic_params.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace robot_model
{

namespace
{

bool contains_text (const std::string& text, std::string_view needle)
{
  return text.find (needle) != std::string::npos;
}

bool has_part_bounds (const std::vector<Robot_Visual_Part>& parts, size_t index)
{
  return index < parts.size ( ) && parts[index].has_raw_bounds;
}

double raw_min_x (const Robot_Visual_Part& part) { return part.raw_bounds[0]; }
double raw_max_x (const Robot_Visual_Part& part) { return part.raw_bounds[1]; }
double raw_center_x (const Robot_Visual_Part& part)
{
  return ( part.raw_bounds[0] + part.raw_bounds[1] ) * 0.5;
}
double raw_center_y (const Robot_Visual_Part& part)
{
  return ( part.raw_bounds[2] + part.raw_bounds[3] ) * 0.5;
}
double raw_center_z (const Robot_Visual_Part& part)
{
  return ( part.raw_bounds[4] + part.raw_bounds[5] ) * 0.5;
}

void set_part_translation (Robot_Assembly_Calibration& calibration,
                           size_t index,
                           double x, double y, double z)
{
  if( index >= calibration.parts.size ( ) )
  {
    return;
  }
  calibration.parts[index].translate[0] = x;
  calibration.parts[index].translate[1] = y;
  calibration.parts[index].translate[2] = z;
}

void set_part_rotation (Robot_Assembly_Calibration& calibration,
                        size_t index,
                        double rx, double ry, double rz)
{
  if( index >= calibration.parts.size ( ) )
  {
    return;
  }
  calibration.parts[index].rotate_xyz[0] = rx;
  calibration.parts[index].rotate_xyz[1] = ry;
  calibration.parts[index].rotate_xyz[2] = rz;
}

void apply_manual_part_calibration (Robot_Assembly_Calibration& calibration,
                                    const Robot_Kinematic_Params& params)
{
  const auto count = std::min (calibration.parts.size ( ),
                               params.manual_part_calibration.size ( ));
  for( size_t i = 0; i < count; ++i )
  {
    auto& dst = calibration.parts[i];
    const auto& manual = params.manual_part_calibration[i];
    dst.translate[0] += manual.translate[0];
    dst.translate[1] += manual.translate[1];
    dst.translate[2] += manual.translate[2];
    dst.rotate_xyz[0] += manual.rotate_xyz[0];
    dst.rotate_xyz[1] += manual.rotate_xyz[1];
    dst.rotate_xyz[2] += manual.rotate_xyz[2];
  }
}

void apply_wrist_calibration (Robot_Assembly_Calibration& calibration,
                              double elbow_z,
                              double wrist_x,
                              double wrist_backoff,
                              double flange_offset)
{
  set_part_translation (calibration, 4, -wrist_backoff, 0.0, elbow_z);
  set_part_translation (calibration, 5, wrist_x, 0.0, elbow_z);
  set_part_translation (calibration, 6, wrist_x + flange_offset, 0.0, elbow_z);
}

bool apply_kr210_r2700_2_bounds_calibration (
  Robot_Assembly_Calibration& calibration,
  const Robot_Kinematic_Params& params,
  const std::vector<Robot_Visual_Part>& parts)
{
  for( size_t i = 2; i <= 6; ++i )
  {
    if( !has_part_bounds (parts, i) )
    {
      return false;
    }
  }

  const double shoulder_z = link_length_at (params, 0, 0.0);
  const double elbow_z = shoulder_z + link_length_at (params, 1, 0.0);
  const double wrist_center_x = link_length_at (params, 3, 0.0);
  const double wrist_d45 = 177.0;
  const double wrist_overlap = 8.0;

  const double part3_axis_y = raw_center_y (parts[3]);
  const double part3_axis_z = elbow_z + raw_center_z (parts[3]);

  set_part_translation (calibration, 2,
                        0.0,
                        part3_axis_y - raw_center_y (parts[2]),
                        shoulder_z);
  set_part_translation (calibration, 3, 0.0, 0.0, elbow_z);

  const auto& part4 = parts[4];
  const double part4_center_target_x = wrist_center_x - wrist_d45;
  const double part4_tx = part4_center_target_x - raw_center_x (part4);
  set_part_translation (calibration, 4,
                        part4_tx,
                        part3_axis_y - raw_center_y (part4),
                        part3_axis_z - raw_center_z (part4));

  const auto& part5 = parts[5];
  const double part5_tx = wrist_center_x - raw_center_x (part5);
  set_part_translation (calibration, 5,
                        part5_tx,
                        part3_axis_y - raw_center_y (part5),
                        part3_axis_z - raw_center_z (part5));

  const auto& part6 = parts[6];
  const double part5_max_x = raw_max_x (part5) + part5_tx;
  const double part6_tx = part5_max_x - raw_min_x (part6) - wrist_overlap;
  set_part_translation (calibration, 6,
                        part6_tx,
                        part3_axis_y - raw_center_y (part6),
                        part3_axis_z - raw_center_z (part6));

  return true;
}

} // anonymous namespace

Robot_Assembly_Calibration Build_Assembly_Calibration (
  const Robot_Kinematic_Params& params, const std::string& model_name,
  const std::vector<Robot_Visual_Part>& parts)
{
  Robot_Assembly_Calibration calibration;
  const auto l1 = link_length_at (params, 0, 0.0);
  const auto l2 = link_length_at (params, 1, 0.0);
  const auto l4 = link_length_at (params, 3, 0.0);
  const auto l5 = link_length_at (params, 4, 0.0);

  if( model_name == "KR210_R2700_2" )
  {
    if( apply_kr210_r2700_2_bounds_calibration (calibration, params, parts) )
    {
      apply_manual_part_calibration (calibration, params);
      return calibration;
    }
  }

  if( contains_text (model_name, "KR210_R2700") )
  {
    // KR210_R2700_2 的 wrist 结构与旧 KR210 不同：
    // - 4.stl 建模原点在旋转中心后方约 1089mm（异常大，需专用补偿）
    // - 5.stl 建模原点在旋转中心后方约 46mm
    // - 6.stl 建模原点在旋转中心前方约 23mm
    // 基于实测网格边界反推的装配校准。
    const double shoulder_z = l1;
    const double elbow_z = shoulder_z + l2;
    const double wrist_center = l4;
    const double wrist_d45 = 177.0;                   // 轴4到轴5的间距（从 bounds_center 实测：1166 - 989 = 177）
    const double part4_rot_offset = 1089.0;           // 4.stl 旋转中心相对建模原点
    const double part5_rot_offset = 46.0;             // 5.stl
    const double part6_rot_offset = -23.0;            // 6.stl（负号表示在前方）

    set_part_translation (calibration, 2, 0.0, 0.0, shoulder_z);
    set_part_translation (calibration, 3, 0.0, 0.0, elbow_z);
    set_part_translation (calibration, 4,
                          wrist_center - wrist_d45 - part4_rot_offset,
                          0.0, elbow_z);
    set_part_translation (calibration, 5,
                          wrist_center - part5_rot_offset,
                          0.0, elbow_z);
    set_part_translation (calibration, 6,
                          wrist_center + l5 - part6_rot_offset,
                          0.0, elbow_z);
  }
  else if( contains_text (model_name, "KR210") )
  {
    const double shoulder_z = l1;
    const double elbow_z = shoulder_z + l2;
    const double wrist_backoff = 100.0;
    const double wrist_x = l4 - wrist_backoff;
    const double flange_offset = l5;

    set_part_translation (calibration, 2, 0.0, 0.0, shoulder_z);
    set_part_translation (calibration, 3, 0.0, 0.0, elbow_z);
    apply_wrist_calibration (calibration, elbow_z, wrist_x,
                             wrist_backoff, flange_offset);
  }
  else if( contains_text (model_name, "KR6") )
  {
    const double shoulder_z = l1;
    const double elbow_z = shoulder_z + l2;
    const double wrist_backoff = 35.0;
    const double wrist_x = l4 - wrist_backoff;
    const double flange_offset = l5;

    set_part_translation (calibration, 2, 0.0, 0.0, shoulder_z);
    set_part_translation (calibration, 3, 0.0, 0.0, elbow_z);
    apply_wrist_calibration (calibration, elbow_z, wrist_x,
                             wrist_backoff, flange_offset);
  }

  apply_manual_part_calibration (calibration, params);
  return calibration;
}

} // namespace robot_model
