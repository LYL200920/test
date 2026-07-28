#ifndef includeguard_progress_flow_transform_h_includeguard
#define includeguard_progress_flow_transform_h_includeguard

#include "robot_teach_point.h"

#include <string>
#include <vector>

namespace robot_model
{

struct Progress_Template_Reference
{
  std::string id;
  std::string name;
  XyzabcPose reference_pose = {};
};

bool Collect_Progress_Template_References(
  const std::vector<Robot_Teach_Point> &points,
  std::vector<Progress_Template_Reference> *references,
  std::string *error_message = nullptr);

bool Transform_Progress_Teach_Points(
  const std::vector<Robot_Teach_Point> &points,
  const std::vector<XyzabcPose> &recognized_poses,
  std::vector<XyzabcPose> *transformed_poses,
  std::vector<Matrix4> *template_transforms = nullptr,
  std::string *error_message = nullptr);

bool Transform_Progress_Teach_Points_In_Coordinate(
  const std::vector<Robot_Teach_Point> &points,
  const std::vector<XyzabcPose> &recognized_poses,
  const XyzabcPose &flange_from_coordinate_pose,
  std::vector<XyzabcPose> *transformed_poses,
  std::vector<XyzabcPose> *source_coordinate_poses = nullptr,
  std::vector<Matrix4> *template_transforms = nullptr,
  std::string *error_message = nullptr);

} // namespace robot_model

#endif
