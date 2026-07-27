#ifndef includeguard_robot_teach_point_store_h_includeguard
#define includeguard_robot_teach_point_store_h_includeguard

#include "robot_teach_point.h"

#include <map>
#include <vector>

namespace robot_model
{

  class Robot_Teach_Point_Store
  {
  public:
    const Robot_Teach_Point &Add_Point(
        const std::string &robot_model_id,
        const std::array<double, 6> &joint_angles_deg,
        const XyzabcPose &world_pose,
        const std::string &point_cloud_path = {},
        const std::string &coordinate_frame_id = {},
        const std::string &coordinate_frame_name = {},
        const XyzabcPose &flange_from_coordinate_pose = {});
    const Robot_Teach_Point &Insert_Point(
        const std::string &robot_model_id,
        std::size_t index,
        const std::array<double, 6> &joint_angles_deg,
        const XyzabcPose &world_pose,
        const std::string &point_cloud_path = {},
        const std::string &coordinate_frame_id = {},
        const std::string &coordinate_frame_name = {},
        const XyzabcPose &flange_from_coordinate_pose = {});
    bool Update_Point(
        const std::string &robot_model_id,
        std::size_t index,
        const std::array<double, 6> &joint_angles_deg,
        const XyzabcPose &world_pose,
        const std::string &point_cloud_path = {},
        const std::string &coordinate_frame_id = {},
        const std::string &coordinate_frame_name = {},
        const XyzabcPose &flange_from_coordinate_pose = {});

    void Replace_Joint_Points(const std::string &robot_model_id, const std::vector<std::array<double, 6>> &joint_points);
    void Replace_Points(
        const std::string &robot_model_id,
        const std::vector<Robot_Teach_Point> &points);
    bool Delete_Point(const std::string &robot_model_id, std::size_t index);
    std::size_t Delete_Points(
        const std::string &robot_model_id,
        const std::vector<std::size_t> &indices);
    void Clear_Points(const std::string &robot_model_id);

    const std::vector<Robot_Teach_Point> &Points(const std::string &robot_model_id) const;
    std::size_t Point_Count(const std::string &robot_model_id) const;

  private:
    struct Model_Points
    {
      std::vector<Robot_Teach_Point> points;
    };

    static void Renumber_Points(Model_Points &model_points);

    std::map<std::string, Model_Points> m_points_by_model;
  };

} // namespace robot_model

#endif
