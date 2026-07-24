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
        const XyzabcPose &world_pose);

    void Replace_Joint_Points(const std::string &robot_model_id, const std::vector<std::array<double, 6>> &joint_points);
    bool Delete_Point(const std::string &robot_model_id, std::size_t index);
    void Clear_Points(const std::string &robot_model_id);

    const std::vector<Robot_Teach_Point> &Points(const std::string &robot_model_id) const;
    std::size_t Point_Count(const std::string &robot_model_id) const;

  private:
    struct Model_Points
    {
      std::vector<Robot_Teach_Point> points;
      std::size_t next_id = 1;
    };

    std::map<std::string, Model_Points> m_points_by_model;
  };

} // namespace robot_model

#endif
