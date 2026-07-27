#ifndef includeguard_robot_progress_io_h_includeguard
#define includeguard_robot_progress_io_h_includeguard

#include "robot_teach_point.h"

#include <filesystem>
#include <string>
#include <vector>

namespace robot_model
{

struct Robot_Progress_File
{
  std::string robot_model_id;
  std::vector<Robot_Teach_Point> points;
};

bool Save_Robot_Progress(
  const std::filesystem::path &path,
  const Robot_Progress_File &progress,
  std::string *error_message = nullptr);
bool Load_Robot_Progress(
  const std::filesystem::path &path,
  Robot_Progress_File *progress,
  std::string *error_message = nullptr);

} // namespace robot_model

#endif
