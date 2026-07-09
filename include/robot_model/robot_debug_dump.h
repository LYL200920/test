#ifndef includeguard_robot_debug_dump_h_includeguard
#define includeguard_robot_debug_dump_h_includeguard

#include "robot_model_data.h"

#include <string>
#include <vector>

namespace robot_model
{

void Dump_Wrist_Bounds (const std::vector<Robot_Visual_Part>& parts,
                        const std::string& model_name);

void Dump_All_Parts_Info (const std::vector<Robot_Visual_Part>& parts,
                          const Robot_Assembly_Calibration& calibration,
                          const std::string& model_name);

} // namespace robot_model

#endif
