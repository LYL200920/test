#include "robot_debug_dump.h"

#include <Windows.h>

#include <algorithm>
#include <sstream>

namespace robot_model
{

void Dump_Wrist_Bounds (const std::vector<Robot_Visual_Part>& parts,
                        const std::string& model_name)
{
  if( parts.size ( ) < 7 )
  {
    return;
  }

  std::ostringstream title;
  title << "[RobotModel] wrist bounds for " << model_name;
  OutputDebugStringA (title.str ( ).c_str ( ));
  OutputDebugStringA ("\n");

  for( size_t i = 4; i <= 6; ++i )
  {
    if( !parts[i].actor )
    {
      continue;
    }

    double bounds[6] = {};
    parts[i].actor->GetBounds (bounds);

    std::ostringstream line;
    line << "[RobotModel] part " << i
         << " bounds x=(" << bounds[0] << ", " << bounds[1] << ")"
         << " y=(" << bounds[2] << ", " << bounds[3] << ")"
         << " z=(" << bounds[4] << ", " << bounds[5] << ")";
    OutputDebugStringA (line.str ( ).c_str ( ));
    OutputDebugStringA ("\n");
  }
}

void Dump_All_Parts_Info (const std::vector<Robot_Visual_Part>& parts,
                          const Robot_Assembly_Calibration& calibration,
                          const std::string& model_name)
{
  std::ostringstream header;
  header << "[RobotModel] ===== All parts dump for " << model_name
         << " (total=" << parts.size ( ) << ") =====";
  OutputDebugStringA (header.str ( ).c_str ( ));
  OutputDebugStringA ("\n");

  const auto count = std::min (parts.size ( ), calibration.parts.size ( ));
  for( size_t i = 0; i < count; ++i )
  {
    const auto& part = parts[i];
    const auto& calib = calibration.parts[i];

    std::ostringstream line;
    line << "[RobotModel] part " << i
         << " file=" << part.mesh_path.filename ( ).string ( );

    line << " calib_T=(" << calib.translate[0] << ","
         << calib.translate[1] << "," << calib.translate[2] << ")";
    line << " calib_R=(" << calib.rotate_xyz[0] << ","
         << calib.rotate_xyz[1] << "," << calib.rotate_xyz[2] << ")";

    if( part.actor )
    {
      double bounds[6] = {};
      part.actor->GetBounds (bounds);
      const double cx = ( bounds[0] + bounds[1] ) * 0.5;
      const double cy = ( bounds[2] + bounds[3] ) * 0.5;
      const double cz = ( bounds[4] + bounds[5] ) * 0.5;
      line << " bounds_center=(" << cx << "," << cy << "," << cz << ")";
      line << " bounds_size=(" << ( bounds[1] - bounds[0] ) << ","
           << ( bounds[3] - bounds[2] ) << ","
           << ( bounds[5] - bounds[4] ) << ")";
      line << " bounds_min=(" << bounds[0] << "," << bounds[2] << ","
           << bounds[4] << ")";
      line << " bounds_max=(" << bounds[1] << "," << bounds[3] << ","
           << bounds[5] << ")";
    }
    else
    {
      line << " (no actor)";
    }

    OutputDebugStringA (line.str ( ).c_str ( ));
    OutputDebugStringA ("\n");
  }

  OutputDebugStringA ("[RobotModel] ----- Neighbour gap analysis -----\n");
  for( size_t i = 0; i + 1 < parts.size ( ); ++i )
  {
    if( !parts[i].actor || !parts[i + 1].actor )
    {
      continue;
    }

    double b0[6] = {};
    double b1[6] = {};
    parts[i].actor->GetBounds (b0);
    parts[i + 1].actor->GetBounds (b1);

    const double gap_x = std::max (b0[0], b1[0]) - std::min (b0[1], b1[1]);
    const double gap_y = std::max (b0[2], b1[2]) - std::min (b0[3], b1[3]);
    const double gap_z = std::max (b0[4], b1[4]) - std::min (b0[5], b1[5]);

    std::ostringstream line;
    line << "[RobotModel] part " << i << " -> " << ( i + 1 )
         << "  gap_xyz=(" << gap_x << "," << gap_y << "," << gap_z << ")";
    if( gap_x < 0 || gap_y < 0 || gap_z < 0 )
    {
      line << " [OVERLAP!]";
    }
    else
    {
      line << " [separated]";
    }
    OutputDebugStringA (line.str ( ).c_str ( ));
    OutputDebugStringA ("\n");
  }
}

} // namespace robot_model
