#include "camera_intrinsic_calibration_workflow.h"

#include <cassert>
#include <string>

namespace
{

camera_calibration::Image_Detection_Result Make_Detection(
  int zone_x,
  int zone_y,
  const std::string &id)
{
  camera_calibration::Image_Detection_Result result;
  result.image_id = id;
  result.chessboard_found = true;
  result.image_width = 3000;
  result.image_height = 3000;
  const double offset_x = zone_x * 1000.0 + 200.0;
  const double offset_y = zone_y * 1000.0 + 200.0;
  for (int row = 0; row < 19; ++row)
  {
    for (int column = 0; column < 17; ++column)
    {
      result.corners.push_back({
        offset_x + column * 30.0,
        offset_y + row * 30.0});
    }
  }
  return result;
}

void Test_Readiness_Requires_Count_And_Coverage()
{
  application::Camera_Intrinsic_Calibration_Workflow workflow;
  for (int index = 0; index < 10; ++index)
  {
    workflow.Add_Capture({Make_Detection(
      index % 3,
      (index / 3) % 3,
      "image_" + std::to_string(index))});
  }
  const auto readiness = workflow.Readiness(10);
  assert(readiness.ready);
  assert(readiness.enabled_image_count == 10);
  assert(readiness.covered_zone_count >= 5);
}

void Test_Failed_Detection_Cannot_Be_Enabled()
{
  application::Camera_Intrinsic_Calibration_Workflow workflow;
  camera_calibration::Image_Detection_Result failed;
  failed.image_id = "failed.png";
  workflow.Add_Capture({failed});
  assert(workflow.Captures().size() == 1);
  assert(!workflow.Captures().front().enabled);
  assert(!workflow.Set_Capture_Enabled(0, true));
}

} // namespace

int main()
{
  Test_Readiness_Requires_Count_And_Coverage();
  Test_Failed_Detection_Cannot_Be_Enabled();
  return 0;
}
