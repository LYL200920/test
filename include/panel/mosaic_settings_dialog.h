#ifndef includeguard_mosaic_settings_dialog_h_includeguard
#define includeguard_mosaic_settings_dialog_h_includeguard

#include "mosaic_configuration.h"
class wxWindow;

// Edits a copy; cancel never changes the supplied configuration or teach points.
bool Edit_Mosaic_Settings(wxWindow *parent,
  const std::vector<robot_model::Robot_Teach_Point> &points,
  robot_model::Mosaic_Configuration *configuration);

#endif
