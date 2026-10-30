#include "robot_model_panel.h"
#include "mosaic_settings_panel.h"
#include "mosaic_preview_panel.h"
#include "right_tool_panel.h"
#include <wx/dialog.h>

#include <type_traits>

static_assert(std::is_final_v<Robot_Model_Panel>,
              "Robot_Model_Panel is a leaf wx host, not an extension point");
static_assert(std::is_base_of_v<wxPanel, Robot_Model_Panel>,
              "Robot_Model_Panel must remain the native wx host");

static_assert(std::is_base_of_v<wxScrolledWindow, Mosaic_Settings_Panel>,
              "Mosaic settings must be an embedded scrollable tool page");
static_assert(!std::is_base_of_v<wxDialog, Mosaic_Settings_Panel>,
              "Mosaic settings must not block the model with a dialog");
static_assert(static_cast<int>(Right_Tool_Page::Mosaic) ==
                static_cast<int>(Right_Tool_Page::Tool) + 1,
              "Mosaic page must follow the existing tool page indices");

static_assert(std::is_base_of_v<wxPanel, Mosaic_Preview_Panel> &&
              !std::is_base_of_v<wxDialog, Mosaic_Preview_Panel>,
              "Mosaic preview must fit inside the existing preview book");

int main()
{
  return 0;
}
