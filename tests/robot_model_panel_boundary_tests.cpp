#include "robot_model_panel.h"

#include <type_traits>

static_assert(std::is_final_v<Robot_Model_Panel>,
              "Robot_Model_Panel is a leaf wx host, not an extension point");
static_assert(std::is_base_of_v<wxPanel, Robot_Model_Panel>,
              "Robot_Model_Panel must remain the native wx host");

int main()
{
  return 0;
}
