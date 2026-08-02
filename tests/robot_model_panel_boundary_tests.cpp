#include "robot_model_panel.h"

#include <type_traits>

static_assert(std::is_final_v<Robot_Model_Panel>,
              "Robot_Model_Panel is a leaf wx host, not an extension point");
static_assert(sizeof(Robot_Model_Panel) <=
                sizeof(wxPanel) + sizeof(void *) * 2,
              "Workflow state leaked back into Robot_Model_Panel");

int main()
{
  return 0;
}
