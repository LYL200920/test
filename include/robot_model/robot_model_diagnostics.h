#ifndef includeguard_robot_model_diagnostics_h_includeguard
#define includeguard_robot_model_diagnostics_h_includeguard

#include <filesystem>
#include <string_view>

namespace robot_model
{

// Temporary diagnostics for cross-machine model loading issues. Disable all
// output with -DENABLE_ROBOT_MODEL_DIAGNOSTICS=OFF when it is no longer needed.
void Initialize_Robot_Model_Diagnostics();
void Write_Robot_Model_Diagnostic(std::string_view message);
std::filesystem::path Robot_Model_Diagnostic_Log_Path();

} // namespace robot_model

#endif
