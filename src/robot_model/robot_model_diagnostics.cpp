#include "robot_model_diagnostics.h"

#include "app_paths.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace robot_model
{
namespace
{
#ifdef ROBOT_MODEL_DIAGNOSTICS_ENABLED
std::mutex g_diagnostic_mutex;
std::once_flag g_initialize_once;
std::filesystem::path g_log_path;

std::string path_as_utf8(const std::filesystem::path &path)
{
  try
  {
    return path.u8string();
  }
  catch (...)
  {
    return "<path conversion failed>";
  }
}

std::string timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto value = std::chrono::system_clock::to_time_t(now);
  std::tm local = {};
#ifdef _WIN32
  localtime_s(&local, &value);
#else
  localtime_r(&value, &local);
#endif
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y-%m-%d %H:%M:%S");
  return stream.str();
}

void append_line(std::string_view message)
{
  if (g_log_path.empty()) return;
  std::ofstream output(g_log_path, std::ios::out | std::ios::app);
  if (output) output << '[' << timestamp() << "] " << message << '\n';
}
#endif
} // namespace

void Initialize_Robot_Model_Diagnostics()
{
#ifdef ROBOT_MODEL_DIAGNOSTICS_ENABLED
  std::call_once(g_initialize_once, []
  {
    const auto paths = application::Get_App_Paths();
    g_log_path = paths.executable_directory / "robot_model_diagnostics.log";
    {
      std::ofstream output(g_log_path, std::ios::out | std::ios::trunc);
      if (!output) return;
      output << "Robot model diagnostics (temporary; disable with "
                "ENABLE_ROBOT_MODEL_DIAGNOSTICS=OFF)\n";
    }
    append_line("executable_directory=" +
                path_as_utf8(paths.executable_directory));
    append_line("resource_root=" + path_as_utf8(paths.resource_root));
    append_line("robot_model_root=" + path_as_utf8(paths.robot_model_root));
    std::error_code error;
    const bool resource_exists =
      std::filesystem::is_directory(paths.resource_root, error);
    append_line(std::string("resource_root_is_directory=") +
                (resource_exists ? "true" : "false") +
                (error ? " error=" + error.message() : ""));
    error.clear();
    const bool robot_root_exists =
      std::filesystem::is_directory(paths.robot_model_root, error);
    append_line(std::string("robot_model_root_is_directory=") +
                (robot_root_exists ? "true" : "false") +
                (error ? " error=" + error.message() : ""));
  });
#endif
}

void Write_Robot_Model_Diagnostic(std::string_view message)
{
#ifdef ROBOT_MODEL_DIAGNOSTICS_ENABLED
  Initialize_Robot_Model_Diagnostics();
  std::lock_guard<std::mutex> lock(g_diagnostic_mutex);
  append_line(message);
#else
  (void)message;
#endif
}

std::filesystem::path Robot_Model_Diagnostic_Log_Path()
{
#ifdef ROBOT_MODEL_DIAGNOSTICS_ENABLED
  Initialize_Robot_Model_Diagnostics();
  return g_log_path;
#else
  return {};
#endif
}

} // namespace robot_model
