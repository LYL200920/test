#include "kuka_connection_config.h"
#include "app_paths.h"

#include <fstream>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace kuka
{
namespace
{

void Set_Error(std::string *error_message, const std::string &message)
{
  if (error_message)
    *error_message = message;
}

std::string Trim_Carriage_Return(std::string value)
{
  if (!value.empty() && value.back() == '\r')
    value.pop_back();
  return value;
}

} // namespace

std::filesystem::path Connection_Config_Path()
{
  return application::Get_App_Paths().config_root /
         "robot_connection.ini";
}

bool Load_Connection_Config(
    const std::filesystem::path &path,
    Connection_Config *configuration,
    std::string *error_message)
{
  if (error_message)
    error_message->clear();
  if (!configuration)
  {
    Set_Error(error_message, "Robot connection configuration output is null.");
    return false;
  }

  *configuration = {};
  if (!std::filesystem::exists(path))
    return true;

  std::ifstream input(path);
  if (!input)
  {
    Set_Error(error_message, "Unable to open robot connection configuration.");
    return false;
  }

  Connection_Config parsed;
  std::string line;
  while (std::getline(input, line))
  {
    line = Trim_Carriage_Return(std::move(line));
    const auto separator = line.find('=');
    if (separator == std::string::npos)
      continue;
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1);
    if (key == "host")
      parsed.host = value;
    else if (key == "port")
    {
      try
      {
        const unsigned long port = std::stoul(value);
        if (port == 0 ||
            port > std::numeric_limits<unsigned short>::max())
          throw std::out_of_range("port");
        parsed.port = static_cast<unsigned short>(port);
      }
      catch (const std::exception &)
      {
        Set_Error(error_message, "Robot connection port is invalid.");
        return false;
      }
    }
    else if (key == "model")
      parsed.model_id = value;
  }

  if (parsed.host.empty() || parsed.model_id.empty())
  {
    Set_Error(
        error_message,
        "Robot connection host or bound model is missing.");
    return false;
  }
  *configuration = std::move(parsed);
  return true;
}

bool Save_Connection_Config(
    const std::filesystem::path &path,
    const Connection_Config &configuration,
    std::string *error_message)
{
  if (error_message)
    error_message->clear();
  if (path.empty() || configuration.host.empty() ||
      configuration.port == 0 || configuration.model_id.empty())
  {
    Set_Error(error_message, "Robot connection configuration is invalid.");
    return false;
  }
  if (configuration.host.find_first_of("\r\n=") != std::string::npos ||
      configuration.model_id.find_first_of("\r\n=") != std::string::npos)
  {
    Set_Error(
        error_message,
        "Robot connection configuration contains invalid characters.");
    return false;
  }

  std::error_code filesystem_error;
  std::filesystem::create_directories(path.parent_path(), filesystem_error);
  if (filesystem_error)
  {
    Set_Error(
        error_message,
        "Unable to create robot connection configuration directory.");
    return false;
  }

  std::ofstream output(path, std::ios::trunc);
  if (!output)
  {
    Set_Error(error_message, "Unable to save robot connection configuration.");
    return false;
  }
  output << "host=" << configuration.host << '\n'
         << "port=" << configuration.port << '\n'
         << "model=" << configuration.model_id << '\n';
  if (!output)
  {
    Set_Error(error_message, "Unable to write robot connection configuration.");
    return false;
  }
  return true;
}

} // namespace kuka
