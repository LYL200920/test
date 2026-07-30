#ifndef includeguard_kuka_connection_config_h_includeguard
#define includeguard_kuka_connection_config_h_includeguard

#include <filesystem>
#include <string>

namespace kuka
{

struct Connection_Config
{
  std::string host = "192.168.1.25";
  unsigned short port = 54600;
  std::string model_id = "KR10_R1100_2";
};

std::filesystem::path Connection_Config_Path();

bool Load_Connection_Config(
    const std::filesystem::path &path,
    Connection_Config *configuration,
    std::string *error_message = nullptr);
bool Save_Connection_Config(
    const std::filesystem::path &path,
    const Connection_Config &configuration,
    std::string *error_message = nullptr);

} // namespace kuka

#endif
