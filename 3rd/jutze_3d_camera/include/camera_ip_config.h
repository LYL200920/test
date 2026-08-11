#ifndef includeguard_camera_ip_config_h_includeguard
#define includeguard_camera_ip_config_h_includeguard

#include <string>

enum class Camera_Ip_Mode
{
  Static,
  Dhcp,
  Lla
};

struct Camera_Ip_Configuration
{
  Camera_Ip_Mode mode = Camera_Ip_Mode::Static;
  std::string ip_address;
  std::string subnet_mask;
  std::string default_gateway;
};

bool Validate_Camera_Ip_Configuration(
    const Camera_Ip_Configuration &configuration,
    std::string *error = nullptr);

const char *Camera_Ip_Mode_Name(Camera_Ip_Mode mode);

#endif
