#include "camera_ip_config.h"

#include <iostream>
#include <stdexcept>

namespace
{
void Require(bool condition, const char *message)
{
  if (!condition)
    throw std::runtime_error(message);
}

void Test_Static_Configuration()
{
  Camera_Ip_Configuration configuration;
  configuration.mode = Camera_Ip_Mode::Static;
  configuration.ip_address = "192.168.1.100";
  configuration.subnet_mask = "255.255.255.0";
  configuration.default_gateway = "192.168.1.1";
  Require(Validate_Camera_Ip_Configuration(configuration),
          "Valid static configuration was rejected");

  configuration.ip_address = "192.168.1.255";
  Require(!Validate_Camera_Ip_Configuration(configuration),
          "Broadcast address was accepted");

  configuration.ip_address = "192.168.1.100.";
  Require(!Validate_Camera_Ip_Configuration(configuration),
          "IPv4 address with trailing separator was accepted");
}

void Test_Mask_And_Gateway()
{
  Camera_Ip_Configuration configuration;
  configuration.mode = Camera_Ip_Mode::Static;
  configuration.ip_address = "192.168.1.100";
  configuration.subnet_mask = "255.0.255.0";
  Require(!Validate_Camera_Ip_Configuration(configuration),
          "Non-contiguous subnet mask was accepted");

  configuration.subnet_mask = "255.255.255.0";
  configuration.default_gateway = "192.168.2.1";
  Require(!Validate_Camera_Ip_Configuration(configuration),
          "Gateway outside the subnet was accepted");
}

void Test_Automatic_Modes()
{
  Camera_Ip_Configuration configuration;
  configuration.mode = Camera_Ip_Mode::Dhcp;
  Require(Validate_Camera_Ip_Configuration(configuration),
          "DHCP mode was rejected");
  configuration.mode = Camera_Ip_Mode::Lla;
  Require(Validate_Camera_Ip_Configuration(configuration),
          "LLA mode was rejected");
}
} // namespace

int main()
{
  try
  {
    Test_Static_Configuration();
    Test_Mask_And_Gateway();
    Test_Automatic_Modes();
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Camera IP configuration tests passed.\n";
  return 0;
}
