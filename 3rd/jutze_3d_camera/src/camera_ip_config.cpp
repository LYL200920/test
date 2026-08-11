#include "camera_ip_config.h"

#include <array>
#include <cctype>
#include <cstdint>

namespace
{
bool Return_Error(const std::string &message, std::string *error)
{
  if (error)
    *error = message;
  return false;
}

bool Parse_Ipv4(const std::string &text, std::uint32_t *value)
{
  if (text.empty() || value == nullptr)
    return false;

  std::array<std::uint32_t, 4> octets{};
  std::size_t octet_index = 0;
  std::size_t position = 0;
  while (position < text.size() && octet_index < octets.size())
  {
    const auto begin = position;
    std::uint32_t octet = 0;
    while (position < text.size() && text[position] != '.')
    {
      const auto character = static_cast<unsigned char>(text[position]);
      if (!std::isdigit(character))
        return false;
      octet = octet * 10U + static_cast<std::uint32_t>(character - '0');
      if (octet > 255U)
        return false;
      ++position;
    }
    if (position == begin)
      return false;
    octets[octet_index++] = octet;
    if (position < text.size())
    {
      if (position + 1U == text.size())
        return false;
      ++position;
    }
  }

  if (position != text.size() || octet_index != octets.size())
    return false;

  *value = (octets[0] << 24U) | (octets[1] << 16U) |
           (octets[2] << 8U) | octets[3];
  return true;
}
} // namespace

bool Validate_Camera_Ip_Configuration(
    const Camera_Ip_Configuration &configuration,
    std::string *error)
{
  if (error)
    error->clear();

  if (configuration.mode == Camera_Ip_Mode::Dhcp ||
      configuration.mode == Camera_Ip_Mode::Lla)
    return true;
  if (configuration.mode != Camera_Ip_Mode::Static)
    return Return_Error("Unknown camera IP configuration mode", error);

  std::uint32_t address = 0;
  if (!Parse_Ipv4(configuration.ip_address, &address) ||
      address == 0U || (address >> 24U) >= 224U)
  {
    return Return_Error("Invalid static IPv4 address", error);
  }

  std::uint32_t mask = 0;
  if (!Parse_Ipv4(configuration.subnet_mask, &mask) || mask == 0U)
    return Return_Error("Invalid IPv4 subnet mask", error);
  const auto inverse_mask = ~mask;
  if ((inverse_mask & (inverse_mask + 1U)) != 0U)
    return Return_Error("IPv4 subnet mask must contain contiguous bits", error);

  const auto host_bits = address & inverse_mask;
  if (host_bits == 0U || host_bits == inverse_mask)
    return Return_Error("Static IPv4 address is a network or broadcast address", error);

  if (!configuration.default_gateway.empty() &&
      configuration.default_gateway != "0.0.0.0")
  {
    std::uint32_t gateway = 0;
    if (!Parse_Ipv4(configuration.default_gateway, &gateway))
      return Return_Error("Invalid IPv4 default gateway", error);
    if ((gateway & mask) != (address & mask))
      return Return_Error("Default gateway is outside the camera subnet", error);
  }

  return true;
}

const char *Camera_Ip_Mode_Name(Camera_Ip_Mode mode)
{
  switch (mode)
  {
  case Camera_Ip_Mode::Static:
    return "Static";
  case Camera_Ip_Mode::Dhcp:
    return "DHCP";
  case Camera_Ip_Mode::Lla:
    return "LLA";
  }
  return "Unknown";
}
