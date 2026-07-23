#ifndef includeguard_camera_device_detail_h_includeguard
#define includeguard_camera_device_detail_h_includeguard

#include "Mv3dRgbdDefine.h"

#include <cstdint>
#include <string>

struct Camera_Device_Detail
{
  std::uint32_t device_type = 0;
  std::uint32_t device_type_info = 0;
  std::uint32_t product_line = 0;
  std::string manufacturer_name;
  std::string model_name;
  std::string device_version;
  std::string manufacturer_specific_info;
  std::string serial_number;
  std::string user_defined_name;

  std::string mac_address;
  std::uint32_t ip_config_mode = 0;
  std::string current_ip;
  std::string subnet_mask;
  std::string default_gateway;
  std::string network_interface_ip;

  std::uint32_t vendor_id = 0;
  std::uint32_t product_id = 0;
  std::uint32_t usb_protocol = 0;
  std::string device_guid;
};

Camera_Device_Detail Build_Camera_Device_Detail(const MV3D_RGBD_DEVICE_INFO &source);

#endif
