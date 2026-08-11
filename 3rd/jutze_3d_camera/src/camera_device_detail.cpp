#include "camera_device_detail.h"

#include <algorithm>
#include <iomanip>
#include <iterator>
#include <sstream>

namespace
{
  template <std::size_t Size>
  std::string From_Fixed_String(const char (&value)[Size])
  {
    const auto end = std::find(std::begin(value), std::end(value), '\0');
    return std::string(std::begin(value), end);
  }

  std::string Format_Mac_Address(const unsigned char (&value)[8])
  {
    std::ostringstream text;
    text << std::hex << std::uppercase << std::setfill('0');
    for (std::size_t index = 0; index < 6; ++index)
    {
      if (index != 0)
      {
        text << ':';
      }
      text << std::setw(2) << static_cast<unsigned int>(value[index]);
    }
    return text.str();
  }

  bool Is_Network_Device(Mv3dRgbdDeviceType device_type)
  {
    return device_type == DeviceType_Ethernet ||
           device_type == DeviceType_Ethernet_Vir;
  }

  bool Is_Usb_Device(Mv3dRgbdDeviceType device_type)
  {
    return device_type == DeviceType_USB ||
           device_type == DeviceType_USB_Vir;
  }
} // namespace

Camera_Device_Detail Build_Camera_Device_Detail(
    const MV3D_RGBD_DEVICE_INFO &source)
{
  Camera_Device_Detail detail;
  detail.device_type = static_cast<std::uint32_t>(source.enDeviceType);
  detail.device_type_info = source.nDevTypeInfo;
  detail.product_line = (source.nDevTypeInfo & 0xFF000000U) >> 24U;
  detail.manufacturer_name = From_Fixed_String(source.chManufacturerName);
  detail.model_name = From_Fixed_String(source.chModelName);
  detail.device_version = From_Fixed_String(source.chDeviceVersion);
  detail.manufacturer_specific_info = From_Fixed_String(source.chManufacturerSpecificInfo);
  detail.serial_number = From_Fixed_String(source.chSerialNumber);
  detail.user_defined_name = From_Fixed_String(source.chUserDefinedName);

  if (Is_Network_Device(source.enDeviceType))
  {
    const auto &network = source.SpecialInfo.stNetInfo;
    detail.mac_address = Format_Mac_Address(network.chMacAddress);
    detail.ip_config_mode = static_cast<std::uint32_t>(network.enIPCfgMode);
    detail.current_ip = From_Fixed_String(network.chCurrentIp);
    detail.subnet_mask = From_Fixed_String(network.chCurrentSubNetMask);
    detail.default_gateway = From_Fixed_String(network.chDefultGateWay);
    detail.network_interface_ip = From_Fixed_String(network.chNetExport);
  }
  else if (Is_Usb_Device(source.enDeviceType))
  {
    const auto &usb = source.SpecialInfo.stUsbInfo;
    detail.vendor_id = usb.nVendorId;
    detail.product_id = usb.nProductId;
    detail.usb_protocol = static_cast<std::uint32_t>(usb.enUsbProtocol);
    detail.device_guid = From_Fixed_String(usb.chDeviceGUID);
  }

  return detail;
}
