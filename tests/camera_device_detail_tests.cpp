#include "camera_device_detail.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace
{
void Require (bool condition, const char* message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}

template <std::size_t Size>
void Copy_Text (char (&destination)[Size], const char* source)
{
  std::strncpy (destination, source, Size - 1);
  destination[Size - 1] = '\0';
}

void Test_Network_Device ( )
{
  MV3D_RGBD_DEVICE_INFO source{};
  source.enDeviceType = DeviceType_Ethernet;
  source.nDevTypeInfo = 0x02000034U;
  Copy_Text (source.chManufacturerName, "HIKROBOT");
  Copy_Text (source.chModelName, "MV-DB1300");
  Copy_Text (source.chDeviceVersion, "V1.2.3");
  Copy_Text (source.chSerialNumber, "SN123456");
  Copy_Text (source.chUserDefinedName, "Camera-A");
  auto& network = source.SpecialInfo.stNetInfo;
  const unsigned char mac[] = { 0x00, 0x11, 0x22, 0xAA, 0xBB, 0xCC };
  std::copy (std::begin (mac), std::end (mac), network.chMacAddress);
  network.enIPCfgMode = IpCfgMode_DHCP;
  Copy_Text (network.chCurrentIp, "192.168.1.10");
  Copy_Text (network.chCurrentSubNetMask, "255.255.255.0");
  Copy_Text (network.chDefultGateWay, "192.168.1.1");
  Copy_Text (network.chNetExport, "192.168.1.20");

  const auto detail = Build_Camera_Device_Detail (source);
  Require (detail.device_type == DeviceType_Ethernet, "Network type mismatch");
  Require (detail.product_line == 2, "Product line mismatch");
  Require (detail.manufacturer_name == "HIKROBOT", "Manufacturer mismatch");
  Require (detail.serial_number == "SN123456", "Serial number mismatch");
  Require (detail.mac_address == "00:11:22:AA:BB:CC", "MAC mismatch");
  Require (detail.current_ip == "192.168.1.10", "IP mismatch");
  Require (detail.ip_config_mode == IpCfgMode_DHCP, "IP mode mismatch");
  Require (detail.vendor_id == 0, "Network device contains USB data");
}

void Test_Usb_Device ( )
{
  MV3D_RGBD_DEVICE_INFO source{};
  source.enDeviceType = DeviceType_USB;
  Copy_Text (source.chModelName, "MV-USB-RGBD");
  Copy_Text (source.chSerialNumber, "USB123");
  auto& usb = source.SpecialInfo.stUsbInfo;
  usb.nVendorId = 0x2BDF;
  usb.nProductId = 0x0001;
  usb.enUsbProtocol = UsbProtocol_USB3;
  Copy_Text (usb.chDeviceGUID, "{1234-5678}");

  const auto detail = Build_Camera_Device_Detail (source);
  Require (detail.device_type == DeviceType_USB, "USB type mismatch");
  Require (detail.vendor_id == 0x2BDF, "VID mismatch");
  Require (detail.product_id == 0x0001, "PID mismatch");
  Require (detail.usb_protocol == UsbProtocol_USB3, "USB protocol mismatch");
  Require (detail.device_guid == "{1234-5678}", "GUID mismatch");
  Require (detail.mac_address.empty ( ), "USB device contains network data");
}
} // namespace

int main ( )
{
  try
  {
    Test_Network_Device ( );
    Test_Usb_Device ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }

  std::cout << "Camera device detail tests passed.\n";
  return 0;
}
