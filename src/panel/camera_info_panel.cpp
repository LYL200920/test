#include "camera_info_panel.h"

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <cstdint>
#include <string>
#include <utility>

namespace
{
wxString From_Utf8 (const char* text)
{
  return wxString::FromUTF8 (text);
}

wxString From_Utf8 (const std::string& text)
{
  return wxString::FromUTF8 (text.c_str ( ));
}

wxString Value_Or_Dash (const std::string& value)
{
  return value.empty ( ) ? wxString ("-") : From_Utf8 (value);
}

wxString Device_Type_Label (std::uint32_t device_type)
{
  switch( device_type )
  {
  case DeviceType_Ethernet:
    return From_Utf8 (u8"以太网");
  case DeviceType_USB:
    return "USB";
  case DeviceType_Ethernet_Vir:
    return From_Utf8 (u8"虚拟以太网");
  case DeviceType_USB_Vir:
    return From_Utf8 (u8"虚拟 USB");
  default:
    return wxString::Format (From_Utf8 (u8"未知 (0x%X)"), device_type);
  }
}

wxString Product_Line_Label (std::uint32_t product_line)
{
  switch( product_line )
  {
  case 1:
    return From_Utf8 (u8"标准产品");
  case 2:
    return From_Utf8 (u8"3D产品");
  case 3:
    return From_Utf8 (u8"智能ID产品");
  default:
    return From_Utf8 (u8"未知");
  }
}

wxString Ip_Config_Mode_Label (std::uint32_t mode)
{
  switch( mode )
  {
  case IpCfgMode_Static:
    return From_Utf8 (u8"静态 IP");
  case IpCfgMode_DHCP:
    return "DHCP";
  case IpCfgMode_LLA:
    return "LLA";
  default:
    return wxString::Format (From_Utf8 (u8"未知 (%u)"), mode);
  }
}

wxString Usb_Protocol_Label (std::uint32_t protocol)
{
  switch( protocol )
  {
  case UsbProtocol_USB2:
    return "USB 2.0";
  case UsbProtocol_USB3:
    return "USB 3.0";
  default:
    return wxString::Format (From_Utf8 (u8"未知 (%u)"), protocol);
  }
}

bool Is_Network_Device (std::uint32_t device_type)
{
  return device_type == DeviceType_Ethernet ||
         device_type == DeviceType_Ethernet_Vir;
}

bool Is_Usb_Device (std::uint32_t device_type)
{
  return device_type == DeviceType_USB ||
         device_type == DeviceType_USB_Vir;
}

wxString Format_Detail (const Camera_Device_Detail& detail)
{
  wxString text;
  text << From_Utf8 (u8"厂商：") << Value_Or_Dash (detail.manufacturer_name)
       << '\n';
  text << From_Utf8 (u8"型号：") << Value_Or_Dash (detail.model_name) << '\n';
  text << From_Utf8 (u8"设备版本：") << Value_Or_Dash (detail.device_version)
       << '\n';
  text << From_Utf8 (u8"序列号：") << Value_Or_Dash (detail.serial_number)
       << '\n';
  text << From_Utf8 (u8"自定义名称：")
       << Value_Or_Dash (detail.user_defined_name) << '\n';
  text << From_Utf8 (u8"连接类型：")
       << Device_Type_Label (detail.device_type) << '\n';
  text << From_Utf8 (u8"产品线：") << Product_Line_Label (detail.product_line)
       << wxString::Format (" (%u)", detail.product_line) << '\n';
  text << From_Utf8 (u8"类型信息：")
       << wxString::Format ("0x%08X", detail.device_type_info);

  if( Is_Network_Device (detail.device_type) )
  {
    text << '\n' << "MAC: " << Value_Or_Dash (detail.mac_address);
    text << '\n' << From_Utf8 (u8"IP模式：")
         << Ip_Config_Mode_Label (detail.ip_config_mode);
    text << '\n' << From_Utf8 (u8"设备IP：")
         << Value_Or_Dash (detail.current_ip);
    text << '\n' << From_Utf8 (u8"子网掩码：")
         << Value_Or_Dash (detail.subnet_mask);
    text << '\n' << From_Utf8 (u8"默认网关：")
         << Value_Or_Dash (detail.default_gateway);
    text << '\n' << From_Utf8 (u8"主机网口IP：")
         << Value_Or_Dash (detail.network_interface_ip);
  }
  else if( Is_Usb_Device (detail.device_type) )
  {
    text << '\n' << "VID: " << wxString::Format ("0x%04X", detail.vendor_id);
    text << '\n' << "PID: " << wxString::Format ("0x%04X", detail.product_id);
    text << '\n' << From_Utf8 (u8"USB协议：")
         << Usb_Protocol_Label (detail.usb_protocol);
    text << '\n' << "GUID: " << Value_Or_Dash (detail.device_guid);
  }

  if( !detail.manufacturer_specific_info.empty ( ) )
  {
    text << '\n' << From_Utf8 (u8"厂商信息：")
         << From_Utf8 (detail.manufacturer_specific_info);
  }
  return text;
}
} // namespace

Camera_Info_Panel::Camera_Info_Panel (wxWindow* parent)
  : wxScrolledWindow (parent, wxID_ANY)
{
  SetScrollRate (0, 10);

  auto* title = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"当前设备信息"));
  m_detail_text = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"相机打开后自动读取"));
  m_refresh_button = new wxButton (
    this, wxID_ANY, From_Utf8 (u8"刷新设备信息"));
  m_refresh_button->Bind (
    wxEVT_BUTTON, &Camera_Info_Panel::On_Refresh, this);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (m_detail_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_refresh_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->AddStretchSpacer (1);
  SetSizer (sizer);
}

void Camera_Info_Panel::Set_On_Refresh (
  std::function<void ( )> callback)
{
  m_on_refresh = std::move (callback);
}

void Camera_Info_Panel::Update_View (
  const std::optional<Camera_Device_Detail>& detail,
  bool device_open,
  bool busy)
{
  m_refresh_button->Enable (!busy && device_open);

  if( detail )
  {
    m_detail_text->SetLabel (Format_Detail (*detail));
    m_detail_text->Wrap (360);
  }
  else
  {
    m_detail_text->SetLabel (
      device_open
        ? From_Utf8 (u8"正在等待设备信息……")
        : From_Utf8 (u8"相机打开后自动读取"));
  }

  Layout ( );
  FitInside ( );
}

void Camera_Info_Panel::On_Refresh (wxCommandEvent&)
{
  if( m_on_refresh )
  {
    m_on_refresh ( );
  }
}
