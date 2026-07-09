#include "camera_config_dialog.h"

#include "camera_service.h"

#include <wx/button.h>
#include <wx/listctrl.h>
#include <wx/sizer.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <cstdint>
#include <string>

namespace
{
wxString From_Utf8(const char *text)
{
  return wxString::FromUTF8(text);
}

wxString From_Utf8(const std::string &text)
{
  return wxString::FromUTF8(text.c_str());
}

wxString Device_Type_Label(std::uint32_t device_type)
{
  switch (device_type)
  {
  case DeviceType_Ethernet:
    return From_Utf8(u8"以太网");
  case DeviceType_USB:
    return "USB";
  case DeviceType_Ethernet_Vir:
    return From_Utf8(u8"虚拟以太网");
  case DeviceType_USB_Vir:
    return From_Utf8(u8"虚拟 USB");
  default:
    return wxString::Format(From_Utf8(u8"未知 (0x%X)"), device_type);
  }
}

wxString Camera_State_Label(Camera_State state)
{
  switch (state)
  {
  case Camera_State::Uninitialized:
    return From_Utf8(u8"未初始化");
  case Camera_State::Initialized:
    return From_Utf8(u8"已初始化");
  case Camera_State::Opened:
    return From_Utf8(u8"已打开");
  case Camera_State::Grabbing:
    return From_Utf8(u8"采集中");
  case Camera_State::Error:
    return From_Utf8(u8"错误");
  }
  return From_Utf8(u8"未知");
}
} // namespace

Camera_Config_Dialog::Camera_Config_Dialog(
    wxWindow *parent, Camera_Service &camera_service)
    : wxDialog(parent, wxID_ANY, From_Utf8(u8"3D相机配置"),
               wxDefaultPosition, wxSize(760, 460),
               wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
      m_camera_service(camera_service)
{
  Build_Ui();
  Bind(wxEVT_SHOW, &Camera_Config_Dialog::On_Show, this);
  m_camera_service.Bind(
      wxEVT_CAMERA_UPDATED,
      &Camera_Config_Dialog::On_Camera_Updated,
      this);
  CentreOnParent();
}

Camera_Config_Dialog::~Camera_Config_Dialog()
{
  m_camera_service.Unbind(
      wxEVT_CAMERA_UPDATED,
      &Camera_Config_Dialog::On_Camera_Updated,
      this);
}

void Camera_Config_Dialog::Build_Ui()
{
  m_version_text = new wxStaticText(
      this, wxID_ANY, From_Utf8(u8"SDK版本：未初始化"));
  m_state_text = new wxStaticText(
      this, wxID_ANY, From_Utf8(u8"状态：未初始化"));
  m_status_text = new wxStaticText(
      this, wxID_ANY, From_Utf8(u8"等待搜索设备"));

  m_device_list = new wxListCtrl(
      this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
      wxLC_REPORT | wxLC_SINGLE_SEL | wxLC_HRULES | wxLC_VRULES);
  m_device_list->InsertColumn(0, From_Utf8(u8"序号"), wxLIST_FORMAT_LEFT, 64);
  m_device_list->InsertColumn(1, From_Utf8(u8"型号"), wxLIST_FORMAT_LEFT, 180);
  m_device_list->InsertColumn(2, From_Utf8(u8"序列号"), wxLIST_FORMAT_LEFT, 180);
  m_device_list->InsertColumn(3, From_Utf8(u8"连接类型"), wxLIST_FORMAT_LEFT, 110);
  m_device_list->InsertColumn(4, From_Utf8(u8"IP地址"), wxLIST_FORMAT_LEFT, 150);

  m_refresh_button = new wxButton(
      this, wxID_REFRESH, From_Utf8(u8"刷新设备"));
  m_ok_button = new wxButton(this, wxID_OK, From_Utf8(u8"打开"));
  auto *cancel_button = new wxButton(
      this, wxID_CANCEL, From_Utf8(u8"取消"));
  m_ok_button->SetDefault();
  m_ok_button->Enable(false);

  m_refresh_button->Bind(
      wxEVT_BUTTON, &Camera_Config_Dialog::On_Refresh, this);
  m_ok_button->Bind(
      wxEVT_BUTTON, &Camera_Config_Dialog::On_Confirm, this);
  m_device_list->Bind(
      wxEVT_LIST_ITEM_SELECTED,
      &Camera_Config_Dialog::On_Device_Selection_Changed, this);
  m_device_list->Bind(
      wxEVT_LIST_ITEM_DESELECTED,
      &Camera_Config_Dialog::On_Device_Selection_Changed, this);
  m_device_list->Bind(
      wxEVT_LIST_ITEM_ACTIVATED,
      &Camera_Config_Dialog::On_Device_Activated, this);

  auto *header_sizer = new wxBoxSizer(wxHORIZONTAL);
  header_sizer->Add(m_version_text, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 16);
  header_sizer->Add(m_state_text, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 16);
  header_sizer->Add(m_status_text, 1, wxALIGN_CENTER_VERTICAL);

  auto *button_sizer = new wxBoxSizer(wxHORIZONTAL);
  button_sizer->Add(m_refresh_button, 0, wxRIGHT, 8);
  button_sizer->AddStretchSpacer(1);
  button_sizer->Add(m_ok_button, 0, wxRIGHT, 8);
  button_sizer->Add(cancel_button, 0);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(header_sizer, 0, wxEXPAND | wxALL, 12);
  sizer->Add(new wxStaticLine(this), 0, wxEXPAND | wxLEFT | wxRIGHT, 12);
  sizer->Add(m_device_list, 1, wxEXPAND | wxALL, 12);
  sizer->Add(button_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 12);
  SetSizer(sizer);
  SetMinSize(wxSize(640, 360));
}

void Camera_Config_Dialog::Load_Devices()
{
  if (m_camera_service.Is_Busy())
  {
    m_status_text->SetLabel(From_Utf8(u8"相机操作正在进行……"));
    Set_Busy(true);
    return;
  }

  if (!m_camera_service.Can_Refresh_Devices())
  {
    Populate_Device_List();
    m_status_text->SetLabel(
        m_camera_service.State() == Camera_State::Error
            ? From_Utf8(u8"相机操作失败：") +
                  From_Utf8(m_camera_service.Last_Error())
            : From_Utf8(u8"请先停止采集并关闭相机后再刷新设备"));
    Set_Busy(false);
    return;
  }

  const bool initializing = !m_camera_service.Is_Initialized();
  m_status_text->SetLabel(initializing
                              ? From_Utf8(u8"正在初始化SDK并搜索设备……")
                              : From_Utf8(u8"正在搜索设备……"));
  Set_Busy(true);
  Layout();
  Update();

  m_close_on_success = false;
  m_pending_request = m_camera_service.Request_Refresh_Devices();
  if (m_pending_request == 0)
  {
    m_status_text->SetLabel(From_Utf8(u8"无法提交相机刷新请求"));
    Set_Busy(false);
  }
}

void Camera_Config_Dialog::Populate_Device_List()
{
  m_device_list->DeleteAllItems();
  const auto &devices = m_camera_service.Devices();
  const auto &selected_serial = m_camera_service.Selected_Serial_Number();

  for (std::size_t i = 0; i < devices.size(); ++i)
  {
    const auto &device = devices[i];
    const auto row = m_device_list->InsertItem(
        static_cast<long>(i), wxString::Format("%zu", device.index + 1));
    m_device_list->SetItem(row, 1, From_Utf8(device.model_name));
    m_device_list->SetItem(row, 2, From_Utf8(device.serial_number));
    m_device_list->SetItem(row, 3, Device_Type_Label(device.device_type));
    m_device_list->SetItem(
        row, 4, device.ip_address.empty() ? "-" : From_Utf8(device.ip_address));
    m_device_list->SetItemData(row, static_cast<wxUIntPtr>(i));

    if (!selected_serial.empty() && device.serial_number == selected_serial)
    {
      m_device_list->SetItemState(
          row, wxLIST_STATE_SELECTED | wxLIST_STATE_FOCUSED,
          wxLIST_STATE_SELECTED | wxLIST_STATE_FOCUSED);
      m_device_list->EnsureVisible(row);
    }
  }
}

void Camera_Config_Dialog::Set_Busy(bool busy)
{
  m_busy = busy;
  m_refresh_button->Enable(
      !busy && m_camera_service.Can_Refresh_Devices());
  m_device_list->Enable(!busy);
  Update_State_Text();
  Update_Selection_State();
}

void Camera_Config_Dialog::Update_State_Text()
{
  m_state_text->SetLabel(
      From_Utf8(u8"状态：") + Camera_State_Label(m_camera_service.State()));
}

void Camera_Config_Dialog::Update_Selection_State()
{
  m_ok_button->Enable(
      !m_busy &&
      m_camera_service.Can_Select_Device() &&
      Selected_Row() != wxNOT_FOUND);
}

long Camera_Config_Dialog::Selected_Row() const
{
  return m_device_list->GetNextItem(
      -1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
}

void Camera_Config_Dialog::On_Show(wxShowEvent &event)
{
  event.Skip();
  if (!event.IsShown() || m_initial_load_started)
    return;

  m_initial_load_started = true;
  CallAfter([this]() { Load_Devices(); });
}

void Camera_Config_Dialog::On_Refresh(wxCommandEvent &)
{
  Load_Devices();
}

void Camera_Config_Dialog::On_Device_Selection_Changed(wxListEvent &)
{
  Update_Selection_State();
}

void Camera_Config_Dialog::On_Device_Activated(wxListEvent &)
{
  wxCommandEvent event;
  On_Confirm(event);
}

void Camera_Config_Dialog::On_Confirm(wxCommandEvent &)
{
  const auto row = Selected_Row();
  if (row == wxNOT_FOUND)
    return;

  const auto device_index = static_cast<std::size_t>(
      m_device_list->GetItemData(row));
  const auto &devices = m_camera_service.Devices();
  if (device_index >= devices.size())
    return;

  m_status_text->SetLabel(From_Utf8(u8"正在选择设备……"));
  m_close_on_success = true;
  m_pending_request = m_camera_service.Request_Select_Device(
      devices[device_index].serial_number);
  if (m_pending_request == 0)
  {
    m_close_on_success = false;
    m_status_text->SetLabel(From_Utf8(u8"无法提交设备选择请求"));
    Set_Busy(false);
    return;
  }
  Set_Busy(true);
}

void Camera_Config_Dialog::On_Camera_Updated(wxThreadEvent &event)
{
  event.Skip();
  const auto completed_request = m_camera_service.Last_Completed_Request();
  const auto operation = m_camera_service.Last_Operation();
  const bool request_matches =
      m_pending_request == 0 || completed_request == m_pending_request;

  Update_State_Text();
  if (operation == Camera_Operation::Refresh_Devices && request_matches)
  {
    if (m_camera_service.Is_Initialized())
    {
      const auto &version = m_camera_service.Version();
      m_version_text->SetLabel(wxString::Format(
          From_Utf8(u8"SDK版本：%u.%u.%u"),
          version.major, version.minor, version.revision));
    }
    else
    {
      m_version_text->SetLabel(From_Utf8(u8"SDK版本：未初始化"));
    }
    Populate_Device_List();

    if (m_camera_service.Last_Operation_Succeeded())
    {
      const auto device_count = m_camera_service.Devices().size();
      m_status_text->SetLabel(
          device_count == 0
              ? From_Utf8(u8"未发现3D相机，请检查连接后刷新")
              : wxString::Format(
                    From_Utf8(u8"已发现 %zu 台设备"), device_count));
    }
    else
    {
      m_status_text->SetLabel(
          From_Utf8(u8"操作失败：") +
          From_Utf8(m_camera_service.Last_Error()));
    }
  }
  else if (operation == Camera_Operation::Select_Device && request_matches)
  {
    if (m_camera_service.Last_Operation_Succeeded() && m_close_on_success)
    {
      m_status_text->SetLabel(From_Utf8(u8"正在打开相机……"));
      m_pending_request = m_camera_service.Request_Open_Selected_Device();
      if (m_pending_request == 0)
      {
        m_close_on_success = false;
        m_status_text->SetLabel(From_Utf8(u8"无法提交相机打开请求"));
        Set_Busy(false);
        return;
      }
      Set_Busy(true);
      return;
    }
    m_status_text->SetLabel(
        From_Utf8(u8"设备选择失败：") +
        From_Utf8(m_camera_service.Last_Error()));
  }
  else if (operation == Camera_Operation::Open_Device && request_matches)
  {
    if (m_camera_service.Last_Operation_Succeeded() && m_close_on_success)
    {
      m_pending_request = 0;
      m_close_on_success = false;
      EndModal(wxID_OK);
      return;
    }
    m_status_text->SetLabel(
        From_Utf8(u8"相机打开失败：") +
        From_Utf8(m_camera_service.Last_Error()));
  }
  else if (operation == Camera_Operation::Frame_Error)
  {
    m_status_text->SetLabel(
        From_Utf8(u8"采集失败：") +
        From_Utf8(m_camera_service.Last_Error()));
  }

  if (request_matches)
  {
    m_pending_request = 0;
    m_close_on_success = false;
  }
  Set_Busy(m_camera_service.Is_Busy());
}
