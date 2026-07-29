#include "camera_2d_control_panel.h"

#include "camera_2d_image_converter.h"
#include "camera_2d_service.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/filedlg.h>
#include <wx/image.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

#include <cstring>

namespace
{
wxString From_Utf8(const std::string &value)
{
  return wxString::FromUTF8(value.c_str());
}

wxString State_Label(Camera_2D_State state)
{
  switch (state)
  {
  case Camera_2D_State::Opened:
    return wxString::FromUTF8(u8"已打开");
  case Camera_2D_State::Grabbing:
    return wxString::FromUTF8(u8"采集中");
  case Camera_2D_State::Error:
    return wxString::FromUTF8(u8"错误");
  default:
    return wxString::FromUTF8(u8"未连接");
  }
}

int Trigger_Selection(jutze_camera::camera_trigger_mode mode)
{
  switch (mode)
  {
  case jutze_camera::camera_trigger_mode::soft_trigger:
    return 1;
  case jutze_camera::camera_trigger_mode::line_trigger:
    return 2;
  default:
    return 0;
  }
}

jutze_camera::camera_trigger_mode Trigger_Mode(int selection)
{
  switch (selection)
  {
  case 1:
    return jutze_camera::camera_trigger_mode::soft_trigger;
  case 2:
    return jutze_camera::camera_trigger_mode::line_trigger;
  default:
    return jutze_camera::camera_trigger_mode::auto_trigger;
  }
}
}

Camera_2D_Control_Panel::Camera_2D_Control_Panel(
  wxWindow *parent,
  Camera_2D_Service &camera_service)
  : wxScrolledWindow(parent, wxID_ANY),
    m_camera_service(camera_service)
{
  SetScrollRate(0, 12);

  auto *title = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"2D Camera"));
  auto title_font = title->GetFont();
  title_font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(title_font);

  auto *device_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"设备"));
  m_device_choice = new wxChoice(this, wxID_ANY);
  m_refresh_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"刷新设备"));
  m_open_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"打开相机"));
  m_close_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"关闭相机"));
  m_state_text = new wxStaticText(this, wxID_ANY, "");
  m_device_info_text = new wxStaticText(this, wxID_ANY, "");
  m_device_info_text->Wrap(350);
  auto *device_buttons = new wxBoxSizer(wxHORIZONTAL);
  device_buttons->Add(m_refresh_button, 1, wxRIGHT, 4);
  device_buttons->Add(m_open_button, 1, wxRIGHT, 4);
  device_buttons->Add(m_close_button, 1);
  device_box->Add(m_device_choice, 0, wxEXPAND | wxALL, 6);
  device_box->Add(device_buttons, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  device_box->Add(m_state_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  device_box->Add(m_device_info_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  auto *configuration_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"配置"));
  auto *configuration_grid = new wxFlexGridSizer(2, 6, 8);
  configuration_grid->AddGrowableCol(1, 1);
  configuration_grid->Add(
    new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"宽度")),
    0, wxALIGN_CENTER_VERTICAL);
  m_width_spin = new wxSpinCtrl(
    this, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 1, 65535, 1);
  configuration_grid->Add(m_width_spin, 1, wxEXPAND);
  configuration_grid->Add(
    new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"高度")),
    0, wxALIGN_CENTER_VERTICAL);
  m_height_spin = new wxSpinCtrl(
    this, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 1, 65535, 1);
  configuration_grid->Add(m_height_spin, 1, wxEXPAND);
  configuration_grid->Add(
    new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"触发模式")),
    0, wxALIGN_CENTER_VERTICAL);
  m_trigger_choice = new wxChoice(this, wxID_ANY);
  m_trigger_choice->Append(wxString::FromUTF8(u8"连续采集"));
  m_trigger_choice->Append(wxString::FromUTF8(u8"软触发"));
  m_trigger_choice->Append(wxString::FromUTF8(u8"硬触发（Line1上升沿）"));
  m_trigger_choice->SetSelection(0);
  configuration_grid->Add(m_trigger_choice, 1, wxEXPAND);
  auto *configuration_buttons = new wxBoxSizer(wxHORIZONTAL);
  m_apply_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"应用配置"));
  m_reload_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"读取当前值"));
  configuration_buttons->Add(m_apply_button, 1, wxRIGHT, 4);
  configuration_buttons->Add(m_reload_button, 1);
  configuration_box->Add(
    configuration_grid, 0, wxEXPAND | wxALL, 6);
  configuration_box->Add(
    configuration_buttons, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  auto *acquisition_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"采集"));
  auto *acquisition_buttons = new wxGridSizer(2, 2, 4, 4);
  m_start_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"开始采集"));
  m_stop_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"停止采集"));
  m_trigger_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"采集一帧"));
  m_save_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"保存当前图像"));
  acquisition_buttons->Add(m_start_button, 1, wxEXPAND);
  acquisition_buttons->Add(m_stop_button, 1, wxEXPAND);
  acquisition_buttons->Add(m_trigger_button, 1, wxEXPAND);
  acquisition_buttons->Add(m_save_button, 1, wxEXPAND);
  m_show_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"显示2D图像"));
  m_frame_info_text = new wxStaticText(this, wxID_ANY, "");
  acquisition_box->Add(
    acquisition_buttons, 0, wxEXPAND | wxALL, 6);
  acquisition_box->Add(
    m_show_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  acquisition_box->Add(
    m_frame_info_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxEXPAND | wxALL, 8);
  sizer->Add(device_box, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(
    configuration_box, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(
    acquisition_box, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->AddStretchSpacer(1);
  SetSizer(sizer);

  m_refresh_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Refresh, this);
  m_open_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Open, this);
  m_close_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Close, this);
  m_start_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Start, this);
  m_stop_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Stop, this);
  m_trigger_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Software_Trigger, this);
  m_apply_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Apply, this);
  m_reload_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Reload, this);
  m_save_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Save, this);
  m_show_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Control_Panel::On_Show_Image, this);

  m_timer.SetOwner(this);
  Bind(
    wxEVT_TIMER, &Camera_2D_Control_Panel::On_Timer,
    this, m_timer.GetId());
  m_timer.Start(250);
  Refresh_Device_List();
  Refresh_View();
}

void Camera_2D_Control_Panel::Set_On_Show_Image(
  std::function<void()> callback)
{
  m_on_show_image = std::move(callback);
}

void Camera_2D_Control_Panel::Refresh_Device_List()
{
  const auto previous =
    m_device_choice->GetSelection() == wxNOT_FOUND
      ? std::string()
      : m_device_keys[
          static_cast<std::size_t>(m_device_choice->GetSelection())];
  std::string message;
  m_camera_service.Refresh_Devices(&message);
  const auto devices = m_camera_service.Devices();
  m_device_choice->Clear();
  m_device_keys.clear();
  int selection = wxNOT_FOUND;
  for (const auto &device : devices)
  {
    const int index = static_cast<int>(m_device_keys.size());
    m_device_choice->Append(From_Utf8(device.display_name));
    m_device_keys.push_back(device.key);
    if (device.key == previous)
    {
      selection = index;
    }
  }
  if (selection == wxNOT_FOUND && !m_device_keys.empty())
  {
    selection = 0;
  }
  if (selection != wxNOT_FOUND)
  {
    m_device_choice->SetSelection(selection);
  }
  if (devices.empty())
  {
    m_state_text->SetLabel(From_Utf8(message));
  }
}

void Camera_2D_Control_Panel::Refresh_View()
{
  const auto status = m_camera_service.Status();
  const bool opened = m_camera_service.Is_Open();
  const bool grabbing = status.state == Camera_2D_State::Grabbing;
  m_device_choice->Enable(!opened);
  m_refresh_button->Enable(!opened);
  m_open_button->Enable(!opened && m_device_choice->GetSelection() != wxNOT_FOUND);
  m_close_button->Enable(opened);
  m_width_spin->Enable(opened);
  m_height_spin->Enable(opened);
  m_trigger_choice->Enable(opened);
  m_apply_button->Enable(opened);
  m_reload_button->Enable(opened);
  m_start_button->Enable(opened && !grabbing);
  m_stop_button->Enable(opened && grabbing);
  m_trigger_button->Enable(
    opened && grabbing &&
    status.trigger_mode == jutze_camera::camera_trigger_mode::soft_trigger);
  m_save_button->Enable(
    static_cast<bool>(m_camera_service.Latest_Frame()));
  m_show_button->Enable(opened);

  m_state_text->SetLabel(
    wxString::FromUTF8(u8"状态：") + State_Label(status.state));
  if (opened)
  {
    m_device_info_text->SetLabel(wxString::Format(
      wxString::FromUTF8(
        u8"型号：%s\n序列号：%s\n像素格式：%s"),
      From_Utf8(status.device_name),
      From_Utf8(status.serial_number),
      wxString::FromUTF8(
        jutze_camera::pixel_format_name(status.pixel_format))));
  }
  else
  {
    m_device_info_text->SetLabel(
      wxString::FromUTF8(u8"尚未打开2D相机"));
  }
  const auto frame = m_camera_service.Latest_Frame();
  if (frame)
  {
    m_frame_info_text->SetLabel(wxString::Format(
      wxString::FromUTF8(
        u8"%u × %u  Frame=%llu  %.1f FPS"),
      frame->m_width,
      frame->m_height,
      frame->m_frame_num,
      status.frames_per_second));
  }
  else
  {
    m_frame_info_text->SetLabel(
      grabbing
        ? wxString::FromUTF8(u8"等待图像…")
        : wxString::FromUTF8(u8"暂无图像"));
  }
}

void Camera_2D_Control_Panel::Sync_Configuration_Editors()
{
  const auto status = m_camera_service.Status();
  if (!m_camera_service.Is_Open())
  {
    return;
  }
  m_width_spin->SetValue(static_cast<int>(status.width));
  m_height_spin->SetValue(static_cast<int>(status.height));
  m_trigger_choice->SetSelection(Trigger_Selection(status.trigger_mode));
}

void Camera_2D_Control_Panel::Show_Error(const std::string &message)
{
  wxMessageBox(
    From_Utf8(message),
    wxString::FromUTF8(u8"2D相机"),
    wxOK | wxICON_ERROR,
    this);
}

void Camera_2D_Control_Panel::On_Refresh(wxCommandEvent &)
{
  Refresh_Device_List();
  Refresh_View();
}

void Camera_2D_Control_Panel::On_Open(wxCommandEvent &)
{
  const int selection = m_device_choice->GetSelection();
  if (selection == wxNOT_FOUND ||
      static_cast<std::size_t>(selection) >= m_device_keys.size())
  {
    Show_Error("请选择要打开的华睿相机");
    return;
  }
  std::string error;
  if (!m_camera_service.Open(
        m_device_keys[static_cast<std::size_t>(selection)], &error))
  {
    Show_Error(error);
    Refresh_View();
    return;
  }
  Sync_Configuration_Editors();
  Refresh_View();
}

void Camera_2D_Control_Panel::On_Close(wxCommandEvent &)
{
  std::string error;
  if (!m_camera_service.Close(&error))
  {
    Show_Error(error);
  }
  Refresh_View();
}

void Camera_2D_Control_Panel::On_Start(wxCommandEvent &)
{
  std::string error;
  if (!m_camera_service.Start(&error))
  {
    Show_Error(error);
  }
  else if (m_on_show_image)
  {
    m_on_show_image();
  }
  Refresh_View();
}

void Camera_2D_Control_Panel::On_Stop(wxCommandEvent &)
{
  std::string error;
  if (!m_camera_service.Stop(&error))
  {
    Show_Error(error);
  }
  Refresh_View();
}

void Camera_2D_Control_Panel::On_Software_Trigger(wxCommandEvent &)
{
  std::string error;
  if (!m_camera_service.Software_Trigger(&error))
  {
    Show_Error(error);
  }
  else if (m_on_show_image)
  {
    m_on_show_image();
  }
}

void Camera_2D_Control_Panel::On_Apply(wxCommandEvent &)
{
  std::string error;
  if (!m_camera_service.Apply_Configuration(
        static_cast<unsigned int>(m_width_spin->GetValue()),
        static_cast<unsigned int>(m_height_spin->GetValue()),
        Trigger_Mode(m_trigger_choice->GetSelection()),
        &error))
  {
    Show_Error(error);
  }
  Sync_Configuration_Editors();
  Refresh_View();
}

void Camera_2D_Control_Panel::On_Reload(wxCommandEvent &)
{
  Sync_Configuration_Editors();
  Refresh_View();
}

void Camera_2D_Control_Panel::On_Save(wxCommandEvent &)
{
  const auto frame = m_camera_service.Latest_Frame();
  if (!frame)
  {
    Show_Error("当前没有可保存的2D图像");
    return;
  }
  Camera_2D_Display_Image converted;
  std::string error;
  if (!Convert_Camera_2D_Frame(*frame, &converted, &error))
  {
    Show_Error(error);
    return;
  }
  wxFileDialog dialog(
    this,
    wxString::FromUTF8(u8"保存2D图像"),
    wxEmptyString,
    wxString::Format("frame_%llu.png", frame->m_frame_num),
    wxString::FromUTF8(
      u8"PNG图像 (*.png)|*.png|BMP图像 (*.bmp)|*.bmp|JPEG图像 (*.jpg)|*.jpg"),
    wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
  if (dialog.ShowModal() != wxID_OK)
  {
    return;
  }
  wxImage image(
    static_cast<int>(converted.width),
    static_cast<int>(converted.height));
  if (!image.IsOk() || !image.GetData())
  {
    Show_Error("创建保存图像失败");
    return;
  }
  std::memcpy(
    image.GetData(), converted.rgb.data(), converted.rgb.size());
  if (!image.SaveFile(dialog.GetPath()))
  {
    Show_Error("保存2D图像失败，请检查文件格式和保存路径");
  }
}

void Camera_2D_Control_Panel::On_Show_Image(wxCommandEvent &)
{
  if (m_on_show_image)
  {
    m_on_show_image();
  }
}

void Camera_2D_Control_Panel::On_Timer(wxTimerEvent &)
{
  Refresh_View();
}
