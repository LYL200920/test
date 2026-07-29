#include "camera_2d_template_panel.h"

#include "camera_2d_image_converter.h"
#include "camera_2d_image_view.h"
#include "camera_2d_service.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <utility>

namespace
{
wxString From_Utf8(const std::string &value)
{
  return wxString::FromUTF8(value.c_str());
}
}

Camera_2D_Template_Panel::Camera_2D_Template_Panel(
  wxWindow *parent,
  Camera_2D_Service &camera_service,
  Camera_2D_Cross_Template_Service &template_service,
  Camera_2D_Image_View &image_view)
  : wxScrolledWindow(parent, wxID_ANY),
    m_camera_service(camera_service),
    m_template_service(template_service),
    m_image_view(image_view)
{
  SetScrollRate(0, 12);
  auto *title = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"模板制作"));
  auto font = title->GetFont();
  font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(font);

  auto *creation_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"制作模板"));
  auto *grid = new wxFlexGridSizer(2, 6, 8);
  grid->AddGrowableCol(1, 1);
  grid->Add(
    new wxStaticText(
      this, wxID_ANY, wxString::FromUTF8(u8"模板类型")),
    0, wxALIGN_CENTER_VERTICAL);
  m_type_choice = new wxChoice(this, wxID_ANY);
  m_type_choice->Append(wxString::FromUTF8(u8"十字模板"));
  m_type_choice->SetSelection(0);
  grid->Add(m_type_choice, 1, wxEXPAND);
  grid->Add(
    new wxStaticText(
      this, wxID_ANY, wxString::FromUTF8(u8"模板名称")),
    0, wxALIGN_CENTER_VERTICAL);
  m_name_text = new wxTextCtrl(this, wxID_ANY);
  grid->Add(m_name_text, 1, wxEXPAND);
  m_create_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"框选ROI并制作"));
  m_instruction_text = new wxStaticText(
    this,
    wxID_ANY,
    wxString::FromUTF8(
      u8"选择“十字模板”，输入名称，然后在2D图像中框选完整十字。"));
  m_instruction_text->Wrap(350);
  creation_box->Add(grid, 0, wxEXPAND | wxALL, 6);
  creation_box->Add(
    m_create_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  creation_box->Add(
    m_instruction_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  auto *result_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"识别结果"));
  m_template_choice = new wxChoice(this, wxID_ANY);
  m_result_text = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"尚未识别十字"));
  result_box->Add(m_template_choice, 0, wxEXPAND | wxALL, 6);
  result_box->Add(
    m_result_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxEXPAND | wxALL, 8);
  sizer->Add(creation_box, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(result_box, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->AddStretchSpacer(1);
  SetSizer(sizer);

  m_create_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Template_Panel::On_Create, this);
  m_template_choice->Bind(
    wxEVT_CHOICE,
    &Camera_2D_Template_Panel::On_Template_Selected,
    this);
  m_timer.SetOwner(this);
  Bind(
    wxEVT_TIMER, &Camera_2D_Template_Panel::On_Timer,
    this, m_timer.GetId());
  m_timer.Start(200);
  Refresh_Template_List();
  Refresh_View();
}

void Camera_2D_Template_Panel::Set_On_Show_Image(
  std::function<void()> callback)
{
  m_on_show_image = std::move(callback);
}

void Camera_2D_Template_Panel::Refresh_Template_List()
{
  const auto active = m_template_service.Active_Template();
  const auto templates = m_template_service.Templates();
  m_template_choice->Clear();
  m_template_ids.clear();
  int selection = wxNOT_FOUND;
  for (std::size_t index = 0; index < templates.size(); ++index)
  {
    m_template_choice->Append(From_Utf8(templates[index].name));
    m_template_ids.push_back(templates[index].id);
    if (active && active->id == templates[index].id)
      selection = static_cast<int>(index);
  }
  if (selection == wxNOT_FOUND && !templates.empty()) selection = 0;
  if (selection != wxNOT_FOUND)
  {
    m_template_choice->SetSelection(selection);
    m_template_service.Set_Active(
      m_template_ids[static_cast<std::size_t>(selection)], nullptr);
  }
}

void Camera_2D_Template_Panel::Refresh_View()
{
  const bool connected = m_camera_service.Is_Open();
  const bool has_frame =
    static_cast<bool>(m_camera_service.Latest_Frame());
  m_type_choice->Enable(connected);
  m_name_text->Enable(connected);
  m_create_button->Enable(connected && has_frame);
  m_template_choice->Enable(connected && !m_template_ids.empty());
  if (!connected)
  {
    m_result_text->SetLabel(
      wxString::FromUTF8(u8"请先在2D相机页连接相机"));
    return;
  }
  const auto detection = m_template_service.Latest_Detection();
  if (detection && detection->found)
  {
    m_result_text->SetLabel(wxString::Format(
      wxString::FromUTF8(
        u8"十字中心（图片像素坐标系）\nX = %.2f px\nY = %.2f px"),
      detection->center_x,
      detection->center_y));
  }
  else
  {
    m_result_text->SetLabel(
      wxString::FromUTF8(u8"当前图片未识别到十字"));
  }
}

void Camera_2D_Template_Panel::Show_Error(const std::string &message)
{
  wxMessageBox(
    From_Utf8(message),
    wxString::FromUTF8(u8"模板制作"),
    wxOK | wxICON_ERROR,
    this);
}

void Camera_2D_Template_Panel::On_Template_Selected(wxCommandEvent &)
{
  const int selection = m_template_choice->GetSelection();
  if (selection == wxNOT_FOUND ||
      static_cast<std::size_t>(selection) >= m_template_ids.size())
    return;
  std::string error;
  if (!m_template_service.Set_Active(
        m_template_ids[static_cast<std::size_t>(selection)], &error))
    Show_Error(error);
}

void Camera_2D_Template_Panel::On_Create(wxCommandEvent &)
{
  if (!m_camera_service.Is_Open())
  {
    Show_Error("请先连接2D相机");
    return;
  }
  if (m_type_choice->GetSelection() != 0)
  {
    Show_Error("当前版本只支持制作十字模板");
    return;
  }
  wxString name = m_name_text->GetValue();
  name.Trim(true).Trim(false);
  if (name.empty())
  {
    Show_Error("请输入模板名称");
    return;
  }
  if (!m_camera_service.Latest_Frame())
  {
    Show_Error("请先采集一幅包含完整十字的图片");
    return;
  }
  m_creation_frame = m_camera_service.Latest_Frame();
  if (m_on_show_image) m_on_show_image();
  const std::string utf8_name = name.ToUTF8().data();
  m_instruction_text->SetLabel(
    wxString::FromUTF8(u8"请在2D图片中拖动鼠标，框选完整十字。"));
  m_image_view.Begin_Template_Roi_Selection(
    [this, utf8_name](Camera_2D_Roi roi)
    {
      On_Roi_Selected(utf8_name, roi);
    });
}

void Camera_2D_Template_Panel::On_Roi_Selected(
  const std::string &name,
  const Camera_2D_Roi &roi)
{
  const auto frame = std::move(m_creation_frame);
  if (!frame)
  {
    Show_Error("框选完成时2D图片已经失效");
    return;
  }
  Camera_2D_Display_Image image;
  std::string error;
  if (!Convert_Camera_2D_Frame(*frame, &image, &error))
  {
    Show_Error(error);
    return;
  }
  std::string created_id;
  if (!m_template_service.Create(
        name, image, roi, &created_id, &error))
  {
    Show_Error(error);
    return;
  }
  const auto detection = m_template_service.Detect(image);
  m_image_view.Show_Template_Detection(detection);
  m_instruction_text->SetLabel(
    wxString::FromUTF8(u8"十字识别成功，已在图片中描边并标出中心。"));
  Refresh_Template_List();
  Refresh_View();
}

void Camera_2D_Template_Panel::On_Timer(wxTimerEvent &)
{
  Refresh_View();
}
