#include "camera_2d_template_panel.h"

#include "camera_2d_image_converter.h"
#include "camera_2d_image_view.h"
#include "camera_2d_service.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/listctrl.h>
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
    this, wxID_ANY, wxString::FromUTF8(u8"2D模板制作"));
  auto font = title->GetFont();
  font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(font);

  auto *creation_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"新增2D模板"));
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
    this, wxID_ANY, wxString::FromUTF8(u8"新增"));
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

  auto *list_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"2D模板列表"));
  m_template_list = new wxListCtrl(
    this,
    wxID_ANY,
    wxDefaultPosition,
    wxSize(-1, 180),
    wxLC_REPORT | wxLC_SINGLE_SEL | wxLC_HRULES | wxLC_VRULES);
  m_template_list->AppendColumn(
    wxString::FromUTF8(u8"模板名称"), wxLIST_FORMAT_LEFT, 230);
  m_template_list->AppendColumn(
    wxString::FromUTF8(u8"模板类型"), wxLIST_FORMAT_LEFT, 110);
  m_delete_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"删除选中模板"));
  list_box->Add(m_template_list, 1, wxEXPAND | wxALL, 6);
  list_box->Add(
    m_delete_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  auto *information_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"当前模板匹配信息"));
  auto *information_grid = new wxFlexGridSizer(2, 5, 10);
  information_grid->AddGrowableCol(1, 1);
  const auto add_information_row =
    [this, information_grid](
      const wxString &label,
      wxStaticText **value)
    {
      information_grid->Add(
        new wxStaticText(this, wxID_ANY, label),
        0,
        wxALIGN_CENTER_VERTICAL);
      *value = new wxStaticText(
        this, wxID_ANY, wxString::FromUTF8(u8"—"));
      information_grid->Add(*value, 1, wxEXPAND);
    };
  add_information_row(
    wxString::FromUTF8(u8"模板名称"), &m_info_name);
  add_information_row(
    wxString::FromUTF8(u8"模板类型"), &m_info_type);
  add_information_row(
    wxString::FromUTF8(u8"匹配状态"), &m_info_status);
  add_information_row(
    wxString::FromUTF8(u8"匹配度"), &m_info_confidence);
  add_information_row(
    wxString::FromUTF8(u8"方向角"), &m_info_angle);
  add_information_row(
    wxString::FromUTF8(u8"匹配区域"), &m_info_roi);
  information_box->Add(information_grid, 0, wxEXPAND | wxALL, 6);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxEXPAND | wxALL, 8);
  sizer->Add(creation_box, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(list_box, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(
    information_box, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->AddStretchSpacer(1);
  SetSizer(sizer);

  m_create_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Template_Panel::On_Create, this);
  m_template_list->Bind(
    wxEVT_LIST_ITEM_SELECTED,
    &Camera_2D_Template_Panel::On_Template_Selected,
    this);
  m_delete_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Template_Panel::On_Delete, this);
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
  m_template_list->Freeze();
  m_template_list->DeleteAllItems();
  m_template_ids.clear();
  for (std::size_t index = 0; index < templates.size(); ++index)
  {
    const long row = m_template_list->InsertItem(
      static_cast<long>(index), From_Utf8(templates[index].name));
    m_template_list->SetItem(
      row, 1, wxString::FromUTF8(u8"十字模板"));
    m_template_ids.push_back(templates[index].id);
    if (active && active->id == templates[index].id)
    {
      m_template_list->SetItemState(
        row,
        wxLIST_STATE_SELECTED | wxLIST_STATE_FOCUSED,
        wxLIST_STATE_SELECTED | wxLIST_STATE_FOCUSED);
      m_template_list->EnsureVisible(row);
    }
  }
  m_template_list->Thaw();
}

void Camera_2D_Template_Panel::Refresh_View()
{
  const bool connected = m_camera_service.Is_Open();
  const bool has_frame =
    static_cast<bool>(m_camera_service.Latest_Frame());
  m_type_choice->Enable(connected);
  m_name_text->Enable(connected);
  m_create_button->Enable(connected && has_frame);
  m_template_list->Enable(connected && !m_template_ids.empty());
  const long selection = m_template_list->GetNextItem(
    -1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
  m_delete_button->Enable(connected && selection != -1);
  const auto active = m_template_service.Active_Template();
  m_info_name->SetLabel(
    active ? From_Utf8(active->name) : wxString::FromUTF8(u8"—"));
  m_info_type->SetLabel(
    active
      ? wxString::FromUTF8(u8"十字模板")
      : wxString::FromUTF8(u8"—"));
  if (!connected)
  {
    m_info_status->SetLabel(
      wxString::FromUTF8(u8"请先连接2D相机"));
    m_info_confidence->SetLabel(wxString::FromUTF8(u8"—"));
    m_info_angle->SetLabel(wxString::FromUTF8(u8"—"));
    m_info_roi->SetLabel(wxString::FromUTF8(u8"—"));
    return;
  }
  const auto detection = m_template_service.Latest_Detection();
  if (!active)
  {
    m_info_status->SetLabel(
      wxString::FromUTF8(u8"请选择一个模板"));
    m_info_confidence->SetLabel(wxString::FromUTF8(u8"—"));
    m_info_angle->SetLabel(wxString::FromUTF8(u8"—"));
    m_info_roi->SetLabel(wxString::FromUTF8(u8"—"));
    return;
  }
  if (detection && detection->template_id == active->id)
  {
    m_info_status->SetLabel(
      detection->found
        ? wxString::FromUTF8(u8"匹配成功")
        : wxString::FromUTF8(u8"未匹配"));
    m_info_confidence->SetLabel(wxString::Format(
      "%.1f%%", detection->confidence * 100.0));
    m_info_angle->SetLabel(
      wxString::Format("%.2f ", detection->angle_deg) +
      wxString::FromUTF8(u8"°"));
    m_info_roi->SetLabel(wxString::Format(
      "X=%d  Y=%d  W=%d  H=%d px",
      detection->search_roi.x,
      detection->search_roi.y,
      detection->search_roi.width,
      detection->search_roi.height));
  }
  else
  {
    m_info_status->SetLabel(
      wxString::FromUTF8(u8"等待当前图像匹配"));
    m_info_confidence->SetLabel(wxString::FromUTF8(u8"—"));
    m_info_angle->SetLabel(wxString::FromUTF8(u8"—"));
    m_info_roi->SetLabel(wxString::FromUTF8(u8"—"));
  }
}

void Camera_2D_Template_Panel::Show_Error(const std::string &message)
{
  wxMessageBox(
    From_Utf8(message),
    wxString::FromUTF8(u8"2D模板制作"),
    wxOK | wxICON_ERROR,
    this);
}

void Camera_2D_Template_Panel::On_Template_Selected(wxListEvent &event)
{
  const long selection = event.GetIndex();
  if (selection < 0 ||
      static_cast<std::size_t>(selection) >= m_template_ids.size())
    return;
  std::string error;
  if (!m_template_service.Set_Active(
        m_template_ids[static_cast<std::size_t>(selection)], &error))
    Show_Error(error);
}

void Camera_2D_Template_Panel::On_Delete(wxCommandEvent &)
{
  const long selection = m_template_list->GetNextItem(
    -1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
  if (selection < 0 ||
      static_cast<std::size_t>(selection) >= m_template_ids.size())
    return;
  if (wxMessageBox(
        wxString::FromUTF8(u8"确定删除选中的2D模板吗？"),
        wxString::FromUTF8(u8"删除2D模板"),
        wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION,
        this) != wxYES)
    return;
  std::string error;
  if (!m_template_service.Remove(
        m_template_ids[static_cast<std::size_t>(selection)], &error))
  {
    Show_Error(error);
    return;
  }
  Refresh_Template_List();
  Refresh_View();
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
    wxString::FromUTF8(
      u8"请在2D图片中绘制ROI；完成后将立即识别并创建模板。"));
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
    wxString::FromUTF8(
      u8"识别成功，模板已添加到下方2D模板列表。"));
  m_name_text->Clear();
  Refresh_Template_List();
  Refresh_View();
}

void Camera_2D_Template_Panel::On_Timer(wxTimerEvent &)
{
  Refresh_View();
}
