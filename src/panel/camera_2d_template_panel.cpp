#include "camera_2d_template_panel.h"

#include "camera_2d_image_converter.h"
#include "camera_2d_image_view.h"
#include "camera_2d_service.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/filedlg.h>
#include <wx/filename.h>
#include <wx/grid.h>
#include <wx/image.h>
#include <wx/listctrl.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <algorithm>
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
  m_confirm_roi_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"确认ROI"));
  m_cancel_roi_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"取消ROI"));
  m_read_image_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"读取图片"));
  m_resume_live_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"恢复实时图像"));
  auto *roi_button_sizer = new wxBoxSizer(wxHORIZONTAL);
  roi_button_sizer->Add(m_confirm_roi_button, 1, wxRIGHT, 4);
  roi_button_sizer->Add(m_cancel_roi_button, 1);
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
    roi_button_sizer,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    6);
  auto *image_button_sizer = new wxBoxSizer(wxHORIZONTAL);
  image_button_sizer->Add(m_read_image_button, 1, wxRIGHT, 4);
  image_button_sizer->Add(m_resume_live_button, 1);
  creation_box->Add(
    image_button_sizer,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    6);
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
  m_template_list->EnableCheckBoxes(true);
  m_template_list->AppendColumn(
    wxString::FromUTF8(u8"模板名称"), wxLIST_FORMAT_LEFT, 230);
  m_template_list->AppendColumn(
    wxString::FromUTF8(u8"模板类型"), wxLIST_FORMAT_LEFT, 110);
  m_delete_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"删除选中模板"));
  m_edit_roi_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"编辑ROI"));
  auto *list_button_sizer = new wxBoxSizer(wxHORIZONTAL);
  list_button_sizer->Add(m_edit_roi_button, 1, wxRIGHT, 4);
  list_button_sizer->Add(m_delete_button, 1);
  list_box->Add(m_template_list, 1, wxEXPAND | wxALL, 6);
  list_box->Add(
    list_button_sizer,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    6);

  auto *information_box = new wxStaticBoxSizer(
    wxVERTICAL, this, wxString::FromUTF8(u8"当前模板匹配信息"));
  m_match_information_grid = new wxGrid(this, wxID_ANY);
  m_match_information_grid->CreateGrid(4, 2);
  m_match_information_grid->EnableEditing(false);
  m_match_information_grid->EnableGridLines(true);
  m_match_information_grid->SetRowLabelSize(0);
  m_match_information_grid->SetColLabelSize(0);
  m_match_information_grid->DisableDragRowSize();
  m_match_information_grid->DisableDragColSize();
  m_match_information_grid->DisableDragGridSize();
  m_match_information_grid->SetDefaultRowSize(28, true);
  m_match_information_grid->SetColSize(0, 105);
  m_match_information_grid->SetColSize(1, 245);
  const std::array<wxString, 4> information_labels{{
    wxString::FromUTF8(u8"匹配状态"),
    wxString::FromUTF8(u8"匹配度"),
    wxString::FromUTF8(u8"方向角"),
    wxString::FromUTF8(u8"中心(px)")}};
  for (int row = 0; row < 4; ++row)
  {
    m_match_information_grid->SetCellValue(
      row, 0, information_labels[static_cast<std::size_t>(row)]);
    m_match_information_grid->SetCellValue(
      row, 1, wxString::FromUTF8(u8"—"));
    m_match_information_grid->SetCellBackgroundColour(
      row, 0, wxColour(240, 240, 240));
    auto label_font = m_match_information_grid->GetCellFont(row, 0);
    label_font.SetWeight(wxFONTWEIGHT_BOLD);
    m_match_information_grid->SetCellFont(row, 0, label_font);
    m_match_information_grid->SetReadOnly(row, 0, true);
    m_match_information_grid->SetReadOnly(row, 1, true);
  }
  m_match_information_grid->SetMinSize(wxSize(-1, 114));
  information_box->Add(
    m_match_information_grid,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    6);

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
  m_template_list->Bind(
    wxEVT_LIST_ITEM_CHECKED,
    &Camera_2D_Template_Panel::On_Template_Check_Changed,
    this);
  m_template_list->Bind(
    wxEVT_LIST_ITEM_UNCHECKED,
    &Camera_2D_Template_Panel::On_Template_Check_Changed,
    this);
  m_delete_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Template_Panel::On_Delete, this);
  m_edit_roi_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Template_Panel::On_Edit_Roi, this);
  m_confirm_roi_button->Bind(
    wxEVT_BUTTON,
    &Camera_2D_Template_Panel::On_Confirm_Roi,
    this);
  m_cancel_roi_button->Bind(
    wxEVT_BUTTON,
    &Camera_2D_Template_Panel::On_Cancel_Roi,
    this);
  m_read_image_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Template_Panel::On_Read_Image, this);
  m_resume_live_button->Bind(
    wxEVT_BUTTON, &Camera_2D_Template_Panel::On_Resume_Live, this);
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
  const auto templates = m_template_service.Templates();
  std::unordered_set<std::string> existing_ids;
  for (const auto &item : templates) existing_ids.insert(item.id);
  for (auto iterator = m_visible_template_ids.begin();
       iterator != m_visible_template_ids.end();)
  {
    if (existing_ids.find(*iterator) == existing_ids.end())
      iterator = m_visible_template_ids.erase(iterator);
    else
      ++iterator;
  }
  // Matching starts with no template selected. Users explicitly check the
  // templates they want to run against the live or loaded image.
  m_refreshing_template_list = true;
  m_template_list->Freeze();
  m_template_list->DeleteAllItems();
  m_template_ids.clear();
  for (std::size_t index = 0; index < templates.size(); ++index)
  {
    const long row = m_template_list->InsertItem(
      static_cast<long>(index), From_Utf8(templates[index].name));
    m_template_list->SetItem(
      row, 1, wxString::FromUTF8(u8"十字模板"));
    m_template_list->CheckItem(
      row,
      m_visible_template_ids.find(templates[index].id) !=
        m_visible_template_ids.end());
    m_template_ids.push_back(templates[index].id);
  }
  m_template_list->Thaw();
  m_refreshing_template_list = false;
  Update_Matching_Selection();
}

void Camera_2D_Template_Panel::Refresh_View()
{
  const bool connected = m_camera_service.Is_Open();
  const bool has_frame =
    static_cast<bool>(m_camera_service.Latest_Frame()) ||
    m_loaded_image.has_value();
  m_type_choice->Enable(connected && !m_roi_editing);
  m_name_text->Enable(connected && !m_roi_editing);
  m_create_button->Enable(
    connected && has_frame && !m_roi_editing);
  m_confirm_roi_button->Enable(
    connected &&
    m_roi_editing &&
    m_image_view.Has_Editable_Template_Roi());
  m_cancel_roi_button->Enable(m_roi_editing);
  m_read_image_button->Enable(!m_roi_editing);
  m_resume_live_button->Enable(
    m_loaded_image.has_value() && !m_roi_editing);
  m_template_list->Enable(
    connected && !m_template_ids.empty() && !m_roi_editing);
  const long selection = m_template_list->GetNextItem(
    -1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
  m_delete_button->Enable(
    connected && selection != -1 && !m_roi_editing);
  m_edit_roi_button->Enable(
    connected && has_frame && selection != -1 && !m_roi_editing);
  const auto active = m_template_service.Active_Template();
  const auto set_information =
    [this](
      const wxString &status,
      const wxString &confidence,
      const wxString &angle,
      const wxString &center)
    {
      m_match_information_grid->SetCellValue(0, 1, status);
      m_match_information_grid->SetCellValue(1, 1, confidence);
      m_match_information_grid->SetCellValue(2, 1, angle);
      m_match_information_grid->SetCellValue(3, 1, center);
    };
  const wxString empty = wxString::FromUTF8(u8"—");
  if (!connected && !m_loaded_image)
  {
    set_information(
      wxString::FromUTF8(u8"请先连接2D相机"),
      empty,
      empty,
      empty);
    return;
  }
  const auto detection = m_template_service.Latest_Detection();
  if (!active)
  {
    set_information(
      wxString::FromUTF8(u8"请选择模板"),
      empty,
      empty,
      empty);
    return;
  }
  if (m_visible_template_ids.find(active->id) ==
      m_visible_template_ids.end())
  {
    set_information(
      wxString::FromUTF8(u8"未勾选"),
      empty,
      empty,
      empty);
    return;
  }
  if (detection && detection->template_id == active->id)
  {
    if (detection->found)
    {
      set_information(
        wxString::FromUTF8(u8"匹配成功"),
        wxString::Format("%.0f%%", detection->confidence * 100.0),
        wxString::Format("%.0f", detection->angle_deg) +
          wxString::FromUTF8(u8"°"),
        wxString::Format(
          "(%.0f,%.0f)",
          detection->center_x,
          detection->center_y));
    }
    else
    {
      set_information(
        wxString::FromUTF8(u8"匹配失败"),
        wxString::Format("%.0f%%", detection->confidence * 100.0),
        empty,
        empty);
    }
  }
  else
  {
    set_information(
      wxString::FromUTF8(u8"等待匹配"),
      empty,
      empty,
      empty);
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

void Camera_2D_Template_Panel::On_Template_Check_Changed(
  wxListEvent &event)
{
  if (m_refreshing_template_list) return;
  const long row = event.GetIndex();
  if (row < 0 ||
      static_cast<std::size_t>(row) >= m_template_ids.size())
    return;
  const auto &template_id =
    m_template_ids[static_cast<std::size_t>(row)];
  if (m_template_list->IsItemChecked(row))
    m_visible_template_ids.insert(template_id);
  else
    m_visible_template_ids.erase(template_id);
  Update_Matching_Selection();
  Refresh_View();
}

void Camera_2D_Template_Panel::Update_Matching_Selection()
{
  std::vector<std::string> template_ids;
  for (const auto &template_id : m_template_ids)
  {
    if (m_visible_template_ids.find(template_id) !=
        m_visible_template_ids.end())
      template_ids.push_back(template_id);
  }
  m_image_view.Set_Matched_Template_Ids(template_ids);
  if (m_loaded_image)
  {
    m_image_view.Show_Static_Image(
      *m_loaded_image,
      template_ids,
      m_loaded_image_name);
  }
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

void Camera_2D_Template_Panel::On_Edit_Roi(wxCommandEvent &)
{
  const long selection = m_template_list->GetNextItem(
    -1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
  if (selection < 0 ||
      static_cast<std::size_t>(selection) >= m_template_ids.size())
    return;
  const auto frame = m_camera_service.Latest_Frame();
  if (!frame && !m_loaded_image)
  {
    Show_Error("请先采集一帧图像再编辑ROI");
    return;
  }
  const std::string template_id =
    m_template_ids[static_cast<std::size_t>(selection)];
  const auto templates = m_template_service.Templates();
  const auto found = std::find_if(
    templates.begin(), templates.end(),
    [&template_id](const auto &item)
    {
      return item.id == template_id;
    });
  if (found == templates.end())
  {
    Show_Error("选中的2D模板不存在");
    Refresh_Template_List();
    return;
  }
  m_template_service.Set_Active(template_id, nullptr);
  m_creation_frame = m_loaded_image ? nullptr : frame;
  m_editing_template_id = template_id;
  m_roi_editing = true;
  if (m_on_show_image) m_on_show_image();
  const std::string name = found->name;
  m_instruction_text->SetLabel(
    wxString::FromUTF8(
      u8"正在编辑已有模板ROI：拖动框内部移动，拖动红色边框或控制点"
      u8"调整大小；完成后点击“确认ROI”。"));
  m_image_view.Begin_Template_Roi_Editing(
    found->roi,
    [this, name](Camera_2D_Roi roi)
    {
      On_Roi_Selected(name, roi);
    });
  Refresh_View();
}

void Camera_2D_Template_Panel::On_Confirm_Roi(wxCommandEvent &)
{
  if (!m_roi_editing ||
      !m_image_view.Has_Editable_Template_Roi())
    return;
  m_roi_editing = false;
  m_image_view.Confirm_Template_Roi_Selection();
  Refresh_View();
}

void Camera_2D_Template_Panel::On_Cancel_Roi(wxCommandEvent &)
{
  if (!m_roi_editing) return;
  m_image_view.Cancel_Template_Roi_Selection();
  m_creation_frame.reset();
  m_editing_template_id.clear();
  m_roi_editing = false;
  m_instruction_text->SetLabel(
    wxString::FromUTF8(u8"已取消ROI编辑，可重新点击“新增”。"));
  Refresh_View();
}

void Camera_2D_Template_Panel::On_Read_Image(wxCommandEvent &)
{
  wxFileDialog dialog(
    this,
    wxString::FromUTF8(u8"读取2D图片并进行模板匹配"),
    wxEmptyString,
    wxEmptyString,
    wxString::FromUTF8(
      u8"图片文件 (*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff)|"
      u8"*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff|"
      u8"所有文件 (*.*)|*.*"),
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if (dialog.ShowModal() != wxID_OK) return;
  wxImage loaded(dialog.GetPath());
  if (!loaded.IsOk() || !loaded.GetData())
  {
    Show_Error("读取图片失败或图片格式不受支持");
    return;
  }
  Camera_2D_Display_Image image;
  image.width = static_cast<unsigned int>(loaded.GetWidth());
  image.height = static_cast<unsigned int>(loaded.GetHeight());
  const std::size_t byte_count =
    static_cast<std::size_t>(image.width) * image.height * 3;
  image.rgb.assign(loaded.GetData(), loaded.GetData() + byte_count);
  m_loaded_image = std::move(image);
  m_loaded_image_name =
    wxFileName(dialog.GetPath()).GetFullName().ToUTF8().data();
  Update_Matching_Selection();
  m_instruction_text->SetLabel(
    wxString::FromUTF8(
      u8"图片读取成功，已使用列表中勾选的模板进行匹配。"));
  if (m_on_show_image) m_on_show_image();
  Refresh_View();
}

void Camera_2D_Template_Panel::On_Resume_Live(wxCommandEvent &)
{
  m_loaded_image.reset();
  m_loaded_image_name.clear();
  m_image_view.Resume_Live_Image();
  m_instruction_text->SetLabel(
    wxString::FromUTF8(u8"已恢复实时相机图像匹配。"));
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
  if (!m_camera_service.Latest_Frame() && !m_loaded_image)
  {
    Show_Error("请先采集一幅包含完整十字的图片");
    return;
  }
  m_creation_frame = m_loaded_image
    ? nullptr
    : m_camera_service.Latest_Frame();
  m_editing_template_id.clear();
  m_roi_editing = true;
  if (m_on_show_image) m_on_show_image();
  const std::string utf8_name = name.ToUTF8().data();
  m_instruction_text->SetLabel(
    wxString::FromUTF8(
      u8"绘制ROI后，可拖动框内部移动，拖动边框或控制点调整大小；"
      u8"完成后点击“确认ROI”。"));
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
  m_roi_editing = false;
  const std::string editing_template_id =
    std::move(m_editing_template_id);
  m_editing_template_id.clear();
  const auto frame = std::move(m_creation_frame);
  if (!frame && !m_loaded_image)
  {
    Show_Error("框选完成时2D图片已经失效");
    return;
  }
  Camera_2D_Display_Image image;
  std::string error;
  if (m_loaded_image)
  {
    image = *m_loaded_image;
  }
  else if (!Convert_Camera_2D_Frame(*frame, &image, &error))
  {
    Show_Error(error);
    return;
  }
  std::string created_id;
  const bool updating = !editing_template_id.empty();
  const bool saved = updating
    ? m_template_service.Update(
        editing_template_id, image, roi, &error)
    : m_template_service.Create(
        name, image, roi, &created_id, &error);
  if (!saved)
  {
    Show_Error(error);
    return;
  }
  m_instruction_text->SetLabel(
    updating
      ? wxString::FromUTF8(
          u8"识别成功，选中模板的ROI和标准模板图已更新。")
      : wxString::FromUTF8(
          u8"识别成功，模板已添加到下方2D模板列表。"));
  if (!updating) m_name_text->Clear();
  Refresh_Template_List();
  Refresh_View();
}

void Camera_2D_Template_Panel::On_Timer(wxTimerEvent &)
{
  Refresh_View();
}
