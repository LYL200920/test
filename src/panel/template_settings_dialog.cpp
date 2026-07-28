#include "template_settings_dialog.h"

#include "pose_transform.h"

#include <wx/button.h>
#include <wx/grid.h>
#include <wx/listbox.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <algorithm>
#include <array>
#include <string>

namespace
{

wxString From_Utf8(const std::string &value)
{
  return wxString::FromUTF8(value.c_str());
}

wxString Trimmed(wxString value)
{
  value.Trim(true);
  value.Trim(false);
  return value;
}

} // namespace

Template_Settings_Dialog::Template_Settings_Dialog(
  wxWindow *parent,
  const robot_model::Template_Configuration &configuration)
  : wxDialog(
      parent,
      wxID_ANY,
      wxString::FromUTF8(u8"模板配置"),
      wxDefaultPosition,
      wxSize(820, 590),
      wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
    m_configuration(configuration)
{
  robot_model::Normalize_Template_Configuration(&m_configuration);

  m_template_list = new wxListBox(this, wxID_ANY);
  auto *new_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"新建"));
  m_copy_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"复制"));
  m_delete_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"删除"));

  auto *list_buttons = new wxBoxSizer(wxHORIZONTAL);
  list_buttons->Add(new_button, 1, wxRIGHT, 5);
  list_buttons->Add(m_copy_button, 1, wxRIGHT, 5);
  list_buttons->Add(m_delete_button, 1);

  auto *left = new wxBoxSizer(wxVERTICAL);
  left->Add(
    new wxStaticText(
      this, wxID_ANY, wxString::FromUTF8(u8"模板列表")),
    0, wxBOTTOM, 6);
  left->Add(m_template_list, 1, wxEXPAND | wxBOTTOM, 8);
  left->Add(list_buttons, 0, wxEXPAND);

  m_template_id = new wxStaticText(this, wxID_ANY, "-");
  m_template_name = new wxTextCtrl(this, wxID_ANY);
  auto *identity_grid = new wxFlexGridSizer(2, 6, 8);
  identity_grid->AddGrowableCol(1, 1);
  identity_grid->Add(
    new wxStaticText(this, wxID_ANY, "ID"),
    0, wxALIGN_CENTER_VERTICAL);
  identity_grid->Add(m_template_id, 1, wxEXPAND);
  identity_grid->Add(
    new wxStaticText(
      this, wxID_ANY, wxString::FromUTF8(u8"模板名称")),
    0, wxALIGN_CENTER_VERTICAL);
  identity_grid->Add(m_template_name, 1, wxEXPAND);

  const std::array<wxString, 6> pose_labels = {
    "X (mm)", "Y (mm)", "Z (mm)",
    wxString::FromUTF8(u8"A (°)"),
    wxString::FromUTF8(u8"B (°)"),
    wxString::FromUTF8(u8"C (°)")};
  auto *pose_grid = new wxFlexGridSizer(3, 4, 6, 8);
  pose_grid->AddGrowableCol(1, 1);
  pose_grid->AddGrowableCol(3, 1);
  for (std::size_t index = 0; index < pose_labels.size(); ++index)
  {
    pose_grid->Add(
      new wxStaticText(this, wxID_ANY, pose_labels[index]),
      0, wxALIGN_CENTER_VERTICAL);
    const double limit = index < 3 ? 1000000.0 : 3600.0;
    const double increment = index < 3 ? 1.0 : 0.1;
    m_pose_controls[index] = new wxSpinCtrlDouble(
      this,
      wxID_ANY,
      "0",
      wxDefaultPosition,
      wxDefaultSize,
      wxSP_ARROW_KEYS,
      -limit,
      limit,
      0.0,
      increment);
    m_pose_controls[index]->SetDigits(3);
    pose_grid->Add(m_pose_controls[index], 1, wxEXPAND);
  }

  m_matrix_grid = new wxGrid(this, wxID_ANY);
  m_matrix_grid->CreateGrid(4, 4);
  m_matrix_grid->EnableEditing(false);
  m_matrix_grid->SetRowLabelSize(0);
  m_matrix_grid->SetColLabelSize(0);
  m_matrix_grid->DisableDragGridSize();
  m_matrix_grid->DisableDragColSize();
  m_matrix_grid->DisableDragRowSize();
  m_matrix_grid->ShowScrollbars(wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
  m_matrix_grid->SetDefaultCellAlignment(
    wxALIGN_CENTER, wxALIGN_CENTER);
  for (int row = 0; row < 4; ++row)
  {
    for (int column = 0; column < 4; ++column)
    {
      m_matrix_grid->SetCellBackgroundColour(
        row, column, wxColour(248, 248, 248));
    }
  }
  m_matrix_grid->SetMinSize(wxSize(480, 180));

  auto *right = new wxBoxSizer(wxVERTICAL);
  right->Add(
    new wxStaticText(
      this, wxID_ANY, wxString::FromUTF8(u8"模板信息")),
    0, wxBOTTOM, 6);
  right->Add(identity_grid, 0, wxEXPAND | wxBOTTOM, 14);
  right->Add(
    new wxStaticText(
      this,
      wxID_ANY,
      wxString::FromUTF8(
        u8"参考抓取点（XYZABC，姿态按 ZYX 旋转）")),
    0, wxBOTTOM, 7);
  right->Add(pose_grid, 0, wxEXPAND | wxBOTTOM, 14);
  right->Add(
    new wxStaticText(
      this, wxID_ANY, wxString::FromUTF8(u8"参考点位姿矩阵")),
    0, wxBOTTOM, 7);
  right->Add(m_matrix_grid, 1, wxEXPAND);
  right->Add(
    new wxStaticText(
      this,
      wxID_ANY,
      wxString::FromUTF8(
        u8"提示：从3D相机识别结果获取参考点将在运行阶段接入。")),
    0, wxTOP, 10);

  auto *content = new wxBoxSizer(wxHORIZONTAL);
  content->Add(left, 1, wxEXPAND | wxRIGHT, 14);
  content->Add(right, 2, wxEXPAND);

  auto *save_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"保存"));
  auto *cancel_button = new wxButton(
    this, wxID_CANCEL, wxString::FromUTF8(u8"取消"));
  auto *actions = new wxBoxSizer(wxHORIZONTAL);
  actions->AddStretchSpacer(1);
  actions->Add(save_button, 0, wxRIGHT, 8);
  actions->Add(cancel_button, 0);

  auto *root = new wxBoxSizer(wxVERTICAL);
  root->Add(content, 1, wxEXPAND | wxALL, 12);
  root->Add(actions, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 12);
  SetSizer(root);
  SetMinSize(wxSize(650, 480));

  m_template_list->Bind(
    wxEVT_LISTBOX,
    &Template_Settings_Dialog::On_Selection_Changed,
    this);
  new_button->Bind(
    wxEVT_BUTTON, &Template_Settings_Dialog::On_New, this);
  m_copy_button->Bind(
    wxEVT_BUTTON, &Template_Settings_Dialog::On_Copy, this);
  m_delete_button->Bind(
    wxEVT_BUTTON, &Template_Settings_Dialog::On_Delete, this);
  save_button->Bind(
    wxEVT_BUTTON, &Template_Settings_Dialog::On_Save, this);
  for (wxSpinCtrlDouble *control : m_pose_controls)
  {
    control->Bind(
      wxEVT_SPINCTRLDOUBLE,
      [this](wxSpinDoubleEvent &) { Update_Matrix_Preview(); });
    control->Bind(
      wxEVT_TEXT,
      [this](wxCommandEvent &) { Update_Matrix_Preview(); });
  }
  m_matrix_grid->Bind(
    wxEVT_SIZE,
    [this](wxSizeEvent &event)
    {
      if (m_matrix_grid)
      {
        const wxSize client_size = m_matrix_grid->GetClientSize();
        const int width = std::max(240, client_size.x);
        const int height = std::max(120, client_size.y);
        const int column_width = std::max(58, (width - 4) / 4);
        const int row_height = std::max(28, (height - 4) / 4);
        for (int column = 0; column < 4; ++column)
        {
          m_matrix_grid->SetColSize(column, column_width);
        }
        for (int row = 0; row < 4; ++row)
        {
          m_matrix_grid->SetRowSize(row, row_height);
        }
      }
      event.Skip();
    });

  if (!m_configuration.templates.empty())
  {
    Refresh_List(0);
    Load_Editor(0);
  }
  else
  {
    Refresh_List(0);
    Update_Editor_State();
    Update_Matrix_Preview();
  }
  CentreOnParent();
}

const robot_model::Template_Configuration &
Template_Settings_Dialog::Configuration() const
{
  return m_configuration;
}

void Template_Settings_Dialog::On_Selection_Changed(wxCommandEvent &)
{
  const int selection = m_template_list->GetSelection();
  if (!Store_Editor())
  {
    if (m_has_editor)
    {
      m_template_list->SetSelection(
        static_cast<int>(m_editor_index));
    }
    return;
  }
  if (selection != wxNOT_FOUND)
  {
    m_template_list->SetSelection(selection);
    Load_Editor(static_cast<std::size_t>(selection));
  }
}

void Template_Settings_Dialog::On_New(wxCommandEvent &)
{
  if (!Store_Editor())
  {
    return;
  }
  robot_model::Template_Profile profile;
  profile.id = Make_Unique_Id();
  profile.name = Make_Unique_Name(
    wxString::FromUTF8(u8"新模板").ToUTF8().data());
  m_configuration.templates.push_back(profile);
  const std::size_t selection = m_configuration.templates.size() - 1;
  Refresh_List(selection);
  Load_Editor(selection);
  m_template_name->SetFocus();
  m_template_name->SelectAll();
}

void Template_Settings_Dialog::On_Copy(wxCommandEvent &)
{
  if (!Store_Editor() || !m_has_editor ||
      m_editor_index >= m_configuration.templates.size())
  {
    return;
  }
  robot_model::Template_Profile profile =
    m_configuration.templates[m_editor_index];
  profile.id = Make_Unique_Id();
  profile.name = Make_Unique_Name(
    profile.name + wxString::FromUTF8(u8" 副本").ToUTF8().data());
  m_configuration.templates.push_back(std::move(profile));
  const std::size_t selection = m_configuration.templates.size() - 1;
  Refresh_List(selection);
  Load_Editor(selection);
}

void Template_Settings_Dialog::On_Delete(wxCommandEvent &)
{
  if (!m_has_editor ||
      m_editor_index >= m_configuration.templates.size())
  {
    return;
  }
  if (wxMessageBox(
        wxString::FromUTF8(u8"确定删除当前模板吗？"),
        wxString::FromUTF8(u8"删除模板"),
        wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
        this) != wxYES)
  {
    return;
  }
  m_configuration.templates.erase(
    m_configuration.templates.begin() +
      static_cast<std::ptrdiff_t>(m_editor_index));
  if (m_configuration.templates.empty())
  {
    m_has_editor = false;
    m_editor_index = 0;
    Refresh_List(0);
    Update_Editor_State();
    Update_Matrix_Preview();
    return;
  }
  const std::size_t selection = std::min(
    m_editor_index, m_configuration.templates.size() - 1);
  Refresh_List(selection);
  Load_Editor(selection);
}

void Template_Settings_Dialog::On_Save(wxCommandEvent &)
{
  if (!Store_Editor())
  {
    return;
  }
  EndModal(wxID_OK);
}

void Template_Settings_Dialog::Load_Editor(std::size_t index)
{
  if (index >= m_configuration.templates.size())
  {
    return;
  }
  m_loading_editor = true;
  m_has_editor = true;
  m_editor_index = index;
  const auto &profile = m_configuration.templates[index];
  m_template_id->SetLabel(From_Utf8(profile.id));
  m_template_name->SetValue(From_Utf8(profile.name));
  for (std::size_t pose_index = 0;
       pose_index < m_pose_controls.size();
       ++pose_index)
  {
    m_pose_controls[pose_index]->SetValue(
      profile.reference_pose[pose_index]);
  }
  m_loading_editor = false;
  Update_Editor_State();
  Update_Matrix_Preview();
}

bool Template_Settings_Dialog::Store_Editor()
{
  if (!m_has_editor)
  {
    return true;
  }
  if (m_editor_index >= m_configuration.templates.size())
  {
    return false;
  }

  const wxString name = Trimmed(m_template_name->GetValue());
  if (name.empty())
  {
    wxMessageBox(
      wxString::FromUTF8(u8"模板名称不能为空。"),
      wxString::FromUTF8(u8"模板配置"),
      wxOK | wxICON_WARNING,
      this);
    m_template_name->SetFocus();
    return false;
  }
  for (std::size_t index = 0;
       index < m_configuration.templates.size();
       ++index)
  {
    if (index != m_editor_index &&
        From_Utf8(m_configuration.templates[index].name)
          .CmpNoCase(name) == 0)
    {
      wxMessageBox(
        wxString::FromUTF8(u8"模板名称不能重复。"),
        wxString::FromUTF8(u8"模板配置"),
        wxOK | wxICON_WARNING,
        this);
      m_template_name->SetFocus();
      return false;
    }
  }

  auto &profile = m_configuration.templates[m_editor_index];
  profile.name = name.ToUTF8().data();
  for (std::size_t index = 0; index < m_pose_controls.size(); ++index)
  {
    profile.reference_pose[index] = m_pose_controls[index]->GetValue();
  }
  Refresh_List(m_editor_index);
  return true;
}

void Template_Settings_Dialog::Refresh_List(std::size_t selection)
{
  m_template_list->Freeze();
  m_template_list->Clear();
  for (const auto &profile : m_configuration.templates)
  {
    m_template_list->Append(From_Utf8(profile.name));
  }
  if (selection < m_configuration.templates.size())
  {
    m_template_list->SetSelection(static_cast<int>(selection));
  }
  m_template_list->Thaw();
}

void Template_Settings_Dialog::Update_Editor_State()
{
  const bool enabled = m_has_editor &&
    m_editor_index < m_configuration.templates.size();
  m_template_id->SetLabel(enabled
    ? From_Utf8(m_configuration.templates[m_editor_index].id)
    : "-");
  m_template_name->Enable(enabled);
  if (!enabled)
  {
    m_template_name->ChangeValue("");
  }
  for (wxSpinCtrlDouble *control : m_pose_controls)
  {
    control->Enable(enabled);
    if (!enabled)
    {
      control->SetValue(0.0);
    }
  }
  m_copy_button->Enable(enabled);
  m_delete_button->Enable(enabled);
}

void Template_Settings_Dialog::Update_Matrix_Preview()
{
  if (!m_matrix_grid || m_loading_editor)
  {
    return;
  }
  robot_model::XyzabcPose pose = {};
  if (m_has_editor)
  {
    for (std::size_t index = 0; index < m_pose_controls.size(); ++index)
    {
      pose[index] = m_pose_controls[index]->GetValue();
    }
  }
  const robot_model::Matrix4 matrix =
    robot_model::Build_Zyx_Pose_Matrix(pose);
  for (int row = 0; row < 4; ++row)
  {
    for (int column = 0; column < 4; ++column)
    {
      m_matrix_grid->SetCellValue(
        row, column,
        wxString::Format("%.6f", matrix[row][column]));
    }
  }
  m_matrix_grid->ForceRefresh();
}

std::string Template_Settings_Dialog::Make_Unique_Id() const
{
  std::size_t suffix = 1;
  std::string id;
  do
  {
    id = "template_" + std::to_string(suffix++);
  }
  while (robot_model::Find_Template_Profile(m_configuration, id));
  return id;
}

std::string Template_Settings_Dialog::Make_Unique_Name(
  const std::string &base) const
{
  const wxString base_name = From_Utf8(base);
  auto name_exists = [this](const wxString &candidate)
  {
    return std::any_of(
      m_configuration.templates.begin(),
      m_configuration.templates.end(),
      [&candidate](const robot_model::Template_Profile &profile)
      {
        return From_Utf8(profile.name).CmpNoCase(candidate) == 0;
      });
  };
  if (!name_exists(base_name))
  {
    return base;
  }
  for (std::size_t suffix = 2; ; ++suffix)
  {
    const wxString candidate =
      wxString::Format("%s %zu", base_name.c_str(), suffix);
    if (!name_exists(candidate))
    {
      return candidate.ToUTF8().data();
    }
  }
}
