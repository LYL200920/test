#include "robot_model_config_dialog.h"

#include <wx/button.h>
#include <wx/listbox.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <algorithm>
#include <limits>

namespace
{

wxString From_Utf8(const std::string &value)
{
  return wxString::FromUTF8(value.c_str());
}

std::string Model_Id(const robot_model::Robot_Model_Info &model)
{
  return model.model_dir.filename().string();
}

}

Robot_Model_Config_Dialog::Robot_Model_Config_Dialog(
  wxWindow *parent,
  const std::vector<robot_model::Robot_Model_Info> &models,
  const std::string &current_model_id,
  const robot_model::Tool_Coordinate_Configuration &tool_configuration)
  : wxDialog(
      parent,
      wxID_ANY,
      wxString::FromUTF8(u8"机械臂设置"),
      wxDefaultPosition,
      wxSize(620, 500),
      wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
    m_models(models),
    m_tool_configuration(tool_configuration)
{
  robot_model::Normalize_Tool_Coordinate_Configuration(
    &m_tool_configuration);
  m_notebook = new wxNotebook(this, wxID_ANY);

  auto *model_page = new wxPanel(m_notebook, wxID_ANY);
  m_model_status_text = new wxStaticText(
    model_page,
    wxID_ANY,
    current_model_id.empty()
      ? wxString::FromUTF8(u8"当前机械臂：未加载")
      : wxString::Format(
          wxString::FromUTF8(u8"当前机械臂：%s"),
          From_Utf8(current_model_id)));
  m_model_list = new wxListBox(model_page, wxID_ANY);
  int current_model_selection = wxNOT_FOUND;
  for (std::size_t index = 0; index < m_models.size(); ++index)
  {
    const std::string label = m_models[index].display_name.empty()
      ? Model_Id(m_models[index])
      : m_models[index].display_name;
    m_model_list->Append(From_Utf8(label));
    if (Model_Id(m_models[index]) == current_model_id)
    {
      current_model_selection = static_cast<int>(index);
    }
  }
  if (current_model_selection != wxNOT_FOUND)
  {
    m_model_list->SetSelection(current_model_selection);
  }
  else if (!m_models.empty())
  {
    m_model_list->SetSelection(0);
  }
  auto *model_sizer = new wxBoxSizer(wxVERTICAL);
  model_sizer->Add(m_model_status_text, 0, wxEXPAND | wxALL, 10);
  model_sizer->Add(m_model_list, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);
  model_page->SetSizer(model_sizer);
  m_notebook->AddPage(
    model_page, wxString::FromUTF8(u8"机械臂模型"), true);

  auto *tool_page = new wxPanel(m_notebook, wxID_ANY);
  m_tool_list = new wxListBox(tool_page, wxID_ANY);
  m_tool_name = new wxTextCtrl(tool_page, wxID_ANY);
  const std::array<wxString, 6> labels = {
    "X (mm)", "Y (mm)", "Z (mm)", "A (°)", "B (°)", "C (°)"};
  auto *pose_grid = new wxFlexGridSizer(2, 6, 6);
  pose_grid->AddGrowableCol(1, 1);
  pose_grid->Add(
    new wxStaticText(tool_page, wxID_ANY, wxString::FromUTF8(u8"工具名称")),
    0, wxALIGN_CENTER_VERTICAL);
  pose_grid->Add(m_tool_name, 1, wxEXPAND);
  for (std::size_t index = 0; index < labels.size(); ++index)
  {
    pose_grid->Add(
      new wxStaticText(tool_page, wxID_ANY, labels[index]),
      0, wxALIGN_CENTER_VERTICAL);
    const double limit = index < 3 ? 10000.0 : 360.0;
    m_tool_pose_controls[index] = new wxSpinCtrlDouble(
      tool_page,
      wxID_ANY,
      "0",
      wxDefaultPosition,
      wxDefaultSize,
      wxSP_ARROW_KEYS,
      -limit,
      limit,
      0.0,
      index < 3 ? 1.0 : 0.1);
    m_tool_pose_controls[index]->SetDigits(index < 3 ? 3 : 3);
    pose_grid->Add(m_tool_pose_controls[index], 1, wxEXPAND);
  }

  auto *new_tool_button = new wxButton(
    tool_page, wxID_ANY, wxString::FromUTF8(u8"新建工具"));
  m_delete_tool_button = new wxButton(
    tool_page, wxID_ANY, wxString::FromUTF8(u8"删除工具"));
  auto *tool_buttons = new wxBoxSizer(wxHORIZONTAL);
  tool_buttons->Add(new_tool_button, 1, wxRIGHT, 6);
  tool_buttons->Add(m_delete_tool_button, 1);

  auto *tool_detail_sizer = new wxBoxSizer(wxVERTICAL);
  tool_detail_sizer->Add(
    new wxStaticText(
      tool_page,
      wxID_ANY,
      wxString::FromUTF8(u8"工具相对于法兰坐标系的位姿")),
    0, wxBOTTOM, 8);
  tool_detail_sizer->Add(pose_grid, 0, wxEXPAND);
  tool_detail_sizer->Add(tool_buttons, 0, wxEXPAND | wxTOP, 10);

  auto *tool_sizer = new wxBoxSizer(wxHORIZONTAL);
  tool_sizer->Add(m_tool_list, 1, wxEXPAND | wxALL, 10);
  tool_sizer->Add(tool_detail_sizer, 2, wxEXPAND | wxTOP | wxRIGHT | wxBOTTOM, 10);
  tool_page->SetSizer(tool_sizer);
  m_notebook->AddPage(
    tool_page, wxString::FromUTF8(u8"工具坐标系"), false);

  m_action_button = new wxButton(this, wxID_ANY, "");
  auto *cancel_button = new wxButton(
    this, wxID_CANCEL, wxString::FromUTF8(u8"取消"));
  auto *button_sizer = new wxBoxSizer(wxHORIZONTAL);
  button_sizer->AddStretchSpacer(1);
  button_sizer->Add(m_action_button, 0, wxRIGHT, 8);
  button_sizer->Add(cancel_button, 0);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(m_notebook, 1, wxEXPAND | wxALL, 10);
  sizer->Add(button_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);
  SetSizer(sizer);
  SetMinSize(wxSize(540, 420));

  m_notebook->Bind(
    wxEVT_NOTEBOOK_PAGE_CHANGED,
    &Robot_Model_Config_Dialog::On_Page_Changed,
    this);
  m_model_list->Bind(
    wxEVT_LISTBOX_DCLICK,
    &Robot_Model_Config_Dialog::On_Model_Activated,
    this);
  m_action_button->Bind(
    wxEVT_BUTTON,
    &Robot_Model_Config_Dialog::On_Action,
    this);
  m_tool_list->Bind(
    wxEVT_LISTBOX,
    &Robot_Model_Config_Dialog::On_Tool_Selection_Changed,
    this);
  new_tool_button->Bind(
    wxEVT_BUTTON,
    &Robot_Model_Config_Dialog::On_New_Tool,
    this);
  m_delete_tool_button->Bind(
    wxEVT_BUTTON,
    &Robot_Model_Config_Dialog::On_Delete_Tool,
    this);

  std::size_t active_tool_index = 0;
  for (std::size_t index = 0;
       index < m_tool_configuration.tools.size();
       ++index)
  {
    if (m_tool_configuration.tools[index].id ==
        m_tool_configuration.active_tool_id)
    {
      active_tool_index = index;
      break;
    }
  }
  Refresh_Tool_List(active_tool_index);
  Load_Tool_Editor(active_tool_index);
  Update_Action_Button();
  CentreOnParent();
}

std::size_t Robot_Model_Config_Dialog::Selected_Model_Index() const
{
  const int selection = m_model_list->GetSelection();
  return selection == wxNOT_FOUND
    ? std::numeric_limits<std::size_t>::max()
    : static_cast<std::size_t>(selection);
}

bool Robot_Model_Config_Dialog::Model_Load_Requested() const
{
  return m_result_action == Result_Action::Load_Model;
}

bool Robot_Model_Config_Dialog::Tool_Apply_Requested() const
{
  return m_result_action == Result_Action::Apply_Tool;
}

const robot_model::Tool_Coordinate_Configuration &
Robot_Model_Config_Dialog::Tool_Configuration() const
{
  return m_tool_configuration;
}

void Robot_Model_Config_Dialog::On_Page_Changed(wxBookCtrlEvent &event)
{
  Update_Action_Button();
  event.Skip();
}

void Robot_Model_Config_Dialog::On_Model_Activated(wxCommandEvent &)
{
  if (Selected_Model_Index() < m_models.size())
  {
    m_result_action = Result_Action::Load_Model;
    EndModal(wxID_OK);
  }
}

void Robot_Model_Config_Dialog::On_Action(wxCommandEvent &)
{
  if (m_notebook->GetSelection() == 0)
  {
    if (Selected_Model_Index() >= m_models.size())
    {
      return;
    }
    m_result_action = Result_Action::Load_Model;
  }
  else
  {
    if (!Store_Tool_Editor())
    {
      return;
    }
    const int selection = m_tool_list->GetSelection();
    if (selection == wxNOT_FOUND)
    {
      return;
    }
    m_tool_configuration.active_tool_id =
      m_tool_configuration.tools[static_cast<std::size_t>(selection)].id;
    robot_model::Normalize_Tool_Coordinate_Configuration(
      &m_tool_configuration);
    m_result_action = Result_Action::Apply_Tool;
  }
  EndModal(wxID_OK);
}

void Robot_Model_Config_Dialog::On_Tool_Selection_Changed(wxCommandEvent &)
{
  const int selection = m_tool_list->GetSelection();
  if (!Store_Tool_Editor())
  {
    m_tool_list->SetSelection(static_cast<int>(m_tool_editor_index));
    return;
  }
  if (selection != wxNOT_FOUND)
  {
    m_tool_list->SetSelection(selection);
    Load_Tool_Editor(static_cast<std::size_t>(selection));
  }
}

void Robot_Model_Config_Dialog::On_New_Tool(wxCommandEvent &)
{
  Store_Tool_Editor();
  std::size_t suffix = 1;
  std::string id;
  do
  {
    id = "tool_" + std::to_string(suffix++);
  }
  while (robot_model::Find_Tool_Coordinate(m_tool_configuration, id));

  robot_model::Tool_Coordinate_Profile tool;
  tool.id = id;
  tool.name = "Tool " + std::to_string(suffix - 1);
  m_tool_configuration.tools.push_back(tool);
  const std::size_t selection = m_tool_configuration.tools.size() - 1;
  Refresh_Tool_List(selection);
  Load_Tool_Editor(selection);
}

void Robot_Model_Config_Dialog::On_Delete_Tool(wxCommandEvent &)
{
  const int selection = m_tool_list->GetSelection();
  if (selection <= 0)
  {
    return;
  }
  const std::size_t index = static_cast<std::size_t>(selection);
  const std::string deleted_id = m_tool_configuration.tools[index].id;
  m_tool_configuration.tools.erase(
    m_tool_configuration.tools.begin() + selection);
  if (m_tool_configuration.active_tool_id == deleted_id)
  {
    m_tool_configuration.active_tool_id = "flange";
  }
  Refresh_Tool_List(0);
  Load_Tool_Editor(0);
}

void Robot_Model_Config_Dialog::Load_Tool_Editor(std::size_t index)
{
  if (index >= m_tool_configuration.tools.size())
  {
    return;
  }
  m_tool_editor_index = index;
  const auto &tool = m_tool_configuration.tools[index];
  m_tool_name->SetValue(From_Utf8(tool.name));
  for (std::size_t pose_index = 0;
       pose_index < m_tool_pose_controls.size();
       ++pose_index)
  {
    m_tool_pose_controls[pose_index]->SetValue(
      tool.flange_from_tool_pose[pose_index]);
  }
  const bool editable = index != 0;
  m_tool_name->Enable(editable);
  for (wxSpinCtrlDouble *control : m_tool_pose_controls)
  {
    control->Enable(editable);
  }
  m_delete_tool_button->Enable(editable);
}

bool Robot_Model_Config_Dialog::Store_Tool_Editor()
{
  if (m_tool_editor_index >= m_tool_configuration.tools.size())
  {
    return false;
  }
  if (m_tool_editor_index == 0)
  {
    return true;
  }

  std::string name = m_tool_name->GetValue().ToUTF8().data();
  if (name.empty())
  {
    m_tool_name->SetFocus();
    return false;
  }
  auto &tool = m_tool_configuration.tools[m_tool_editor_index];
  tool.name = std::move(name);
  for (std::size_t index = 0;
       index < m_tool_pose_controls.size();
       ++index)
  {
    tool.flange_from_tool_pose[index] =
      m_tool_pose_controls[index]->GetValue();
  }
  Refresh_Tool_List(m_tool_editor_index);
  return true;
}

void Robot_Model_Config_Dialog::Refresh_Tool_List(std::size_t selection)
{
  m_tool_list->Clear();
  for (const auto &tool : m_tool_configuration.tools)
  {
    wxString label = From_Utf8(tool.name);
    if (tool.id == m_tool_configuration.active_tool_id)
    {
      label += wxString::FromUTF8(u8"（当前）");
    }
    m_tool_list->Append(label);
  }
  if (!m_tool_configuration.tools.empty())
  {
    m_tool_list->SetSelection(static_cast<int>(
      std::min(selection, m_tool_configuration.tools.size() - 1)));
  }
}

void Robot_Model_Config_Dialog::Update_Action_Button()
{
  const bool model_page = m_notebook->GetSelection() == 0;
  m_action_button->SetLabel(wxString::FromUTF8(
    model_page ? u8"加载机械臂" : u8"应用工具坐标"));
  m_action_button->Enable(
    model_page
      ? Selected_Model_Index() < m_models.size()
      : !m_tool_configuration.tools.empty());
  m_action_button->SetDefault();
}
