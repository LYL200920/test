#include "tool_panel.h"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/simplebook.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>

#include <algorithm>
#include <utility>

namespace
{

void Add_Labeled_Control(
  wxWindow *parent,
  wxFlexGridSizer *sizer,
  const char *label_utf8,
  wxWindow *control)
{
  sizer->Add(
    new wxStaticText(
      parent,
      wxID_ANY,
      wxString::FromUTF8(label_utf8)),
    0,
    wxALIGN_CENTER_VERTICAL);
  sizer->Add(control, 1, wxEXPAND);
}

} // namespace

Tool_Panel::Tool_Panel(wxWindow *parent)
  : wxPanel(parent, wxID_ANY)
{
  m_page_book = new wxSimplebook(this, wxID_ANY);

  auto *home_page = new wxPanel(m_page_book, wxID_ANY);
  auto *home_title = new wxStaticText(
    home_page,
    wxID_ANY,
    wxString::FromUTF8(u8"Tool"));
  auto *open_fov_button = new wxButton(
    home_page,
    wxID_ANY,
    wxString::FromUTF8(u8"FOV  >"));
  open_fov_button->SetMinSize(wxSize(-1, 44));
  auto *tool_frame_title = new wxStaticText(
    home_page,
    wxID_ANY,
    wxString::FromUTF8(u8"工具坐标系显示"));
  m_tool_frame_visible_checkbox = new wxCheckBox(
    home_page,
    wxID_ANY,
    wxString::FromUTF8(u8"显示当前工具坐标系"));
  m_tool_frame_scale_slider = new wxSlider(
    home_page,
    wxID_ANY,
    50,
    10,
    100);
  m_tool_frame_scale_text = new wxStaticText(
    home_page, wxID_ANY, "50%");
  auto *tool_frame_scale_label = new wxStaticText(
    home_page,
    wxID_ANY,
    wxString::FromUTF8(u8"整体大小"));
  auto *tool_frame_scale_sizer = new wxBoxSizer(wxHORIZONTAL);
  tool_frame_scale_sizer->Add(
    tool_frame_scale_label,
    0,
    wxALIGN_CENTER_VERTICAL | wxRIGHT,
    8);
  tool_frame_scale_sizer->Add(
    m_tool_frame_scale_slider,
    1,
    wxALIGN_CENTER_VERTICAL | wxRIGHT,
    8);
  tool_frame_scale_sizer->Add(
    m_tool_frame_scale_text,
    0,
    wxALIGN_CENTER_VERTICAL);
  auto *apply_tool_frame_button = new wxButton(
    home_page,
    wxID_ANY,
    wxString::FromUTF8(u8"应用坐标系显示设置"));
  auto *home_sizer = new wxBoxSizer(wxVERTICAL);
  home_sizer->Add(home_title, 0, wxALL, 12);
  home_sizer->Add(
    tool_frame_title,
    0,
    wxLEFT | wxRIGHT | wxBOTTOM,
    12);
  home_sizer->Add(
    m_tool_frame_visible_checkbox,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    12);
  home_sizer->Add(
    tool_frame_scale_sizer,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    12);
  home_sizer->Add(
    apply_tool_frame_button,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    12);
  home_sizer->Add(
    open_fov_button,
    0,
    wxEXPAND | wxLEFT | wxRIGHT,
    12);
  home_sizer->AddStretchSpacer(1);
  home_page->SetSizer(home_sizer);

  auto *fov_page = new wxPanel(m_page_book, wxID_ANY);
  auto *back_button = new wxButton(
    fov_page,
    wxID_ANY,
    wxString::FromUTF8(u8"< 返回"));
  auto *fov_title = new wxStaticText(
    fov_page,
    wxID_ANY,
    wxString::FromUTF8(u8"FOV 设置"));
  m_visible_checkbox = new wxCheckBox(
    fov_page,
    wxID_ANY,
    wxString::FromUTF8(u8"在 3D 视图中显示 FOV"));
  m_coordinate_choice = new wxChoice(fov_page, wxID_ANY);
  m_width_control = new wxSpinCtrlDouble(
    fov_page,
    wxID_ANY,
    "500",
    wxDefaultPosition,
    wxDefaultSize,
    wxSP_ARROW_KEYS,
    1.0,
    100000.0,
    500.0,
    1.0);
  m_length_control = new wxSpinCtrlDouble(
    fov_page,
    wxID_ANY,
    "300",
    wxDefaultPosition,
    wxDefaultSize,
    wxSP_ARROW_KEYS,
    1.0,
    100000.0,
    300.0,
    1.0);
  m_width_control->SetDigits(2);
  m_length_control->SetDigits(2);
  m_width_axis_choice = new wxChoice(fov_page, wxID_ANY);
  m_length_axis_choice = new wxChoice(fov_page, wxID_ANY);
  for (const char *axis : {"X", "Y", "Z"})
  {
    m_width_axis_choice->Append(axis);
    m_length_axis_choice->Append(axis);
  }

  auto *form_sizer = new wxFlexGridSizer(2, 8, 8);
  form_sizer->AddGrowableCol(1, 1);
  Add_Labeled_Control(
    fov_page,
    form_sizer,
    u8"绑定工具坐标系",
    m_coordinate_choice);
  Add_Labeled_Control(
    fov_page,
    form_sizer,
    u8"宽度 (mm)",
    m_width_control);
  Add_Labeled_Control(
    fov_page,
    form_sizer,
    u8"长度 (mm)",
    m_length_control);
  Add_Labeled_Control(
    fov_page,
    form_sizer,
    u8"宽度对应轴",
    m_width_axis_choice);
  Add_Labeled_Control(
    fov_page,
    form_sizer,
    u8"长度对应轴",
    m_length_axis_choice);

  auto *apply_button = new wxButton(
    fov_page,
    wxID_ANY,
    wxString::FromUTF8(u8"应用 FOV 设置"));
  m_status_text = new wxStaticText(fov_page, wxID_ANY, wxEmptyString);

  auto *header_sizer = new wxBoxSizer(wxHORIZONTAL);
  header_sizer->Add(back_button, 0, wxRIGHT, 8);
  header_sizer->Add(
    fov_title,
    0,
    wxALIGN_CENTER_VERTICAL);
  auto *fov_sizer = new wxBoxSizer(wxVERTICAL);
  fov_sizer->Add(header_sizer, 0, wxEXPAND | wxALL, 10);
  fov_sizer->Add(
    m_visible_checkbox,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    10);
  fov_sizer->Add(
    form_sizer,
    0,
    wxEXPAND | wxLEFT | wxRIGHT,
    10);
  fov_sizer->Add(
    apply_button,
    0,
    wxEXPAND | wxALL,
    10);
  fov_sizer->Add(
    m_status_text,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    10);
  fov_sizer->AddStretchSpacer(1);
  fov_page->SetSizer(fov_sizer);

  m_page_book->AddPage(home_page, wxEmptyString, true);
  m_page_book->AddPage(fov_page, wxEmptyString, false);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(m_page_book, 1, wxEXPAND);
  SetSizer(sizer);

  open_fov_button->Bind(
    wxEVT_BUTTON, &Tool_Panel::On_Open_Fov, this);
  back_button->Bind(
    wxEVT_BUTTON, &Tool_Panel::On_Back, this);
  apply_button->Bind(
    wxEVT_BUTTON, &Tool_Panel::On_Apply_Fov, this);
  apply_tool_frame_button->Bind(
    wxEVT_BUTTON, &Tool_Panel::On_Apply_Tool_Frame, this);
  m_tool_frame_scale_slider->Bind(
    wxEVT_SLIDER,
    &Tool_Panel::On_Tool_Frame_Scale_Changed,
    this);
  Load_Fov_Controls();
  Load_Tool_Frame_Controls();
}

void Tool_Panel::Set_Tool_Coordinates(
  const robot_model::Tool_Coordinate_Configuration &configuration)
{
  m_tool_coordinates = configuration;
  robot_model::Normalize_Tool_Coordinate_Configuration(
    &m_tool_coordinates);
  Refresh_Coordinate_Choices();
}

void Tool_Panel::Set_Configuration(
  const robot_model::Tool_Visualization_Configuration &configuration)
{
  m_configuration = configuration;
  robot_model::Normalize_Tool_Visualization_Configuration(
    &m_configuration);
  Load_Fov_Controls();
  Load_Tool_Frame_Controls();
}

void Tool_Panel::Set_On_Configuration_Changed(
  std::function<void(
    const robot_model::Tool_Visualization_Configuration &)> callback)
{
  m_on_configuration_changed = std::move(callback);
}

void Tool_Panel::On_Open_Fov(wxCommandEvent &)
{
  Load_Fov_Controls();
  m_page_book->SetSelection(1);
}

void Tool_Panel::On_Back(wxCommandEvent &)
{
  m_page_book->SetSelection(0);
}

void Tool_Panel::On_Apply_Fov(wxCommandEvent &)
{
  const int coordinate_selection =
    m_coordinate_choice->GetSelection();
  if (m_visible_checkbox->GetValue() &&
      (coordinate_selection == wxNOT_FOUND ||
       static_cast<std::size_t>(coordinate_selection) >=
         m_coordinate_ids.size()))
  {
    Set_Status(
      wxString::FromUTF8(u8"请选择用户设置的工具坐标系"),
      true);
    return;
  }
  if (m_width_axis_choice->GetSelection() ==
      m_length_axis_choice->GetSelection())
  {
    Set_Status(
      wxString::FromUTF8(u8"宽度轴和长度轴不能相同"),
      true);
    return;
  }

  auto &fov = m_configuration.fov;
  fov.visible = m_visible_checkbox->GetValue();
  if (coordinate_selection != wxNOT_FOUND &&
      static_cast<std::size_t>(coordinate_selection) <
        m_coordinate_ids.size())
  {
    fov.tool_coordinate_id =
      m_coordinate_ids[static_cast<std::size_t>(
        coordinate_selection)];
  }
  fov.width_mm = m_width_control->GetValue();
  fov.length_mm = m_length_control->GetValue();
  fov.width_axis = static_cast<robot_model::Coordinate_Axis>(
    m_width_axis_choice->GetSelection());
  fov.length_axis = static_cast<robot_model::Coordinate_Axis>(
    m_length_axis_choice->GetSelection());
  robot_model::Normalize_Tool_Visualization_Configuration(
    &m_configuration);
  if (m_on_configuration_changed)
  {
    m_on_configuration_changed(m_configuration);
  }
  Set_Status(wxString::FromUTF8(u8"FOV 设置已应用"));
}

void Tool_Panel::On_Apply_Tool_Frame(wxCommandEvent &)
{
  auto &tool_frame = m_configuration.tool_frame;
  tool_frame.visible =
    m_tool_frame_visible_checkbox->GetValue();
  tool_frame.size_scale =
    static_cast<double>(m_tool_frame_scale_slider->GetValue()) /
    100.0;
  robot_model::Normalize_Tool_Visualization_Configuration(
    &m_configuration);
  if (m_on_configuration_changed)
  {
    m_on_configuration_changed(m_configuration);
  }
  Set_Status(
    wxString::FromUTF8(u8"工具坐标系显示设置已应用"));
}

void Tool_Panel::On_Tool_Frame_Scale_Changed(wxCommandEvent &)
{
  m_tool_frame_scale_text->SetLabel(
    wxString::Format(
      "%d%%",
      m_tool_frame_scale_slider->GetValue()));
}

void Tool_Panel::Refresh_Coordinate_Choices()
{
  const std::string selected_id =
    m_configuration.fov.tool_coordinate_id;
  m_coordinate_choice->Clear();
  m_coordinate_ids.clear();
  int selection = wxNOT_FOUND;
  for (const auto &tool : m_tool_coordinates.tools)
  {
    if (tool.id == "flange")
    {
      continue;
    }
    const int index = static_cast<int>(m_coordinate_ids.size());
    m_coordinate_ids.push_back(tool.id);
    m_coordinate_choice->Append(
      wxString::FromUTF8(tool.name.c_str()));
    if (tool.id == selected_id)
    {
      selection = index;
    }
  }
  if (selection != wxNOT_FOUND)
  {
    m_coordinate_choice->SetSelection(selection);
    Set_Status(wxEmptyString);
  }
  else if (!selected_id.empty())
  {
    Set_Status(
      wxString::FromUTF8(u8"绑定坐标系不存在，请重新选择"),
      true);
  }
}

void Tool_Panel::Load_Fov_Controls()
{
  if (!m_visible_checkbox)
  {
    return;
  }
  const auto &fov = m_configuration.fov;
  m_visible_checkbox->SetValue(fov.visible);
  m_width_control->SetValue(fov.width_mm);
  m_length_control->SetValue(fov.length_mm);
  m_width_axis_choice->SetSelection(
    static_cast<int>(robot_model::Coordinate_Axis_Index(
      fov.width_axis)));
  m_length_axis_choice->SetSelection(
    static_cast<int>(robot_model::Coordinate_Axis_Index(
      fov.length_axis)));
  Refresh_Coordinate_Choices();
}

void Tool_Panel::Load_Tool_Frame_Controls()
{
  if (!m_tool_frame_visible_checkbox ||
      !m_tool_frame_scale_slider)
  {
    return;
  }
  const auto &tool_frame = m_configuration.tool_frame;
  m_tool_frame_visible_checkbox->SetValue(tool_frame.visible);
  m_tool_frame_scale_slider->SetValue(
    static_cast<int>(tool_frame.size_scale * 100.0 + 0.5));
  m_tool_frame_scale_text->SetLabel(
    wxString::Format(
      "%d%%",
      m_tool_frame_scale_slider->GetValue()));
}

void Tool_Panel::Set_Status(const wxString &text, bool error)
{
  if (!m_status_text)
  {
    return;
  }
  m_status_text->SetLabel(text);
  m_status_text->SetForegroundColour(
    error ? wxColour(190, 45, 45) : wxNullColour);
  Layout();
}
