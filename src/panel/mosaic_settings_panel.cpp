#include "mosaic_settings_panel.h"

#include <wx/button.h>
#include <wx/checklst.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/panel.h>
#include <wx/listbox.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/spinctrl.h>
#include <algorithm>
#include <limits>

class Mosaic_Settings_Panel::Editor : public wxPanel
{
public:
  explicit Editor(wxWindow *parent)
    : wxPanel(parent, wxID_ANY)
  {
    auto *root = new wxBoxSizer(wxVERTICAL);
    auto *help = new wxStaticText(this, wxID_ANY, wxString::FromUTF8(
      u8"仅影响拼图，不改变运动或采图。无分组时：全部运动采图点 + 首张相机平面。\n"
      u8"只列出可采图的运动点；非采图点不可选。水平正向向右，垂直正向向下。"));
    help->Wrap(350);
    root->Add(help, 0, wxALL, 10);
    auto *body = new wxBoxSizer(wxVERTICAL);
    auto *left = new wxBoxSizer(wxVERTICAL);
    m_groups = new wxListBox(this, wxID_ANY, wxDefaultPosition, wxSize(-1, 85));
    left->Add(m_groups, 1, wxEXPAND);
    auto *add = new wxButton(this, wxID_ADD, wxString::FromUTF8(u8"新增组"));
    auto *remove = new wxButton(this, wxID_REMOVE, wxString::FromUTF8(u8"删除组"));
    left->Add(add, 0, wxEXPAND | wxTOP, 6);
    left->Add(remove, 0, wxEXPAND | wxTOP, 6);
    body->Add(left, 0, wxEXPAND | wxBOTTOM, 10);
    auto *right = new wxBoxSizer(wxVERTICAL);
    m_name = new wxTextCtrl(this, wxID_ANY);
    right->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"组名")), 0);
    right->Add(m_name, 0, wxEXPAND | wxBOTTOM, 6);
    m_selection = new wxCheckListBox(this, wxID_ANY, wxDefaultPosition, wxSize(-1, 200));
    right->Add(m_selection, 1, wxEXPAND | wxBOTTOM, 6);
    auto *select_row = new wxBoxSizer(wxHORIZONTAL);
    auto *all = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"全选"));
    auto *none = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"清空"));
    select_row->Add(all, 0, wxRIGHT, 6);
    select_row->Add(none);
    right->Add(select_row, 0, wxBOTTOM, 6);
    auto *range_row = new wxBoxSizer(wxHORIZONTAL);
    auto *first = m_first = new wxChoice(this, wxID_ANY);
    auto *last = m_last = new wxChoice(this, wxID_ANY);
    auto *range = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"选中区间"));
    range_row->Add(first, 1, wxRIGHT, 4);
    range_row->Add(last, 1, wxRIGHT, 4);
    range_row->Add(range);
    range->Bind(wxEVT_BUTTON, [this, first, last](wxCommandEvent &)
    {
      if (m_current < 0 || first->GetSelection() < 0 || last->GetSelection() < 0) return;
      const int begin = std::min(first->GetSelection(), last->GetSelection());
      const int end = std::max(first->GetSelection(), last->GetSelection());
      for (int index = begin; index <= end; ++index) m_selection->Check(index);
    });
    right->Add(range_row, 0, wxEXPAND | wxBOTTOM, 6);
    m_reference = new wxChoice(this, wxID_ANY);
    m_reference->Append(wxString::FromUTF8(u8"首张相机平面（兼容原模式）"));
    m_reference->Append(wxString::FromUTF8(u8"机器人基坐标系两轴"));
    right->Add(m_reference, 0, wxEXPAND | wxBOTTOM, 6);
    auto *axes = new wxBoxSizer(wxHORIZONTAL);
    m_horizontal = new wxChoice(this, wxID_ANY);
    m_vertical = new wxChoice(this, wxID_ANY);
    for (const auto *axis : {"X", "Y", "Z"})
    {
      m_horizontal->Append(axis);
      m_vertical->Append(axis);
    }
    axes->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"横轴")), 0, wxALIGN_CENTER_VERTICAL);
    axes->Add(m_horizontal, 1, wxLEFT | wxRIGHT, 6);
    axes->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"纵轴")), 0, wxALIGN_CENTER_VERTICAL);
    axes->Add(m_vertical, 1, wxLEFT, 6);
    right->Add(axes, 0, wxEXPAND | wxBOTTOM, 6);
    m_reverse_h = new wxCheckBox(this, wxID_ANY, wxString::FromUTF8(u8"水平反向"));
    m_reverse_v = new wxCheckBox(this, wxID_ANY, wxString::FromUTF8(u8"垂直反向"));
    right->Add(m_reverse_h);
    right->Add(m_reverse_v);
    right->Add(new wxStaticText(this, wxID_ANY,
      wxString::FromUTF8(u8"逐点纠偏：选择上方已勾选点，ΔU向右 / ΔV向下（mm）")),
      0, wxTOP | wxBOTTOM, 6);
    auto *correction_row = new wxBoxSizer(wxHORIZONTAL);
    m_u = new wxSpinCtrlDouble(this, wxID_ANY, "0", wxDefaultPosition, wxSize(110, -1),
      wxSP_ARROW_KEYS, -10000, 10000, 0, 0.1);
    m_v = new wxSpinCtrlDouble(this, wxID_ANY, "0", wxDefaultPosition, wxSize(110, -1),
      wxSP_ARROW_KEYS, -10000, 10000, 0, 0.1);
    m_u->SetDigits(3); m_v->SetDigits(3);
    correction_row->Add(new wxStaticText(this, wxID_ANY, "U"), 0, wxALIGN_CENTER_VERTICAL);
    correction_row->Add(m_u, 1, wxRIGHT, 4);
    correction_row->Add(new wxStaticText(this, wxID_ANY, "V"), 0, wxALIGN_CENTER_VERTICAL);
    correction_row->Add(m_v, 1);
    right->Add(correction_row, 0, wxEXPAND);
    auto *zero = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"当前点归零"));
    right->Add(zero, 0, wxTOP, 4);
    zero->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_u->SetValue(0); m_v->SetValue(0); Store_Correction(); });
    m_selection->Bind(wxEVT_LISTBOX, [this](wxCommandEvent &) { Store_Correction(); Load_Correction(); });
    m_selection->Bind(wxEVT_CHECKLISTBOX, [this](wxCommandEvent &) { Store_Correction(); Load_Correction(); });
    auto *correction_help = new wxStaticText(this, wxID_ANY, wxString::FromUTF8(
      u8"仅补偿此组内该点图片，不改机器人运动。应用后下次生成拼图生效，已有PNG不变。建议保留一张图为零补偿参考。"));
    correction_help->Wrap(350);
    right->Add(correction_help, 0, wxTOP, 6);
    body->Add(right, 0, wxEXPAND);
    root->Add(body, 0, wxEXPAND | wxLEFT | wxRIGHT, 10);
    auto *apply = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"应用拼图设置"));
    root->Add(apply, 0, wxEXPAND | wxALL, 10);
    m_status = new wxStaticText(this, wxID_ANY,
      wxString::FromUTF8(u8"修改后点击应用，再保存 Progress。切换工具页保留草稿。"));
    m_status->Wrap(350);
    root->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);
    SetSizer(root);
    m_groups->Bind(wxEVT_LISTBOX, [this](wxCommandEvent &)
    { Store(); m_current = m_groups->GetSelection(); Load(); });
    add->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
    {
      Store();
      robot_model::Mosaic_Group group;
      for (const auto &existing : m_configuration.groups)
      {
        if (existing.id == std::numeric_limits<std::size_t>::max()) return;
        group.id = std::max(group.id, existing.id + 1);
      }
      group.name = "Group " + std::to_string(group.id);
      m_configuration.groups.push_back(group);
      m_current = static_cast<int>(m_configuration.groups.size()) - 1;
      Refresh_Groups(); Load();
    });
    remove->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
    {
      if (m_current < 0) return;
      m_configuration.groups.erase(m_configuration.groups.begin() + m_current);
      m_current = m_configuration.groups.empty() ? -1 : 0;
      Refresh_Groups(); Load();
    });
    all->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
    { for (unsigned int i = 0; i < m_selection->GetCount(); ++i) m_selection->Check(i); });
    none->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
    { for (unsigned int i = 0; i < m_selection->GetCount(); ++i) m_selection->Check(i, false); });
    m_reference->Bind(wxEVT_CHOICE, [this](wxCommandEvent &) { Enable_Axes(); });
    apply->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
    {
      Store();
      std::string error;
      if (!robot_model::Validate_Mosaic_Configuration(m_configuration, m_points, &error))
      {
        Set_Status(wxString::FromUTF8(error));
        return;
      }
      if (m_on_apply && m_on_apply(m_configuration))
        Set_Status(wxString::FromUTF8(u8"已应用；请保存 Progress 以保留设置。"));
      else
        Set_Status(wxString::FromUTF8(u8"当前正在运行或处理图像，未应用。"));
    });
    m_current = m_configuration.groups.empty() ? -1 : 0;
    Refresh_Groups(); Load();

  }

  void Set_Context(const std::string &model,
    const std::vector<robot_model::Robot_Teach_Point> &points,
    const robot_model::Mosaic_Configuration &configuration, bool reset)
  {
    if (!reset && m_model == model) Store();
    else
    {
      m_configuration = configuration;
      m_current = m_configuration.groups.empty() ? -1 : 0;
      Set_Status(wxString::FromUTF8(u8"修改后点击应用，再保存 Progress。"));
    }
    m_model = model;
    m_points = points;
    m_point_ids.clear(); m_selection->Clear(); m_first->Clear(); m_last->Clear();
    for (const auto &point : points)
    {
      if (point.type != robot_model::Robot_Teach_Point_Type::Motion) continue;
      m_point_ids.push_back(point.id);
      m_selection->Append(wxString::FromUTF8(robot_model::Format_Teach_Point_Name(point.id)) +
        wxString::Format("  XYZ: %.1f, %.1f, %.1f", point.world_pose[0], point.world_pose[1], point.world_pose[2]));
    }
    for (const auto &group : m_configuration.groups)
      for (const auto id : group.point_ids)
        if (std::find(m_point_ids.begin(), m_point_ids.end(), id) == m_point_ids.end())
        {
          m_point_ids.push_back(id);
          m_selection->Append(wxString::FromUTF8(robot_model::Format_Teach_Point_Name(id) + " [invalid]"));
        }
    for (auto id : m_point_ids)
    {
      m_first->Append(wxString::FromUTF8(robot_model::Format_Teach_Point_Name(id)));
      m_last->Append(wxString::FromUTF8(robot_model::Format_Teach_Point_Name(id)));
    }
    if (!m_point_ids.empty()) { m_first->SetSelection(0); m_last->SetSelection(m_point_ids.size()-1); }
    Refresh_Groups(); Load();
  }
  std::function<bool(const robot_model::Mosaic_Configuration &)> m_on_apply;

private:
  void Store_Correction()
  {
    if (m_current < 0 || !m_correction_id) return;
    auto &values = m_configuration.groups[m_current].corrections;
    values.erase(std::remove_if(values.begin(), values.end(), [this](const auto &v)
      { return v.point_id == m_correction_id; }), values.end());
    if (m_u->GetValue() != 0 || m_v->GetValue() != 0)
      values.push_back({m_correction_id, m_u->GetValue(), m_v->GetValue()});
  }
  void Load_Correction()
  {
    m_correction_id = 0;
    m_u->SetValue(0); m_v->SetValue(0);
    const int selected = m_selection->GetSelection();
    if (m_current >= 0 && selected >= 0 && m_selection->IsChecked(selected))
    {
      m_correction_id = m_point_ids[selected];
      for (const auto &value : m_configuration.groups[m_current].corrections)
        if (value.point_id == m_correction_id)
        { m_u->SetValue(value.offset_u_mm); m_v->SetValue(value.offset_v_mm); }
    }
    m_u->Enable(m_correction_id != 0); m_v->Enable(m_correction_id != 0);
  }
  void Store()
  {
    if (m_current < 0) return;
    Store_Correction();
    auto &group = m_configuration.groups[m_current];
    group.name = m_name->GetValue().ToStdString(wxConvUTF8);
    group.point_ids.clear();
    for (unsigned int i = 0; i < m_point_ids.size(); ++i)
      if (m_selection->IsChecked(i)) group.point_ids.push_back(m_point_ids[i]);
    group.corrections.erase(std::remove_if(group.corrections.begin(), group.corrections.end(),
      [&group](const auto &value) { return std::find(group.point_ids.begin(), group.point_ids.end(), value.point_id) == group.point_ids.end(); }), group.corrections.end());
    group.reference = m_reference->GetSelection() == 1
      ? robot_model::Mosaic_Reference::Robot_Base : robot_model::Mosaic_Reference::First_Camera;
    group.horizontal_axis = static_cast<robot_model::Coordinate_Axis>(m_horizontal->GetSelection());
    group.vertical_axis = static_cast<robot_model::Coordinate_Axis>(m_vertical->GetSelection());
    group.reverse_horizontal = m_reverse_h->GetValue();
    group.reverse_vertical = m_reverse_v->GetValue();
    m_groups->SetString(m_current, wxString::FromUTF8(group.name));
  }
  void Refresh_Groups()
  {
    m_groups->Clear();
    for (const auto &group : m_configuration.groups) m_groups->Append(wxString::FromUTF8(group.name));
    if (m_current >= 0) m_groups->SetSelection(m_current);
  }
  void Enable_Axes()
  {
    const bool enabled = m_current >= 0 && m_reference->GetSelection() == 1;
    m_horizontal->Enable(enabled); m_vertical->Enable(enabled);
  }
  void Load()
  {
    const bool enabled = m_current >= 0;
    m_name->Enable(enabled); m_selection->Enable(enabled); m_reference->Enable(enabled);
    m_reverse_h->Enable(enabled); m_reverse_v->Enable(enabled);
    const robot_model::Mosaic_Group empty;
    const auto &group = enabled ? m_configuration.groups[m_current] : empty;
    m_name->SetValue(wxString::FromUTF8(group.name));
    m_reference->SetSelection(group.reference == robot_model::Mosaic_Reference::Robot_Base ? 1 : 0);
    m_horizontal->SetSelection(static_cast<int>(group.horizontal_axis));
    m_vertical->SetSelection(static_cast<int>(group.vertical_axis));
    m_reverse_h->SetValue(group.reverse_horizontal); m_reverse_v->SetValue(group.reverse_vertical);
    for (unsigned int i = 0; i < m_point_ids.size(); ++i)
      m_selection->Check(i, std::find(group.point_ids.begin(), group.point_ids.end(), m_point_ids[i]) != group.point_ids.end());
    Load_Correction();
    Enable_Axes();
  }
  void Set_Status(const wxString &text)
  {
    m_status->SetLabel(text); m_status->Wrap(350); Layout();
    static_cast<wxScrolledWindow *>(GetParent())->FitInside();
  }
  wxSpinCtrlDouble *m_u = nullptr, *m_v = nullptr;
  std::size_t m_correction_id = 0;
  std::string m_model;
  std::vector<robot_model::Robot_Teach_Point> m_points;
  wxChoice *m_first = nullptr, *m_last = nullptr;
  wxStaticText *m_status = nullptr;
  robot_model::Mosaic_Configuration m_configuration;
  int m_current = -1;
  std::vector<std::size_t> m_point_ids;
  wxListBox *m_groups = nullptr;
  wxCheckListBox *m_selection = nullptr;
  wxTextCtrl *m_name = nullptr;
  wxChoice *m_reference = nullptr, *m_horizontal = nullptr, *m_vertical = nullptr;
  wxCheckBox *m_reverse_h = nullptr, *m_reverse_v = nullptr;
};

Mosaic_Settings_Panel::Mosaic_Settings_Panel(wxWindow *parent)
  : wxScrolledWindow(parent, wxID_ANY)
{
  SetScrollRate(0, 12);
  m_editor = new Editor(this);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(m_editor, 1, wxEXPAND);
  SetSizer(sizer);
  FitInside();
}
Mosaic_Settings_Panel::~Mosaic_Settings_Panel() = default;
void Mosaic_Settings_Panel::Set_Context(const std::string &model,
  const std::vector<robot_model::Robot_Teach_Point> &points,
  const robot_model::Mosaic_Configuration &configuration, bool reset)
{
  m_editor->Set_Context(model, points, configuration, reset);
  Layout(); FitInside();
}
void Mosaic_Settings_Panel::Set_On_Apply(
  std::function<bool(const robot_model::Mosaic_Configuration &)> callback)
{
  m_editor->m_on_apply = std::move(callback);
}
