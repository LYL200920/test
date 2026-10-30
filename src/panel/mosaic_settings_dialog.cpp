#include "mosaic_settings_dialog.h"

#include <wx/button.h>
#include <wx/checklst.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/dialog.h>
#include <wx/listbox.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <algorithm>
#include <limits>

namespace
{
class Mosaic_Settings_Dialog : public wxDialog
{
public:
  Mosaic_Settings_Dialog(wxWindow *parent,
    const std::vector<robot_model::Robot_Teach_Point> &points,
    const robot_model::Mosaic_Configuration &configuration)
    : wxDialog(parent, wxID_ANY, wxString::FromUTF8(u8"拼图设置"),
        wxDefaultPosition, wxSize(760, 650), wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
      m_points(points), m_configuration(configuration)
  {
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(
      u8"仅影响拼图，不改变运动或采图。无分组时：全部运动采图点 + 首张相机平面。\n"
      u8"只列出可采图的运动点；非采图点不可选。水平正向向右，垂直正向向下。")),
      0, wxALL, 10);
    auto *body = new wxBoxSizer(wxHORIZONTAL);
    auto *left = new wxBoxSizer(wxVERTICAL);
    m_groups = new wxListBox(this, wxID_ANY);
    left->Add(m_groups, 1, wxEXPAND);
    auto *add = new wxButton(this, wxID_ADD, wxString::FromUTF8(u8"新增组"));
    auto *remove = new wxButton(this, wxID_REMOVE, wxString::FromUTF8(u8"删除组"));
    left->Add(add, 0, wxEXPAND | wxTOP, 6);
    left->Add(remove, 0, wxEXPAND | wxTOP, 6);
    body->Add(left, 1, wxEXPAND | wxRIGHT, 10);
    auto *right = new wxBoxSizer(wxVERTICAL);
    m_name = new wxTextCtrl(this, wxID_ANY);
    right->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"组名")), 0);
    right->Add(m_name, 0, wxEXPAND | wxBOTTOM, 6);
    m_selection = new wxCheckListBox(this, wxID_ANY);
    for (const auto &point : points)
    {
      if (point.type != robot_model::Robot_Teach_Point_Type::Motion) continue;
      m_point_ids.push_back(point.id);
      const auto label = robot_model::Format_Teach_Point_Name(point.id) +
        "  XYZ=" + std::to_string(point.world_pose[0]) + ", " +
        std::to_string(point.world_pose[1]) + ", " + std::to_string(point.world_pose[2]) +
        (point.has_world_pose ? "" : " [missing pose]");
      m_selection->Append(wxString::FromUTF8(label));
    }
    // Keep stale references visible so opening and accepting the dialog cannot
    // silently remove deleted/non-capture points from a group.
    for (const auto &group : configuration.groups)
      for (const auto id : group.point_ids)
        if (std::find(m_point_ids.begin(), m_point_ids.end(), id) == m_point_ids.end())
        {
          m_point_ids.push_back(id);
          m_selection->Append(wxString::FromUTF8(
            robot_model::Format_Teach_Point_Name(id) + " [invalid: missing/non-capture]"));
        }
    right->Add(m_selection, 1, wxEXPAND | wxBOTTOM, 6);
    auto *select_row = new wxBoxSizer(wxHORIZONTAL);
    auto *all = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"全选"));
    auto *none = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"清空"));
    select_row->Add(all, 0, wxRIGHT, 6);
    select_row->Add(none);
    right->Add(select_row, 0, wxBOTTOM, 6);
    auto *range_row = new wxBoxSizer(wxHORIZONTAL);
    auto *first = new wxChoice(this, wxID_ANY);
    auto *last = new wxChoice(this, wxID_ANY);
    for (const auto id : m_point_ids)
    {
      first->Append(wxString::FromUTF8(robot_model::Format_Teach_Point_Name(id)));
      last->Append(wxString::FromUTF8(robot_model::Format_Teach_Point_Name(id)));
    }
    if (!m_point_ids.empty())
    {
      first->SetSelection(0);
      last->SetSelection(static_cast<int>(m_point_ids.size()) - 1);
    }
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
    body->Add(right, 3, wxEXPAND);
    root->Add(body, 1, wxEXPAND | wxLEFT | wxRIGHT, 10);
    root->Add(CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, 10);
    SetSizer(root);
    SetMinSize(wxSize(650, 520));
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
    Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
    {
      Store();
      std::string error;
      if (!robot_model::Validate_Mosaic_Configuration(m_configuration, m_points, &error))
      {
        wxMessageBox(wxString::FromUTF8(error), wxString::FromUTF8(u8"拼图设置无效"), wxOK | wxICON_WARNING, this);
        return;
      }
      EndModal(wxID_OK);
    }, wxID_OK);
    m_current = m_configuration.groups.empty() ? -1 : 0;
    Refresh_Groups(); Load();
    CentreOnParent();
  }

  const robot_model::Mosaic_Configuration &Configuration() const { return m_configuration; }

private:
  void Store()
  {
    if (m_current < 0) return;
    auto &group = m_configuration.groups[m_current];
    group.name = m_name->GetValue().ToStdString(wxConvUTF8);
    group.point_ids.clear();
    for (unsigned int i = 0; i < m_point_ids.size(); ++i)
      if (m_selection->IsChecked(i)) group.point_ids.push_back(m_point_ids[i]);
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
    Enable_Axes();
  }
  const std::vector<robot_model::Robot_Teach_Point> &m_points;
  robot_model::Mosaic_Configuration m_configuration;
  int m_current = -1;
  std::vector<std::size_t> m_point_ids;
  wxListBox *m_groups = nullptr;
  wxCheckListBox *m_selection = nullptr;
  wxTextCtrl *m_name = nullptr;
  wxChoice *m_reference = nullptr, *m_horizontal = nullptr, *m_vertical = nullptr;
  wxCheckBox *m_reverse_h = nullptr, *m_reverse_v = nullptr;
};
}

bool Edit_Mosaic_Settings(wxWindow *parent,
  const std::vector<robot_model::Robot_Teach_Point> &points,
  robot_model::Mosaic_Configuration *configuration)
{
  if (!configuration) return false;
  Mosaic_Settings_Dialog dialog(parent, points, *configuration);
  if (dialog.ShowModal() != wxID_OK) return false;
  *configuration = dialog.Configuration();
  return true;
}
