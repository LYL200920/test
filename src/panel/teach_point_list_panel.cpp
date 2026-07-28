#include "teach_point_list_panel.h"

#include <wx/button.h>
#include <wx/colour.h>
#include <wx/grid.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/treectrl.h>

#include <algorithm>
#include <map>
#include <utility>

namespace
{

class Teach_Point_Item_Data : public wxTreeItemData
{
public:
  explicit Teach_Point_Item_Data(int point_index)
    : point_index(point_index)
  {
  }

  explicit Teach_Point_Item_Data(std::string group_key)
    : group_key(std::move(group_key))
  {
  }

  int point_index = wxNOT_FOUND;
  std::string group_key;
};

Teach_Point_Item_Data *item_data(
  wxTreeCtrl *tree,
  const wxTreeItemId &item)
{
  return tree && item.IsOk()
    ? dynamic_cast<Teach_Point_Item_Data *>(tree->GetItemData(item))
    : nullptr;
}

wxTreeItemId find_point_item(
  wxTreeCtrl *tree,
  int point_index)
{
  if (!tree)
  {
    return {};
  }
  const wxTreeItemId root = tree->GetRootItem();
  wxTreeItemIdValue group_cookie;
  for (wxTreeItemId group = tree->GetFirstChild(root, group_cookie);
       group.IsOk();
       group = tree->GetNextChild(root, group_cookie))
  {
    wxTreeItemIdValue point_cookie;
    for (wxTreeItemId point = tree->GetFirstChild(group, point_cookie);
         point.IsOk();
         point = tree->GetNextChild(group, point_cookie))
    {
      const auto *data = item_data(tree, point);
      if (data && data->point_index == point_index)
      {
        return point;
      }
    }
  }
  return {};
}

void configure_read_only_grid(wxGrid *grid, int rows, int columns)
{
  grid->CreateGrid(rows, columns);
  grid->EnableEditing(false);
  grid->EnableGridLines(true);
  grid->SetRowLabelSize(0);
  grid->SetColLabelSize(0);
  grid->DisableDragGridSize();
  grid->DisableDragColSize();
  grid->DisableDragRowSize();
  grid->SetDefaultCellAlignment(wxALIGN_CENTER, wxALIGN_CENTER);
  grid->SetGridLineColour(wxColour(205, 205, 205));
  grid->SetMargins(0, 0);
  grid->ShowScrollbars(wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
}

} // namespace

Teach_Point_List_Panel::Teach_Point_List_Panel(wxWindow *parent)
  : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
            wxBORDER_SIMPLE)
{
  auto *header = new wxBoxSizer(wxHORIZONTAL);
  m_title = new wxStaticText(this, wxID_ANY, "Progress");
  m_toggle_button = new wxButton(
    this, wxID_ANY, "<", wxDefaultPosition, wxSize(30, -1),
    wxBU_EXACTFIT);
  header->Add(m_title, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 6);
  header->Add(m_toggle_button, 0, wxALIGN_CENTER_VERTICAL);

  m_point_list = new wxTreeCtrl(
    this,
    wxID_ANY,
    wxDefaultPosition,
    wxDefaultSize,
    wxTR_HIDE_ROOT | wxTR_HAS_BUTTONS | wxTR_LINES_AT_ROOT |
      wxTR_MULTIPLE);
  m_point_list->AddRoot("Progress");

  m_info_grid = new wxGrid(this, wxID_ANY);
  configure_read_only_grid(m_info_grid, 3, 2);
  const std::array<wxString, 3> info_names = {
    wxString::FromUTF8(u8"类型"),
    wxString::FromUTF8(u8"坐标系"),
    wxString::FromUTF8(u8"点云")};
  for (int row = 0; row < 3; ++row)
  {
    m_info_grid->SetCellValue(row, 0, info_names[row]);
    m_info_grid->SetCellValue(row, 1, wxString::FromUTF8(u8"未选择"));
    m_info_grid->SetCellBackgroundColour(row, 0, wxColour(238, 238, 238));
    m_info_grid->SetRowSize(row, 24);
  }
  m_info_grid->SetMinSize(wxSize(210, 74));

  m_pose_grid = new wxGrid(this, wxID_ANY);
  configure_read_only_grid(m_pose_grid, 4, 3);
  const std::array<wxString, 6> pose_names = {
    "X (mm)", "Y (mm)", "Z (mm)",
    wxString::FromUTF8(u8"A (°)"),
    wxString::FromUTF8(u8"B (°)"),
    wxString::FromUTF8(u8"C (°)")};
  for (int column = 0; column < 3; ++column)
  {
    m_pose_grid->SetCellValue(0, column, pose_names[column]);
    m_pose_grid->SetCellValue(1, column, "-");
    m_pose_grid->SetCellValue(2, column, pose_names[column + 3]);
    m_pose_grid->SetCellValue(3, column, "-");
    m_pose_grid->SetCellBackgroundColour(
      0, column, wxColour(238, 238, 238));
    m_pose_grid->SetCellBackgroundColour(
      2, column, wxColour(238, 238, 238));
  }
  for (int row = 0; row < 4; ++row)
  {
    m_pose_grid->SetRowSize(row, 24);
  }
  m_pose_grid->SetMinSize(wxSize(210, 98));

  m_toggle_button->Bind(
    wxEVT_BUTTON,
    [this](wxCommandEvent &) { Toggle_Collapsed(); });
  m_point_list->Bind(
    wxEVT_TREE_SEL_CHANGED,
    [this](wxTreeEvent &)
    {
      if (!m_updating_selection && m_on_selection_changed)
      {
        m_on_selection_changed();
      }
    });
  Bind(
    wxEVT_SIZE,
    [this](wxSizeEvent &event)
    {
      if (m_info_grid)
      {
        const int width = std::max(120, m_info_grid->GetClientSize().x);
        m_info_grid->SetColSize(0, 64);
        m_info_grid->SetColSize(1, std::max(56, width - 64));
      }
      if (m_pose_grid)
      {
        const int width = std::max(120, m_pose_grid->GetClientSize().x);
        const int column_width = std::max(40, width / 3);
        for (int column = 0; column < 3; ++column)
        {
          m_pose_grid->SetColSize(column, column_width);
        }
      }
      event.Skip();
    });
  m_point_list->Bind(
    wxEVT_TREE_ITEM_COLLAPSED,
    [this](wxTreeEvent &event)
    {
      const auto *data = item_data(m_point_list, event.GetItem());
      if (data && !data->group_key.empty())
      {
        m_collapsed_group_keys.insert(data->group_key);
      }
    });
  m_point_list->Bind(
    wxEVT_TREE_ITEM_EXPANDED,
    [this](wxTreeEvent &event)
    {
      const auto *data = item_data(m_point_list, event.GetItem());
      if (data && !data->group_key.empty())
      {
        m_collapsed_group_keys.erase(data->group_key);
      }
    });

  auto *root = new wxBoxSizer(wxVERTICAL);
  root->Add(header, 0, wxEXPAND | wxALL, 4);
  root->Add(m_point_list, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 4);
  root->Add(m_info_grid, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  root->Add(m_pose_grid, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  SetSizer(root);
  SetMinSize(wxSize(220, -1));
}

void Teach_Point_List_Panel::Set_Point_Details(
  const wxString &type,
  const wxString &coordinate,
  const wxString &cloud,
  robot_model::Robot_Teach_Point_Type point_type,
  bool highlight_type)
{
  if (!m_info_grid)
  {
    return;
  }
  m_info_grid->SetCellValue(0, 1, type);
  m_info_grid->SetCellValue(1, 1, coordinate);
  m_info_grid->SetCellValue(2, 1, cloud);
  wxColour type_colour = *wxWHITE;
  if (highlight_type &&
      point_type == robot_model::Robot_Teach_Point_Type::Transition)
  {
    type_colour = wxColour(255, 249, 196);
  }
  else if (highlight_type &&
           point_type == robot_model::Robot_Teach_Point_Type::Wait)
  {
    type_colour = wxColour(255, 224, 224);
  }
  m_info_grid->SetCellBackgroundColour(0, 1, type_colour);
  m_info_grid->ForceRefresh();
}

void Teach_Point_List_Panel::Set_Point_Pose(
  const robot_model::XyzabcPose &pose,
  bool has_pose)
{
  if (!m_pose_grid)
  {
    return;
  }
  for (int column = 0; column < 3; ++column)
  {
    m_pose_grid->SetCellValue(
      1, column,
      has_pose ? wxString::Format("%.2f", pose[column]) : "-");
    m_pose_grid->SetCellValue(
      3, column,
      has_pose ? wxString::Format("%.2f", pose[column + 3]) : "-");
  }
  m_pose_grid->ForceRefresh();
}

void Teach_Point_List_Panel::Set_Point_Names(
  const std::vector<wxString> &names,
  const std::vector<robot_model::Robot_Teach_Point_Type> &types,
  const std::vector<wxString> &cloud_names,
  const std::vector<std::string> &cloud_keys)
{
  if (!m_point_list)
  {
    return;
  }
  const std::vector<int> old_selections = Selected_Point_Indices();
  m_updating_selection = true;
  m_point_list->DeleteAllItems();
  const wxTreeItemId root = m_point_list->AddRoot("Progress");

  std::map<std::string, std::size_t> occurrences;
  std::size_t start = 0;
  while (start < names.size())
  {
    const std::string cloud_key =
      start < cloud_keys.size() ? cloud_keys[start] : std::string();
    std::size_t end = start + 1;
    while (end < names.size())
    {
      const std::string next_key =
        end < cloud_keys.size() ? cloud_keys[end] : std::string();
      if (next_key != cloud_key)
      {
        break;
      }
      ++end;
    }

    const std::size_t occurrence = occurrences[cloud_key]++;
    const std::string group_key =
      cloud_key + "#" + std::to_string(occurrence);
    const wxString cloud_name =
      start < cloud_names.size() && !cloud_names[start].empty()
        ? cloud_names[start]
        : wxString::FromUTF8(u8"未绑定点云");
    const wxTreeItemId group = m_point_list->AppendItem(
      root,
      wxString::Format("%s (%zu)", cloud_name.c_str(), end - start),
      -1,
      -1,
      new Teach_Point_Item_Data(group_key));
    m_point_list->SetItemBold(group, true);
    m_point_list->SetItemBackgroundColour(
      group, wxColour(235, 235, 235));

    for (std::size_t index = start; index < end; ++index)
    {
      const wxTreeItemId point = m_point_list->AppendItem(
        group,
        names[index],
        -1,
        -1,
        new Teach_Point_Item_Data(static_cast<int>(index)));
      const auto type = index < types.size()
        ? types[index]
        : robot_model::Robot_Teach_Point_Type::Motion;
      if (type == robot_model::Robot_Teach_Point_Type::Transition)
      {
        m_point_list->SetItemBackgroundColour(
          point, wxColour(255, 249, 196));
      }
      else if (type == robot_model::Robot_Teach_Point_Type::Wait)
      {
        m_point_list->SetItemBackgroundColour(
          point, wxColour(255, 224, 224));
      }
    }

    if (m_collapsed_group_keys.count(group_key) == 0)
    {
      m_point_list->Expand(group);
    }
    start = end;
  }

  std::vector<int> selections;
  for (const int selection : old_selections)
  {
    if (selection >= 0 &&
        selection < static_cast<int>(names.size()))
    {
      selections.push_back(selection);
    }
  }
  if (selections.empty() && !names.empty())
  {
    selections.push_back(static_cast<int>(names.size() - 1));
  }
  Set_Point_Selections(selections);
  m_updating_selection = false;
}

int Teach_Point_List_Panel::Selected_Point_Index() const
{
  const auto selections = Selected_Point_Indices();
  return selections.size() == 1 ? selections.front() : wxNOT_FOUND;
}

std::vector<int> Teach_Point_List_Panel::Selected_Point_Indices() const
{
  std::vector<int> selections;
  if (!m_point_list)
  {
    return selections;
  }
  wxArrayTreeItemIds selected_items;
  m_point_list->GetSelections(selected_items);
  for (const auto &item : selected_items)
  {
    const auto *data = item_data(m_point_list, item);
    if (data && data->point_index != wxNOT_FOUND)
    {
      selections.push_back(data->point_index);
    }
  }
  std::sort(selections.begin(), selections.end());
  return selections;
}

void Teach_Point_List_Panel::Set_Point_Selection(int selection)
{
  Set_Point_Selections(
    selection == wxNOT_FOUND
      ? std::vector<int>{}
      : std::vector<int>{selection});
}

void Teach_Point_List_Panel::Set_Point_Selections(
  const std::vector<int> &selections)
{
  if (!m_point_list)
  {
    return;
  }
  const bool was_updating = m_updating_selection;
  m_updating_selection = true;
  m_point_list->UnselectAll();
  for (const int selection : selections)
  {
    const wxTreeItemId item = find_point_item(m_point_list, selection);
    if (item.IsOk())
    {
      m_point_list->SelectItem(item);
    }
  }
  if (!selections.empty())
  {
    const wxTreeItemId first =
      find_point_item(m_point_list, selections.front());
    if (first.IsOk())
    {
      m_point_list->EnsureVisible(first);
    }
  }
  m_updating_selection = was_updating;
}

void Teach_Point_List_Panel::Set_Dirty(bool dirty)
{
  m_dirty = dirty;
  if (m_title)
  {
    m_title->SetLabel(m_dirty ? "Progress *" : "Progress");
  }
}

void Teach_Point_List_Panel::Set_List_Enabled(bool enabled)
{
  if (m_point_list)
  {
    m_point_list->Enable(enabled);
  }
}

void Teach_Point_List_Panel::Set_On_Selection_Changed(
  std::function<void()> callback)
{
  m_on_selection_changed = std::move(callback);
}

void Teach_Point_List_Panel::Set_On_Collapsed_Changed(
  std::function<void(bool)> callback)
{
  m_on_collapsed_changed = std::move(callback);
}

void Teach_Point_List_Panel::Toggle_Collapsed()
{
  m_collapsed = !m_collapsed;
  Update_Collapsed_State();
}

void Teach_Point_List_Panel::Update_Collapsed_State()
{
  if (m_title)
  {
    m_title->Show(!m_collapsed);
  }
  if (m_point_list)
  {
    m_point_list->Show(!m_collapsed);
  }
  if (m_info_grid)
  {
    m_info_grid->Show(!m_collapsed);
  }
  if (m_pose_grid)
  {
    m_pose_grid->Show(!m_collapsed);
  }
  if (m_toggle_button)
  {
    m_toggle_button->SetLabel(m_collapsed ? ">" : "<");
  }
  SetMinSize(wxSize(m_collapsed ? 38 : 220, -1));
  Layout();
  if (m_on_collapsed_changed)
  {
    m_on_collapsed_changed(m_collapsed);
  }
}
