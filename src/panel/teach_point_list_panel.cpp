#include "teach_point_list_panel.h"

#include <wx/button.h>
#include <wx/brush.h>
#include <wx/choice.h>
#include <wx/colour.h>
#include <wx/dcmemory.h>
#include <wx/grid.h>
#include <wx/imaglist.h>
#include <wx/menu.h>
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

  Teach_Point_Item_Data(std::string group_key, int first_point_index)
    : group_key(std::move(group_key)),
      first_point_index(first_point_index)
  {
  }

  int point_index = wxNOT_FOUND;
  std::string group_key;
  int first_point_index = wxNOT_FOUND;
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

enum Execution_Image
{
  Execution_Pending = 0,
  Execution_Moving,
  Execution_Waiting,
  Execution_Completed,
  Execution_Failed
};

wxBitmap execution_bitmap(const wxColour &colour)
{
  wxBitmap bitmap(10, 14);
  wxMemoryDC dc(bitmap);
  dc.SetBackground(wxBrush(colour));
  dc.Clear();
  dc.SelectObject(wxNullBitmap);
  return bitmap;
}

int execution_image(
  Teach_Point_List_Panel::Point_Execution_State state)
{
  using State = Teach_Point_List_Panel::Point_Execution_State;
  switch (state)
  {
  case State::Pending: return Execution_Pending;
  case State::Moving: return Execution_Moving;
  case State::Waiting: return Execution_Waiting;
  case State::Completed: return Execution_Completed;
  case State::Failed: return Execution_Failed;
  case State::None: break;
  }
  return -1;
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
    wxTR_DEFAULT_STYLE | wxTR_HIDE_ROOT | wxTR_MULTIPLE |
      wxTR_FULL_ROW_HIGHLIGHT);
  auto *execution_images = new wxImageList(10, 14, false, 5);
  execution_images->Add(execution_bitmap(wxColour(190, 190, 190)));
  execution_images->Add(execution_bitmap(wxColour(55, 135, 225)));
  execution_images->Add(execution_bitmap(wxColour(235, 165, 35)));
  execution_images->Add(execution_bitmap(wxColour(55, 165, 80)));
  execution_images->Add(execution_bitmap(wxColour(210, 60, 60)));
  m_point_list->AssignImageList(execution_images);
  m_point_list->AddRoot("Progress");

  m_info_grid = new wxGrid(this, wxID_ANY);
  configure_read_only_grid(m_info_grid, 4, 2);
  const std::array<wxString, 4> info_names = {
    wxString::FromUTF8(u8"类型"),
    wxString::FromUTF8(u8"绑定坐标系"),
    wxString::FromUTF8(u8"点云"),
    wxString::FromUTF8(u8"模板")};
  for (int row = 0; row < 4; ++row)
  {
    m_info_grid->SetCellValue(row, 0, info_names[row]);
    m_info_grid->SetCellValue(row, 1, wxString::FromUTF8(u8"未选择"));
    m_info_grid->SetCellBackgroundColour(row, 0, wxColour(238, 238, 238));
    m_info_grid->SetRowSize(row, 24);
  }
  m_info_grid->SetMinSize(wxSize(210, 98));

  auto *pose_coordinate_row = new wxBoxSizer(wxHORIZONTAL);
  m_pose_coordinate_label = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"姿态显示坐标系"));
  pose_coordinate_row->Add(
    m_pose_coordinate_label,
    0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
  m_pose_coordinate_choice = new wxChoice(this, wxID_ANY);
  pose_coordinate_row->Add(m_pose_coordinate_choice, 1, wxEXPAND);

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
  m_pose_coordinate_choice->Bind(
    wxEVT_CHOICE,
    [this](wxCommandEvent &)
    {
      if (m_on_pose_coordinate_changed)
      {
        m_on_pose_coordinate_changed(
          m_pose_coordinate_choice->GetSelection());
      }
    });
  m_point_list->Bind(
    wxEVT_TREE_SEL_CHANGED,
    [this](wxTreeEvent &)
    {
      if (!m_updating_selection && m_on_selection_changed)
      {
        m_on_selection_changed();
      }
    });
  m_point_list->Bind(
    wxEVT_TREE_ITEM_ACTIVATED,
    [this](wxTreeEvent &event)
    {
      const auto *data = item_data(m_point_list, event.GetItem());
      if (data && data->point_index != wxNOT_FOUND)
      {
        if (m_on_point_activated)
        {
          m_on_point_activated(data->point_index);
        }
        return;
      }
      event.Skip();
    });
  m_point_list->Bind(
    wxEVT_TREE_ITEM_MENU,
    [this](wxTreeEvent &event)
    {
      wxTreeItemId group = event.GetItem();
      auto *data = item_data(m_point_list, group);
      if (data && data->group_key.empty())
      {
        group = m_point_list->GetItemParent(group);
        data = item_data(m_point_list, group);
      }
      if (!data || data->group_key.empty() ||
          data->first_point_index == wxNOT_FOUND)
      {
        return;
      }

      const int first_point_index = data->first_point_index;
      wxMenu menu;
      const int bind_id = wxWindow::NewControlId();
      const int unbind_id = wxWindow::NewControlId();
      menu.Append(
        bind_id, wxString::FromUTF8(u8"绑定模板..."));
      menu.Append(
        unbind_id, wxString::FromUTF8(u8"解绑模板"));
      menu.Bind(
        wxEVT_MENU,
        [this, first_point_index](wxCommandEvent &)
        {
          if (m_on_bind_cloud_template)
          {
            m_on_bind_cloud_template(first_point_index);
          }
        },
        bind_id);
      menu.Bind(
        wxEVT_MENU,
        [this, first_point_index](wxCommandEvent &)
        {
          if (m_on_unbind_cloud_template)
          {
            m_on_unbind_cloud_template(first_point_index);
          }
        },
        unbind_id);
      m_point_list->PopupMenu(&menu);
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
  root->Add(
    pose_coordinate_row,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    6);
  root->Add(m_pose_grid, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  SetSizer(root);
  SetMinSize(wxSize(220, -1));
}

void Teach_Point_List_Panel::Set_Point_Details(
  const wxString &type,
  const wxString &coordinate,
  const wxString &cloud,
  const wxString &template_name,
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
  m_info_grid->SetCellValue(3, 1, template_name);
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

void Teach_Point_List_Panel::Set_Pose_Coordinate_Choices(
  const std::vector<wxString> &names,
  int selection)
{
  if (!m_pose_coordinate_choice)
  {
    return;
  }
  m_pose_coordinate_choice->Clear();
  for (const auto &name : names)
  {
    m_pose_coordinate_choice->Append(name);
  }
  if (selection >= 0 &&
      selection < static_cast<int>(names.size()))
  {
    m_pose_coordinate_choice->SetSelection(selection);
  }
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
  m_point_names = names;
  m_point_types = types;
  m_point_execution_states.assign(
    names.size(), Point_Execution_State::None);
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
      new Teach_Point_Item_Data(
        group_key, static_cast<int>(start)));
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
  for (std::size_t index = 0; index < names.size(); ++index)
  {
    Update_Point_Execution_Appearance(static_cast<int>(index));
  }
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

void Teach_Point_List_Panel::Begin_Point_Execution()
{
  m_point_execution_states.assign(
    m_point_names.size(), Point_Execution_State::Pending);
  for (std::size_t index = 0; index < m_point_names.size(); ++index)
  {
    Update_Point_Execution_Appearance(static_cast<int>(index));
  }
}

void Teach_Point_List_Panel::Reset_Point_Execution()
{
  m_point_execution_states.assign(
    m_point_names.size(), Point_Execution_State::None);
  for (std::size_t index = 0; index < m_point_names.size(); ++index)
  {
    Update_Point_Execution_Appearance(static_cast<int>(index));
  }
}

void Teach_Point_List_Panel::Set_Point_Execution_State(
  int point_index,
  Point_Execution_State state,
  bool ensure_visible)
{
  if (point_index < 0 ||
      static_cast<std::size_t>(point_index) >=
        m_point_execution_states.size())
  {
    return;
  }
  m_point_execution_states[static_cast<std::size_t>(point_index)] = state;
  Update_Point_Execution_Appearance(point_index);
  const wxTreeItemId item = find_point_item(m_point_list, point_index);
  if (ensure_visible && item.IsOk())
  {
    m_point_list->EnsureVisible(item);
  }
}

void Teach_Point_List_Panel::Update_Point_Execution_Appearance(
  int point_index)
{
  if (!m_point_list || point_index < 0 ||
      static_cast<std::size_t>(point_index) >= m_point_names.size() ||
      static_cast<std::size_t>(point_index) >=
        m_point_execution_states.size())
  {
    return;
  }
  const auto index = static_cast<std::size_t>(point_index);
  const wxTreeItemId item = find_point_item(m_point_list, point_index);
  if (!item.IsOk())
  {
    return;
  }

  const auto state = m_point_execution_states[index];
  m_point_list->SetItemText(item, m_point_names[index]);
  wxColour background = *wxWHITE;
  if (index < m_point_types.size() &&
      m_point_types[index] == robot_model::Robot_Teach_Point_Type::Transition)
  {
    background = wxColour(255, 249, 196);
  }
  else if (index < m_point_types.size() &&
           m_point_types[index] == robot_model::Robot_Teach_Point_Type::Wait)
  {
    background = wxColour(255, 224, 224);
  }

  m_point_list->SetItemBackgroundColour(item, background);
  m_point_list->SetItemTextColour(
    item, m_point_list->GetForegroundColour());
  m_point_list->SetItemBold(item, false);
  const int image = execution_image(state);
  m_point_list->SetItemImage(item, image, wxTreeItemIcon_Normal);
  m_point_list->SetItemImage(item, image, wxTreeItemIcon_Selected);
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

void Teach_Point_List_Panel::Set_On_Point_Activated(
  std::function<void(int)> callback)
{
  m_on_point_activated = std::move(callback);
}

void Teach_Point_List_Panel::Set_On_Collapsed_Changed(
  std::function<void(bool)> callback)
{
  m_on_collapsed_changed = std::move(callback);
}

void Teach_Point_List_Panel::Set_On_Bind_Cloud_Template(
  std::function<void(int)> callback)
{
  m_on_bind_cloud_template = std::move(callback);
}

void Teach_Point_List_Panel::Set_On_Unbind_Cloud_Template(
  std::function<void(int)> callback)
{
  m_on_unbind_cloud_template = std::move(callback);
}

void Teach_Point_List_Panel::Set_On_Pose_Coordinate_Changed(
  std::function<void(int)> callback)
{
  m_on_pose_coordinate_changed = std::move(callback);
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
  if (m_pose_coordinate_choice)
  {
    m_pose_coordinate_choice->Show(!m_collapsed);
  }
  if (m_pose_coordinate_label)
  {
    m_pose_coordinate_label->Show(!m_collapsed);
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
