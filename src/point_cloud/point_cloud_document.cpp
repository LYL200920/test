#include "point_cloud_document.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace point_cloud
{
namespace
{

constexpr std::size_t kMaximumUndoDepth = 20;

}

void Point_Cloud_Document::Set_Data(Point_Cloud_Data data)
{
  m_data = std::move(data);
  m_selection.clear();
  m_undo_stack.clear();
  m_redo_stack.clear();
}

void Point_Cloud_Document::Clear()
{
  Set_Data({});
}

void Point_Cloud_Document::Normalize_Ids(
  std::vector<std::size_t> *point_ids,
  std::size_t point_count)
{
  point_ids->erase(
    std::remove_if(
      point_ids->begin(), point_ids->end(),
      [point_count](std::size_t point_id) { return point_id >= point_count; }),
    point_ids->end());
  std::sort(point_ids->begin(), point_ids->end());
  point_ids->erase(
    std::unique(point_ids->begin(), point_ids->end()),
    point_ids->end());
}

void Point_Cloud_Document::Set_Selection(
  std::vector<std::size_t> point_ids,
  Point_Selection_Mode mode)
{
  Normalize_Ids(&point_ids, Point_Count());
  if (mode == Point_Selection_Mode::Replace)
  {
    m_selection = std::move(point_ids);
    return;
  }

  std::vector<std::size_t> updated;
  if (mode == Point_Selection_Mode::Add)
  {
    updated.reserve(m_selection.size() + point_ids.size());
    std::set_union(
      m_selection.begin(), m_selection.end(),
      point_ids.begin(), point_ids.end(),
      std::back_inserter(updated));
  }
  else
  {
    updated.reserve(m_selection.size() + point_ids.size());
    std::set_symmetric_difference(
      m_selection.begin(), m_selection.end(),
      point_ids.begin(), point_ids.end(),
      std::back_inserter(updated));
  }
  m_selection = std::move(updated);
}

void Point_Cloud_Document::Clear_Selection()
{
  m_selection.clear();
}

void Point_Cloud_Document::Apply_Delete(
  Point_Cloud_Data *data,
  const std::vector<std::size_t> &point_ids)
{
  if (point_ids.empty())
  {
    return;
  }

  Point_Cloud_Data remaining;
  const std::size_t remaining_count = data->Point_Count() - point_ids.size();
  remaining.xyz.reserve(remaining_count * 3);
  if (data->Has_Color())
  {
    remaining.rgb.reserve(remaining_count * 3);
  }

  std::size_t deleted_index = 0;
  for (std::size_t point_index = 0; point_index < data->Point_Count(); ++point_index)
  {
    if (deleted_index < point_ids.size() &&
        point_ids[deleted_index] == point_index)
    {
      ++deleted_index;
      continue;
    }

    remaining.xyz.insert(
      remaining.xyz.end(),
      data->xyz.begin() + static_cast<std::ptrdiff_t>(point_index * 3),
      data->xyz.begin() + static_cast<std::ptrdiff_t>(point_index * 3 + 3));
    if (data->Has_Color())
    {
      remaining.rgb.insert(
        remaining.rgb.end(),
        data->rgb.begin() + static_cast<std::ptrdiff_t>(point_index * 3),
        data->rgb.begin() + static_cast<std::ptrdiff_t>(point_index * 3 + 3));
    }
  }
  *data = std::move(remaining);
}

void Point_Cloud_Document::Restore_Delete(
  Point_Cloud_Data *data,
  const Delete_Command &command)
{
  Point_Cloud_Data restored;
  const std::size_t restored_count =
    data->Point_Count() + command.point_ids.size();
  restored.xyz.reserve(restored_count * 3);
  const bool has_color = data->Has_Color() || !command.rgb.empty();
  if (has_color)
  {
    restored.rgb.reserve(restored_count * 3);
  }

  std::size_t remaining_index = 0;
  std::size_t deleted_index = 0;
  for (std::size_t point_index = 0; point_index < restored_count; ++point_index)
  {
    const bool restore_deleted =
      deleted_index < command.point_ids.size() &&
      command.point_ids[deleted_index] == point_index;
    const auto &xyz = restore_deleted ? command.xyz : data->xyz;
    const std::size_t source_index =
      restore_deleted ? deleted_index : remaining_index;
    restored.xyz.insert(
      restored.xyz.end(),
      xyz.begin() + static_cast<std::ptrdiff_t>(source_index * 3),
      xyz.begin() + static_cast<std::ptrdiff_t>(source_index * 3 + 3));

    if (has_color)
    {
      const auto &rgb = restore_deleted ? command.rgb : data->rgb;
      restored.rgb.insert(
        restored.rgb.end(),
        rgb.begin() + static_cast<std::ptrdiff_t>(source_index * 3),
        rgb.begin() + static_cast<std::ptrdiff_t>(source_index * 3 + 3));
    }

    if (restore_deleted)
    {
      ++deleted_index;
    }
    else
    {
      ++remaining_index;
    }
  }
  *data = std::move(restored);
}

std::size_t Point_Cloud_Document::Delete_Selected()
{
  Normalize_Ids(&m_selection, Point_Count());
  if (m_selection.empty())
  {
    return 0;
  }

  Delete_Command command;
  command.point_ids = m_selection;
  command.xyz.reserve(command.point_ids.size() * 3);
  if (m_data.Has_Color())
  {
    command.rgb.reserve(command.point_ids.size() * 3);
  }
  for (const std::size_t point_id : command.point_ids)
  {
    command.xyz.insert(
      command.xyz.end(),
      m_data.xyz.begin() + static_cast<std::ptrdiff_t>(point_id * 3),
      m_data.xyz.begin() + static_cast<std::ptrdiff_t>(point_id * 3 + 3));
    if (m_data.Has_Color())
    {
      command.rgb.insert(
        command.rgb.end(),
        m_data.rgb.begin() + static_cast<std::ptrdiff_t>(point_id * 3),
        m_data.rgb.begin() + static_cast<std::ptrdiff_t>(point_id * 3 + 3));
    }
  }

  const std::size_t deleted_count = command.point_ids.size();
  Apply_Delete(&m_data, command.point_ids);
  m_selection.clear();
  m_redo_stack.clear();
  m_undo_stack.push_back(std::move(command));
  if (m_undo_stack.size() > kMaximumUndoDepth)
  {
    m_undo_stack.erase(m_undo_stack.begin());
  }
  return deleted_count;
}

bool Point_Cloud_Document::Undo()
{
  if (m_undo_stack.empty())
  {
    return false;
  }
  Delete_Command command = std::move(m_undo_stack.back());
  m_undo_stack.pop_back();
  Restore_Delete(&m_data, command);
  m_selection = command.point_ids;
  m_redo_stack.push_back(std::move(command));
  return true;
}

bool Point_Cloud_Document::Redo()
{
  if (m_redo_stack.empty())
  {
    return false;
  }
  Delete_Command command = std::move(m_redo_stack.back());
  m_redo_stack.pop_back();
  Apply_Delete(&m_data, command.point_ids);
  m_selection.clear();
  m_undo_stack.push_back(std::move(command));
  return true;
}

} // namespace point_cloud
