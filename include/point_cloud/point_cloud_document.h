#ifndef includeguard_point_cloud_document_h_includeguard
#define includeguard_point_cloud_document_h_includeguard

#include "point_cloud_data.h"

#include <cstddef>
#include <vector>

namespace point_cloud
{

enum class Point_Selection_Mode
{
  Replace,
  Add,
  Toggle
};

class Point_Cloud_Document
{
public:
  void Set_Data(Point_Cloud_Data data);
  void Clear();

  const Point_Cloud_Data &Data() const { return m_data; }
  const std::vector<std::size_t> &Selection() const { return m_selection; }
  std::size_t Point_Count() const { return m_data.Point_Count(); }

  void Set_Selection(std::vector<std::size_t> point_ids,
                     Point_Selection_Mode mode = Point_Selection_Mode::Replace);
  void Clear_Selection();

  std::size_t Delete_Selected();
  bool Can_Undo() const { return !m_undo_stack.empty(); }
  bool Can_Redo() const { return !m_redo_stack.empty(); }
  bool Undo();
  bool Redo();

private:
  struct Delete_Command
  {
    std::vector<std::size_t> point_ids;
    std::vector<float> xyz;
    std::vector<std::uint8_t> rgb;
  };

  static void Normalize_Ids(std::vector<std::size_t> *point_ids,
                            std::size_t point_count);
  static void Apply_Delete(Point_Cloud_Data *data,
                           const std::vector<std::size_t> &point_ids);
  static void Restore_Delete(Point_Cloud_Data *data,
                             const Delete_Command &command);

private:
  Point_Cloud_Data m_data;
  std::vector<std::size_t> m_selection;
  std::vector<Delete_Command> m_undo_stack;
  std::vector<Delete_Command> m_redo_stack;
};

} // namespace point_cloud

#endif
