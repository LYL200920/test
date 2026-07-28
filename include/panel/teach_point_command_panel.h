#ifndef includeguard_teach_point_command_panel_h_includeguard
#define includeguard_teach_point_command_panel_h_includeguard

#include "robot_teach_point.h"

#include <wx/panel.h>

#include <functional>
#include <cstddef>

class wxButton;
class wxChoice;

class Teach_Point_Command_Panel : public wxPanel
{
public:
  struct Callbacks
  {
    std::function<void()> add;
    std::function<void()> update;
    std::function<void()> insert_before;
    std::function<void()> insert_after;
    std::function<void()> delete_selected;
    std::function<void()> clear;
    std::function<void()> save;
    std::function<void()> load;
  };

  explicit Teach_Point_Command_Panel (wxWindow* parent);

  void Set_Callbacks(Callbacks callbacks);
  robot_model::Robot_Teach_Point_Type Selected_Point_Type() const;
  void Set_Selected_Point_Type(robot_model::Robot_Teach_Point_Type type);
  void Refresh_Command_State(
    bool enabled,
    std::size_t selected_count,
    std::size_t point_count);

private:
  void Bind_Button(
    wxButton *button,
    const std::function<void()> &callback);

private:
  Callbacks m_callbacks;
  wxButton* m_add_button = nullptr;
  wxButton* m_update_button = nullptr;
  wxButton* m_insert_before_button = nullptr;
  wxButton* m_insert_after_button = nullptr;
  wxButton* m_delete_button = nullptr;
  wxButton* m_clear_button = nullptr;
  wxButton* m_save_button = nullptr;
  wxButton* m_load_button = nullptr;
  wxChoice* m_type_choice = nullptr;
};

#endif
