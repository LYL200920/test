#ifndef includeguard_teach_point_command_panel_h_includeguard
#define includeguard_teach_point_command_panel_h_includeguard

#include "robot_teach_point.h"

#include <wx/panel.h>

#include <functional>
#include <cstddef>

class wxButton;
class wxChoice;
class wxSlider;
class wxStaticText;

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
    std::function<void()> complete;
    std::function<void()> step_back;
    std::function<void()> step_next;
  };

  explicit Teach_Point_Command_Panel (wxWindow* parent);

  void Set_Callbacks(Callbacks callbacks);
  robot_model::Robot_Teach_Point_Type Selected_Point_Type() const;
  void Set_Selected_Point_Type(robot_model::Robot_Teach_Point_Type type);
  int Step_Speed_Percent() const;
  void Refresh_Command_State(
    bool enabled,
    std::size_t selected_count,
    std::size_t point_count,
    bool can_step_back,
    bool can_step_next);

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
  wxButton* m_complete_button = nullptr;
  wxButton* m_step_back_button = nullptr;
  wxButton* m_step_next_button = nullptr;
  wxChoice* m_type_choice = nullptr;
  wxSlider* m_step_speed_slider = nullptr;
  wxStaticText* m_step_speed_label = nullptr;
};

#endif
