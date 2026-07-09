#ifndef includeguard_trajectory_control_panel_h_includeguard
#define includeguard_trajectory_control_panel_h_includeguard

#include <wx/panel.h>
#include <wx/string.h>

#include <cstddef>
#include <functional>
#include <vector>

class wxButton;
class wxListBox;
class wxSlider;
class wxStaticText;

class Trajectory_Control_Panel : public wxPanel
{
public:
  struct Callbacks
  {
    std::function<void ( )> add_point;
    std::function<void ( )> clear_points;
    std::function<void ( )> go_to_point;
    std::function<void ( )> delete_point;
    std::function<void ( )> save;
    std::function<void ( )> load;
    std::function<void ( )> play;
    std::function<void ( )> pause_resume;
    std::function<void ( )> stop;
    std::function<void ( )> speed_changed;
    std::function<void ( )> selection_changed;
  };

  Trajectory_Control_Panel (wxWindow* parent,
                            int speed_default_index,
                            int speed_max_index);

  void Set_Callbacks (Callbacks callbacks);

  int Selected_Point_Index ( ) const;
  void Set_Point_Labels (const std::vector<wxString>& labels);
  void Set_Point_Selection (int selection);

  int Speed_Index ( ) const;
  void Set_Speed_Label (const wxString& label);
  void Set_Status_Text (const wxString& label);

  void Refresh_Command_State (bool active,
                              bool paused,
                              bool playing,
                              size_t point_count);

private:
  void Bind_Button (wxButton* button, const std::function<void ( )>& callback);

private:
  Callbacks m_callbacks;

  wxButton* m_add_point_button = nullptr;
  wxButton* m_clear_points_button = nullptr;
  wxButton* m_go_to_point_button = nullptr;
  wxButton* m_delete_point_button = nullptr;
  wxButton* m_save_button = nullptr;
  wxButton* m_load_button = nullptr;
  wxButton* m_play_button = nullptr;
  wxButton* m_pause_resume_button = nullptr;
  wxButton* m_stop_button = nullptr;
  wxSlider* m_speed_slider = nullptr;
  wxStaticText* m_speed_label = nullptr;
  wxStaticText* m_status_text = nullptr;
  wxListBox* m_point_list = nullptr;
};

#endif
