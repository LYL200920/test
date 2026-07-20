#ifndef includeguard_teach_point_list_panel_h_includeguard
#define includeguard_teach_point_list_panel_h_includeguard

#include <wx/panel.h>
#include <wx/string.h>

#include <functional>
#include <vector>

class wxButton;
class wxListBox;
class wxStaticText;

class Teach_Point_List_Panel : public wxPanel
{
public:
  explicit Teach_Point_List_Panel (wxWindow* parent);

  void Set_Point_Names (const std::vector<wxString>& names);
  int Selected_Point_Index ( ) const;
  void Set_Point_Selection (int selection);
  void Set_List_Enabled (bool enabled);
  void Set_On_Selection_Changed (std::function<void ( )> callback);
  void Set_On_Collapsed_Changed (
    std::function<void (bool)> callback);

private:
  void Toggle_Collapsed ( );
  void Update_Collapsed_State ( );

private:
  wxStaticText* m_title = nullptr;
  wxButton* m_toggle_button = nullptr;
  wxListBox* m_point_list = nullptr;
  std::function<void ( )> m_on_selection_changed;
  std::function<void (bool)> m_on_collapsed_changed;
  bool m_collapsed = false;
};

#endif
