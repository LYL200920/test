#ifndef includeguard_mosaic_settings_panel_h_includeguard
#define includeguard_mosaic_settings_panel_h_includeguard

#include "mosaic_configuration.h"
#include <wx/scrolwin.h>
#include <functional>
#include <memory>

class Mosaic_Settings_Panel : public wxScrolledWindow
{
public:
  explicit Mosaic_Settings_Panel(wxWindow *parent);
  ~Mosaic_Settings_Panel() override;
  void Set_Context(const std::string &model,
    const std::vector<robot_model::Robot_Teach_Point> &points,
    const robot_model::Mosaic_Configuration &configuration, bool reset = false);
  void Set_On_Apply(std::function<bool(const robot_model::Mosaic_Configuration &)> callback);
  void Set_On_Preview(std::function<void(const robot_model::Mosaic_Configuration &, std::size_t)> callback);
private:
  class Editor;
  Editor *m_editor = nullptr;
};
#endif
