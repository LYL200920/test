#ifndef includeguard_mosaic_preview_panel_h
#define includeguard_mosaic_preview_panel_h
#include "mosaic_viewer.h"
#include <wx/panel.h>
class wxChoice;
class wxStaticText;
class Camera_2D_Bitmap_Canvas;
class Mosaic_Preview_Panel : public wxPanel
{
public:
  explicit Mosaic_Preview_Panel(wxWindow *parent);
  void Set_Groups(std::vector<Mosaic_Viewer_Item> groups);
  void Select_Group(std::size_t index);
private:
  wxChoice *m_groups = nullptr;
  wxStaticText *m_status = nullptr;
  Camera_2D_Bitmap_Canvas *m_canvas = nullptr;
  std::vector<Mosaic_Viewer_Item> m_items;
};
#endif
