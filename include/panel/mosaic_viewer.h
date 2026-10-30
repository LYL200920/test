#ifndef includeguard_mosaic_viewer_h_includeguard
#define includeguard_mosaic_viewer_h_includeguard

#include <filesystem>
#include <string>
#include <vector>

struct Mosaic_Viewer_Item
{
  std::string name;
  std::filesystem::path path;
  std::string error;
};

class wxWindow;

// Opens a modeless, full-resolution viewer. An empty path prompts for a PNG.
void Show_Mosaic_Viewer(wxWindow *parent,
                        const std::filesystem::path &path = {});

void Show_Mosaic_Group_Viewer(wxWindow *parent,
  std::vector<Mosaic_Viewer_Item> groups, std::size_t selected = 0);

#endif
