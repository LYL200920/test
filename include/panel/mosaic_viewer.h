#ifndef includeguard_mosaic_viewer_h_includeguard
#define includeguard_mosaic_viewer_h_includeguard

#include <filesystem>

class wxWindow;

// Opens a modeless, full-resolution viewer. An empty path prompts for a PNG.
void Show_Mosaic_Viewer(wxWindow *parent,
                        const std::filesystem::path &path = {});

#endif
