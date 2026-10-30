#include "mosaic_viewer.h"

#include "app_paths.h"
#include "camera_2d_image_view.h"

#include <wx/button.h>
#include <wx/filedlg.h>
#include <wx/frame.h>
#include <wx/image.h>
#include <wx/log.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

namespace
{
bool Select_Mosaic(wxWindow *parent, std::filesystem::path *path)
{
  const auto directory = path->empty()
    ? application::Get_App_Paths().run_image_root : path->parent_path();
  wxFileDialog dialog(parent, wxString::FromUTF8(u8"打开已保存的拼图"),
    wxString(directory.wstring()), "mosaic.png",
    wxString::FromUTF8(u8"PNG 图片 (*.png)|*.png"),
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if (dialog.ShowModal() != wxID_OK) return false;
  *path = std::filesystem::path(dialog.GetPath().ToStdWstring());
  return true;
}

class Mosaic_Viewer : public wxFrame
{
public:
  explicit Mosaic_Viewer(wxWindow *parent)
    : wxFrame(parent, wxID_ANY, wxString::FromUTF8(u8"拼图查看器"),
              wxDefaultPosition, wxSize(1100, 800))
  {
    auto *panel = new wxPanel(this);
    auto *layout = new wxBoxSizer(wxVERTICAL);
    auto *tools = new wxBoxSizer(wxHORIZONTAL);
    auto *open = new wxButton(panel, wxID_OPEN, wxString::FromUTF8(u8"打开拼图..."));
    auto *zoom_in = new wxButton(panel, wxID_ZOOM_IN, wxString::FromUTF8(u8"放大 +"));
    auto *zoom_out = new wxButton(panel, wxID_ZOOM_OUT, wxString::FromUTF8(u8"缩小 −"));
    auto *fit = new wxButton(panel, wxID_ZOOM_FIT, wxString::FromUTF8(u8"适应窗口"));
    for (auto *button : {open, zoom_in, zoom_out, fit})
      tools->Add(button, 0, wxRIGHT, 8);
    layout->Add(tools, 0, wxALL, 8);
    layout->Add(new wxStaticText(panel, wxID_ANY,
      wxString::FromUTF8(u8"滚轮缩放 · 按住鼠标右键拖动平移 · 查看的是保存的完整分辨率图片")),
      0, wxLEFT | wxRIGHT | wxBOTTOM, 8);
    m_canvas = new Camera_2D_Bitmap_Canvas(panel);
    layout->Add(m_canvas, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
    panel->SetSizer(layout);
    auto *frame_layout = new wxBoxSizer(wxVERTICAL);
    frame_layout->Add(panel, 1, wxEXPAND);
    SetSizer(frame_layout);
    CreateStatusBar();
    SetMinSize(wxSize(640, 480));
    open->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
    {
      auto path = m_path;
      if (Select_Mosaic(this, &path)) Load(path);
    });
    zoom_in->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_canvas->Zoom_In(); });
    zoom_out->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_canvas->Zoom_Out(); });
    fit->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_canvas->Fit_To_Window(); });
  }

  bool Load(const std::filesystem::path &path)
  {
    wxImage image;
    bool loaded = false;
    {
      wxLogNull suppress_decoder_errors;
      loaded = image.LoadFile(wxString(path.wstring()), wxBITMAP_TYPE_PNG);
    }
    if (!loaded || !image.IsOk())
    {
      wxMessageBox(wxString::FromUTF8(u8"无法读取拼图文件，请确认文件存在且为有效的 PNG 图片。\n") +
                     wxString(path.wstring()),
                   wxString::FromUTF8(u8"打开拼图失败"), wxOK | wxICON_ERROR, this);
      return false;
    }
    wxBitmap bitmap(image);
    if (!bitmap.IsOk())
    {
      wxMessageBox(wxString::FromUTF8(u8"无法创建拼图显示位图，可能内存不足。"),
                   wxString::FromUTF8(u8"打开拼图失败"), wxOK | wxICON_ERROR, this);
      return false;
    }
    m_path = path;
    m_canvas->Set_Bitmap(std::move(bitmap));
    m_canvas->Fit_To_Window();
    SetTitle(wxString::FromUTF8(u8"拼图查看器 — ") + wxString(path.filename().wstring()));
    SetStatusText(wxString::Format("%d × %d  |  ", image.GetWidth(), image.GetHeight()) +
                  wxString(path.wstring()));
    return true;
  }

private:
  Camera_2D_Bitmap_Canvas *m_canvas = nullptr;
  std::filesystem::path m_path;
};
} // namespace

void Show_Mosaic_Viewer(wxWindow *parent, const std::filesystem::path &path)
{
  auto selected = path;
  if (selected.empty() && !Select_Mosaic(parent, &selected)) return;
  auto *viewer = new Mosaic_Viewer(parent);
  if (!viewer->Load(selected))
  {
    viewer->Destroy();
    return;
  }
  viewer->CentreOnParent();
  viewer->Show();
}
