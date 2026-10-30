#include "mosaic_preview_panel.h"
#include "camera_2d_image_view.h"
#include "app_paths.h"
#include <wx/button.h>
#include <wx/choice.h>
#include <wx/filedlg.h>
#include <wx/image.h>
#include <wx/log.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

Mosaic_Preview_Panel::Mosaic_Preview_Panel(wxWindow *parent) : wxPanel(parent)
{
  auto *root = new wxBoxSizer(wxVERTICAL);
  auto *tools = new wxBoxSizer(wxHORIZONTAL);
  auto *fit = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"适应窗口"));
  auto *open = new wxButton(this, wxID_ANY, wxString::FromUTF8(u8"打开图片"));
  tools->Add(fit, 0, wxRIGHT, 4); tools->Add(open, 0, wxRIGHT, 4);
  m_groups = new wxChoice(this, wxID_ANY);
  tools->Add(m_groups, 1, wxEXPAND);
  root->Add(tools, 0, wxEXPAND | wxALL, 4);
  m_canvas = new Camera_2D_Bitmap_Canvas(this);
  root->Add(m_canvas, 1, wxEXPAND);
  m_status = new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"尚无拼图；滚轮缩放，右键拖动"));
  root->Add(m_status, 0, wxEXPAND | wxALL, 4);
  SetSizer(root);
  fit->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_canvas->Fit_To_Window(); });
  m_groups->Bind(wxEVT_CHOICE, [this](wxCommandEvent &)
  { Select_Group(static_cast<std::size_t>(m_groups->GetSelection())); });
  open->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
  {
    wxFileDialog dialog(this, wxString::FromUTF8(u8"打开拼图"),
      wxString(application::Get_App_Paths().run_image_root.wstring()), "",
      "PNG (*.png)|*.png", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dialog.ShowModal() != wxID_OK) return;
    const std::filesystem::path path(dialog.GetPath().ToStdWstring());
    m_items.push_back({path.filename().u8string(), path, {}});
    m_groups->Append(wxString::FromUTF8(m_items.back().name));
    Select_Group(m_items.size()-1);
  });
}
void Mosaic_Preview_Panel::Set_Groups(std::vector<Mosaic_Viewer_Item> groups)
{
  m_items = std::move(groups); m_groups->Clear(); m_canvas->Clear_Bitmap();
  for (const auto &item : m_items) m_groups->Append(wxString::FromUTF8(item.name));
  if (!m_items.empty()) Select_Group(0);
  else m_status->SetLabel(wxString::FromUTF8(u8"尚无拼图"));
}
void Mosaic_Preview_Panel::Select_Group(std::size_t index)
{
  if (index >= m_items.size()) return;
  m_groups->SetSelection(static_cast<int>(index));
  m_canvas->Clear_Bitmap();
  const auto &item = m_items[index];
  wxImage image;
  bool ok = false;
  if (!item.path.empty())
  {
    wxLogNull suppress;
    ok = image.LoadFile(wxString(item.path.wstring()), wxBITMAP_TYPE_PNG);
  }
  if (!ok || !image.IsOk())
  {
    m_status->SetLabel(wxString::FromUTF8(item.error.empty() ? "图片读取失败" : item.error));
    m_status->SetToolTip(m_status->GetLabel());
    return;
  }
  m_canvas->Set_Bitmap(wxBitmap(image)); m_canvas->Fit_To_Window();
  m_status->SetLabel(wxString::Format("%d x %d | ", image.GetWidth(), image.GetHeight()) +
    wxString::FromUTF8(item.name));
  m_status->SetToolTip(wxString(item.path.wstring()));
}
