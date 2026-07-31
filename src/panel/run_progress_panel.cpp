#include "run_progress_panel.h"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <algorithm>
#include <cstdint>
#include <utility>

namespace
{
const wxColour READY_GREEN(35, 155, 70);
const wxColour STOP_RED(205, 55, 55);
const wxColour TEXT_WHITE(255, 255, 255);
}

Run_Progress_Panel::Run_Progress_Panel(wxWindow *parent)
  : wxPanel(parent, wxID_ANY)
{
  auto *title = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"Progress 连续运行"));
  wxFont title_font = title->GetFont();
  title_font.MakeBold();
  title->SetFont(title_font);

  auto *ct_label = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"运行 CT 时间"));
  m_ct_value = new wxStaticText(this, wxID_ANY, "00:00.000");
  wxFont ct_font = m_ct_value->GetFont();
  ct_font.SetPointSize(std::max(ct_font.GetPointSize() + 6, 16));
  ct_font.MakeBold();
  m_ct_value->SetFont(ct_font);

  m_save_images = new wxCheckBox(
    this, wxID_ANY, wxString::FromUTF8(u8"保存每个到位点的 2D 图片"));

  m_run_button = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"请先复位"));
  m_run_button->SetMinSize(wxSize(-1, 58));
  m_run_button->Bind(
    wxEVT_BUTTON,
    [this](wxCommandEvent &)
    {
      if (m_running)
      {
        if (m_callbacks.emergency_stop)
          m_callbacks.emergency_stop();
      }
      else if (m_progress_ready && m_robot_ready && m_callbacks.start)
      {
        m_callbacks.start();
      }
    });

  m_status = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"请先在 Teach 中完成 Progress"));
  m_status->Wrap(360);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxEXPAND | wxALL, 10);
  sizer->Add(ct_label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 10);
  sizer->Add(m_ct_value, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 14);
  sizer->Add(m_save_images, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);
  sizer->Add(m_run_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);
  sizer->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 10);
  sizer->AddStretchSpacer(1);
  SetSizer(sizer);
  Refresh_Run_Button();
}

void Run_Progress_Panel::Set_Callbacks(Callbacks callbacks)
{
  m_callbacks = std::move(callbacks);
}

bool Run_Progress_Panel::Save_Images() const
{
  return m_save_images && m_save_images->GetValue();
}

void Run_Progress_Panel::Set_Progress_Ready(bool ready)
{
  m_progress_ready = ready;
  Refresh_Run_Button();
}

void Run_Progress_Panel::Set_Robot_Ready(
  bool ready,
  const std::string &reason)
{
  m_robot_ready = ready;
  m_robot_reason = reason;
  Refresh_Run_Button();
}

void Run_Progress_Panel::Set_Running(bool running, bool stopping)
{
  m_running = running;
  m_stopping = running && stopping;
  if (m_save_images)
    m_save_images->Enable(!running);
  Refresh_Run_Button();
}

void Run_Progress_Panel::Set_Elapsed(std::chrono::milliseconds elapsed)
{
  const auto total_ms = std::max<std::int64_t>(elapsed.count(), 0);
  if (m_ct_value)
  {
    m_ct_value->SetLabel(wxString::Format(
      "%02lld:%02lld.%03lld",
      static_cast<long long>(total_ms / 60000),
      static_cast<long long>((total_ms / 1000) % 60),
      static_cast<long long>(total_ms % 1000)));
  }
}

void Run_Progress_Panel::Set_Status(
  const std::string &status,
  bool error)
{
  if (!m_status)
    return;
  m_status->SetLabel(wxString::FromUTF8(status));
  m_status->SetForegroundColour(error ? STOP_RED : wxNullColour);
  m_status->Wrap(360);
  Layout();
}

void Run_Progress_Panel::Refresh_Run_Button()
{
  if (!m_run_button)
    return;
  if (m_running)
  {
    m_run_button->SetLabel(
      m_stopping
        ? wxString::FromUTF8(u8"正在急停...")
        : wxString::FromUTF8(u8"紧急停止"));
    m_run_button->SetBackgroundColour(STOP_RED);
    m_run_button->SetForegroundColour(TEXT_WHITE);
    m_run_button->Enable(!m_stopping);
    return;
  }

  const bool ready = m_progress_ready && m_robot_ready;
  m_run_button->SetLabel(
    ready ? wxString::FromUTF8(u8"运行")
          : wxString::FromUTF8(u8"请先复位"));
  m_run_button->SetBackgroundColour(ready ? READY_GREEN : STOP_RED);
  m_run_button->SetForegroundColour(TEXT_WHITE);
  m_run_button->Enable(m_progress_ready);
  if (!m_progress_ready)
    Set_Status("请先在 Teach 中完成 Progress");
  else if (!m_robot_ready && !m_robot_reason.empty())
    Set_Status(m_robot_reason, true);
  else if (ready)
    Set_Status("机械臂已在复位位置，可以运行");
}
