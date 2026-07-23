#include "camera_acquisition_panel.h"

#include "camera_frame.h"
#include "camera_manager.h"

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

#include <utility>

namespace
{
  wxString From_Utf8(const char *text)
  {
    return wxString::FromUTF8(text);
  }

  wxString From_Utf8(const std::string &text)
  {
    return wxString::FromUTF8(text.c_str());
  }

  wxString State_Label(Camera_State state)
  {
    switch (state)
    {
    case Camera_State::Uninitialized:
      return From_Utf8(u8"未初始化");
    case Camera_State::Initialized:
      return From_Utf8(u8"已初始化");
    case Camera_State::Opened:
      return From_Utf8(u8"已打开");
    case Camera_State::Grabbing:
      return From_Utf8(u8"采集中");
    case Camera_State::Error:
      return From_Utf8(u8"错误");
    }
    return From_Utf8(u8"未知");
  }

  wxString Format_Stream_Configuration(const std::vector<Camera_Stream_Config> &streams, bool loaded)
  {
    if (!loaded)
    {
      return From_Utf8(u8"尚未读取流配置");
    }
    if (streams.empty())
    {
      return From_Utf8(u8"当前配置未报告输出流");
    }

    wxString text = wxString::Format(From_Utf8(u8"预计输出 %zu 路："), streams.size());
    for (std::size_t index = 0; index < streams.size(); ++index)
    {
      const auto &stream = streams[index];
      text += wxString::Format(
          "\n%zu. %s  %ux%u  [0x%08X]",
          index + 1,
          From_Utf8(Camera_Image_Type_Name(stream.image_type)),
          stream.width,
          stream.height,
          stream.image_type);
    }
    return text;
  }
} // namespace

Camera_Acquisition_Panel::Camera_Acquisition_Panel(wxWindow *parent)
    : wxScrolledWindow(parent, wxID_ANY)
{
  SetScrollRate(0, 10);

  auto *title = new wxStaticText(this, wxID_ANY, From_Utf8(u8"采集控制"));
  m_device_text = new wxStaticText(this, wxID_ANY, From_Utf8(u8"设备：未打开"));
  m_state_text = new wxStaticText(this, wxID_ANY, From_Utf8(u8"状态：未初始化"));
  m_stream_config_text = new wxStaticText(this, wxID_ANY, From_Utf8(u8"尚未读取流配置"));
  m_frame_text = new wxStaticText(this, wxID_ANY, From_Utf8(u8"帧数据：未采集"));

  m_refresh_streams_button = new wxButton(this, wxID_ANY, From_Utf8(u8"刷新流配置"));
  m_start_button = new wxButton(this, wxID_ANY, From_Utf8(u8"开始采集"));
  m_stop_button = new wxButton(this, wxID_ANY, From_Utf8(u8"停止采集"));
  m_close_button = new wxButton(this, wxID_ANY, From_Utf8(u8"关闭相机"));

  m_start_button->Bind(wxEVT_BUTTON, &Camera_Acquisition_Panel::On_Start, this);
  m_stop_button->Bind(wxEVT_BUTTON, &Camera_Acquisition_Panel::On_Stop, this);
  m_close_button->Bind(wxEVT_BUTTON, &Camera_Acquisition_Panel::On_Close, this);
  m_refresh_streams_button->Bind(wxEVT_BUTTON, &Camera_Acquisition_Panel::On_Refresh_Streams, this);

  auto *acquisition_buttons = new wxBoxSizer(wxHORIZONTAL);
  acquisition_buttons->Add(m_start_button, 1, wxEXPAND | wxRIGHT, 4);
  acquisition_buttons->Add(m_stop_button, 1, wxEXPAND);

  auto *frame_sizer = new wxStaticBoxSizer(wxVERTICAL, this, From_Utf8(u8"采集数据"));
  frame_sizer->Add(m_frame_text, 0, wxEXPAND | wxALL, 8);

  auto *stream_sizer = new wxStaticBoxSizer(wxVERTICAL, this, From_Utf8(u8"预计输出流"));
  stream_sizer->Add(m_stream_config_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  stream_sizer->Add(m_refresh_streams_button, 0, wxEXPAND | wxALL, 8);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxEXPAND | wxALL, 8);
  sizer->Add(m_device_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(m_state_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(stream_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(acquisition_buttons, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(m_close_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add(frame_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->AddStretchSpacer(1);
  SetSizer(sizer);
}

void Camera_Acquisition_Panel::Set_Callbacks(Callbacks callbacks)
{
  m_callbacks = std::move(callbacks);
}

void Camera_Acquisition_Panel::Update_View(
    Camera_State state,
    bool device_open,
    bool busy,
    const std::string &serial_number,
    const std::vector<Camera_Stream_Config> &stream_configuration,
    bool stream_configuration_loaded)
{
  m_device_text->SetLabel(serial_number.empty() ? From_Utf8(u8"设备：未选择")
                                                : From_Utf8(u8"设备：") + From_Utf8(serial_number));
  m_state_text->SetLabel(From_Utf8(u8"状态：") + State_Label(state));
  m_start_button->Enable(!busy && state == Camera_State::Opened);
  m_stop_button->Enable(!busy && state == Camera_State::Grabbing);
  m_close_button->Enable(!busy && device_open);
  m_refresh_streams_button->Enable(!busy && device_open && state == Camera_State::Opened);
  m_stream_config_text->SetLabel(Format_Stream_Configuration(stream_configuration, stream_configuration_loaded));

  m_stream_config_text->Wrap(350);

  if (!device_open)
  {
    m_frame_text->SetLabel(From_Utf8(u8"帧数据：未采集"));
    m_last_generation = 0;
    m_last_frame_time = {};
    m_measured_fps = 0.0;
  }
  Layout();
  FitInside();
}

void Camera_Acquisition_Panel::Update_Frame(const std::shared_ptr<const Camera_Frame> &frame)
{
  if (!frame)
  {
    m_frame_text->SetLabel(From_Utf8(u8"帧数据：等待采集"));
    m_last_generation = 0;
    m_last_frame_time = {};
    m_measured_fps = 0.0;
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (m_last_generation == 0)
  {
    m_last_generation = frame->generation;
    m_last_frame_time = now;
  }
  else if (frame->generation > m_last_generation && m_last_frame_time.time_since_epoch().count() != 0)
  {
    const auto seconds = std::chrono::duration<double>(now - m_last_frame_time).count();
    if (seconds > 0.0)
    {
      const auto sample = static_cast<double>(frame->generation - m_last_generation) / seconds;
      m_measured_fps = m_measured_fps <= 0.0 ? sample
                                             : m_measured_fps * 0.75 + sample * 0.25;
    }
    m_last_generation = frame->generation;
    m_last_frame_time = now;
  }

  wxString text = wxString::Format(
      From_Utf8(u8"缓存帧序号：#%llu\n图像数量：%zu\n测量帧率：%.2f fps"
                u8"\n有效标记：0x%08X\n触发ID：%u"),
      static_cast<unsigned long long>(frame->generation),
      frame->images.size(),
      m_measured_fps,
      frame->valid_info,
      frame->trigger_id);

  for (std::size_t index = 0; index < frame->images.size(); ++index)
  {
    const auto &image = frame->images[index];
    text += wxString::Format(
        From_Utf8(u8"\n\n%zu. %s / %s\n%ux%u，Frame=%u，Bytes=%zu"
                  u8"\nTimestamp=%lld，Coordinate=%s，Rectified=%s"),
        index + 1,
        From_Utf8(Camera_Image_Type_Name(image.image_type)),
        From_Utf8(Camera_Stream_Type_Name(image.stream_type)),
        image.width,
        image.height,
        image.frame_number,
        image.data.size(),
        static_cast<long long>(image.timestamp),
        From_Utf8(Camera_Coordinate_Type_Name(image.coordinate_type)),
        image.is_rectified ? "yes" : "no");
  }
  m_frame_text->SetLabel(text);
  m_frame_text->Wrap(350);
  Layout();
  FitInside();
}

void Camera_Acquisition_Panel::On_Start(wxCommandEvent &)
{
  if (m_callbacks.start)
  {
    m_callbacks.start();
  }
}

void Camera_Acquisition_Panel::On_Stop(wxCommandEvent &)
{
  if (m_callbacks.stop)
  {
    m_callbacks.stop();
  }
}

void Camera_Acquisition_Panel::On_Close(wxCommandEvent &)
{
  if (m_callbacks.close)
  {
    m_callbacks.close();
  }
}

void Camera_Acquisition_Panel::On_Refresh_Streams(wxCommandEvent &)
{
  if (m_callbacks.refresh_streams)
  {
    m_callbacks.refresh_streams();
  }
}
