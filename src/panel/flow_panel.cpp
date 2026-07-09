#include "flow_panel.h"

#include <wx/button.h>
#include <wx/combobox.h>
#include <wx/datetime.h>
#include <wx/file.h>
#include <wx/fileconf.h>
#include <wx/filename.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/stdpaths.h>
#include <wx/tokenzr.h>

#include <algorithm>
#include <vector>

namespace
{
constexpr int WAIT_TIMEOUT_MS = 60000;   // 每步等待超时 60s
constexpr size_t MAX_LOG_MSG_LEN = 2000;

const wxString HIK_NAME = "HIK";
const wxString KUKA_NAME = "KUKA";
const wxString HIK_DEFAULT_IP = "192.168.1.45";
const wxString HIK_DEFAULT_PORT = "7930";
const wxString KUKA_DEFAULT_IP = "192.168.1.25";
const wxString KUKA_DEFAULT_PORT = "54600";

const wxString RECV_DELIMITER = "\r\n";
// HIK 回复终止符：async_read_some 是字节流，一条回复可能分多次到达，
// HIK 回复以 ';' 结尾，按 ';' 重组为完整消息后再转发给 KUKA。
const char HIK_PACKET_TERM = ';';

const wxString FLOW_CONFIG_SECTION = "/Flow";
const wxString FLOW_CONFIG_KEY = "last_message";

wxString Trim_Value (wxString value)
{
  value.Trim (true);
  value.Trim (false);
  return value;
}
}

// 流程内部 recv 事件：携带一条完整消息投递到 UI 线程
wxDEFINE_EVENT(wxEVT_FLOW_HIK_RECV, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_FLOW_KUKA_RECV, wxThreadEvent);
// 状态事件（IO 线程 → UI 线程）
wxDEFINE_EVENT(wxEVT_FLOW_HIK_STATUS, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_FLOW_KUKA_STATUS, wxThreadEvent);

enum
{
  ID_CONNECT_BTN = 2001,
  ID_DISCONNECT_BTN,
  ID_RUN_BTN,
  ID_STOP_BTN,
  ID_CLEAR_BTN,
  ID_WAIT_TIMER
};

wxBEGIN_EVENT_TABLE(Flow_Panel, wxPanel)
    EVT_BUTTON(ID_CONNECT_BTN, Flow_Panel::On_Connect_Click)
        EVT_BUTTON(ID_DISCONNECT_BTN, Flow_Panel::On_Disconnect_Click)
            EVT_BUTTON(ID_RUN_BTN, Flow_Panel::On_Run_Click)
                EVT_BUTTON(ID_STOP_BTN, Flow_Panel::On_Stop_Click)
                    EVT_BUTTON(ID_CLEAR_BTN, Flow_Panel::On_Clear_Click)
                        EVT_TIMER(ID_WAIT_TIMER, Flow_Panel::On_Timeout)
                            wxEND_EVENT_TABLE()

                                Flow_Panel::Flow_Panel(wxWindow *parent, wxWindowID id)
    : wxPanel(parent, id)
{
  // 端点初始化
  m_hik.name = HIK_NAME;
  m_hik.default_ip = HIK_DEFAULT_IP;
  m_hik.default_port = HIK_DEFAULT_PORT;
  m_kuka.name = KUKA_NAME;
  m_kuka.default_ip = KUKA_DEFAULT_IP;
  m_kuka.default_port = KUKA_DEFAULT_PORT;

  Build_Ui();

  Bind(wxEVT_FLOW_HIK_RECV, &Flow_Panel::On_HIK_Recv_Event, this);
  Bind(wxEVT_FLOW_KUKA_RECV, &Flow_Panel::On_KUKA_Recv_Event, this);
  Bind(wxEVT_FLOW_HIK_STATUS, [this](wxThreadEvent &e) {
    Update_Status(m_hik, e.GetInt() != 0, std::string(e.GetString().mb_str(wxConvUTF8)));
  });
  Bind(wxEVT_FLOW_KUKA_STATUS, [this](wxThreadEvent &e) {
    Update_Status(m_kuka, e.GetInt() != 0, std::string(e.GetString().mb_str(wxConvUTF8)));
  });
  m_wait_timer.SetOwner(this, ID_WAIT_TIMER);

  // 创建两条独立连接（与 Net_Panel 完全隔离）
  m_hik.client = std::make_shared<tcp_client>();
  m_kuka.client = std::make_shared<tcp_client>();

  m_hik.client->set_status_callback([this](bool connected, const std::string &info) {
    auto* evt = new wxThreadEvent(wxEVT_FLOW_HIK_STATUS);
    evt->SetInt(connected ? 1 : 0);
    evt->SetString(wxString::FromUTF8(info));
    wxQueueEvent(this, evt);
  });
  m_hik.client->set_recv_callback(
    [this](const std::string &msg) { Feed_HIK_Recv(msg); });

  m_kuka.client->set_status_callback([this](bool connected, const std::string &info) {
    auto* evt = new wxThreadEvent(wxEVT_FLOW_KUKA_STATUS);
    evt->SetInt(connected ? 1 : 0);
    evt->SetString(wxString::FromUTF8(info));
    wxQueueEvent(this, evt);
  });
  m_kuka.client->set_recv_callback(
    [this](const std::string &msg) { Feed_KUKA_Recv(msg); });
}

Flow_Panel::~Flow_Panel()
{
  Stop_Wait_Timer();
  // client 为本面板私有，安全解绑并断开
  for (auto* ep : {&m_hik, &m_kuka})
  {
    if (ep->client)
    {
      ep->client->set_status_callback(nullptr);
      ep->client->set_recv_callback(nullptr);
      ep->client->disconnect();
      ep->client.reset();
    }
  }
  DeletePendingEvents();
}

void Flow_Panel::Build_Ui()
{
  m_config_path = wxFileName(wxStandardPaths::Get().GetExecutablePath()).GetPath() +
                  "/vtk_flow_config.ini";

  const wxString last_msg = Load_Config();

  auto* sizer = new wxBoxSizer(wxVERTICAL);

  auto build_endpoint = [this, sizer](Flow_Endpoint &ep) {
    auto* title = new wxStaticText(
      this, wxID_ANY, ep.name + wxString::FromUTF8(u8"（Server）"));
    wxFont tf = title->GetFont();
    tf.MakeBold();
    title->SetFont(tf);

    auto* ip_label = new wxStaticText(this, wxID_ANY, "IP:");
    ep.ip_input = new wxComboBox(this, wxID_ANY, ep.default_ip,
                                 wxDefaultPosition, wxDefaultSize, 0, nullptr, wxCB_DROPDOWN);
    ep.ip_input->SetMinSize(wxSize(150, -1));

    auto* port_label = new wxStaticText(this, wxID_ANY, "Port:");
    ep.port_input = new wxComboBox(this, wxID_ANY, ep.default_port,
                                   wxDefaultPosition, wxDefaultSize, 0, nullptr, wxCB_DROPDOWN);
    ep.port_input->SetMinSize(wxSize(80, -1));

    ep.connect_btn = new wxButton(this, ID_CONNECT_BTN, wxString::FromUTF8(u8"连接"));
    ep.disconnect_btn = new wxButton(this, ID_DISCONNECT_BTN, wxString::FromUTF8(u8"断开"));
    ep.disconnect_btn->Disable();

    ep.status_label = new wxStaticText(this, wxID_ANY,
                                       wxString::FromUTF8(u8"状态：未连接"));

    auto* grid = new wxFlexGridSizer(2, 2, 4, 6);
    grid->Add(ip_label, 0, wxALIGN_CENTER_VERTICAL);
    grid->Add(ep.ip_input, 1, wxEXPAND);
    grid->Add(port_label, 0, wxALIGN_CENTER_VERTICAL);
    grid->Add(ep.port_input, 1, wxEXPAND);
    grid->AddGrowableCol(1, 1);

    auto* btn_row = new wxBoxSizer(wxHORIZONTAL);
    btn_row->Add(ep.connect_btn, 1, wxEXPAND | wxRIGHT, 4);
    btn_row->Add(ep.disconnect_btn, 1, wxEXPAND);

    sizer->Add(title, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
    sizer->Add(grid, 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
    sizer->Add(btn_row, 0, wxEXPAND | wxLEFT | wxRIGHT, 4);
    sizer->Add(ep.status_label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  };

  build_endpoint(m_hik);
  build_endpoint(m_kuka);

  // ---- 发送内容 + 运行控制 ----
  auto* input_label = new wxStaticText(this, wxID_ANY,
    wxString::FromUTF8(u8"发送内容（发往 HIK）："));
  m_input = new wxTextCtrl(this, wxID_ANY, last_msg,
                           wxDefaultPosition, wxSize(-1, 60), wxTE_MULTILINE);

  m_run_btn = new wxButton(this, ID_RUN_BTN, wxString::FromUTF8(u8"▶ 运行流程"));
  m_stop_btn = new wxButton(this, ID_STOP_BTN, wxString::FromUTF8(u8"■ 停止"));
  m_stop_btn->Disable();

  m_flow_status_label = new wxStaticText(this, wxID_ANY,
                                         wxString::FromUTF8(u8"流程状态：空闲"));

  auto* btn_sizer = new wxBoxSizer(wxHORIZONTAL);
  btn_sizer->Add(m_run_btn, 1, wxEXPAND | wxRIGHT, 4);
  btn_sizer->Add(m_stop_btn, 1, wxEXPAND);

  sizer->Add(input_label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 4);
  sizer->Add(m_input, 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
  sizer->Add(btn_sizer, 0, wxEXPAND | wxALL, 8);
  sizer->Add(m_flow_status_label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 4);

  // ---- 流程日志 ----
  auto* log_title = new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"流程日志"));
  wxFont lf = log_title->GetFont();
  lf.MakeBold();
  log_title->SetFont(lf);

  m_log = new wxTextCtrl(this, wxID_ANY, wxEmptyString,
                         wxDefaultPosition, wxDefaultSize,
                         wxTE_MULTILINE | wxTE_READONLY | wxTE_RICH);
  wxFont mono;
  mono.SetFamily(wxFONTFAMILY_TELETYPE);
  m_log->SetFont(mono);

  m_clear_btn = new wxButton(this, ID_CLEAR_BTN, wxString::FromUTF8(u8"清空日志"));
  auto* clear_sizer = new wxBoxSizer(wxHORIZONTAL);
  clear_sizer->AddStretchSpacer(1);
  clear_sizer->Add(m_clear_btn, 0);

  sizer->Add(log_title, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 4);
  sizer->Add(m_log, 1, wxEXPAND | wxLEFT | wxRIGHT, 6);
  sizer->Add(clear_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  SetSizer(sizer);
}

Flow_Panel::Flow_Endpoint* Flow_Panel::Find_Endpoint_By_Window(wxWindow* win)
{
  if (win == nullptr)
    return nullptr;
  for (auto* ep : {&m_hik, &m_kuka})
  {
    if (ep->connect_btn == win || ep->disconnect_btn == win)
      return ep;
  }
  return nullptr;
}

void Flow_Panel::On_Connect_Click(wxCommandEvent& event)
{
  auto* ep = Find_Endpoint_By_Window(dynamic_cast<wxWindow*>(event.GetEventObject()));
  if (ep != nullptr)
    Try_Connect(*ep);
}

void Flow_Panel::On_Disconnect_Click(wxCommandEvent& event)
{
  auto* ep = Find_Endpoint_By_Window(dynamic_cast<wxWindow*>(event.GetEventObject()));
  if (ep == nullptr)
    return;

  ep->client->disconnect();
  Update_Status(*ep, false, "Disconnected");
}

void Flow_Panel::Try_Connect(Flow_Endpoint& ep)
{
  if (ep.client->is_connected())
    return;

  wxString ip = Trim_Value(ep.ip_input->GetValue());
  wxString port_str = Trim_Value(ep.port_input->GetValue());

  if (ip.IsEmpty())
  {
    Append_Log("[Error " + ep.name + "]", wxString::FromUTF8(u8"IP 为空"));
    return;
  }

  long port_val = 0;
  if (!port_str.ToLong(&port_val) || port_val <= 0 || port_val > 65535)
  {
    Append_Log("[Error " + ep.name + "]", wxString::FromUTF8(u8"端口非法"));
    return;
  }

  ep.connect_btn->Disable();
  ep.ip_input->Disable();
  ep.port_input->Disable();
  ep.status_label->SetLabel(wxString::FromUTF8(u8"状态：连接中…"));

  // 重连时清空重组缓冲，避免上次的残留片段混入新会话
  {
    std::lock_guard<std::mutex> lock(ep.reassembly_mtx);
    ep.reassembly.clear();
  }

  ep.client->connect(std::string(ip.mb_str(wxConvUTF8)),
                     static_cast<unsigned short>(port_val));
}

void Flow_Panel::Update_Status(Flow_Endpoint& ep, bool connected, const std::string& info)
{
  if (connected)
  {
    ep.connect_btn->Disable();
    ep.disconnect_btn->Enable();
    ep.ip_input->Disable();
    ep.port_input->Disable();
    ep.status_label->SetLabel(wxString::FromUTF8(u8"状态：已连接"));
    Append_Log("[i " + ep.name + "]", wxString::FromUTF8(info));
  }
  else
  {
    ep.connect_btn->Enable();
    ep.disconnect_btn->Disable();
    ep.ip_input->Enable();
    ep.port_input->Enable();
    ep.status_label->SetLabel(wxString::FromUTF8(u8"状态：未连接"));
    Append_Log("[i " + ep.name + "]", wxString::FromUTF8(info));
  }
}

void Flow_Panel::On_Run_Click(wxCommandEvent&)
{
  if (m_step == Flow_Step::Step1_Wait_Recv1 || m_step == Flow_Step::Step3_Wait_Recv2)
    return;

  if (!m_hik.client || !m_hik.client->is_connected())
  {
    Append_Log("[E HIK]", wxString::FromUTF8(u8"HIK 未连接，无法开始流程"));
    Set_Step(Flow_Step::Error);
    return;
  }
  if (!m_kuka.client || !m_kuka.client->is_connected())
  {
    Append_Log("[E KUKA]", wxString::FromUTF8(u8"KUKA 未连接，无法开始流程"));
    Set_Step(Flow_Step::Error);
    return;
  }

  wxString text = Trim_Value(m_input->GetValue());
  if (text.IsEmpty())
  {
    Append_Log("[E HIK]", wxString::FromUTF8(u8"发送内容为空"));
    Set_Step(Flow_Step::Error);
    return;
  }

  Save_Config();

  // ① 向 HIK 发送
  std::string msg(text.mb_str(wxConvUTF8));
  msg += "\r\n";
  m_hik.client->send(msg);
  Append_Log("[HIK >]", text);

  Set_Step(Flow_Step::Step1_Wait_Recv1);
  Start_Wait_Timer();
}

void Flow_Panel::On_Stop_Click(wxCommandEvent&)
{
  Stop_Wait_Timer();
  Append_Log("[i]", wxString::FromUTF8(u8"流程已停止"));
  Set_Step(Flow_Step::Idle);
}

void Flow_Panel::On_Clear_Click(wxCommandEvent&)
{
  m_log->Clear();
}

void Flow_Panel::On_Timeout(wxTimerEvent&)
{
  Stop_Wait_Timer();
  Append_Log("[E]", wxString::FromUTF8(u8"等待超时（60s），流程中止"));
  Set_Step(Flow_Step::Error);
}

void Flow_Panel::On_HIK_Recv_Event(wxThreadEvent& event)
{
  Handle_HIK_Recv(event.GetString());
}

void Flow_Panel::On_KUKA_Recv_Event(wxThreadEvent& event)
{
  Handle_KUKA_Recv(event.GetString());
}

void Flow_Panel::Handle_HIK_Recv(const wxString& msg)
{
  // 始终打印收到的完整回复
  Append_Log("[HIK <]", msg);

  if (m_step == Flow_Step::Step1_Wait_Recv1)
  {
    // ② 把完整 HIK 回复原样转发给 KUKA（KUKA 协议以 ';' 结尾）
    if (!m_kuka.client || !m_kuka.client->is_connected())
    {
      Append_Log("[E KUKA]", wxString::FromUTF8(u8"KUKA 未连接，无法转发"));
      Set_Step(Flow_Step::Error);
      return;
    }

    std::string fwd(msg.mb_str(wxConvUTF8));
    fwd += ";";
    m_kuka.client->send(fwd);
    Append_Log("[KUKA >]", msg);

    Set_Step(Flow_Step::Step3_Wait_Recv2);
    Start_Wait_Timer();
  }
  // 其它状态下收到的 HIK 消息只记录，不影响流程
}

void Flow_Panel::Handle_KUKA_Recv(const wxString& msg)
{
  // 始终打印收到的消息
  Append_Log("[KUKA <]", msg);

  if (m_step == Flow_Step::Step3_Wait_Recv2)
  {
    Stop_Wait_Timer();
    Set_Step(Flow_Step::Done);
    Append_Log("[i]", wxString::FromUTF8(u8"流程完成"));
    Set_Step(Flow_Step::Idle);
  }
}

void Flow_Panel::Feed_HIK_Recv(const std::string& bytes)
{
  // HIK 回复以 ';' 结尾。持续拼接收到的字节，每遇到 ';' 视为一条完整回复，
  // 投递到 UI 线程；剩余尾部留在缓冲等下次到达再续。
  std::vector<std::string> packets;
  {
    std::lock_guard<std::mutex> lock(m_hik.reassembly_mtx);
    m_hik.reassembly += bytes;
    for (;;)
    {
      auto pos = m_hik.reassembly.find(HIK_PACKET_TERM);
      if (pos == std::string::npos)
        break;
      packets.emplace_back(m_hik.reassembly, 0, pos);  // 不含终止符 ';'
      m_hik.reassembly.erase(0, pos + 1);
    }
  }

  for (const auto& p : packets)
  {
    auto* evt = new wxThreadEvent(wxEVT_FLOW_HIK_RECV);
    evt->SetString(wxString::FromUTF8(p));
    wxQueueEvent(this, evt);
  }
}

void Flow_Panel::Feed_KUKA_Recv(const std::string& bytes)
{
  // KUKA 回复原样投递到 UI 线程（不做分包）
  auto* evt = new wxThreadEvent(wxEVT_FLOW_KUKA_RECV);
  evt->SetString(wxString::FromUTF8(bytes));
  wxQueueEvent(this, evt);
}

void Flow_Panel::Set_Step(Flow_Step step)
{
  m_step = step;
  wxString label;
  switch (step)
  {
    case Flow_Step::Idle: label = wxString::FromUTF8(u8"流程状态：空闲"); break;
    case Flow_Step::Step1_Wait_Recv1: label = wxString::FromUTF8(u8"流程状态：等待 HIK 回复…"); break;
    case Flow_Step::Step3_Wait_Recv2: label = wxString::FromUTF8(u8"流程状态：等待 KUKA 回复…"); break;
    case Flow_Step::Done: label = wxString::FromUTF8(u8"流程状态：完成"); break;
    case Flow_Step::Error: label = wxString::FromUTF8(u8"流程状态：出错"); break;
  }
  m_flow_status_label->SetLabel(label);

  const bool running = (step == Flow_Step::Step1_Wait_Recv1 ||
                        step == Flow_Step::Step3_Wait_Recv2);
  m_run_btn->Enable(!running);
  m_stop_btn->Enable(running);
}

void Flow_Panel::Start_Wait_Timer()
{
  if (!m_wait_timer.IsRunning())
    m_wait_timer.Start(WAIT_TIMEOUT_MS, wxTIMER_ONE_SHOT);
}

void Flow_Panel::Stop_Wait_Timer()
{
  if (m_wait_timer.IsRunning())
    m_wait_timer.Stop();
}

void Flow_Panel::Append_Log(const wxString& tag, const wxString& msg)
{
  wxDateTime now = wxDateTime::Now();
  wxString time_str = now.Format("%H:%M:%S");

  wxString body = msg;
  if (body.Length() > MAX_LOG_MSG_LEN)
  {
    body.Truncate(MAX_LOG_MSG_LEN);
    body += wxString::Format(" ...(truncated %zu chars)",
                            msg.Length() - MAX_LOG_MSG_LEN);
  }

  wxString line;
  line.Printf("[%s] %s %s\n", time_str, tag, body);

  // 着色规则：错误红；按 server 分蓝/绿主色，方向(>/<)用深浅区分；Info 橙
  wxColour color = *wxBLACK;
  if (tag.Contains("[E") || tag.Contains("[Error"))
    color = *wxRED;
  else if (tag.Contains("HIK") && tag.Contains(">"))
    color = wxColour(0, 0, 160);      // 深蓝：发给 HIK
  else if (tag.Contains("HIK") && tag.Contains("<"))
    color = *wxBLUE;                  // 蓝：收到 HIK
  else if (tag.Contains("KUKA") && tag.Contains(">"))
    color = wxColour(0, 110, 0);      // 深绿：发给 KUKA
  else if (tag.Contains("KUKA") && tag.Contains("<"))
    color = wxColour(0, 160, 0);      // 绿：收到 KUKA
  else if (tag.Contains("[i"))
    color = wxColour(180, 120, 0);    // 橙：状态/Info

  m_log->SetDefaultStyle(wxTextAttr(color));
  m_log->AppendText(line);
}

wxString Flow_Panel::Load_Config()
{
  wxString last_msg;
  if (wxFileExists(m_config_path))
  {
    wxFileConfig config(wxEmptyString, wxEmptyString, m_config_path, wxEmptyString,
                        wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);
    config.SetPath(FLOW_CONFIG_SECTION);
    config.Read(FLOW_CONFIG_KEY, &last_msg, wxEmptyString);
  }
  return last_msg;
}

void Flow_Panel::Save_Config()
{
  if (!wxFileExists(m_config_path))
  {
    wxFile file(m_config_path, wxFile::write);
    file.Close();
  }
  wxString msg = m_input ? m_input->GetValue() : wxString();
  wxFileConfig config(wxEmptyString, wxEmptyString, m_config_path, wxEmptyString,
                      wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);
  config.SetPath(FLOW_CONFIG_SECTION);
  config.Write(FLOW_CONFIG_KEY, msg);
  config.Flush();
}
