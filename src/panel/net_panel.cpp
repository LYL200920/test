#include "net_panel.h"

#include <wx/datetime.h>
#include <wx/file.h>
#include <wx/fileconf.h>
#include <wx/filename.h>
#include <wx/sizer.h>
#include <wx/stdpaths.h>
#include <wx/tokenzr.h>

#include <algorithm>

namespace
{
constexpr size_t MAX_HISTORY_COUNT = 10;
constexpr size_t MAX_LOG_MSG_LEN = 2000;  // 单条日志最大字符数

const wxString KUKA_NAME = "KUKA";
const wxString HIK_NAME = "HIK";

const wxString HIK_DEFAULT_IP = "192.168.1.45";
const wxString HIK_DEFAULT_PORT = "7930";
const wxString KUKA_DEFAULT_IP = "192.168.1.25";
const wxString KUKA_DEFAULT_PORT = "54600";

wxString Trim_Value (wxString value)
{
  value.Trim (true);
  value.Trim (false);
  return value;
}

std::vector<wxString> Split_History (const wxString& text)
{
  std::vector<wxString> values;
  wxStringTokenizer tokenizer (text, ";");
  while (tokenizer.HasMoreTokens ( ))
  {
    wxString value = Trim_Value (tokenizer.GetNextToken ( ));
    if (!value.IsEmpty ( ) &&
        std::find (values.begin ( ), values.end ( ), value) == values.end ( ))
    {
      values.push_back (value);
    }
  }
  return values;
}

wxString Join_History (const std::vector<wxString>& values)
{
  wxString text;
  for (size_t i = 0; i < values.size ( ); ++i)
  {
    if (i > 0)
      text += ";";
    text += values[i];
  }
  return text;
}

void Add_History_Value (std::vector<wxString>& values, const wxString& value)
{
  wxString trimmed = Trim_Value (value);
  if (trimmed.IsEmpty ( ))
    return;

  values.erase (std::remove (values.begin ( ), values.end ( ), trimmed), values.end ( ));
  values.insert (values.begin ( ), trimmed);
  if (values.size ( ) > MAX_HISTORY_COUNT)
    values.resize (MAX_HISTORY_COUNT);
}

void Fill_Combo_Box (wxComboBox* combo, const std::vector<wxString>& values, const wxString& current)
{
  if (combo == nullptr)
    return;

  combo->Clear ( );
  for (const auto& value : values)
    combo->Append (value);
  combo->SetValue (current);
}
}

// ---- 自定义事件定义：每个端点独立的事件类型，天然区分来源 ----
wxDEFINE_EVENT(wxEVT_NET_KUKA_STATUS, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_NET_HIK_STATUS, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_NET_KUKA_RECV, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_NET_HIK_RECV, wxThreadEvent);

enum
{
  ID_CONNECT_BTN = 1001,
  ID_DISCONNECT_BTN,
  ID_SEND_BTN,
  ID_CLEAR_BTN,
};

wxBEGIN_EVENT_TABLE(Net_Panel, wxPanel)
    EVT_BUTTON(ID_CONNECT_BTN, Net_Panel::On_Connect_Click)
        EVT_BUTTON(ID_DISCONNECT_BTN, Net_Panel::On_Disconnect_Click)
            EVT_BUTTON(ID_SEND_BTN, Net_Panel::On_Send_Click)
                EVT_BUTTON(ID_CLEAR_BTN, Net_Panel::On_Clear_Click)
                    wxEND_EVENT_TABLE()

                        Net_Panel::Net_Panel(wxWindow *parent, wxWindowID id)
    : wxPanel(parent, id)
{
  // 端点角色初始化：第一个 HIK，第二个 KUKA
  m_endpoints[0].name = HIK_NAME;
  m_endpoints[0].default_ip = HIK_DEFAULT_IP;
  m_endpoints[0].default_port = HIK_DEFAULT_PORT;
  m_endpoints[1].name = KUKA_NAME;
  m_endpoints[1].default_ip = KUKA_DEFAULT_IP;
  m_endpoints[1].default_port = KUKA_DEFAULT_PORT;

  Build_Ui();

  // 用 Bind() 绑定各端点的事件类型
  Bind(wxEVT_NET_KUKA_STATUS, &Net_Panel::On_KUKA_Status_Event, this);
  Bind(wxEVT_NET_HIK_STATUS, &Net_Panel::On_HIK_Status_Event, this);
  Bind(wxEVT_NET_KUKA_RECV, &Net_Panel::On_KUKA_Recv_Event, this);
  Bind(wxEVT_NET_HIK_RECV, &Net_Panel::On_HIK_Recv_Event, this);

  // HIK client（第一个端点）
  {
    auto& ep = m_endpoints[0];
    ep.client = std::make_shared<tcp_client>();
    ep.client->set_status_callback([this](bool connected, const std::string &info)
                                   {
      auto* evt = new wxThreadEvent(wxEVT_NET_HIK_STATUS);
      evt->SetInt(connected ? 1 : 0);
      evt->SetString(wxString::FromUTF8(info));
      wxQueueEvent(this, evt); });
    ep.client->set_recv_callback([this](const std::string &msg)
                                 {
      auto* evt = new wxThreadEvent(wxEVT_NET_HIK_RECV);
      evt->SetString(wxString::FromUTF8(msg));
      wxQueueEvent(this, evt); });
  }

  // KUKA client（第二个端点）
  {
    auto& ep = m_endpoints[1];
    ep.client = std::make_shared<tcp_client>();
    ep.client->set_status_callback([this](bool connected, const std::string &info)
                                   {
      auto* evt = new wxThreadEvent(wxEVT_NET_KUKA_STATUS);
      evt->SetInt(connected ? 1 : 0);
      evt->SetString(wxString::FromUTF8(info));
      wxQueueEvent(this, evt); });
    ep.client->set_recv_callback([this](const std::string &msg)
                                 {
      auto* evt = new wxThreadEvent(wxEVT_NET_KUKA_RECV);
      evt->SetString(wxString::FromUTF8(msg));
      wxQueueEvent(this, evt); });
  }
}

Net_Panel::~Net_Panel()
{
  for (auto& ep : m_endpoints)
  {
    if (ep.client)
    {
      ep.client->set_status_callback(nullptr);
      ep.client->set_recv_callback(nullptr);
      ep.client->disconnect();
      ep.client.reset();
    }
  }

  DeletePendingEvents();
  Unbind(wxEVT_NET_KUKA_STATUS, &Net_Panel::On_KUKA_Status_Event, this);
  Unbind(wxEVT_NET_HIK_STATUS, &Net_Panel::On_HIK_Status_Event, this);
  Unbind(wxEVT_NET_KUKA_RECV, &Net_Panel::On_KUKA_Recv_Event, this);
  Unbind(wxEVT_NET_HIK_RECV, &Net_Panel::On_HIK_Recv_Event, this);
}

void Net_Panel::Build_Ui()
{
  Load_Net_Config();

  auto *sizer = new wxBoxSizer(wxVERTICAL);

  for (auto& ep : m_endpoints)
  {
    // ---- 端点标题 ----
    auto *title = new wxStaticText(this, wxID_ANY, ep.name + " Server");
    wxFont title_font = title->GetFont();
    title_font.MakeBold();
    title_font.SetPointSize(title_font.GetPointSize() + 1);
    title->SetFont(title_font);

    auto *ip_label = new wxStaticText(this, wxID_ANY, "Server IP:");
    ep.ip_input = new wxComboBox(this, wxID_ANY, wxEmptyString,
                                 wxDefaultPosition, wxDefaultSize, 0, nullptr, wxCB_DROPDOWN);
    ep.ip_input->SetMinSize(wxSize(180, -1));

    auto *port_label = new wxStaticText(this, wxID_ANY, "Port:");
    ep.port_input = new wxComboBox(this, wxID_ANY, wxEmptyString,
                                   wxDefaultPosition, wxDefaultSize, 0, nullptr, wxCB_DROPDOWN);
    ep.port_input->SetMinSize(wxSize(100, -1));

    Refresh_Net_Inputs(ep,
                       ep.ip_history.empty ( ) ? ep.default_ip : ep.ip_history.front ( ),
                       ep.port_history.empty ( ) ? ep.default_port : ep.port_history.front ( ));

    ep.connect_btn = new wxButton(this, ID_CONNECT_BTN, "Connect");
    ep.disconnect_btn = new wxButton(this, ID_DISCONNECT_BTN, "Disconnect");
    ep.disconnect_btn->Disable();

    ep.status_label = new wxStaticText(this, wxID_ANY, "Status: Not connected");

    auto *endpoint_sizer = new wxFlexGridSizer(2, 2, 6, 8);
    endpoint_sizer->Add(ip_label, 0, wxALIGN_CENTER_VERTICAL);
    endpoint_sizer->Add(ep.ip_input, 1, wxEXPAND);
    endpoint_sizer->Add(port_label, 0, wxALIGN_CENTER_VERTICAL);
    endpoint_sizer->Add(ep.port_input, 1, wxEXPAND);
    endpoint_sizer->AddGrowableCol(1, 1);

    auto *connection_button_sizer = new wxBoxSizer(wxHORIZONTAL);
    connection_button_sizer->Add(ep.connect_btn, 1, wxEXPAND | wxRIGHT, 4);
    connection_button_sizer->Add(ep.disconnect_btn, 1, wxEXPAND);

    // ---- 发送区 ----
    ep.send_input = new wxTextCtrl(this, wxID_ANY, wxEmptyString,
                                   wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    ep.send_input->Bind(wxEVT_KEY_DOWN, &Net_Panel::On_Send_Key, this);

    ep.send_btn = new wxButton(this, ID_SEND_BTN, "Send");
    ep.send_btn->Disable();

    auto *send_sizer = new wxBoxSizer(wxHORIZONTAL);
    send_sizer->Add(ep.send_input, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    send_sizer->Add(ep.send_btn, 0, wxALIGN_CENTER_VERTICAL);

    sizer->Add(title, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
    sizer->Add(endpoint_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
    sizer->Add(connection_button_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT, 4);
    sizer->Add(ep.status_label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 4);
    sizer->Add(send_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
  }

  // ---- 共享日志区 ----
  auto *log_title = new wxStaticText(this, wxID_ANY, "Log");
  wxFont log_font = log_title->GetFont();
  log_font.MakeBold();
  log_title->SetFont(log_font);

  m_log = new wxTextCtrl(this, wxID_ANY, wxEmptyString,
                         wxDefaultPosition, wxDefaultSize,
                         wxTE_MULTILINE | wxTE_READONLY | wxTE_RICH);
  // 等宽字体，便于对齐日志列
  wxFont mono_font;
  mono_font.SetFamily(wxFONTFAMILY_TELETYPE);
  m_log->SetFont(mono_font);

  m_clear_btn = new wxButton(this, ID_CLEAR_BTN, "Clear Log");

  auto *log_sizer = new wxBoxSizer(wxHORIZONTAL);
  log_sizer->AddStretchSpacer(1);
  log_sizer->Add(m_clear_btn, 0);

  sizer->Add(log_title, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->Add(m_log, 1, wxEXPAND | wxLEFT | wxRIGHT, 6);
  sizer->Add(log_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  SetSizer(sizer);
}

Net_Panel::Net_Endpoint* Net_Panel::Find_Endpoint_By_Window(wxWindow* win)
{
  if (win == nullptr)
    return nullptr;
  for (auto& ep : m_endpoints)
  {
    if (ep.connect_btn == win || ep.disconnect_btn == win ||
        ep.send_btn == win || ep.send_input == win ||
        ep.ip_input == win || ep.port_input == win)
    {
      return &ep;
    }
  }
  return nullptr;
}

void Net_Panel::On_Connect_Click(wxCommandEvent& event)
{
  auto* ep = Find_Endpoint_By_Window(dynamic_cast<wxWindow*>(event.GetEventObject()));
  if (ep != nullptr)
    Try_Connect(*ep);
}

void Net_Panel::Try_Connect(Net_Endpoint& ep)
{
  if (ep.client->is_connected())
    return;

  wxString ip = Trim_Value(ep.ip_input->GetValue());
  wxString port_str = Trim_Value(ep.port_input->GetValue());

  if (ip.IsEmpty())
  {
    Append_Log(ep.name, "[Error]", "IP address is empty");
    return;
  }

  long port_val = 0;
  if (!port_str.ToLong(&port_val) || port_val <= 0 || port_val > 65535)
  {
    Append_Log(ep.name, "[Error]", "Invalid port number");
    return;
  }

  Save_Net_Config(ep);
  Refresh_Net_Inputs(ep, ip, port_str);

  ep.connect_btn->Disable();
  ep.ip_input->Disable();
  ep.port_input->Disable();
  ep.status_label->SetLabel("Status: Connecting...");

  ep.client->connect(std::string(ip.mb_str(wxConvUTF8)),
                     static_cast<unsigned short>(port_val));
}

void Net_Panel::On_Disconnect_Click(wxCommandEvent& event)
{
  auto* ep = Find_Endpoint_By_Window(dynamic_cast<wxWindow*>(event.GetEventObject()));
  if (ep == nullptr)
    return;

  ep->client->disconnect();

  ep->connect_btn->Enable();
  ep->disconnect_btn->Disable();
  ep->send_btn->Disable();
  ep->ip_input->Enable();
  ep->port_input->Enable();
  ep->status_label->SetLabel("Status: Disconnected");
}

void Net_Panel::On_Send_Click(wxCommandEvent& event)
{
  auto* ep = Find_Endpoint_By_Window(dynamic_cast<wxWindow*>(event.GetEventObject()));
  if (ep == nullptr)
    return;

  if (!ep->client->is_connected())
    return;

  wxString text = ep->send_input->GetValue();
  if (text.IsEmpty())
    return;

  std::string msg(text.mb_str(wxConvUTF8));
  msg += "\r\n";

  ep->client->send(msg);
  Append_Log(ep->name, "[Send]", text);

  // 保留输入框内容，方便重复发送，光标置末尾
  ep->send_input->SetInsertionPointEnd();
}

void Net_Panel::On_Send_Key(wxKeyEvent& event)
{
  if (event.GetKeyCode() == WXK_RETURN && !event.ShiftDown())
  {
    auto* ep = Find_Endpoint_By_Window(dynamic_cast<wxWindow*>(event.GetEventObject()));
    if (ep != nullptr && ep->client->is_connected())
    {
      wxCommandEvent dummy;
      dummy.SetEventObject(ep->send_btn);
      On_Send_Click(dummy);
    }
  }
  else
  {
    event.Skip();
  }
}

void Net_Panel::On_HIK_Status_Event(wxThreadEvent& event)
{
  Update_Status(m_endpoints[0], event.GetInt() != 0, std::string(event.GetString().mb_str(wxConvUTF8)));
}

void Net_Panel::On_KUKA_Status_Event(wxThreadEvent& event)
{
  Update_Status(m_endpoints[1], event.GetInt() != 0, std::string(event.GetString().mb_str(wxConvUTF8)));
}

void Net_Panel::On_HIK_Recv_Event(wxThreadEvent& event)
{
  Append_Log(HIK_NAME, "[Recv]", event.GetString());
}

void Net_Panel::On_KUKA_Recv_Event(wxThreadEvent& event)
{
  Append_Log(KUKA_NAME, "[Recv]", event.GetString());
}

void Net_Panel::On_Clear_Click(wxCommandEvent&)
{
  m_log->Clear();
}

void Net_Panel::Update_Status(Net_Endpoint& ep, bool connected, const std::string& info)
{
  if (connected)
  {
    ep.connect_btn->Disable();
    ep.disconnect_btn->Enable();
    ep.send_btn->Enable();
    ep.ip_input->Disable();
    ep.port_input->Disable();
    ep.status_label->SetLabel("Status: Connected");
    Append_Log(ep.name, "[Info]", wxString::FromUTF8(info));
  }
  else
  {
    ep.connect_btn->Enable();
    ep.disconnect_btn->Disable();
    ep.send_btn->Disable();
    ep.ip_input->Enable();
    ep.port_input->Enable();
    ep.status_label->SetLabel("Status: Disconnected");
    Append_Log(ep.name, "[Info]", wxString::FromUTF8(info));
  }
}

void Net_Panel::Append_Log(const wxString& source, const wxString& prefix, const wxString& msg)
{
  wxDateTime now = wxDateTime::Now();
  wxString time_str = now.Format("%H:%M:%S");

  // 超长消息截断
  wxString body = msg;
  if (body.Length() > MAX_LOG_MSG_LEN)
  {
    body.Truncate(MAX_LOG_MSG_LEN);
    body += wxString::Format(" ...(truncated %zu chars)",
                            msg.Length() - MAX_LOG_MSG_LEN);
  }

  wxString line;
  line.Printf("[%s] [%-4s] %s %s\n", time_str, source, prefix, body);

  // 按来源 + 方向上色，一眼区分
  wxColour color = *wxBLACK;
  if (prefix == "[Send]")
    color = (source == KUKA_NAME) ? wxColour(30, 30, 30) : wxColour(120, 120, 120);
  else if (prefix == "[Recv]")
    color = (source == KUKA_NAME) ? *wxBLUE : wxColour(0, 140, 0);
  else if (prefix == "[Error]")
    color = *wxRED;
  else if (prefix == "[Info]")
    color = wxColour(180, 120, 0);

  m_log->SetDefaultStyle(wxTextAttr(color));
  m_log->AppendText(line);
}

void Net_Panel::Load_Net_Config()
{
  wxString exe_path = wxStandardPaths::Get().GetExecutablePath();
  m_config_path = wxFileName(exe_path).GetPath() + "/vtk_net_config.ini";

  wxFileConfig config(wxEmptyString, wxEmptyString, m_config_path, wxEmptyString,
                      wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

  for (auto& ep : m_endpoints)
  {
    wxString key_lower = ep.name.Lower();

    wxString last_ip = ep.default_ip;
    wxString last_port = ep.default_port;
    wxString ip_history_text = ep.default_ip;
    wxString port_history_text = ep.default_port;

    config.SetPath("/Net");
    config.Read(key_lower + "_last_ip", &last_ip, ep.default_ip);
    config.Read(key_lower + "_last_port", &last_port, ep.default_port);
    config.Read(key_lower + "_ip_history", &ip_history_text, ep.default_ip);
    config.Read(key_lower + "_port_history", &port_history_text, ep.default_port);

    ep.ip_history = Split_History(ip_history_text);
    ep.port_history = Split_History(port_history_text);
    Add_History_Value(ep.ip_history, last_ip);
    Add_History_Value(ep.port_history, last_port);
  }
}

void Net_Panel::Save_Net_Config(const Net_Endpoint& ep)
{
  if (m_config_path.IsEmpty())
  {
    wxString exe_path = wxStandardPaths::Get().GetExecutablePath();
    m_config_path = wxFileName(exe_path).GetPath() + "/vtk_net_config.ini";
  }

  if (!wxFileExists(m_config_path))
  {
    wxFile file(m_config_path, wxFile::write);
    file.Close();
  }

  // ep 是 const 引用，通过指针回写历史
  std::vector<wxString>& ip_hist = const_cast<Net_Endpoint&>(ep).ip_history;
  std::vector<wxString>& port_hist = const_cast<Net_Endpoint&>(ep).port_history;
  wxString ip = Trim_Value(ep.ip_input->GetValue());
  wxString port = Trim_Value(ep.port_input->GetValue());
  Add_History_Value(ip_hist, ip);
  Add_History_Value(port_hist, port);

  wxFileConfig config(wxEmptyString, wxEmptyString, m_config_path, wxEmptyString,
                      wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);
  wxString key_lower = ep.name.Lower();
  config.SetPath("/Net");
  config.Write(key_lower + "_last_ip", ip);
  config.Write(key_lower + "_last_port", port);
  config.Write(key_lower + "_ip_history", Join_History(ip_hist));
  config.Write(key_lower + "_port_history", Join_History(port_hist));
  config.Flush();
}

void Net_Panel::Refresh_Net_Inputs(Net_Endpoint& ep, const wxString& ip, const wxString& port)
{
  Fill_Combo_Box(ep.ip_input, ep.ip_history, ip);
  Fill_Combo_Box(ep.port_input, ep.port_history, port);
}
