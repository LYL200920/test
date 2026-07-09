#ifndef includeguard_net_panel_h_includeguard
#define includeguard_net_panel_h_includeguard

#include "tcp_client.h"

#include <wx/button.h>
#include <wx/combobox.h>
#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/timer.h>

#include <array>
#include <memory>
#include <vector>

wxDECLARE_EVENT (wxEVT_NET_KUKA_STATUS, wxThreadEvent);
wxDECLARE_EVENT (wxEVT_NET_HIK_STATUS, wxThreadEvent);
wxDECLARE_EVENT (wxEVT_NET_KUKA_RECV, wxThreadEvent);
wxDECLARE_EVENT (wxEVT_NET_HIK_RECV, wxThreadEvent);

class Net_Panel : public wxPanel
{
public:
  explicit Net_Panel (wxWindow* parent, wxWindowID id = wxID_ANY);
  ~Net_Panel ( ) override;

private:
  // 一个连接端点：对应一个独立的 tcp_client（KUKA / HIK）
  struct Net_Endpoint
  {
    wxString name;                       // "KUKA" / "HIK"
    wxString default_ip;
    wxString default_port;
    std::shared_ptr<tcp_client> client;

    wxComboBox* ip_input = nullptr;
    wxComboBox* port_input = nullptr;
    wxButton* connect_btn = nullptr;
    wxButton* disconnect_btn = nullptr;
    wxStaticText* status_label = nullptr;

    wxTextCtrl* send_input = nullptr;
    wxButton* send_btn = nullptr;

    std::vector<wxString> ip_history;
    std::vector<wxString> port_history;
  };

  void Build_Ui ( );

  void On_Connect_Click (wxCommandEvent& event);
  void On_Disconnect_Click (wxCommandEvent& event);
  void On_Send_Click (wxCommandEvent& event);
  void On_Send_Key (wxKeyEvent& event);
  void On_KUKA_Status_Event (wxThreadEvent& event);
  void On_HIK_Status_Event (wxThreadEvent& event);
  void On_KUKA_Recv_Event (wxThreadEvent& event);
  void On_HIK_Recv_Event (wxThreadEvent& event);
  void On_Clear_Click (wxCommandEvent& event);

  // 根据 UI 事件源定位是哪个端点
  Net_Endpoint* Find_Endpoint_By_Window (wxWindow* win);

  void Update_Status (Net_Endpoint& ep, bool connected, const std::string& info);
  void Append_Log (const wxString& source, const wxString& prefix, const wxString& msg);

  void Try_Connect (Net_Endpoint& ep);
  void Load_Net_Config ( );
  void Save_Net_Config (const Net_Endpoint& ep);
  void Refresh_Net_Inputs (Net_Endpoint& ep, const wxString& ip, const wxString& port);

private:
  std::array<Net_Endpoint, 2> m_endpoints;  // [0]=KUKA, [1]=HIK

  wxTextCtrl* m_log = nullptr;
  wxButton* m_clear_btn = nullptr;

  wxString m_config_path;

  wxDECLARE_EVENT_TABLE ( );
};

#endif
