#ifndef includeguard_flow_panel_h_includeguard
#define includeguard_flow_panel_h_includeguard

#include "tcp_client.h"

#include <wx/panel.h>
#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/timer.h>

#include <memory>
#include <mutex>
#include <string>

class wxButton;
class wxComboBox;
class wxStaticText;

// 流程编排面板：
//   ① 向 HIK(Server1)发送用户编辑内容
//   ② 收到 HIK 回复后，原样转发给 KUKA(Server2)
//   ③ 收到 KUKA 回复后，显示出来
//
// 本面板独立持有两条 TCP 连接，与 Net_Panel(TCP 工具栏)完全隔离、互不影响。
// recv 回调收到的字节流原样投递到 UI 线程（与 Net_Panel 行为一致），
// 不做 \r\n 分包：HIK 回复以 ';' 结尾、不含 \r\n，分包会把它滞留在缓冲里。
class Flow_Panel : public wxPanel
{
public:
  explicit Flow_Panel (wxWindow* parent, wxWindowID id = wxID_ANY);
  ~Flow_Panel ( ) override;

private:
  // 流程编排中的两条连接端点
  struct Flow_Endpoint
  {
    wxString name;                       // "HIK" / "KUKA"
    wxString default_ip;
    wxString default_port;
    std::shared_ptr<tcp_client> client;

    wxComboBox* ip_input = nullptr;
    wxComboBox* port_input = nullptr;
    wxButton* connect_btn = nullptr;
    wxButton* disconnect_btn = nullptr;
    wxStaticText* status_label = nullptr;

    // HIK 回复重组缓冲：async_read_some 是字节流、一条回复可能分多次到达，
    // HIK 回复以 ';' 结尾，按 ';' 重组为完整消息后再转发，保证 KUKA 收到完整内容。
    std::string reassembly;
    std::mutex reassembly_mtx;
  };

  enum class Flow_Step
  {
    Idle,
    Step1_Wait_Recv1,   // 已发给 HIK，等 HIK 回复
    Step3_Wait_Recv2,   // 已转发给 KUKA，等 KUKA 回复
    Done,
    Error
  };

  void Build_Ui ( );

  // 连接 / 发送按钮（按事件来源定位端点）
  void On_Connect_Click (wxCommandEvent& event);
  void On_Disconnect_Click (wxCommandEvent& event);
  Flow_Endpoint* Find_Endpoint_By_Window (wxWindow* win);

  void On_Run_Click (wxCommandEvent& event);
  void On_Stop_Click (wxCommandEvent& event);
  void On_Clear_Click (wxCommandEvent& event);
  void On_Timeout (wxTimerEvent& event);

  // recv 事件（UI 线程）：把一条完整消息喂给状态机
  void On_HIK_Recv_Event (wxThreadEvent& event);
  void On_KUKA_Recv_Event (wxThreadEvent& event);
  void Handle_HIK_Recv (const wxString& msg);   // 已重组的完整 HIK 回复
  void Handle_KUKA_Recv (const wxString& msg);

  // 给 client 的 recv 回调（IO 线程）：把收到的字节流原样投递一个事件到 UI 线程
  void Feed_HIK_Recv (const std::string& bytes);
  void Feed_KUKA_Recv (const std::string& bytes);

  void Try_Connect (Flow_Endpoint& ep);
  void Update_Status (Flow_Endpoint& ep, bool connected, const std::string& info);

  void Set_Step (Flow_Step step);
  void Start_Wait_Timer ( );
  void Stop_Wait_Timer ( );
  void Append_Log (const wxString& tag, const wxString& msg);

  wxString Load_Config ( );
  void Save_Config ( );

private:
  Flow_Endpoint m_hik;    // Server1
  Flow_Endpoint m_kuka;   // Server2

  wxTextCtrl* m_input = nullptr;       // 用户编辑的初始消息
  wxButton* m_run_btn = nullptr;
  wxButton* m_stop_btn = nullptr;
  wxStaticText* m_flow_status_label = nullptr;
  wxTextCtrl* m_log = nullptr;
  wxButton* m_clear_btn = nullptr;

  wxTimer m_wait_timer;
  Flow_Step m_step = Flow_Step::Idle;

  wxString m_config_path;

  wxDECLARE_EVENT_TABLE ( );
};

#endif
