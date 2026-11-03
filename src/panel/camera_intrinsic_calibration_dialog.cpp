#include "camera_intrinsic_calibration_dialog.h"
#include "camera_intrinsic_profile_dialog.h"

#include "app_paths.h"
#include "camera_2d_service.h"
#include "camera_intrinsics_repository.h"
#include "intrinsic_calibration_dataset.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/dirdlg.h>
#include <wx/dcmemory.h>
#include <wx/filedlg.h>
#include <wx/gauge.h>
#include <wx/image.h>
#include <wx/listctrl.h>
#include <wx/msgdlg.h>
#include <wx/simplebook.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbmp.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/textdlg.h>
#include <wx/datetime.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

namespace
{

wxDEFINE_EVENT(wxEVT_INTRINSIC_CALIBRATION_WORKER, wxThreadEvent);

std::filesystem::path Make_Session_Image_Directory()
{
  const auto now = std::chrono::system_clock::to_time_t(
    std::chrono::system_clock::now());
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &now);
#else
  localtime_r(&now, &local_time);
#endif
  std::ostringstream name;
  name << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return application::Get_App_Paths().run_image_root /
         "CameraCalibration" / name.str() / "images";
}

wxString From_Utf8(const std::string &text)
{
  return wxString::FromUTF8(text.c_str());
}

} // namespace

Camera_Intrinsic_Calibration_Dialog::
Camera_Intrinsic_Calibration_Dialog(
  wxWindow *parent,
  Camera_2D_Service &camera_service,
  Camera_Calibration_Progress_Host *progress_host)
  : wxDialog(
      parent,
      wxID_ANY,
      wxString::FromUTF8(u8"2D相机内参标定向导"),
      wxDefaultPosition,
      wxSize(1080, 760),
      wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
    camera_service_(camera_service),
    progress_host_(progress_host),
    session_image_directory_(Make_Session_Image_Directory())
{
  Build_Ui();
  if (progress_path_text_)
  {
    progress_path_text_->SetValue(
      (application::Get_App_Paths().progress_root /
       "progress_camera_calib_board.xml").wstring());
  }
  Bind(
    wxEVT_INTRINSIC_CALIBRATION_WORKER,
    &Camera_Intrinsic_Calibration_Dialog::On_Worker_Complete,
    this);
  Refresh_Preparation();
  Inspect_Progress(false);
  Refresh_Navigation();
  CentreOnParent();
}

Camera_Intrinsic_Calibration_Dialog::
~Camera_Intrinsic_Calibration_Dialog()
{
  calculation_timer_.Stop();
  closing_.store(true);
  if (worker_.joinable())
  {
    worker_.join();
  }
  if (progress_host_)
  {
    progress_host_->Detach_Calibration_Progress_Observer(this);
  }
}

void Camera_Intrinsic_Calibration_Dialog::Build_Ui()
{
  book_ = new wxSimplebook(this, wxID_ANY);
  book_->AddPage(Build_Preparation_Page(), wxEmptyString, true);
  book_->AddPage(Build_Board_Page(), wxEmptyString, false);
  book_->AddPage(Build_Capture_Page(), wxEmptyString, false);
  book_->AddPage(Build_Review_Page(), wxEmptyString, false);
  book_->AddPage(Build_Result_Page(), wxEmptyString, false);

  back_button_ = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"上一步"));
  next_button_ = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"下一步"));
  cancel_button_ = new wxButton(
    this, wxID_CANCEL, wxString::FromUTF8(u8"取消"));
  auto *buttons = new wxBoxSizer(wxHORIZONTAL);
  buttons->AddStretchSpacer(1);
  buttons->Add(back_button_, 0, wxRIGHT, 8);
  buttons->Add(next_button_, 0, wxRIGHT, 8);
  buttons->Add(cancel_button_, 0);

  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(book_, 1, wxEXPAND | wxALL, 12);
  sizer->Add(buttons, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 12);
  SetSizer(sizer);

  back_button_->Bind(
    wxEVT_BUTTON, &Camera_Intrinsic_Calibration_Dialog::On_Back, this);
  next_button_->Bind(
    wxEVT_BUTTON, &Camera_Intrinsic_Calibration_Dialog::On_Next, this);
  cancel_button_->Bind(
    wxEVT_BUTTON, &Camera_Intrinsic_Calibration_Dialog::On_Cancel, this);
  Bind(
    wxEVT_CLOSE_WINDOW,
    &Camera_Intrinsic_Calibration_Dialog::On_Close,
    this);
}

wxWindow *Camera_Intrinsic_Calibration_Dialog::Build_Preparation_Page()
{
  auto *page = new wxPanel(book_, wxID_ANY);
  auto *title = new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"步骤 1/5：准备相机"));
  auto font = title->GetFont();
  font.SetPointSize(font.GetPointSize() + 3);
  font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(font);
  camera_status_ = new wxStaticText(page, wxID_ANY, wxEmptyString);
  camera_status_->Wrap(900);
  lens_choice_ = new wxChoice(page, wxID_ANY);
  lens_choice_->Append(wxString::FromUTF8(u8"普通透视镜头（针孔模型）"));
  lens_choice_->Append(wxString::FromUTF8(u8"远心镜头（固定平面位移映射）"));
  lens_choice_->Append(wxString::FromUTF8(u8"不确定"));
  lens_choice_->SetSelection(0);
  auto *notice = new wxStaticText(
    page,
    wxID_ANY,
    wxString::FromUTF8(
      u8"标定期间必须保持分辨率、ROI、焦距和光圈不变。相机未连接时仍可在后续步骤导入已有图片。\n"
      u8"普通镜头使用针孔 Brown-5；远心模式只计算固定平面映射，不估计焦距或非线性畸变。\n"
      u8"远心采图：固定标定板、工作高度及相对姿态，仅沿平面平移，覆盖视野各区域；不要混入倾斜图。"));
  notice->Wrap(900);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxBOTTOM, 24);
  sizer->Add(camera_status_, 0, wxEXPAND | wxBOTTOM, 20);
  sizer->Add(
    new wxStaticText(page, wxID_ANY, wxString::FromUTF8(u8"镜头类型")),
    0, wxBOTTOM, 6);
  sizer->Add(lens_choice_, 0, wxEXPAND | wxBOTTOM, 20);
  sizer->Add(notice, 0, wxEXPAND);
  auto *profiles = new wxButton(page, wxID_ANY,
    wxString::FromUTF8(u8"所选模型版本管理（针孔支持去畸变对照）..."));
  profiles->Bind(wxEVT_BUTTON, [this](wxCommandEvent &)
  {
    Camera_Intrinsic_Profile_Dialog dialog(this, camera_service_, nullptr, nullptr,
      lens_choice_->GetSelection() == 1);
    dialog.ShowModal();
  });
  sizer->Add(profiles, 0, wxTOP, 16);
  translation_button_ = new wxButton(page, wxID_ANY, wxString::FromUTF8(u8"建立远心平移采集会话（实际状态审计）..."));
  translation_button_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { Prepare_Translation_Session(); });
  sizer->Add(translation_button_, 0, wxTOP, 10);
  page->SetSizer(sizer);
  lens_choice_->Bind(wxEVT_CHOICE, [this](wxCommandEvent &)
  {
    Refresh_Navigation();
  });
  return page;
}

wxWindow *Camera_Intrinsic_Calibration_Dialog::Build_Board_Page()
{
  auto *page = new wxPanel(book_, wxID_ANY);
  auto *title = new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"步骤 2/5：配置标定板"));
  auto font = title->GetFont();
  font.SetPointSize(font.GetPointSize() + 3);
  font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(font);
  auto *grid = new wxFlexGridSizer(2, 12, 12);
  grid->AddGrowableCol(1, 1);
  grid->Add(new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"内角点列数")),
    0, wxALIGN_CENTER_VERTICAL);
  columns_spin_ = new wxSpinCtrl(
    page, wxID_ANY, "17", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 2, 200, 17);
  grid->Add(columns_spin_, 1, wxEXPAND);
  grid->Add(new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"内角点行数")),
    0, wxALIGN_CENTER_VERTICAL);
  rows_spin_ = new wxSpinCtrl(
    page, wxID_ANY, "19", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 2, 200, 19);
  grid->Add(rows_spin_, 1, wxEXPAND);
  grid->Add(new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"格点间距 (mm)")),
    0, wxALIGN_CENTER_VERTICAL);
  square_size_spin_ = new wxSpinCtrlDouble(
    page, wxID_ANY, "0.5", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 0.001, 1000.0, 0.5, 0.1);
  square_size_spin_->SetDigits(3);
  grid->Add(square_size_spin_, 1, wxEXPAND);
  auto *notice = new wxStaticText(
    page,
    wxID_ANY,
    wxString::FromUTF8(
      u8"这里填写的是内角点数量，不是黑白方格数量。当前 CC10-17X19-0.5 标定板已由实图确认使用 17 × 19。\n"
      u8"格点间距只影响棋盘位姿的毫米尺度，不影响像素焦距和畸变系数。"));
  notice->Wrap(900);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxBOTTOM, 24);
  sizer->Add(grid, 0, wxEXPAND | wxBOTTOM, 20);
  sizer->Add(notice, 0, wxEXPAND);
  page->SetSizer(sizer);
  return page;
}

wxWindow *Camera_Intrinsic_Calibration_Dialog::Build_Capture_Page()
{
  auto *page = new wxPanel(book_, wxID_ANY);
  auto *title = new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"步骤 3/5：采集标定图像"));
  auto font = title->GetFont();
  font.SetPointSize(font.GetPointSize() + 3);
  font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(font);

  auto *progress_box = new wxStaticBoxSizer(
    wxVERTICAL,
    page,
    wxString::FromUTF8(u8"Progress 自动运动取图"));
  progress_path_text_ = new wxTextCtrl(page, wxID_ANY);
  progress_browse_button_ = new wxButton(
    page, wxID_ANY, wxString::FromUTF8(u8"浏览…"));
  progress_inspect_button_ = new wxButton(
    page, wxID_ANY, wxString::FromUTF8(u8"检查"));
  auto *path_row = new wxBoxSizer(wxHORIZONTAL);
  path_row->Add(progress_path_text_, 1, wxEXPAND | wxRIGHT, 6);
  path_row->Add(progress_browse_button_, 0, wxRIGHT, 6);
  path_row->Add(progress_inspect_button_, 0);

  progress_speed_spin_ = new wxSpinCtrl(
    page, wxID_ANY, "20", wxDefaultPosition, wxSize(70, -1),
    wxSP_ARROW_KEYS, 1, 100, 20);
  progress_settle_spin_ = new wxSpinCtrl(
    page, wxID_ANY, "500", wxDefaultPosition, wxSize(85, -1),
    wxSP_ARROW_KEYS, 0, 5000, 500);
  progress_start_button_ = new wxButton(
    page, wxID_ANY, wxString::FromUTF8(u8"开始自动取图"));
  progress_stop_button_ = new wxButton(
    page, wxID_ANY, wxString::FromUTF8(u8"停止运动"));
  auto *run_row = new wxBoxSizer(wxHORIZONTAL);
  run_row->Add(new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"LIN速度(mm/s)")),
    0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
  run_row->Add(progress_speed_spin_, 0, wxRIGHT, 12);
  run_row->Add(new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"到位稳定(ms)")),
    0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
  run_row->Add(progress_settle_spin_, 0, wxRIGHT, 12);
  run_row->Add(progress_start_button_, 0, wxRIGHT, 6);
  run_row->Add(progress_stop_button_, 0);
  progress_summary_text_ = new wxStaticText(page, wxID_ANY, wxEmptyString);
  progress_status_text_ = new wxStaticText(page, wxID_ANY, wxEmptyString);
  progress_gauge_ = new wxGauge(page, wxID_ANY, 1);
  progress_box->Add(path_row, 0, wxEXPAND | wxALL, 6);
  progress_box->Add(progress_summary_text_, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);
  progress_box->Add(run_row, 0, wxEXPAND | wxALL, 6);
  progress_box->Add(progress_gauge_, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);
  progress_box->Add(
    progress_status_text_, 0, wxEXPAND | wxALL, 6);

  preview_ = new wxStaticBitmap(
    page, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize(620, 300));
  preview_->SetBackgroundColour(wxColour(35, 35, 35));
  capture_list_ = new wxListCtrl(
    page, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxLC_REPORT | wxLC_SINGLE_SEL);
  capture_list_->AppendColumn(wxString::FromUTF8(u8"图像"), wxLIST_FORMAT_LEFT, 190);
  capture_list_->AppendColumn(wxString::FromUTF8(u8"角点"), wxLIST_FORMAT_LEFT, 70);
  capture_list_->AppendColumn(wxString::FromUTF8(u8"状态"), wxLIST_FORMAT_LEFT, 90);
  capture_list_->AppendColumn(wxString::FromUTF8(u8"检测诊断"), wxLIST_FORMAT_LEFT, 360);
  capture_list_->Bind(wxEVT_LIST_ITEM_ACTIVATED, [this](wxListEvent &event)
  {
    const auto index = static_cast<std::size_t>(event.GetIndex());
    if (index < workflow_.Captures().size())
    {
      const auto &detection = workflow_.Captures()[index].detection;
      wxMessageBox(From_Utf8(detection.diagnostic), From_Utf8(detection.image_id), wxOK, this);
    }
  });
  capture_button_ = new wxButton(
    page, wxID_ANY, wxString::FromUTF8(u8"采集当前帧"));
  import_button_ = new wxButton(
    page, wxID_ANY, wxString::FromUTF8(u8"导入图片目录"));
  remove_button_ = new wxButton(
    page, wxID_ANY, wxString::FromUTF8(u8"删除选中"));
  capture_status_ = new wxStaticText(page, wxID_ANY, wxEmptyString);
  capture_status_->Wrap(900);
  auto *actions = new wxBoxSizer(wxHORIZONTAL);
  actions->Add(capture_button_, 0, wxRIGHT, 6);
  actions->Add(import_button_, 0, wxRIGHT, 6);
  actions->Add(remove_button_, 0);
  auto *content = new wxBoxSizer(wxHORIZONTAL);
  content->Add(preview_, 3, wxEXPAND | wxRIGHT, 10);
  content->Add(capture_list_, 2, wxEXPAND);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxBOTTOM, 10);
  sizer->Add(new wxStaticText(
    page,
    wxID_ANY,
    wxString::FromUTF8(
      u8"覆盖中心、四边、四角，建议15～20张以上。普通针孔需增加X/Y倾斜视角；远心固定平面模式禁止改变姿态/高度，仅平移。")),
    0, wxEXPAND | wxBOTTOM, 10);
  sizer->Add(progress_box, 0, wxEXPAND | wxBOTTOM, 10);
  sizer->Add(content, 1, wxEXPAND | wxBOTTOM, 10);
  sizer->Add(actions, 0, wxBOTTOM, 8);
  sizer->Add(capture_status_, 0, wxEXPAND);
  page->SetSizer(sizer);
  capture_button_->Bind(
    wxEVT_BUTTON, &Camera_Intrinsic_Calibration_Dialog::On_Capture, this);
  import_button_->Bind(
    wxEVT_BUTTON, &Camera_Intrinsic_Calibration_Dialog::On_Import, this);
  remove_button_->Bind(
    wxEVT_BUTTON, &Camera_Intrinsic_Calibration_Dialog::On_Remove, this);
  progress_browse_button_->Bind(
    wxEVT_BUTTON,
    &Camera_Intrinsic_Calibration_Dialog::On_Browse_Progress,
    this);
  progress_inspect_button_->Bind(
    wxEVT_BUTTON,
    &Camera_Intrinsic_Calibration_Dialog::On_Inspect_Progress,
    this);
  progress_start_button_->Bind(
    wxEVT_BUTTON,
    &Camera_Intrinsic_Calibration_Dialog::On_Start_Progress,
    this);
  progress_stop_button_->Bind(
    wxEVT_BUTTON,
    &Camera_Intrinsic_Calibration_Dialog::On_Stop_Progress,
    this);
  progress_path_text_->Bind(wxEVT_TEXT, [this](wxCommandEvent &)
  {
    progress_checked_ = false;
    Refresh_Progress_Controls();
  });
  return page;
}

wxWindow *Camera_Intrinsic_Calibration_Dialog::Build_Review_Page()
{
  auto *page = new wxPanel(book_, wxID_ANY);
  auto *title = new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"步骤 4/5：检查数据并计算"));
  auto font = title->GetFont();
  font.SetPointSize(font.GetPointSize() + 3);
  font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(font);
  review_text_ = new wxStaticText(page, wxID_ANY, wxEmptyString);
  review_text_->Wrap(900);
  auto *notice = new wxStaticText(
    page,
    wxID_ANY,
    wxString::FromUTF8(
      u8"计算在后台执行并记录每张图的误差。针孔模式可能多轮剔除异常图；远心模式不自动剔除。\n"
      u8"位置覆盖和低RMS均不等于独立精度验证。远心模式仅估计固定平面映射，不求针孔内参或非线性畸变。"));
  notice->Wrap(900);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxBOTTOM, 24);
  sizer->Add(review_text_, 0, wxEXPAND | wxBOTTOM, 20);
  sizer->Add(notice, 0, wxEXPAND);
  calculation_gauge_ = new wxGauge(page, wxID_ANY, 100);
  calculation_status_ = new wxStaticText(page, wxID_ANY, wxString::FromUTF8(u8"尚未开始计算"));
  calculation_log_ = new wxTextCtrl(page, wxID_ANY, "", wxDefaultPosition, wxSize(-1, 180),
    wxTE_MULTILINE | wxTE_READONLY);
  sizer->Add(calculation_gauge_, 0, wxEXPAND | wxTOP, 12);
  sizer->Add(calculation_status_, 0, wxEXPAND | wxTOP, 6);
  sizer->Add(calculation_log_, 1, wxEXPAND | wxTOP, 8);
  calculation_timer_.SetOwner(this);
  Bind(wxEVT_TIMER, [this](wxTimerEvent &)
  {
    if (!calculating_) return;
    Drain_Calculation_Progress();
    calculation_gauge_->Pulse();
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - calculation_started_).count();
    calculation_status_->SetLabel(calculation_stage_ + wxString::Format(
      wxString::FromUTF8(u8" | 已耗时 %lld 秒（单轮求解无百分比回调）"), static_cast<long long>(elapsed)));
  }, calculation_timer_.GetId());
  page->SetSizer(sizer);
  return page;
}

wxWindow *Camera_Intrinsic_Calibration_Dialog::Build_Result_Page()
{
  auto *page = new wxPanel(book_, wxID_ANY);
  auto *title = new wxStaticText(
    page, wxID_ANY, wxString::FromUTF8(u8"步骤 5/5：验证并保存"));
  auto font = title->GetFont();
  font.SetPointSize(font.GetPointSize() + 3);
  font.SetWeight(wxFONTWEIGHT_BOLD);
  title->SetFont(font);
  result_text_ = new wxTextCtrl(
    page,
    wxID_ANY,
    wxEmptyString,
    wxDefaultPosition,
    wxDefaultSize,
    wxTE_MULTILINE | wxTE_READONLY);
  result_path_text_ = new wxStaticText(page, wxID_ANY, wxEmptyString);
  result_path_text_->Wrap(900);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxBOTTOM, 16);
  sizer->Add(result_text_, 1, wxEXPAND | wxBOTTOM, 12);
  sizer->Add(result_path_text_, 0, wxEXPAND);
  page->SetSizer(sizer);
  return page;
}

void Camera_Intrinsic_Calibration_Dialog::Refresh_Preparation()
{
  const auto status = camera_service_.Status();
  if (!camera_service_.Is_Open())
  {
    camera_status_->SetLabel(wxString::FromUTF8(
      u8"相机状态：未连接。可以继续并在采集步骤导入已有图片。"));
    return;
  }
  camera_status_->SetLabel(wxString::Format(
    wxString::FromUTF8(
      u8"相机状态：已连接\n型号：%s\n序列号：%s\n分辨率：%u × %u\n像素格式：%s"),
    From_Utf8(status.device_name),
    From_Utf8(status.serial_number),
    status.width,
    status.height,
    wxString::FromUTF8(
      jutze_camera::pixel_format_name(status.pixel_format))));
}

void Camera_Intrinsic_Calibration_Dialog::Refresh_Capture_List()
{
  capture_list_->DeleteAllItems();
  const auto &captures = workflow_.Captures();
  for (std::size_t index = 0; index < captures.size(); ++index)
  {
    const auto &capture = captures[index];
    const long row = capture_list_->InsertItem(
      static_cast<long>(index), From_Utf8(capture.detection.image_id));
    capture_list_->SetItem(
      row,
      1,
      capture.detection.chessboard_found
        ? wxString::Format("%zu", capture.detection.corners.size())
        : wxString::FromUTF8(u8"未检出"));
    capture_list_->SetItem(
      row,
      2,
      capture.enabled
        ? wxString::FromUTF8(u8"有效")
        : wxString::FromUTF8(u8"排除"));
    capture_list_->SetItem(row, 3, From_Utf8(capture.detection.diagnostic));
  }
  const auto readiness = workflow_.Readiness(10);
  capture_status_->SetLabel(wxString::Format(
    wxString::FromUTF8(
      u8"总图像：%zu，检出：%zu，有效：%zu，九宫格覆盖：%zu/9%s"),
    readiness.total_image_count,
    readiness.detected_image_count,
    readiness.enabled_image_count,
    readiness.covered_zone_count,
    readiness.ready ? wxString::FromUTF8(u8"，可以进入计算")
                    : wxString::FromUTF8(u8"，尚未满足计算条件")));
  Refresh_Navigation();
}

void Camera_Intrinsic_Calibration_Dialog::Refresh_Progress_Controls()
{
  if (!progress_start_button_)
  {
    return;
  }
  const bool available = progress_host_ != nullptr;
  progress_path_text_->Enable(available && !progress_active_ && !busy_);
  progress_browse_button_->Enable(available && !progress_active_ && !busy_);
  progress_inspect_button_->Enable(available && !progress_active_ && !busy_);
  progress_speed_spin_->Enable(available && !progress_active_ && !busy_);
  progress_settle_spin_->Enable(available && !progress_active_ && !busy_);
  progress_start_button_->Enable(
    available && progress_checked_ && !progress_active_ && !busy_);
  progress_stop_button_->Enable(available && progress_active_);
  if (!available)
  {
    progress_summary_text_->SetLabel(wxString::FromUTF8(
      u8"当前以离线模式打开，Progress 自动运动不可用。"));
  }
}

bool Camera_Intrinsic_Calibration_Dialog::Inspect_Progress(bool show_error)
{
  if (!progress_host_ || !progress_path_text_)
  {
    progress_checked_ = false;
    Refresh_Progress_Controls();
    return false;
  }
  const std::filesystem::path path(
    std::wstring(progress_path_text_->GetValue().wc_str()));
  Camera_Calibration_Progress_Summary summary;
  std::string error;
  if (!progress_host_->Inspect_Calibration_Progress(
        path, &summary, &error))
  {
    progress_checked_ = false;
    progress_summary_text_->SetLabel(
      wxString::FromUTF8(u8"检查失败：") + From_Utf8(error));
    if (show_error)
    {
      wxMessageBox(
        From_Utf8(error),
        wxString::FromUTF8(u8"标定 Progress 检查失败"),
        wxOK | wxICON_ERROR,
        this);
    }
    Refresh_Progress_Controls();
    return false;
  }
  progress_checked_ = true;
  progress_summary_text_->SetLabel(wxString::Format(
    wxString::FromUTF8(
      u8"检查通过：机器人 %s；总点 %zu；取图点 %zu；过渡点 %zu"),
    From_Utf8(summary.robot_model_id),
    summary.total_point_count,
    summary.capture_point_count,
    summary.transition_point_count));
  progress_gauge_->SetRange(
    std::max(1, static_cast<int>(summary.total_point_count)));
  progress_gauge_->SetValue(0);
  progress_status_text_->SetLabel(wxString::FromUTF8(
    u8"开始前请确认标定板固定、运动区域无人且机械臂已回到复位位置。"));
  Refresh_Progress_Controls();
  return true;
}

void Camera_Intrinsic_Calibration_Dialog::Refresh_Review()
{
  const auto readiness = workflow_.Readiness(10);
  wxString text = wxString::Format(
    wxString::FromUTF8(
      u8"图像总数：%zu\n角点检出：%zu\n参与计算：%zu\n九宫格覆盖：%zu/9\n\n"),
    readiness.total_image_count,
    readiness.detected_image_count,
    readiness.enabled_image_count,
    readiness.covered_zone_count);
  if (readiness.messages.empty())
  {
    text += wxString::FromUTF8(lens_choice_->GetSelection() == 1
      ? u8"基础覆盖通过。远心模式要求固定相对姿态/高度，仅平面内平移。\n单图RMS≤0.75px，最大角点残差≤2px，线性映射偏差≤1%；不自动剔除图像。\n通过仅代表训练数据一致，仍需独立验证精度。"
      : u8"数据数量和位置覆盖检查通过。仍需由最终求解判断透视约束是否充分。");
  }
  else
  {
    for (const std::string &message : readiness.messages)
    {
      text += wxString::FromUTF8(u8"• ") + From_Utf8(message) + "\n";
    }
  }
  review_text_->SetLabel(text);
}

void Camera_Intrinsic_Calibration_Dialog::Refresh_Navigation()
{
  const std::size_t page = book_->GetSelection();
  back_button_->Enable(!busy_ && !progress_active_ && page > 0 && page < 4);
  cancel_button_->Enable(!busy_ && !progress_active_);
  next_button_->SetLabel(page == 2
    ? wxString::FromUTF8(u8"审查数据")
    : (page == 3
      ? wxString::FromUTF8(u8"开始计算")
      : (page == 4
        ? wxString::FromUTF8(u8"保存并完成")
        : wxString::FromUTF8(u8"下一步"))));
  bool next_enabled = !busy_ && !progress_active_;
  if (page == 0)
  {
    next_enabled = next_enabled && (lens_choice_->GetSelection() == 0 || lens_choice_->GetSelection() == 1);
  }
  else if (page == 2 || page == 3)
  {
    next_enabled = next_enabled && workflow_.Readiness(10).ready;
  }
  else if (page == 4)
  {
    next_enabled = next_enabled && has_result_;
  }
  next_button_->Enable(next_enabled);
  Refresh_Progress_Controls();
}

void Camera_Intrinsic_Calibration_Dialog::Show_Preview(
  const Camera_2D_Display_Image &image,
  const camera_calibration::Image_Detection_Result *detection)
{
  if (image.width == 0 || image.height == 0 || image.rgb.empty())
  {
    return;
  }
  wxImage preview_image(
    static_cast<int>(image.width), static_cast<int>(image.height));
  if (!preview_image.IsOk() || !preview_image.GetData())
  {
    return;
  }
  std::memcpy(
    preview_image.GetData(), image.rgb.data(), image.rgb.size());
  const wxSize area = preview_->GetSize();
  const double scale = std::min(
    static_cast<double>(area.GetWidth()) / image.width,
    static_cast<double>(area.GetHeight()) / image.height);
  const int width = std::max(1, static_cast<int>(image.width * scale));
  const int height = std::max(1, static_cast<int>(image.height * scale));
  preview_image.Rescale(width, height, wxIMAGE_QUALITY_HIGH);
  wxBitmap bitmap(preview_image);
  if (detection && detection->chessboard_found)
  {
    wxMemoryDC dc(bitmap);
    dc.SetPen(wxPen(wxColour(0, 230, 90), 1));
    dc.SetBrush(wxBrush(wxColour(0, 230, 90)));
    for (const auto &corner : detection->corners)
    {
      dc.DrawCircle(
        wxPoint(
          static_cast<int>(corner.x * scale),
          static_cast<int>(corner.y * scale)),
        2);
    }
    dc.SelectObject(wxNullBitmap);
  }
  preview_->SetBitmap(bitmap);
}

void Camera_Intrinsic_Calibration_Dialog::Set_Busy(
  bool busy,
  const wxString &message)
{
  busy_ = busy;
  capture_button_->Enable(
    !translation_session_ && !busy && !progress_active_ && camera_service_.Latest_Frame() != nullptr);
  import_button_->Enable(!translation_session_ && !busy && !progress_active_);
  remove_button_->Enable(!translation_session_ && !busy && !progress_active_);
  if (!message.empty())
  {
    capture_status_->SetLabel(message);
  }
  Refresh_Navigation();
}

void Camera_Intrinsic_Calibration_Dialog::Drain_Calculation_Progress()
{
  std::vector<camera_calibration::Calibration_Progress> updates;
  std::vector<std::string> planar_updates;
  {
    std::lock_guard<std::mutex> lock(calculation_progress_mutex_);
    updates.swap(calculation_updates_);
    planar_updates.swap(planar_updates_);
  }
  for (const auto &message : planar_updates)
  {
    calculation_stage_ = From_Utf8(message);
    calculation_log_->AppendText(calculation_stage_ + "\n");
  }
  using Stage = camera_calibration::Calibration_Stage;
  for (const auto &update : updates)
  {
    wxString message;
    switch (update.stage)
    {
      case Stage::Solving:
        message = wxString::Format(wxString::FromUTF8(u8"第 %zu 轮：求解内参，%zu 张有效图片"), update.round, update.active_views);
        break;
      case Stage::Evaluating:
        message = wxString::Format(wxString::FromUTF8(u8"评估误差：总体 %.4f px，最差图片 %.4f px："), update.rms_px, update.worst_rms_px) + From_Utf8(update.image_id);
        break;
      case Stage::Rejecting:
        message = wxString::FromUTF8(u8"剔除异常图片并重新求解：") + From_Utf8(update.image_id);
        break;
      case Stage::Validating:
        message = wxString::FromUTF8(u8"正在检查焦距、数值有效性与结果质量");
        break;
      case Stage::Completed:
        message = wxString::Format(wxString::FromUTF8(u8"计算通过：保留 %zu 张，总体 RMS %.4f px"), update.active_views, update.rms_px);
        break;
    }
    calculation_stage_ = message;
    calculation_log_->AppendText(message + "\n");
  }
}

void Camera_Intrinsic_Calibration_Dialog::Start_Worker(
  std::function<Worker_Result()> task)
{
  if (worker_.joinable())
  {
    worker_.join();
  }
  Set_Busy(true, wxString::FromUTF8(u8"处理中，请稍候…"));
  worker_ = std::thread([this, task = std::move(task)]() mutable
  {
    Worker_Result result;
    try { result = task(); }
    catch (const std::exception &error) { result.error = error.what(); }
    catch (...) { result.error = "后台处理发生未知异常"; }
    {
      std::lock_guard<std::mutex> lock(worker_result_mutex_);
      worker_result_ = std::move(result);
    }
    if (!closing_.load())
    {
      wxQueueEvent(
        this,
        new wxThreadEvent(wxEVT_INTRINSIC_CALIBRATION_WORKER));
    }
  });
}

void Camera_Intrinsic_Calibration_Dialog::On_Worker_Complete(
  wxThreadEvent &)
{
  if (worker_.joinable())
  {
    worker_.join();
  }
  Worker_Result result;
  {
    std::lock_guard<std::mutex> lock(worker_result_mutex_);
    if (!worker_result_)
    {
      Set_Busy(false);
      return;
    }
    result = std::move(*worker_result_);
    worker_result_.reset();
  }
  if (calculating_)
  {
    calculation_timer_.Stop();
    Drain_Calculation_Progress();
    calculating_ = false;
    calculation_gauge_->SetValue(result.success ? 100 : 0);
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - calculation_started_).count();
    calculation_status_->SetLabel(wxString::Format(
      wxString::FromUTF8(u8"%s | 耗时 %lld 秒"),
      wxString::FromUTF8(result.success ? u8"计算完成" : u8"计算失败，原始数据保留，可返回检查"),
      static_cast<long long>(elapsed)));
    if (!result.success)
    {
      wxString explanation = From_Utf8(result.error);
      if (result.error.find("degenerate") != std::string::npos)
        explanation = wxString::FromUTF8(u8"焦距异常：当前图片无法可靠约束普通针孔内参。请核实是否为远心镜头，检查倾斜图的透视变化及棋盘大小；不要直接放宽阈值。\n原始诊断：") + explanation;
      calculation_log_->AppendText(explanation + "\n");
    }
  }
  Set_Busy(false);
  if (!result.success)
  {
    if ((result.operation == Worker_Operation::Progress_Capture ||
         (result.operation == Worker_Operation::None && progress_active_)) && progress_host_)
    {
      progress_host_->Complete_Calibration_Image_Processing(
        false, result.error);
    }
    wxMessageBox(
      From_Utf8(result.error),
      wxString::FromUTF8(u8"内参标定"),
      wxOK | wxICON_ERROR,
      this);
    capture_status_->SetLabel(From_Utf8(result.error));
    return;
  }
  if (result.operation == Worker_Operation::Capture ||
      result.operation == Worker_Operation::Progress_Capture ||
      result.operation == Worker_Operation::Import)
  {
    std::optional<camera_calibration::Image_Detection_Result>
      preview_detection;
    if (!result.preview_image.rgb.empty() && !result.captures.empty())
    {
      preview_detection = result.captures.front().detection;
    }
    const bool progress_accepted =
      result.operation == Worker_Operation::Progress_Capture &&
      !result.captures.empty() &&
      result.captures.front().detection.chessboard_found;
    for (auto &capture : result.captures)
    {
      if (translation_session_ && result.operation == Worker_Operation::Progress_Capture)
      {
        camera_calibration::Telecentric_Translation_Sample sample;
        sample.id = capture.detection.image_id;
        sample.captured_utc = wxDateTime::UNow().ToUTC().FormatISOCombined('T').ToStdString() + "Z";
        sample.point_id = result.progress_point_id;
        sample.retry_index = result.progress_retry_index;
        sample.role = result.progress_point_index % 4 == 3
          ? camera_calibration::Translation_Sample_Role::Validation
          : camera_calibration::Translation_Sample_Role::Training;
        sample.image_path = "images/" + capture.source_path.filename().u8string();
        sample.detection = capture.detection;
        sample.evidence = capture_evidence_;
        translation_session_->samples.push_back(std::move(sample));
        if (!Save_Translation_Checkpoint())
        {
          if (progress_host_) progress_host_->Complete_Calibration_Image_Processing(false, "平移会话归档失败，停止采集；原图保留");
          return;
        }
      }
      workflow_.Add_Capture(std::move(capture));
    }
    if (!result.preview_image.rgb.empty())
    {
      Show_Preview(
        result.preview_image,
        preview_detection ? &*preview_detection : nullptr);
    }
    Refresh_Capture_List();
    capture_status_->SetLabel(wxString::Format(
      wxString::FromUTF8(u8"处理完成，当前共有 %zu 张图像"),
      workflow_.Captures().size()));
    if (result.operation == Worker_Operation::Progress_Capture &&
        progress_host_)
    {
      progress_host_->Complete_Calibration_Image_Processing(
        progress_accepted, std::string());
    }
  }
  else if (result.operation == Worker_Operation::Calibrate)
  {
    planar_result_ = std::move(result.planar);
    if (planar_result_)
    {
      const auto &p = *planar_result_;
      const auto &a = p.pixels_from_plane_mm;
      const double det = a[0] * a[3] - a[1] * a[2];
      std::ostringstream text;
      text << std::setprecision(12)
           << "远心固定平面：训练数据一致性检查通过\n\n"
           << "图像分辨率：" << p.width << " × " << p.height << "\n"
           << "有效图像：" << p.views.size() << "（未自动剔除样本）\n"
           << "总体 RMS：" << p.rms_px << " px\n"
           << "最大角点残差：" << p.maximum_error_px << " px\n"
           << "最大线性映射偏差：" << p.maximum_linear_deviation_percent << "%\n\n"
           << "像素/毫米矩阵 A（棋盘平面轴，不是机器人轴）：\n"
           << a[0] << "  " << a[1] << "\n" << a[2] << "  " << a[3] << "\n\n"
           << "毫米/像素逆矩阵：\n"
           << a[3]/det << "  " << -a[1]/det << "\n" << -a[2]/det << "  " << a[0]/det << "\n\n"
           << "限制：只映射相对位移；棋盘轴有180°歧义。未校正非线性畸变。\n"
           << "未求解深度、绝对原点、Tool或手眼外参；未用于拼图。\n"
           << "以上为训练残差，不是独立测量精度；请另取已知长度验证。\n\n";
      for (const auto &view : p.views)
        text << view.image_id << ": RMS=" << view.rms_px << " px, max="
             << view.maximum_error_px << " px, linear=" << view.linear_deviation_percent << "%\n";
      result_text_->SetValue(From_Utf8(text.str()));
      result_path_text_->SetLabel(wxString::FromUTF8(u8"保存时确认来源与固定工况，生成独立远心配置；不覆盖旧标定。"));
      has_result_ = true;
      book_->SetSelection(4);
      Refresh_Navigation();
      return;
    }
    result_ = std::move(result.intrinsics);
    // Dataset identity is confirmed explicitly when saving; a connected camera
    // does not prove the provenance of imported images.
    result_.camera_serial_number.clear();
    result_.camera_model.clear();
    result_.pixel_format.clear();
    has_result_ = true;
    std::ostringstream text;
    text << std::setprecision(12)
         << "校验结果：通过\n\n"
         << "图像分辨率：" << result_.image_width << " × "
         << result_.image_height << "\n"
         << "有效图像：" << result_.accepted_views.size() << "\n"
         << "剔除图像：" << result_.rejected_views.size() << "\n"
         << "总体 RMS：" << result_.overall_rms_px << " px\n\n"
         << "fx = " << result_.camera_matrix[0] << "\n"
         << "fy = " << result_.camera_matrix[4] << "\n"
         << "cx = " << result_.camera_matrix[2] << "\n"
         << "cy = " << result_.camera_matrix[5] << "\n\n"
         << "k1 = " << result_.distortion[0] << "\n"
         << "k2 = " << result_.distortion[1] << "\n"
         << "p1 = " << result_.distortion[2] << "\n"
         << "p2 = " << result_.distortion[3] << "\n"
         << "k3 = " << result_.distortion[4] << "\n";
    result_text_->SetValue(From_Utf8(text.str()));
    result_path_text_->SetLabel(wxString::FromUTF8(
      u8"保存时确认相机/ROI/镜头来源，生成独立内参版本；不覆盖旧标定。"));
    book_->SetSelection(4);
    Refresh_Navigation();
  }
}

void Camera_Intrinsic_Calibration_Dialog::On_Back(wxCommandEvent &)
{
  const std::size_t page = book_->GetSelection();
  if (page > 0 && page < 4)
  {
    book_->SetSelection(page - 1);
    Refresh_Navigation();
  }
}

void Camera_Intrinsic_Calibration_Dialog::On_Next(wxCommandEvent &)
{
  const std::size_t page = book_->GetSelection();
  if (page == 0)
  {
    if (lens_choice_->GetSelection() != 0 && lens_choice_->GetSelection() != 1)
    {
      wxMessageBox(
        wxString::FromUTF8(u8"请先确认镜头类型，再选择对应模型。"),
        wxString::FromUTF8(u8"镜头模型不匹配"),
        wxOK | wxICON_WARNING,
        this);
      return;
    }
    book_->SetSelection(1);
  }
  else if (page == 1)
  {
    if (!Apply_Board_Configuration())
    {
      return;
    }
    book_->SetSelection(2);
    Refresh_Capture_List();
  }
  else if (page == 2)
  {
    Refresh_Review();
    book_->SetSelection(3);
  }
  else if (page == 3)
  {
    Start_Calibration();
  }
  else if (page == 4 && Save_Result())
  {
    EndModal(wxID_OK);
    return;
  }
  Refresh_Navigation();
}

void Camera_Intrinsic_Calibration_Dialog::On_Capture(wxCommandEvent &)
{
  if (translation_session_) return;
  const auto frame = camera_service_.Latest_Frame();
  if (!frame)
  {
    wxMessageBox(
      wxString::FromUTF8(u8"当前没有可采集的2D图像。"),
      wxString::FromUTF8(u8"采集失败"),
      wxOK | wxICON_WARNING,
      this);
    return;
  }
  const auto board = workflow_.Chessboard();
  const auto image_path = session_image_directory_ /
    ("frame_" + std::to_string(frame->m_frame_num) + ".png");
  for (const auto &capture : workflow_.Captures())
  {
    if (capture.source_path == image_path)
    {
      wxMessageBox(
        wxString::FromUTF8(u8"当前帧已经采集，请等待相机产生新帧后再采集。"),
        wxString::FromUTF8(u8"重复图像"),
        wxOK | wxICON_INFORMATION,
        this);
      return;
    }
  }
  Start_Worker([frame, board, image_path]()
  {
    Worker_Result result;
    result.operation = Worker_Operation::Capture;
    Camera_2D_Display_Image image;
    if (!Convert_Camera_2D_Frame(*frame, &image, &result.error))
    {
      return result;
    }
    const camera_calibration::Image_View view{
      image.rgb.data(),
      static_cast<int>(image.width),
      static_cast<int>(image.height),
      static_cast<std::size_t>(image.width) * 3,
      camera_calibration::Pixel_Format::RGB8};
    if (!camera_calibration::Save_Calibration_Image(
          image_path, view, &result.error))
    {
      return result;
    }
    camera_calibration::Intrinsic_Calibration_Session session(board);
    camera_calibration::Image_Detection_Result detection;
    if (!session.Add_Image(
          view,
          image_path.filename().string(),
          &detection,
          &result.error))
    {
      return result;
    }
    result.captures.push_back({detection, image_path, true});
    result.preview_image = std::move(image);
    result.success = true;
    return result;
  });
}

void Camera_Intrinsic_Calibration_Dialog::On_Import(wxCommandEvent &)
{
  if (translation_session_) return;
  wxDirDialog dialog(
    this,
    wxString::FromUTF8(u8"选择标定图片目录"),
    wxEmptyString,
    wxDD_DIR_MUST_EXIST);
  if (dialog.ShowModal() != wxID_OK)
  {
    return;
  }
  const std::filesystem::path directory(
    std::wstring(dialog.GetPath().wc_str()));
  const auto board = workflow_.Chessboard();
  Start_Worker([directory, board]()
  {
    Worker_Result result;
    result.operation = Worker_Operation::Import;
    std::vector<camera_calibration::Image_Detection_Result> detections;
    if (!camera_calibration::Detect_Calibration_Image_Directory(
          directory, board, &detections, &result.error))
    {
      return result;
    }
    for (auto &detection : detections)
    {
      const auto path = directory / detection.image_id;
      result.captures.push_back({
        std::move(detection), path, true});
    }
    result.success = true;
    return result;
  });
}

void Camera_Intrinsic_Calibration_Dialog::On_Remove(wxCommandEvent &)
{
  if (translation_session_) return;
  const long selection = capture_list_->GetNextItem(
    -1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
  if (selection == -1)
  {
    return;
  }
  workflow_.Remove_Capture(static_cast<std::size_t>(selection));
  Refresh_Capture_List();
}

void Camera_Intrinsic_Calibration_Dialog::On_Browse_Progress(wxCommandEvent &)
{
  wxFileDialog dialog(
    this,
    wxString::FromUTF8(u8"选择相机标定 Progress"),
    wxEmptyString,
    wxEmptyString,
    wxString::FromUTF8(u8"Progress XML (*.xml)|*.xml"),
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if (dialog.ShowModal() != wxID_OK)
  {
    return;
  }
  progress_path_text_->SetValue(dialog.GetPath());
  Inspect_Progress(true);
}

void Camera_Intrinsic_Calibration_Dialog::On_Inspect_Progress(wxCommandEvent &)
{
  Inspect_Progress(true);
}

void Camera_Intrinsic_Calibration_Dialog::On_Start_Progress(wxCommandEvent &)
{
  if (!progress_host_ || progress_active_ || busy_)
  {
    return;
  }
  if (!Inspect_Progress(true))
  {
    return;
  }
  if (lens_choice_->GetSelection() == 1 && wxMessageBox(wxString::FromUTF8(
        u8"远心固定平面模式不会自动修改Progress。\n请确认所选轨迹保持相同高度和姿态，仅沿标定板平面平移；不要使用包含倾斜点的旧针孔采图轨迹。\n已人工检查该轨迹符合要求？"),
        wxString::FromUTF8(u8"确认远心采图轨迹"),
        wxYES_NO | wxNO_DEFAULT | wxICON_WARNING, this) != wxYES) return;
  if (!workflow_.Captures().empty() &&
      wxMessageBox(
        wxString::FromUTF8(
          u8"开始 Progress 自动取图会清空当前采集记录，是否继续？"),
        wxString::FromUTF8(u8"确认自动取图"),
        wxYES_NO | wxICON_WARNING,
        this) != wxYES)
  {
    return;
  }
  if (wxMessageBox(
        wxString::FromUTF8(
          u8"请确认：\n"
          u8"1. 标定板已经固定；\n"
          u8"2. 机械臂运动区域无人且无障碍物；\n"
          u8"3. 已准备好使用急停。\n\n"
          u8"是否开始自动运动取图？"),
        wxString::FromUTF8(u8"运动安全确认"),
        wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
        this) != wxYES)
  {
    return;
  }

  const auto board = workflow_.Chessboard();
  if (translation_session_)
  {
    if (!translation_session_->samples.empty())
    {
      wxMessageBox(wxString::FromUTF8(u8"此会话已有记录。请重新打开向导建立新会话，避免混合采集批次。"));
      return;
    }
    Camera_2D_Image_Geometry geometry;
    std::string error;
    if (!camera_service_.Read_Image_Geometry(&geometry, &error))
    { wxMessageBox(From_Utf8(error)); return; }
    auto image = translation_session_->planar_profile.image;
    image.camera_serial = geometry.serial; image.width = geometry.width; image.height = geometry.height;
    image.roi_x = geometry.offset_x; image.roi_y = geometry.offset_y;
    image.roi_width = geometry.width; image.roi_height = geometry.height;
    image.binning_x = image.binning_y = 1;
    Camera_Calibration_Progress_Summary summary;
    if (!camera_calibration::Image_Configurations_Match(translation_session_->planar_profile.image, image, &error) ||
        !progress_host_->Inspect_Calibration_Progress(std::filesystem::path(progress_path_text_->GetValue().ToStdWstring()), &summary, &error))
    { wxMessageBox(From_Utf8(error)); return; }
    const auto &expected = translation_session_->planar_profile.result.board;
    if (board.inner_corner_columns != expected.inner_corner_columns || board.inner_corner_rows != expected.inner_corner_rows ||
        board.square_size_mm != expected.square_size_mm)
    { wxMessageBox(wxString::FromUTF8(u8"棋盘设置与绑定远心版本不一致")); return; }
    translation_session_->robot.robot_model_id = summary.robot_model_id;
    if (!Save_Translation_Checkpoint()) return;
  }
  workflow_.Reset(board);
  has_result_ = false;
  session_image_directory_ = translation_session_ ? translation_directory_ / "images" : Make_Session_Image_Directory();
  Refresh_Capture_List();

  Camera_Calibration_Progress_Options options;
  options.sample_robot_states = translation_session_.has_value();
  options.linear_velocity_mm_s = progress_speed_spin_->GetValue();
  options.settling_time_ms = progress_settle_spin_->GetValue();
  options.image_timeout_ms = 3000;
  options.maximum_detection_retries = 1;
  const std::filesystem::path path(
    std::wstring(progress_path_text_->GetValue().wc_str()));
  progress_active_ = true;
  Refresh_Navigation();
  std::string error;
  if (!progress_host_->Start_Calibration_Progress(
        path, options, this, &error))
  {
    progress_active_ = false;
    progress_status_text_->SetLabel(
      wxString::FromUTF8(u8"启动失败：") + From_Utf8(error));
    Refresh_Navigation();
    wxMessageBox(
      From_Utf8(error),
      wxString::FromUTF8(u8"无法开始自动取图"),
      wxOK | wxICON_ERROR,
      this);
  }
}

void Camera_Intrinsic_Calibration_Dialog::On_Stop_Progress(wxCommandEvent &)
{
  if (progress_host_ && progress_active_)
  {
    progress_stop_button_->Enable(false);
    progress_status_text_->SetLabel(wxString::FromUTF8(
      u8"正在请求停止机械臂，请等待停止确认…"));
    progress_host_->Request_Calibration_Progress_Stop();
  }
}

void Camera_Intrinsic_Calibration_Dialog::
On_Calibration_Progress_Update(
  const Camera_Calibration_Progress_Update &update)
{
  progress_active_ = update.active;
  if (progress_gauge_)
  {
    const int range = std::max(1, static_cast<int>(update.point_count));
    progress_gauge_->SetRange(range);
    progress_gauge_->SetValue(std::min(
      range,
      static_cast<int>(update.point_index + (update.active ? 1 : 0))));
  }
  if (progress_status_text_)
  {
    progress_status_text_->SetLabel(wxString::Format(
      wxString::FromUTF8(
        u8"点位 %zu/%zu，当前 P[%zu]，有效图 %zu，失败点 %zu%s\n%s"),
      update.point_count == 0 ? 0 : update.point_index + 1,
      update.point_count,
      update.point_id,
      update.accepted_image_count,
      update.rejected_point_count,
      update.retry_index == 0
        ? wxString()
        : wxString::Format(
            wxString::FromUTF8(u8"，第 %zu 次重拍"),
            update.retry_index),
      From_Utf8(update.message)));
  }
  Refresh_Navigation();
}

void Camera_Intrinsic_Calibration_Dialog::On_Calibration_Capture_Evidence(
  const camera_calibration::Translation_Capture_Evidence &evidence)
{
  capture_evidence_ = evidence;
}

void Camera_Intrinsic_Calibration_Dialog::On_Calibration_Progress_Frame(
  std::shared_ptr<const jutze_camera::camera_frame> frame,
  std::size_t point_index,
  std::size_t point_id,
  std::size_t retry_index)
{
  if (!frame || busy_)
  {
    if (progress_host_)
    {
      progress_host_->Complete_Calibration_Image_Processing(
        false,
        frame ? "上一张标定图片仍在处理" : "相机返回了空图片");
    }
    return;
  }
  if (translation_session_)
  {
    const auto &image = translation_session_->planar_profile.image;
    if (frame->m_device_serial != image.camera_serial || frame->m_width != image.width ||
        frame->m_height != image.height || capture_evidence_.camera_frame_id != frame->m_frame_num ||
        !capture_evidence_.software_triggered)
    {
      if (progress_host_) progress_host_->Complete_Calibration_Image_Processing(false,
        "平移会话的帧来源/尺寸或采集证据不匹配，停止采集");
      return;
    }
  }
  std::ostringstream filename;
  filename << 'P' << std::setw(3) << std::setfill('0') << point_id
           << "_try" << (retry_index + 1)
           << "_frame" << frame->m_frame_num << ".png";
  const auto image_path = session_image_directory_ / filename.str();
  const auto board = workflow_.Chessboard();
  Start_Worker([
    frame, board, image_path, point_index, point_id, retry_index]()
  {
    Worker_Result result;
    result.operation = Worker_Operation::Progress_Capture;
    result.progress_point_index = point_index;
    result.progress_point_id = point_id;
    result.progress_retry_index = retry_index;
    Camera_2D_Display_Image image;
    if (!Convert_Camera_2D_Frame(*frame, &image, &result.error))
    {
      return result;
    }
    const camera_calibration::Image_View view{
      image.rgb.data(),
      static_cast<int>(image.width),
      static_cast<int>(image.height),
      static_cast<std::size_t>(image.width) * 3,
      camera_calibration::Pixel_Format::RGB8};
    if (!camera_calibration::Save_Calibration_Image(
          image_path, view, &result.error))
    {
      return result;
    }
    camera_calibration::Intrinsic_Calibration_Session session(board);
    camera_calibration::Image_Detection_Result detection;
    if (!session.Add_Image(
          view,
          image_path.filename().string(),
          &detection,
          &result.error))
    {
      return result;
    }
    result.captures.push_back({detection, image_path, true});
    result.preview_image = std::move(image);
    result.success = true;
    return result;
  });
}

void Camera_Intrinsic_Calibration_Dialog::
On_Calibration_Progress_Finished(
  bool success,
  const std::string &message)
{
  progress_active_ = false;
  if (progress_gauge_ && success)
  {
    progress_gauge_->SetValue(progress_gauge_->GetRange());
  }
  progress_status_text_->SetLabel(
    (success ? wxString::FromUTF8(u8"完成：")
             : wxString::FromUTF8(u8"已中止：")) +
    From_Utf8(message));
  Refresh_Capture_List();
  Refresh_Navigation();
  if (!success && message.find("stopped") == std::string::npos)
  {
    wxMessageBox(
      From_Utf8(message),
      wxString::FromUTF8(u8"Progress 自动取图"),
      wxOK | wxICON_WARNING,
      this);
  }
}

void Camera_Intrinsic_Calibration_Dialog::On_Cancel(wxCommandEvent &)
{
  if (progress_active_ && progress_host_)
  {
    progress_stop_button_->Enable(false);
    progress_status_text_->SetLabel(wxString::FromUTF8(
      u8"正在请求停止机械臂，请等待停止确认…"));
    progress_host_->Request_Calibration_Progress_Stop();
    return;
  }
  EndModal(wxID_CANCEL);
}

void Camera_Intrinsic_Calibration_Dialog::On_Close(wxCloseEvent &event)
{
  if (progress_active_ && progress_host_)
  {
    progress_host_->Request_Calibration_Progress_Stop();
    event.Veto();
    return;
  }
  event.Skip();
}

bool Camera_Intrinsic_Calibration_Dialog::Apply_Board_Configuration()
{
  camera_calibration::Chessboard_Configuration board;
  board.inner_corner_columns = columns_spin_->GetValue();
  board.inner_corner_rows = rows_spin_->GetValue();
  board.square_size_mm = square_size_spin_->GetValue();
  std::string error;
  if (!camera_calibration::Validate_Chessboard_Configuration(board, &error))
  {
    wxMessageBox(
      From_Utf8(error),
      wxString::FromUTF8(u8"标定板参数错误"),
      wxOK | wxICON_ERROR,
      this);
    return false;
  }
  if (!workflow_.Captures().empty())
  {
    const auto &old = workflow_.Chessboard();
    if (old.inner_corner_columns != board.inner_corner_columns ||
        old.inner_corner_rows != board.inner_corner_rows ||
        old.square_size_mm != board.square_size_mm)
    {
      if (wxMessageBox(
            wxString::FromUTF8(u8"修改标定板参数会清空当前采集记录，是否继续？"),
            wxString::FromUTF8(u8"确认修改"),
            wxYES_NO | wxICON_WARNING,
            this) != wxYES)
      {
        return false;
      }
    }
  }
  const auto &old = workflow_.Chessboard();
  const bool changed =
    old.inner_corner_columns != board.inner_corner_columns ||
    old.inner_corner_rows != board.inner_corner_rows ||
    old.square_size_mm != board.square_size_mm;
  if (changed || workflow_.Captures().empty())
  {
    workflow_.Reset(board);
  }
  has_result_ = false;
  return true;
}

void Camera_Intrinsic_Calibration_Dialog::Prepare_Translation_Session()
{
  if (busy_ || progress_active_ || translation_session_) return;
  wxFileDialog file(this, wxString::FromUTF8(u8"选择已保存的远心固定平面版本"),
    wxString((application::Get_App_Paths().config_root / "CameraCalibration").wstring()), "",
    "XML (*.xml)|*.xml", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if (file.ShowModal() != wxID_OK) return;
  camera_calibration::Telecentric_Translation_Session session;
  std::string error;
  if (!camera_calibration::Load_Telecentric_Planar_Profile(
        std::filesystem::path(file.GetPath().ToStdWstring()), &session.planar_profile, &error))
  { wxMessageBox(From_Utf8(error)); return; }
  wxTextEntryDialog reference(this, wxString::FromUTF8(u8"填写机器人参考坐标来源标签（仅记录，不代表已核验BASE/TOOL）"),
    wxString::FromUTF8(u8"平移会话参考"));
  if (reference.ShowModal() != wxID_OK || reference.GetValue().Strip(wxString::both).empty()) return;
  session.robot.reference_frame_id = reference.GetValue().ToStdString(wxConvUTF8);
  wxTextEntryDialog board(this, wxString::FromUTF8(u8"填写固定标定板/安装位置ID；整个会话中不得移动标定板"),
    wxString::FromUTF8(u8"固定标定板"));
  if (board.ShowModal() != wxID_OK || board.GetValue().Strip(wxString::both).empty()) return;
  session.board_frame_id = board.GetValue().ToStdString(wxConvUTF8);
  if (wxMessageBox(wxString::FromUTF8(u8"本轮仅建立采集审计记录，不求解或启用拼图。\n将采集触发前后专用GET_STATE并检查端点稳定性（0.05mm/0.02°），但仍不能证明曝光时法兰位姿；BASE/TOOL和帧观察时刻按原始证据保存。\n确认镜头/对焦/光圈和固定工况与所选远心版本一致？\n建立会话不会启动机器人；运动仍需单独确认。"),
      wxString::FromUTF8(u8"确认采集范围"), wxYES_NO | wxNO_DEFAULT | wxICON_WARNING, this) != wxYES) return;
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  session.identity = {"translation-" + std::to_string(stamp),
    wxDateTime::UNow().ToUTC().FormatISOCombined('T').ToStdString() + "Z"};
  translation_directory_ = application::Get_App_Paths().config_root / "CameraCalibration" / "TranslationSessions" / session.identity.id;
  std::error_code ec;
  std::filesystem::create_directories(translation_directory_.parent_path(), ec);
  if (ec || !std::filesystem::create_directory(translation_directory_, ec))
  { wxMessageBox(From_Utf8("无法独占创建会话目录：" + ec.message())); return; }
  std::filesystem::create_directory(translation_directory_ / "images", ec);
  if (ec) { wxMessageBox(From_Utf8(ec.message())); return; }
  translation_session_ = std::move(session);
  workflow_.Reset(translation_session_->planar_profile.result.board);
  Refresh_Capture_List();
  const auto &configuration = translation_session_->planar_profile.result.board;
  columns_spin_->SetValue(configuration.inner_corner_columns); rows_spin_->SetValue(configuration.inner_corner_rows);
  square_size_spin_->SetValue(configuration.square_size_mm);
  lens_choice_->SetSelection(1); lens_choice_->Disable(); translation_button_->Disable();
  columns_spin_->Disable(); rows_spin_->Disable(); square_size_spin_->Disable();
  capture_button_->Disable(); import_button_->Disable(); remove_button_->Disable();
  camera_status_->SetLabel(wxString::FromUTF8(u8"平移采集会话已准备（位姿未核验，不可求解）。请在采图页选择已检查的平移Progress。\n") +
    wxString(translation_directory_.wstring()));
  Refresh_Navigation();
}

bool Camera_Intrinsic_Calibration_Dialog::Save_Translation_Checkpoint()
{
  if (!translation_session_) return false;
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const auto path = translation_directory_ / ("session-" + std::to_string(stamp) + ".xml");
  std::string error;
  if (!camera_calibration::Save_Telecentric_Translation_Session(path, *translation_session_, &error))
  { wxMessageBox(From_Utf8(error), wxString::FromUTF8(u8"保存平移会话失败")); return false; }
  progress_status_text_->SetLabel(wxString::FromUTF8(u8"已保存未核验采集记录：") + wxString(path.wstring()));
  return true;
}

void Camera_Intrinsic_Calibration_Dialog::Start_Calibration()
{
  if (translation_session_)
  {
    wxMessageBox(wxString::FromUTF8(u8"当前为平移采集审计会话，每张图及原始状态已另存。\n尚未具备曝光关联和参考坐标核验，本轮不提供映射求解；不能把缓存状态当作已验证法兰位姿。"));
    return;
  }
  const bool telecentric = lens_choice_->GetSelection() == 1;
  if (telecentric && wxMessageBox(wxString::FromUTF8(
      u8"此模式要求固定标定板、工作高度和相对姿态，仅进行平面内平移。\n"
      u8"确认已排除倾斜、变高度和旋转图？旧采图流程中的倾斜点不能直接混用。\n"
      u8"至少10张图、覆盖5个区域只是基础门槛，不代表测量精度已验证。"),
      wxString::FromUTF8(u8"确认远心固定平面数据"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) != wxYES) return;
  const auto captures = workflow_.Captures();
  const auto board = workflow_.Chessboard();
  {
    std::lock_guard<std::mutex> lock(calculation_progress_mutex_);
    calculation_updates_.clear();
    planar_updates_.clear();
  }
  calculation_log_->Clear();
  for (const auto &capture : captures)
    calculation_log_->AppendText(From_Utf8(capture.detection.image_id + ": " + capture.detection.diagnostic) + "\n");
  calculation_stage_ = wxString::FromUTF8(u8"正在准备有效角点数据");
  calculation_status_->SetLabel(calculation_stage_);
  calculation_gauge_->SetValue(0);
  calculation_started_ = std::chrono::steady_clock::now();
  calculating_ = true;
  calculation_timer_.Start(200);
  Start_Worker([this, captures, board, telecentric]() mutable
  {
    Worker_Result result;
    result.operation = Worker_Operation::Calibrate;
    if (telecentric)
    {
      std::vector<camera_calibration::Image_Detection_Result> views;
      for (const auto &capture : captures)
        if (capture.enabled && capture.detection.chessboard_found) views.push_back(capture.detection);
      camera_calibration::Telecentric_Planar_Options options;
      options.progress = [this](const std::string &message)
      {
        std::lock_guard<std::mutex> lock(calculation_progress_mutex_);
        planar_updates_.push_back(message);
      };
      camera_calibration::Telecentric_Planar_Result planar;
      result.success = camera_calibration::Calibrate_Telecentric_Plane(board, views, options, &planar, &result.error);
      if (result.success) result.planar = std::move(planar);
      return result;
    }
    application::Camera_Intrinsic_Calibration_Workflow workflow(board);
    for (auto &capture : captures)
    {
      workflow.Add_Capture(std::move(capture));
    }
    camera_calibration::Calibration_Options options;
    options.minimum_view_count = 10;
    options.maximum_view_rms_px = 1.0;
    options.maximum_outlier_removals = 6;
    options.progress = [this](const camera_calibration::Calibration_Progress &progress)
    {
      std::lock_guard<std::mutex> lock(calculation_progress_mutex_);
      calculation_updates_.push_back(progress);
    };
    result.success = workflow.Calibrate(
      options, &result.intrinsics, &result.error);
    return result;
  });
}

bool Camera_Intrinsic_Calibration_Dialog::Save_Result()
{
  const auto board = workflow_.Chessboard();
  Camera_Intrinsic_Profile_Dialog dialog(this, camera_service_,
    planar_result_ ? nullptr : &result_, &board, planar_result_.has_value(),
    planar_result_ ? &*planar_result_ : nullptr);
  return dialog.ShowModal() == wxID_OK;
}
