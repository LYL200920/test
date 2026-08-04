#include "camera_intrinsic_calibration_dialog.h"

#include "app_paths.h"
#include "camera_2d_service.h"
#include "camera_intrinsics_repository.h"
#include "intrinsic_calibration_dataset.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/dirdlg.h>
#include <wx/dcmemory.h>
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

std::string Safe_File_Component(std::string text)
{
  for (char &character : text)
  {
    const bool valid =
      (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') || character == '-' ||
      character == '_';
    if (!valid)
    {
      character = '_';
    }
  }
  return text.empty() ? "offline" : text;
}

} // namespace

Camera_Intrinsic_Calibration_Dialog::
Camera_Intrinsic_Calibration_Dialog(
  wxWindow *parent,
  Camera_2D_Service &camera_service)
  : wxDialog(
      parent,
      wxID_ANY,
      wxString::FromUTF8(u8"2D相机内参标定向导"),
      wxDefaultPosition,
      wxSize(1080, 760),
      wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
    camera_service_(camera_service),
    session_image_directory_(Make_Session_Image_Directory())
{
  Build_Ui();
  Bind(
    wxEVT_INTRINSIC_CALIBRATION_WORKER,
    &Camera_Intrinsic_Calibration_Dialog::On_Worker_Complete,
    this);
  Refresh_Preparation();
  Refresh_Navigation();
  CentreOnParent();
}

Camera_Intrinsic_Calibration_Dialog::
~Camera_Intrinsic_Calibration_Dialog()
{
  closing_.store(true);
  if (worker_.joinable())
  {
    worker_.join();
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
  lens_choice_->Append(wxString::FromUTF8(u8"远心镜头"));
  lens_choice_->Append(wxString::FromUTF8(u8"不确定"));
  lens_choice_->SetSelection(0);
  auto *notice = new wxStaticText(
    page,
    wxID_ANY,
    wxString::FromUTF8(
      u8"标定期间必须保持分辨率、ROI、焦距和光圈不变。相机未连接时仍可在后续步骤导入已有图片。\n"
      u8"当前版本的正式求解模型为普通针孔 Brown-5；远心镜头不能使用该模型。"));
  notice->Wrap(900);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxBOTTOM, 24);
  sizer->Add(camera_status_, 0, wxEXPAND | wxBOTTOM, 20);
  sizer->Add(
    new wxStaticText(page, wxID_ANY, wxString::FromUTF8(u8"镜头类型")),
    0, wxBOTTOM, 6);
  sizer->Add(lens_choice_, 0, wxEXPAND | wxBOTTOM, 20);
  sizer->Add(notice, 0, wxEXPAND);
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
  preview_ = new wxStaticBitmap(
    page, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize(620, 430));
  preview_->SetBackgroundColour(wxColour(35, 35, 35));
  capture_list_ = new wxListCtrl(
    page, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxLC_REPORT | wxLC_SINGLE_SEL);
  capture_list_->AppendColumn(wxString::FromUTF8(u8"图像"), wxLIST_FORMAT_LEFT, 190);
  capture_list_->AppendColumn(wxString::FromUTF8(u8"角点"), wxLIST_FORMAT_LEFT, 70);
  capture_list_->AppendColumn(wxString::FromUTF8(u8"状态"), wxLIST_FORMAT_LEFT, 90);
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
      u8"依次覆盖中心、四边、四角，并采集绕 X/Y 两轴正负倾斜的图像。建议 15～20 张以上。")),
    0, wxEXPAND | wxBOTTOM, 10);
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
      u8"点击“开始计算”后将在后台求解。结果必须同时通过重投影误差和物理合理性检查；低 RMS 的退化解不会被保存。"));
  notice->Wrap(900);
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(title, 0, wxBOTTOM, 24);
  sizer->Add(review_text_, 0, wxEXPAND | wxBOTTOM, 20);
  sizer->Add(notice, 0, wxEXPAND);
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
    text += wxString::FromUTF8(u8"数据数量和位置覆盖检查通过。仍需由最终求解判断透视约束是否充分。");
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
  back_button_->Enable(!busy_ && page > 0 && page < 4);
  cancel_button_->Enable(!busy_);
  next_button_->SetLabel(page == 2
    ? wxString::FromUTF8(u8"审查数据")
    : (page == 3
      ? wxString::FromUTF8(u8"开始计算")
      : (page == 4
        ? wxString::FromUTF8(u8"保存并完成")
        : wxString::FromUTF8(u8"下一步"))));
  bool next_enabled = !busy_;
  if (page == 0)
  {
    next_enabled = next_enabled && lens_choice_->GetSelection() == 0;
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
  capture_button_->Enable(!busy && camera_service_.Latest_Frame() != nullptr);
  import_button_->Enable(!busy);
  remove_button_->Enable(!busy);
  if (!message.empty())
  {
    capture_status_->SetLabel(message);
  }
  Refresh_Navigation();
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
    Worker_Result result = task();
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
  Set_Busy(false);
  if (!result.success)
  {
    wxMessageBox(
      From_Utf8(result.error),
      wxString::FromUTF8(u8"内参标定"),
      wxOK | wxICON_ERROR,
      this);
    capture_status_->SetLabel(From_Utf8(result.error));
    return;
  }
  if (result.operation == Worker_Operation::Capture ||
      result.operation == Worker_Operation::Import)
  {
    std::optional<camera_calibration::Image_Detection_Result>
      preview_detection;
    if (!result.preview_image.rgb.empty() && !result.captures.empty())
    {
      preview_detection = result.captures.front().detection;
    }
    for (auto &capture : result.captures)
    {
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
  }
  else if (result.operation == Worker_Operation::Calibrate)
  {
    result_ = std::move(result.intrinsics);
    const auto camera_status = camera_service_.Status();
    result_.camera_serial_number = camera_status.serial_number;
    result_.camera_model = camera_status.device_name;
    result_.pixel_format = jutze_camera::pixel_format_name(
      camera_status.pixel_format);
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
    result_path_text_->SetLabel(
      wxString::FromUTF8(u8"保存位置：") +
      Default_Result_Path().wstring());
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
    if (lens_choice_->GetSelection() != 0)
    {
      wxMessageBox(
        wxString::FromUTF8(u8"远心镜头不能使用当前针孔模型，请先确认镜头类型。"),
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
  const long selection = capture_list_->GetNextItem(
    -1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
  if (selection == -1)
  {
    return;
  }
  workflow_.Remove_Capture(static_cast<std::size_t>(selection));
  Refresh_Capture_List();
}

void Camera_Intrinsic_Calibration_Dialog::On_Cancel(wxCommandEvent &)
{
  EndModal(wxID_CANCEL);
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

void Camera_Intrinsic_Calibration_Dialog::Start_Calibration()
{
  const auto captures = workflow_.Captures();
  const auto board = workflow_.Chessboard();
  Start_Worker([captures, board]() mutable
  {
    Worker_Result result;
    result.operation = Worker_Operation::Calibrate;
    application::Camera_Intrinsic_Calibration_Workflow workflow(board);
    for (auto &capture : captures)
    {
      workflow.Add_Capture(std::move(capture));
    }
    camera_calibration::Calibration_Options options;
    options.minimum_view_count = 10;
    options.maximum_view_rms_px = 1.0;
    options.maximum_outlier_removals = 6;
    result.success = workflow.Calibrate(
      options, &result.intrinsics, &result.error);
    return result;
  });
}

std::filesystem::path
Camera_Intrinsic_Calibration_Dialog::Default_Result_Path() const
{
  const auto status = camera_service_.Status();
  const std::string serial = Safe_File_Component(status.serial_number);
  return application::Get_App_Paths().config_root /
         "CameraCalibration" /
         (serial + "_" + std::to_string(result_.image_width) + "x" +
          std::to_string(result_.image_height) + ".xml");
}

bool Camera_Intrinsic_Calibration_Dialog::Save_Result()
{
  std::string error;
  const auto path = Default_Result_Path();
  if (!camera_calibration::Save_Camera_Intrinsics(
        path, workflow_.Chessboard(), result_, &error))
  {
    wxMessageBox(
      From_Utf8(error),
      wxString::FromUTF8(u8"保存内参失败"),
      wxOK | wxICON_ERROR,
      this);
    return false;
  }
  result_path_text_->SetLabel(
    wxString::FromUTF8(u8"已保存：") + path.wstring());
  return true;
}
