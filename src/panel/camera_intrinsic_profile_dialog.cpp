#include "camera_intrinsic_profile_dialog.h"
#include "calibration_archive_repository.h"
#include "camera_intrinsics_repository.h"
#include "intrinsic_undistortion.h"
#include "camera_2d_service.h"
#include "camera_2d_image_view.h"
#include "app_paths.h"

#include <wx/button.h>
#include <wx/datetime.h>
#include <wx/filedlg.h>
#include <wx/image.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>

wxDEFINE_EVENT(wxEVT_INTRINSIC_PREVIEW_READY, wxThreadEvent);
namespace
{
struct Preview_Result
{
  Camera_2D_Display_Image original;
  std::vector<std::uint8_t> corrected;
  std::string error;
};
std::string Utf8(wxTextCtrl *text) { return text->GetValue().ToStdString(wxConvUTF8); }
void Show_Image(Camera_2D_Bitmap_Canvas *canvas, const Camera_2D_Display_Image &image,
  const std::vector<std::uint8_t> &rgb)
{
  wxImage bitmap(static_cast<int>(image.width), static_cast<int>(image.height));
  if (!bitmap.IsOk() || rgb.size() != static_cast<std::size_t>(image.width) * image.height * 3) return;
  std::memcpy(bitmap.GetData(), rgb.data(), rgb.size());
  canvas->Set_Bitmap(wxBitmap(bitmap));
  canvas->Fit_To_Window();
}
}
Camera_Intrinsic_Profile_Dialog::Camera_Intrinsic_Profile_Dialog(wxWindow *parent,
  Camera_2D_Service &camera, const camera_calibration::Camera_Intrinsics *result,
  const camera_calibration::Chessboard_Configuration *board, bool telecentric,
  const camera_calibration::Telecentric_Planar_Result *planar_result)
  : wxDialog(parent, wxID_ANY, wxString::FromUTF8(telecentric ? u8"远心固定平面配置" : u8"内参配置与去畸变对照"),
      wxDefaultPosition, telecentric ? wxSize(900, 560) : wxSize(1050, 780),
      wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER), camera_(camera), telecentric_(telecentric)
{
  auto *root = new wxBoxSizer(wxVERTICAL);
  root->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(
    u8"配置只用于本窗口验证，不会自动启用到实时画面、Tool或拼图。镜头配置ID须代表固定的焦距/对焦/光圈。\n"
    u8"离线图片必须填写其真实来源，不要借用当前连接相机的身份。ROI以binning前传感器像素计。")),
    0, wxEXPAND | wxALL, 8);
  auto *identity = new wxFlexGridSizer(2, 6, 8);
  identity->AddGrowableCol(1);
  identity->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"来源相机序列号")));
  serial_ = new wxTextCtrl(this, wxID_ANY); identity->Add(serial_, 1, wxEXPAND);
  identity->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"镜头配置ID（手动确认）")));
  lens_ = new wxTextCtrl(this, wxID_ANY); identity->Add(lens_, 1, wxEXPAND);
  if (telecentric_)
  {
    identity->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"固定工况ID（平面/高度/姿态）")));
    setup_ = new wxTextCtrl(this, wxID_ANY); identity->Add(setup_, 1, wxEXPAND);
  }
  root->Add(identity, 0, wxEXPAND | wxALL, 8);
  auto *geometry = new wxFlexGridSizer(4, 6, 8);
  geometry->AddGrowableCol(1); geometry->AddGrowableCol(3);
  const char *labels[] = {"Width", "Height", "ROI X", "ROI Y", "ROI Width", "ROI Height", "Binning X", "Binning Y"};
  for (int i = 0; i < 8; ++i)
  {
    geometry->Add(new wxStaticText(this, wxID_ANY, labels[i]), 0, wxALIGN_CENTER_VERTICAL);
    numbers_[i] = new wxTextCtrl(this, wxID_ANY, i >= 6 ? "1" : "");
    geometry->Add(numbers_[i], 1, wxEXPAND);
  }
  root->Add(geometry, 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
  auto *actions = new wxBoxSizer(wxHORIZONTAL);
  const auto button = [&](const char *label, auto action)
  {
    auto *b = new wxButton(this, wxID_ANY, wxString::FromUTF8(label));
    b->Bind(wxEVT_BUTTON, [action](wxCommandEvent &) { action(); });
    buttons_.push_back(b); actions->Add(b, 0, wxRIGHT, 4);
  };
  button("读取连接相机ROI", [this] { Read_Camera(); });
  button(telecentric_ ? "加载远心配置" : "加载内参", [this] { Load_Profile(); });
  button("保存新版本", [this] { Save_Profile(); });
  if (!telecentric_)
  {
    button("对照最新帧", [this] { Preview(true); });
    button("对照离线图片", [this] { Preview(false); });
  }
  root->Add(actions, 0, wxEXPAND | wxALL, 8);
  auto *images = new wxBoxSizer(wxHORIZONTAL);
  for (int i = 0; !telecentric_ && i < 2; ++i)
  {
    auto *column = new wxBoxSizer(wxVERTICAL);
    column->Add(new wxStaticText(this, wxID_ANY, wxString::FromUTF8(i == 0 ? "原图" : "去畸变：K不变、不裁剪，边缘可能为黑色")), 0, wxBOTTOM, 4);
    auto *canvas = new Camera_2D_Bitmap_Canvas(this);
    column->Add(canvas, 1, wxEXPAND);
    images->Add(column, 1, wxEXPAND | wxALL, 4);
    if (i == 0) original_ = canvas; else corrected_ = canvas;
  }
  root->Add(images, 1, wxEXPAND);
  status_ = new wxStaticText(this, wxID_ANY, wxString::FromUTF8(u8"未选择内参；请加载，或从标定结果页保存新配置。"));
  root->Add(status_, 0, wxEXPAND | wxALL, 8);
  auto *close = new wxButton(this, wxID_CANCEL, wxString::FromUTF8(u8"关闭"));
  close->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { if (!busy_) EndModal(saved_ ? wxID_OK : wxID_CANCEL); });
  root->Add(close, 0, wxALIGN_RIGHT | wxALL, 8);
  SetSizer(root);
  Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &e)
  { if (busy_) { if (e.CanVeto()) e.Veto(); } else EndModal(saved_ ? wxID_OK : wxID_CANCEL); });
  Bind(wxEVT_INTRINSIC_PREVIEW_READY, [this](wxThreadEvent &event)
  {
    if (worker_.joinable()) worker_.join();
    auto value = event.GetPayload<std::shared_ptr<Preview_Result>>();
    Set_Busy(false);
    if (!value->error.empty()) { Status(value->error); return; }
    Show_Image(original_, value->original, value->original.rgb);
    Show_Image(corrected_, value->original, value->corrected);
    Status("已完成冻结帧对照；原图未修改。当前内参ID：" + profile_.identity.id);
  });
  if (telecentric_)
  {
    if (planar_result)
    {
      planar_profile_.result = *planar_result;
      has_profile_ = true;
      numbers_[0]->SetValue(std::to_string(planar_result->width));
      numbers_[1]->SetValue(std::to_string(planar_result->height));
    }
    Status("远心配置仅含固定平面位移映射；未校正非线性畸变，不含Tool/手眼外参。请确认原图来源和固定工况后保存。更换平面、高度或相对姿态后须重新验证。");
  }
  if (!telecentric_ && result && board)
  {
    profile_.intrinsics = *result; profile_.board = *board;
    profile_.intrinsics.camera_serial_number.clear();
    profile_.intrinsics.camera_model.clear(); profile_.intrinsics.pixel_format.clear();
    has_profile_ = true;
    numbers_[0]->SetValue(std::to_string(result->image_width));
    numbers_[1]->SetValue(std::to_string(result->image_height));
    Status("新计算结果尚未绑定来源。请填写真实相机/ROI/镜头配置，再保存新版本。");
  }
}
Camera_Intrinsic_Profile_Dialog::~Camera_Intrinsic_Profile_Dialog()
{
  if (worker_.joinable()) worker_.join();
  DeletePendingEvents();
}
void Camera_Intrinsic_Profile_Dialog::Status(const std::string &text)
{
  status_->SetLabel(wxString::FromUTF8(text)); status_->SetToolTip(status_->GetLabel());
  status_->Wrap(std::max(200, GetClientSize().x - 20)); Layout();
}
void Camera_Intrinsic_Profile_Dialog::Set_Busy(bool busy)
{
  busy_ = busy;
  for (auto *button : buttons_) button->Enable(!busy);
  serial_->Enable(!busy); lens_->Enable(!busy);
  for (auto *number : numbers_) number->Enable(!busy);
}
bool Camera_Intrinsic_Profile_Dialog::Read_Configuration(camera_calibration::Camera_Image_Configuration *image)
{
  camera_calibration::Camera_Image_Configuration value;
  value.camera_serial = Utf8(serial_); value.lens_configuration_id = Utf8(lens_);
  int *fields[] = {&value.width, &value.height, &value.roi_x, &value.roi_y,
    &value.roi_width, &value.roi_height, &value.binning_x, &value.binning_y};
  for (int i = 0; i < 8; ++i)
  {
    long number = 0;
    if (!numbers_[i]->GetValue().ToLong(&number) || number < 0 || number > std::numeric_limits<int>::max())
    { Status("请完整填写有效的图像尺寸/ROI/binning，未知值不能默认猜测"); return false; }
    *fields[i] = static_cast<int>(number);
  }
  std::string error;
  if (!camera_calibration::Validate_Image_Configuration(value, &error)) { Status(error); return false; }
  *image = value; return true;
}
void Camera_Intrinsic_Profile_Dialog::Read_Camera()
{
  Camera_2D_Image_Geometry value; std::string error;
  if (!camera_.Read_Image_Geometry(&value, &error)) { Status(error); return; }
  serial_->SetValue(wxString::FromUTF8(value.serial));
  const int fields[] = {value.width, value.height, value.offset_x, value.offset_y, value.width, value.height, 1, 1};
  for (int i = 0; i < 8; ++i) numbers_[i]->SetValue(std::to_string(fields[i]));
  Status("已读取连接相机的ROI；请确认这些设置与原始标定图片一致，并填写镜头配置ID。");
}
void Camera_Intrinsic_Profile_Dialog::Load_Profile()
{
  wxFileDialog dialog(this, wxString::FromUTF8(u8"加载内参版本或旧内参XML"),
    wxString((application::Get_App_Paths().config_root / "CameraCalibration").wstring()), "",
    "XML (*.xml)|*.xml", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if (dialog.ShowModal() != wxID_OK) return;
  const std::filesystem::path path(dialog.GetPath().ToStdWstring());
  camera_calibration::Calibration_Archive archive; std::string error;
  if (telecentric_)
  {
    camera_calibration::Telecentric_Planar_Profile candidate;
    if (!camera_calibration::Load_Telecentric_Planar_Profile(path, &candidate, &error))
    { Status(error); return; }
    planar_profile_ = std::move(candidate);
    const auto &v = planar_profile_.image;
    serial_->SetValue(wxString::FromUTF8(v.camera_serial));
    lens_->SetValue(wxString::FromUTF8(v.lens_configuration_id));
    setup_->SetValue(wxString::FromUTF8(planar_profile_.fixed_setup_id));
    const int fields[] = {v.width,v.height,v.roi_x,v.roi_y,v.roi_width,v.roi_height,v.binning_x,v.binning_y};
    for (int i = 0; i < 8; ++i) numbers_[i]->SetValue(std::to_string(fields[i]));
    has_profile_ = true;
    Status("已加载远心固定平面版本：" + planar_profile_.identity.id +
      "；RMS=" + std::to_string(planar_profile_.result.rms_px) + " px。未校正畸变，未全局启用。");
    return;
  }
  if (camera_calibration::Load_Calibration_Archive(path, &archive, &error) && archive.intrinsic)
  {
    profile_ = *archive.intrinsic;
    const auto &v = profile_.image;
    serial_->SetValue(wxString::FromUTF8(v.camera_serial)); lens_->SetValue(wxString::FromUTF8(v.lens_configuration_id));
    const int fields[] = {v.width,v.height,v.roi_x,v.roi_y,v.roi_width,v.roi_height,v.binning_x,v.binning_y};
    for (int i=0;i<8;++i) numbers_[i]->SetValue(std::to_string(fields[i]));
    has_profile_ = true;
    Status("已选择内参版本：" + profile_.identity.id + "（仅本窗口，未全局启用）");
  }
  else
  {
    camera_calibration::Intrinsic_Profile legacy;
    if (!camera_calibration::Load_Camera_Intrinsics(path, &legacy.board, &legacy.intrinsics, &error))
    { Status(error); return; }
    profile_ = std::move(legacy); has_profile_ = true;
    serial_->SetValue(wxString::FromUTF8(profile_.intrinsics.camera_serial_number)); lens_->Clear();
    for (int i=0;i<8;++i) numbers_[i]->Clear();
    numbers_[0]->SetValue(std::to_string(profile_.intrinsics.image_width));
    numbers_[1]->SetValue(std::to_string(profile_.intrinsics.image_height));
    Status("旧格式缺少ROI/镜头元信息：请人工确认并补全，保存为新版本后再使用。旧文件不会修改。");
  }
  original_->Clear_Bitmap(); corrected_->Clear_Bitmap();
}
void Camera_Intrinsic_Profile_Dialog::Save_Profile()
{
  if (!has_profile_) { Status("请先计算或加载内参"); return; }
  camera_calibration::Camera_Image_Configuration image;
  if (!Read_Configuration(&image)) return;
  if (telecentric_)
  {
    auto candidate = planar_profile_;
    candidate.image = image;
    candidate.fixed_setup_id = Utf8(setup_);
    std::string error;
    if (!planar_profile_.identity.id.empty() &&
        !camera_calibration::Match_Telecentric_Planar_Profile(planar_profile_, image, candidate.fixed_setup_id, &error))
    { Status("不能将已标定版本重新绑定到其他设备或工况：" + error); return; }
    const auto stamp = std::chrono::system_clock::now().time_since_epoch();
    candidate.identity.id = "telecentric-plane-" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
    candidate.identity.created_utc = wxDateTime::UNow().ToUTC().FormatISOCombined('T').ToStdString() + "Z";
    if (!camera_calibration::Match_Telecentric_Planar_Profile(candidate, image, candidate.fixed_setup_id, &error))
    { Status(error); return; }
    if (wxMessageBox(wxString::FromUTF8(u8"确认全部原图属于填写的相机/ROI/镜头配置，且固定工况ID代表相同平面、高度及相对姿态？\n此版本只含平面位移映射，不包含非线性畸变校正或手眼外参，不会自动启用到拼图。"),
        wxString::FromUTF8(u8"确认远心标定来源"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) != wxYES) return;
    const auto directory = application::Get_App_Paths().config_root / "CameraCalibration";
    std::error_code ec; std::filesystem::create_directories(directory, ec);
    if (ec) { Status(ec.message()); return; }
    const auto path = directory / (candidate.identity.id + ".xml");
    if (!camera_calibration::Save_Telecentric_Planar_Profile(path, candidate, &error))
    { Status(error); return; }
    planar_profile_ = std::move(candidate); saved_ = true;
    Status("已保存独立远心版本（未全局启用）：" + path.u8string());
    return;
  }
  if (image.width != profile_.intrinsics.image_width || image.height != profile_.intrinsics.image_height)
  { Status("来源尺寸与标定结果不一致"); return; }
  if (!profile_.intrinsics.camera_serial_number.empty() &&
      profile_.intrinsics.camera_serial_number != "offline" &&
      profile_.intrinsics.camera_serial_number != image.camera_serial)
  { Status("原标定记录的相机序列号与填写值不一致，不能重新标记设备身份"); return; }
  if (!profile_.identity.id.empty())
  {
    std::string error;
    if (!camera_calibration::Image_Configurations_Match(profile_.image, image, &error))
    { Status("不可把已标定版本重新标记为其他设备或配置：" + error); return; }
  }
  if (wxMessageBox(wxString::FromUTF8(u8"确认填写的序列号、ROI、binning和镜头配置与全部标定原图一致？\n此操作只保存新版本，不自动启用或覆盖旧文件。"),
      wxString::FromUTF8(u8"确认标定来源"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) != wxYES) return;
  auto candidate = profile_;
  candidate.image = image;
  candidate.intrinsics.camera_serial_number = image.camera_serial;
  const auto stamp = std::chrono::system_clock::now().time_since_epoch();
  candidate.identity.id = "intrinsic-" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
  candidate.identity.created_utc = wxDateTime::UNow().ToUTC().FormatISOCombined('T').ToStdString() + "Z";
  const auto directory = application::Get_App_Paths().config_root / "CameraCalibration";
  std::error_code ec; std::filesystem::create_directories(directory, ec);
  if (ec) { Status(ec.message()); return; }
  camera_calibration::Calibration_Archive archive; archive.intrinsic = candidate;
  const auto path = directory / (candidate.identity.id + ".xml");
  std::string error;
  if (!camera_calibration::Save_Calibration_Archive(path, archive, &error)) { Status(error); return; }
  profile_ = std::move(candidate); saved_ = true;
  Status("已保存新版本（未全局启用）：" + path.u8string());
}
void Camera_Intrinsic_Profile_Dialog::Preview(bool live)
{
  if (!has_profile_ || profile_.identity.id.empty())
  { Status("请先加载带元信息的版本，或补全来源并保存新版本"); return; }
  camera_calibration::Camera_Image_Configuration configuration;
  if (!Read_Configuration(&configuration)) return;
  std::string error;
  if (!camera_calibration::Image_Configurations_Match(profile_.image, configuration, &error))
  { Status(error); return; }
  std::shared_ptr<const jutze_camera::camera_frame> frame;
  Camera_2D_Display_Image image;
  if (live)
  {
    Camera_2D_Image_Geometry geometry;
    if (!camera_.Read_Image_Geometry(&geometry, &error)) { Status(error); return; }
    auto actual = configuration;
    actual.camera_serial = geometry.serial; actual.width = geometry.width; actual.height = geometry.height;
    actual.roi_x = geometry.offset_x; actual.roi_y = geometry.offset_y;
    actual.roi_width = geometry.width; actual.roi_height = geometry.height;
    actual.binning_x = actual.binning_y = 1;
    if (!camera_calibration::Image_Configurations_Match(profile_.image, actual, &error))
    { Status(error); return; }
    if (wxMessageBox(wxString::FromUTF8(u8"设备与ROI已匹配。请确认镜头、对焦和光圈仍对应此镜头配置ID；这些物理设置无法自动读取。"),
      wxString::FromUTF8(u8"确认镜头设置"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) != wxYES) return;
    frame = camera_.Latest_Frame();
    if (!frame || frame->m_device_serial != geometry.serial || frame->m_width != geometry.width || frame->m_height != geometry.height)
    { Status("没有与当前相机配置匹配的最新帧；本按钮不会触发相机"); return; }
  }
  else
  {
    wxFileDialog dialog(this, wxString::FromUTF8(u8"选择与标定来源一致的原图"), "", "",
      "Images|*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dialog.ShowModal() != wxID_OK) return;
    wxImage source;
    if (!source.LoadFile(dialog.GetPath()) || !source.IsOk()) { Status("读取原图失败"); return; }
    if (source.GetWidth() != configuration.width || source.GetHeight() != configuration.height)
    { Status("原图尺寸与内参不一致"); return; }
    if (wxMessageBox(wxString::FromUTF8(u8"离线图片无法自动验证序列号和ROI，确认此图来自当前内参的同一相机、ROI和镜头设置？"),
      wxString::FromUTF8(u8"确认图片来源"), wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION, this) != wxYES) return;
    image.width = source.GetWidth(); image.height = source.GetHeight();
    image.rgb.assign(source.GetData(), source.GetData() + static_cast<std::size_t>(image.width) * image.height * 3);
  }
  if (worker_.joinable()) worker_.join();
  original_->Clear_Bitmap(); corrected_->Clear_Bitmap();
  Set_Busy(true); Status("正在后台生成去畸变对照...");
  const auto intrinsics = profile_.intrinsics;
  worker_ = std::thread([this, frame, image = std::move(image), intrinsics]() mutable
  {
    auto result = std::make_shared<Preview_Result>();
    try
    {
      if (frame && !Convert_Camera_2D_Frame(*frame, &image, &result->error)) {}
      else
      {
        result->original = std::move(image);
        const auto &original = result->original;
        camera_calibration::Undistort_Intrinsic_Image(
          {original.rgb.data(), static_cast<int>(original.width), static_cast<int>(original.height),
           static_cast<std::size_t>(original.width) * 3, camera_calibration::Pixel_Format::RGB8},
          intrinsics, &result->corrected, &result->error);
      }
    }
    catch (const std::exception &e) { result->error = e.what(); }
    auto *event = new wxThreadEvent(wxEVT_INTRINSIC_PREVIEW_READY);
    event->SetPayload(result); wxQueueEvent(this, event);
  });
}
