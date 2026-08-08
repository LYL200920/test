#ifndef PANEL_CAMERA_INTRINSIC_CALIBRATION_DIALOG_H_
#define PANEL_CAMERA_INTRINSIC_CALIBRATION_DIALOG_H_

#include "camera_2d_image_converter.h"
#include "camera_calibration_progress_host.h"
#include "camera_intrinsic_calibration_workflow.h"

#include <wx/dialog.h>

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

class Camera_2D_Service;
class wxButton;
class wxChoice;
class wxCloseEvent;
class wxGauge;
class wxListCtrl;
class wxSimplebook;
class wxSpinCtrl;
class wxSpinCtrlDouble;
class wxStaticBitmap;
class wxStaticText;
class wxTextCtrl;
class wxThreadEvent;

class Camera_Intrinsic_Calibration_Dialog final :
  public wxDialog,
  public Camera_Calibration_Progress_Observer
{
public:
  Camera_Intrinsic_Calibration_Dialog(
    wxWindow *parent,
    Camera_2D_Service &camera_service,
    Camera_Calibration_Progress_Host *progress_host = nullptr);
  ~Camera_Intrinsic_Calibration_Dialog() override;

  void On_Calibration_Progress_Update(
    const Camera_Calibration_Progress_Update &update) override;
  void On_Calibration_Progress_Frame(
    std::shared_ptr<const jutze_camera::camera_frame> frame,
    std::size_t point_index,
    std::size_t point_id,
    std::size_t retry_index) override;
  void On_Calibration_Progress_Finished(
    bool success,
    const std::string &message) override;

private:
  enum class Worker_Operation
  {
    None,
    Capture,
    Progress_Capture,
    Import,
    Calibrate
  };

  struct Worker_Result
  {
    Worker_Operation operation = Worker_Operation::None;
    bool success = false;
    std::string error;
    Camera_2D_Display_Image preview_image;
    std::vector<application::Camera_Calibration_Capture> captures;
    camera_calibration::Camera_Intrinsics intrinsics;
    std::size_t progress_point_index = 0;
    std::size_t progress_point_id = 0;
    std::size_t progress_retry_index = 0;
  };

  void Build_Ui();
  wxWindow *Build_Preparation_Page();
  wxWindow *Build_Board_Page();
  wxWindow *Build_Capture_Page();
  wxWindow *Build_Review_Page();
  wxWindow *Build_Result_Page();
  void Refresh_Preparation();
  void Refresh_Capture_List();
  void Refresh_Review();
  void Refresh_Navigation();
  void Show_Preview(
    const Camera_2D_Display_Image &image,
    const camera_calibration::Image_Detection_Result *detection = nullptr);
  void Set_Busy(bool busy, const wxString &message = wxString());
  void Start_Worker(std::function<Worker_Result()> task);
  void On_Worker_Complete(wxThreadEvent &event);
  void On_Back(wxCommandEvent &event);
  void On_Next(wxCommandEvent &event);
  void On_Capture(wxCommandEvent &event);
  void On_Import(wxCommandEvent &event);
  void On_Remove(wxCommandEvent &event);
  void On_Browse_Progress(wxCommandEvent &event);
  void On_Inspect_Progress(wxCommandEvent &event);
  void On_Start_Progress(wxCommandEvent &event);
  void On_Stop_Progress(wxCommandEvent &event);
  void On_Cancel(wxCommandEvent &event);
  void On_Close(wxCloseEvent &event);
  void Refresh_Progress_Controls();
  bool Inspect_Progress(bool show_error);
  bool Apply_Board_Configuration();
  void Start_Calibration();
  bool Save_Result();
  std::filesystem::path Default_Result_Path() const;

  Camera_2D_Service &camera_service_;
  Camera_Calibration_Progress_Host *progress_host_ = nullptr;
  application::Camera_Intrinsic_Calibration_Workflow workflow_;
  std::filesystem::path session_image_directory_;
  camera_calibration::Camera_Intrinsics result_;
  bool has_result_ = false;
  bool busy_ = false;
  bool progress_active_ = false;
  bool progress_checked_ = false;
  std::atomic<bool> closing_{false};
  std::thread worker_;
  std::mutex worker_result_mutex_;
  std::optional<Worker_Result> worker_result_;

  wxSimplebook *book_ = nullptr;
  wxChoice *lens_choice_ = nullptr;
  wxStaticText *camera_status_ = nullptr;
  wxSpinCtrl *columns_spin_ = nullptr;
  wxSpinCtrl *rows_spin_ = nullptr;
  wxSpinCtrlDouble *square_size_spin_ = nullptr;
  wxStaticBitmap *preview_ = nullptr;
  wxListCtrl *capture_list_ = nullptr;
  wxStaticText *capture_status_ = nullptr;
  wxTextCtrl *progress_path_text_ = nullptr;
  wxStaticText *progress_summary_text_ = nullptr;
  wxStaticText *progress_status_text_ = nullptr;
  wxGauge *progress_gauge_ = nullptr;
  wxSpinCtrl *progress_speed_spin_ = nullptr;
  wxSpinCtrl *progress_settle_spin_ = nullptr;
  wxStaticText *review_text_ = nullptr;
  wxTextCtrl *result_text_ = nullptr;
  wxStaticText *result_path_text_ = nullptr;
  wxButton *capture_button_ = nullptr;
  wxButton *import_button_ = nullptr;
  wxButton *remove_button_ = nullptr;
  wxButton *progress_browse_button_ = nullptr;
  wxButton *progress_inspect_button_ = nullptr;
  wxButton *progress_start_button_ = nullptr;
  wxButton *progress_stop_button_ = nullptr;
  wxButton *back_button_ = nullptr;
  wxButton *next_button_ = nullptr;
  wxButton *cancel_button_ = nullptr;
};

#endif // PANEL_CAMERA_INTRINSIC_CALIBRATION_DIALOG_H_
