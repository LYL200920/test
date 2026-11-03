#ifndef CAMERA_INTRINSIC_PROFILE_DIALOG_H_
#define CAMERA_INTRINSIC_PROFILE_DIALOG_H_

#include "calibration_geometry.h"
#include "telecentric_planar_calibration.h"
#include "camera_2d_image_converter.h"
#include <wx/dialog.h>
#include <thread>

class Camera_2D_Service;
class Camera_2D_Bitmap_Canvas;
class wxTextCtrl;
class wxStaticText;
class wxButton;

// Explicit profile selection and a frozen-frame preview; never modifies the
// live view, camera configuration, robot Tool or mosaic pipeline.
class Camera_Intrinsic_Profile_Dialog : public wxDialog
{
public:
  Camera_Intrinsic_Profile_Dialog(wxWindow *parent, Camera_2D_Service &camera,
    const camera_calibration::Camera_Intrinsics *result = nullptr,
    const camera_calibration::Chessboard_Configuration *board = nullptr,
    bool telecentric = false,
    const camera_calibration::Telecentric_Planar_Result *planar_result = nullptr);
  ~Camera_Intrinsic_Profile_Dialog() override;
private:
  bool Read_Configuration(camera_calibration::Camera_Image_Configuration *image);
  void Read_Camera();
  void Load_Profile();
  void Save_Profile();
  void Preview(bool live);
  void Set_Busy(bool busy);
  void Status(const std::string &text);
  Camera_2D_Service &camera_;
  camera_calibration::Intrinsic_Profile profile_;
  camera_calibration::Telecentric_Planar_Profile planar_profile_;
  bool telecentric_ = false;
  wxTextCtrl *setup_ = nullptr;
  bool has_profile_ = false;
  bool busy_ = false;
  bool saved_ = false;
  wxTextCtrl *serial_ = nullptr, *lens_ = nullptr;
  wxTextCtrl *numbers_[8]{};
  wxStaticText *status_ = nullptr;
  Camera_2D_Bitmap_Canvas *original_ = nullptr, *corrected_ = nullptr;
  std::vector<wxButton *> buttons_;
  std::thread worker_;
};
#endif
