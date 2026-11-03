#ifndef CAMERA_CALIBRATION_INTRINSIC_UNDISTORTION_H_
#define CAMERA_CALIBRATION_INTRINSIC_UNDISTORTION_H_

#include "intrinsic_calibration.h"

namespace camera_calibration
{

// Undistorts Gray8/RGB8/BGR8 into tightly packed RGB8 at the input size.
// The output camera matrix K is the original K, with D = 0: no automatic
// resizing, optimal-camera-matrix adjustment, or cropping is performed.
// Pixels mapped outside the input are black (edges may therefore be black).
// The input image and intrinsics are never modified. On failure, *rgb is
// unchanged; error, if supplied, describes the failure (cleared on success).
//
// Image size must match the calibration size. K must be a standard OpenCV
// pinhole matrix [fx,0,cx; 0,fy,cy; 0,0,1]. The caller owns the input buffer
// and must supply readable rows of width * channels bytes at the given
// stride; Image_View has no buffer-size field, so allocation length cannot
// be checked here. Padded rows are supported; zero/undersized strides are not.
bool Undistort_Intrinsic_Image(
  const Image_View &image,
  const Camera_Intrinsics &intrinsics,
  std::vector<std::uint8_t> *rgb,
  std::string *error = nullptr);

} // namespace camera_calibration

#endif // CAMERA_CALIBRATION_INTRINSIC_UNDISTORTION_H_
