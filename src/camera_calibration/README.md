# Camera intrinsic calibration module

This directory owns camera intrinsic calibration and deliberately stays
separate from camera acquisition, robot poses, hand-eye calibration and image
stitching.

Layers:

- `intrinsic_calibration.cpp`: image-view API, chessboard corner detection,
  Brown-5 pinhole calibration, per-view RMS outlier removal and degenerate
  solution rejection.
- `intrinsic_calibration_dataset.cpp`: sorted directory scanning and PNG/JPEG
  loading for offline calibration.
- `camera_intrinsics_repository.cpp`: versioned XML persistence without exposing
  OpenCV types in public interfaces.
- `tools/camera_intrinsic_calibration_cli.cpp`: offline verification entry point.
- `camera_intrinsic_calibration_workflow.*`: UI-independent capture review,
  nine-zone coverage checks and calibration orchestration.
- `camera_intrinsic_calibration_dialog.*`: wxWidgets five-step wizard. This UI
  lives under `src/panel`, outside the algorithm module.

Board dimensions always mean inner corners. The project board marked
`CC10-17X19-0.5` is configured as 17 columns by 19 rows. The square-size value
sets the scale of the estimated board poses; it does not change the recovered
pixel focal length or distortion coefficients.

Example:

```powershell
camera_intrinsic_calibration_cli.exe <image-directory> <output.xml> 17 19 0.5
```

An XML file is written only after the result passes basic physical plausibility
checks. A low reprojection RMS alone is not considered a valid calibration.
