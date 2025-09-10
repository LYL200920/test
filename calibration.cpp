#include "calibration.h"



calibration::calibration (cv::Size image_size, cv::Size board_size, int image_count)
  :image_size (image_size),
  board_size (board_size),
  image_count (image_count)
{
  CalPose = (cv::Mat_<double> (image_count, 6));
  ToolPose = (cv::Mat_<double> (image_count, 6));
}

void calibration::corner_Subpix (cv::Mat mat, std::vector <cv::Point2f> corners)
{

}

bool calibration::find_chess_board_corners (cv::Mat& mat, int image_count, double scale, std::string folder)
{
  if (mat.empty ( ))
  {
    std::cerr << "[Error] Input image is empty.\n";
    return false;
  }

  cv::Mat frame_gray;
  cv::cvtColor (mat, frame_gray, cv::COLOR_BGR2GRAY);

  cv::Mat frame;
  if (scale != 1.0)
  {
    cv::resize (frame_gray, frame, cv::Size ( ), scale, scale);
  }
  else
  {
    frame = frame_gray;
  }

#ifdef DEBUG_VIEW
  cv::imshow ("frame_gray", frame);
  cv::waitKey(0);
  cv::destroyWindow ("frame_gray");
#endif 
  std::cout << "[Info] Start find corners!\n";
  std::cout << "       image size: " << image_size.height << " x " << image_size.width << "\n";
  std::cout << "       board size: " << board_size.height << " x " << board_size.width << "\n";

  bool pattern_found = cv::findChessboardCornersSB (frame_gray, board_size, corners);

  std::cout << "[Result] pattern_found = " << std::boolalpha << pattern_found << "\n";

  if (!pattern_found)
  {
    return false;
  }

  cv::cornerSubPix (frame_gray, corners, {11,11}, {-1,-1},
		   {cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,30,0.001});

  cv::Mat frame_draw = frame_gray.clone ( );
  cv::drawChessboardCorners (frame_draw, board_size, corners, pattern_found);

  if (scale != 1.0)
  {
    cv::resize (frame_draw, frame, cv::Size ( ), scale, scale);
  }
  else
  {
    frame = frame_draw;
  }

#ifdef DEBUG_VIEW
  cv::imshow ("frame_corner", frame);
  cv::waitKey (0);
  cv::destroyWindow ("frame_corner");
#endif 

  mat = frame_gray;
  fs::create_directories (folder);
  fs::path filename = fs::path (folder) / (std::to_string (image_count) + "_d.png");

  cv::imwrite (filename.string ( ), mat);

  corners_count += static_cast<int>(corners.size ( ));
  std::cout << "[Info] corners size = " << corners.size ( ) << "\n";
  std::cout << "[Info] total corners count = " << corners_count << "\n";

  corners_Seq.push_back (corners);
  return true;

}

bool calibration::start_Camera_Calibration (double width, double height, float z, std::string folder)
{
  //square_size = cv::Size (width, height);
  image_points = cv::Mat (1, corners_count, CV_32FC2, cv::Scalar::all (0));
  intrinsic_matrix = cv::Mat (3, 3, CV_32FC1, cv::Scalar::all (0));
  distortion_coeffs = cv::Mat (1, 5, CV_32FC1, cv::Scalar::all (0));

  std::cout << "start calibrating!" << std::endl;

  for (int t = 0; t < image_count; t++)
  {
    std::vector<cv::Point3f> tempPointSet;
    for (int i = 0; i < board_size.height; i++)
    {
      for (int j = 0; j < board_size.width; j++)
      {
	/* suppose the scaler plate is placed on the plane of z=0 in the world coordinate system */
	cv::Point3f tempPoint;
	tempPoint.x = i * width;
	tempPoint.y = j * height;
	tempPoint.z = z;
	tempPointSet.push_back (tempPoint);
      }
    }
    object_Points.push_back (tempPointSet);
  }

  savePoint3fToFile (object_Points, "Point3f", folder);
  savePoint2fToFile (corners_Seq, "Point2f", folder);

  std::cout << "object_points init finish!" << std::endl;

  for (int i = 0; i < image_count; i++)
  {
    point_counts.push_back (board_size.width * board_size.height);
  }

  //under 0.3 pixel
  //double RMS = 0.0;
  int flags = 0;
  flags |= cv::CALIB_FIX_K3;
  rms = cv::calibrateCamera (object_Points, corners_Seq, image_size,
			     intrinsic_matrix, distortion_coeffs,
			     rotation_vectors, translation_vectors);
  //                           flags, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));
  //cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);

  std::cout << ("rms: %.3f", rms);

  for (int i = 0; i < image_count; i++)
  {
    point_counts.push_back (board_size.width * board_size.height);
  }

  if (rms > 8)
    return false;

  return true;
}

void calibration::camera_calibration_undistort (const cv::Mat input, std::string folder, const int cnt)
{
  cv::Mat out;
  cv::undistort(input, out, intrinsic_matrix, distortion_coeffs);

  cv::imwrite(folder + "/" + std::to_string (cnt) + "_u.png", out);
}

void calibration::calibration_err (std::ofstream& fout, std::string folder)
{
  fout << "The Reprojection error is " << rms << " pixel" << std::endl;

  std::vector<cv::Point2f> image_points2; //存储重投影点

  for (int i = 0; i < image_count; i++)
  {
    std::vector<cv::Point3f> tempPointSet = object_Points[i]; //世界棋盘格坐标点

    projectPoints (tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);

    std::vector<cv::Point2f> tempImagePoint = corners_Seq[i]; //检测到的角点

    cv::Mat tempImagePointMat = cv::Mat (1, tempImagePoint.size ( ), CV_32FC2);
    cv::Mat image_points2Mat = cv::Mat (1, image_points2.size ( ), CV_32FC2);

    for (int j = 0; j < tempImagePoint.size ( ); j++)
    {
      image_points2Mat.at<cv::Vec2f> (0, j) = cv::Vec2f (image_points2[j].x, image_points2[j].y);
      tempImagePointMat.at<cv::Vec2f> (0, j) = cv::Vec2f (tempImagePoint[j].x, tempImagePoint[j].y);
    }
    err = norm (image_points2Mat, tempImagePointMat, cv::NORM_L2);
    total_err += err /= point_counts[i];

    fout << "The average error of the image " << i + 1 << " : " << err << " pixel" << std::endl;

    cv::Mat vis, binary;
    images[i].copyTo(vis);
    if (vis.channels ( ) == 1)
      cv::cvtColor (vis, vis, cv::COLOR_GRAY2BGR);
    for (size_t j = 0; j < image_points2.size ( ); j++)
    {
      // 绿色：检测到的角点
      cv::circle (vis, tempImagePoint[j], 10, cv::Scalar (0, 255, 0), -1);
      // 红色：投影点
      cv::circle (vis, image_points2[j], 1, cv::Scalar (0, 0, 255), -1);

      
    }
    
    cv::imwrite (folder + "/" + std::to_string (i + 1) + "_c.png", vis);

    //cv::resize (vis, binary, cv::Size ( ), 0.25, 0.25);
    //cv::imshow ("Reprojection Check " + std::to_string (i+1), binary);
    //cv::waitKey (0);
    //cv::destroyWindow ("Reprojection Check " + std::to_string (i + 1));

    //cv::imshow ("Reprojection Check " + std::to_string (i), vis);
    //cv::waitKey (2000);  // 每张图显示 0.5 秒

  }

  fout << "The total average error: " << total_err / image_count << " pixel" << std::endl << std::endl;

  cv::Mat rotation_matrix = cv::Mat (3, 3, CV_32FC1, cv::Scalar::all (0));

  fout << "The camera interal parameter metrix:" << std::endl;
  fout << intrinsic_matrix << std::endl << std::endl;
  fout << "The distortion factor:\n";
  fout << distortion_coeffs << std::endl << std::endl << std::endl;
  for (int i = 0; i < image_count; i++)
  {
    fout << "The rotation vector of the image " << i + 1 << " :" << std::endl;
    fout << rotation_vectors[i] << std::endl;

    /* converts the rotation vector to the corresponding rotation matrix */
    Rodrigues (rotation_vectors[i], rotation_matrix);
    fout << "The rotation matrix of the image " << i + 1 << " : " << std::endl;
    fout << rotation_matrix << std::endl;
    fout << "The translation vector of the image " << i + 1 << " : " << std::endl;
    fout << translation_vectors[i] << std::endl << std::endl;
  }

  fout << std::endl;
}

void calibration::savePoint3fToFile (const std::vector<std::vector<cv::Point3f>>& object_Points, const std::string& filename, std::string folder)
{
  std::cout << "Start save file! " << std::endl;

  std::ofstream outFile (folder + "/" + filename + ".txt");
  if (!outFile.is_open ( ))
  {
    std::cout << "Cannot open file: " << filename << std::endl;
    return;
  }

  for (const auto& pointsSet : object_Points)
  {
    for (const auto& pt : pointsSet)
    {
      outFile << pt.x << " " << pt.y << " " << pt.z << "\n";
    }
    outFile << "\n";
  }

  outFile.close ( );
  std::cout << "Success save ! " << std::endl;
}

void calibration::savePoint2fToFile (const std::vector<std::vector<cv::Point2f>>& _Points, const std::string& filename, std::string folder)
{
  std::cout << "Start save file! " << std::endl;

  std::ofstream outFile (folder + "/" + filename + ".txt");
  if (!outFile.is_open ( ))
  {
    std::cout << "Cannot open file: " << filename << std::endl;
    return;
  }

  for (const auto& pointsSet : _Points)
  {
    for (const auto& pt : pointsSet)
    {
      outFile << pt.x << " " << pt.y << "\n";
    }
    outFile << "\n";
  }

  outFile.close ( );
  std::cout << "Success save ! " << std::endl;
}

void calibration::changeConfig (cv::Size i_size, cv::Size b_size, int _count)
{
  image_size = i_size;
  board_size = b_size;
  image_count = _count;
}

cv::Mat calibration::R_T2RT (cv::Mat& R, cv::Mat& T)
{
  cv::Mat RT;

  cv::Mat_<double> R1 = (cv::Mat_<double> (4, 3) << R.at<double> (0, 0), R.at<double> (0, 1), R.at<double> (0, 2),
			 R.at<double> (1, 0), R.at<double> (1, 1), R.at<double> (1, 2),
			 R.at<double> (2, 0), R.at<double> (2, 1), R.at<double> (2, 2),
			 0.0, 0.0, 0.0);

  cv::Mat_<double> T1 = (cv::Mat_<double> (4, 1) << T.at<double> (0, 0), T.at<double> (1, 0), T.at<double> (2, 0), 1.0);

  cv::hconcat (R1, T1, RT);

  return RT;
}

void calibration::RT2R_T (cv::Mat& RT, cv::Mat& R, cv::Mat& T)
{
  cv::Rect R_rect (0, 0, 3, 3);
  cv::Rect T_rect (3, 0, 1, 3);

  R = RT (R_rect);
  T = RT (T_rect);
}

bool calibration::isRotationMatrix (const cv::Mat& R)
{
  cv::Mat tmp33 = R ({0,0,3,3});
  cv::Mat isI;

  isI = tmp33.t ( ) * tmp33;

  cv::Mat I = cv::Mat::eye (3, 3, isI.type ( ));

  return cv::norm (I, isI) < 1e-6;
}

cv::Mat calibration::eulerAngle2RotaotionMatrix (const cv::Mat& eulerAngle, const std::string& seq)
{
  CV_Assert (eulerAngle.rows == 1 && eulerAngle.cols == 3);

  eulerAngle /= 180 / CV_PI;
  cv::Matx13d m (eulerAngle);
  auto rx = m (0, 0), ry = m (0, 1), rz = m (0, 2);

  auto xs = sin (rx), xc = cos (rx);
  auto ys = sin (ry), yc = cos (ry);
  auto zs = sin (rz), zc = cos (rz);

  cv::Mat rotX = (cv::Mat_<double> (3, 3) << 1, 0, 0,
		  0, xc, -xs,
		  0, xs, xc);
  cv::Mat rotY = (cv::Mat_<double> (3, 3) << yc, 0, ys,
		  0, 1, 0,
		  -ys, 0, yc);
  cv::Mat rotZ = (cv::Mat_<double> (3, 3) << zc, -zs, 0,
		  zs, zc, 0,
		  0, 0, 1);

  cv::Mat rotMat;

  if (seq == "zyx")      rotMat = rotZ * rotY * rotX;
  else if (seq == "yzx") rotMat = rotY * rotZ * rotX;
  else if (seq == "zxy") rotMat = rotZ * rotX * rotY;
  else if (seq == "xzy") rotMat = rotX * rotZ * rotY;
  else if (seq == "yxz") rotMat = rotY * rotX * rotZ;
  else if (seq == "xyz") rotMat = rotX * rotY * rotZ;
  else
  {
    cv::error (cv::Error::StsAssert, "Error angle sequence string is wrong.", __FUNCTION__, __FILE__, __LINE__);
  }

  if (!isRotationMatrix (rotMat))
  {
    cv::error (cv::Error::StsAssert, "Error angle can not convert to rotated matrix.", __FUNCTION__, __FILE__, __LINE__);
  }

  return rotMat;
}

cv::Mat calibration::quaternion2RotationMatrix (const cv::Vec4d& q)
{
  double w = q[0], x = q[1], y = q[2], z = q[3];

  double x2 = x * x, y2 = y * y, z2 = z * z;
  double xy = x * y, xz = x * z, yz = y * z;
  double wx = w * x, wy = w * y, wz = w * z;

  cv::Matx33d res {
    1 - 2 * (y2 + z2),   2 * (xy - wz),      2 * (xz + wy),
    2 * (xy + wz),       1 - 2 * (x2 + z2),  2 * (yz - wx),
    2 * (xz - wy),       2 * (yz + wx),      1 - 2 * (x2 + y2),
  };

  return cv::Mat (res);
}

cv::Mat calibration::attitudeVectorToMatrix (cv::Mat& m, bool useQuaternion, const std::string& seq)
{
  CV_Assert (m.total ( ) == 6 || m.total ( ) == 10);

  if (m.cols == 1)
    m = m.t ( );
  cv::Mat tmp = cv::Mat::eye (4, 4, CV_64FC1);

  if (useQuaternion)
  {
    cv::Vec4d quaternionVec = m ({3,0,4,1});
    quaternion2RotationMatrix (quaternionVec).copyTo (tmp ({0,0,3,3}));
  }
  else
  {
    cv::Mat rotVec;
    if (m.total ( ) == 6)
    {
      rotVec = m.colRange (3, 6);
    }
    else
    {
      rotVec = m ({7,0,3,1});
    }

    if (seq.compare ("") == 0)
      cv::Rodrigues (rotVec, tmp ({0,0,3,3}));
    else
      eulerAngle2RotaotionMatrix (rotVec, seq).copyTo (tmp ({0,0,3,3}));
  }
  tmp ({3,0,1,3}) = m ({0,0,3,1}).t ( );

  return tmp;
}

std::vector<double> calibration::matrixToXYZABC (const cv::Mat& T)
{
  CV_Assert (T.rows == 4 && T.cols == 4);

  double x = T.at<double> (0, 3);
  double y = T.at<double> (1, 3);
  double z = T.at<double> (2, 3);

  cv::Mat R = T (cv::Range (0, 3), cv::Range (0, 3));

  double r00 = R.at<double> (0, 0);
  double r01 = R.at<double> (0, 1);
  double r02 = R.at<double> (0, 2);
  double r10 = R.at<double> (1, 0);
  double r11 = R.at<double> (1, 1);
  double r12 = R.at<double> (1, 2);
  double r20 = R.at<double> (2, 0);
  double r21 = R.at<double> (2, 1);
  double r22 = R.at<double> (2, 2);

  double a, b, c;
  b = std::asin (-r20);
  if (std::cos (b) > 1e-6)
  {
    a = std::atan2 (r21, r22);
    c = std::atan2 (r10, r00);
  }
  else
  {
    a = std::atan2 (-r12, r11);
    c = 0;
  }

  double rad2deg = 180.0 / M_PI;
  a *= rad2deg;
  b *= rad2deg;
  c *= rad2deg;

  return {x, y, z, a, b, c};
}


void calibration::handeyeCalibration (int& image_count, std::string str)
{
  for (size_t i = 0; i < object_Points.size ( ); ++i)
  {
    cv::Mat rvec, tvec;
    cv::Mat R, RT;

    bool ok = cv::solvePnP (
      object_Points[i],       
      corners_Seq[i],        
      intrinsic_matrix,
      distortion_coeffs,
      rvec,
      tvec
    );

    cv::Rodrigues (rvec, R); //matrix to vector

    if (ok)
    {
      R_target2cam.push_back (R.clone ( ));
      t_target2cam.push_back (tvec.clone ( ));

      RT = R_T2RT (R, tvec);
      vecHc.push_back (RT); //
    }
    else
    {
      std::cout << "第 %zu 张图像 solvePnP 失败" << i << std::endl;
    }
  }

  for (size_t i = 0; i < image_count; i++)
  {
    cv::Mat tmp = attitudeVectorToMatrix (CalPose.row (i), false, "zyx");
    vecHg.push_back (tmp);
    RT2R_T (tmp, tempR, tempT);

    R_gripper2base.push_back (tempR);
    t_gripper2base.push_back (tempT);
  }

  std::ofstream ofs (str);
  if (!ofs.is_open ( ))
  {
    std::cout << "Cannot open file" << std::endl;
    return;
  }
  ofs << "\xEF\xBB\xBF";

  for (size_t i = 0; i < R_gripper2base.size ( ); ++i)
  {
    ofs << "===== The data " << i << " ==== = " << std::endl;

    ofs << "[1] The rotation matrix of the end of the robotic arm relative to the base:" << std::endl << "R_gripper2base:" << std::endl;
    ofs << R_gripper2base[i] << std::endl;

    ofs << "[2] The translation vector of the end of the robotic arm relative to the base:" << std::endl << "t_gripper2base:" << std::endl;
    ofs << t_gripper2base[i] << std::endl;

    ofs << "[3] The rotation matrix of the calibration plate relative to the camera:" << std::endl << "R_target2cam:" << std::endl;
    ofs << R_target2cam[i] << std::endl;

    ofs << "[4] The Translation vector of the calibration plate relative to the camera:" << std::endl << "t_target2cam:" << std::endl;
    ofs << t_target2cam[i] << std::endl;

    ofs << std::endl;
  }

  calibrateHandEye (R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);

  Hcg = R_T2RT (R_cam2gripper, t_cam2gripper);

  ofs << "The rotation matrix of the camera relative to the end of the robotic arm:" << std::endl << "R_cam2gripper:" << std::endl;
  ofs << R_cam2gripper << std::endl;

  ofs << "The Translation vector of the camera relative to the end of the robotic arm:" << std::endl << "t_cam2gripper:" << std::endl;
  ofs << t_cam2gripper << std::endl;

  ofs << "The Hcg:" << std::endl;
  ofs << Hcg << std::endl;

  ofs << "Verification of hand-eye data in the first group and the second group:" << std::endl;
  ofs << vecHg[0] * Hcg * vecHc[0] << std::endl;
  ofs << vecHg[1] * Hcg * vecHc[1] << std::endl;

  ofs << "The posture of the calibration plate in the camera:" << std::endl;
  ofs << vecHc[1] << std::endl;

  ofs << "The pose retrieved by the hand-eye system is:" << std::endl;
  ofs << Hcg.inv ( ) * vecHg[1].inv ( ) * vecHg[0] * Hcg * vecHc[0] << std::endl;

  ofs << "----Hand-eye system test----" << std::endl;
  ofs << "The calibration plate XYZ under the robotic arm is:" << std::endl;



  std::ostringstream oss;
  oss << "Hcg 矩阵为：\n" << Hcg << std::endl;
  oss << "是否为旋转矩阵：" << isRotationMatrix (Hcg) << "\n";

  oss << "第一组和第二组手眼数据验证：" << std::endl;
  oss << vecHg[0] * Hcg * vecHc[0] << std::endl;
  oss << vecHg[1] * Hcg * vecHc[1] << std::endl << std::endl;

  oss << "标定板在相机中的位姿：" << std::endl;
  oss << vecHc[1] << std::endl;

  oss << "手眼系统反演的位姿为：" << std::endl;
  oss << Hcg.inv ( ) * vecHg[1].inv ( ) * vecHg[0] * Hcg * vecHc[0] << std::endl << std::endl;

  oss << "----手眼系统测试----" << std::endl;
  oss << "机械臂下标定板XYZ为：" << std::endl;

  for (int i = 0; i < vecHc.size ( ); ++i)
  {
    cv::Mat cheesePos {0.0,0.0,0.0,1.0};
    cv::Mat worldPos = vecHg[i] * Hcg * vecHc[i] * cheesePos;
    oss << i << ": " << worldPos.t ( ) << std::endl;
    ofs << i << ": " << worldPos.t ( ) << std::endl;
  }
  std::cout << ("%s", oss.str ( ));

  ofs.close ( );
}

void calibration::handle_coordination (std::string str, int pointCount)
{
  std::ifstream in_file (str);
  if (!in_file.is_open ( ))
  {
    std::cerr << "can not open file" << std ::endl;
  }

  std::vector<std::vector<double>> data;
  std::string line;

  while (std::getline (in_file, line) && (int)data.size ( ) < pointCount)
  {
    std::stringstream ss (line);
    std::string value;
    std::vector<double> row;

    while (std::getline (ss, value, ','))
    {
      row.push_back (std::stod (value));
    }

    if (!row.empty ( ))
    {
      data.push_back (row);
    }
  }

  in_file.close ( );

  if (data.empty ( ))
  {
    throw std::runtime_error ("文件为空或数据不合法: " + str);
  }

  int rows = static_cast<int>(data.size ( ));
  int cols = static_cast<int>(data[0].size ( ));
  CalPose.create (rows, cols);   

  for (int i = 0; i < rows; ++i)
  {
    if ((int)data[i].size ( ) != cols)
    {
      throw std::runtime_error ("文件数据列数不一致: " + str);
    }
    for (int j = 0; j < cols; ++j)
    {
      if (j <= 2)
	CalPose (i, j) = data[i][j] / 1000;
      else
	CalPose (i, j) = data[i][j];
    }
  }

  //double tmp;
  //for (int i = 0; i < rows; ++i)
  //{
  //  tmp = CalPose (i, 5);
  //  CalPose (i, 5) = CalPose (i, 3);
  //  CalPose (i, 3) = tmp;
  //}
}

cv::Mat calibration::invertH (const cv::Mat& Hcg) const
{
  CV_Assert (Hcg.rows == 4 && Hcg.cols == 4 && Hcg.type ( ) == CV_64F);

  cv::Mat R = Hcg (cv::Range (0, 3), cv::Range (0, 3));  
  cv::Mat t = Hcg (cv::Range (0, 3), cv::Range (3, 4));  

  cv::Mat Rt = R.t ( );             
  cv::Mat t_new = -Rt * t;          

  cv::Mat Hgc = cv::Mat::eye (4, 4, CV_64F);
  Rt.copyTo (Hgc (cv::Range (0, 3), cv::Range (0, 3)));
  t_new.copyTo (Hgc (cv::Range (0, 3), cv::Range (3, 4)));

  return Hgc;
}

