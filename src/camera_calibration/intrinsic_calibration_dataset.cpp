#include "intrinsic_calibration_dataset.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>

namespace camera_calibration
{
namespace
{

void Set_Error(std::string *error_message, const std::string &message)
{
  if (error_message)
  {
    *error_message = message;
  }
}

bool Is_Supported_Image(const std::filesystem::path &path)
{
  std::string extension = path.extension().string();
  std::transform(
    extension.begin(), extension.end(), extension.begin(),
    [](unsigned char character)
    {
      return static_cast<char>(std::tolower(character));
    });
  return extension == ".png" || extension == ".jpg" ||
         extension == ".jpeg" || extension == ".bmp" ||
         extension == ".tif" || extension == ".tiff";
}

bool Supported_Image_Paths(
  const std::filesystem::path &directory,
  std::vector<std::filesystem::path> *paths,
  std::string *error_message)
{
  std::error_code filesystem_error;
  if (!std::filesystem::is_directory(directory, filesystem_error) ||
      filesystem_error)
  {
    Set_Error(error_message, "Calibration image directory does not exist");
    return false;
  }
  for (const std::filesystem::directory_entry &entry :
       std::filesystem::directory_iterator(directory))
  {
    if (entry.is_regular_file() && Is_Supported_Image(entry.path()))
    {
      paths->push_back(entry.path());
    }
  }
  std::sort(paths->begin(), paths->end());
  if (paths->empty())
  {
    Set_Error(error_message, "Calibration image directory contains no supported images");
    return false;
  }
  return true;
}

} // namespace

bool Detect_Calibration_Image_File(
  const std::filesystem::path &image_path,
  const Chessboard_Configuration &chessboard,
  Image_Detection_Result *detection,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!detection)
  {
    Set_Error(error_message, "Calibration image detection output is null");
    return false;
  }
  std::ifstream input(image_path, std::ios::binary);
    const auto encoded_begin = std::istreambuf_iterator<char>(input);
    const auto encoded_end = std::istreambuf_iterator<char>();
    const std::vector<std::uint8_t> encoded_bytes(encoded_begin, encoded_end);
  const cv::Mat image = cv::imdecode(encoded_bytes, cv::IMREAD_COLOR);
  if (image.empty())
  {
    Set_Error(error_message, "Unable to read calibration image: " +
                             image_path.string());
    return false;
  }
  Intrinsic_Calibration_Session session(chessboard);
  const Image_View view{
    image.data,
    image.cols,
    image.rows,
    image.step,
    Pixel_Format::BGR8};
  return session.Add_Image(
    view, image_path.filename().string(), detection, error_message);
}

bool Detect_Calibration_Image_Directory(
  const std::filesystem::path &image_directory,
  const Chessboard_Configuration &chessboard,
  std::vector<Image_Detection_Result> *detections,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!detections)
  {
    Set_Error(error_message, "Calibration detections output is null");
    return false;
  }
  detections->clear();
  std::vector<std::filesystem::path> image_paths;
  if (!Supported_Image_Paths(
        image_directory, &image_paths, error_message))
  {
    return false;
  }
  detections->reserve(image_paths.size());
  for (const std::filesystem::path &image_path : image_paths)
  {
    Image_Detection_Result detection;
    if (!Detect_Calibration_Image_File(
          image_path, chessboard, &detection, error_message))
    {
      return false;
    }
    detections->push_back(std::move(detection));
  }
  return true;
}

bool Save_Calibration_Image(
  const std::filesystem::path &image_path,
  const Image_View &image,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!image.data || image.width <= 0 || image.height <= 0)
  {
    Set_Error(error_message, "Calibration image is empty");
    return false;
  }
  const bool gray = image.pixel_format == Pixel_Format::Gray8;
  const std::size_t bytes_per_pixel = gray ? 1U : 3U;
  if (image.row_stride_bytes <
      static_cast<std::size_t>(image.width) * bytes_per_pixel)
  {
    Set_Error(error_message, "Calibration image row stride is too small");
    return false;
  }
  std::error_code filesystem_error;
  if (!image_path.parent_path().empty())
  {
    std::filesystem::create_directories(
      image_path.parent_path(), filesystem_error);
    if (filesystem_error)
    {
      Set_Error(error_message, "Unable to create calibration image directory");
      return false;
    }
  }
  try
  {
    const int type = gray ? CV_8UC1 : CV_8UC3;
    const cv::Mat source(
      image.height,
      image.width,
      type,
      const_cast<std::uint8_t *>(image.data),
      image.row_stride_bytes);
    cv::Mat encoded = source;
    if (image.pixel_format == Pixel_Format::RGB8)
    {
      cv::cvtColor(source, encoded, cv::COLOR_RGB2BGR);
    }
    std::string extension = image_path.extension().string();
    std::transform(
      extension.begin(), extension.end(), extension.begin(),
      [](unsigned char character)
      {
        return static_cast<char>(std::tolower(character));
      });
    if (extension.empty())
    {
      extension = ".png";
    }
    std::vector<std::uint8_t> output_bytes;
    if (!cv::imencode(extension, encoded, output_bytes))
    {
      Set_Error(error_message, "Unable to encode calibration image");
      return false;
    }
    std::ofstream output(image_path, std::ios::binary | std::ios::trunc);
    output.write(
      reinterpret_cast<const char *>(output_bytes.data()),
      static_cast<std::streamsize>(output_bytes.size()));
    if (!output)
    {
      Set_Error(error_message, "Unable to save calibration image");
      return false;
    }
    return true;
  }
  catch (const cv::Exception &exception)
  {
    Set_Error(error_message, std::string("OpenCV image save failed: ") +
                             exception.what());
    return false;
  }
}

bool Calibrate_Image_Directory(
  const Dataset_Calibration_Request &request,
  Dataset_Calibration_Report *report,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!report)
  {
    Set_Error(error_message, "Dataset calibration report output is null");
    return false;
  }
  *report = {};

  if (!Detect_Calibration_Image_Directory(
        request.image_directory,
        request.chessboard,
        &report->detections,
        error_message))
  {
    return false;
  }

  Intrinsic_Calibration_Session session(request.chessboard);
  for (const Image_Detection_Result &detection : report->detections)
  {
    if (!detection.chessboard_found)
    {
      continue;
    }
    if (!session.Add_Observation(
          detection.image_width,
          detection.image_height,
          detection.corners,
          detection.image_id,
          error_message))
    {
      return false;
    }
  }

  return session.Calibrate(
    request.options, &report->intrinsics, error_message);
}

} // namespace camera_calibration
