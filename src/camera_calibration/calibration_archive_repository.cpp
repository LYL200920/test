#include "calibration_archive_repository.h"
#include "pugixml.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace camera_calibration
{
namespace
{
// One codec describes both directions, including required fields and allowed
// children, so the writer and strict reader cannot drift into different schemas.
class Codec
{
public:
  Codec(pugi::xml_node node, bool reading) : node_(node), reading_(reading) {}

  void Field(const char *name, std::string &value)
  {
    attributes_.insert(name);
    if (reading_)
    {
      if (!node_.attribute(name)) throw std::runtime_error(std::string("Missing attribute: ") + name);
      value = node_.attribute(name).value();
    }
    else node_.append_attribute(name).set_value(value.c_str());
  }

  template <typename T> void Field(const char *name, T &value)
  {
    std::string text;
    if (!reading_)
    {
      if constexpr (std::is_same_v<T, bool>) text = value ? "true" : "false";
      else
      {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
        text = stream.str();
      }
    }
    Field(name, text);
    if (!reading_) return;
    if constexpr (std::is_same_v<T, bool>)
    {
      if (text != "true" && text != "false") throw std::runtime_error("Invalid boolean");
      value = text == "true";
    }
    else if constexpr (std::is_integral_v<T>)
    {
      const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
      if (text.empty() || result.ec != std::errc{} || result.ptr != text.data() + text.size())
        throw std::runtime_error(std::string("Invalid integer: ") + name);
    }
    else
    {
      if (text.empty() || text.find_first_not_of("0123456789+-.eE") != std::string::npos)
        throw std::runtime_error(std::string("Invalid decimal: ") + name);
      std::istringstream stream(text);
      stream.imbue(std::locale::classic());
      stream >> std::noskipws >> value;
      if (!stream || !stream.eof() || !std::isfinite(value))
        throw std::runtime_error(std::string("Invalid finite number: ") + name);
    }
  }

  void Literal(const char *name, const char *expected)
  {
    std::string value = expected;
    Field(name, value);
    if (value != expected) throw std::runtime_error(std::string("Unsupported convention: ") + name);
  }

  template <typename T, typename Function>
  void Object(const char *name, T &value, Function function)
  {
    children_.insert(name);
    auto child = reading_ ? node_.child(name) : node_.append_child(name);
    if (!child || (reading_ && child.next_sibling(name)))
      throw std::runtime_error(std::string("Missing or duplicate element: ") + name);
    Codec codec(child, reading_);
    function(codec, value);
    codec.Finish();
  }

  template <typename T, typename Function>
  void Optional(const char *name, std::optional<T> &value, Function function)
  {
    children_.insert(name);
    if (reading_)
    {
      if (!node_.child(name)) { value.reset(); return; }
      value.emplace();
    }
    if (value) Object(name, *value, function);
  }

  template <typename T, typename Function>
  void List(const char *name, std::vector<T> &values, Function function)
  {
    children_.insert(name);
    if (reading_)
    {
      values.clear();
      for (auto child : node_.children(name))
      {
        if (values.size() >= 100000) throw std::runtime_error("Too many archive entries");
        T value{};
        Codec codec(child, true);
        function(codec, value);
        codec.Finish();
        values.push_back(std::move(value));
      }
    }
    else
      for (auto &value : values)
      {
        Codec codec(node_.append_child(name), false);
        function(codec, value);
        codec.Finish();
      }
  }

  void Finish()
  {
    if (!reading_) return;
    std::set<std::string> seen;
    for (auto attribute : node_.attributes())
      if (!attributes_.count(attribute.name()) || !seen.insert(attribute.name()).second)
        throw std::runtime_error("Unknown or duplicate archive attribute");
    for (auto child : node_.children())
      if (child.type() != pugi::node_element || !children_.count(child.name()))
        throw std::runtime_error("Unknown archive element or unexpected text");
  }

private:
  pugi::xml_node node_;
  bool reading_;
  std::set<std::string> attributes_, children_;
};

void Identity(Codec &c, Calibration_Identity &v)
{
  c.Field("id", v.id); c.Field("createdUtc", v.created_utc);
}
void Reference(Codec &c, Robot_Reference &v)
{
  c.Field("robotModel", v.robot_model_id); c.Field("referenceFrame", v.reference_frame_id);
}
void Image(Codec &c, Camera_Image_Configuration &v)
{
  c.Field("serial", v.camera_serial); c.Field("lensConfiguration", v.lens_configuration_id);
  c.Field("width", v.width); c.Field("height", v.height);
  c.Field("roiX", v.roi_x); c.Field("roiY", v.roi_y);
  c.Field("roiWidth", v.roi_width); c.Field("roiHeight", v.roi_height);
  c.Field("binningX", v.binning_x); c.Field("binningY", v.binning_y);
}
void Board(Codec &c, Chessboard_Configuration &v)
{
  c.Field("columns", v.inner_corner_columns); c.Field("rows", v.inner_corner_rows);
  c.Field("squareMm", v.square_size_mm);
}
template <typename T, std::size_t N> void Array(Codec &c, std::array<T, N> &v)
{
  for (std::size_t i = 0; i < N; ++i) c.Field(("v" + std::to_string(i)).c_str(), v[i]);
}
void Matrix(Codec &c, robot_model::Matrix4 &v)
{
  for (std::size_t row = 0; row < 4; ++row)
    for (std::size_t col = 0; col < 4; ++col)
      c.Field(("m" + std::to_string(row) + std::to_string(col)).c_str(), v[row][col]);
}
void View_Error(Codec &c, View_Calibration_Error &v)
{
  c.Field("image", v.image_id); c.Field("rmsPx", v.rms_px);
}
void Intrinsic(Codec &c, Intrinsic_Profile &v)
{
  c.Literal("model", "pinhole-brown5");
  c.Object("Identity", v.identity, Identity);
  c.Object("Image", v.image, Image);
  c.Object("Board", v.board, Board);
  auto &i = v.intrinsics;
  c.Field("cameraSerial", i.camera_serial_number); c.Field("cameraModel", i.camera_model);
  c.Field("pixelFormat", i.pixel_format);
  c.Field("imageWidth", i.image_width); c.Field("imageHeight", i.image_height);
  c.Field("rmsPx", i.overall_rms_px);
  c.Object("K", i.camera_matrix, Array<double, 9>);
  c.Object("D", i.distortion, Array<double, 5>);
  c.Object("StandardDeviations", i.intrinsic_standard_deviations, Array<double, 9>);
  c.List("AcceptedView", i.accepted_views, View_Error);
  c.List("RejectedView", i.rejected_views, View_Error);
}
void Hand_Eye(Codec &c, Hand_Eye_Profile &v)
{
  c.Object("Identity", v.identity, Identity);
  c.Object("Robot", v.robot, Reference);
  c.Object("Image", v.image, Image);
  c.Field("intrinsicId", v.intrinsic_id); c.Field("method", v.method);
  c.Field("trainingSamples", v.training_samples); c.Field("validationSamples", v.validation_samples);
  c.Field("translationRmsMm", v.translation_rms_mm);
  c.Field("rotationRmsDeg", v.rotation_rms_deg); c.Field("reprojectionRmsPx", v.reprojection_rms_px);
  c.Object("FlangeFromCamera", v.flange_from_camera, Matrix);
}
void Plane(Codec &c, Work_Plane_Profile &v)
{
  c.Object("Identity", v.identity, Identity);
  c.Object("Robot", v.robot, Reference);
  std::string source = v.source == Plane_Source::Calibration_Board ? "board" :
    v.source == Plane_Source::Fixed_3D_Camera ? "fixed-3d" : "manual";
  c.Field("source", source);
  if (source == "board") v.source = Plane_Source::Calibration_Board;
  else if (source == "fixed-3d") v.source = Plane_Source::Fixed_3D_Camera;
  else if (source == "manual") v.source = Plane_Source::Manual;
  else throw std::runtime_error("Unknown plane source");
  c.Field("sourceCalibrationId", v.source_calibration_id);
  c.Object("ReferenceFromPlane", v.reference_from_plane, Matrix);
  c.Field("minXmm", v.min_x_mm); c.Field("maxXmm", v.max_x_mm);
  c.Field("minYmm", v.min_y_mm); c.Field("maxYmm", v.max_y_mm);
  c.Field("fitRmsMm", v.fit_rms_mm);
}
void Corner(Codec &c, Image_Point &v)
{
  c.Field("u", v.x); c.Field("v", v.y);
}
void Sample(Codec &c, Hand_Eye_Sample &v)
{
  c.Field("sessionId", v.session_id); c.Field("sampleId", v.sample_id);
  c.Field("capturedUtc", v.captured_utc); c.Field("intrinsicId", v.intrinsic_id);
  c.Field("boardFrameId", v.board_frame_id);
  c.Object("Robot", v.robot, Reference); c.Object("Image", v.image, Image);
  c.Field("pointId", v.point_id); c.Field("motionSequence", v.motion_sequence);
  c.Field("stateRequestSequence", v.state_request_sequence); c.Field("stateRevision", v.state_revision);
  c.Field("stateReceivedNs", v.state_received_ns); c.Field("triggerRequestedNs", v.trigger_requested_ns);
  c.Field("frameReceivedNs", v.frame_received_ns);
  c.Field("reportedBase", v.reported_base); c.Field("reportedTool", v.reported_tool);
  c.Object("ReferenceFromFlange", v.reference_from_flange, Matrix);
  c.Object("ActualJointsDeg", v.actual_joints_deg, Array<double, 6>);
  c.Field("poseVerified", v.pose_verified); c.Field("cameraFrameId", v.camera_frame_id);
  c.Optional("CameraTimestamp", v.camera_timestamp_ticks,
    [](Codec &child, std::uint64_t &ticks) { child.Field("ticks", ticks); });
  c.Field("cameraClockId", v.camera_clock_id); c.Field("cameraTickPeriodNs", v.camera_tick_period_ns);
  c.Field("imagePath", v.image_path);
  std::string status = v.status == Sample_Status::Accepted ? "accepted" :
    v.status == Sample_Status::Rejected ? "rejected" : "pending";
  c.Field("status", status);
  if (status == "accepted") v.status = Sample_Status::Accepted;
  else if (status == "rejected") v.status = Sample_Status::Rejected;
  else if (status == "pending") v.status = Sample_Status::Pending;
  else throw std::runtime_error("Unknown sample status");
  c.Field("rejectionReason", v.rejection_reason);
  c.List("Corner", v.corners, Corner);
}
void Archive(Codec &c, Calibration_Archive &v)
{
  c.Literal("version", "1");
  c.Literal("lengthUnit", "mm");
  c.Literal("angleUnit", "degree");
  c.Literal("transformConvention", "A_from_B-column-vector");
  c.Literal("cameraAxes", "x-right-y-down-z-forward");
  c.Literal("robotPose", "reference-from-flange");
  c.Optional("Intrinsic", v.intrinsic, Intrinsic);
  c.Optional("HandEye", v.hand_eye, Hand_Eye);
  c.Optional("WorkPlane", v.work_plane, Plane);
  c.List("Sample", v.samples, Sample);
}
bool Fail(std::string *error, const std::string &message)
{
  if (error) *error = message;
  return false;
}
}

bool Save_Calibration_Archive(const std::filesystem::path &path,
  const Calibration_Archive &archive, std::string *error)
{
  if (!Validate_Calibration_Archive(archive, error)) return false;
  try
  {
    if (path.empty() || std::filesystem::exists(path))
      return Fail(error, "Choose a new calibration archive path; overwrite is not allowed");
    // Reserve a sibling staging directory exclusively. Publishing a hard link
    // is atomic and fails if the destination exists (same local filesystem).
    auto staging = path;
    staging += ".staging";
    if (!std::filesystem::create_directory(staging))
      return Fail(error, "Calibration staging path is already in use");
    struct Cleanup
    {
      std::filesystem::path path;
      ~Cleanup() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
    } cleanup{staging};
    auto value = archive;
    pugi::xml_document document;
    Codec codec(document.append_child("CalibrationArchive"), false);
    Archive(codec, value);
    const auto temporary = staging / "archive.xml";
    if (!document.save_file(temporary.wstring().c_str(), "  "))
      return Fail(error, "Unable to write calibration archive");
    Calibration_Archive verified;
    if (!Load_Calibration_Archive(temporary, &verified, error)) return false;
    std::filesystem::create_hard_link(temporary, path);
    return true;
  }
  catch (const std::exception &exception) { return Fail(error, exception.what()); }
}

bool Load_Calibration_Archive(const std::filesystem::path &path,
  Calibration_Archive *archive, std::string *error)
{
  if (error) error->clear();
  if (!archive) return Fail(error, "Calibration archive output is null");
  try
  {
    constexpr std::uintmax_t maximum_bytes = 32 * 1024 * 1024;
    const auto size = std::filesystem::file_size(path);
    if (size == 0 || size > maximum_bytes) return Fail(error, "Invalid calibration archive size");
    std::ifstream input(path, std::ios::binary);
    std::string bytes(static_cast<std::size_t>(size), '\0');
    if (!input.read(bytes.data(), static_cast<std::streamsize>(size)) ||
        input.peek() != std::char_traits<char>::eof() || bytes.find('\0') != std::string::npos)
      return Fail(error, "Unable to read complete UTF-8 calibration archive");
    pugi::xml_document document;
    const auto parsed = document.load_buffer(bytes.data(), bytes.size(),
      pugi::parse_default | pugi::parse_doctype, pugi::encoding_utf8);
    if (!parsed) return Fail(error, "Unable to parse calibration archive XML");
    auto root = document.child("CalibrationArchive");
    if (!root || root != document.first_child() || root.next_sibling())
      return Fail(error, "Expected a single CalibrationArchive root");
    Calibration_Archive value;
    Codec codec(root, true);
    Archive(codec, value);
    codec.Finish();
    if (!Validate_Calibration_Archive(value, error)) return false;
    *archive = std::move(value);
    return true;
  }
  catch (const std::exception &exception) { return Fail(error, exception.what()); }
}
} // namespace camera_calibration
