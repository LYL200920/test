#include "telecentric_planar_calibration.h"
#include "calibration_archive_repository.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
using namespace camera_calibration;
const Chessboard_Configuration board{7, 5, 0.5};
const std::array<double, 4> affine{30.0, 4.0, -3.0, 25.0};
const double nan = std::numeric_limits<double>::quiet_NaN();
const double infinity = std::numeric_limits<double>::infinity();

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}
void Near(double actual, double expected, const char *message)
{
  Require(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, message);
}
std::vector<Image_Detection_Result> Views(const std::array<double, 4> &a = affine)
{
  std::vector<Image_Detection_Result> views;
  for (int i = 0; i < 10; ++i)
  {
    Image_Detection_Result view;
    view.image_id = "view-" + std::to_string(i);
    view.chessboard_found = true;
    view.image_width = 640; view.image_height = 480;
    for (int y = 0; y < board.inner_corner_rows; ++y)
      for (int x = 0; x < board.inner_corner_columns; ++x)
      {
        const double X = (x - 3) * board.square_size_mm;
        const double Y = (y - 2) * board.square_size_mm;
        view.corners.push_back({220 + 13 * i + a[0] * X + a[1] * Y,
                                160 + 7 * i + a[2] * X + a[3] * Y});
      }
    views.push_back(std::move(view));
  }
  return views;
}
bool Same_Result(const Telecentric_Planar_Result &a, const Telecentric_Planar_Result &b)
{
  if (a.width != b.width || a.height != b.height ||
      a.board.inner_corner_columns != b.board.inner_corner_columns ||
      a.board.inner_corner_rows != b.board.inner_corner_rows ||
      a.board.square_size_mm != b.board.square_size_mm ||
      a.pixels_from_plane_mm != b.pixels_from_plane_mm || a.rms_px != b.rms_px ||
      a.maximum_error_px != b.maximum_error_px ||
      a.maximum_linear_deviation_percent != b.maximum_linear_deviation_percent ||
      a.views.size() != b.views.size()) return false;
  for (std::size_t i = 0; i < a.views.size(); ++i)
    if (a.views[i].image_id != b.views[i].image_id || a.views[i].rms_px != b.views[i].rms_px ||
        a.views[i].maximum_error_px != b.views[i].maximum_error_px ||
        a.views[i].linear_deviation_percent != b.views[i].linear_deviation_percent) return false;
  return true;
}
Telecentric_Planar_Profile Fixture()
{
  Telecentric_Planar_Profile profile;
  profile.identity = {"telecentric-1", "2026-11-03T10:00:00Z"};
  profile.image = {"SN-123", "telecentric-fixed-focus", 640, 480, 8, 12, 1280, 960, 2, 2};
  profile.fixed_setup_id = "fixed-board-plane-and-working-distance-1";
  std::string error;
  Require(Calibrate_Telecentric_Plane(board, Views(), {}, &profile.result, &error),
    "Synthetic fixture calibration failed");
  return profile;
}
void Reject_Calibration(const std::vector<Image_Detection_Result> &views,
  const Telecentric_Planar_Options &options = {}, const Chessboard_Configuration &configuration = board)
{
  const auto previous = Fixture().result;
  auto output = previous;
  std::string error;
  Require(!Calibrate_Telecentric_Plane(configuration, views, options, &output, &error),
    "Invalid planar calibration accepted");
  Require(!error.empty() && Same_Result(output, previous),
    "Failed calibration changed output or omitted diagnostic");
}
void Test_Affine_And_Order()
{
  const auto good = Fixture();
  const auto &result = good.result;
  Require(result.width == 640 && result.height == 480 && result.views.size() == 10,
    "Calibration lost image metadata or views");
  Require(result.board.inner_corner_columns == 7 && result.board.inner_corner_rows == 5 &&
    result.board.square_size_mm == 0.5, "Calibration lost board metadata");
  for (std::size_t j = 0; j < affine.size(); ++j)
    Near(result.pixels_from_plane_mm[j], affine[j], "Incorrect shared affine map");
  Near(result.rms_px, 0, "Translations produced residual error");
  Near(result.maximum_error_px, 0, "Translations produced corner error");
  Near(result.maximum_linear_deviation_percent, 0, "Translations changed scale/orientation");
  for (std::size_t i = 0; i < result.views.size(); ++i)
  {
    Require(result.views[i].image_id == "view-" + std::to_string(i), "View identity lost");
    Near(result.views[i].rms_px, 0, "Unexpected per-view RMS");
    Near(result.views[i].maximum_error_px, 0, "Unexpected per-view corner error");
    Near(result.views[i].linear_deviation_percent, 0, "Unexpected per-view affine deviation");
  }
  auto views = Views();
  for (std::size_t i = 1; i < views.size(); i += 2)
    std::reverse(views[i].corners.begin(), views[i].corners.end());
  Telecentric_Planar_Result reversed;
  std::string error;
  std::size_t progress_count = 0;
  Telecentric_Planar_Options options;
  options.progress = [&](const std::string &message) { Require(!message.empty(), "Empty progress"); ++progress_count; };
  Require(Calibrate_Telecentric_Plane(board, views, options, &reversed, &error),
    "180-degree corner-order ambiguity rejected");
  Require(progress_count > 0 && reversed.views.size() == views.size(), "Progress or samples lost");
  for (std::size_t j = 0; j < affine.size(); ++j)
    Near(reversed.pixels_from_plane_mm[j], affine[j], "Reversed order changed affine map");
  // The first view defines the board-axis signs; there is no claimed absolute origin.
  std::reverse(views.front().corners.begin(), views.front().corners.end());
  Require(Calibrate_Telecentric_Plane(board, views, {}, &reversed, &error), "Reversed reference rejected");
  for (std::size_t j = 0; j < affine.size(); ++j)
    Near(reversed.pixels_from_plane_mm[j], -affine[j], "Reference corner order not respected");
}
void Test_Inconsistent_Views()
{
  auto views = Views();
  for (auto &p : views.back().corners)
  {
    p.x = 337 + 1.04 * (p.x - 337);
    p.y = 223 + 1.04 * (p.y - 223);
  }
  Telecentric_Planar_Options options;
  options.maximum_view_rms_px = 100;
  options.maximum_corner_error_px = 100;
  Reject_Calibration(views, options); // Isolate the linear-consistency gate.
  views = Views();
  const double angle = 0.08;
  for (auto &p : views.back().corners)
  {
    const double x = p.x - 337, y = p.y - 223;
    p = {337 + std::cos(angle) * x - std::sin(angle) * y,
         223 + std::sin(angle) * x + std::cos(angle) * y};
  }
  Reject_Calibration(views, options);
  views = Views(); views.back().corners[0].x += 8;
  options.maximum_linear_deviation_percent = 100;
  options.maximum_corner_error_px = 2;
  Reject_Calibration(views, options); // Corner gate even when RMS/linear gates are permissive.
  options.maximum_corner_error_px = 100;
  options.maximum_view_rms_px = 0.1;
  Reject_Calibration(views, options); // Independent RMS gate; no automatic view deletion.
}
void Test_Invalid_Inputs()
{
  const auto good = Views();
  Reject_Calibration({});
  auto views = good; views.resize(9); Reject_Calibration(views);
  views = good; views[1].image_id = views[0].image_id; Reject_Calibration(views);
  views = good; views[0].image_id.clear(); Reject_Calibration(views);
  views = good; views[0].image_id = " \t"; Reject_Calibration(views);
  views = good; views[0].image_id = std::string("view\0hidden", 11); Reject_Calibration(views);
  views = good; views[0].chessboard_found = false; Reject_Calibration(views);
  views = good; views[0].corners.pop_back(); Reject_Calibration(views);
  views = good; views[1].image_width = 641; Reject_Calibration(views);
  views = good; views[1].image_height = 481; Reject_Calibration(views);
  views = good; for (auto &v : views) v.image_width = 0; Reject_Calibration(views);
  for (double invalid : {nan, infinity, -1.0, 640.0})
  {
    views = good; views[0].corners[0].x = invalid; Reject_Calibration(views);
  }
  for (double invalid : {nan, infinity, -1.0, 480.0})
  {
    views = good; views[0].corners[0].y = invalid; Reject_Calibration(views);
  }
  auto bad_board = board; bad_board.inner_corner_columns = 1; Reject_Calibration(good, {}, bad_board);
  bad_board = board; bad_board.inner_corner_rows = 0; Reject_Calibration(good, {}, bad_board);
  for (double invalid : {0.0, -1.0, nan, infinity})
  {
    bad_board = board; bad_board.square_size_mm = invalid; Reject_Calibration(good, {}, bad_board);
    Telecentric_Planar_Options options;
    options.maximum_view_rms_px = invalid; Reject_Calibration(good, options);
    options = {}; options.maximum_corner_error_px = invalid; Reject_Calibration(good, options);
    options = {}; options.maximum_linear_deviation_percent = invalid; Reject_Calibration(good, options);
  }
  Telecentric_Planar_Options options;
  options.minimum_views = 2; Reject_Calibration(good, options);
  options.minimum_views = 11; Reject_Calibration(good, options);
  options.minimum_views = 3;
  views = good; views.resize(3);
  Telecentric_Planar_Result output;
  Require(Calibrate_Telecentric_Plane(board, views, options, &output), "Valid explicit minimum rejected");
  Require(!Calibrate_Telecentric_Plane(board, good, {}, nullptr), "Null calibration output accepted");
  Reject_Calibration(Views({30, 15, 20, 10})); // Collinear board image.
  Reject_Calibration(Views({0, 0, 0, 0}));
  Reject_Calibration(Views({30, 0, 0, 1e-9})); // Ill-conditioned, not exactly singular.
}
void Test_Result_And_Displacement()
{
  const auto good = Fixture().result;
  std::string error;
  const auto reject = [&](const Telecentric_Planar_Result &bad)
  {
    Require(!Validate_Telecentric_Planar_Result(bad, &error) && !error.empty(), "Invalid result accepted");
    Image_Point output{123, 456};
    Require(!Telecentric_Pixel_Displacement_To_Plane(bad, {2, 3}, &output, &error) &&
      output.x == 123 && output.y == 456, "Invalid result changed displacement output");
  };
  auto bad = good; bad.width = 0; reject(bad);
  bad = good; bad.height = -1; reject(bad);
  bad = good; bad.views.resize(2); reject(bad);
  bad = good; bad.views[1].image_id = bad.views[0].image_id; reject(bad);
  bad = good; bad.views[0].image_id.clear(); reject(bad);
  bad = good; bad.board.square_size_mm = 0; reject(bad);
  for (double invalid : {-1.0, nan, infinity})
  {
    bad = good; bad.rms_px = invalid; reject(bad);
    bad = good; bad.maximum_error_px = invalid; reject(bad);
    bad = good; bad.maximum_linear_deviation_percent = invalid; reject(bad);
    bad = good; bad.views[0].rms_px = invalid; reject(bad);
    bad = good; bad.views[0].maximum_error_px = invalid; reject(bad);
    bad = good; bad.views[0].linear_deviation_percent = invalid; reject(bad);
  }
  bad = good; bad.pixels_from_plane_mm = {1, 2, 2, 4}; reject(bad);
  bad = good; bad.pixels_from_plane_mm = {1, 0, 0, 1e-10}; reject(bad);
  for (std::size_t j = 0; j < 4; ++j)
    for (double invalid : {nan, infinity})
    { bad = good; bad.pixels_from_plane_mm[j] = invalid; reject(bad); }
  for (const Image_Point mm : {Image_Point{0, 0}, Image_Point{1.25, -0.75}, Image_Point{-4, 2}})
  {
    Image_Point output{123, 456};
    const Image_Point pixels{affine[0] * mm.x + affine[1] * mm.y,
                             affine[2] * mm.x + affine[3] * mm.y};
    Require(Telecentric_Pixel_Displacement_To_Plane(good, pixels, &output, &error), "Inverse failed");
    Near(output.x, mm.x, "Wrong board X displacement");
    Near(output.y, mm.y, "Wrong board Y displacement");
  }
  for (const Image_Point invalid : {Image_Point{nan, 1}, Image_Point{1, infinity},
       Image_Point{std::numeric_limits<double>::max(), std::numeric_limits<double>::max()}})
  {
    Image_Point output{123, 456};
    Require(!Telecentric_Pixel_Displacement_To_Plane(good, invalid, &output, &error) &&
      output.x == 123 && output.y == 456 && !error.empty(), "Invalid displacement changed output");
  }
  Require(!Telecentric_Pixel_Displacement_To_Plane(good, {0, 0}, nullptr), "Null displacement output accepted");
}
void Test_Profile_Matching()
{
  const auto profile = Fixture();
  std::string error;
  Require(Match_Telecentric_Planar_Profile(profile, profile.image, profile.fixed_setup_id, &error),
    "Matching fixed setup rejected");
  Require(!Match_Telecentric_Planar_Profile(profile, profile.image, "different", &error),
    "Wrong setup accepted");
  const auto reject = [&](const Camera_Image_Configuration &image)
  {
    Require(!Match_Telecentric_Planar_Profile(profile, image, profile.fixed_setup_id, &error) &&
      !error.empty(), "Changed camera context accepted");
  };
  auto image = profile.image; image.camera_serial += "other"; reject(image);
  image = profile.image; image.lens_configuration_id += "refocused"; reject(image);
  image = profile.image; ++image.roi_x; reject(image);
  image = profile.image; ++image.roi_y; reject(image);
  image = profile.image; ++image.roi_width; reject(image);
  image = profile.image; ++image.roi_height; reject(image);
  image = profile.image; image.binning_x = 1; image.roi_width = image.width; reject(image);
  image = profile.image; image.binning_y = 1; image.roi_height = image.height; reject(image);
  image = profile.image; ++image.width; image.roi_width += 2; reject(image);
  image = profile.image; ++image.height; image.roi_height += 2; reject(image);
  const auto reject_profile = [&](const Telecentric_Planar_Profile &bad)
  {
    Require(!Match_Telecentric_Planar_Profile(bad, bad.image, bad.fixed_setup_id, &error),
      "Invalid profile matched itself");
  };
  auto bad = profile; bad.identity.id.clear(); reject_profile(bad);
  bad = profile; bad.identity.created_utc.clear(); reject_profile(bad);
  bad = profile; bad.fixed_setup_id.clear(); reject_profile(bad);
  bad = profile; ++bad.result.width; reject_profile(bad);
  bad = profile; bad.result.pixels_from_plane_mm = {}; reject_profile(bad);
  bad = profile; bad.image.binning_x = 0; reject_profile(bad);
}
struct Temporary_Directory
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("telecentric_planar_" + std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
  Temporary_Directory() { Require(std::filesystem::create_directory(path), "Temporary directory collision"); }
  ~Temporary_Directory() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
};
std::string Read_Text(const std::filesystem::path &path)
{
  std::ifstream stream(path, std::ios::binary);
  Require(static_cast<bool>(stream), "Unable to read fixture XML");
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
void Write_Text(const std::filesystem::path &path, const std::string &text)
{
  std::ofstream stream(path, std::ios::binary);
  stream.write(text.data(), static_cast<std::streamsize>(text.size()));
  stream.close();
  Require(static_cast<bool>(stream), "Unable to write fixture XML");
}
void Test_Persistence()
{
  Temporary_Directory directory;
  const auto path = directory.path / std::filesystem::path(L"远心平面.xml");
  auto good = Fixture();
  good.identity.id = "profile<&\"-1";
  good.result.views[0].image_id = "棋盘<&\".png";
  // Include nonzero, non-integral statistics and a full-precision coefficient.
  good.result.pixels_from_plane_mm[1] = 4.123456789012345;
  good.result.rms_px = good.result.views[0].rms_px = 0.125;
  good.result.maximum_error_px = good.result.views[0].maximum_error_px = 0.25;
  good.result.maximum_linear_deviation_percent = good.result.views[0].linear_deviation_percent = 0.375;
  std::string error;
  Require(Save_Telecentric_Planar_Profile(path, good, &error), "Profile save failed");
  const auto original = Read_Text(path);
  Require(original.find("<TelecentricPlanarProfile ") != std::string::npos &&
    original.find("model=\"affine-fixed-plane\"") != std::string::npos &&
    original.find("distortion=\"uncorrected\"") != std::string::npos &&
    original.find("displacementConvention=\"pixel-displacement-from-board-axis-mm-row-major\"") != std::string::npos,
    "Profile model/conventions not explicit");
  Telecentric_Planar_Profile loaded;
  Require(Load_Telecentric_Planar_Profile(path, &loaded, &error), "Profile load failed");
  Require(Same_Result(good.result, loaded.result) && good.identity.id == loaded.identity.id &&
    good.identity.created_utc == loaded.identity.created_utc && good.fixed_setup_id == loaded.fixed_setup_id &&
    Image_Configurations_Match(good.image, loaded.image), "Profile round-trip lost fields or precision");
  const auto second = directory.path / "second.xml";
  Require(Save_Telecentric_Planar_Profile(second, loaded, &error) && Read_Text(second) == original,
    "Round-trip changed serialized fields");
  loaded.identity.id = "different";
  Require(!Save_Telecentric_Planar_Profile(path, loaded, &error) && Read_Text(path) == original && !error.empty(),
    "Existing profile overwritten");
  auto staging = path; staging += ".staging";
  Require(!std::filesystem::exists(staging), "Successful save left staging directory");
  const auto reserved = directory.path / "reserved.xml";
  staging = reserved; staging += ".staging";
  Require(std::filesystem::create_directory(staging), "Unable to reserve staging fixture");
  Write_Text(staging / "owner.txt", "do not remove");
  Require(!Save_Telecentric_Planar_Profile(reserved, good, &error) &&
    !std::filesystem::exists(reserved) && Read_Text(staging / "owner.txt") == "do not remove",
    "Occupied staging path was reused or removed");
  const auto invalid_path = directory.path / "invalid.xml";
  auto bad = good; bad.fixed_setup_id.clear();
  Require(!Save_Telecentric_Planar_Profile(invalid_path, bad, &error) && !std::filesystem::exists(invalid_path),
    "Invalid profile published");
  bad = good; bad.result.pixels_from_plane_mm[0] = nan;
  Require(!Save_Telecentric_Planar_Profile(invalid_path, bad, &error) && !std::filesystem::exists(invalid_path),
    "Nonfinite profile published");
  bad = good; bad.identity.id = "   ";
  Require(!Save_Telecentric_Planar_Profile(invalid_path, bad, &error) && !std::filesystem::exists(invalid_path),
    "Whitespace-only identity published");
  bad = good; bad.result.views[0].image_id = std::string("view\0hidden", 11);
  Require(!Save_Telecentric_Planar_Profile(invalid_path, bad, &error) && !std::filesystem::exists(invalid_path),
    "Embedded NUL identifier silently truncated");
  Require(!Save_Telecentric_Planar_Profile({}, good, &error), "Empty save path accepted");
  Require(!Save_Telecentric_Planar_Profile(directory.path / "missing" / "profile.xml", good, &error),
    "Save into missing parent unexpectedly succeeded");
  const auto corrupt = directory.path / "corrupt.xml";
  const auto reject = [&](const std::string &text)
  {
    Write_Text(corrupt, text);
    loaded = good;
    loaded.identity.id = "sentinel";
    Require(!Load_Telecentric_Planar_Profile(corrupt, &loaded, &error) && !error.empty(),
      "Malformed profile accepted or missing diagnostic");
    Require(loaded.identity.id == "sentinel" && loaded.identity.created_utc == good.identity.created_utc &&
      loaded.fixed_setup_id == good.fixed_setup_id && Image_Configurations_Match(loaded.image, good.image) &&
      Same_Result(loaded.result, good.result), "Failed load changed output");
  };
  const auto mutate = [&](const std::string &from, const std::string &to)
  {
    auto text = original;
    const auto position = text.find(from);
    Require(position != std::string::npos, "Mutation fixture token missing");
    text.replace(position, from.size(), to);
    reject(text);
  };
  mutate("version=\"1\"", "version=\"2\"");
  mutate("version=\"1\"", "");
  mutate("model=\"affine-fixed-plane\"", "model=\"pinhole-brown5\"");
  mutate("distortion=\"uncorrected\"", "distortion=\"corrected\"");
  mutate("lengthUnit=\"mm\"", "lengthUnit=\"m\"");
  mutate("pixelAxes=\"u-right-v-down\"", "pixelAxes=\"u-up-v-left\"");
  mutate("pixel-displacement-from-board-axis-mm-row-major", "robot-position-from-pixel");
  mutate("<TelecentricPlanarProfile ", "<TelecentricPlanarProfile unknown=\"1\" ");
  mutate("<Image ", "<Image unknown=\"1\" ");
  mutate("<PixelsFromPlaneMm ", "<PixelsFromPlaneMm extra=\"1\" ");
  mutate("</TelecentricPlanarProfile>", "<Unknown /></TelecentricPlanarProfile>");
  mutate("</Result>", "<Unknown /></Result>");
  mutate("</TelecentricPlanarProfile>", "<Image /></TelecentricPlanarProfile>");
  mutate("<Image ", "<MissingImage ");
  mutate("roiX=\"8\"", "roiX=\"8\" roiX=\"9\"");
  mutate("roiX=\"8\"", "roiX=\"8junk\"");
  mutate("roiX=\"8\"", "roiX=\"999999999999999999999\"");
  mutate("roiX=\"8\"", "");
  mutate("rmsPx=\"0.125\"", "rmsPx=\"nan\"");
  mutate("rmsPx=\"0.125\"", "rmsPx=\"1e9999\"");
  mutate("rmsPx=\"0.125\"", "rmsPx=\"0x1p2\"");
  mutate("rmsPx=\"0.125\"", "rmsPx=\"-1\"");
  mutate("image=\"view-2\"", "image=\"view-1\"");
  mutate("fixedSetupId=\"" + good.fixed_setup_id + "\"", "fixedSetupId=\"\"");
  mutate("<Result width=\"640\"", "<Result width=\"641\"");
  mutate("</Result>", "unexpected text</Result>");
  reject(original + "<Extra />");
  reject("<!DOCTYPE TelecentricPlanarProfile>" + original);
  reject(original.substr(0, original.size() / 2));
  reject(std::string(1, '\0') + original);
  reject("");
  reject("<CalibrationArchive version=\"1\" />");
  Require(!Load_Telecentric_Planar_Profile(path, nullptr, &error), "Null profile output accepted");
  loaded = good;
  Require(!Load_Telecentric_Planar_Profile(directory.path / "absent.xml", &loaded, &error) &&
    Same_Result(loaded.result, good.result) && loaded.identity.id == good.identity.id,
    "Missing profile changed output");
  const auto oversized = directory.path / "oversized.xml";
  {
    std::ofstream stream(oversized, std::ios::binary);
    stream.seekp(32 * 1024 * 1024);
    stream.put('x');
    stream.close();
    Require(static_cast<bool>(stream), "Unable to write oversized fixture");
  }
  Require(!Load_Telecentric_Planar_Profile(oversized, &loaded, &error) && !error.empty() &&
    Same_Result(loaded.result, good.result) && loaded.identity.id == good.identity.id,
    "Oversized profile accepted or changed output");
  Calibration_Archive archive;
  Require(!Load_Calibration_Archive(path, &archive, &error), "Telecentric profile accepted as pinhole/robot archive");
}
}
int main()
{
  try
  {
    Test_Affine_And_Order();
    Test_Inconsistent_Views();
    Test_Invalid_Inputs();
    Test_Result_And_Displacement();
    Test_Profile_Matching();
    Test_Persistence();
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Telecentric planar calibration and profile tests passed.\n";
  return 0;
}
