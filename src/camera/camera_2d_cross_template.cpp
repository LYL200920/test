#include "camera_2d_cross_template.h"

#include <pugixml.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::filesystem::path Template_Root()
{
#ifdef CAMERA_2D_RESOURCE_ROOT
  return std::filesystem::path(CAMERA_2D_RESOURCE_ROOT);
#else
  return std::filesystem::path("Resource") / "Camera2D";
#endif
}

std::filesystem::path Configuration_Path()
{
  return Template_Root() / "cross_templates.xml";
}

std::string Make_Id()
{
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
#ifdef _WIN32
  localtime_s(&local, &value);
#else
  localtime_r(&value, &local);
#endif
  std::ostringstream stream;
  stream << "cross_" << std::put_time(&local, "%Y%m%d_%H%M%S")
         << '_' << std::setw(3) << std::setfill('0')
         << milliseconds.count();
  return stream.str();
}

Camera_2D_Roi Clamp_Roi(
  Camera_2D_Roi roi,
  unsigned int image_width,
  unsigned int image_height)
{
  roi.x = std::clamp(roi.x, 0, static_cast<int>(image_width));
  roi.y = std::clamp(roi.y, 0, static_cast<int>(image_height));
  roi.width = std::clamp(
    roi.width, 0, static_cast<int>(image_width) - roi.x);
  roi.height = std::clamp(
    roi.height, 0, static_cast<int>(image_height) - roi.y);
  return roi;
}

std::vector<std::uint8_t> Grayscale(
  const Camera_2D_Display_Image &image,
  const Camera_2D_Roi &roi)
{
  std::vector<std::uint8_t> gray(
    static_cast<std::size_t>(roi.width) * roi.height);
  for (int y = 0; y < roi.height; ++y)
  {
    for (int x = 0; x < roi.width; ++x)
    {
      const auto source =
        (static_cast<std::size_t>(roi.y + y) * image.width +
         static_cast<std::size_t>(roi.x + x)) * 3;
      gray[static_cast<std::size_t>(y) * roi.width + x] =
        static_cast<std::uint8_t>(
          (static_cast<unsigned int>(image.rgb[source]) * 77U +
           static_cast<unsigned int>(image.rgb[source + 1]) * 150U +
           static_cast<unsigned int>(image.rgb[source + 2]) * 29U) >> 8);
    }
  }
  return gray;
}

int Otsu_Threshold(const std::vector<std::uint8_t> &gray)
{
  std::array<unsigned int, 256> histogram{};
  for (const auto value : gray) ++histogram[value];
  const double count = static_cast<double>(gray.size());
  double total = 0.0;
  for (int value = 0; value < 256; ++value)
    total += value * static_cast<double>(histogram[value]);

  double background_sum = 0.0;
  double best_variance = -1.0;
  unsigned int background_count = 0;
  int best = 128;
  for (int threshold = 0; threshold < 256; ++threshold)
  {
    background_count += histogram[threshold];
    if (background_count == 0) continue;
    const unsigned int foreground_count =
      static_cast<unsigned int>(gray.size()) - background_count;
    if (foreground_count == 0) break;
    background_sum += threshold * static_cast<double>(histogram[threshold]);
    const double background_mean =
      background_sum / background_count;
    const double foreground_mean =
      (total - background_sum) / foreground_count;
    const double difference = background_mean - foreground_mean;
    const double variance =
      static_cast<double>(background_count) * foreground_count *
      difference * difference / (count * count);
    if (variance > best_variance)
    {
      best_variance = variance;
      best = threshold;
    }
  }
  return best;
}

bool Infer_Dark_On_Light(
  const std::vector<std::uint8_t> &gray,
  int width,
  int height)
{
  double center_sum = 0.0;
  int center_count = 0;
  double border_sum = 0.0;
  int border_count = 0;
  const int cx0 = width * 3 / 8;
  const int cx1 = width * 5 / 8;
  const int cy0 = height * 3 / 8;
  const int cy1 = height * 5 / 8;
  const int border = std::max(1, std::min(width, height) / 10);
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const auto value = gray[static_cast<std::size_t>(y) * width + x];
      if (x >= cx0 && x < cx1 && y >= cy0 && y < cy1)
      {
        center_sum += value;
        ++center_count;
      }
      if (x < border || y < border ||
          x >= width - border || y >= height - border)
      {
        border_sum += value;
        ++border_count;
      }
    }
  }
  return center_count > 0 && border_count > 0 &&
    center_sum / center_count < border_sum / border_count;
}

struct Component
{
  std::vector<int> pixels;
  double center_x = 0.0;
  double center_y = 0.0;
};

std::optional<Component> Select_Component(
  const std::vector<std::uint8_t> &gray,
  int width,
  int height,
  int threshold,
  bool dark_on_light,
  double expected_ratio)
{
  const int count = width * height;
  std::vector<std::uint8_t> foreground(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index)
  {
    foreground[index] = dark_on_light
      ? gray[index] <= threshold
      : gray[index] >= threshold;
  }
  std::vector<std::uint8_t> visited(static_cast<std::size_t>(count));
  std::optional<Component> best;
  double best_score = -std::numeric_limits<double>::infinity();
  const int minimum_area = std::max(12, count / 1000);
  const std::array<int, 4> dx{-1, 1, 0, 0};
  const std::array<int, 4> dy{0, 0, -1, 1};
  for (int seed = 0; seed < count; ++seed)
  {
    if (!foreground[seed] || visited[seed]) continue;
    Component component;
    std::queue<int> pending;
    pending.push(seed);
    visited[seed] = 1;
    double x_sum = 0.0;
    double y_sum = 0.0;
    while (!pending.empty())
    {
      const int index = pending.front();
      pending.pop();
      component.pixels.push_back(index);
      const int x = index % width;
      const int y = index / width;
      x_sum += x;
      y_sum += y;
      for (int direction = 0; direction < 4; ++direction)
      {
        const int nx = x + dx[direction];
        const int ny = y + dy[direction];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        const int neighbor = ny * width + nx;
        if (foreground[neighbor] && !visited[neighbor])
        {
          visited[neighbor] = 1;
          pending.push(neighbor);
        }
      }
    }
    if (static_cast<int>(component.pixels.size()) < minimum_area) continue;
    component.center_x = x_sum / component.pixels.size();
    component.center_y = y_sum / component.pixels.size();
    const double ratio =
      static_cast<double>(component.pixels.size()) / count;
    const double center_distance = std::hypot(
      component.center_x - (width - 1) * 0.5,
      component.center_y - (height - 1) * 0.5) /
      std::max(1.0, std::hypot(width, height) * 0.5);
    const double ratio_error = expected_ratio > 0.0
      ? std::abs(ratio - expected_ratio) /
          std::max(0.01, expected_ratio)
      : 0.0;
    const double score =
      static_cast<double>(component.pixels.size()) -
      center_distance * count * 0.08 -
      ratio_error * count * 0.04;
    if (score > best_score)
    {
      best_score = score;
      best = std::move(component);
    }
  }
  return best;
}

double Cross_Angle(
  const Component &component,
  int width,
  int height,
  double *line_confidence)
{
  (void)height;
  double harmonic_x = 0.0;
  double harmonic_y = 0.0;
  double weight_sum = 0.0;
  for (const int index : component.pixels)
  {
    const double dx =
      static_cast<double>(index % width) - component.center_x;
    const double dy =
      static_cast<double>(index / width) - component.center_y;
    const double radius_squared = dx * dx + dy * dy;
    if (radius_squared < 1.0) continue;
    const double angle = std::atan2(dy, dx);
    // A cross is invariant after a 90-degree rotation. The fourth angular
    // harmonic therefore estimates both perpendicular arms as one direction
    // and avoids the edge-biased plateau produced by line-scanning thick arms.
    harmonic_x += radius_squared * std::cos(4.0 * angle);
    harmonic_y += radius_squared * std::sin(4.0 * angle);
    weight_sum += radius_squared;
  }
  if (weight_sum <= 0.0)
  {
    if (line_confidence) *line_confidence = 0.0;
    return 0.0;
  }
  double angle_deg =
    std::atan2(harmonic_y, harmonic_x) * 180.0 / (4.0 * kPi);
  while (angle_deg < 0.0) angle_deg += 90.0;
  while (angle_deg >= 90.0) angle_deg -= 90.0;
  if (line_confidence)
  {
    *line_confidence = std::clamp(
      std::hypot(harmonic_x, harmonic_y) / weight_sum,
      0.0,
      1.0);
  }
  return angle_deg;
}

bool Write_Pgm(
  const std::filesystem::path &path,
  const std::vector<std::uint8_t> &gray,
  int width,
  int height)
{
  std::ofstream stream(path, std::ios::binary);
  if (!stream) return false;
  stream << "P5\n" << width << ' ' << height << "\n255\n";
  stream.write(
    reinterpret_cast<const char *>(gray.data()),
    static_cast<std::streamsize>(gray.size()));
  return static_cast<bool>(stream);
}

bool Analyze(
  const Camera_2D_Display_Image &image,
  const Camera_2D_Roi &input_roi,
  bool dark_on_light,
  int threshold,
  double expected_ratio,
  Camera_2D_Cross_Detection *detection,
  std::vector<std::uint8_t> *gray_output,
  std::string *error_message)
{
  const auto roi = Clamp_Roi(input_roi, image.width, image.height);
  if (roi.width < 20 || roi.height < 20)
  {
    if (error_message) *error_message = "十字模板ROI至少需要20×20像素";
    return false;
  }
  const auto gray = Grayscale(image, roi);
  const auto component = Select_Component(
    gray, roi.width, roi.height, threshold, dark_on_light, expected_ratio);
  if (!component)
  {
    if (error_message) *error_message = "ROI中没有找到完整的十字连通区域";
    return false;
  }
  const double ratio =
    static_cast<double>(component->pixels.size()) /
    (roi.width * roi.height);
  double line_confidence = 0.0;
  const double angle = Cross_Angle(
    *component, roi.width, roi.height, &line_confidence);
  const double area_confidence = expected_ratio <= 0.0
    ? 1.0
    : std::max(
        0.0,
        1.0 - std::abs(ratio - expected_ratio) /
          std::max(0.01, expected_ratio));
  detection->found = line_confidence >= 0.25;
  detection->search_roi = roi;
  detection->center_x = roi.x + component->center_x;
  detection->center_y = roi.y + component->center_y;
  detection->angle_deg = angle;
  detection->confidence = std::clamp(
    line_confidence * 0.75 + area_confidence * 0.25, 0.0, 1.0);
  std::vector<std::uint8_t> component_mask(
    static_cast<std::size_t>(roi.width) * roi.height);
  for (const int index : component->pixels) component_mask[index] = 1;
  detection->outline.clear();
  for (const int index : component->pixels)
  {
    const int x = index % roi.width;
    const int y = index / roi.width;
    bool boundary = x == 0 || y == 0 ||
      x + 1 == roi.width || y + 1 == roi.height;
    if (!boundary)
    {
      boundary =
        !component_mask[index - 1] ||
        !component_mask[index + 1] ||
        !component_mask[index - roi.width] ||
        !component_mask[index + roi.width];
    }
    if (boundary)
      detection->outline.push_back({roi.x + x, roi.y + y});
  }
  if (detection->outline.size() > 4000)
  {
    const std::size_t step =
      (detection->outline.size() + 3999) / 4000;
    std::vector<std::array<int, 2>> reduced;
    reduced.reserve(4000);
    for (std::size_t index = 0;
         index < detection->outline.size();
         index += step)
      reduced.push_back(detection->outline[index]);
    detection->outline = std::move(reduced);
  }
  if (gray_output) *gray_output = gray;
  if (error_message) error_message->clear();
  return true;
}
}

bool Detect_Camera_2D_Cross(
  const Camera_2D_Display_Image &image,
  const Camera_2D_Cross_Template &cross_template,
  Camera_2D_Cross_Detection *detection,
  std::string *error_message)
{
  if (!detection)
  {
    if (error_message) *error_message = "十字识别输出无效";
    return false;
  }
  auto scaled_roi = cross_template.roi;
  if (cross_template.reference_width > 0 &&
      cross_template.reference_height > 0 &&
      (image.width != cross_template.reference_width ||
       image.height != cross_template.reference_height))
  {
    const double sx =
      static_cast<double>(image.width) / cross_template.reference_width;
    const double sy =
      static_cast<double>(image.height) / cross_template.reference_height;
    scaled_roi.x = static_cast<int>(std::lround(scaled_roi.x * sx));
    scaled_roi.y = static_cast<int>(std::lround(scaled_roi.y * sy));
    scaled_roi.width =
      static_cast<int>(std::lround(scaled_roi.width * sx));
    scaled_roi.height =
      static_cast<int>(std::lround(scaled_roi.height * sy));
  }
  Camera_2D_Cross_Detection result;
  result.template_id = cross_template.id;
  result.template_name = cross_template.name;
  if (!Analyze(
        image,
        scaled_roi,
        cross_template.dark_on_light,
        Otsu_Threshold(Grayscale(image, Clamp_Roi(
          scaled_roi, image.width, image.height))),
        cross_template.foreground_ratio,
        &result,
        nullptr,
        error_message))
  {
    return false;
  }
  result.template_id = cross_template.id;
  result.template_name = cross_template.name;
  *detection = std::move(result);
  return true;
}

Camera_2D_Cross_Template_Service::Camera_2D_Cross_Template_Service()
{
  Reload(nullptr);
}

bool Camera_2D_Cross_Template_Service::Reload(std::string *error_message)
{
  std::vector<Camera_2D_Cross_Template> loaded;
  pugi::xml_document document;
  const auto path = Configuration_Path();
  if (std::filesystem::exists(path))
  {
    const auto result = document.load_file(path.string().c_str());
    if (!result)
    {
      if (error_message)
        *error_message = "加载2D十字模板配置失败";
      return false;
    }
    const auto root = document.child("CrossTemplates");
    for (const auto node : root.children("Template"))
    {
      Camera_2D_Cross_Template item;
      item.id = node.attribute("id").as_string();
      item.name = node.attribute("name").as_string();
      item.roi.x = node.attribute("x").as_int();
      item.roi.y = node.attribute("y").as_int();
      item.roi.width = node.attribute("width").as_int();
      item.roi.height = node.attribute("height").as_int();
      item.reference_width = node.attribute("imageWidth").as_uint();
      item.reference_height = node.attribute("imageHeight").as_uint();
      item.dark_on_light = node.attribute("darkOnLight").as_bool(true);
      item.threshold = node.attribute("threshold").as_int(128);
      item.foreground_ratio =
        node.attribute("foregroundRatio").as_double();
      item.reference_angle_deg =
        node.attribute("referenceAngle").as_double();
      item.reference_image = node.attribute("referenceImage").as_string();
      if (!item.id.empty() && !item.name.empty()) loaded.push_back(item);
    }
  }
  std::lock_guard<std::mutex> lock(m_mutex);
  m_templates = std::move(loaded);
  if (!m_active_template_id.empty() &&
      std::none_of(
        m_templates.begin(), m_templates.end(),
        [this](const auto &item)
        {
          return item.id == m_active_template_id;
        }))
  {
    m_active_template_id.clear();
  }
  if (m_active_template_id.empty() && !m_templates.empty())
    m_active_template_id = m_templates.front().id;
  if (error_message) error_message->clear();
  return true;
}

std::vector<Camera_2D_Cross_Template>
Camera_2D_Cross_Template_Service::Templates() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_templates;
}

std::optional<Camera_2D_Cross_Template>
Camera_2D_Cross_Template_Service::Active_Template() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  const auto found = std::find_if(
    m_templates.begin(), m_templates.end(),
    [this](const auto &item)
    {
      return item.id == m_active_template_id;
    });
  if (found == m_templates.end()) return std::nullopt;
  return *found;
}

bool Camera_2D_Cross_Template_Service::Set_Active(
  const std::string &template_id,
  std::string *error_message)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  const auto found = std::find_if(
    m_templates.begin(), m_templates.end(),
    [&template_id](const auto &item)
    {
      return item.id == template_id;
    });
  if (found == m_templates.end())
  {
    if (error_message) *error_message = "选择的十字模板不存在";
    return false;
  }
  m_active_template_id = template_id;
  m_latest_detection.reset();
  if (error_message) error_message->clear();
  return true;
}

bool Camera_2D_Cross_Template_Service::Create(
  const std::string &name,
  const Camera_2D_Display_Image &image,
  const Camera_2D_Roi &input_roi,
  std::string *created_id,
  std::string *error_message)
{
  if (name.empty())
  {
    if (error_message) *error_message = "请输入十字模板名称";
    return false;
  }
  const auto roi = Clamp_Roi(input_roi, image.width, image.height);
  const auto gray = Grayscale(image, roi);
  if (gray.empty())
  {
    if (error_message) *error_message = "十字模板ROI无效";
    return false;
  }
  const int threshold = Otsu_Threshold(gray);
  const bool dark_on_light =
    Infer_Dark_On_Light(gray, roi.width, roi.height);
  Camera_2D_Cross_Detection detection;
  std::vector<std::uint8_t> reference_gray;
  if (!Analyze(
        image, roi, dark_on_light, threshold, 0.0,
        &detection, &reference_gray, error_message))
  {
    return false;
  }
  if (!detection.found || detection.confidence < 0.3)
  {
    if (error_message)
      *error_message =
        "框选区域不像完整十字，请缩小ROI并确保包含两条清晰交叉线";
    return false;
  }
  const std::string id = Make_Id();
  const auto root = Template_Root();
  std::error_code filesystem_error;
  std::filesystem::create_directories(root, filesystem_error);
  if (filesystem_error)
  {
    if (error_message) *error_message = "无法创建2D模板保存目录";
    return false;
  }
  const std::string image_name = id + ".pgm";
  if (!Write_Pgm(
        root / image_name, reference_gray, roi.width, roi.height))
  {
    if (error_message) *error_message = "保存十字模板参考图失败";
    return false;
  }

  Camera_2D_Cross_Template item;
  item.id = id;
  item.name = name;
  item.roi = roi;
  item.reference_width = image.width;
  item.reference_height = image.height;
  item.dark_on_light = dark_on_light;
  item.threshold = threshold;
  item.foreground_ratio = 0.0;
  {
    const auto component = Select_Component(
      reference_gray, roi.width, roi.height,
      threshold, dark_on_light, 0.0);
    if (component)
      item.foreground_ratio =
        static_cast<double>(component->pixels.size()) /
        (roi.width * roi.height);
  }
  item.reference_angle_deg = detection.angle_deg;
  item.reference_image = image_name;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto duplicate = std::find_if(
      m_templates.begin(), m_templates.end(),
      [&name](const auto &existing)
      {
        return existing.name == name;
      });
    if (duplicate != m_templates.end())
    {
      std::filesystem::remove(root / image_name, filesystem_error);
      if (error_message) *error_message = "十字模板名称已经存在";
      return false;
    }
    m_templates.push_back(item);
    m_active_template_id = item.id;
  }
  if (!Save(error_message))
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_templates.erase(
      std::remove_if(
        m_templates.begin(), m_templates.end(),
        [&id](const auto &value) { return value.id == id; }),
      m_templates.end());
    std::filesystem::remove(root / image_name, filesystem_error);
    return false;
  }
  if (created_id) *created_id = id;
  if (error_message) error_message->clear();
  return true;
}

bool Camera_2D_Cross_Template_Service::Remove(
  const std::string &template_id,
  std::string *error_message)
{
  std::string reference_image;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto found = std::find_if(
      m_templates.begin(), m_templates.end(),
      [&template_id](const auto &item)
      {
        return item.id == template_id;
      });
    if (found == m_templates.end())
    {
      if (error_message) *error_message = "要删除的十字模板不存在";
      return false;
    }
    reference_image = found->reference_image;
    m_templates.erase(found);
    if (m_active_template_id == template_id)
      m_active_template_id =
        m_templates.empty() ? std::string() : m_templates.front().id;
  }
  if (!Save(error_message)) return false;
  if (!reference_image.empty())
  {
    std::error_code ignored;
    std::filesystem::remove(
      Template_Root() / reference_image, ignored);
  }
  return true;
}

std::optional<Camera_2D_Cross_Detection>
Camera_2D_Cross_Template_Service::Detect(
  const Camera_2D_Display_Image &image) const
{
  const auto active = Active_Template();
  if (!active)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_latest_detection.reset();
    return std::nullopt;
  }
  Camera_2D_Cross_Detection result;
  if (!Detect_Camera_2D_Cross(image, *active, &result, nullptr))
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_latest_detection.reset();
    return std::nullopt;
  }
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_latest_detection = result;
  }
  return result;
}

std::optional<Camera_2D_Cross_Detection>
Camera_2D_Cross_Template_Service::Latest_Detection() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_latest_detection;
}

bool Camera_2D_Cross_Template_Service::Save(
  std::string *error_message) const
{
  std::vector<Camera_2D_Cross_Template> snapshot;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    snapshot = m_templates;
  }
  std::error_code filesystem_error;
  std::filesystem::create_directories(
    Template_Root(), filesystem_error);
  if (filesystem_error)
  {
    if (error_message) *error_message = "无法创建2D模板配置目录";
    return false;
  }
  pugi::xml_document document;
  auto root = document.append_child("CrossTemplates");
  for (const auto &item : snapshot)
  {
    auto node = root.append_child("Template");
    node.append_attribute("id") = item.id.c_str();
    node.append_attribute("name") = item.name.c_str();
    node.append_attribute("x") = item.roi.x;
    node.append_attribute("y") = item.roi.y;
    node.append_attribute("width") = item.roi.width;
    node.append_attribute("height") = item.roi.height;
    node.append_attribute("imageWidth") = item.reference_width;
    node.append_attribute("imageHeight") = item.reference_height;
    node.append_attribute("darkOnLight") = item.dark_on_light;
    node.append_attribute("threshold") = item.threshold;
    node.append_attribute("foregroundRatio") = item.foreground_ratio;
    node.append_attribute("referenceAngle") = item.reference_angle_deg;
    node.append_attribute("referenceImage") =
      item.reference_image.c_str();
  }
  if (!document.save_file(Configuration_Path().string().c_str(), "  "))
  {
    if (error_message) *error_message = "保存2D十字模板配置失败";
    return false;
  }
  if (error_message) error_message->clear();
  return true;
}
