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
#include <numeric>
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

bool Read_Pgm(
  const std::filesystem::path &path,
  std::vector<std::uint8_t> *gray,
  int *width,
  int *height)
{
  if (!gray || !width || !height) return false;
  std::ifstream stream(path, std::ios::binary);
  std::string magic;
  int maximum = 0;
  if (!(stream >> magic) || magic != "P5" ||
      !(stream >> *width >> *height >> maximum) ||
      *width <= 0 || *height <= 0 || maximum != 255)
    return false;
  stream.get();
  gray->resize(static_cast<std::size_t>(*width) * *height);
  stream.read(
    reinterpret_cast<char *>(gray->data()),
    static_cast<std::streamsize>(gray->size()));
  return static_cast<bool>(stream);
}

struct Gray_Image
{
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> pixels;
};

Gray_Image Downsample(const Gray_Image &source)
{
  Gray_Image result;
  result.width = std::max(1, source.width / 2);
  result.height = std::max(1, source.height / 2);
  result.pixels.resize(
    static_cast<std::size_t>(result.width) * result.height);
  for (int y = 0; y < result.height; ++y)
  {
    for (int x = 0; x < result.width; ++x)
    {
      unsigned int sum = 0;
      int count = 0;
      for (int dy = 0; dy < 2; ++dy)
      {
        for (int dx = 0; dx < 2; ++dx)
        {
          const int sx = x * 2 + dx;
          const int sy = y * 2 + dy;
          if (sx < source.width && sy < source.height)
          {
            sum += source.pixels[
              static_cast<std::size_t>(sy) * source.width + sx];
            ++count;
          }
        }
      }
      result.pixels[
        static_cast<std::size_t>(y) * result.width + x] =
        static_cast<std::uint8_t>(sum / std::max(1, count));
    }
  }
  return result;
}

Gray_Image Resize_Gray(
  const std::vector<std::uint8_t> &pixels,
  int source_width,
  int source_height,
  int target_width,
  int target_height)
{
  Gray_Image result;
  result.width = target_width;
  result.height = target_height;
  result.pixels.resize(
    static_cast<std::size_t>(target_width) * target_height);
  for (int y = 0; y < target_height; ++y)
  {
    const int sy = std::min(
      source_height - 1,
      static_cast<int>(
        static_cast<long long>(y) * source_height / target_height));
    for (int x = 0; x < target_width; ++x)
    {
      const int sx = std::min(
        source_width - 1,
        static_cast<int>(
          static_cast<long long>(x) * source_width / target_width));
      result.pixels[
        static_cast<std::size_t>(y) * target_width + x] =
        pixels[static_cast<std::size_t>(sy) * source_width + sx];
    }
  }
  return result;
}

std::vector<int> Matching_Samples(const Gray_Image &reference)
{
  const double mean = std::accumulate(
    reference.pixels.begin(), reference.pixels.end(), 0.0) /
    std::max<std::size_t>(1, reference.pixels.size());
  std::vector<int> ranked(reference.pixels.size());
  for (std::size_t index = 0; index < ranked.size(); ++index)
    ranked[index] = static_cast<int>(index);
  const std::size_t feature_count =
    std::min<std::size_t>(80, ranked.size());
  std::partial_sort(
    ranked.begin(),
    ranked.begin() + feature_count,
    ranked.end(),
    [&reference, mean](int left, int right)
    {
      return std::abs(reference.pixels[left] - mean) >
             std::abs(reference.pixels[right] - mean);
    });
  ranked.resize(feature_count);
  const int grid = 8;
  for (int gy = 0; gy < grid; ++gy)
  {
    for (int gx = 0; gx < grid; ++gx)
    {
      const int x = std::min(
        reference.width - 1,
        (2 * gx + 1) * reference.width / (2 * grid));
      const int y = std::min(
        reference.height - 1,
        (2 * gy + 1) * reference.height / (2 * grid));
      ranked.push_back(y * reference.width + x);
    }
  }
  std::sort(ranked.begin(), ranked.end());
  ranked.erase(std::unique(ranked.begin(), ranked.end()), ranked.end());
  return ranked;
}

double Sampled_Ncc(
  const Gray_Image &image,
  const Gray_Image &reference,
  const std::vector<int> &samples,
  int x,
  int y)
{
  if (samples.empty()) return -1.0;
  double reference_mean = 0.0;
  double image_mean = 0.0;
  for (const int index : samples)
  {
    const int tx = index % reference.width;
    const int ty = index / reference.width;
    reference_mean += reference.pixels[index];
    image_mean += image.pixels[
      static_cast<std::size_t>(y + ty) * image.width + x + tx];
  }
  reference_mean /= samples.size();
  image_mean /= samples.size();
  double numerator = 0.0;
  double reference_energy = 0.0;
  double image_energy = 0.0;
  for (const int index : samples)
  {
    const int tx = index % reference.width;
    const int ty = index / reference.width;
    const double tv = reference.pixels[index] - reference_mean;
    const double iv = image.pixels[
      static_cast<std::size_t>(y + ty) * image.width + x + tx] -
      image_mean;
    numerator += tv * iv;
    reference_energy += tv * tv;
    image_energy += iv * iv;
  }
  const double denominator = std::sqrt(reference_energy * image_energy);
  return denominator > 1e-9 ? numerator / denominator : -1.0;
}

struct Match_Candidate
{
  int x = 0;
  int y = 0;
  double score = -1.0;
};

void Keep_Candidate(
  std::vector<Match_Candidate> *best,
  Match_Candidate candidate,
  std::size_t limit)
{
  if (best->size() < limit)
  {
    best->push_back(candidate);
    return;
  }
  auto worst = std::min_element(
    best->begin(), best->end(),
    [](const auto &left, const auto &right)
    {
      return left.score < right.score;
    });
  if (candidate.score > worst->score) *worst = candidate;
}

double Dense_Ncc(
  const Gray_Image &image,
  const Gray_Image &reference,
  int x,
  int y)
{
  const std::size_t count = reference.pixels.size();
  double reference_mean = std::accumulate(
    reference.pixels.begin(), reference.pixels.end(), 0.0) / count;
  double image_mean = 0.0;
  for (int ty = 0; ty < reference.height; ++ty)
    for (int tx = 0; tx < reference.width; ++tx)
      image_mean += image.pixels[
        static_cast<std::size_t>(y + ty) * image.width + x + tx];
  image_mean /= count;
  double numerator = 0.0;
  double reference_energy = 0.0;
  double image_energy = 0.0;
  for (int ty = 0; ty < reference.height; ++ty)
  {
    for (int tx = 0; tx < reference.width; ++tx)
    {
      const double tv =
        reference.pixels[
          static_cast<std::size_t>(ty) * reference.width + tx] -
        reference_mean;
      const double iv =
        image.pixels[
          static_cast<std::size_t>(y + ty) * image.width + x + tx] -
        image_mean;
      numerator += tv * iv;
      reference_energy += tv * tv;
      image_energy += iv * iv;
    }
  }
  const double denominator = std::sqrt(reference_energy * image_energy);
  return denominator > 1e-9 ? numerator / denominator : -1.0;
}

std::optional<Match_Candidate> Match_Template(
  const Gray_Image &image,
  const Gray_Image &reference)
{
  if (reference.width < 4 || reference.height < 4 ||
      reference.width > image.width || reference.height > image.height)
    return std::nullopt;
  std::vector<Gray_Image> image_pyramid{image};
  std::vector<Gray_Image> reference_pyramid{reference};
  while (std::min(
           reference_pyramid.back().width,
           reference_pyramid.back().height) > 16 &&
         std::max(
           image_pyramid.back().width,
           image_pyramid.back().height) > 700)
  {
    image_pyramid.push_back(Downsample(image_pyramid.back()));
    reference_pyramid.push_back(Downsample(reference_pyramid.back()));
  }
  const int coarsest =
    static_cast<int>(image_pyramid.size()) - 1;
  const auto &coarse_image = image_pyramid[coarsest];
  const auto &coarse_reference = reference_pyramid[coarsest];
  const auto coarse_samples = Matching_Samples(coarse_reference);
  const int scan_step = std::max(
    1, std::max(coarse_image.width, coarse_image.height) / 700);
  std::vector<Match_Candidate> candidates;
  for (int y = 0;
       y + coarse_reference.height <= coarse_image.height;
       y += scan_step)
  {
    for (int x = 0;
         x + coarse_reference.width <= coarse_image.width;
         x += scan_step)
    {
      Keep_Candidate(
        &candidates,
        {x, y, Sampled_Ncc(
          coarse_image, coarse_reference, coarse_samples, x, y)},
        24);
    }
  }
  if (scan_step > 1)
  {
    const auto coarse_candidates = candidates;
    std::vector<Match_Candidate> refined;
    for (const auto &candidate : coarse_candidates)
    {
      for (int dy = -scan_step; dy <= scan_step; ++dy)
      {
        for (int dx = -scan_step; dx <= scan_step; ++dx)
        {
          const int x = candidate.x + dx;
          const int y = candidate.y + dy;
          if (x < 0 || y < 0 ||
              x + coarse_reference.width > coarse_image.width ||
              y + coarse_reference.height > coarse_image.height)
            continue;
          Keep_Candidate(
            &refined,
            {x, y, Sampled_Ncc(
              coarse_image, coarse_reference, coarse_samples, x, y)},
            24);
        }
      }
    }
    candidates = std::move(refined);
  }
  for (int level = coarsest - 1; level >= 0; --level)
  {
    const auto &level_image = image_pyramid[level];
    const auto &level_reference = reference_pyramid[level];
    const auto samples = Matching_Samples(level_reference);
    std::vector<Match_Candidate> refined;
    for (const auto &candidate : candidates)
    {
      const int expected_x = candidate.x * 2;
      const int expected_y = candidate.y * 2;
      for (int dy = -3; dy <= 3; ++dy)
      {
        for (int dx = -3; dx <= 3; ++dx)
        {
          const int x = expected_x + dx;
          const int y = expected_y + dy;
          if (x < 0 || y < 0 ||
              x + level_reference.width > level_image.width ||
              y + level_reference.height > level_image.height)
            continue;
          Keep_Candidate(
            &refined,
            {x, y, Sampled_Ncc(
              level_image, level_reference, samples, x, y)},
            24);
        }
      }
    }
    candidates = std::move(refined);
  }
  if (candidates.empty()) return std::nullopt;
  Match_Candidate best;
  for (const auto &candidate : candidates)
  {
    auto dense = candidate;
    dense.score = Dense_Ncc(
      image, reference, candidate.x, candidate.y);
    if (dense.score > best.score) best = dense;
  }
  return best;
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
  if (cross_template.reference_gray.size() !=
      static_cast<std::size_t>(cross_template.roi.width) *
        cross_template.roi.height)
  {
    if (error_message) *error_message = "标准模板图像未加载";
    return false;
  }
  const double sx = cross_template.reference_width > 0
    ? static_cast<double>(image.width) / cross_template.reference_width
    : 1.0;
  const double sy = cross_template.reference_height > 0
    ? static_cast<double>(image.height) / cross_template.reference_height
    : 1.0;
  const int template_width = std::max(
    4, static_cast<int>(std::lround(cross_template.roi.width * sx)));
  const int template_height = std::max(
    4, static_cast<int>(std::lround(cross_template.roi.height * sy)));
  const Gray_Image reference = Resize_Gray(
    cross_template.reference_gray,
    cross_template.roi.width,
    cross_template.roi.height,
    template_width,
    template_height);
  const Camera_2D_Roi whole_image{
    0, 0, static_cast<int>(image.width), static_cast<int>(image.height)};
  const Gray_Image gray_image{
    static_cast<int>(image.width),
    static_cast<int>(image.height),
    Grayscale(image, whole_image)};
  const auto match = Match_Template(gray_image, reference);
  if (!match)
  {
    if (error_message) *error_message = "标准模板尺寸大于当前图像";
    return false;
  }
  Camera_2D_Cross_Detection result;
  result.template_id = cross_template.id;
  result.template_name = cross_template.name;
  const Camera_2D_Roi matched_roi{
    match->x, match->y, template_width, template_height};
  const auto matched_gray = Grayscale(image, matched_roi);
  Camera_2D_Cross_Detection feature_detection;
  const bool feature_analyzed = Analyze(
    image,
    matched_roi,
    cross_template.dark_on_light,
    Otsu_Threshold(matched_gray),
    cross_template.foreground_ratio,
    &feature_detection,
    nullptr,
    nullptr);
  const double template_confidence = std::clamp(
    (match->score + 1.0) * 0.5, 0.0, 1.0);
  if (feature_analyzed)
  {
    result = std::move(feature_detection);
    result.found = result.found && match->score >= 0.15;
    result.confidence = std::clamp(
      result.confidence * 0.55 + template_confidence * 0.45,
      0.0,
      1.0);
  }
  else
  {
    result.found = false;
    result.search_roi = matched_roi;
    result.center_x =
      match->x + cross_template.feature_center_x * sx;
    result.center_y =
      match->y + cross_template.feature_center_y * sy;
    result.angle_deg = cross_template.reference_angle_deg;
    result.confidence = template_confidence;
  }
  result.template_id = cross_template.id;
  result.template_name = cross_template.name;
  *detection = std::move(result);
  if (error_message) error_message->clear();
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
      item.feature_center_x =
        node.attribute("featureCenterX").as_double(
          item.roi.width * 0.5);
      item.feature_center_y =
        node.attribute("featureCenterY").as_double(
          item.roi.height * 0.5);
      item.reference_image = node.attribute("referenceImage").as_string();
      int pgm_width = 0;
      int pgm_height = 0;
      if (!item.reference_image.empty())
      {
        Read_Pgm(
          Template_Root() / item.reference_image,
          &item.reference_gray,
          &pgm_width,
          &pgm_height);
      }
      if (pgm_width == item.roi.width &&
          pgm_height == item.roi.height &&
          !item.reference_gray.empty())
      {
        Camera_2D_Display_Image reference_image;
        reference_image.width = static_cast<unsigned int>(pgm_width);
        reference_image.height = static_cast<unsigned int>(pgm_height);
        reference_image.rgb.resize(item.reference_gray.size() * 3);
        for (std::size_t index = 0;
             index < item.reference_gray.size(); ++index)
        {
          reference_image.rgb[index * 3] =
            reference_image.rgb[index * 3 + 1] =
            reference_image.rgb[index * 3 + 2] =
              item.reference_gray[index];
        }
        Camera_2D_Cross_Detection reference_detection;
        if (Analyze(
              reference_image,
              {0, 0, pgm_width, pgm_height},
              item.dark_on_light,
              item.threshold,
              item.foreground_ratio,
              &reference_detection,
              nullptr,
              nullptr))
        {
          item.feature_center_x = reference_detection.center_x;
          item.feature_center_y = reference_detection.center_y;
          item.reference_outline = reference_detection.outline;
        }
      }
      if (!item.id.empty() && !item.name.empty() &&
          !item.reference_gray.empty())
        loaded.push_back(std::move(item));
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
  item.feature_center_x = detection.center_x - roi.x;
  item.feature_center_y = detection.center_y - roi.y;
  item.reference_image = image_name;
  item.reference_gray = reference_gray;
  item.reference_outline.reserve(detection.outline.size());
  for (const auto &point : detection.outline)
    item.reference_outline.push_back(
      {point[0] - roi.x, point[1] - roi.y});
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

bool Camera_2D_Cross_Template_Service::Update(
  const std::string &template_id,
  const Camera_2D_Display_Image &image,
  const Camera_2D_Roi &input_roi,
  std::string *error_message)
{
  Camera_2D_Cross_Template previous;
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
      if (error_message) *error_message = "要编辑的2D模板不存在";
      return false;
    }
    previous = *found;
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
    return false;
  if (!detection.found || detection.confidence < 0.3)
  {
    if (error_message)
      *error_message =
        "编辑后的ROI没有识别到完整十字，请调整位置或大小";
    return false;
  }

  Camera_2D_Cross_Template updated = previous;
  updated.roi = roi;
  updated.reference_width = image.width;
  updated.reference_height = image.height;
  updated.dark_on_light = dark_on_light;
  updated.threshold = threshold;
  updated.foreground_ratio = 0.0;
  const auto component = Select_Component(
    reference_gray,
    roi.width,
    roi.height,
    threshold,
    dark_on_light,
    0.0);
  if (component)
    updated.foreground_ratio =
      static_cast<double>(component->pixels.size()) /
      (roi.width * roi.height);
  updated.reference_angle_deg = detection.angle_deg;
  updated.feature_center_x = detection.center_x - roi.x;
  updated.feature_center_y = detection.center_y - roi.y;
  updated.reference_gray = reference_gray;
  updated.reference_outline.clear();
  updated.reference_outline.reserve(detection.outline.size());
  for (const auto &point : detection.outline)
    updated.reference_outline.push_back(
      {point[0] - roi.x, point[1] - roi.y});

  if (updated.reference_image.empty() ||
      !Write_Pgm(
        Template_Root() / updated.reference_image,
        reference_gray,
        roi.width,
        roi.height))
  {
    if (error_message) *error_message = "更新十字模板参考图失败";
    return false;
  }
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
      Write_Pgm(
        Template_Root() / previous.reference_image,
        previous.reference_gray,
        previous.roi.width,
        previous.roi.height);
      if (error_message) *error_message = "编辑过程中2D模板已被删除";
      return false;
    }
    *found = updated;
    m_active_template_id = template_id;
    m_latest_detection.reset();
  }
  if (!Save(error_message))
  {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      const auto found = std::find_if(
        m_templates.begin(), m_templates.end(),
        [&template_id](const auto &item)
        {
          return item.id == template_id;
        });
      if (found != m_templates.end()) *found = previous;
    }
    Write_Pgm(
      Template_Root() / previous.reference_image,
      previous.reference_gray,
      previous.roi.width,
      previous.roi.height);
    return false;
  }
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
    m_latest_detection.reset();
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
    node.append_attribute("featureCenterX") = item.feature_center_x;
    node.append_attribute("featureCenterY") = item.feature_center_y;
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
