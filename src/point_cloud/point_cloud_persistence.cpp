#include "point_cloud_persistence.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

namespace point_cloud
{
  namespace
  {

    constexpr const char *kApplicationMarker = "comment application robot_point_cloud_overlay";
    constexpr std::size_t kMaximumFilePoints = 10000000;
    constexpr std::size_t kMaximumHeaderBytes = 65536;

    template<typename Integer>
    bool parse_integer(const std::string &text, Integer *value)
    {
      const auto result = std::from_chars(text.data(), text.data() + text.size(), *value);
      return result.ec == std::errc{} && result.ptr == text.data() + text.size();
    }

    void set_error(std::string *error_message, const std::string &message)
    {
      if (error_message)
        *error_message = message;
    }

    bool is_safe_comment_value(const std::string &value)
    {
      return value.find('\n') == std::string::npos &&
             value.find('\r') == std::string::npos &&
             value.find('\0') == std::string::npos;
    }

    bool starts_with(const std::string &text, const std::string &prefix)
    {
      return text.size() >= prefix.size() && std::equal(prefix.begin(), prefix.end(), text.begin());
    }

    std::string comment_value(const std::string &line, const char *key)
    {
      const std::string prefix = std::string("comment ") + key + " ";
      return starts_with(line, prefix) ? line.substr(prefix.size()) : "";
    }

  } // namespace

  bool Save_Robot_Base_Point_Cloud_Ply(const std::filesystem::path &path,
                                       const Point_Cloud_Data &cloud,
                                       const Point_Cloud_File_Metadata &metadata,
                                       std::string *error_message)
  {
    if (error_message)
      error_message->clear();
    if (path.empty() || path.extension() != ".ply")
    {
      set_error(error_message, "Point-cloud file must use the .ply extension");
      return false;
    }
    if (cloud.xyz.empty() || (cloud.xyz.size() % 3) != 0 || cloud.Point_Count() > kMaximumFilePoints)
    {
      set_error(error_message, "Point cloud is empty, malformed or too large");
      return false;
    }
    if (!cloud.rgb.empty() && !cloud.Has_Color())
    {
      set_error(error_message, "Point-cloud color array is malformed");
      return false;
    }
    if (metadata.coordinate_frame != "robot_base" || metadata.unit != "millimeter" || !is_safe_comment_value(metadata.robot_model))
    {
      set_error(error_message, "Point-cloud metadata must declare robot_base/millimeter");
      return false;
    }
    if (std::filesystem::exists(path))
    {
      set_error(error_message, "Point-cloud file already exists");
      return false;
    }

    std::ostringstream header;
    header << "ply\n"
         << "format binary_little_endian 1.0\n"
         << kApplicationMarker << '\n'
         << "comment coordinate_frame " << metadata.coordinate_frame << '\n'
         << "comment unit " << metadata.unit << '\n'
         << "comment robot_model " << metadata.robot_model << '\n'
         << "comment source_frame_number "
         << metadata.source_frame_number << '\n'
         << "comment saved_unix_milliseconds "
         << metadata.saved_unix_milliseconds << '\n'
         << "element vertex " << cloud.Point_Count() << '\n'
         << "property float x\nproperty float y\nproperty float z\n";
    if (cloud.Has_Color())
    {
      header << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    }
    header << "end_header\n";
    const auto header_text = header.str();
    if (header_text.size() > kMaximumHeaderBytes)
    {
      set_error(error_message, "PLY header is too large");
      return false;
    }

    const auto temporary_path = path.string() + ".tmp";
    std::ofstream file(temporary_path, std::ios::binary | std::ios::trunc);
    if (!file)
    {
      set_error(error_message, "Unable to create point-cloud file");
      return false;
    }
    file << header_text;
    for (std::size_t index = 0; index < cloud.Point_Count(); ++index)
    {
      const float *xyz = cloud.xyz.data() + index * 3;
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
      {
        file.close();
        std::filesystem::remove(temporary_path);
        set_error(error_message, "Point cloud contains a non-finite coordinate");
        return false;
      }
      file.write(reinterpret_cast<const char *>(xyz), sizeof(float) * 3);
      if (cloud.Has_Color())
      {
        file.write(reinterpret_cast<const char *>(cloud.rgb.data() + index * 3), 3);
      }
    }
    file.close();
    if (!file)
    {
      std::filesystem::remove(temporary_path);
      set_error(error_message, "Writing point-cloud file failed");
      return false;
    }

    std::error_code rename_error;
    std::filesystem::rename(temporary_path, path, rename_error);
    if (rename_error)
    {
      std::filesystem::remove(temporary_path);
      set_error(error_message, "Unable to finalize point-cloud file: " + rename_error.message());
      return false;
    }
    return true;
  }

  bool Load_Robot_Base_Point_Cloud_Ply(const std::filesystem::path &path,
                                       Point_Cloud_Data *cloud,
                                       Point_Cloud_File_Metadata *metadata,
                                       std::string *error_message)
  {
    if (error_message)
      error_message->clear();
    if (cloud)
      *cloud = {};
    if (metadata)
      *metadata = {};
    if (cloud == nullptr || metadata == nullptr)
    {
      set_error(error_message, "Point-cloud output is null");
      return false;
    }
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
      set_error(error_message, "Unable to open point-cloud file");
      return false;
    }

    Point_Cloud_Data loaded;
    Point_Cloud_File_Metadata loaded_metadata;
    std::string line;
    bool saw_ply = false;
    bool saw_format = false;
    bool saw_application = false;
    bool saw_end_header = false;
    bool saw_vertex = false;
    std::size_t vertex_count = 0;
    std::size_t property_count = 0;
    std::size_t header_bytes = 0;
    const std::array<const char *, 6> properties = {
        "property float x", "property float y", "property float z",
        "property uchar red", "property uchar green", "property uchar blue"};
    // Bound line reads as well as the total header before allocating point data.
    std::array<char, kMaximumHeaderBytes + 1> header_line{};
    while (file.getline(header_line.data(), header_line.size()))
    {
      header_bytes += static_cast<std::size_t>(file.gcount());
      if (header_bytes > kMaximumHeaderBytes)
      {
        set_error(error_message, "PLY header is too large");
        return false;
      }
      line.assign(header_line.data(), static_cast<std::size_t>(file.gcount()) -
                  (file.eof() ? 0 : 1));
      if (line.find('\0') != std::string::npos)
      {
        set_error(error_message, "PLY header contains a null byte");
        return false;
      }
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      if (!saw_ply)
      {
        if (line != "ply")
        {
          set_error(error_message, "Invalid PLY signature");
          return false;
        }
        saw_ply = true;
      }
      else if (!saw_format)
      {
        if (line != "format binary_little_endian 1.0")
        {
          set_error(error_message, "Unsupported PLY format");
          return false;
        }
        saw_format = true;
      }
      else if (line == kApplicationMarker)
        saw_application = true;
      else if (starts_with(line, "comment coordinate_frame "))
        loaded_metadata.coordinate_frame = comment_value(line, "coordinate_frame");
      else if (starts_with(line, "comment unit "))
        loaded_metadata.unit = comment_value(line, "unit");
      else if (starts_with(line, "comment robot_model "))
        loaded_metadata.robot_model = comment_value(line, "robot_model");
      else if (starts_with(line, "comment source_frame_number "))
      {
        if (!parse_integer(comment_value(line, "source_frame_number"),
                           &loaded_metadata.source_frame_number))
        {
          set_error(error_message, "Invalid source frame metadata");
          return false;
        }
      }
      else if (starts_with(line, "comment saved_unix_milliseconds "))
      {
        if (!parse_integer(comment_value(line, "saved_unix_milliseconds"),
                           &loaded_metadata.saved_unix_milliseconds))
        {
          set_error(error_message, "Invalid save-time metadata");
          return false;
        }
      }
      else if (starts_with(line, "element vertex "))
      {
        if (saw_vertex || !parse_integer(line.substr(15), &vertex_count))
        {
          set_error(error_message, "Invalid or repeated PLY vertex element");
          return false;
        }
        saw_vertex = true;
      }
      else if (starts_with(line, "property "))
      {
        if (!saw_vertex || property_count >= properties.size() ||
            line != properties[property_count])
        {
          set_error(error_message, "PLY vertex property type or order is unsupported");
          return false;
        }
        ++property_count;
      }
      else if (line == "end_header")
      {
        saw_end_header = true;
        break;
      }
      else if (line != "comment" && !starts_with(line, "comment "))
      {
        set_error(error_message, "Unsupported PLY header declaration");
        return false;
      }
    }

    if (!saw_ply || !saw_format || !saw_application || !saw_end_header ||
        loaded_metadata.coordinate_frame != "robot_base" ||
        loaded_metadata.unit != "millimeter")
    {
      set_error(error_message, "PLY is not a robot-base/millimeter overlay file created by this application");
      return false;
    }
    if (!saw_vertex || vertex_count == 0 || vertex_count > kMaximumFilePoints ||
        (property_count != 3 && property_count != 6))
    {
      set_error(error_message, "PLY vertex layout is unsupported or empty");
      return false;
    }
    const bool has_color = property_count == 6;
    const auto data_start = file.tellg();
    file.seekg(0, std::ios::end);
    const auto data_end = file.tellg();
    const auto expected_bytes = static_cast<std::streamoff>(
        vertex_count * (sizeof(float) * 3 + (has_color ? 3 : 0)));
    if (data_start == std::streampos(-1) || data_end == std::streampos(-1) ||
        data_end - data_start != expected_bytes)
    {
      set_error(error_message, "PLY point data size does not match the vertex layout");
      return false;
    }
    file.seekg(data_start);

    loaded.xyz.resize(vertex_count * 3);
    if (has_color)
      loaded.rgb.resize(vertex_count * 3);
    for (std::size_t index = 0; index < vertex_count; ++index)
    {
      float *xyz = loaded.xyz.data() + index * 3;
      file.read(reinterpret_cast<char *>(xyz), sizeof(float) * 3);
      if (has_color)
        file.read(reinterpret_cast<char *>(loaded.rgb.data() + index * 3), 3);
      if (!file)
      {
        set_error(error_message, "PLY point data is truncated");
        return false;
      }
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
      {
        set_error(error_message, "PLY contains a non-finite coordinate");
        return false;
      }
    }
    *cloud = std::move(loaded);
    *metadata = std::move(loaded_metadata);
    return true;
  }

} // namespace point_cloud
