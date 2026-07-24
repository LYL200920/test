#include "point_cloud_persistence.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace point_cloud
{
  namespace
  {

    constexpr const char *kApplicationMarker = "comment application robot_point_cloud_overlay";
    constexpr std::size_t kMaximumFilePoints = 10000000;

    void set_error(std::string *error_message, const std::string &message)
    {
      if (error_message)
        *error_message = message;
    }

    bool is_safe_comment_value(const std::string &value)
    {
      return value.find('\n') == std::string::npos && value.find('\r') == std::string::npos;
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

    const auto temporary_path = path.string() + ".tmp";
    std::ofstream file(temporary_path, std::ios::binary | std::ios::trunc);
    if (!file)
    {
      set_error(error_message, "Unable to create point-cloud file");
      return false;
    }
    file << "ply\n"
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
      file << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    }
    file << "end_header\n";

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
    if (cloud == nullptr || metadata == nullptr)
    {
      set_error(error_message, "Point-cloud output is null");
      return false;
    }
    *cloud = {};
    *metadata = {};
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
      set_error(error_message, "Unable to open point-cloud file");
      return false;
    }

    std::string line;
    bool saw_ply = false;
    bool saw_format = false;
    bool saw_application = false;
    bool saw_end_header = false;
    bool has_color = false;
    std::size_t vertex_count = 0;
    std::size_t property_count = 0;
    while (std::getline(file, line))
    {
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      if (line == "ply")
        saw_ply = true;
      else if (line == "format binary_little_endian 1.0")
        saw_format = true;
      else if (line == kApplicationMarker)
        saw_application = true;
      else if (starts_with(line, "comment coordinate_frame "))
        metadata->coordinate_frame = comment_value(line, "coordinate_frame");
      else if (starts_with(line, "comment unit "))
        metadata->unit = comment_value(line, "unit");
      else if (starts_with(line, "comment robot_model "))
        metadata->robot_model = comment_value(line, "robot_model");
      else if (starts_with(line, "comment source_frame_number "))
      {
        try
        {
          metadata->source_frame_number = static_cast<std::uint32_t>(std::stoul(comment_value(line, "source_frame_number")));
        }
        catch (...)
        {
          set_error(error_message, "Invalid source frame metadata");
          return false;
        }
      }
      else if (starts_with(line, "comment saved_unix_milliseconds "))
      {
        try
        {
          metadata->saved_unix_milliseconds = std::stoll(comment_value(line, "saved_unix_milliseconds"));
        }
        catch (...)
        {
          set_error(error_message, "Invalid save-time metadata");
          return false;
        }
      }
      else if (starts_with(line, "element vertex "))
      {
        try
        {
          vertex_count = std::stoull(line.substr(15));
        }
        catch (...)
        {
          set_error(error_message, "Invalid PLY vertex count");
          return false;
        }
      }
      else if (starts_with(line, "property "))
      {
        ++property_count;
        if (line == "property uchar red" || line == "property uchar green" || line == "property uchar blue")
          has_color = true;
      }
      else if (line == "end_header")
      {
        saw_end_header = true;
        break;
      }
    }

    if (!saw_ply || !saw_format || !saw_application || !saw_end_header ||
        metadata->coordinate_frame != "robot_base" ||
        metadata->unit != "millimeter")
    {
      set_error(error_message, "PLY is not a robot-base/millimeter overlay file created by this application");
      return false;
    }
    const std::size_t expected_properties = has_color ? 6 : 3;
    if (vertex_count == 0 || vertex_count > kMaximumFilePoints || property_count != expected_properties)
    {
      set_error(error_message, "PLY vertex layout is unsupported or empty");
      return false;
    }

    cloud->xyz.resize(vertex_count * 3);
    if (has_color)
      cloud->rgb.resize(vertex_count * 3);
    for (std::size_t index = 0; index < vertex_count; ++index)
    {
      file.read(reinterpret_cast<char *>(cloud->xyz.data() + index * 3), sizeof(float) * 3);
      if (has_color)
      {
        file.read(reinterpret_cast<char *>(cloud->rgb.data() + index * 3), 3);
      }
      if (!file)
      {
        *cloud = {};
        set_error(error_message, "PLY point data is truncated");
        return false;
      }
    }
    return true;
  }

} // namespace point_cloud
