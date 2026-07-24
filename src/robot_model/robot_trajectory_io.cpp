#include "robot_trajectory_io.h"

#include <cctype>
#include <fstream>
#include <sstream>

namespace robot_model
{
  namespace
  {

    void set_error(std::string *error_message, const std::string &message)
    {
      if (error_message)
      {
        *error_message = message;
      }
    }

    std::string trim(const std::string &text)
    {
      size_t first = 0;
      while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])))
      {
        ++first;
      }

      size_t last = text.size();
      while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])))
      {
        --last;
      }

      return text.substr(first, last - first);
    }

    bool parse_csv_point_line(const std::string &line, std::array<double, 6> *point)
    {
      if (point == nullptr)
      {
        return false;
      }

      std::stringstream stream(line);
      std::string cell;
      std::array<double, 6> values = {};
      size_t value_count = 0;

      while (std::getline(stream, cell, ','))
      {
        if (value_count >= values.size())
        {
          return false;
        }

        std::stringstream value_stream(trim(cell));
        double value = 0.0;
        value_stream >> value;
        if (value_stream.fail())
        {
          return false;
        }

        value_stream >> std::ws;
        if (!value_stream.eof())
        {
          return false;
        }

        values[value_count] = value;
        ++value_count;
      }

      if (value_count != values.size())
      {
        return false;
      }

      *point = values;
      return true;
    }

    bool looks_like_header(const std::string &line)
    {
      for (const char ch : line)
      {
        if (std::isalpha(static_cast<unsigned char>(ch)))
        {
          return true;
        }
      }

      return false;
    }

  } // namespace

  bool Save_Joint_Trajectory_Csv(const std::filesystem::path &path,
                                 const std::vector<std::array<double, 6>> &points,
                                 std::string *error_message)
  {
    std::ofstream file(path);
    if (!file)
    {
      set_error(error_message, "Cannot open file for writing.");
      return false;
    }

    file << "A1,A2,A3,A4,A5,A6\n";
    for (const auto &point : points)
    {
      file << point[0] << ','
           << point[1] << ','
           << point[2] << ','
           << point[3] << ','
           << point[4] << ','
           << point[5] << '\n';
    }

    if (!file)
    {
      set_error(error_message, "Failed while writing trajectory file.");
      return false;
    }

    return true;
  }

  bool Load_Joint_Trajectory_Csv(const std::filesystem::path &path,
                                 std::vector<std::array<double, 6>> *points,
                                 std::string *error_message)
  {
    if (points == nullptr)
    {
      set_error(error_message, "Output point container is null.");
      return false;
    }

    std::ifstream file(path);
    if (!file)
    {
      set_error(error_message, "Cannot open file for reading.");
      return false;
    }

    std::vector<std::array<double, 6>> loaded_points;
    std::string line;
    size_t line_number = 0;
    while (std::getline(file, line))
    {
      ++line_number;
      const std::string trimmed_line = trim(line);
      if (trimmed_line.empty())
      {
        continue;
      }

      if (line_number == 1 && looks_like_header(trimmed_line))
      {
        continue;
      }

      std::array<double, 6> point = {};
      if (!parse_csv_point_line(trimmed_line, &point))
      {
        std::stringstream message;
        message << "Invalid CSV data at line " << line_number
                << ". Expected 6 numeric values.";
        set_error(error_message, message.str());
        return false;
      }

      loaded_points.push_back(point);
    }

    if (loaded_points.empty())
    {
      set_error(error_message, "Trajectory file has no points.");
      return false;
    }

    *points = loaded_points;
    return true;
  }

} // namespace robot_model
