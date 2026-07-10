#include "pose_point_io.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace robot_model
{
namespace
{

void set_error (std::string* error_message, const std::string& message)
{
  if( error_message != nullptr )
  {
    *error_message = message;
  }
}

std::string trim (const std::string& text)
{
  size_t first = 0;
  while( first < text.size ( ) &&
         std::isspace (static_cast<unsigned char> (text[first])) )
  {
    ++first;
  }

  size_t last = text.size ( );
  while( last > first &&
         std::isspace (static_cast<unsigned char> (text[last - 1])) )
  {
    --last;
  }

  return text.substr (first, last - first);
}

std::string strip_comment (const std::string& line)
{
  const size_t comment_pos = line.find ('#');
  if( comment_pos == std::string::npos )
  {
    return line;
  }
  return line.substr (0, comment_pos);
}

bool looks_like_header (const std::string& line)
{
  return std::any_of (
    line.begin ( ), line.end ( ),
    [] (unsigned char ch) { return std::isalpha (ch) != 0; });
}

bool parse_pose_line (const std::string& line, XyzabcPose* pose)
{
  if( pose == nullptr )
  {
    return false;
  }

  std::string normalized = line;
  for( char& ch : normalized )
  {
    if( ch == ',' || ch == ';' || ch == '\t' )
    {
      ch = ' ';
    }
  }

  std::stringstream stream (normalized);
  XyzabcPose parsed = { };
  for( double& value : parsed )
  {
    stream >> value;
    if( stream.fail ( ) )
    {
      return false;
    }
  }

  stream >> std::ws;
  if( !stream.eof ( ) )
  {
    return false;
  }

  *pose = parsed;
  return true;
}

} // namespace

bool Load_Pose_Point_File (const std::filesystem::path& path,
                           Pose_Point_File* point_file,
                           std::string* error_message)
{
  if( point_file == nullptr )
  {
    set_error (error_message, "Output point file is null.");
    return false;
  }

  std::ifstream file (path);
  if( !file )
  {
    set_error (error_message, "Cannot open point file for reading.");
    return false;
  }

  std::vector<XyzabcPose> loaded_points;
  std::string line;
  size_t line_number = 0;
  bool data_seen = false;
  while( std::getline (file, line) )
  {
    ++line_number;
    const std::string trimmed_line = trim (strip_comment (line));
    if( trimmed_line.empty ( ) )
    {
      continue;
    }

    if( !data_seen && looks_like_header (trimmed_line) )
    {
      data_seen = true;
      continue;
    }
    data_seen = true;

    XyzabcPose pose = { };
    if( !parse_pose_line (trimmed_line, &pose) )
    {
      std::ostringstream message;
      message << "Invalid point data at line " << line_number
              << ". Expected 6 numeric values.";
      set_error (error_message, message.str ( ));
      return false;
    }
    loaded_points.push_back (pose);
  }

  if( loaded_points.empty ( ) )
  {
    set_error (error_message, "Point file has no points.");
    return false;
  }

  Pose_Point_File result;
  result.default_pose = loaded_points.front ( );
  if( loaded_points.size ( ) > 1 )
  {
    result.transform_points.assign (loaded_points.begin ( ) + 1,
                                    loaded_points.end ( ));
  }
  *point_file = result;
  return true;
}

} // namespace robot_model
