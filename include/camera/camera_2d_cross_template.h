#ifndef includeguard_camera_2d_cross_template_h_includeguard
#define includeguard_camera_2d_cross_template_h_includeguard

#include "camera_2d_image_converter.h"

#include <mutex>
#include <array>
#include <optional>
#include <string>
#include <vector>

struct Camera_2D_Roi
{
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

struct Camera_2D_Cross_Template
{
  std::string id;
  std::string name;
  Camera_2D_Roi roi;
  unsigned int reference_width = 0;
  unsigned int reference_height = 0;
  bool dark_on_light = true;
  int threshold = 128;
  double foreground_ratio = 0.0;
  double reference_angle_deg = 0.0;
  double feature_center_x = 0.0;
  double feature_center_y = 0.0;
  std::string reference_image;
  std::vector<std::uint8_t> reference_gray;
  std::vector<std::array<int, 2>> reference_outline;
};

struct Camera_2D_Cross_Detection
{
  bool found = false;
  Camera_2D_Roi search_roi;
  double center_x = 0.0;
  double center_y = 0.0;
  double angle_deg = 0.0;
  double confidence = 0.0;
  std::string template_id;
  std::string template_name;
  std::vector<std::array<int, 2>> outline;
};

bool Detect_Camera_2D_Cross(
  const Camera_2D_Display_Image &image,
  const Camera_2D_Cross_Template &cross_template,
  Camera_2D_Cross_Detection *detection,
  std::string *error_message = nullptr);

class Camera_2D_Cross_Template_Service
{
public:
  Camera_2D_Cross_Template_Service();

  bool Reload(std::string *error_message = nullptr);
  std::vector<Camera_2D_Cross_Template> Templates() const;
  std::optional<Camera_2D_Cross_Template> Active_Template() const;
  bool Set_Active(
    const std::string &template_id,
    std::string *error_message = nullptr);
  bool Create(
    const std::string &name,
    const Camera_2D_Display_Image &image,
    const Camera_2D_Roi &roi,
    std::string *created_id,
    std::string *error_message = nullptr);
  bool Update(
    const std::string &template_id,
    const Camera_2D_Display_Image &image,
    const Camera_2D_Roi &roi,
    std::string *error_message = nullptr);
  bool Remove(
    const std::string &template_id,
    std::string *error_message = nullptr);
  std::optional<Camera_2D_Cross_Detection> Detect(
    const Camera_2D_Display_Image &image) const;
  std::optional<Camera_2D_Cross_Detection> Latest_Detection() const;

private:
  bool Save(std::string *error_message) const;

  mutable std::mutex m_mutex;
  std::vector<Camera_2D_Cross_Template> m_templates;
  std::string m_active_template_id;
  mutable std::optional<Camera_2D_Cross_Detection> m_latest_detection;
};

#endif
