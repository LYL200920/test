#ifndef includeguard_camera_frame_h_includeguard
#define includeguard_camera_frame_h_includeguard

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

struct Camera_Image_Frame
{
  std::uint32_t image_type = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t frame_number = 0;
  std::int64_t timestamp = 0;
  bool is_rectified = false;
  std::uint32_t stream_type = 0;
  std::uint32_t coordinate_type = 0;
  std::vector<std::uint8_t> data;
};

struct Camera_Frame
{
  std::uint64_t generation = 0;
  std::uint32_t valid_info = 0;
  std::uint32_t trigger_id = 0;
  std::vector<Camera_Image_Frame> images;
};

class Camera_Frame_Buffer
{
public:
  void Publish (Camera_Frame frame);
  std::shared_ptr<const Camera_Frame> Latest ( ) const;
  void Clear ( );

private:
  mutable std::mutex m_mutex;
  std::shared_ptr<const Camera_Frame> m_latest_frame;
  std::uint64_t m_generation = 0;
};

#endif
