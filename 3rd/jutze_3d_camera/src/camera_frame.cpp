#include "camera_frame.h"

#include <utility>

void Camera_Frame_Buffer::Publish (Camera_Frame frame)
{
  std::lock_guard<std::mutex> lock (m_mutex);
  frame.generation = ++m_generation;
  m_latest_frame = std::make_shared<Camera_Frame> (std::move (frame));
}

std::shared_ptr<const Camera_Frame> Camera_Frame_Buffer::Latest ( ) const
{
  std::lock_guard<std::mutex> lock (m_mutex);
  return m_latest_frame;
}

void Camera_Frame_Buffer::Clear ( )
{
  std::lock_guard<std::mutex> lock (m_mutex);
  m_latest_frame.reset ( );
}
