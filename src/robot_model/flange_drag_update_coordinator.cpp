#include "flange_drag_update_coordinator.h"

#include <algorithm>
#include <cstdint>

namespace robot_model
{

  Flange_Drag_Update_Coordinator::Flange_Drag_Update_Coordinator(std::chrono::milliseconds minimum_interval)
      : m_minimum_interval(std::max(minimum_interval, std::chrono::milliseconds(0)))
  {
  }

  void Flange_Drag_Update_Coordinator::Begin()
  {
    m_active = true;
    m_has_pending = false;
    m_last_processed_at = {};
  }

  Flange_Drag_Schedule Flange_Drag_Update_Coordinator::Queue(
      int display_x,
      int display_y)
  {
    return Queue_At(display_x, display_y, Clock::now());
  }

  Flange_Drag_Schedule Flange_Drag_Update_Coordinator::Queue_At(int display_x,
                                                                int display_y,
                                                                Clock::time_point now)
  {
    if (!m_active)
      return {};

    m_pending_pointer = {display_x, display_y};
    m_has_pending = true;
    if (m_last_processed_at.time_since_epoch().count() == 0 ||
        now - m_last_processed_at >= m_minimum_interval)
    {
      return {true, 0};
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_processed_at);
    const auto remaining = m_minimum_interval - elapsed;
    return {false, static_cast<int>(std::max<std::int64_t>(1, remaining.count()))};
  }

  void Flange_Drag_Update_Coordinator::Set_Final(int display_x, int display_y)
  {
    if (!m_active)
      return;
    m_pending_pointer = {display_x, display_y};
    m_has_pending = true;
  }

  bool Flange_Drag_Update_Coordinator::Take_Pending(Flange_Drag_Pointer *pointer)
  {
    if (!pointer || !m_active || !m_has_pending)
      return false;
    *pointer = m_pending_pointer;
    m_has_pending = false;
    return true;
  }

  void Flange_Drag_Update_Coordinator::Mark_Processed()
  {
    Mark_Processed_At(Clock::now());
  }

  void Flange_Drag_Update_Coordinator::Mark_Processed_At(Clock::time_point now)
  {
    if (m_active)
      m_last_processed_at = now;
  }

  void Flange_Drag_Update_Coordinator::Cancel()
  {
    m_active = false;
    m_has_pending = false;
    m_last_processed_at = {};
  }

} // namespace robot_model
