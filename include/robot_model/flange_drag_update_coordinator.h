#ifndef includeguard_flange_drag_update_coordinator_h_includeguard
#define includeguard_flange_drag_update_coordinator_h_includeguard

#include <chrono>

namespace robot_model
{

struct Flange_Drag_Pointer
{
  int display_x = 0;
  int display_y = 0;
};

struct Flange_Drag_Schedule
{
  bool process_now = false;
  int timer_delay_ms = 0;
};

// Coalesces high-frequency pointer events without depending on a UI toolkit.
// The view owns the actual timer and executes updates requested by this policy.
class Flange_Drag_Update_Coordinator
{
public:
  using Clock = std::chrono::steady_clock;

  explicit Flange_Drag_Update_Coordinator (
    std::chrono::milliseconds minimum_interval =
      std::chrono::milliseconds (16));

  void Begin ( );
  Flange_Drag_Schedule Queue (int display_x, int display_y);
  Flange_Drag_Schedule Queue_At (
    int display_x,
    int display_y,
    Clock::time_point now);
  void Set_Final (int display_x, int display_y);
  bool Take_Pending (Flange_Drag_Pointer* pointer);
  void Mark_Processed ( );
  void Mark_Processed_At (Clock::time_point now);
  void Cancel ( );

private:
  std::chrono::milliseconds m_minimum_interval;
  Clock::time_point m_last_processed_at;
  Flange_Drag_Pointer m_pending_pointer;
  bool m_active = false;
  bool m_has_pending = false;
};

} // namespace robot_model

#endif
