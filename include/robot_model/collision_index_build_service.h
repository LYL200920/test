#ifndef includeguard_collision_index_build_service_h_includeguard
#define includeguard_collision_index_build_service_h_includeguard

#include "robot_collision_settings.h"
#include "robot_model_data.h"
#include "robot_visual_data.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace robot_model
{

  struct Collision_Index_Build_Request
  {
    std::shared_ptr<const std::vector<float>> source_xyz;
    Robot_Collision_Settings settings;
    Robot_Joint_State reference_joint_state;
    std::vector<Robot_Visual_Part> robot_parts;
    std::vector<Matrix4> reference_world_from_parts;
  };

  struct Collision_Index_Build_Result
  {
    std::shared_ptr<const Point_Cloud_Collision_Snapshot> obstacle_snapshot;
    std::shared_ptr<const std::vector<float>> source_xyz;
    Robot_Collision_Settings settings;
    Robot_Joint_State reference_joint_state;
    Robot_Collision_Point_Cloud_Stats stats;
    std::string error_message;
    bool success = false;
    bool cancelled = false;
  };

  Collision_Index_Build_Result Build_Collision_Index_Snapshot(
      Collision_Index_Build_Request request,
      const std::atomic_bool *cancel_requested = nullptr);

  // Owns the single collision-index worker. New submissions cancel active and
  // queued work; completion runs on the worker thread and carries a generation
  // that the UI must validate after dispatching back to its own thread.
  class Collision_Index_Build_Service
  {
  public:
    using Completion = std::function<void(std::uint64_t, Collision_Index_Build_Result)>;

    Collision_Index_Build_Service();
    ~Collision_Index_Build_Service();

    Collision_Index_Build_Service(const Collision_Index_Build_Service &) = delete;
    Collision_Index_Build_Service &operator=(const Collision_Index_Build_Service &) = delete;

    std::uint64_t Submit(Collision_Index_Build_Request request, Completion completion);
    void Cancel();
    void Shutdown();
    bool Busy() const { return m_busy.load(); }
    bool Is_Current(std::uint64_t generation) const
    {
      return generation == m_generation.load();
    }

  private:
    struct Task
    {
      std::uint64_t generation = 0;
      Collision_Index_Build_Request request;
      std::shared_ptr<std::atomic_bool> cancel_requested;
      Completion completion;
    };

    void Worker_Loop();
    void Cancel_Locked();

  private:
    std::thread m_worker;
    mutable std::mutex m_mutex;
    std::condition_variable m_condition;
    std::optional<Task> m_pending;
    std::shared_ptr<std::atomic_bool> m_active_cancel;
    std::atomic<std::uint64_t> m_generation{0};
    std::atomic<bool> m_busy{false};
    bool m_stopping = false;
  };

} // namespace robot_model

#endif
