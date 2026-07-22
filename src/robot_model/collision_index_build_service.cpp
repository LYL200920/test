#include "collision_index_build_service.h"

#include <chrono>
#include <utility>

namespace robot_model
{

Collision_Index_Build_Result Build_Collision_Index_Snapshot (
  Collision_Index_Build_Request request,
  const std::atomic_bool* cancel_requested)
{
  const auto started_at = std::chrono::steady_clock::now ( );
  Collision_Index_Build_Result result;
  result.source_xyz = std::move (request.source_xyz);
  result.settings = request.settings;
  result.reference_joint_state = request.reference_joint_state;
  const auto finish = [&]
  {
    result.stats.build_time_ms = std::chrono::duration<double, std::milli> (
      std::chrono::steady_clock::now ( ) - started_at).count ( );
  };
  if( cancel_requested && cancel_requested->load ( ) )
  {
    result.cancelled = true;
    finish ( );
    return result;
  }

  Robot_Collision_Detector builder;
  builder.Set_Robot_Parts_For_Obstacle_Filtering (request.robot_parts);
  if( cancel_requested && cancel_requested->load ( ) )
  {
    result.cancelled = true;
    finish ( );
    return result;
  }
  if( !result.source_xyz || result.source_xyz->empty ( ) )
  {
    result.success = true;
    finish ( );
    return result;
  }

  result.success = builder.Set_Obstacle_Points (
    *result.source_xyz, result.settings.point_cloud,
    request.reference_world_from_parts, &result.stats,
    &result.error_message, cancel_requested);
  if( result.success )
    result.obstacle_snapshot = builder.Obstacle_Snapshot ( );
  result.cancelled = cancel_requested &&
    cancel_requested->load (std::memory_order_relaxed);
  finish ( );
  return result;
}

Collision_Index_Build_Service::Collision_Index_Build_Service ( )
  : m_worker (&Collision_Index_Build_Service::Worker_Loop, this)
{
}

Collision_Index_Build_Service::~Collision_Index_Build_Service ( )
{
  Shutdown ( );
}

std::uint64_t Collision_Index_Build_Service::Submit (
  Collision_Index_Build_Request request,
  Completion completion)
{
  const auto generation = ++m_generation;
  auto cancel_requested = std::make_shared<std::atomic_bool> (false);
  {
    std::lock_guard<std::mutex> lock (m_mutex);
    if( m_stopping ) return 0;
    Cancel_Locked ( );
    m_pending = Task {
      generation, std::move (request), std::move (cancel_requested),
      std::move (completion)
    };
    m_busy.store (true);
  }
  m_condition.notify_one ( );
  return generation;
}

void Collision_Index_Build_Service::Cancel ( )
{
  ++m_generation;
  {
    std::lock_guard<std::mutex> lock (m_mutex);
    Cancel_Locked ( );
    m_busy.store (false);
  }
}

void Collision_Index_Build_Service::Shutdown ( )
{
  {
    std::lock_guard<std::mutex> lock (m_mutex);
    if( m_stopping ) return;
    ++m_generation;
    m_stopping = true;
    Cancel_Locked ( );
    m_busy.store (false);
  }
  m_condition.notify_one ( );
  if( m_worker.joinable ( ) ) m_worker.join ( );
}

void Collision_Index_Build_Service::Cancel_Locked ( )
{
  if( m_active_cancel ) m_active_cancel->store (true);
  if( m_pending && m_pending->cancel_requested )
    m_pending->cancel_requested->store (true);
  m_pending.reset ( );
}

void Collision_Index_Build_Service::Worker_Loop ( )
{
  for( ;; )
  {
    Task task;
    {
      std::unique_lock<std::mutex> lock (m_mutex);
      m_condition.wait (lock, [this]
      {
        return m_stopping || m_pending.has_value ( );
      });
      if( m_stopping ) return;
      task = std::move ( *m_pending );
      m_pending.reset ( );
      m_active_cancel = task.cancel_requested;
    }

    auto result = Build_Collision_Index_Snapshot (
      std::move (task.request), task.cancel_requested.get ( ));
    Completion completion;
    {
      std::lock_guard<std::mutex> lock (m_mutex);
      if( m_active_cancel == task.cancel_requested )
        m_active_cancel.reset ( );
      if( m_stopping || result.cancelled ||
          task.generation != m_generation.load ( ) )
      {
        continue;
      }
      m_busy.store (false);
      completion = std::move (task.completion);
    }
    if( completion ) completion (task.generation, std::move (result));
  }
}

} // namespace robot_model
