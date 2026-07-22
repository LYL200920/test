#ifndef includeguard_collision_index_rebuild_coordinator_h_includeguard
#define includeguard_collision_index_rebuild_coordinator_h_includeguard

#include "collision_index_build_service.h"

#include <atomic>
#include <functional>
#include <memory>
#include <optional>

namespace robot_model
{

// Coordinates rebuild requests above the worker service. It preserves the
// newest point-cloud request when settings change and suppresses completions
// that become obsolete before the UI dispatcher executes them.
class Collision_Index_Rebuild_Coordinator
{
public:
  using Dispatch = std::function<void (std::function<void ( )>)>;
  using Completion = std::function<void (Collision_Index_Build_Result)>;

  Collision_Index_Rebuild_Coordinator ( );
  ~Collision_Index_Rebuild_Coordinator ( );

  Collision_Index_Rebuild_Coordinator (
    const Collision_Index_Rebuild_Coordinator&) = delete;
  Collision_Index_Rebuild_Coordinator& operator= (
    const Collision_Index_Rebuild_Coordinator&) = delete;

  bool Submit_Source (
    Collision_Index_Build_Request request,
    Dispatch dispatch,
    Completion completion);
  bool Submit_Settings (
    Collision_Index_Build_Request request,
    Dispatch dispatch,
    Completion completion);
  void Reset ( );
  void Shutdown ( );
  bool Busy ( ) const;

private:
  bool Submit_Prepared (
    Collision_Index_Build_Request request,
    Dispatch dispatch,
    Completion completion);

private:
  Collision_Index_Build_Service m_build_service;
  std::optional<Collision_Index_Build_Request> m_latest_request;
  std::shared_ptr<std::atomic<std::uint64_t>> m_publish_generation;
  bool m_shutdown = false;
};

} // namespace robot_model

#endif
