#include "collision_index_rebuild_coordinator.h"

#include <utility>

namespace robot_model
{

Collision_Index_Rebuild_Coordinator::Collision_Index_Rebuild_Coordinator ( )
  : m_publish_generation (
      std::make_shared<std::atomic<std::uint64_t>> (0))
{
}

Collision_Index_Rebuild_Coordinator::~Collision_Index_Rebuild_Coordinator ( )
{
  Shutdown ( );
}

bool Collision_Index_Rebuild_Coordinator::Submit_Source (
  Collision_Index_Build_Request request,
  Dispatch dispatch,
  Completion completion)
{
  if( m_shutdown ) return false;
  m_latest_request = request;
  return Submit_Prepared (
    std::move (request), std::move (dispatch), std::move (completion));
}

bool Collision_Index_Rebuild_Coordinator::Submit_Settings (
  Collision_Index_Build_Request request,
  Dispatch dispatch,
  Completion completion)
{
  if( m_shutdown ) return false;
  if( m_latest_request )
  {
    const auto settings = request.settings;
    request = *m_latest_request;
    request.settings = settings;
  }
  m_latest_request = request;
  return Submit_Prepared (
    std::move (request), std::move (dispatch), std::move (completion));
}

void Collision_Index_Rebuild_Coordinator::Reset ( )
{
  ++( *m_publish_generation );
  m_build_service.Cancel ( );
  m_latest_request.reset ( );
}

void Collision_Index_Rebuild_Coordinator::Shutdown ( )
{
  if( m_shutdown ) return;
  m_shutdown = true;
  ++( *m_publish_generation );
  m_latest_request.reset ( );
  m_build_service.Shutdown ( );
}

bool Collision_Index_Rebuild_Coordinator::Busy ( ) const
{
  return m_build_service.Busy ( );
}

bool Collision_Index_Rebuild_Coordinator::Submit_Prepared (
  Collision_Index_Build_Request request,
  Dispatch dispatch,
  Completion completion)
{
  if( !dispatch || !completion || m_shutdown ) return false;

  const auto publish_generation = ++( *m_publish_generation );
  const auto generation_token = m_publish_generation;
  const auto worker_generation = m_build_service.Submit (
    std::move (request),
    [publish_generation, generation_token,
     dispatch = std::move (dispatch),
     completion = std::move (completion)] (
      std::uint64_t, Collision_Index_Build_Result result) mutable
    {
      dispatch ([publish_generation, generation_token,
                 completion = std::move (completion),
                 result = std::move (result)] ( ) mutable
      {
        if( generation_token->load ( ) != publish_generation ) return;
        completion (std::move (result));
      });
    });
  return worker_generation != 0;
}

} // namespace robot_model
