#ifndef includeguard_mcb_axis_diagnostics_hpp
#define includeguard_mcb_axis_diagnostics_hpp

#include "mcb_binary_service.hpp"

#include <string_view>

namespace mcb_binary
{

struct axis_error_sources
{
  bool axis_error = false;
  bool alarm_input = false;
  bool home_error = false;
  bool interpolation_error = false;
  bool emergency_input = false;
  bool emergency_stop = false;
  bool alarm_stop = false;

  bool any (void) const;
};

axis_error_sources error_sources (unsigned int index, const axis_diagnostics& registers);
bool apply_diagnostics (unsigned int index, axis_snapshot& snapshot);
uint8_t axis_status (const axis_snapshot& snapshot);

struct diagnostic_response
{
  unsigned int status_code;
  std::string_view reason;
  std::string_view body;
};

// Copies the serialized body into caller-owned storage. No partial JSON is
// returned on overflow; body must be copied before that storage is reused.
diagnostic_response diagnose_axis (std::string_view method, std::string_view query,
                                   const read_only_backend* backend,
                                   char* buffer, std::size_t capacity);

} // namespace mcb_binary
#endif
