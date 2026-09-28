#include "mcb_axis_diagnostics.hpp"

#include <cinttypes>
#include <cstdio>

namespace mcb_binary
{
namespace
{
bool bit (uint16_t value, unsigned int index)
{
  return (value & (1U << index)) != 0;
}

const char* boolean (bool value) { return value ? "true" : "false"; }

diagnostic_response failure (unsigned int code, std::string_view reason, std::string_view body)
{
  return { code, reason, body };
}
}

bool axis_error_sources::any (void) const
{
  return axis_error || alarm_input || home_error || interpolation_error
      || emergency_input || emergency_stop || alarm_stop;
}

axis_error_sources error_sources (unsigned int index, const axis_diagnostics& registers)
{
  if (index >= 4 || !registers.available)
    return { };
  return { bit (registers.rr0_raw, index + 4), bit (registers.rr3_decoded, 6),
           bit (registers.rr2_raw, 6), bit (registers.rr2_raw, 7),
           bit (registers.rr2_raw, 5), bit (registers.rr2_raw, 15), bit (registers.rr2_raw, 14) };
}

bool apply_diagnostics (unsigned int index, axis_snapshot& snapshot)
{
  if (index >= 4 || !snapshot.diagnostics.available)
    return false;
  const auto& registers = snapshot.diagnostics;
  snapshot.moving = bit (registers.rr0_raw, index);
  snapshot.homing = ((registers.rr3_decoded >> 9) & 63) != 0;
  snapshot.error = error_sources (index, registers).any ();
  snapshot.positive_limit = bit (registers.rr3_decoded, 7);
  snapshot.negative_limit = bit (registers.rr3_decoded, 8);
  return true;
}

uint8_t axis_status (const axis_snapshot& snapshot)
{
  return (snapshot.homing ? 0x40 : 0) | (snapshot.moving ? 0x20 : 0)
       | (!snapshot.moving && !snapshot.homing ? 0x10 : 0) | (snapshot.error ? 0x08 : 0)
       | (snapshot.positive_limit ? 0x02 : 0) | (snapshot.negative_limit ? 0x01 : 0);
}

diagnostic_response diagnose_axis (std::string_view method, std::string_view query,
                                   const read_only_backend* backend,
                                   char* buffer, std::size_t capacity)
{
  if (method != "GET")
    return failure (405, "Method Not Allowed", "{\"error\":\"method_not_allowed\"}");
  // This endpoint deliberately accepts one canonical parameter only. Do not
  // accept ambiguous duplicates, trailing data, or encoded control arguments.
  if (query.size () != 6 || query.substr (0, 5) != "axis=" || query[5] < '0' || query[5] > '3')
    return failure (400, "Bad Request", "{\"error\":\"invalid_axis_query\"}");
  if (backend == nullptr)
    return failure (503, "Service Unavailable", "{\"error\":\"diagnostics_unsupported\"}");

  const unsigned int index = query[5] - '0';
  axis_snapshot snapshot;
  if (!backend->read_axis (index, snapshot))
    return failure (503, "Service Unavailable", "{\"error\":\"axis_read_failed\"}");
  if (!snapshot.diagnostics.available)
    return failure (503, "Service Unavailable", "{\"error\":\"diagnostics_unavailable\"}");
  // An inconsistent backend must not report a different fault than TCP does.
  const auto sources = error_sources (index, snapshot.diagnostics);
  if (snapshot.error != sources.any ())
    return failure (503, "Service Unavailable", "{\"error\":\"inconsistent_snapshot\"}");
  if (buffer == nullptr || capacity == 0)
    return failure (500, "Internal Server Error", "{\"error\":\"diagnostic_capacity\"}");

  const auto rr2 = snapshot.diagnostics.rr2_raw;
  const auto rr3 = snapshot.diagnostics.rr3_decoded;
  const int size = std::snprintf (buffer, capacity,
      "{\"schema_version\":1,\"axis_index\":%u,\"axis_name\":\"%c\",\"axis_mask\":%u,"
      "\"binary_status\":%u,\"logical_position\":%" PRId32 ",\"encoder_position\":%" PRId32 ","
      "\"speed_pps\":%" PRIu32 ",\"moving\":%s,\"homing\":%s,\"error\":%s,"
      "\"positive_limit\":%s,\"negative_limit\":%s,\"home\":null,\"homing_ok\":null,"
      "\"rr0_raw\":%u,\"rr2_raw\":%u,\"rr3_decoded\":%u,"
      "\"rr3_interpretation\":\"driver_polarity_and_swap_normalized\",\"home_search_state\":%u,"
      "\"error_sources\":{\"axis_error\":%s,\"alarm_input\":%s,\"home_error\":%s,"
      "\"interpolation_error\":%s,\"emergency_input\":%s,\"emergency_stop\":%s,\"alarm_stop\":%s},"
      "\"drive_flags\":{\"alarm\":%s,\"sw_limit_positive\":%s,\"sw_limit_negative\":%s,"
      "\"hw_limit_positive\":%s,\"hw_limit_negative\":%s,\"sync_stop\":%s,"
      "\"stop0_stop\":%s,\"stop1_stop\":%s,\"stop2_stop\":%s,"
      "\"limit_positive_stop\":%s,\"limit_negative_stop\":%s},"
      "\"signal_inputs\":{\"stop0\":%s,\"stop1\":%s,\"stop2\":%s,"
      "\"encoder_a\":%s,\"encoder_b\":%s,\"in_position\":%s}}",
      index, "XYZU"[index], 1U << index, unsigned (axis_status (snapshot)),
      snapshot.logical_position, snapshot.encoder_position, snapshot.speed_pps,
      boolean (snapshot.moving), boolean (snapshot.homing), boolean (snapshot.error),
      boolean (snapshot.positive_limit), boolean (snapshot.negative_limit),
      unsigned (snapshot.diagnostics.rr0_raw), unsigned (rr2), unsigned (rr3), unsigned ((rr3 >> 9) & 63),
      boolean (sources.axis_error), boolean (sources.alarm_input), boolean (sources.home_error),
      boolean (sources.interpolation_error), boolean (sources.emergency_input),
      boolean (sources.emergency_stop), boolean (sources.alarm_stop),
      boolean (bit (rr2, 4)), boolean (bit (rr2, 0)), boolean (bit (rr2, 1)),
      boolean (bit (rr2, 2)), boolean (bit (rr2, 3)), boolean (bit (rr2, 8)),
      boolean (bit (rr2, 9)), boolean (bit (rr2, 10)), boolean (bit (rr2, 11)),
      boolean (bit (rr2, 12)), boolean (bit (rr2, 13)),
      boolean (bit (rr3, 0)), boolean (bit (rr3, 1)), boolean (bit (rr3, 2)),
      boolean (bit (rr3, 3)), boolean (bit (rr3, 4)), boolean (bit (rr3, 5)));
  if (size < 0 || std::size_t (size) >= capacity)
  {
    buffer[0] = '\0';
    return failure (500, "Internal Server Error", "{\"error\":\"diagnostic_capacity\"}");
  }
  return { 200, "OK", std::string_view (buffer, std::size_t (size)) };
}

} // namespace mcb_binary
