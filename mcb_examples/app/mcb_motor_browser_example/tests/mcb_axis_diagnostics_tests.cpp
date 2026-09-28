#include "mcb_axis_diagnostics.hpp"

#include <array>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
void require (bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error (message);
}

class fake_backend : public mcb_binary::read_only_backend
{
public:
  mutable unsigned int calls = 0;
  mutable unsigned int last_axis = 99;
  bool fail = false;
  mcb_binary::axis_snapshot sample;

  bool read_axis (unsigned int index, mcb_binary::axis_snapshot& result) const override
  {
    ++calls;
    last_axis = index;
    result = sample;
    return !fail;
  }
};

void test_sources (void)
{
  for (unsigned int index = 0; index < 4; ++index)
  {
    for (unsigned int combination = 0; combination < 128; ++combination)
    {
      mcb_binary::axis_snapshot sample;
      auto& registers = sample.diagnostics;
      registers.available = true;
      if (combination & 1) registers.rr0_raw |= 1U << (index + 4);
      if (combination & 2) registers.rr3_decoded |= 1U << 6;
      if (combination & 4) registers.rr2_raw |= 1U << 6;
      if (combination & 8) registers.rr2_raw |= 1U << 7;
      if (combination & 16) registers.rr2_raw |= 1U << 5;
      if (combination & 32) registers.rr2_raw |= 1U << 15;
      if (combination & 64) registers.rr2_raw |= 1U << 14;
      const auto sources = mcb_binary::error_sources (index, registers);
      require (sources.axis_error == bool (combination & 1), "RR0 axis error mapping");
      require (sources.alarm_input == bool (combination & 2), "Decoded RR3 alarm mapping");
      require (sources.home_error == bool (combination & 4), "RR2 home error mapping");
      require (sources.interpolation_error == bool (combination & 8), "RR2 interpolation mapping");
      require (sources.emergency_input == bool (combination & 16), "RR2 emergency mapping");
      require (sources.emergency_stop == bool (combination & 32), "RR2 emergency stop mapping");
      require (sources.alarm_stop == bool (combination & 64), "RR2 alarm stop mapping");
      require (sources.any () == (combination != 0), "Exact seven-source OR");
      require (mcb_binary::apply_diagnostics (index, sample), "Apply valid sample");
      require (sample.error == sources.any (), "Summary agrees with sources");
      require (mcb_binary::axis_status (sample) == (combination ? 0x18 : 0x10), "Stopped/error status");
    }
    // Unrelated axis errors and RR2 alarm/limit flags must not broaden the old OR.
    mcb_binary::axis_snapshot sample;
    sample.diagnostics = { true, uint16_t (0xF0 & ~(1U << (index + 4))), 0x3F1F, 0 };
    require (mcb_binary::apply_diagnostics (index, sample), "Read RR2 alarm only");
    require (!sample.error, "RR2 alarm itself not an extra aggregate source");
    require (!sample.positive_limit && !sample.negative_limit, "Latched limits not live inputs");
    sample.diagnostics.rr3_decoded = 0x0180;
    mcb_binary::apply_diagnostics (index, sample);
    require (!sample.error && sample.positive_limit && sample.negative_limit, "Live RR3 limit inputs");
  }
  mcb_binary::axis_snapshot unavailable;
  unavailable.error = true;
  require (!mcb_binary::apply_diagnostics (0, unavailable), "Unavailable sample rejected");
  require (unavailable.error, "Unavailable sample not overwritten with healthy zeros");
  unavailable.diagnostics.available = true;
  require (!mcb_binary::apply_diagnostics (4, unavailable), "Invalid index rejected");
  require (!mcb_binary::error_sources (4, unavailable.diagnostics).any (), "Invalid source index guarded");
}

void test_status_and_wire (void)
{
  for (unsigned int index = 0; index < 4; ++index)
  {
    for (unsigned int flags = 0; flags < 32; ++flags)
    {
      fake_backend backend;
      auto& sample = backend.sample;
      sample.diagnostics = { true, uint16_t ((flags & 1) ? 1U << index : 0),
                            uint16_t ((flags & 4) ? 1U << 15 : 0),
                            uint16_t (((flags & 2) ? 1U << 9 : 0)
                                    | ((flags & 8) ? 1U << 7 : 0)
                                    | ((flags & 16) ? 1U << 8 : 0)) };
      mcb_binary::apply_diagnostics (index, sample);
      const unsigned int expected = ((flags & 1) ? 0x20 : 0)
                                  | ((flags & 2) ? 0x40 : 0)
                                  | ((flags & 3) ? 0 : 0x10)
                                  | ((flags & 4) ? 0x08 : 0)
                                  | ((flags & 8) ? 0x02 : 0)
                                  | ((flags & 16) ? 0x01 : 0);
      require (mcb_binary::axis_status (sample) == expected, "Status bit semantics preserved");
      mcb_binary::session session;
      session.reset (0);
      const std::array<uint8_t, 8> request = { 0, uint8_t ((1U << (index + 4)) | 3), 1, 0, 0, 0, 0, 0 };
      require (session.receive (request.data (), request.size (), 0), "Receive status request");
      session.exec (backend, 0);
      require (session.output_size () == 8 && session.output_data ()[0] == 0
               && session.output_data ()[3] == expected, "Wire matches diagnostic status");
    }
    for (unsigned int state = 0; state < 64; ++state)
    {
      mcb_binary::axis_snapshot sample;
      sample.diagnostics = { true, 0, 0, uint16_t (state << 9) };
      mcb_binary::apply_diagnostics (index, sample);
      require (sample.homing == (state != 0), "All home search states mapped; zero is idle");
    }
  }
}

void test_validation (void)
{
  fake_backend backend;
  char buffer[2048];
  for (const auto query : { "", "axis", "axis=", "axis=-1", "axis=4", "axis=00", "axis=1x",
                            "axis=%30", "axis=0&axis=1", "axis=0&x=1", "x=1&axis=0", "axis=0&",
                            "Axis=0", "?axis=0", "axis= 0" })
    require (mcb_binary::diagnose_axis ("GET", query, &backend, buffer, sizeof (buffer)).status_code == 400,
             "Bad queries return 400");
  for (const auto method : { "POST", "PUT", "DELETE", "HEAD", "get", "" })
    require (mcb_binary::diagnose_axis (method, "axis=0", &backend, buffer, sizeof (buffer)).status_code == 405,
             "Non-GET returns 405");
  require (backend.calls == 0, "Invalid requests never query backend");
  auto response = mcb_binary::diagnose_axis ("GET", "axis=0", nullptr, buffer, sizeof (buffer));
  require (response.status_code == 503 && response.body == "{\"error\":\"diagnostics_unsupported\"}", "Unsupported backend");
  backend.fail = true;
  response = mcb_binary::diagnose_axis ("GET", "axis=0", &backend, buffer, sizeof (buffer));
  require (response.status_code == 503 && response.body == "{\"error\":\"axis_read_failed\"}", "Read failure");
  backend.fail = false;
  response = mcb_binary::diagnose_axis ("GET", "axis=0", &backend, buffer, sizeof (buffer));
  require (response.status_code == 503 && response.body == "{\"error\":\"diagnostics_unavailable\"}", "No invented zero registers");
  backend.sample.diagnostics = { true, 0x10, 0, 0 };
  response = mcb_binary::diagnose_axis ("GET", "axis=0", &backend, buffer, sizeof (buffer));
  require (response.status_code == 503 && response.body == "{\"error\":\"inconsistent_snapshot\"}", "Inconsistent aggregate rejected");
}

void test_serialization (bool print_fixtures)
{
  char buffer[2048];
  for (unsigned int index = 0; index < 4; ++index)
  {
    fake_backend backend;
    backend.sample.logical_position = (index & 1) ? std::numeric_limits<int32_t>::max () : std::numeric_limits<int32_t>::min ();
    backend.sample.encoder_position = (index & 1) ? std::numeric_limits<int32_t>::min () : std::numeric_limits<int32_t>::max ();
    backend.sample.speed_pps = std::numeric_limits<uint32_t>::max ();
    const std::array<mcb_binary::axis_diagnostics, 4> registers = {{
      { true, 0xFFF0, 0, 0x8000 },       // Stopped + axis error: the observed 0x18 case.
      { true, 0xFFFF, 0xFFFF, 0xFFFF },  // Maximum registers, all defined flags set.
      { true, 0xFF00, 0x3F1F, 0x8000 },  // RR2 alarm/stop flags without aggregate error.
      { true, 0x0008, 0x8040, 0x0189 }   // Mixed sources and live inputs.
    }};
    backend.sample.diagnostics = registers[index];
    mcb_binary::apply_diagnostics (index, backend.sample);
    const std::string query = "axis=" + std::to_string (index);
    const auto response = mcb_binary::diagnose_axis ("GET", query, &backend, buffer, sizeof (buffer));
    require (response.status_code == 200 && backend.calls == 1 && backend.last_axis == index, "Exactly one requested axis read");
    require (response.body.front () == '{' && response.body.back () == '}', "Full bounded JSON");
    require (response.body.size () < sizeof (buffer) - 1, "Worst-size samples fit 2 KiB");
    require (response.body.find ("\"home\":null,\"homing_ok\":null") != std::string_view::npos, "Home completion unknown");
    require (response.body.find ("rr3_raw") == std::string_view::npos, "Normalized RR3 not mislabeled raw");
    if (print_fixtures)
      std::cout << response.body << '\n';

    const std::string saved (response.body);
    std::array<char, 2050> guarded;
    guarded.fill ('#');
    auto small = mcb_binary::diagnose_axis ("GET", query, &backend, guarded.data () + 1, saved.size ());
    require (small.status_code == 500 && small.body == "{\"error\":\"diagnostic_capacity\"}", "Truncated JSON never returned");
    require (guarded[0] == '#' && guarded[saved.size () + 1] == '#' && guarded[1] == '\0', "Truncation bounds and scratch invalidation");
    auto exact = mcb_binary::diagnose_axis ("GET", query, &backend, guarded.data () + 1, saved.size () + 1);
    require (exact.status_code == 200 && exact.body == saved, "Exact capacity including terminator");
    require (guarded[saved.size () + 2] == '#', "Exact output respects bound");
    require (mcb_binary::diagnose_axis ("GET", query, &backend, nullptr, 2048).status_code == 500, "Null buffer rejected");
    require (mcb_binary::diagnose_axis ("GET", query, &backend, buffer, 0).status_code == 500, "Zero capacity rejected");
    require (mcb_binary::diagnose_axis ("GET", query, &backend, buffer, 1).status_code == 500 && buffer[0] == '\0', "One-byte capacity handled");
    backend.sample = { };
    backend.sample.diagnostics.available = true;
    auto reused = mcb_binary::diagnose_axis ("GET", query, &backend, buffer, sizeof (buffer));
    require (reused.status_code == 200 && reused.body.find ("\"speed_pps\":0,") != std::string_view::npos,
             "Scratch reuse has no stale values");
  }
}
}

int main (int argc, char** argv)
{
  try
  {
    const bool fixtures = argc == 2 && std::strcmp (argv[1], "--json-fixtures") == 0;
    test_sources ();
    test_status_and_wire ();
    test_validation ();
    test_serialization (fixtures);
    if (!fixtures)
      std::cout << "MCB axis diagnostics tests passed\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what () << '\n';
    return 1;
  }
}
