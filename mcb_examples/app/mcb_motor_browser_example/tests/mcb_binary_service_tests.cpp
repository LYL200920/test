#include "mcb_binary_service.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using frame = std::array<uint8_t, 8>;

void require (bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error (message);
}

struct backend : mcb_binary::read_only_backend
{
  std::array<mcb_binary::axis_snapshot, 4> axes { };
  mutable unsigned int calls = 0;
  bool failed = false;

  bool read_axis (unsigned int index, mcb_binary::axis_snapshot& result) const override
  {
    ++calls;
    if (failed || index >= axes.size ())
      return false;
    result = axes[index];
    return true;
  }
};

frame request (uint32_t command, unsigned int axes, unsigned int gantry = 0)
{
  return { 0, static_cast<uint8_t> ((axes << 4) | ((command >> 8) & 15)),
           static_cast<uint8_t> ((gantry << 4) | (command & 15)), 0, 0, 0, 0, 0 };
}

frame take (mcb_binary::session& session, uint64_t now = 0)
{
  require (session.output_size () >= 8, "Missing response");
  frame response;
  std::memcpy (response.data (), session.output_data (), response.size ());
  session.consume_output (response.size (), now);
  return response;
}

frame invoke (mcb_binary::session& session, const backend& source, const frame& input)
{
  require (session.receive (input.data (), input.size (), 0), "Receive rejected");
  session.exec (source, 0);
  return take (session);
}

uint32_t value (const frame& response)
{
  return (uint32_t (response[4]) << 24) | (uint32_t (response[5]) << 16)
       | (uint32_t (response[6]) << 8) | response[7];
}

void test_queries ()
{
  backend source;
  mcb_binary::session session;
  session.reset (0);
  require (invoke (session, source, request (0x0301, 0)) == request (0x0301, 0), "Discovery frame");
  require (source.calls == 0, "Discovery read hardware");

  const std::array<int32_t, 4> positions { 0, -1, std::numeric_limits<int32_t>::min (),
                                          std::numeric_limits<int32_t>::max () };
  for (unsigned int axis = 0; axis < 4; ++axis)
  {
    source.axes[axis].logical_position = positions[axis];
    source.axes[axis].encoder_position = positions[3 - axis];
    source.axes[axis].speed_pps = axis == 3 ? 0x89ABCDEFU : 8000;
    for (uint32_t command : { 0x0306U, 0x0307U, 0x0507U })
    {
      auto response = invoke (session, source, request (command, 1U << axis));
      require (response[0] == 0 && response[1] == request (command, 1U << axis)[1]
               && response[2] == request (command, 1U << axis)[2] && response[3] == 0, "Query header");
      const uint32_t expected = command == 0x0306 ? uint32_t (positions[axis])
                              : command == 0x0307 ? uint32_t (positions[3 - axis])
                              : source.axes[axis].speed_pps;
      require (value (response) == expected, "Position/speed endian representation");
    }
  }

  source.axes[0].moving = true;
  source.axes[1].homing = true;
  source.axes[2].error = true;
  source.axes[2].positive_limit = true;
  source.axes[3].negative_limit = true;
  const std::array<uint8_t, 4> statuses { 0x20, 0x40, 0x1A, 0x11 };
  for (unsigned int mask = 0; mask < 16; ++mask)
  {
    uint8_t expected = 0;
    for (unsigned int axis = 0; axis < 4; ++axis)
      if (mask & (1U << axis))
        expected |= statuses[axis];
    auto response = invoke (session, source, request (0x0301, mask));
    require (response[3] == expected, "Status OR mapping");
    require ((response[3] & 0x84) == 0, "Fabricated home state");
    require (value (response) == 0, "Status reserved bytes");
  }
  source.failed = true;
  require (invoke (session, source, request (0x0306, 1)) == frame { 0xFE, 0, 0, 0, 0, 0, 0, 0 }, "Backend failure response");
}

void test_rejections ()
{
  backend source;
  mcb_binary::session session;
  session.reset (0);
  const frame invalid { 0xFF, 0, 0, 0, 0, 0, 0, 0 };
  for (unsigned int gantry = 1; gantry < 16; ++gantry)
    for (uint32_t command : { 0x0301U, 0x0306U, 0x0307U, 0x0507U })
      require (invoke (session, source, request (command, 1, gantry)) == invalid, "Invalid gantry accepted");
  for (unsigned int mask = 0; mask < 16; ++mask)
    if (mask == 0 || (mask & (mask - 1)) != 0)
      for (uint32_t command : { 0x0306U, 0x0307U, 0x0507U })
        require (invoke (session, source, request (command, mask)) == invalid, "Invalid numeric mask accepted");

  for (uint32_t command : { 0x0302U, 0x0303U, 0x0304U, 0x0305U, 0x0308U, 0x0309U,
                             0x030AU, 0x030BU, 0x0101U, 0x0502U, 0x0F0FU })
    for (unsigned int subcommand = 0; subcommand < 256; ++subcommand)
    {
      auto input = request (command, 15);
      input[3] = static_cast<uint8_t> (subcommand);
      require (invoke (session, source, input) == invalid, "Mutation/unknown subcommand accepted");
    }
  for (frame input : { frame { 1, 0x50, 0, 0, 0, 0, 0, 0 }, frame { 1, 0x51, 0, 0, 0, 0, 0, 0 },
                       frame { 0xFF, 0, 0, 0, 0, 0, 0, 0 } })
    require (invoke (session, source, input) == invalid, "Non-query command accepted");
  require (source.calls == 0, "Rejected request accessed backend");

  auto input = request (0x0306, 1);
  std::fill (input.begin () + 3, input.end (), 0xCC);
  require (invoke (session, source, input)[0] == 0, "Reserved request bytes rejected");
}

void test_framing ()
{
  backend source;
  const auto input = request (0x0306, 8);
  for (std::size_t split = 0; split <= input.size (); ++split)
  {
    mcb_binary::session session;
    session.reset (0);
    session.receive (input.data (), split, 0);
    session.exec (source, 0);
    if (split < input.size ())
      require (session.output_size () == 0, "Early fixed-frame response");
    session.receive (input.data () + split, input.size () - split, 1);
    session.exec (source, 1);
    require (take (session, 1)[1] == 0x83, "Split frame lost U mask");
  }
  for (std::size_t payload : { std::size_t (0), std::size_t (1), std::size_t (8), std::size_t (1024) })
  {
    std::vector<uint8_t> bytes { 0xEF, 0xAA, static_cast<uint8_t> (payload >> 8), static_cast<uint8_t> (payload) };
    for (std::size_t i = 0; i < payload; ++i)
      bytes.push_back (input[i % input.size ()]);
    for (std::size_t split = 0; split <= bytes.size (); ++split)
    {
      mcb_binary::session session;
      session.reset (0);
      const auto before = source.calls;
      session.receive (bytes.data (), split, 0);
      session.exec (source, 0);
      if (split < bytes.size ())
        require (session.output_size () == 0, "Early variable-frame response");
      session.receive (bytes.data () + split, bytes.size () - split, 1);
      session.receive (input.data (), input.size (), 1);
      session.exec (source, 1);
      require (take (session, 1)[0] == 0xFF, "Variable payload interpreted as commands");
      require (take (session, 1)[0] == 0, "Frame after variable payload lost alignment");
      require (source.calls == before + 1, "Variable payload accessed backend");
    }
  }
  mcb_binary::session session;
  session.reset (0);
  for (std::size_t index = 0; index < input.size (); ++index)
  {
    session.receive (&input[index], 1, index);
    session.exec (source, index);
    require (session.output_size () == (index == 7 ? 8U : 0U), "Byte-at-a-time framing");
  }
}

void test_bounds_and_timeouts ()
{
  backend source;
  mcb_binary::session session;
  session.reset (0);
  const auto input = request (0x0306, 1);
  std::vector<uint8_t> batch;
  for (unsigned int i = 0; i < 20; ++i)
    batch.insert (batch.end (), input.begin (), input.end ());
  session.receive (batch.data (), batch.size (), 0);
  session.exec (source, 0);
  require (source.calls == 4 && session.output_size () == 32, "Per-exec budget");
  for (unsigned int i = 0; i < 4; ++i)
    session.exec (source, 0);
  require (source.calls == 16 && session.output_size () == mcb_binary::session::tx_capacity, "TX bounds");
  const auto first_byte = session.output_data ()[0];
  session.consume_output (0, 1);
  require (session.output_size () == 128 && session.output_data ()[0] == first_byte, "Zero send consumed data");
  session.consume_output (3, 1);
  require (session.output_size () == 125, "Partial send offset");
  session.exec (source, 1);
  require (source.calls == 16, "Dispatch without response capacity");
  session.consume_output (5, 1);
  session.exec (source, 1);
  require (source.calls == 17 && session.output_size () == 128, "Queue did not recover");
  session.exec (source, 10001);
  require (session.should_close (), "TX stall deadline");

  session.reset (0);
  session.receive (input.data (), input.size (), 0);
  session.receive (input.data (), 1, 4999);
  session.exec (source, 5000);
  require (!session.should_close (), "Later partial frame inherited earlier request deadline");
  require (take (session, 5000)[0] == 0, "Queued complete request lost");
  session.exec (source, 9998);
  require (!session.should_close (), "Premature trailing frame deadline");
  session.exec (source, 9999);
  require (session.should_close (), "Trailing partial frame deadline");

  session.reset (0);
  session.receive (input.data (), 1, 0);
  require (!session.receive (input.data () + 1, 7, 5000), "Late completion bypassed frame deadline");
  session.reset (0);
  session.receive (input.data (), 1, 0);
  session.receive (input.data () + 1, 1, 4999);
  session.exec (source, 5000);
  require (session.should_close (), "Trickle refreshed frame deadline");
  require (!session.receive (input.data (), input.size (), 5001), "Closed receive accepted");
  session.reset (0);
  session.exec (source, 59999);
  require (!session.should_close (), "Premature idle timeout");
  session.exec (source, 60000);
  require (session.should_close (), "Idle deadline");

  session.reset (0);
  const std::array<uint8_t, 4> too_large { 0xE0, 0, 0xFF, 0xFF };
  session.receive (too_large.data (), too_large.size (), 0);
  session.exec (source, 0);
  require (session.should_close (), "Oversized frame accepted");
  session.reset (0);
  std::vector<uint8_t> overflow (mcb_binary::session::rx_capacity + 1);
  require (!session.receive (overflow.data (), overflow.size (), 0) && session.should_close (), "RX overflow");
  session.reset (1);
  require (session.receive_space () == mcb_binary::session::rx_capacity && session.output_size () == 0, "Reset retained data");
  require (invoke (session, source, input)[0] == 0, "Session reuse");
}
}

int main ()
{
  try
  {
    test_queries ();
    test_rejections ();
    test_framing ();
    test_bounds_and_timeouts ();
    std::cout << "PASS: queries, rejections, framing, bounds/timeouts\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what () << '\n';
    return 1;
  }
}
