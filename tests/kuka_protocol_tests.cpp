#include "kuka_protocol.h"
#include "kuka_connection_config.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void Require(bool condition, const char *message)
{
  if (!condition)
    throw std::runtime_error(message);
}
} // namespace

int main()
{
  try
  {
    kuka::Frame_Decoder decoder;
    std::vector<std::string> frames;
    std::string error;
    Require(decoder.Feed("V1,A,10,ACCEP", &frames, &error),
            "Partial frame was rejected");
    Require(frames.empty(), "Partial frame was emitted");
    Require(decoder.Feed("TED,0;V1,A,10,RUNNING,0;", &frames, &error),
            "Combined frames were rejected");
    Require(frames.size() == 2, "Combined frames were not split");

    kuka::Protocol_Message message;
    Require(kuka::Decode_Message(frames[0], &message, &error),
            "Acknowledgement decode failed");
    kuka::Acknowledgement acknowledgement;
    Require(kuka::Parse_Acknowledgement(message, &acknowledgement, &error),
            "Acknowledgement parse failed");
    Require(acknowledgement.sequence == 10,
            "Acknowledgement sequence mismatch");
    Require(acknowledgement.state == kuka::Acknowledgement_State::Accepted,
            "Acknowledgement state mismatch");

    kuka::Axis axis{0.0, -90.0, 90.0, 0.0, 45.0, 0.0};
    const std::string move_joint = kuka::Encode_Move_Joint(42, axis);
    Require(move_joint == "V1,C,42,MOVEJ,20,50,0,0,-90,90,0,45,0;",
            "MOVEJ encoding mismatch");

    kuka::Pose pose{520.0, 10.0, 630.0, 180.0, 0.0, 90.0};
    kuka::Cartesian_Motion_Options linear_options;
    linear_options.velocity = 150.0;
    linear_options.base = 2;
    linear_options.tool = 3;
    const std::string move_linear =
        kuka::Encode_Move_Linear(43, pose, linear_options);
    Require(move_linear ==
                "V1,C,43,MOVEL,150,50,0,2,3,520,10,630,180,0,90,NEAR,0,0;",
            "MOVEL encoding mismatch");

    kuka::Cartesian_Motion_Options ptp_options = linear_options;
    ptp_options.velocity = 20.0;
    const std::string guided_move_ptp =
        kuka::Encode_Move_Pose_Ptp(44, pose, axis, ptp_options);
    Require(guided_move_ptp ==
                "V1,C,44,MOVEPTP,20,50,0,2,3,"
                "520,10,630,180,0,90,NEAR,0,0,"
                "AXIS,0,-90,90,0,45,0;",
            "Axis-guided MOVEPTP encoding mismatch");

    Require(kuka::Decode_Message(
                "V1,S,45,IDLE,0,0,-90,90,0,45,0,"
                "520,10,630,180,0,90,2,3,80",
                &message, &error),
            "Robot state decode failed");
    kuka::Robot_State state;
    Require(kuka::Parse_Robot_State(message, &state, &error),
            "Robot state parse failed");
    Require(state.axis[1] == -90.0 && state.pose[2] == 630.0,
            "Robot state values mismatch");
    Require(state.base == 2 && state.tool == 3 &&
                state.override_percent == 80.0,
            "Robot state metadata mismatch");

    kuka::Joint_Motion_Options invalid_options;
    invalid_options.velocity_percent = 0.0;
    Require(!kuka::Validate_Joint_Motion(axis, invalid_options, &error),
            "Invalid velocity was accepted");

    kuka::Frame_Decoder small_decoder(8);
    frames.clear();
    Require(!small_decoder.Feed("123456789", &frames, &error),
            "Oversized unterminated frame was accepted");

    const auto config_test_path =
        std::filesystem::temp_directory_path() /
        ("kuka_connection_" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()) +
         ".ini");
    kuka::Connection_Config saved_config;
    saved_config.host = "10.20.30.40";
    saved_config.port = 12345;
    saved_config.model_id = "test_model";
    Require(
        kuka::Save_Connection_Config(
            config_test_path, saved_config, &error),
        "Connection configuration save failed");
    kuka::Connection_Config loaded_config;
    Require(
        kuka::Load_Connection_Config(
            config_test_path, &loaded_config, &error),
        "Connection configuration load failed");
    Require(
        loaded_config.host == saved_config.host &&
            loaded_config.port == saved_config.port &&
            loaded_config.model_id == saved_config.model_id,
        "Connection configuration round-trip mismatch");
    std::error_code remove_error;
    std::filesystem::remove(config_test_path, remove_error);
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }

  std::cout << "KUKA protocol tests passed.\n";
  return 0;
}
