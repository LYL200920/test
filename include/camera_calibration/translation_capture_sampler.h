#ifndef TRANSLATION_CAPTURE_SAMPLER_H_
#define TRANSLATION_CAPTURE_SAMPLER_H_
#include "telecentric_translation_session.h"

namespace camera_calibration
{
enum class Translation_Sampling_State { Idle, Before, Trigger_Ready, Image, After, Complete, Failed };
// Effects (GET_STATE, camera trigger) belong to the caller. Register requests
// before draining buffered callbacks; synchronous backend replies are allowed.
class Translation_Capture_Sampler
{
public:
  void Begin(std::uint32_t motion, std::uint64_t minimum_revision,
    std::uint32_t request, std::uint64_t requested_ns);
  void Observe(const Translation_Pose_Snapshot &pose);
  bool Trigger(std::uint64_t now);
  bool Image(std::uint64_t frame, std::uint64_t observed_ns,
    std::uint32_t request, std::uint64_t requested_ns);
  void Tick(std::uint64_t now);
  void Reset();
  Translation_Sampling_State State() const { return state_; }
  const Translation_Capture_Evidence &Evidence() const { return evidence_; }
  const std::string &Error() const { return error_; }
private:
  void Fail(const std::string &message);
  Translation_Sampling_State state_ = Translation_Sampling_State::Idle;
  Translation_Capture_Evidence evidence_;
  std::uint32_t expected_ = 0;
  std::uint64_t requested_ns_ = 0, minimum_revision_ = 0;
  std::string error_;
};
// Endpoint consistency only: not proof of stillness throughout exposure.
bool Check_Translation_Capture_Stability(const Translation_Capture_Evidence &evidence,
  std::string *error = nullptr);
}
#endif
