#include "kinova_lowlevel/sim_transport.h"

#include <time.h>

#include <array>
#include <cmath>

#include "kinova_lowlevel/units.h"  // wrap_to_pi

namespace kinova {

namespace {
inline int64_t ns_now() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}
constexpr float kSettledEps = 1e-6f;
}  // namespace

SimTransport::SimTransport(const JointFeedback& initial, int latency_us)
    : state_(initial), latency_us_(latency_us) {
  // The simulated robot always has a gripper, so say so from the start rather
  // than only once step_gripper() has run. receive() copies state_ without
  // stepping, so without this a caller that reads feedback BEFORE issuing any
  // command sees present=false -- indistinguishable from a real arm with no
  // interconnect gripper attached, which is exactly the mis-mapping `present`
  // was added to kill.
  state_.gripper.present = true;
}

void SimTransport::connect() {}
void SimTransport::set_servoing_low_level() {}
void SimTransport::set_actuator_modes(const ActuatorModes&) {}
void SimTransport::safe_shutdown() {}

// Advance the simulated gripper one cycle toward its target. Extracted because
// exchange() and send() must step it identically -- they already duplicate the
// command latch, and a divergence here would make send-driven tests lie.
void SimTransport::step_gripper(const GripperCommand& g) {
  state_.gripper.present = true;
  if (!g.active) return;

  float target = g.position;
  if (gripper_block_ >= 0.0f && target > gripper_block_) target = gripper_block_;

  const float before = state_.gripper.position;
  state_.gripper.position += (target - before) * gripper_lag_;
  // An object is a hard stop, not an asymptote: once we're within kSettledEps of the
  // block that IS the effective target, snap onto it exactly rather than let the
  // effort gate below depend on a float residual that -ffp-contract=fast (FMA fusion)
  // can shift across build flags/toolchains/architectures.
  if (gripper_block_ >= 0.0f && target == gripper_block_ &&
      std::fabs(state_.gripper.position - gripper_block_) < kSettledEps) {
    state_.gripper.position = gripper_block_;
  }
  // Loaded only when an object is what stopped us -- i.e. we are held at the block
  // while the caller is still asking for more. Reaching a freely-commanded target is
  // not a grasp and must not report effort, or every close looks like a grasp.
  // THIS IS A MODEL, not measured hardware behaviour: on the real arm a sustained
  // grasp settles to a low holding current (effort ~0.05), not the commanded force
  // cap reported here. Do not calibrate a holding-detector against this sim.
  const bool blocked_by_object = gripper_block_ >= 0.0f && g.position > gripper_block_ &&
                                 std::fabs(state_.gripper.position - gripper_block_) < kSettledEps;
  state_.gripper.effort = blocked_by_object ? g.force : 0.0f;
  // Use the real normalizer so current/kGripperMaxCurrentA round-trips to the same
  // effort on sim and hardware -- see GripperFeedback::current's documented units.
  state_.gripper.current = state_.gripper.effort * kGripperMaxCurrentA;
}

// Report angles the way the REAL transport does: wrapped into (-pi, pi], all seven
// joints unconditionally, mirroring kortex_transport.cpp. Without this the sim and
// hardware paths disagree about the representation every consumer downstream has to
// honour -- and since kortex_transport.cpp is only compiled with KINOVA_ENABLE_KORTEX,
// the whole suite would run in a representation where a continuous joint's wrap can
// never occur. That is how #52 reached the arm: the bug was structurally unreachable
// from CI. Kinematically a no-op for the bounded joints, whose limits are inside +/-pi.
static void report_wrapped(const JointFeedback& state, JointFeedback& fb) {
  fb = state;
  for (int i = 0; i < kNumJoints; ++i) fb.q[i] = wrap_to_pi(fb.q[i]);
}

void SimTransport::exchange(const JointCommand& cmd, JointFeedback& fb) {
  last_cmd_ = cmd;
  step_gripper(cmd.gripper);
  if (latency_us_ > 0) {
    const int64_t deadline = ns_now() + int64_t(latency_us_) * 1000LL;
    while (ns_now() < deadline) { /* busy-wait, off-RT friendly */
    }
  }
  ++frame_;
  state_.frame_id = frame_;
  report_wrapped(state_, fb);
}

void SimTransport::send(const JointCommand& cmd) {
  last_cmd_ = cmd;
  step_gripper(cmd.gripper);
  ++frame_;
  state_.frame_id = frame_;
}

void SimTransport::receive(JointFeedback& fb) { report_wrapped(state_, fb); }

}  // namespace kinova
