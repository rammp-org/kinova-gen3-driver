#pragma once
namespace kinova {

// What the 2F-85 accepts, normalized. KORTEX speaks percent (0..100); the single
// conversion happens inside KortexTransport, exactly as the arm's degrees do.
struct GripperCommand {
  float position = 0.0f;   // 0 (open) .. 1 (closed)
  float speed    = 1.0f;   // fraction of max closing speed
  // A CEILING on motor current, not a force setpoint. The gripper closes at
  // `speed` toward `position` and stalls when it reaches this limit. No force
  // servo exists on this hardware -- GripperMode has no force mode at all, and
  // the high-level API that would host one needs SINGLE_LEVEL_SERVOING, which is
  // incompatible with the low-level servoing our 1 kHz torque control requires.
  float force    = 0.5f;   // fraction of max grip force
  // When false, no gripper command is emitted at all. This keeps the gripper limp
  // at startup rather than actuating it from a seeded default.
  bool  active   = false;
};

// Normalizer for GripperFeedback::effort/current. MotorFeedback carries NO force field --
// only current_motor -- so effort is |current| / this, a fraction of maximum rather than a
// force in Newtons.
//
// PROVENANCE: MEASURED on the arm (gripper_check, force=1.0, closing on a compliant
// object). Peak grip current was 1.00 A. The previous value of 0.8 A came from the
// datasheet's rated stall current and was WRONG in the direction that hides the error:
// at 0.8 the two highest samples computed 1.20 and 1.25 and were clamped to exactly
// 1.0000, so effort silently saturated during an ordinary grasp.
//
// Read as a floor rather than a ceiling: the trace sampled at 50 Hz and the squeeze is
// transient, so the true peak may be slightly higher. Re-measure if grasps start
// clamping again.
inline constexpr float kGripperMaxCurrentA = 1.0f;

// What it reports back. NOTE the asymmetry with the command: MotorFeedback carries
// no force field, so `effort` is DERIVED from motor current and is a fraction of
// maximum, never Newtons. Publishing a number labelled in force units that is wrong
// by an unknown factor would be worse than publishing nothing.
// There is deliberately NO velocity field. MotorFeedback has one, but it was measured
// on the arm to be the COMMANDED speed echoed back while the gripper considers itself
// moving, and 0 otherwise -- unsigned, and identical opening and closing. While the
// fingers were being physically stopped by an object the position increments shrank
// while that field held exactly the commanded value. It therefore carries no information
// the caller does not already have, and carrying it would invite exactly one mistake:
// publishing a setpoint echo into a units-bearing field such as sensor_msgs/JointState's
// `velocity`. Differentiate `position` if a rate is genuinely needed.
struct GripperFeedback {
  float position = 0.0f;   // 0 (open) .. 1 (closed)
  // 0..1, |current| / kGripperMaxCurrentA. Measured on the arm: a grasp SPIKES
  // (up to 1.0) while the fingers close on the object, then settles to a low
  // holding current -- about 0.05 A, i.e. effort ~0.05. A sustained grasp therefore
  // reports a SMALL effort, not a large one; anything keying off "high effort means
  // holding something" will be wrong.
  float effort   = 0.0f;
  float current  = 0.0f;   // amps, raw, exactly as reported
  // False when no interconnect gripper is attached. Without this, a missing gripper
  // and a fully-open one are both position 0 -- a silent mis-mapping.
  bool  present  = false;
};

}  // namespace kinova
