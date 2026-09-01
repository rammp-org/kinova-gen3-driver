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

// What it reports back. NOTE the asymmetry with the command: MotorFeedback carries
// no force field, so `effort` is DERIVED from motor current and is a fraction of
// maximum, never Newtons. Publishing a number labelled in force units that is wrong
// by an unknown factor would be worse than publishing nothing.
struct GripperFeedback {
  float position = 0.0f;   // 0 (open) .. 1 (closed)
  float velocity = 0.0f;   // normalized; sign and scale TO CONFIRM on hardware
  float effort   = 0.0f;   // 0..1, derived from current
  float current  = 0.0f;   // amps, raw, exactly as reported
  // False when no interconnect gripper is attached. Without this, a missing gripper
  // and a fully-open one are both position 0 -- a silent mis-mapping.
  bool  present  = false;
};

}  // namespace kinova
