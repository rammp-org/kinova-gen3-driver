#pragma once
#include <string>

#include "kinova_lowlevel/interface/value_types.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
namespace kinova::interface {

// Validation bounds. Rejection posture throughout: a value outside these is
// REFUSED with a reason, never clamped -- clamping is how #64 dropped the arm.
constexpr double kKqMin = 1.0;     // N*m/rad; below this it is not a spring
constexpr double kKqMax = 400.0;   // 5x the default shoulder stiffness
constexpr double kZetaMin = 0.05;  // an (almost) undamped spring oscillates
constexpr double kZetaMax = 2.0;
// Gravity headroom per joint: the impedance clamp applies to the TOTAL torque,
// gravity included, so a limit below worst-case |g_i(q)| lets the clamp eat
// gravity and the arm falls (#64). Pinned constants; the GainsFloor test
// re-derives the worst case from the URDF and fails loudly if these go stale.
extern const JointVec kTorqueLimitFloor;
extern const JointVec kTorqueLimitCeil;  // URDF effort limits

struct GainsCheck {
  bool ok = false;
  std::string message;
};
// Bounds-check raw (kCustom) gains. Pure; callable from any thread.
GainsCheck validate_custom(const JointGainValues& g);
// True iff p is one of the five enumerators. An ImpedanceGains arrives as a raw
// byte from the C++ API (the ROS boundary filters its own); an out-of-enum
// value must die at ACCEPT, because resolve_gains on it throws on the
// sampler thread, where nothing catches.
bool known_profile(GainsProfile p);
// The complete parameter set a NAMED profile stands for. kCustom and
// kSessionDefault are not names -- std::invalid_argument, fail loud.
kinova::JointImpedanceParams profile_params(GainsProfile p);
// What a command's spec means, given the current session default:
//   kSessionDefault -> session_default verbatim
//   named profile   -> profile_params(p)
//   kCustom         -> session_default with kq/zeta/torque_limit overridden
//                      (validate_custom MUST have accepted it upstream)
kinova::JointImpedanceParams resolve_gains(const ImpedanceGains& s,
                                           const kinova::JointImpedanceParams& session_default);
}  // namespace kinova::interface
