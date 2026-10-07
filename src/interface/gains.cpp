#include "kinova_lowlevel/interface/gains.h"

#include <cmath>
#include <stdexcept>
namespace kinova::interface {

// Pinned from the GainsFloor test's measured worst-case gravity on the 2F-85
// model: 1.05x margin rounded up to one decimal, with a 0.5 N*m nominal floor
// for the joints whose gravity torque is ~0 (axis near vertical). The margin
// is deliberately thin at joint 1 -- worst-case |g| there is 35.7 N*m against
// a 39 N*m URDF effort ceiling, so there is no room for a fat one; the floor's
// job is "the arm cannot fall", not "generous control authority".
const JointVec kTorqueLimitFloor = (JointVec() << 0.5, 37.6, 14.5, 14.5, 3.7, 3.7, 0.5).finished();
const JointVec kTorqueLimitCeil = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();

GainsCheck validate_custom(const JointGainValues& g) {
  auto fail = [](const std::string& m) { return GainsCheck{false, m}; };
  if (!std::isfinite(g.zeta) || g.zeta < kZetaMin || g.zeta > kZetaMax)
    return fail("zeta must be finite and in [" + std::to_string(kZetaMin) + ", " +
                std::to_string(kZetaMax) + "]");
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(g.kq[i]) || g.kq[i] < kKqMin || g.kq[i] > kKqMax)
      return fail("kq[" + std::to_string(i) + "] must be finite and in [" +
                  std::to_string(kKqMin) + ", " + std::to_string(kKqMax) + "]");
    if (!std::isfinite(g.torque_limit[i]) || g.torque_limit[i] < kTorqueLimitFloor[i] ||
        g.torque_limit[i] > kTorqueLimitCeil[i])
      return fail("torque_limit[" + std::to_string(i) + "] must be finite and in [" +
                  std::to_string(kTorqueLimitFloor[i]) + ", " +
                  std::to_string(kTorqueLimitCeil[i]) +
                  "]: below the floor the clamp eats gravity and the arm falls");
  }
  return {true, ""};
}

kinova::JointImpedanceParams profile_params(GainsProfile p) {
  kinova::JointImpedanceParams jp;  // the defaults ARE the medium tuning
  switch (p) {
    case GainsProfile::kMedium:
      return jp;
    case GainsProfile::kSoft:
      jp.Kq = (JointVec() << 40, 40, 40, 40, 15, 15, 15).finished();
      jp.zeta = 0.4;
      jp.max_tracking_error = 0.45;  // softer spring, longer leash
      return jp;
    case GainsProfile::kStiff:
      jp.Kq = (JointVec() << 160, 160, 160, 160, 60, 60, 60).finished();
      jp.zeta = 0.7;
      jp.max_tracking_error = 0.25;  // stiffer spring, tighter leash
      return jp;
    case GainsProfile::kCustom:
    case GainsProfile::kSessionDefault:
      break;
  }
  throw std::invalid_argument("profile_params: not a named profile");
}

bool known_profile(GainsProfile p) {
  switch (p) {
    case GainsProfile::kSessionDefault:
    case GainsProfile::kSoft:
    case GainsProfile::kMedium:
    case GainsProfile::kStiff:
    case GainsProfile::kCustom:
      return true;
  }
  return false;
}

kinova::JointImpedanceParams resolve_gains(const ImpedanceGains& s,
                                           const kinova::JointImpedanceParams& session_default) {
  switch (s.profile) {
    case GainsProfile::kSessionDefault:
      return session_default;
    case GainsProfile::kCustom: {
      kinova::JointImpedanceParams jp = session_default;
      jp.Kq = s.custom.kq;
      jp.zeta = s.custom.zeta;
      jp.torque_limit = s.custom.torque_limit;
      return jp;
    }
    default:
      return profile_params(s.profile);
  }
}
}  // namespace kinova::interface
