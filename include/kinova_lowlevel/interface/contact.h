#pragma once
#include <limits>

#include "kinova_lowlevel/contact_monitor.h"
#include "kinova_lowlevel/interface/value_types.h"
namespace kinova::interface {

// Contact-detection threshold profiles from startup config, N·m per joint. `normal`
// is ContactProfile::kDefault. The ceiling bounds every profile and every custom
// value and is not runtime-editable. No usable default: every field starts NaN, so
// a config that never sets one is refused by validate_profiles.
struct ContactProfiles {
  JointVec sensitive = JointVec::Constant(std::numeric_limits<double>::quiet_NaN());
  JointVec normal = JointVec::Constant(std::numeric_limits<double>::quiet_NaN());
  JointVec loose = JointVec::Constant(std::numeric_limits<double>::quiet_NaN());
  JointVec ceiling = JointVec::Constant(std::numeric_limits<double>::quiet_NaN());
};

// Pure. Checks the ceiling, then every named profile against it. The message names
// the profile, the joint (1-based) and the limit, e.g. "contact profile loose: joint 1
// threshold 31 N·m must be finite, > 0 and <= ceiling 30 N·m".
ThresholdCheck validate_profiles(const ContactProfiles& p);

// Pure. The thresholds a request asks for: an unknown profile is refused; a named
// one comes from `p`, kCustom from req.custom; the result is then checked against
// p.ceiling. theta_out is written only when the check passes.
ThresholdCheck resolve_contact(const ContactThresholdsRequest& req, const ContactProfiles& p,
                               JointVec& theta_out);

}  // namespace kinova::interface
