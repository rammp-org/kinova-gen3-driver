#include "kinova_lowlevel/interface/contact.h"

#include <string>
namespace kinova::interface {

ThresholdCheck validate_profiles(const ContactProfiles& p) {
  const ThresholdCheck ceil = validate_contact_thresholds(p.ceiling, p.ceiling);
  if (!ceil.ok) return {false, "contact ceiling: " + ceil.message};
  const struct {
    const char* name;
    const JointVec& theta;
  } named[] = {{"sensitive", p.sensitive}, {"default", p.normal}, {"loose", p.loose}};
  for (const auto& n : named) {
    const ThresholdCheck c = validate_contact_thresholds(n.theta, p.ceiling);
    if (!c.ok) return {false, std::string("contact profile ") + n.name + ": " + c.message};
  }
  return {true, {}};
}

ThresholdCheck resolve_contact(const ContactThresholdsRequest& req, const ContactProfiles& p,
                               JointVec& theta_out) {
  const auto check = [&](const JointVec& theta) {
    ThresholdCheck c = validate_contact_thresholds(theta, p.ceiling);
    if (c.ok) theta_out = theta;
    return c;
  };
  // No default: adding a profile is a compile warning here, not a silent case.
  switch (req.profile) {
    case ContactProfile::kSensitive:
      return check(p.sensitive);
    case ContactProfile::kDefault:
      return check(p.normal);
    case ContactProfile::kLoose:
      return check(p.loose);
    case ContactProfile::kCustom:
      return check(req.custom);
  }
  // An out-of-enum byte from the wire.
  return {false, "unknown contact profile: expected sensitive, default, loose or custom"};
}

}  // namespace kinova::interface
