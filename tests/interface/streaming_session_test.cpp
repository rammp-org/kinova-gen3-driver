#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/streaming_session.h"

using namespace kinova::interface;

namespace {
StreamOpenRequest req(SetpointKind k, ControlModeKind m, double timeout = 0.1) {
  StreamOpenRequest r; r.kind = k; r.control_mode = m; r.timeout_s = timeout; return r;
}
}  // namespace

TEST(StreamingSession, SupportedPairsOpenAndUnsupportedAreRefused) {
  StreamingSession s;
  EXPECT_TRUE (s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kJointPosition, ControlModeKind::kImpedance), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kEePose, ControlModeKind::kImpedance), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kJointTorque, ControlModeKind::kTorque), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kEePose, ControlModeKind::kPosition), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kEeTwist, ControlModeKind::kVelocity), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kJointVelocity, ControlModeKind::kVelocity), 0.0).accepted);
  s.close();
  // Nonsense pairing -- a client that thinks it streams twist into impedance is told.
  EXPECT_FALSE(s.open(req(SetpointKind::kEeTwist, ControlModeKind::kImpedance), 0.0).accepted);
}

TEST(StreamingSession, ZeroOrNegativeTimeoutIsRefused) {
  StreamingSession s;
  EXPECT_FALSE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, 0.0), 0.0).accepted);
  EXPECT_FALSE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, -1.0), 0.0).accepted);
  EXPECT_FALSE(s.is_open());
}

TEST(StreamingSession, SecondOpenIsRefusedCloseFirst) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  EXPECT_FALSE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kImpedance), 0.0).accepted);
  EXPECT_EQ(s.control_mode(), ControlModeKind::kPosition);   // the first session still owns it
}

TEST(StreamingSession, AdmitRequiresOpenAndMatchingKind) {
  StreamingSession s;
  EXPECT_FALSE(s.admit(SetpointKind::kJointPosition, 0.0));       // closed
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  EXPECT_TRUE (s.admit(SetpointKind::kJointPosition, 0.01));
  EXPECT_FALSE(s.admit(SetpointKind::kEePose, 0.02));             // wrong method for this session
  EXPECT_EQ(s.rejected_count(), 2u);
}

TEST(StreamingSession, ExpiresOnlyAfterTheDeadlineAndFreshSetpointsPushItOut) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, 0.1), 0.0).accepted);
  EXPECT_FALSE(s.expired(0.09));
  ASSERT_TRUE (s.admit(SetpointKind::kJointPosition, 0.09));      // fresh command resets the clock
  EXPECT_FALSE(s.expired(0.18));
  EXPECT_TRUE (s.expired(0.20));
}

TEST(StreamingSession, RejectedSetpointDoesNotRefreshTheDeadline) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, 0.1), 0.0).accepted);
  EXPECT_FALSE(s.admit(SetpointKind::kEeTwist, 0.09));            // wrong kind -> rejected
  EXPECT_TRUE (s.expired(0.11));   // still measured from open, not from the rejected setpoint
}

TEST(StreamingSession, CloseReturnsToClosedAndRefusesFurtherSetpoints) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  s.close();
  EXPECT_FALSE(s.is_open());
  EXPECT_FALSE(s.admit(SetpointKind::kJointPosition, 0.01));
  EXPECT_FALSE(s.expired(100.0));  // a closed session never expires; there is nothing to tear down
}

TEST(PairSupported, VelocityKindsAreNowSupportedInVelocityMode) {
  EXPECT_TRUE(pair_supported(SetpointKind::kJointVelocity, ControlModeKind::kVelocity));
  EXPECT_TRUE(pair_supported(SetpointKind::kEeTwist,       ControlModeKind::kVelocity));
}

TEST(PairSupported, EePoseIsNowSupportedInPositionMode) {
  EXPECT_TRUE(pair_supported(SetpointKind::kEePose, ControlModeKind::kPosition));
}

TEST(PairSupported, VelocityKindsAreStillRefusedInEveryOtherMode) {
  for (auto m : {ControlModeKind::kPosition, ControlModeKind::kImpedance,
                 ControlModeKind::kTorque}) {
    EXPECT_FALSE(pair_supported(SetpointKind::kJointVelocity, m));
    EXPECT_FALSE(pair_supported(SetpointKind::kEeTwist, m));
  }
}

TEST(PairSupported, VelocityModeRefusesEveryNonVelocityKind) {
  for (auto k : {SetpointKind::kJointPosition, SetpointKind::kEePose,
                 SetpointKind::kJointTorque}) {
    EXPECT_FALSE(pair_supported(k, ControlModeKind::kVelocity));
  }
}
