#include <gtest/gtest.h>

#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/transport.h"

using namespace kinova;

namespace {
// Fake transport: returns a feedback whose q is a known ramp, and records the
// last command it was handed so we can assert FeedbackTap forwards to it.
struct FakeTransport : Transport {
  JointFeedback to_return;
  JointCommand last_cmd;
  bool connected = false;
  void connect() override { connected = true; }
  void set_servoing_low_level() override {}
  void set_actuator_modes(const ActuatorModes&) override {}
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    last_cmd = c;
    fb = to_return;
  }
  void send(const JointCommand& c) override { last_cmd = c; }
  void receive(JointFeedback& fb) override { fb = to_return; }
  void safe_shutdown() override {}
};

JointFeedback fb_with_q(double v) {
  JointFeedback fb;
  fb.q.setConstant(v);
  return fb;
}
}  // namespace

TEST(Seqlock, StoreThenLoadReturnsValue) {
  Seqlock<JointFeedback> snap;
  JointFeedback out;
  snap.store(fb_with_q(0.7));
  ASSERT_TRUE(snap.load(out));
  EXPECT_NEAR(out.q[0], 0.7, 1e-12);
  // A later store is what a subsequent load sees (latest-wins).
  snap.store(fb_with_q(-1.3));
  ASSERT_TRUE(snap.load(out));
  EXPECT_NEAR(out.q[3], -1.3, 1e-12);
}

TEST(FeedbackTap, ExchangePublishesInnerFeedbackIntoSnapshot) {
  FakeTransport inner;
  inner.to_return = fb_with_q(0.42);
  Seqlock<JointFeedback> snap;
  FeedbackTap tap(inner, snap);

  JointCommand cmd;
  JointFeedback fb;
  tap.exchange(cmd, fb);

  EXPECT_NEAR(fb.q[0], 0.42, 1e-12);   // caller still gets the feedback
  JointFeedback snapped;
  ASSERT_TRUE(snap.load(snapped));     // and it was tapped into the snapshot
  EXPECT_NEAR(snapped.q[0], 0.42, 1e-12);
}

TEST(FeedbackTap, ReceivePublishesInnerFeedbackIntoSnapshot) {
  FakeTransport inner;
  inner.to_return = fb_with_q(1.1);
  Seqlock<JointFeedback> snap;
  FeedbackTap tap(inner, snap);

  JointFeedback fb;
  tap.receive(fb);

  JointFeedback snapped;
  ASSERT_TRUE(snap.load(snapped));
  EXPECT_NEAR(snapped.q[6], 1.1, 1e-12);
}

TEST(FeedbackTap, ForwardsCommandsAndCallsToInner) {
  FakeTransport inner;
  Seqlock<JointFeedback> snap;
  FeedbackTap tap(inner, snap);

  tap.connect();
  EXPECT_TRUE(inner.connected);

  JointCommand cmd;
  cmd.torque.setConstant(3.0);
  JointFeedback fb;
  tap.exchange(cmd, fb);
  EXPECT_NEAR(inner.last_cmd.torque[0], 3.0, 1e-12);   // reached the wrapped transport
}
