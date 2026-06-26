#include <gtest/gtest.h>
#include <cstring>
#include "kinova_lowlevel/teleop_protocol.h"

namespace tp = kinova::teleop;

// Sizes are the hard acceptance check; this mirrors the header's compile-time
// static_asserts at runtime so a regression names itself in CTest output. These
// match the Python codec (kinova_teleop/protocol.py / tests/test_protocol.py).
TEST(TeleopProtocol, PacketSizes) {
  EXPECT_EQ(sizeof(tp::Header), 20u);
  EXPECT_EQ(sizeof(tp::PoseTargetPacket), 84u);
  EXPECT_EQ(sizeof(tp::GainsPacket), 157u);
  EXPECT_EQ(sizeof(tp::ControlPacket), 28u);
  EXPECT_EQ(sizeof(tp::FeedbackPacket), 261u);
}

TEST(TeleopProtocol, Constants) {
  EXPECT_EQ(tp::kMagic, 0x4B544C50u);  // "KTLP"
  EXPECT_EQ(tp::kVersion, 1u);
  EXPECT_EQ(tp::kNumJointsProto, 7);
}

// Serialize -> raw bytes -> deserialize must preserve every field bit-for-bit.
// Proves the packed layout survives a memcpy round-trip through a byte buffer
// (the exact path the socket takes).
TEST(TeleopProtocol, PoseTargetRoundTrip) {
  tp::PoseTargetPacket in{};
  in.h.magic = tp::kMagic;
  in.h.version = tp::kVersion;
  in.h.msg_type = static_cast<uint16_t>(tp::MsgType::kPoseTarget);
  in.h.seq = 42;
  in.h.timestamp_ns = 123456789ull;
  in.pos[0] = 0.10; in.pos[1] = -0.20; in.pos[2] = 0.30;
  in.quat_wxyz[0] = 1.0; in.quat_wxyz[1] = 0.0;
  in.quat_wxyz[2] = 0.0; in.quat_wxyz[3] = 0.0;
  in.gripper = 0.5f;
  in.flags = tp::kFlagEngaged | tp::kFlagFreeze;

  unsigned char buf[sizeof(in)];
  std::memcpy(buf, &in, sizeof(in));
  tp::PoseTargetPacket out{};
  std::memcpy(&out, buf, sizeof(out));

  EXPECT_EQ(out.h.magic, in.h.magic);
  EXPECT_EQ(out.h.msg_type, in.h.msg_type);
  EXPECT_EQ(out.h.seq, in.h.seq);
  EXPECT_EQ(out.h.timestamp_ns, in.h.timestamp_ns);
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(out.pos[i], in.pos[i]);
  for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(out.quat_wxyz[i], in.quat_wxyz[i]);
  EXPECT_FLOAT_EQ(out.gripper, in.gripper);
  EXPECT_EQ(out.flags, in.flags);
}

TEST(TeleopProtocol, FeedbackRoundTrip) {
  tp::FeedbackPacket in{};
  in.h.magic = tp::kMagic;
  in.h.version = tp::kVersion;
  in.h.msg_type = static_cast<uint16_t>(tp::MsgType::kFeedback);
  for (int i = 0; i < 7; ++i) { in.q[i] = i * 0.1; in.qd[i] = -i * 0.2; in.tau[i] = i * 1.5; }
  in.ee_pos[0] = 0.4; in.ee_pos[1] = 0.0; in.ee_pos[2] = 0.5;
  in.ee_quat_wxyz[0] = 0.0; in.ee_quat_wxyz[1] = 1.0;
  in.ee_quat_wxyz[2] = 0.0; in.ee_quat_wxyz[3] = 0.0;
  in.gripper_state = 0.25f;
  in.fault = 1;
  in.frame_id = 99999ull;
  in.last_control_seq = 7;

  unsigned char buf[sizeof(in)];
  std::memcpy(buf, &in, sizeof(in));
  tp::FeedbackPacket out{};
  std::memcpy(&out, buf, sizeof(out));

  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(out.q[i], in.q[i]);
    EXPECT_DOUBLE_EQ(out.qd[i], in.qd[i]);
    EXPECT_DOUBLE_EQ(out.tau[i], in.tau[i]);
  }
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(out.ee_pos[i], in.ee_pos[i]);
  for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(out.ee_quat_wxyz[i], in.ee_quat_wxyz[i]);
  EXPECT_FLOAT_EQ(out.gripper_state, in.gripper_state);
  EXPECT_EQ(out.fault, in.fault);
  EXPECT_EQ(out.frame_id, in.frame_id);
  EXPECT_EQ(out.last_control_seq, in.last_control_seq);
}
