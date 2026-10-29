#include "kinova_lowlevel/contact_replay.h"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace kinova;

namespace {
const std::string kUrdf = URDF_PATH;
const std::string kCsv = std::string(TEST_DATA_DIR) + "/contact_synthetic.csv";

ReplayOptions options() {
  ReplayOptions o;
  o.urdf_path = kUrdf;
  o.ee_frame = "gen3_end_effector_link";
  o.theta << 4, 4, 3, 3, 1.2, 1.2, 1.2;
  return o;
}
}  // namespace

// tests/data/contact_synthetic.csv: 100 Hz, 4 s sinusoid (Sinusoid in fixtures.h) with a
// 6 N·m external torque on joint 4 from t = 2 s to 3 s.
TEST(ContactReplay, SyntheticPushFound) {
  const ReplayInput in = load_replay_csv(kCsv);
  ASSERT_EQ(in.t.size(), 401u);
  const ReplayStats s = replay(in, options());
  ASSERT_EQ(s.r.size(), in.t.size());
  EXPECT_EQ(s.would_trips, 1u);
  ASSERT_TRUE(s.first_trip[3].valid);
  EXPECT_GT(s.first_trip[3].t, 2.0);
  EXPECT_LT(s.first_trip[3].t, 2.2);
  EXPECT_GT(s.first_trip[3].r, 3.0);
  for (int j = 0; j < kNumJoints; ++j)
    if (j != 3) EXPECT_FALSE(s.first_trip[j].valid) << j;
  EXPECT_GT(s.peak[3], 5.0);
  EXPECT_GT(s.moving_samples, 300u);
}

TEST(ContactReplay, TauSignFlipsConvention) {
  // Flipped sign is not command convention: the observer sees a huge residual.
  const ReplayStats s = replay(load_replay_csv(kCsv, -1.0), options());
  EXPECT_GE(s.would_trips, 1u);
}

TEST(ContactReplay, MissingFileThrows) {
  EXPECT_THROW(load_replay_csv(std::string(TEST_DATA_DIR) + "/nonexistent.csv"),
               std::runtime_error);
}
