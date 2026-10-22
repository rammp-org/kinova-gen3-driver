#include <gtest/gtest.h>

#include <atomic>
#include <functional>
#include <stdexcept>
#include <vector>

#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/telemetry_consumers.h"
#include "kinova_lowlevel/transport.h"
using namespace kinova;

namespace {

// Fake robot whose exchange() fails on chosen call indices (0 = the seed read).
// Every command passed in is recorded, sent or lost. A successful exchange stamps
// fb.frame_id with its call index, so a consumer can tell fresh fb from stale.
// Stops the loop itself after `stop_after` calls: no wall-clock dependence.
class DroppingTransport : public Transport {
 public:
  DroppingTransport(std::function<bool(uint64_t)> drop, uint64_t stop_after,
                    std::atomic<bool>& stop)
      : drop_(std::move(drop)), stop_after_(stop_after), stop_(stop) {
    sent.reserve(stop_after + 16);
  }
  void connect() override {}
  void set_servoing_low_level() override {}
  void set_actuator_modes(const ActuatorModes&) override {}
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    const uint64_t i = calls++;
    sent.push_back(c);
    if (on_call) on_call(i);
    if (calls >= stop_after_) stop_.store(true);
    if (throw_runtime_at == i) throw std::runtime_error("not a timeout");
    if (drop_(i)) throw TransportTimeout("timeout detected: fake");
    fb.frame_id = i;
  }
  void send(const JointCommand&) override {}
  void receive(JointFeedback&) override {}
  void safe_shutdown() override {}

  uint64_t calls = 0;
  std::vector<JointCommand> sent;
  uint64_t throw_runtime_at = UINT64_MAX;
  std::function<void(uint64_t)> on_call;

 private:
  std::function<bool(uint64_t)> drop_;
  uint64_t stop_after_;
  std::atomic<bool>& stop_;
};

// Output = number of compute() calls so far, so any extra (or missing) compute
// shows in the very next command. Records the frame each compute and on_enter saw.
class CountingMode : public ControlMode {
 public:
  ActuatorModes required_modes() const override {
    ActuatorModes m;
    m.fill(ActuatorMode::kTorque);
    return m;
  }
  void on_enter(const JointFeedback& fb) override { entered_frame = fb.frame_id; }
  void compute(const JointFeedback& fb, double, JointCommand& out) override {
    seen_frames.push_back(fb.frame_id);
    out.mode = ActuatorMode::kTorque;
    out.torque.setConstant(double(seen_frames.size()));
  }
  void on_exit() override {}

  uint64_t entered_frame = UINT64_MAX;
  std::vector<uint64_t> seen_frames;
};

ExecutorConfig cfg(int max_misses) {
  ExecutorConfig c{10000.0, Pacing::kSleepSpin, {0, -1, false, false}};
  c.max_consecutive_misses = max_misses;
  return c;
}

bool in(const std::vector<uint64_t>& v, uint64_t x) {
  for (uint64_t e : v)
    if (e == x) return true;
  return false;
}

}  // namespace

// (a) Isolated misses, and a burst one short of the limit, are survived: the loop
// runs to its own stop, each miss is flagged in its cycle sample, and the
// telemetry sink counts them on its console line.
TEST(RtExecutorMiss, IsolatedMissesAreSurvivedAndCounted) {
  const std::vector<uint64_t> drops = {5, 20, 21, 40};  // 20,21 = max-1 in a row
  std::atomic<bool> stop{false};
  DroppingTransport t([&](uint64_t i) { return in(drops, i); }, 60, stop);
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  ex.request_mode(&mode);

  ASSERT_NO_THROW(ex.run(stop));
  EXPECT_EQ(t.calls, 60u);  // ran to the end, not cut short

  TelemetrySink sink;
  CycleSample s;
  uint64_t samples = 0, missed = 0;
  while (ring.pop(s)) {
    sink.consume(s);
    ++samples;
    const bool flagged = (s.flags & 4u) != 0;
    // Cycle k (from 0) made exchange call k+1; the seed (call 0) has no sample.
    EXPECT_EQ(flagged, in(drops, s.cycle_index + 1)) << "cycle " << s.cycle_index;
    missed += flagged ? 1 : 0;
  }
  EXPECT_EQ(samples, 59u);
  EXPECT_EQ(missed, drops.size());
  EXPECT_NE(sink.console_line().find("comm_timeouts=4"), std::string::npos) << sink.console_line();
  EXPECT_EQ(mode.seen_frames.size(), 59u - drops.size());
}

// (b) max_consecutive_misses in a row is comms loss: run() throws CommsLost right
// at the Nth miss, having stopped exchanging.
TEST(RtExecutorMiss, ConsecutiveMissesThrowCommsLost) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t i) { return i >= 10; }, 1000, stop);
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  ex.request_mode(&mode);

  EXPECT_THROW(ex.run(stop), CommsLost);
  EXPECT_EQ(t.calls, 13u);                 // calls 10, 11, 12 missed; no 4th try
  EXPECT_EQ(mode.seen_frames.size(), 9u);  // computes only after good calls 1..9
}

// A lost seed read is retried; the mode first sees a real one.
TEST(RtExecutorMiss, SeedMissIsRetried) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t i) { return i <= 1; }, 10, stop);
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  ex.request_mode(&mode);

  ASSERT_NO_THROW(ex.run(stop));
  EXPECT_EQ(mode.entered_frame, 2u);
}

// (c) The command after a miss is the one the controller had already produced,
// re-sent unchanged: compute() is not run on the stale feedback, so nothing is
// integrated and nothing new is extrapolated.
TEST(RtExecutorMiss, CommandAfterMissIsResentUnchanged) {
  const std::vector<uint64_t> drops = {10, 30, 31};
  std::atomic<bool> stop{false};
  DroppingTransport t([&](uint64_t i) { return in(drops, i); }, 50, stop);
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  ex.request_mode(&mode);
  ASSERT_NO_THROW(ex.run(stop));

  // Expected: the command sent at call i carries the number of computes so far,
  // i.e. the good loop exchanges in [1, i). A miss adds no compute.
  double computes = 0.0;
  for (uint64_t i = 1; i < t.calls; ++i) {
    EXPECT_EQ(t.sent[i].torque[0], computes) << "call " << i;
    if (!in(drops, i)) computes += 1.0;
  }
  for (uint64_t d : drops) {
    const JointCommand& a = t.sent[d];
    const JointCommand& b = t.sent[d + 1];
    EXPECT_EQ(a.mode, b.mode);
    EXPECT_EQ(a.torque, b.torque);
    EXPECT_EQ(a.position, b.position);
    EXPECT_EQ(a.velocity, b.velocity);
  }
  // compute() only ever saw fresh feedback: each frame once, never a dropped one.
  for (size_t k = 1; k < mode.seen_frames.size(); ++k)
    EXPECT_GT(mode.seen_frames[k], mode.seen_frames[k - 1]);
  for (uint64_t f : mode.seen_frames) EXPECT_FALSE(in(drops, f));
}

// A mode requested while the last exchange missed is adopted only after a good
// one, so on_enter never seeds from stale feedback.
TEST(RtExecutorMiss, ModeAdoptionWaitsForFreshFeedback) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t i) { return i == 5; }, 20, stop);
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  t.on_call = [&](uint64_t i) {
    if (i == 5) ex.request_mode(&mode);
  };
  ASSERT_NO_THROW(ex.run(stop));
  // Without the deferral it would enter on call 4's (stale) frame.
  EXPECT_EQ(mode.entered_frame, 6u);
}

// Only the timeout is survived: any other exchange() error propagates as before,
// on its first occurrence.
TEST(RtExecutorMiss, OtherExceptionsPropagate) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 1000, stop);
  t.throw_runtime_at = 5;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  try {
    ex.run(stop);
    FAIL() << "expected an exception";
  } catch (const CommsLost&) {
    FAIL() << "a non-timeout error must not be reported as comms loss";
  } catch (const TransportTimeout&) {
    FAIL() << "a non-timeout error must not be mapped to a timeout";
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "not a timeout");
  }
  EXPECT_EQ(t.calls, 6u);
}
