#include <gtest/gtest.h>
#include <sched.h>

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
  void set_actuator_modes(const ActuatorModes&) override {
    if (modes_timeout) throw TransportTimeout("timeout detected: fake pump");
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    const uint64_t i = calls++;
    sent.push_back(c);
    if (on_call) on_call(i);
    if (calls >= stop_after_) stop_.store(true);
    if (throw_runtime_at == i) throw std::runtime_error("not a timeout");
    if (drop_(i)) throw TransportTimeout("timeout detected: fake");
    fb.frame_id = i;
    if (on_fb) on_fb(i, fb);
  }
  void send(const JointCommand&) override {}
  void receive(JointFeedback&) override {}
  void safe_shutdown() override {}
  void release_low_level() override {}
  bool arm_in_fault() override { return false; }

  uint64_t calls = 0;
  std::vector<JointCommand> sent;
  uint64_t throw_runtime_at = UINT64_MAX;
  bool modes_timeout = false;
  std::function<void(uint64_t)> on_call;
  std::function<void(uint64_t, JointFeedback&)> on_fb;  // edits a good exchange's feedback

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

// Misses that never come in a row still end in CommsLost once max_window_misses
// fall within miss_window exchanges: every other frame lost trips it at the 5th.
TEST(RtExecutorMiss, AlternatingMissesTripTheWindowBudget) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t i) { return i >= 10 && i % 2 == 1; }, 1000, stop);
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  ex.request_mode(&mode);

  EXPECT_THROW(ex.run(stop), CommsLost);
  EXPECT_EQ(t.calls, 20u);  // misses at 11, 13, 15, 17, 19
}

// The window boundary: 3 misses spanning exactly miss_window exchanges trip it,
// 3 spanning one more do not.
TEST(RtExecutorMiss, WindowBudgetBoundary) {
  ExecutorConfig c = cfg(3);
  c.max_window_misses = 3;
  c.miss_window = 100;
  {
    std::atomic<bool> stop{false};
    DroppingTransport t([](uint64_t i) { return i > 0 && i % 50 == 0; }, 400, stop);
    SampleRing ring(1024);
    RtExecutor ex(t, ring, c);
    ASSERT_NO_THROW(ex.run(stop));  // 50, 100, 150: span 101 exchanges
  }
  {
    std::atomic<bool> stop{false};
    DroppingTransport t([](uint64_t i) { return (i > 0 && i % 50 == 0) || i == 149; }, 400, stop);
    SampleRing ring(1024);
    RtExecutor ex(t, ring, c);
    EXPECT_THROW(ex.run(stop), CommsLost);  // 50, 100, 149: span 100
    EXPECT_EQ(t.calls, 150u);
  }
}

// A mode switch whose pumps keep timing out is comms loss, not a raw timeout.
TEST(RtExecutorMiss, ModeSwitchTimeoutIsCommsLost) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 1000, stop);
  t.modes_timeout = true;
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  ex.request_mode(&mode);

  EXPECT_THROW(ex.run(stop), CommsLost);
  EXPECT_EQ(mode.entered_frame, UINT64_MAX);  // never entered
}

// Miss limits below 1 are rejected, not clamped.
TEST(RtExecutorMiss, InvalidMissConfigIsRejected) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 10, stop);
  SampleRing ring(16);
  for (int field = 0; field < 3; ++field) {
    ExecutorConfig c = cfg(3);
    (field == 0 ? c.max_consecutive_misses : field == 1 ? c.max_window_misses : c.miss_window) = 0;
    EXPECT_THROW(RtExecutor(t, ring, c), std::invalid_argument) << "field " << field;
  }
}

// --- exit_on_fault (never-crash design): a fault confirmed in feedback ends run().

namespace {
ExecutorConfig fault_cfg() {
  ExecutorConfig c = cfg(3);
  c.exit_on_fault = true;
  return c;
}
}  // namespace

TEST(RtExecutorFault, ConfirmedFaultThrowsArmFault) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 1000, stop);
  t.on_fb = [](uint64_t i, JointFeedback& fb) { fb.fault = i >= 5; };
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, fault_cfg());
  ex.request_mode(&mode);
  try {
    ex.run(stop);
    FAIL() << "expected ArmFault";
  } catch (const ArmFault& e) {
    EXPECT_NE(std::string(e.what()).find("fault"), std::string::npos) << e.what();
  }
  // Faulty calls 5, 6, 7: the third confirms.
  EXPECT_EQ(t.calls, 5u + uint64_t(kFaultConfirmExchanges));
}

TEST(RtExecutorFault, ShortFaultBlipIsSurvived) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 100, stop);
  t.on_fb = [](uint64_t i, JointFeedback& fb) {
    fb.fault = i >= 5 && i < 5 + uint64_t(kFaultConfirmExchanges) - 1;
  };
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, fault_cfg());
  ex.request_mode(&mode);
  EXPECT_NO_THROW(ex.run(stop));
  EXPECT_EQ(t.calls, 100u);
}

TEST(RtExecutorFault, LeavingLowLevelIsAFault) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 1000, stop);
  t.on_fb = [](uint64_t i, JointFeedback& fb) { fb.low_level = i < 5; };
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, fault_cfg());
  ex.request_mode(&mode);
  try {
    ex.run(stop);
    FAIL() << "expected ArmFault";
  } catch (const ArmFault& e) {
    EXPECT_NE(std::string(e.what()).find("low-level"), std::string::npos) << e.what();
  }
  EXPECT_EQ(t.calls, 5u + uint64_t(kLeftLowLevelConfirmExchanges));
}

// A miss between faulty frames neither confirms nor resets the count.
TEST(RtExecutorFault, MissesDoNotCountTowardsTheFault) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t i) { return i == 6; }, 1000, stop);
  t.on_fb = [](uint64_t i, JointFeedback& fb) { fb.fault = i >= 5; };
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, fault_cfg());
  ex.request_mode(&mode);
  EXPECT_THROW(ex.run(stop), ArmFault);
  EXPECT_EQ(t.calls, 5u + uint64_t(kFaultConfirmExchanges) + 1u);
}

TEST(RtExecutorFault, OffByDefaultKeepsTodaysBehaviour) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 100, stop);
  t.on_fb = [](uint64_t i, JointFeedback& fb) {
    fb.fault = i >= 5;
    fb.low_level = i < 5;
  };
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, cfg(3));
  ex.request_mode(&mode);
  EXPECT_NO_THROW(ex.run(stop));
  EXPECT_EQ(t.calls, 100u);
  CycleSample s;
  bool flagged = false;
  while (ring.pop(s)) flagged = flagged || (s.flags & 2u) != 0;
  EXPECT_TRUE(flagged);
}

TEST(RtExecutorFault, ArmRejectedPropagates) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 1000, stop);
  t.on_call = [](uint64_t i) {
    if (i == 7) throw ArmRejected("fake: WRONG_SERVOING_MODE");
  };
  CountingMode mode;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, fault_cfg());
  ex.request_mode(&mode);
  EXPECT_THROW(ex.run(stop), ArmRejected);
}

// ArmLink calls run() again after a recovery; a mode requested in between is
// adopted by the new run.
TEST(RtExecutorFault, ModeRequestedBetweenRunsIsAdopted) {
  std::atomic<bool> stop{false};
  uint64_t stop_at = 20;
  DroppingTransport t([](uint64_t) { return false; }, 1000, stop);
  t.on_call = [&](uint64_t i) {
    if (i >= stop_at) stop.store(true);
  };
  CountingMode first, second;
  SampleRing ring(1024);
  RtExecutor ex(t, ring, fault_cfg());
  ex.request_mode(&first);
  ex.run(stop);
  ASSERT_FALSE(first.seen_frames.empty());
  ex.request_mode(&second);
  stop_at = 40;
  stop.store(false);
  ex.run(stop);
  EXPECT_NE(second.entered_frame, UINT64_MAX);
  EXPECT_FALSE(second.seen_frames.empty());
}

TEST(RtExecutorFault, ConfigIsReadable) {
  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 1, stop);
  SampleRing ring(16);
  RtExecutor ex(t, ring, fault_cfg());
  EXPECT_TRUE(ex.config().exit_on_fault);
}

// run() puts the calling thread back as it found it. ArmLink reconnects from the
// thread that ran the loop, and every thread KORTEX creates in connect() inherits
// its scheduling: left at SCHED_FIFO on the RT core, the SDK's TCP receive and
// keepalive threads would land there too. Affinity is the part an unprivileged
// test can change.
TEST(RtExecutorFault, RunRestoresTheThreadsSchedulingOnExit) {
  cpu_set_t before;
  CPU_ZERO(&before);
  ASSERT_EQ(sched_getaffinity(0, sizeof(before), &before), 0);
  if (CPU_COUNT(&before) < 2) GTEST_SKIP() << "needs two CPUs to observe a pin";
  int pin = -1;
  for (int c = 0; c < CPU_SETSIZE && pin < 0; ++c)
    if (CPU_ISSET(c, &before)) pin = c;

  std::atomic<bool> stop{false};
  DroppingTransport t([](uint64_t) { return false; }, 1000, stop);
  t.on_call = [](uint64_t i) {
    if (i == 5) throw ArmRejected("fake: rejected");  // the exception path too
  };
  SampleRing ring(64);
  ExecutorConfig c = fault_cfg();
  c.rt.cpu = pin;
  RtExecutor ex(t, ring, c);
  EXPECT_THROW(ex.run(stop), ArmRejected);

  cpu_set_t after;
  CPU_ZERO(&after);
  ASSERT_EQ(sched_getaffinity(0, sizeof(after), &after), 0);
  EXPECT_TRUE(CPU_EQUAL(&before, &after)) << "CPUs after run: " << CPU_COUNT(&after);
}
