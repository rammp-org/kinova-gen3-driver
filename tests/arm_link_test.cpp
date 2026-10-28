#include "kinova_lowlevel/arm_link.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
using namespace kinova;

namespace {

class RecordingListener : public LinkListener {
 public:
  void on_link_state(LinkState s) override {
    std::lock_guard<std::mutex> l(m_);
    seen_.push_back(s);
  }
  std::vector<LinkState> seen() const {
    std::lock_guard<std::mutex> l(m_);
    return seen_;
  }

 private:
  mutable std::mutex m_;
  std::vector<LinkState> seen_;
};

ExecutorConfig fast_exec(bool exit_on_fault = true) {
  ExecutorConfig c{10000.0, Pacing::kSleepSpin, {0, -1, false, false}};
  c.exit_on_fault = exit_on_fault;
  return c;
}

ArmLinkConfig fast_link() {
  ArmLinkConfig c;
  c.retry_period_s = 0.01;
  c.fault_poll_hz = 1000.0;
  c.health_period_s = 0.005;
  return c;
}

// SimTransport -> RtExecutor -> ArmLink, the link running on its own thread.
struct Rig {
  SimTransport sim{JointFeedback{}};
  SampleRing ring{1u << 12};
  RtExecutor exec{sim, ring, fast_exec()};
  RecordingListener listener;
  ArmLink link{sim, exec, &listener, fast_link()};
  std::atomic<bool> stop{false};
  std::thread th;
  bool threw_logic = false;

  void start() {
    th = std::thread([this] {
      try {
        link.run(stop);
      } catch (const std::logic_error&) {
        threw_logic = true;
      }
    });
  }
  // Stops the link and joins; returns how long run() took to return.
  double stop_and_join() {
    const auto t0 = std::chrono::steady_clock::now();
    stop.store(true);
    if (th.joinable()) th.join();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  ~Rig() { stop_and_join(); }

  bool wait_for(const std::function<bool(const LinkStatus&)>& pred, double timeout_s = 5.0) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred(link.status())) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }
  bool wait_state(LinkState s, double timeout_s = 5.0) {
    return wait_for([s](const LinkStatus& st) { return st.state == s; }, timeout_s);
  }
  // Stays in `s` for `hold_s` (polling).
  bool holds_state(LinkState s, double hold_s) {
    return !wait_for([s](const LinkStatus& st) { return st.state != s; }, hold_s);
  }
};

}  // namespace

TEST(ArmLink, RejectsAnExecutorWithoutExitOnFault) {
  SimTransport sim{JointFeedback{}};
  SampleRing ring{16};
  RtExecutor exec{sim, ring, fast_exec(false)};
  EXPECT_THROW(ArmLink(sim, exec, nullptr), std::invalid_argument);
}

TEST(ArmLink, RetriesConnectUntilTheArmAnswers) {
  Rig r;
  r.sim.fail_connects(3);
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  EXPECT_EQ(r.sim.connects.load(), 4);
  EXPECT_EQ(r.sim.low_level_entries.load(), 1);
  const auto seen = r.listener.seen();
  ASSERT_EQ(seen.size(), 2u);
  EXPECT_EQ(seen[0], LinkState::kConnecting);
  EXPECT_EQ(seen[1], LinkState::kRunning);
  EXPECT_EQ(r.link.status().reconnects, 0u);  // the first connect is not a reconnect
}

TEST(ArmLink, ConnectingReportsTheLastError) {
  Rig r;
  r.sim.fail_connects(1000000);
  r.start();
  ASSERT_TRUE(r.wait_for([](const LinkStatus& s) { return !s.reason.empty(); }));
  const LinkStatus s = r.link.status();
  EXPECT_EQ(s.state, LinkState::kConnecting);
  EXPECT_EQ(s.reason, "sim: arm unreachable");
}

TEST(ArmLink, CommsLossReconnectsWithoutAClear) {
  Rig r;
  std::atomic<bool> drop{false};
  r.sim.throw_on_exchange([&drop] {
    if (drop.load()) throw TransportTimeout("sim: lost");
  });
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  drop.store(true);
  ASSERT_TRUE(r.wait_state(LinkState::kConnecting));
  EXPECT_GE(r.sim.shutdowns.load(), 1);
  drop.store(false);
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  EXPECT_GE(r.link.status().reconnects, 1u);
  EXPECT_EQ(r.sim.clears.load(), 0);
  EXPECT_EQ(r.link.status().faults, 0u);
}

TEST(ArmLink, FaultWaitsForAnExplicitClear) {
  Rig r;
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  r.sim.set_arm_fault(true);
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  EXPECT_GE(r.sim.releases.load(), 1);
  EXPECT_TRUE(r.holds_state(LinkState::kFaulted, 0.1));  // never clears on its own
  EXPECT_EQ(r.sim.clears.load(), 0);
  EXPECT_NE(r.link.status().reason.find("fault"), std::string::npos) << r.link.status().reason;
  EXPECT_EQ(r.link.status().clears_done, 0u);
  EXPECT_TRUE(r.link.request_clear());
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  EXPECT_EQ(r.link.status().clears_done, 1u);
  EXPECT_EQ(r.sim.clears.load(), 1);
  EXPECT_EQ(r.sim.low_level_entries.load(), 2);
  EXPECT_EQ(r.link.status().faults, 1u);
  EXPECT_TRUE(r.link.status().reason.empty());
}

TEST(ArmLink, LeavingLowLevelIsAFault) {
  Rig r;
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  r.sim.set_low_level_feedback(false);
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  EXPECT_NE(r.link.status().reason.find("low-level"), std::string::npos);
  r.sim.set_low_level_feedback(true);
  EXPECT_TRUE(r.link.request_clear());
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
}

TEST(ArmLink, ArmRejectedIsAFault) {
  Rig r;
  std::atomic<bool> reject{false};
  r.sim.throw_on_exchange([&reject] {
    if (reject.exchange(false)) throw ArmRejected("sim: WRONG_SERVOING_MODE");
  });
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  reject.store(true);
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  EXPECT_NE(r.link.status().reason.find("WRONG_SERVOING_MODE"), std::string::npos);
}

TEST(ArmLink, UnknownExceptionIsAFault) {
  Rig r;
  std::atomic<bool> boom{false};
  r.sim.throw_on_exchange([&boom] {
    if (boom.exchange(false)) throw std::runtime_error("sim: unparseable feedback");
  });
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  boom.store(true);
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  EXPECT_NE(r.link.status().reason.find("unparseable"), std::string::npos);
}

TEST(ArmLink, ArmAlreadyFaultedAtStartWaits) {
  Rig r;
  r.sim.set_arm_fault(true);
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  EXPECT_TRUE(r.holds_state(LinkState::kFaulted, 0.05));
  EXPECT_EQ(r.sim.low_level_entries.load(), 0);
  EXPECT_EQ(r.sim.clears.load(), 0);
}

TEST(ArmLink, ClearWhileStillFaultedStaysFaulted) {
  Rig r;
  r.sim.set_arm_fault(true);
  r.sim.set_fault_sticky(true);
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  EXPECT_TRUE(r.link.request_clear());
  ASSERT_TRUE(r.wait_for([](const LinkStatus& s) { return s.clears_done == 1; }));
  EXPECT_EQ(r.link.status().reason, "fault persists after clear");
  EXPECT_EQ(r.link.status().state, LinkState::kFaulted);
  EXPECT_EQ(r.sim.clears.load(), 1);
  EXPECT_EQ(r.sim.low_level_entries.load(), 0);
  // A later clear, once the arm's fault is gone, does recover.
  r.sim.set_fault_sticky(false);
  EXPECT_TRUE(r.link.request_clear());
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
}

TEST(ArmLink, FaultSurvivesAReconnect) {
  Rig r;
  r.sim.set_arm_fault(true);
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  // The base stops answering while faulted, then comes back with its fault cleared
  // (power-cycled, say): the latch, not the arm, decides.
  r.sim.set_arm_fault(false);
  r.sim.fail_health(3);
  ASSERT_TRUE(r.wait_state(LinkState::kConnecting));
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  EXPECT_TRUE(r.holds_state(LinkState::kFaulted, 0.05));
  EXPECT_EQ(r.sim.low_level_entries.load(), 0);
  EXPECT_GE(r.link.status().reconnects, 1u);
}

TEST(ArmLink, RequestClearIsRefusedUnlessFaulted) {
  Rig r;
  r.sim.fail_connects(1000000);
  r.start();
  ASSERT_TRUE(r.wait_for([](const LinkStatus& s) { return !s.reason.empty(); }));
  EXPECT_FALSE(r.link.request_clear());  // connecting
  r.sim.fail_connects(0);
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  EXPECT_FALSE(r.link.request_clear());  // running
}

TEST(ArmLink, StopReturnsFromConnecting) {
  Rig r;
  r.sim.fail_connects(1000000);
  r.start();
  ASSERT_TRUE(r.wait_for([](const LinkStatus& s) { return !s.reason.empty(); }));
  EXPECT_LT(r.stop_and_join(), 1.0);
  EXPECT_GE(r.sim.shutdowns.load(), 1);
}

TEST(ArmLink, StopReturnsFromFaulted) {
  Rig r;
  r.sim.set_arm_fault(true);
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kFaulted));
  const int before = r.sim.shutdowns.load();
  EXPECT_LT(r.stop_and_join(), 1.0);
  EXPECT_GT(r.sim.shutdowns.load(), before);
}

TEST(ArmLink, StopReturnsFromRunning) {
  Rig r;
  r.start();
  ASSERT_TRUE(r.wait_state(LinkState::kRunning));
  EXPECT_LT(r.stop_and_join(), 1.0);
  EXPECT_GE(r.sim.shutdowns.load(), 1);
}

namespace {
// connect() fails the way an actuator-count mismatch does.
class WrongRobot : public SimTransport {
 public:
  WrongRobot() : SimTransport(JointFeedback{}) {}
  void connect() override { throw std::invalid_argument("robot reports 6 actuators"); }
};
}  // namespace

TEST(ArmLink, ConfigErrorEscapes) {
  WrongRobot t;
  SampleRing ring{16};
  RtExecutor exec{t, ring, fast_exec()};
  ArmLink link{t, exec, nullptr, fast_link()};
  std::atomic<bool> stop{false};
  EXPECT_THROW(link.run(stop), std::invalid_argument);
}
