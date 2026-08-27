#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/interface/value_types.h"
#include "kinova_lowlevel/interface/ports.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "interface/fake_backend.h"
using namespace kinova;

// RUSAGE_THREAD note: read_usage() reports the CALLING thread's faults. To make
// the major-fault check meaningful it must be sampled ON the RT loop thread, not
// the test/main thread. We therefore run the executor on a dedicated loop thread
// and sample read_usage() from INSIDE that thread's lambda, both right after a
// warm-up window and right before stop. The warm-up (a separate short run plus a
// settle sleep) faults in code+data first so the measured steady-state window
// sees zero NEW major faults. The shared atomics carry the readings back to the
// test thread for assertions.
TEST(RtSafety, NoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  JointTorqueMode mode(dyn);      // defaults: tau_ff never set == gravity-comp hold
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {2000.0, Pacing::kSleepSpin, {0, -1, true}});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> majflt_delta{~0ull};
  std::atomic<uint64_t> minflt_delta{~0ull};

  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) {} } });

  std::thread loop([&] {
    // Warm-up window: run the loop ~200ms so all first-touch faults (code,
    // Eigen/Pinocchio scratch, ring pages) happen before we start measuring.
    ex.request_mode(&mode);
    std::atomic<bool> warm_stop{false};
    std::thread warm_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      warm_stop.store(true);
    });
    ex.run(warm_stop);
    warm_watch.join();

    // Baseline on THIS (loop) thread, after warm-up.
    ResourceUsage u0 = read_usage();

    // Steady-state window: re-arm the mode (the warm-up run consumed the request)
    // and run ~500ms on this same thread.
    ex.request_mode(&mode);
    std::atomic<bool> measure_stop{false};
    std::thread measure_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      measure_stop.store(true);
    });
    ex.run(measure_stop);
    measure_watch.join();

    // Final reading on the same loop thread.
    ResourceUsage u1 = read_usage();
    majflt_delta.store(u1.majflt - u0.majflt);
    minflt_delta.store(u1.minflt - u0.minflt);

    stop.store(true);  // release the drain thread
  });

  loop.join();
  drain.join();

  EXPECT_EQ(majflt_delta.load(), 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}

TEST(RtSafety, ImpedanceModeNoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceMode mode(dyn);              // defaults; nullspace on
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {2000.0, Pacing::kSleepSpin, {0, -1, true}});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> majflt_delta{~0ull};
  std::atomic<uint64_t> minflt_delta{~0ull};
  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) {} } });

  std::thread loop([&] {
    // Warm-up window: fault in all code/scratch pages before measuring.
    ex.request_mode(&mode);
    std::atomic<bool> warm_stop{false};
    std::thread warm_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      warm_stop.store(true);
    });
    ex.run(warm_stop);
    warm_watch.join();

    ResourceUsage u0 = read_usage();
    // Re-arm the mode (the warm-up run consumed the request) and measure the
    // steady-state window on this same loop thread.
    ex.request_mode(&mode);
    std::atomic<bool> measure_stop{false};
    std::thread measure_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      measure_stop.store(true);
    });
    ex.run(measure_stop);
    measure_watch.join();

    ResourceUsage u1 = read_usage();
    majflt_delta.store(u1.majflt - u0.majflt);
    minflt_delta.store(u1.minflt - u0.minflt);
    stop.store(true);
  });

  loop.join();
  drain.join();
  EXPECT_EQ(majflt_delta.load(), 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}

// Same structure as the Cartesian case above, for the joint-space mode. This is
// the check that proves the IK running INSIDE the 1 kHz cycle does not allocate.
TEST(RtSafety, JointImpedanceModeNoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode mode(dyn);                  // defaults: IK runs every cycle
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {2000.0, Pacing::kSleepSpin, {0, -1, true}});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> majflt_delta{~0ull};
  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) {} } });

  std::thread loop([&] {
    // Warm-up window: fault in all code/scratch pages before measuring.
    ex.request_mode(&mode);
    std::atomic<bool> warm_stop{false};
    std::thread warm_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      warm_stop.store(true);
    });
    ex.run(warm_stop);
    warm_watch.join();

    ResourceUsage u0 = read_usage();
    // Re-arm the mode (the warm-up run consumed the request) and measure the
    // steady-state window on this same loop thread.
    ex.request_mode(&mode);
    std::atomic<bool> measure_stop{false};
    std::thread measure_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      measure_stop.store(true);
    });
    ex.run(measure_stop);
    measure_watch.join();

    ResourceUsage u1 = read_usage();
    majflt_delta.store(u1.majflt - u0.majflt);
    stop.store(true);
  });

  loop.join();
  drain.join();
  EXPECT_EQ(majflt_delta.load(), 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}

// The freeze branch is a DIFFERENT code path from the tracking one, and the
// tests above never reach it: they run with the default cmd_timeout_s = 0.0, so
// the watchdog is disarmed and compute() only ever takes the tracking branch.
// This case arms a deadline and lets it lapse inside the MEASURED window, so the
// staleness freeze runs under the same zero-major-faults / zero-dropped-samples
// assertions as everything else. Targets are published for the first ~100 ms and
// then stop, which exercises the tracking branch, the fresh->frozen transition
// and the sustained frozen path in one window.
TEST(RtSafety, JointImpedanceModeStaleFreezeNoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.cmd_timeout_s = 0.05;                        // armed: the freeze branch is live
  JointImpedanceMode mode(dyn, p);
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {2000.0, Pacing::kSleepSpin, {0, -1, true}});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> majflt_delta{~0ull};
  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) {} } });

  std::thread loop([&] {
    // Warm-up window: fault in all code/scratch pages -- INCLUDING the freeze
    // branch, which is why the warm-up also runs long enough to go stale.
    ex.request_mode(&mode);
    std::atomic<bool> warm_stop{false};
    std::thread warm_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      warm_stop.store(true);
    });
    ex.run(warm_stop);
    warm_watch.join();

    ResourceUsage u0 = read_usage();
    // Re-arm the mode (the warm-up run consumed the request) and measure the
    // steady-state window on this same loop thread.
    ex.request_mode(&mode);
    std::atomic<bool> measure_stop{false};
    std::thread measure_watch([&] {
      // Publish from a non-RT thread once on_enter has run -- it resets the
      // watchdog by design. Pose targets for ~100 ms, then silence: the 50 ms
      // deadline lapses and the mode freezes for the rest of the window.
      const Pose away = dyn.fk(JointVec::Constant(0.2));
      for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        mode.set_target(away);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      measure_stop.store(true);
    });
    ex.run(measure_stop);
    measure_watch.join();

    ResourceUsage u1 = read_usage();
    majflt_delta.store(u1.majflt - u0.majflt);
    stop.store(true);
  });

  loop.join();
  drain.join();
  EXPECT_EQ(majflt_delta.load(), 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}

// Position mode does no dynamics at all — no RNEA, no CRBA, no IK — so this is
// the cheapest control path we have. The check still matters: the reference
// integrator runs every cycle and must stay allocation-free like the rest.
TEST(RtSafety, JointPositionModeNoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  JointPositionMode mode(dyn);
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {2000.0, Pacing::kSleepSpin, {0, -1, true}});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> majflt_delta{~0ull};
  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) {} } });

  std::thread loop([&] {
    // Warm-up window: fault in all code/scratch pages before measuring.
    ex.request_mode(&mode);
    std::atomic<bool> warm_stop{false};
    std::thread warm_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      warm_stop.store(true);
    });
    ex.run(warm_stop);
    warm_watch.join();

    ResourceUsage u0 = read_usage();
    // Re-arm the mode (the warm-up run consumed the request) and measure the
    // steady-state window on this same loop thread.
    ex.request_mode(&mode);
    std::atomic<bool> measure_stop{false};
    std::thread measure_watch([&] {
      // Publish a target from a non-RT thread once on_enter has run — it clears
      // any pre-entry target by design. This is the real usage pattern, and it
      // keeps the integrator working during the measured window instead of
      // idling at the entry configuration where every clamp is a no-op.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      JointVec target; target.setConstant(0.4);
      mode.set_target(target);
      std::this_thread::sleep_for(std::chrono::milliseconds(480));
      measure_stop.store(true);
    });
    ex.run(measure_stop);
    measure_watch.join();

    ResourceUsage u1 = read_usage();
    majflt_delta.store(u1.majflt - u0.majflt);
    stop.store(true);
  });

  loop.join();
  drain.join();
  EXPECT_EQ(majflt_delta.load(), 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}

// Smoke test for the clock_nanosleep(ABSTIME) pacing path (the default benchmark
// and the test above exercise kSleepSpin; this confirms the other strategy runs
// the loop and produces samples without crashing).
TEST(RtSafety, NanosleepPacingProducesSamples) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  JointTorqueMode mode(dyn);      // defaults: tau_ff never set == gravity-comp hold
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {1000.0, Pacing::kClockNanosleep, {0, -1, true}});
  ex.request_mode(&mode);

  std::atomic<bool> stop{false};
  uint64_t consumed = 0;
  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) ++consumed; } });
  std::thread watch([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    stop.store(true);
  });
  ex.run(stop);
  watch.join();
  drain.join();
  // ~300 cycles at 1 kHz; allow generous slack for scheduling on a shared box.
  EXPECT_GT(consumed, 50u);
  EXPECT_EQ(ring.dropped(), 0u);
}

namespace {
// Local copy of supervisor_test.cpp's ramp7 helper (internal linkage, no ODR
// conflict across translation units) — builds a two-point 7-joint ramp goal.
interface::Trajectory ramp7(double from, double to, double dur) {
  interface::Trajectory t;
  JointVec a = JointVec::Constant(from), b = JointVec::Constant(to);
  t.points = {{a, 0.0}, {b, dur}};
  return t;
}
}  // namespace

// The Supervisor adds two non-RT threads (sampler @250Hz, pump @100Hz) that run
// concurrently with the RT loop, touching a Seqlock<JointFeedback>, a mutex-
// guarded inbox, and the ControlMode's set_target() from off the RT thread.
// This test proves that traffic does NOT show up as major page faults or
// dropped telemetry samples on the RT thread. Structure mirrors
// RtSafety.JointImpedanceModeNoMajorFaultsSteadyState exactly (warm-up run to
// fault in code/scratch pages, then a re-armed measured run, both timed and
// read_usage()-sampled on the SAME loop thread that calls exec.run()) — the
// only difference is the Supervisor + a long-running position goal are live
// for the whole test so the sampler is actively calling pos.set_target() and
// the pump is actively reading feedback throughout the measured window.
TEST(RtSafety, SupervisorInLoopNoMajorFaultsSteadyState) {
  using namespace kinova::interface;
  JointFeedback init; init.q.setZero();
  SimTransport sim(init);
  Dynamics dyn(URDF_PATH), pump_dyn(URDF_PATH);
  Seqlock<JointFeedback> snap;
  FeedbackTap tap(sim, snap);
  SampleRing ring(8192);
  JointPositionMode pos(dyn);
  JointImpedanceMode imp(dyn);
  JointTorqueMode tau(dyn);
  RtExecutor ex(tap, ring, {1000.0, Pacing::kSleepSpin, {}});
  FakeBackend be;
  Supervisor sup(pos, imp, tau, ex, snap, pump_dyn, be, be);

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> majflt_delta{~0ull};
  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) {} } });

  sup.start();   // requests position mode; spawns sampler + pump threads now

  // Long ramp (5s) so it is still executing across the whole warm-up + measured
  // window below: the sampler keeps ticking pos.set_target() and the pump keeps
  // reading the Seqlock + publishing state the entire time.
  TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.2, 5.0);
  g.control_mode = ControlModeKind::kPosition;
  g.preemption = Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);   // guard off: SimTransport is a static echo
  GoalId id{}; id[0] = 1;
  ASSERT_EQ(sup.on_trajectory_goal(g), GoalResponse::kAccept);
  sup.on_trajectory_accepted(id, g);

  std::thread loop([&] {
    // Warm-up window: run the loop ~200ms so first-touch faults (executor
    // code/scratch pages, mode entry) happen before we start measuring.
    std::atomic<bool> warm_stop{false};
    std::thread warm_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      warm_stop.store(true);
    });
    ex.run(warm_stop);
    warm_watch.join();

    // Baseline on THIS (loop) thread, after warm-up.
    ResourceUsage u0 = read_usage();

    // Steady-state window: re-arm the mode (the warm-up run consumed the
    // request — RtExecutor::run() resets its local `active` on each call) and
    // run ~2s on this same thread while the sampler/pump keep driving traffic.
    ex.request_mode(&pos);
    std::atomic<bool> measure_stop{false};
    std::thread measure_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      measure_stop.store(true);
    });
    ex.run(measure_stop);
    measure_watch.join();

    // Final reading on the same loop thread.
    ResourceUsage u1 = read_usage();
    majflt_delta.store(u1.majflt - u0.majflt);

    stop.store(true);  // release the drain thread
  });

  loop.join();
  drain.join();
  sup.stop();

  EXPECT_EQ(majflt_delta.load(), 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}
