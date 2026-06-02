#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/gravity_comp_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/rt_system.h"
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
  GravityCompTorqueMode mode(dyn);
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

// Smoke test for the clock_nanosleep(ABSTIME) pacing path (the default benchmark
// and the test above exercise kSleepSpin; this confirms the other strategy runs
// the loop and produces samples without crashing).
TEST(RtSafety, NanosleepPacingProducesSamples) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  GravityCompTorqueMode mode(dyn);
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
