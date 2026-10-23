#include "kinova_lowlevel/rt_system.h"

#include <gtest/gtest.h>
#include <pthread.h>
#include <sched.h>

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

using namespace kinova;
TEST(RtSystem, ReportPopulated) {
  auto rep = enable_rt({});  // may fail to set FIFO w/o privilege; must NOT throw
  EXPECT_GE(rep.policy, 0);
  auto u = read_usage();
  (void)u;  // counters readable, no crash
  EXPECT_FALSE(introspect().empty());
}

namespace {
// A thread that idles until released, so raise_thread() has a live target.
struct IdleThread {
  std::mutex m;
  std::condition_variable cv;
  bool done = false;
  std::thread t{[this] {
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [this] { return done; });
  }};
  ~IdleThread() {
    {
      std::lock_guard<std::mutex> lk(m);
      done = true;
    }
    cv.notify_one();
    t.join();
  }
};

cpu_set_t affinity(pthread_t t) {
  cpu_set_t set;
  CPU_ZERO(&set);
  EXPECT_EQ(pthread_getaffinity_np(t, sizeof(set), &set), 0);
  return set;
}

int first_cpu(const cpu_set_t& set) {
  for (int c = 0; c < CPU_SETSIZE; ++c)
    if (CPU_ISSET(c, &set)) return c;
  return -1;
}
}  // namespace

// Priority 0 is invalid for SCHED_FIFO whatever the privilege, so this pins down "the CPU is
// pinned only if the priority took" deterministically.
TEST(RtSystem, RaiseThreadDoesNotPinWhenThePriorityFails) {
  IdleThread idle;
  const pthread_t h = idle.t.native_handle();
  const cpu_set_t before = affinity(h);
  if (CPU_COUNT(&before) < 2) GTEST_SKIP() << "pinning to the only CPU changes nothing";
  const std::string err = raise_thread(h, 0, first_cpu(before));
  EXPECT_NE(err.find("SCHED_FIFO failed"), std::string::npos) << err;
  const cpu_set_t after = affinity(h);
  EXPECT_TRUE(CPU_EQUAL(&before, &after));
}

// With RT privilege the thread ends up FIFO on exactly `cpu`; without it (CI) the call reports
// the failure and leaves the thread where it was.
TEST(RtSystem, RaiseThreadSetsFifoThenPins) {
  IdleThread idle;
  const pthread_t h = idle.t.native_handle();
  const cpu_set_t before = affinity(h);
  const int cpu = first_cpu(before);
  const std::string err = raise_thread(h, 1, cpu);
  int policy = -1;
  sched_param p{};
  ASSERT_EQ(pthread_getschedparam(h, &policy, &p), 0);
  const cpu_set_t after = affinity(h);
  if (err.empty()) {
    EXPECT_EQ(policy, SCHED_FIFO);
    EXPECT_EQ(p.sched_priority, 1);
    EXPECT_EQ(CPU_COUNT(&after), 1);
    EXPECT_TRUE(CPU_ISSET(cpu, &after));
  } else {
    EXPECT_NE(err.find("SCHED_FIFO failed"), std::string::npos) << err;
    EXPECT_NE(policy, SCHED_FIFO);
    EXPECT_TRUE(CPU_EQUAL(&before, &after));
  }
}
