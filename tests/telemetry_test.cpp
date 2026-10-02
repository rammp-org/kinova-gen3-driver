#include "kinova_lowlevel/telemetry.h"

#include <gtest/gtest.h>

#include <thread>

#include "kinova_lowlevel/telemetry_consumers.h"
using namespace kinova;
TEST(SampleRing, FifoOrder) {
  SampleRing r(8);
  for (uint32_t i = 0; i < 4; ++i) {
    CycleSample s;
    s.cycle_index = i;
    ASSERT_TRUE(r.push(s));
  }
  CycleSample out;
  for (uint32_t i = 0; i < 4; ++i) {
    ASSERT_TRUE(r.pop(out));
    EXPECT_EQ(out.cycle_index, i);
  }
  EXPECT_FALSE(r.pop(out));
}
TEST(SampleRing, DropsWhenFull) {
  SampleRing r(2);
  CycleSample s;
  int ok = 0;
  for (int i = 0; i < 10; ++i) ok += r.push(s) ? 1 : 0;
  EXPECT_GT(r.dropped(), 0u);
  EXPECT_EQ(ok + (int)r.dropped(), 10);
}
TEST(SampleRing, SpscNoLoss) {
  SampleRing r(1024);
  constexpr uint64_t N = 100000;
  std::thread prod([&] {
    for (uint64_t i = 0; i < N;) {
      CycleSample s;
      s.cycle_index = i;
      if (r.push(s)) ++i;
    }
  });
  uint64_t expect = 0;
  CycleSample o;
  while (expect < N) {
    if (r.pop(o)) {
      ASSERT_EQ(o.cycle_index, expect);
      ++expect;
    }
  }
  prod.join();
  EXPECT_EQ(expect, N);
}
TEST(NanoHistogram, PercentilesMonotonic) {
  NanoHistogram h;
  for (uint32_t v = 1000; v <= 100000; v += 1000) h.add(v);
  EXPECT_EQ(h.count(), 100u);
  EXPECT_LE(h.percentile(0.5), h.percentile(0.99));
  EXPECT_GE(h.max(), h.percentile(0.99));
}
TEST(TelemetrySink, CountsOverruns) {
  TelemetrySink s;
  CycleSample c;
  c.flags = 1;
  c.cycle_ns = 1100000;
  c.compute_ns = 5000;
  s.consume(c);
  EXPECT_NE(s.console_line().find("overrun"), std::string::npos);
}

// The comm-budget warning. This exists because NIC rx interrupt coalescing ate
// 80% of the cycle budget for weeks while every in-process metric looked
// perfect, so the threshold itself is the thing worth pinning down.
namespace {
void feed_comm(TelemetrySink& s, uint32_t comm_ns, int n) {
  CycleSample c;
  c.comm_ns = comm_ns;
  c.cycle_ns = comm_ns;  // comm dominates the cycle, as it does in practice
  for (int i = 0; i < n; ++i) s.consume(c);
}
constexpr uint32_t kPeriod1kHz = 1000000;  // ns
}  // namespace

TEST(TelemetrySink, CommBudgetWarningFiresOnTheCoalescingSignature) {
  // The real measured failure: comm p50 799 us of a 1000 us budget.
  TelemetrySink s;
  feed_comm(s, 799000, 100);
  EXPECT_NE(s.console_line(kPeriod1kHz).find("coalescing"), std::string::npos);
}

TEST(TelemetrySink, CommBudgetWarningSilentWhenHealthy) {
  // The measured post-fix value, 281.6 us, must NOT warn. 281.6 us lands in the
  // 262144 ns log2 bucket, which is 26% of the budget.
  TelemetrySink s;
  feed_comm(s, 281600, 100);
  EXPECT_EQ(s.console_line(kPeriod1kHz).find("coalescing"), std::string::npos);
}

TEST(TelemetrySink, CommBudgetWarningOffByDefault) {
  // No period => no check, so existing callers are unaffected.
  TelemetrySink s;
  feed_comm(s, 799000, 100);
  EXPECT_EQ(s.console_line().find("coalescing"), std::string::npos);
}

TEST(TelemetrySink, CommBudgetWarningScalesWithTheLoopRate) {
  // 799 us is fine inside a 500 Hz (2000 us) budget -- 40%, and it buckets to
  // 524288 ns = 26%. The warning is about the FRACTION of the budget, not an
  // absolute latency, which is why a slower loop tolerates the same comm cost.
  TelemetrySink s;
  feed_comm(s, 799000, 100);
  EXPECT_EQ(s.console_line(2000000).find("coalescing"), std::string::npos);
}
