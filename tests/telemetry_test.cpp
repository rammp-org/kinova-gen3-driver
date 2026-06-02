#include <gtest/gtest.h>
#include <thread>
#include "kinova_lowlevel/telemetry.h"
using namespace kinova;
TEST(SampleRing, FifoOrder) {
  SampleRing r(8);
  for (uint32_t i = 0; i < 4; ++i) { CycleSample s; s.cycle_index = i; ASSERT_TRUE(r.push(s)); }
  CycleSample out;
  for (uint32_t i = 0; i < 4; ++i) { ASSERT_TRUE(r.pop(out)); EXPECT_EQ(out.cycle_index, i); }
  EXPECT_FALSE(r.pop(out));
}
TEST(SampleRing, DropsWhenFull) {
  SampleRing r(2);
  CycleSample s; int ok = 0;
  for (int i = 0; i < 10; ++i) ok += r.push(s) ? 1 : 0;
  EXPECT_GT(r.dropped(), 0u);
  EXPECT_EQ(ok + (int)r.dropped(), 10);
}
TEST(SampleRing, SpscNoLoss) {
  SampleRing r(1024);
  constexpr uint64_t N = 100000;
  std::thread prod([&]{ for (uint64_t i=0;i<N;){ CycleSample s; s.cycle_index=i; if (r.push(s)) ++i; } });
  uint64_t expect=0; CycleSample o;
  while (expect<N){ if (r.pop(o)){ ASSERT_EQ(o.cycle_index,expect); ++expect; } }
  prod.join(); EXPECT_EQ(expect,N);
}
