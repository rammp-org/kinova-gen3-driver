#include <gtest/gtest.h>
#include "kinova_lowlevel/rt_system.h"
using namespace kinova;
TEST(RtSystem, ReportPopulated) {
  auto rep = enable_rt({});            // may fail to set FIFO w/o privilege; must NOT throw
  EXPECT_GE(rep.policy, 0);
  auto u = read_usage();
  (void)u;                              // counters readable, no crash
  EXPECT_FALSE(introspect().empty());
}
