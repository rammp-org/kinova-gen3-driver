#include <gtest/gtest.h>
#include "kinova_lowlevel/command_watchdog.h"

using kinova::CommandWatchdog;

TEST(CommandWatchdog, DisarmedNeverGoesStale) {
  CommandWatchdog w;                      // default: not armed
  EXPECT_FALSE(w.armed());
  for (int i = 0; i < 1000; ++i) EXPECT_FALSE(w.tick(0.001));
}

TEST(CommandWatchdog, GoesStaleOnlyAfterTheTimeout) {
  CommandWatchdog w; w.arm(0.05); w.bump(); w.reset(); w.bump();
  // A bump is pending, so the FIRST tick is the fresh one: it adopts the command
  // and zeroes staleness rather than accumulating dt. The deadline is measured
  // from there -- same semantics JointTorqueMode has always had.
  EXPECT_FALSE(w.tick(0.001));                                // fresh: stale_for == 0
  for (int i = 0; i < 49; ++i) EXPECT_FALSE(w.tick(0.001));   // 49 ms
  EXPECT_TRUE(w.tick(0.001));                                 // 50 ms -> stale
}

TEST(CommandWatchdog, AFreshBumpClearsStaleness) {
  CommandWatchdog w; w.arm(0.05); w.reset(); w.bump();
  for (int i = 0; i < 60; ++i) w.tick(0.001);
  ASSERT_TRUE(w.tick(0.001));
  w.bump();
  EXPECT_FALSE(w.tick(0.001));            // re-armed by the new command
  EXPECT_NEAR(w.stale_for(), 0.0, 1e-12);
}

TEST(CommandWatchdog, StaleForGrowsAndSizesADecayRamp) {
  CommandWatchdog w; w.arm(0.01); w.reset(); w.bump();
  w.tick(0.001);                             // the fresh tick: staleness resets to 0
  for (int i = 0; i < 30; ++i) w.tick(0.001);
  EXPECT_NEAR(w.stale_for(), 0.030, 1e-9);   // torque's ramp reads this
}

TEST(CommandWatchdog, ResetDoesNotTreatAPriorCommandAsFresh) {
  CommandWatchdog w; w.arm(0.05);
  w.bump();                                // command sent BEFORE entry
  w.reset();                               // on_enter adopts the count without honouring it
  for (int i = 0; i < 49; ++i) EXPECT_FALSE(w.tick(0.001));
  EXPECT_TRUE(w.tick(0.001));              // still goes stale on schedule
}

TEST(CommandWatchdog, ArmingWithZeroDisablesItMidRun) {
  CommandWatchdog w; w.arm(0.01); w.reset(); w.bump();
  for (int i = 0; i < 30; ++i) w.tick(0.001);
  ASSERT_TRUE(w.tick(0.001));
  w.arm(0.0);                              // session closed
  EXPECT_FALSE(w.tick(0.001));
}

// fresh() is what gates a mode's double-buffer read, so it must be true ONLY on
// the cycle the counter moves -- and false after reset(), even while the stream
// is still inside its timeout. Adopting a payload on "not stale yet" instead
// would resurrect a command published before on_enter.
TEST(CommandWatchdog, FreshIsTrueOnlyOnTheCycleTheCounterMoves) {
  CommandWatchdog w; w.arm(0.05);
  w.bump();                                // command sent BEFORE entry
  w.reset();
  w.tick(0.001);
  EXPECT_FALSE(w.fresh());                 // not stale yet, but NOT fresh either
  w.bump();
  w.tick(0.001);
  EXPECT_TRUE(w.fresh());                  // the new command lands
  w.tick(0.001);
  EXPECT_FALSE(w.fresh());                 // and is adopted exactly once
}
