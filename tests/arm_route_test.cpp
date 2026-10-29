#include "kinova_lowlevel/arm_route.h"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace kinova;

TEST(ArmRoute, LoopbackIsDirect) { EXPECT_EQ(arm_route_problem("127.0.0.1"), ""); }

// TEST-NET-3 is on no host's own network: either it leaves by a default route or there is none.
TEST(ArmRoute, AnAddressOffEveryLocalNetworkIsRefused) {
  const std::string p = arm_route_problem("203.0.113.1");
  EXPECT_NE(p.find("203.0.113.1"), std::string::npos) << p;
}

TEST(ArmRoute, NotAnIpv4AddressIsAConfigurationError) {
  EXPECT_THROW(arm_route_problem("arm.local"), std::invalid_argument);
  EXPECT_THROW(arm_route_problem(""), std::invalid_argument);
}
