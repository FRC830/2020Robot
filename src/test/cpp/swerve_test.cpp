#include <gtest/gtest.h>

#include "SwerveDrive.h"

TEST(swerve, angle) {
  EXPECT_EQ(SwerveModule::calculateTargetSetpoint(0.5, 0.75), 0.75);
  EXPECT_EQ(SwerveModule::calculateTargetSetpoint(0, 0.75), -0.25);
}
