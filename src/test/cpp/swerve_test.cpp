#include <gtest/gtest.h>

#include "SwerveDrive.h"

TEST(swerve, angle) {
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.5, 0.75), 0.75);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0, 0.75), -0.25);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.1, 0.75), -0.25);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.4, 0.75), 0.75);

  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.1, 0.4), 0.4);

  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.8, 1.2), 1.2);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.8, 0.2), 1.2);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(1.8, 0.2), 2.2);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.8, 2.2), 1.2);

  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.1, -0.1), -0.1);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(0.1, 0.9), -0.1);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(-0.1, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(-0.1, -0.9), 0.1);

  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(-0.5, -0.75), -0.75);
  EXPECT_DOUBLE_EQ(SwerveModule::calculateTargetSetpoint(-0.4, -0.75), -0.75);
}
