// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <limits>
#include <gtest/gtest.h>
#include "small_gicp_relocalization/registration_quality.hpp"

using small_gicp_relocalization::assessRegistration;
using small_gicp_relocalization::RegistrationQualityLimits;

TEST(RegistrationQuality, AcceptsSupportedSmallCorrection)
{
  const auto identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d candidate = identity;
  candidate.translation().x() = 0.1;
  const auto quality = assessRegistration(true, candidate, identity, {0.01, 0.01, 9.0}, {});
  EXPECT_EQ(quality.rejection, nullptr);
  EXPECT_NEAR(quality.overlap, 2.0 / 3.0, 1e-8);
  EXPECT_NEAR(quality.rmse, 0.1, 1e-8);
}

TEST(RegistrationQuality, ConvergenceAloneDoesNotAcceptWrongMap)
{
  const auto identity = Eigen::Isometry3d::Identity();
  const auto quality = assessRegistration(true, identity, identity, {0.01, 4., 4., 4.}, {});
  EXPECT_STREQ(quality.rejection, "insufficient map overlap");
}

TEST(RegistrationQuality, RejectsLargeResidualsInsideSupportRadius)
{
  const auto identity = Eigen::Isometry3d::Identity();
  const auto quality = assessRegistration(true, identity, identity, {0.09, 0.09}, {});
  EXPECT_STREQ(quality.rejection, "excessive matching RMSE");
}

TEST(RegistrationQuality, RejectsTranslationAndRotationJumps)
{
  const auto identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d candidate = identity;
  candidate.translation().x() = 1.1;
  EXPECT_STREQ(assessRegistration(true, candidate, identity, {0.0}, {}).rejection,
    "excessive map-to-odom correction jump");
  candidate = identity;
  candidate.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_STREQ(assessRegistration(true, candidate, identity, {0.0}, {}).rejection,
    "excessive map-to-odom correction jump");
}

TEST(RegistrationQuality, RobotMotionIsNotCorrectionMotion)
{
  Eigen::Isometry3d previous = Eigen::Isometry3d::Identity();
  previous.translation().x() = 100.0;
  EXPECT_EQ(assessRegistration(true, previous, previous, {0.0}, {}).rejection, nullptr);
}

TEST(RegistrationQuality, RejectsMissingDataInvalidPoseAndUnconvergedResults)
{
  const auto identity = Eigen::Isometry3d::Identity();
  EXPECT_STREQ(assessRegistration(true, identity, identity, {}, {}).rejection,
    "insufficient map overlap");
  EXPECT_STREQ(assessRegistration(false, identity, identity, {0.0}, {}).rejection,
    "optimizer did not converge");
  Eigen::Isometry3d invalid = identity;
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_STREQ(assessRegistration(true, invalid, identity, {0.0}, {}).rejection,
    "non-finite transform");
}
