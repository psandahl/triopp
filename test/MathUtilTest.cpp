#include "MathUtil.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(MathUtilTest, radToDeg)
{
  EXPECT_DOUBLE_EQ(0.0, trio::radToDeg(0.0));
  EXPECT_DOUBLE_EQ(90.0, trio::radToDeg(M_PI / 2.0));
  EXPECT_DOUBLE_EQ(180.0, trio::radToDeg(M_PI));
  EXPECT_DOUBLE_EQ(270.0, trio::radToDeg(M_PI * 1.5));
}

TEST(MathUtilTest, degToRad)
{
  EXPECT_DOUBLE_EQ(0.0, trio::degToRad(0.0));
  EXPECT_DOUBLE_EQ(M_PI / 2.0, trio::degToRad(90.0));
  EXPECT_DOUBLE_EQ(M_PI, trio::degToRad(180.0));
  EXPECT_DOUBLE_EQ(M_PI * 1.5, trio::degToRad(270.0));
}

TEST(MathUtilTest, focalLength)
{
  EXPECT_DOUBLE_EQ(1.0, trio::focalLength(trio::degToRad(90.0), 2.0));
}

TEST(MathUtilTest, fieldOfView)
{
  EXPECT_DOUBLE_EQ(90.0, trio::radToDeg(trio::fieldOfView(1.0, 2.0)));
}
