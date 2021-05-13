#include "TestUtil.hpp"

#include <gtest/gtest.h>

void expectEqual(const cv::Vec3d& exp, const cv::Vec3d& act)
{
  EXPECT_DOUBLE_EQ(exp[0], act[0]);
  EXPECT_DOUBLE_EQ(exp[1], act[1]);
  EXPECT_DOUBLE_EQ(exp[2], act[2]);
}
