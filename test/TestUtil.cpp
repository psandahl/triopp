#include "TestUtil.hpp"

#include <gtest/gtest.h>

void expectEqual(const cv::Vec2d& exp, const cv::Vec2d& act)
{
  EXPECT_NEAR(exp[0], act[0], 0.000000000001);
  EXPECT_NEAR(exp[1], act[1], 0.000000000001);
}

void expectEqual(const cv::Vec3d& exp, const cv::Vec3d& act)
{
  EXPECT_NEAR(exp[0], act[0], 0.000000000001);
  EXPECT_NEAR(exp[1], act[1], 0.000000000001);
  EXPECT_NEAR(exp[2], act[2], 0.000000000001);
}

void expectEqual(const cv::Mat& exp, const cv::Mat& act)
{
  ASSERT_EQ(exp.size(), act.size());
  ASSERT_EQ(exp.depth(), CV_64FC1);
  ASSERT_EQ(exp.depth(), act.depth());

  for (int i = 0; i < exp.cols; ++i) {
    for (int j = 0; j < exp.rows; ++j) {
      EXPECT_NEAR(exp.at<double>(j, i),
		  act.at<double>(j, i),
		  0.000000000001);
    }
  }
}
