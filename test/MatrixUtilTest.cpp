#include "MatrixUtil.hpp"

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

TEST(MatrixUtilTest, matrixRank)
{
  EXPECT_EQ(0, trio::matrixRank(cv::Mat()));
  EXPECT_EQ(0, trio::matrixRank(cv::Mat::zeros(3, 3, CV_64FC1)));
  EXPECT_EQ(3, trio::matrixRank(cv::Mat::eye(3, 3, CV_64FC1)));

  const cv::Mat rank1Matrix = (cv::Mat_<double>(3, 3)
    << 1, 0, 0,
       0, 0, 0,
       0, 0, 0);
  EXPECT_EQ(1, trio::matrixRank(rank1Matrix));

  const cv::Mat rank2Matrix = (cv::Mat_<double>(3, 3)
    << 1, 0, 0,
       0, 0, 0,
       0, 0, 1);
  EXPECT_EQ(2, trio::matrixRank(rank2Matrix));
}
