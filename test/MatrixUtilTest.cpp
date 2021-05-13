#include "MatrixUtil.hpp"
#include "MathUtil.hpp"

#include "TestUtil.hpp"

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

TEST(MatrixUtilTest, matrixYPRAndDecompose)
{
  // Just zeros.
  const cv::Vec3d r0(0, 0, 0);
  const cv::Mat m0(trio::matrixRotateYPR(trio::degToRad(r0)));
  expectEqual(r0, trio::radToDeg(trio::decomposeEuler(m0)));

  // Only yaw (Z).
  const cv::Vec3d r1(43, 0, 0);
  const cv::Mat m1(trio::matrixRotateYPR(trio::degToRad(r1)));
  expectEqual(r1, trio::radToDeg(trio::decomposeEuler(m1)));

  const cv::Vec3d r2(-43, 0, 0);
  const cv::Mat m2(trio::matrixRotateYPR(trio::degToRad(r2)));
  expectEqual(r2, trio::radToDeg(trio::decomposeEuler(m2)));

  // Only pitch (Y)
  const cv::Vec3d r3(0, 43, 0);
  const cv::Mat m3(trio::matrixRotateYPR(trio::degToRad(r3)));
  expectEqual(r3, trio::radToDeg(trio::decomposeEuler(m3)));

  const cv::Vec3d r4(0, -43, 0);
  const cv::Mat m4(trio::matrixRotateYPR(trio::degToRad(r4)));
  expectEqual(r4, trio::radToDeg(trio::decomposeEuler(m4)));

  // Only roll (X)
  const cv::Vec3d r5(0, 43, 43);
  const cv::Mat m5(trio::matrixRotateYPR(trio::degToRad(r5)));
  expectEqual(r5, trio::radToDeg(trio::decomposeEuler(m5)));

  const cv::Vec3d r6(0, 0, -43);
  const cv::Mat m6(trio::matrixRotateYPR(trio::degToRad(r6)));
  expectEqual(r6, trio::radToDeg(trio::decomposeEuler(m6)));

  // Some random mix cases.
  const cv::Vec3d r7(13, -46, 174);
  const cv::Mat m7(trio::matrixRotateYPR(trio::degToRad(r7)));
  expectEqual(r7, trio::radToDeg(trio::decomposeEuler(m7)));

  const cv::Vec3d r8(-78, -23, 179);
  const cv::Mat m8(trio::matrixRotateYPR(trio::degToRad(r8)));
  expectEqual(r8, trio::radToDeg(trio::decomposeEuler(m8)));
}
