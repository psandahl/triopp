#include "MatrixUtil.hpp"
#include "MathUtil.hpp"

#include "TestUtil.hpp"

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>

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

TEST(MatrixUtilTest, matrixLookAtAndDecompose)
{
  // Shall be no rotation.
  const cv::Mat m0(trio::matrixRotateLookAt({0, 0, 0}, {1, 0, 0}, {0, 0, 1}));
  expectEqual(cv::Vec3d(0, 0, 0), trio::radToDeg(trio::decomposeEuler(m0)));

  // Shall be 45 degree yaw.
  const cv::Mat m1(trio::matrixRotateLookAt({0, 0, 0}, {1, 1, 0}, {0, 0, 1}));
  expectEqual(cv::Vec3d(45, 0, 0), trio::radToDeg(trio::decomposeEuler(m1)));

  // Shall be -45 degree yaw.
  const cv::Mat m2(trio::matrixRotateLookAt({0, 0, 0}, {1, -1, 0}, {0, 0, 1}));
  expectEqual(cv::Vec3d(-45, 0, 0), trio::radToDeg(trio::decomposeEuler(m2)));

  // Shall be 45 degree pitch.
  const cv::Mat m3(trio::matrixRotateLookAt({0, 0, 0}, {1, 0, -1}, {0, 0, 1}));
  expectEqual(cv::Vec3d(0, 45, 0), trio::radToDeg(trio::decomposeEuler(m3)));

  // Shall be -45 degree pitch.
  const cv::Mat m4(trio::matrixRotateLookAt({0, 0, 0}, {1, 0, 1}, {0, 0, 1}));
  expectEqual(cv::Vec3d(0, -45, 0), trio::radToDeg(trio::decomposeEuler(m4)));

  // Shall be 45 degree roll.
  const cv::Mat m5(trio::matrixRotateLookAt({0, 0, 0}, {1, 0, 0}, {0, -1, 1}));
  expectEqual(cv::Vec3d(0, 0, 45), trio::radToDeg(trio::decomposeEuler(m5)));

  // Shall be -45 degree roll.
  const cv::Mat m6(trio::matrixRotateLookAt({0, 0, 0}, {1, 0, 0}, {0, 1, 1}));
  expectEqual(cv::Vec3d(0, 0, -45), trio::radToDeg(trio::decomposeEuler(m6)));

  // A mixed case with both yaw and pitch, each 45 degrees.
  const cv::Mat m7(trio::matrixRotateLookAt({0, 0, 0},
					    {1, 1, -std::hypot(1, 1)},
					    {0, 0, 1}));
  expectEqual(cv::Vec3d(45, 45, 0), trio::radToDeg(trio::decomposeEuler(m7)));
}

TEST(MatrixUtilTest, matrixRelative)
{
  // Relative orientation between equal rotations shall be identity matrix.
  const cv::Mat m0(trio::matrixRotateYPR(trio::degToRad({123, 88, -15})));
  const cv::Mat m1(trio::matrixRotateRelative(m0, m0));  
  expectEqual(cv::Mat::eye(3, 3, CV_64FC1), m1);

  // Relative orientation between different rotations shall be usable
  // to transfer between those two.
  const cv::Mat m2(trio::matrixRotateYPR(trio::degToRad({42, 3, 5})));
  const cv::Mat m3(trio::matrixRotateYPR(trio::degToRad({49,-13, 5})));

  const cv::Mat m4(trio::matrixRotateRelative(m2, m3));

  // m4 * m2 shall yield m3.
  expectEqual(m3, m4 * m2);

  // m4.t() * m3 shall yield m2.
  expectEqual(m2, m4.t() * m3);
}

TEST(MatrixUtilTest, matrixPermute)
{
  const cv::Mat worldToCam(trio::matrixWorldToCameraPermute());

  const cv::Mat w0(trio::matrixRotateYPR(trio::degToRad({-17, 139, -11})));
  const cv::Mat c0(w0 * worldToCam);

  // Check that the axes are the expected.
  expectEqual(c0.col(0), w0.col(1).mul(-1));
  expectEqual(c0.col(1), w0.col(2).mul(-1));
  expectEqual(c0.col(2), w0.col(0));

  // And check that we can go back to world frame.
  expectEqual(w0, c0 * worldToCam.t());
}
