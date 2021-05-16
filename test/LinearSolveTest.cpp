#include "LinearSolve.hpp"
#include "Camera.hpp"
#include "MathUtil.hpp"
#include "MatrixUtil.hpp"

#include "TestUtil.hpp"

#include <opencv2/core.hpp>

#include <iostream>
#include <vector>

#include <gtest/gtest.h>

/*
px: [0.191869, -0.113622], pt: [-3.2, 1.3, 1.1]
px: [-0.0160151, -0.135319], pt: [-1.6, -2, 0.8]
px: [6.6513e-17, 0.055068], pt: [0, 0, -1]
px: [-0.149331, 0.0114188], pt: [1.8, -1.6, -0.1]
px: [0.0432935, 0.140696], pt: [1.2, 2.1, -0.6]
px: [-0.27106, -0.0789765], pt: [3.1, -2.7, 1.5]
px: [-0.0354929, 0.092276], pt: [3.3, 2.7, 1.8]
*/

TEST(LinearSolveTest, simpleDLT)
{
  // Produce point correspondences using the camera.
  const cv::Vec3d ypr
    (
     trio::decomposeEuler(trio::matrixRotateLookAt(cv::Point3d(10, 10, 10),
						   cv::Point3d(0, 0, 0),
						   cv::Vec3d(0, 0, 1)))
     );
  
  const trio::Camera c0(cv::Point3d(10, 10, 10), ypr,
			trio::degToRad(cv::Vec2d(50, 45)));

  const std::vector<cv::Point3d> points =
    {
      cv::Point3d(-3.2, 1.3, 1.1),
      cv::Point3d(-1.6, -2, 0.8),
      cv::Point3d(0, 0, -1),
      cv::Point3d(1.8, -1.6, -0.1),
      cv::Point3d(1.2, 2.1, -0.6),
      cv::Point3d(3.1, -2.7, 1.5),
      cv::Point3d(3.3, 2.7, 1.8)
    };

  std::vector<std::pair<cv::Point2d, cv::Point3d>> correspondences;
  for (const cv::Point3d& point : points) {
    correspondences.emplace_back(std::pair(c0.project(point), point));
  }

  // Check that the solving went ok.
  cv::Mat projection;
  ASSERT_EQ(true, trio::solveDLT(correspondences, projection));
  ASSERT_EQ(3, projection.rows);
  ASSERT_EQ(4, projection.cols);
  ASSERT_EQ(CV_64F, projection.depth());

  // Check that the projection matrix is reprojecting stuff as expected.
  for (const std::pair<cv::Point2d, cv::Point3d>& corr : correspondences) {
    const cv::Mat point(trio::toHomogeneous(corr.second));
    const cv::Point2d px(trio::toEuclidean2d(projection * point));
    expectEqual(corr.first, px);
  }

  // Decompose the projection matrix.
  cv::Point3d position;
  cv::Vec3d orientation;
  cv::Vec2d focalLength;
  trio::decomposeDLTMatrix(projection, position, orientation, focalLength);

  expectEqual(cv::Point3d(10, 10, 10), position);
  expectEqual(orientation, ypr);
  EXPECT_NEAR(trio::fieldOfView(focalLength[0], 1.0), trio::degToRad(50),
	      0.00000000001);
  EXPECT_NEAR(trio::fieldOfView(focalLength[1], 1.0), trio::degToRad(45),
	      0.00000000001);
}
