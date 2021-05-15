#include "Camera.hpp"
#include "MathUtil.hpp"

#include "TestUtil.hpp"

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

TEST(CameraTest, cameraPosition)
{
  // Shall always give back given position.
  const trio::Camera c0(cv::Point3d(0, 0, 0),
			trio::degToRad(cv::Vec3d(0, 0, 0)),
			trio::degToRad(cv::Vec2d(30, 20)));
  expectEqual(cv::Point3d(0, 0, 0), c0.position());

  const trio::Camera c1(cv::Point3d(1456, -145.35, 987),
			trio::degToRad(cv::Vec3d(5, -13, 5)),
			trio::degToRad(cv::Vec2d(30, 20)));
  expectEqual(cv::Point3d(1456, -145.35, 987), c1.position());
}

TEST(CameraTest, cameraPositionInCameraSpace)
{
  // Shall always be zero.
  const trio::Camera c0(cv::Point3d(0, 0, 0),
			trio::degToRad(cv::Vec3d(0, 0, 0)),
			trio::degToRad(cv::Vec2d(30, 20)));
  expectEqual(cv::Point3d(0, 0, 0), c0.cameraSpace(c0.position()));

  const trio::Camera c1(cv::Point3d(1456, -145.35, 987),
			trio::degToRad(cv::Vec3d(5, -13, 5)),
			trio::degToRad(cv::Vec2d(30, 20)));
  expectEqual(cv::Point3d(0, 0, 0), c1.cameraSpace(c1.position()));
}

TEST(CameraTest, otherPositionsInCameraSpace)
{
   const trio::Camera c0(cv::Point3d(0, 0, 0),
			 trio::degToRad(cv::Vec3d(0, 0, 0)),
			 trio::degToRad(cv::Vec2d(30, 20)));
   expectEqual(cv::Point3d(0, 0, 1), c0.cameraSpace(cv::Point3d(1, 0, 0)));
   expectEqual(cv::Point3d(-1, 0, 0), c0.cameraSpace(cv::Point3d(0, 1, 0)));
   expectEqual(cv::Point3d(0, -1, 0), c0.cameraSpace(cv::Point3d(0, 0, 1)));

   const trio::Camera c1(cv::Point3d(2, 2, 0),
			 trio::degToRad(cv::Vec3d(-90, 0, 0)),
			 trio::degToRad(cv::Vec2d(30, 20)));
   expectEqual(cv::Point3d(1, 0, 2), c1.cameraSpace(cv::Point3d(1, 0, 0)));
   expectEqual(cv::Point3d(2, 0, 1), c1.cameraSpace(cv::Point3d(0, 1, 0)));
   expectEqual(cv::Point3d(2, -1, 2), c1.cameraSpace(cv::Point3d(0, 0, 1)));
}
