#pragma once

#include <opencv2/core.hpp>

#include <cmath>

namespace trio {
  
// Convert between degrees and radians.
inline double degToRad(double t)
{
  return t * (M_PI / 180.0);
}

inline double radToDeg(double t)
{
  return t / (M_PI / 180.0);
}

cv::Vec2d degToRad(const cv::Vec2d& v);
cv::Vec2d radToDeg(const cv::Vec2d& v);
cv::Vec3d degToRad(const cv::Vec3d& v);
cv::Vec3d radToDeg(const cv::Vec3d& v);

// Calculate focal length given a field of view and the length of a
// side.
double focalLength(double fov, double side);

// Calculate field of view given focal length and the length of a side.
double fieldOfView(double focalLength, double side);

// Conversion between points and column matrices.
cv::Mat toHomogeneous(const cv::Point2d& p);
cv::Mat toHomogeneous(const cv::Point3d& p);

cv::Point2d toEuclidean2d(const cv::Mat& m);
cv::Point3d toEuclidean3d(const cv::Mat& m);

// Conversion between vectors and column matrices.
cv::Mat toColumn(const cv::Vec3d& v);

cv::Vec2d toVec2d(const cv::Mat& m);
cv::Vec3d toVec3d(const cv::Mat& m);
  
}
