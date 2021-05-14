#pragma once

#include <opencv2/core.hpp>

#include <cmath>

namespace trio
{
  
// Convert between degrees and radians.
inline double degToRad(double t)
{
  return t * (M_PI / 180.0);
}

inline double radToDeg(double t)
{
  return t / (M_PI / 180.0);
}

cv::Vec3d degToRad(const cv::Vec3d& v);
cv::Vec3d radToDeg(const cv::Vec3d& v);

// Calculate focal length given a field of view and the length of a
// side.
double focalLength(double fov, double side);

// Calculate field of view given focal length and the length of a side.
double fieldOfView(double focalLength, double side);
  
}
