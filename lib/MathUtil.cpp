#include "MathUtil.hpp"

#include <cmath>

namespace trio
{

cv::Vec3d degToRad(const cv::Vec3d& v)
{
  return { degToRad(v[0]), degToRad(v[1]), degToRad(v[2]) };
}
  
cv::Vec3d radToDeg(const cv::Vec3d& v)
{
  return { radToDeg(v[0]), radToDeg(v[1]), radToDeg(v[2]) };
}

double focalLength(double fov, double side)
{
  return (side / 2.0) / std::tan(fov / 2.0);
}

double fieldOfView(double focalLength, double side)
{
  return std::atan2(side / 2.0, focalLength) * 2.0;
}
  
}
