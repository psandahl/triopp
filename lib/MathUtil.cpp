#include "MathUtil.hpp"

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

}
