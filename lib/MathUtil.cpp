#include "MathUtil.hpp"

#include <cassert>
#include <cmath>

namespace trio
{

cv::Vec2d degToRad(const cv::Vec2d& v)
{
  return { degToRad(v[0]), degToRad(v[1]) };
}
  
cv::Vec2d radToDeg(const cv::Vec2d& v)
{
  return { radToDeg(v[0]), radToDeg(v[1]) };
}
  
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

cv::Mat toColumn(const cv::Vec3d& v)
{
  return (cv::Mat_<double>(3, 1, CV_64FC1) << v[0], v[1], v[2]);
}
  
cv::Mat toColumn(const cv::Vec4d& v)
{
  return (cv::Mat_<double>(4, 1, CV_64FC1) << v[0], v[1], v[2], v[3]);
}

cv::Vec2d toEuclidean2d(const cv::Mat& m)
{
  assert(m.rows == 2 || m.rows == 3);
  assert(m.cols == 1);

  if (m.rows == 2 && m.cols == 1) {
    return { m.at<double>(0, 0), m.at<double>(1, 0) };
  } else if (m.rows == 3 && m.cols == 1) {
    return { m.at<double>(0, 0) / m.at<double>(2, 0),
             m.at<double>(1, 0) / m.at<double>(2, 0) };
  } else {
    return { 0, 0 };
  }
}
  
cv::Vec3d toEuclidean3d(const cv::Mat& m)
{
  assert(m.rows == 3 || m.rows == 4);
  assert(m.cols == 1);

  if (m.rows == 3 && m.cols == 1) {
    return { m.at<double>(0, 0), m.at<double>(1, 0), m.at<double>(2, 0) };
  } else if (m.rows == 4 && m.cols == 1) {
    return { m.at<double>(0, 0) / m.at<double>(3, 0),
             m.at<double>(1, 0) / m.at<double>(3, 0),
	     m.at<double>(2, 0) / m.at<double>(3, 0) };
  } else {
    return { 0, 0, 0 };
  }
}

cv::Mat toHomogeneous(const cv::Vec2d& v, double z)
{
  return toColumn(cv::Vec3d(v[0], v[1], z));
}
  
cv::Mat toHomogeneous(const cv::Vec3d& v, double w)
{
  return toColumn(cv::Vec4d(v[0], v[1], v[2], w));
}
  
}
