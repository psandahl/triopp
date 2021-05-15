#include "MathUtil.hpp"

#include <cassert>
#include <cmath>

namespace trio {

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

cv::Mat toHomogeneous(const cv::Point2d& p)
{
  return (cv::Mat_<double>(3, 1, CV_64FC1) << p.x, p.y, 1.0);
}
  
cv::Mat toHomogeneous(const cv::Point3d& p)
{
  return (cv::Mat_<double>(4, 1, CV_64FC1) << p.x, p.y, p.z, 1.0);
}
  
cv::Point2d toEuclidean2d(const cv::Mat& m)
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
  
cv::Point3d toEuclidean3d(const cv::Mat& m)
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

cv::Mat toColumn(const cv::Vec3d& v)
{
  return (cv::Mat_<double>(3, 1, CV_64FC1) << v[0], v[1], v[2]);
}
  
cv::Vec2d toVec2d(const cv::Mat& m)
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
  
cv::Vec3d toVec3d(const cv::Mat& m)
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
  
}
