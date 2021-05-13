#include "MatrixUtil.hpp"

#include <cassert>
#include <cmath>
#include <limits>

namespace trio
{

cv::Mat matrixRotateYPR(double y, double p, double r)
{
  const double sinY = std::sin(y);
  const double cosY = std::cos(y);

  const double sinP = std::sin(p);
  const double cosP = std::cos(p);

  const double sinR = std::sin(r);
  const double cosR = std::cos(r);

  return (cv::Mat_<double>(3, 3) << 
    cosY * cosP, cosY * sinP * sinR - sinY * cosR, cosY * sinP * cosR + sinY * sinR, 
    sinY * cosP, sinY * sinP * sinR + cosY * cosR, sinY * sinP * cosR - cosY * sinR, 
    -sinP      , cosP * sinR                     , cosP * cosR);
}
  
cv::Mat matrixRotateYPR(const cv::Vec3d& ypr)
{
  return matrixRotateYPR(ypr[0], ypr[1], ypr[2]);
}

cv::Vec3d decomposeEuler(const cv::Mat& mat)
{
  assert(mat.rows == 3);
  assert(mat.cols == 3);
  assert(mat.depth() == CV_64F);

  return {
    std::atan2(mat.at<double>(1, 0), mat.at<double>(0, 0))
    , -std::asin(mat.at<double>(2, 0))
    , std::atan2(mat.at<double>(2, 1), mat.at<double>(2, 2))
  };
}

cv::Mat matrixRotateLookAt(const cv::Point3d& eye, const cv::Point3d& at,
			   const cv::Vec3d& up)
{
  const cv::Vec3d front(cv::normalize(cv::Vec3d(at - eye)));
  const cv::Vec3d side(cv::normalize(up.cross(front)));
  const cv::Vec3d newUp(front.cross(side));

  return (cv::Mat_<double>(3, 3) <<
	  front[0], side[0], newUp[0],
	  front[1], side[1], newUp[1],
	  front[2], side[2], newUp[2]);
}

cv::Mat matrixRotateRelative(const cv::Mat& a, const cv::Mat& b)
{
  assert(a.rows == 3);
  assert(a.cols == 3);
  assert(a.size() == b.size());

  return b * a.t();
}
  
int matrixRank(const cv::Mat& m)
{
  if (m.empty()) {
    return 0;
  }

  // Do SVD decomposition and count the singular values > 0.
  cv::Mat w;
  cv::SVD::compute(m, w, cv::SVD::NO_UV);

  for (int i = 0; i < w.rows; ++i) {
    if (w.at<double>(i, 0) < std::numeric_limits<double>::epsilon()) {
      return i;
    }
  }

  return w.rows;
}
  
}
