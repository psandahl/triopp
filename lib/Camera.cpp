#include "Camera.hpp"
#include "MathUtil.hpp"
#include "MatrixUtil.hpp"

namespace {

cv::Mat matrixCamera(const cv::Point3d& position,
		     const cv::Vec3d& orientation)
{
  cv::Mat camera(cv::Mat::zeros(3, 4, CV_64FC1));

  cv::Mat r(trio::matrixRotateYPR(orientation).t());
  cv::Mat t(r * trio::toColumn(position).mul(-1));

  r.copyTo(camera.colRange(0, 3));
  t.copyTo(camera.col(3));

  return camera;
}
  
}

namespace trio {

Camera::Camera(const cv::Point3d& position,
	       const cv::Vec3d& orientation,
	       const cv::Vec2d& fov,
	       const cv::Rect2d& sensor,
	       double k2, double k3, double k4)
  : _worldToCamera(matrixCamera(position, orientation))
  , _permuteT(matrixWorldToCameraPermute().t())
  , _intrinsic(matrixIntrinsic(fov, sensor))
  , _k2(k2)
  , _k3(k3)
  , _k4(k4)
{}

cv::Point3d Camera::position() const
{
  return toEuclidean3d(_worldToCamera.colRange(0, 3).t() *
		       _worldToCamera.col(3).mul(-1));
}

cv::Point3d Camera::cameraSpace(const cv::Point3d& point) const
{
  return toEuclidean3d(_permuteT * _worldToCamera * toHomogeneous(point));
}
  
}
