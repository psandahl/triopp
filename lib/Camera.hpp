#pragma once

#include <opencv2/core.hpp>

namespace trio {

class Camera {
public:
  // Construct a camera at the given position and orientation in the
  // world coordinate frame.
  Camera(const cv::Point3d& position,
	 const cv::Vec3d& orientation,
	 const cv::Vec2d& fov,
	 const cv::Rect2d& sensor=cv::Rect2d(-0.5, -0.5, 1.0, 1.0),
	 double k2=0.0, double k3=0.0, double k4=0.0);

  Camera() = delete;

  ~Camera() = default;

  Camera(const Camera&) = default;
  Camera& operator=(const Camera&) = default;

  // Get the Camera's position in world coordinate frame.
  cv::Point3d position() const;

  // Get a world coordinate mapped to the camera coordinate frame.
  cv::Point3d cameraSpace(const cv::Point3d& point) const;
  
private:
  // 3x4 world to camera matrix.
  const cv::Mat _worldToCamera;

  // 3x3 world to camera permute transpose.
  const cv::Mat _permuteT;
  
  // 3x3 intrinsic matrix.
  const cv::Mat _intrinsic;

  // Radial distortion parameters.
  const double _k2;
  const double _k3;
  const double _k4;
};
  
}
