#pragma once

#include <opencv2/core.hpp>

namespace trio {

// The world coordinate system for trio is a right handed system like
// the following.
//
//            Z
//            |
//            |____ Y
//            /
//           /
//          X
//
// The Euler rotations - yaw, pitch and roll - will be a rotation on
// the world axis in the order Y, Y and X.
//
//
// The camera coordinate system is a right hand system where the axis
// are permuted like:
//
//      X ____
//           /|
//          / |
//         Z  Y
//
// I.e. the Z axis is the direction for the camera, X is to the right,
// and Y is down.

// Create a world rotation matrix from yaw, pitch and roll. Angles
// must be in radians, and rotations are in counter clockwise order.
cv::Mat matrixRotateYPR(double y, double p, double r);
cv::Mat matrixRotateYPR(const cv::Vec3d& ypr);

// Decompose a world matrix into Euler angles yaw, pitch and roll.
cv::Vec3d decomposeEuler(const cv::Mat& mat);

// Create a world rotation matrix from an eye origin, direction where
// to look (which is along the X axis) and an initial up axis (which
// is Z).
cv::Mat matrixRotateLookAt(const cv::Point3d& eye, const cv::Point3d& at,
			   const cv::Vec3d& up);

// Create a rotation matrix that reflects the relative rotation from A to B.
cv::Mat matrixRotateRelative(const cv::Mat& a, const cv::Mat& b);

// Create a permutation matrix where axes are transformed from world
// coordinate frame to camera coordinate frame. To permute columns of
// a matrix it must be multiplied to the right of the matrix.
cv::Mat matrixWorldToCameraPermute();

// Create a 3x3 matrix carrying camera intrinsics. The width and
// height for the rect is inclusive. If using a e.g. unit size rect
// that is what to expect, but if the rect is representing pixels in
// an actual image the image's dimensions shall be reduced by - 1.
cv::Mat matrixIntrinsic(const cv::Vec2d& fieldOfView, const cv::Rect2d& rect);
  
// Give the rank for the given matrix.
int matrixRank(const cv::Mat& m);
  
}
