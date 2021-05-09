#pragma once

#include <opencv2/core.hpp>

namespace trio
{

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
// must be in radians.
cv::Mat matrixRotateYPR(double y, double p, double r);
cv::Mat matrixRotateYPR(const cv::Vec3d& ypr);

// Decompose a world matrix into Euler angles yaw, pitch and roll.
cv::Vec3d decomposeEuler(const cv::Mat& mat);
  
// Give the rank for the given matrix.
int matrixRank(const cv::Mat& m);
  
}
