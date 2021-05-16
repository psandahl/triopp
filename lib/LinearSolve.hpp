#pragma once

#include <opencv2/core.hpp>

#include <utility>
#include <vector>

namespace trio {

// Solve a DLT with at least six "good" correspondences. If the DLT
// was successfull it will output the resulting 3x4 projection matrix.
bool solveDLT(const std::vector<std::pair<cv::Point2d, cv::Point3d>>& points,
	      cv::Mat& projection);

// Decompose a projection matrix from solveDLT.
void decomposeDLTMatrix(const cv::Mat& projection, cv::Point3d& position,
			cv::Vec3d& orientation, cv::Vec2d& focalLength);
  
}
