#include "LinearSolve.hpp"

#include "MathUtil.hpp"
#include "MatrixUtil.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace trio {

bool solveDLT(const std::vector<std::pair<cv::Point2d, cv::Point3d>>& points,
	      cv::Mat& projection)
{
  if (points.size() < 6) {
    return false;
  }

  cv::Mat A(cv::Mat::zeros(points.size() * 2, 12, CV_64FC1));
  for (int i = 0; i < int(points.size()); ++i) {
    const cv::Point2d& x = points[i].first;
    const cv::Point3d& X = points[i].second;
    
    const int r1 = i * 2;
    A.at<double>(r1, 0) = X.x;
    A.at<double>(r1, 1) = X.y;
    A.at<double>(r1, 2) = X.z;
    A.at<double>(r1, 3) = 1.0;
    A.at<double>(r1, 8) = -x.x * X.x;
    A.at<double>(r1, 9) = -x.x * X.y;
    A.at<double>(r1, 10) = -x.x * X.z;
    A.at<double>(r1, 11) = -x.x;
    
    const int r2 = r1 + 1;
    A.at<double>(r2, 4) = X.x;
    A.at<double>(r2, 5) = X.y;
    A.at<double>(r2, 6) = X.z;
    A.at<double>(r2, 7) = 1.0;
    A.at<double>(r2, 8) = -x.y * X.x;
    A.at<double>(r2, 9) = -x.y * X.y;
    A.at<double>(r2, 10) = -x.y * X.z;
    A.at<double>(r2, 11) = -x.y;
  }

  cv::Mat w, u, vt;
  cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);

  const double smallestSingularValue = w.at<double>(w.rows - 1, 0);
  if (smallestSingularValue > 0.000001) {
    return false;
  }

  cv::Mat h(vt.row(vt.rows - 1));
  
  if (h.at<double>(0, 11) < 0.0) h = h.mul(-1);

  const double norm = std::sqrt(h.at<double>(0, 8) * h.at<double>(0, 8) +
				h.at<double>(0, 9) * h.at<double>(0, 9) +
				h.at<double>(0, 10) * h.at<double>(0, 10));
  h = h / norm;
  projection = h.reshape(0, 3);
  
  return true;
}

void decomposeDLTMatrix(const cv::Mat& projection, cv::Point3d& position,
			cv::Vec3d& orientation, cv::Vec2d& focalLength)
{
  assert(projection.rows == 3);
  assert(projection.cols == 4);
  assert(projection.depth() == CV_64F);

  // First split projection matrix into two parts.
  cv::Mat m33(projection.colRange(0, 3));
  cv::Mat m31(projection.col(3));

  // RQ decompose the 3x3 matrix into an intrinsics matrix and one
  // rotation matrix.
  cv::Mat R, Q;
  decomposeRQ3x3(m33, R, Q);

  // Permute stuff to order the rows.
  cv::Mat permute(matrixWorldToCameraPermute());

  cv::Mat r = permute * Q;
  cv::Mat t = permute * m31;

  // Invert the rotation to extract euler and translation.
  r = r.t();
  t = r * t.mul(-1);

  position = toEuclidean3d(t);
  orientation = decomposeEuler(r);
  focalLength[0] = R.at<double>(0, 0);
  focalLength[1] = R.at<double>(1, 1);
}
  
}
