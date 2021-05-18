#include "MathUtil.hpp"
#include "MatrixUtil.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <iostream>

int main()
{
  cv::Mat i = (cv::Mat_<double>(3, 3) << 1.2, 0, 0, 0, 1.3, 0, 0, 0, 1);
  cv::Mat r = trio::matrixRotateYPR(trio::degToRad(cv::Vec3d(10, -20, -181)));

  cv::Mat a = i * r;

  std::cout << "I:\n" << i << std::endl;
  std::cout << "Rot:\n" << r << std::endl;

  cv::Mat Q, R;
  trio::decomposeRQ3x3(a, R, Q);

  std::cout << "Q:\n" << Q << std::endl;
  std::cout << "R:\n" << R << std::endl;

  /*  cv::Mat R, Q;
  cv::RQDecomp3x3(a, R, Q);

  std::cout << "R2:\n" << R << std::endl;
  std::cout << "Q2:\n" << Q << std::endl;*/
  
  return 0;
}
