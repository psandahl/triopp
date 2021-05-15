#include "MathUtil.hpp"
#include "MatrixUtil.hpp"

#include <opencv2/core.hpp>

#include <iostream>

int main()
{
  std::cout << ">\n" << trio::matrixRotateYPR(trio::degToRad(cv::Vec3d(90, -90, 180))) << std::endl;
  return 0;
}
