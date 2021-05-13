#include "MathUtil.hpp"
#include "MatrixUtil.hpp"

#include <opencv2/core.hpp>

#include <iostream>

int main()
{
  cv::Mat m0(trio::matrixRotateLookAt({0, 0, 0}, {0, 1, 0}, {0, 0, 1}));

  std::cout << "m0:\n" << m0 << std::endl;
  return 0;
}
