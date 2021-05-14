#include <opencv2/core.hpp>

void expectEqual(const cv::Vec2d& exp, const cv::Vec2d& act);
void expectEqual(const cv::Vec3d& exp, const cv::Vec3d& act);
void expectEqual(const cv::Mat& exp, const cv::Mat& act);
