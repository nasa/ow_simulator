#ifndef OPENCV_UTIL_H
#define OPENCV_UTIL_H

#include <cv_bridge/cv_bridge.h>

namespace ow_dynamic_terrain
{
class OpenCV_Util
{
public:
  static cv::Mat expandImage(const cv::Mat& image);
  static cv::Mat rotateImage(const cv::Mat& image, float angle);

  static cv::Mat scaleImage_32FC1_To_8UC1(const cv::Mat& image);
};
}  // namespace ow_dynamic_terrain

#endif  // OPENCV_UTIL_H