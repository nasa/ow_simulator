#ifndef OPENCV_UTIL_H
#define OPENCV_UTIL_H

#include <cv_bridge/cv_bridge.h>

namespace ow_dynamic_terrain
{
class OpenCV_Util
{
  static cv::Mat expandImage(cv::Mat image);
  static cv::Mat rotateImage(cv::Mat image, float angle);
};
}  // namespace ow_dynamic_terrain

#endif  // OPENCV_UTIL_H