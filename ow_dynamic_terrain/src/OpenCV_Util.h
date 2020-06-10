#ifndef OPENCV_UTIL_H
#define OPENCV_UTIL_H

#include <cv_bridge/cv_bridge.h>

namespace ow_dynamic_terrain
{
class OpenCV_Util
{
public:
  // A utility function that expand the image by the longest stretch from the image center
  static cv::Mat expandImage(const cv::Mat& image);
  // A utility function that returns the result of rotating an image around its center by an angle (given in degrees)
  static cv::Mat rotateImage(const cv::Mat& image, float angle);
  // A utility function that linearly scales a single channel 32-bit float format image to an 8-bit unsigned int format
  static cv::Mat scaleImage_32FC1_To_8UC1(const cv::Mat& image);
};
}  // namespace ow_dynamic_terrain

#endif  // OPENCV_UTIL_H