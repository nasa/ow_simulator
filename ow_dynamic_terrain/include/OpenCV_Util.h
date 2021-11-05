// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OPENCV_UTIL_H
#define OPENCV_UTIL_H

#include <opencv2/core/mat.hpp>

// TODO (optimization): provide an optional parameter to rotateImage in which it speicifies if the rotation should not
// lose details. In case this is selected the function can internally return an expanded image with the exact required
// size to contain the output of the rotation (which saves memory and time to copy and fill the expanded image).

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
  // A utility function that returns a Mat filled with zeroes that is the same type and size as the image argument
  static cv::Mat createZerosMatLike(const cv::Mat &image);
};
}  // namespace ow_dynamic_terrain

#endif  // OPENCV_UTIL_H
