// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include "OpenCV_Util.h"

using namespace std;
using namespace cv;
using namespace ow_dynamic_terrain;

Mat OpenCV_Util::expandImage(const Mat& image)
{
  auto center = Point2f(0.5f * image.size().width, 0.5f * image.size().height);
  auto radius = sqrtf(center.x * center.x + center.y * center.y);
  radius = ceilf(radius);
  auto h_add = static_cast<int>(radius - center.x);
  auto v_add = static_cast<int>(radius - center.y);
  auto result = Mat(2 * radius, 2 * radius, image.type());
  copyMakeBorder(image, result, v_add, v_add, h_add, h_add, BORDER_CONSTANT, Scalar::all(0.0));
  return result;
}

Mat OpenCV_Util::rotateImage(const Mat& image, float angle)
{
  auto center = Point2f(0.5f * image.size().width, 0.5f * image.size().height);
  auto rot_mat = getRotationMatrix2D(center, angle, 1.0);
  auto result = Mat(image.size(), image.type());
  warpAffine(image, result, rot_mat, result.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0.0));
  return result;
}

Mat OpenCV_Util::scaleImage_32FC1_To_8UC1(const Mat& image)
{
  auto min_intensity = 0.0;
  auto max_intensity = 0.0;
  minMaxLoc(image, &min_intensity, &max_intensity);
  auto result = Mat(image.size(), CV_8UC1);
  auto z_scale = 255.0 / (max_intensity - min_intensity);

  result.forEach<uchar>([&image, min_intensity, z_scale](uchar& pixel_value, const int pixel_index[]) {
    pixel_value = lround(z_scale * (image.at<float>(pixel_index) - min_intensity));
  });

  return result;
}

Mat OpenCV_Util::createZerosMatLike(const Mat &image)
{
  return Mat::zeros(image.size(), image.type());
}
