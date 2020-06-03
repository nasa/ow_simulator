#include <cmath>
#include "OpenCV_Util.h"

using namespace cv;
using namespace ow_dynamic_terrain;

Mat OpenCV_Util::expandImage(Mat image)
{
  auto center = Point2f(0.5f * image.size().width, 0.5 * image.size().height);
  auto radius = sqrtf(center.x * center.x + center.y * center.y);
  radius = ceilf(radius);
  auto h_add = int(radius - center.x);
  auto v_add = int(radius - center.y);
  auto result = Mat(2 * radius, 2 * radius, image.type());
  copyMakeBorder(image, result, v_add, v_add, h_add, h_add, BORDER_CONSTANT, Scalar::all(0));
  return result;
}

Mat OpenCV_Util::rotateImage(Mat image, float angle)
{
  auto center = Point2f(0.5f * image.size().width, 0.5 * image.size().height);
  auto rot_mat = getRotationMatrix2D(center, angle, 1.0);
  auto result = Mat(image.size(), image.type());
  warpAffine(image, result, rot_mat, result.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
  return result;
}