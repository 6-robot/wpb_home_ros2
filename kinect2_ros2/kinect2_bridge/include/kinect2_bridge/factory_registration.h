#ifndef KINECT2_BRIDGE_FACTORY_REGISTRATION_H
#define KINECT2_BRIDGE_FACTORY_REGISTRATION_H

#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>

namespace kinect2_bridge
{
// libfreenect2 bigdepth has one guard row above and below the color image.
// Match the horizontal flip used by the bridge's color images, keep millimeters,
// and replace unobserved pixels (infinity) with the ROS depth-image invalid value.
inline cv::Mat factoryDepthToMirroredColor(const cv::Mat &bigDepth)
{
  CV_Assert(bigDepth.type() == CV_32FC1 && bigDepth.rows > 2);
  cv::Mat depth(bigDepth.rows - 2, bigDepth.cols, CV_16UC1, cv::Scalar(0));
  for(int y = 0; y < depth.rows; ++y)
  {
    const float *src = bigDepth.ptr<float>(y + 1);
    uint16_t *dst = depth.ptr<uint16_t>(y);
    for(int x = 0; x < depth.cols; ++x)
    {
      const float value = src[depth.cols - 1 - x];
      if(std::isfinite(value) && value > 0.0f && value <= 65535.0f)
      {
        dst[x] = cv::saturate_cast<uint16_t>(value);
      }
    }
  }
  return depth;
}
}  // namespace kinect2_bridge

#endif
