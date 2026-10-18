#include <limits>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <kinect2_bridge/factory_registration.h>

TEST(FactoryRegistration, RemovesGuardRowsAndMatchesColorFlip)
{
  cv::Mat input = (cv::Mat_<float>(4, 3) <<
    9999, 9999, 9999, 100, 200, 300, 400, 500, 600, 8888, 8888, 8888);
  const cv::Mat output = kinect2_bridge::factoryDepthToMirroredColor(input);
  ASSERT_EQ(output.type(), CV_16UC1);
  ASSERT_EQ(output.rows, 2);
  ASSERT_EQ(output.cols, 3);
  EXPECT_EQ(output.at<uint16_t>(0, 0), 300);
  EXPECT_EQ(output.at<uint16_t>(0, 2), 100);
  EXPECT_EQ(output.at<uint16_t>(1, 0), 600);
  EXPECT_EQ(output.at<uint16_t>(1, 2), 400);
}

TEST(FactoryRegistration, UnobservedAndInvalidDepthIsZero)
{
  cv::Mat input(3, 8, CV_32FC1, cv::Scalar(0));
  float *row = input.ptr<float>(1);
  row[0] = std::numeric_limits<float>::infinity();
  row[1] = std::numeric_limits<float>::quiet_NaN();
  row[2] = -std::numeric_limits<float>::infinity();
  row[3] = -20;
  row[4] = 0;
  row[5] = 65536;
  row[6] = 1234.2f;
  row[7] = 65535;
  const cv::Mat output = kinect2_bridge::factoryDepthToMirroredColor(input);
  EXPECT_EQ(output.at<uint16_t>(0, 0), 65535);
  EXPECT_EQ(output.at<uint16_t>(0, 1), 1234);
  for(int x = 2; x < 8; ++x)
  {
    EXPECT_EQ(output.at<uint16_t>(0, x), 0);
  }
}

TEST(FactoryRegistration, QhdMatchesHalfColorIntrinsicsWithoutVerticalOffset)
{
  cv::Mat input(1082, 1920, CV_32FC1, cv::Scalar(0));
  // Native color (1019, 300) becomes mirrored color (900, 300), QHD (450, 150).
  input.at<float>(301, 1019) = 2000;
  const cv::Mat depth = kinect2_bridge::factoryDepthToMirroredColor(input);
  const cv::Mat camera = (cv::Mat_<double>(3, 3) <<
    1081, 0, 959.5, 0, 1081, 539.5, 0, 0, 1);
  cv::Mat halfCamera = camera.clone();
  halfCamera.row(0) *= 0.5;
  halfCamera.row(1) *= 0.5;
  cv::Mat map1, map2, qhd;
  cv::initUndistortRectifyMap(camera, cv::Mat::zeros(1, 5, CV_64F), cv::Mat(),
    halfCamera, cv::Size(960, 540), CV_32FC2, map1, map2);
  cv::remap(depth, qhd, map1, map2, cv::INTER_NEAREST);
  EXPECT_EQ(qhd.size(), cv::Size(960, 540));
  EXPECT_EQ(qhd.at<uint16_t>(150, 450), 2000);
  EXPECT_EQ(cv::countNonZero(qhd), 1);
}
