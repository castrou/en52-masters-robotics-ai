#ifndef ZED_STREAM__HPP_
#define ZED_STREAM__HPP_

#include <memory>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

#include "stereo.hpp"

namespace zed_stream
{

class ZedCameraNode
{
public:
  ZedCameraNode(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::shared_ptr<image_transport::ImageTransport> & it);
  void run();

private:
  void CameraInit();
  void SensorInit();
  void PublishImages();
  void PublishIMU();

  std::string node_name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  image_transport::Publisher left_rect_pub_;
  image_transport::Publisher right_rect_pub_;
  std::unique_ptr<sl_oc::video::VideoCapture> cap_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
  std::string calibration_file;
  int frame_width;
  int frame_height;
  double baseline;
  double fx, fy, cx, cy;
  cv::Mat map_left_x, map_left_y;
  cv::Mat map_right_x, map_right_y;
  cv::Mat cameraMatrix_left, cameraMatrix_right;
  cv::Ptr<cv::StereoSGBM> left_matcher;
  sl_oc::tools::StereoSgbmPar stereoPar;
};

}  // namespace zed_stream

#endif // ZED_STREAM__HPP_