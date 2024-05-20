#include <memory>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

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
  cv::Mat map_left_x, map_left_y;
  cv::Mat map_right_x, map_right_y;
  cv::Mat cameraMatrix_left, cameraMatrix_right;
};

}  // namespace zed_stream