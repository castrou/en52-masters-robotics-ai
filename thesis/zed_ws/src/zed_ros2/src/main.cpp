#include "zed_stream.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("zed_camera");
  auto it = std::make_shared<image_transport::ImageTransport>(node);

  zed_stream::ZedCameraNode zed_camera_node(node, it);

  rclcpp::Rate rate(30); // Set the loop rate, e.g., 30 Hz
  while (rclcpp::ok()) {
    zed_camera_node.run();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}