#include <zed_stream.hpp>

#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

#include "calibration.hpp"

namespace zed_stream
{

ZedCameraNode::ZedCameraNode(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::shared_ptr<image_transport::ImageTransport> & it)
: node_(node), it_(it)
{
  // RCLCPP initialization
  node_name_ = node_->get_name();
  left_image_pub_ = it_->advertise("rgb/left_image", 1);
  right_image_pub_ = it_->advertise("rgb/right_image", 1);
  left_rect_pub_ = it_->advertise("rgb/left_rect", 1);
  right_rect_pub_ = it_->advertise("rgb/right_rect", 1);
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);

  CameraInit();
  SensorInit();

  RCLCPP_INFO(node_->get_logger(), "[%s] Node started", node_name_.c_str());
}

void ZedCameraNode::run()
{
  PublishImages();
  PublishIMU();
}

void ZedCameraNode::CameraInit()
{
  // Initialize ZED camera
  sl_oc::video::VideoParams params;
  params.res = sl_oc::video::RESOLUTION::HD720;
  params.fps = sl_oc::video::FPS::FPS_60;
  params.verbose = sl_oc::VERBOSITY::ERROR;

  // Create Video Capture
  cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
  if (!cap_->initializeVideo()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Cannot open camera video capture", node_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "[%s] Connected to camera sn: %d [%s]", node_name_.c_str(), cap_->getSerialNumber(), cap_->getDeviceName().c_str());

   // ZED Calibration
  int sn = cap_->getSerialNumber();
  unsigned int serial_number = sn;
  // Download camera calibration file
  if( !sl_oc::tools::downloadCalibrationFile(serial_number, calibration_file) )
  {
      std::cerr << "Could not load calibration file from Stereolabs servers" << std::endl;
      return;
  }
  std::cout << "Calibration file found. Loading..." << std::endl;
  // ----> Frame Size
  cap_->getFrameSize(frame_width,frame_height);
  // ----> Initialize calibration
  sl_oc::tools::initCalibration(calibration_file, cv::Size(frame_width/2,frame_height), map_left_x, map_left_y, map_right_x, map_right_y,
                  cameraMatrix_left, cameraMatrix_right);
}

void ZedCameraNode::SensorInit()
{
  sens_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

  std::vector<int> devs = sens_->getDeviceList();

  if (devs.size() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] No available ZED 2, ZED 2i or ZED Mini cameras", node_name_.c_str());
    rclcpp::shutdown();
    return;
  }

  uint16_t fw_maior;
  uint16_t fw_minor;
  sens_->getFirmwareVersion(fw_maior, fw_minor);
  RCLCPP_INFO(node_->get_logger(), "[%s] Connected to IMU firmware version: %d.%d", node_name_.c_str(), fw_maior, fw_minor);

  // Initialize the sensors
  if (!sens_->initializeSensors(devs[0])) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] IMU initialize failed", node_name_.c_str());
    rclcpp::shutdown();
    return;
  }
}

void ZedCameraNode::PublishImages()
{
  // Get last available frame
  const sl_oc::video::Frame frame = cap_->getLastFrame();

  // Process and publish the frame
  if (frame.data != nullptr) {
    cv::Mat frame_yuv = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frame_bgr;
    cv::cvtColor(frame_yuv, frame_bgr, cv::COLOR_YUV2BGR_YUYV);

    // Split the frame into left and right images
    cv::Mat left_img = frame_bgr(cv::Rect(0, 0, frame_bgr.cols / 2, frame_bgr.rows));
    cv::Mat right_img = frame_bgr(cv::Rect(frame_bgr.cols / 2, 0, frame_bgr.cols / 2, frame_bgr.rows));

    // ----> Apply rectification
    cv::Mat left_rect, right_rect;
    cv::remap(left_img, left_rect, map_left_x, map_left_y, cv::INTER_LINEAR );
    cv::remap(right_img, right_rect, map_right_x, map_right_y, cv::INTER_LINEAR );

    // sl_oc::tools::showImage("right RECT", right_rect, params.res);
    // sl_oc::tools::showImage("left RECT", left_rect, params.res);

    // Convert the OpenCV images to RCLCPP image messages
    sensor_msgs::msg::Image::SharedPtr left_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_img).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr right_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_img).toImageMsg();

    // Publish the left and right image messages
    left_image_pub_.publish(*left_msg.get());
    right_image_pub_.publish(*right_msg.get());


    // ----> Publish Rectified
    sensor_msgs::msg::Image::SharedPtr left_rect_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_rect).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr right_rect_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_rect).toImageMsg();
    // Publish the left and right image messages
    left_rect_pub_.publish(*left_rect_msg.get());
    right_rect_pub_.publish(*right_rect_msg.get());
  }
}

void ZedCameraNode::PublishIMU()
{
  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(5000);

  if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = node_->now();
    imu_msg.header.frame_id = "imu_frame";

    // Convert the IMU data to the sensor_msgs/Imu message fields
    imu_msg.linear_acceleration.x = -imu_data.aX;
    imu_msg.linear_acceleration.y = imu_data.aY;
    imu_msg.linear_acceleration.z = imu_data.aZ;

    imu_msg.angular_velocity.x = -imu_data.gX;
    imu_msg.angular_velocity.y = imu_data.gY;
    imu_msg.angular_velocity.z = imu_data.gZ;

    // Publish the sensor_msgs/Imu message
    imu_pub_->publish(imu_msg);
  }
}

}  // namespace zed_stream