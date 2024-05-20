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
#include "stopwatch.hpp"
#include "stereo.hpp"

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
      RCLCPP_ERROR(node_->get_logger(), "Could not load calibration file from Stereolabs servers");
      return;
  }
  RCLCPP_INFO(node_->get_logger(), "Calibration file found. Loading...");
  // ----> Frame Size
  cap_->getFrameSize(frame_width,frame_height);
  // ----> Initialize calibration
  sl_oc::tools::initCalibration(calibration_file, cv::Size(frame_width/2,frame_height), map_left_x, map_left_y, map_right_x, map_right_y,
                  cameraMatrix_left, cameraMatrix_right);

  // ----> Camera Param
  fx = cameraMatrix_left.at<double>(0,0);
  fy = cameraMatrix_left.at<double>(1,1);
  cx = cameraMatrix_left.at<double>(0,2);
  cy = cameraMatrix_left.at<double>(1,2);

  // ----> Declare OpenCV
   // ----> Stereo matcher initialization
    //Note: you can use the tool 'zed_open_capture_depth_tune_stereo' to tune the parameters and save them to YAML
    if(!stereoPar.load())
    {
        stereoPar.save(); // Save default parameters.
    }
  
    left_matcher = cv::StereoSGBM::create(stereoPar.minDisparity,stereoPar.numDisparities,stereoPar.blockSize);
    left_matcher->setMinDisparity(stereoPar.minDisparity);
    left_matcher->setNumDisparities(stereoPar.numDisparities);
    left_matcher->setBlockSize(stereoPar.blockSize);
    left_matcher->setP1(stereoPar.P1);
    left_matcher->setP2(stereoPar.P2);
    left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
    left_matcher->setMode(stereoPar.mode);
    left_matcher->setPreFilterCap(stereoPar.preFilterCap);
    left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
    left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
    left_matcher->setSpeckleRange(stereoPar.speckleRange);

    stereoPar.print();
    // <---- Stereo matcher initialization
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

    // ----> Stereo Matching
    cv::Mat left_depth_map, left_for_matcher, right_for_matcher;
    cv::Mat left_disp_half, left_disp, left_disp_float, left_disp_image;
    sl_oc::tools::StopWatch stereo_clock;
    double resize_fact = 1.0;
#ifdef USE_HALF_SIZE_DISP
    resize_fact = 0.5;
    // Resize the original images to improve performances
    cv::resize(left_rect,  left_for_matcher,  cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
    cv::resize(right_rect, right_for_matcher, cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
#else
    left_for_matcher = left_rect; // No data copy
    right_for_matcher = right_rect; // No data copy
#endif
    // Apply stereo matching
    left_matcher->compute(left_for_matcher, right_for_matcher,left_disp_half);

#ifdef USE_HALF_SIZE_DISP
    cv::multiply(left_disp_float,2.,left_disp_float); // Last 4 bits of SGBM disparity are decimal
    cv::UMat tmp = left_disp_float; // Required for OpenCV 3.2
    cv::resize(tmp, left_disp_float, cv::Size(), 1./resize_fact, 1./resize_fact, cv::INTER_AREA);
#else
    left_disp = left_disp_float;
#endif

    double elapsed = stereo_clock.toc();
    std::stringstream stereoElabInfo;
    sl_oc::tools::StereoSgbmPar stereoPar;

    cv::add(left_disp_float,-static_cast<double>(stereoPar.minDisparity-1),left_disp_float); // Minimum disparity offset correction
    cv::multiply(left_disp_float,1./stereoPar.numDisparities,left_disp_image,255., CV_8UC1 ); // Normalization and rescaling

    // ----> Extract Depth map
    // The DISPARITY MAP can be now transformed in DEPTH MAP using the formula
    // depth = (f * B) / disparity
    // where 'f' is the camera focal, 'B' is the camera baseline, 'disparity' is the pixel disparity

    double num = static_cast<double>(fx*baseline);
    cv::divide(num,left_disp_float,left_depth_map);

    float central_depth = left_depth_map.at<float>(left_depth_map.rows/2, left_depth_map.cols/2 );
    RCLCPP_INFO(node_->get_logger(), "Depth of the central pixel: [%d] mm", central_depth);
    // <---- 

    // ----> Create Point Cloud
    cv::Mat cloudMat;
    sl_oc::tools::StopWatch pc_clock;
    size_t buf_size = static_cast<size_t>(left_depth_map.cols * left_depth_map.rows);
    std::vector<cv::Vec3d> buffer( buf_size, cv::Vec3f::all( std::numeric_limits<float>::quiet_NaN() ) );
    cv::Mat depth_map_cpu = left_depth_map;
    float* depth_vec = (float*)(&(depth_map_cpu.data[0]));

    // ----> Publish Raw
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

    // ----> Publish Depth

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