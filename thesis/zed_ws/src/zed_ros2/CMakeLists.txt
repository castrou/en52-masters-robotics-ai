cmake_minimum_required(VERSION 3.8)
project(zed_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(HIDAPI REQUIRED)
find_package(LIBUSB 1 REQUIRED)
find_package(CURL REQUIRED)

include_directories(
  ${HIDAPI_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/zed_ros2
)

add_library(zed_lib
  src/zed_lib/videocapture.cpp
  src/zed_lib/sensorcapture.cpp
)

target_link_libraries(zed_lib
  ${OpenCV_LIBS}
  ${LibUSB_LIBRARIES}
  ${HIDAPI_LIBRARIES}
)

add_executable(zed_stream src/zed_stream.cpp src/main.cpp)
target_include_directories(zed_stream PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/zed_ros2
  ${CMAKE_CURRENT_SOURCE_DIR}/include/zed_ros2/zed_lib
  ${CURL_INCLUDE_DIRS})
target_compile_features(zed_stream PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS zed_stream
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(zed_stream
  zed_lib
  ${PROJECT_SOURCE_DIR}/include/zed_ros2
  ${CURL_LIBRARIES}
)

ament_target_dependencies(zed_stream
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
