cmake_minimum_required(VERSION 3.5)
project(yoloinference)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interface code for our msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  DEPENDENCIES std_msgs
)

# Export runtime dependency so downstream can find it
ament_export_dependencies(rosidl_default_runtime)
ament_package()
