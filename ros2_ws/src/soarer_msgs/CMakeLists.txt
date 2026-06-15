cmake_minimum_required(VERSION 3.8)
project(soarer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThermalState.msg"
  "msg/ThermalField.msg"
  "msg/VerticalWind.msg"
  "msg/GroundEvent.msg"
  "msg/GroundEventArray.msg"
  "msg/FovDetection.msg"
  "msg/FovDetectionArray.msg"
  "msg/BatteryEstimate.msg"
  "msg/FsmState.msg"
  "msg/ThermalCue.msg"
  "msg/TelemetryExchange.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
