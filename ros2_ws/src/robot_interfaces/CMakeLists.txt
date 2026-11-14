cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  # Messages
  "msg/MotorCommand.msg"

  # Actions
  "action/Precision.action"
  "action/Ball.action"
  "action/Motor.action"
  DEPENDENCIES action_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

